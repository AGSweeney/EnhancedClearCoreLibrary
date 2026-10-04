/* MIT License. Copyright (c) 2026 Adam G. Sweeney <agsweeney@gmail.com> */
/*
 * Independent joint step/dir motion for a ros2_control-style position interface.
 *
 * StepGenerator::Move() retargets and keeps the current velocity, but each
 * call plans a stop at the new endpoint. A goal that is still changing is
 * tracked with a latched velocity. Any new integer steps/s value is applied.
 * A track frame adds a bounded correction from the scheduled-position error.
 * Once an absolute goal has been stable, one Move() lands on the nearest step.
 *
 * Joint commands are absolute. Rounding is nearest-step on the absolute
 * target, so the relative-move residual drift fixed in ClearAI does not apply.
 */

#include "MotionCore.h"

#include "ClearCore.h"
#include "SysTiming.h"

#include <stdio.h>
#include <string.h>

static uint32_t g_stepsPerRev[CCROS_AXIS_COUNT];
static double g_pitchMm[CCROS_AXIS_COUNT];
static uint32_t g_axisMask = CCROS_DEFAULT_AXIS_MASK;
static uint32_t g_vel = CCROS_DEFAULT_VEL_STEPS;
static uint32_t g_accel = CCROS_DEFAULT_ACCEL_STEPS;
static uint32_t g_decel = CCROS_DEFAULT_DECEL_STEPS;
static uint32_t g_watchdogMs = CCROS_DEFAULT_WATCHDOG_MS;
static uint8_t g_estopDi6 = CCROS_DEFAULT_ESTOP_DI6;
static bool g_testMode = false;
static bool g_enabled = false;
static bool g_interrupted = false;
static bool g_watchdogTripped = false;
static uint32_t g_lastHostMs = 0;
static uint16_t g_lastCmdSeq = 0;

static bool g_goalValid[CCROS_AXIS_COUNT];
static int32_t g_goalSteps[CCROS_AXIS_COUNT];
static uint32_t g_goalChangedMs[CCROS_AXIS_COUNT];
static bool g_absIssued[CCROS_AXIS_COUNT];
static bool g_absRetried[CCROS_AXIS_COUNT];
static bool g_axisVelMode[CCROS_AXIS_COUNT];
static bool g_axisTrack[CCROS_AXIS_COUNT];
static float g_trackPos[CCROS_AXIS_COUNT];
static float g_trackVel[CCROS_AXIS_COUNT];
static bool g_trackDirty[CCROS_AXIS_COUNT];
static uint32_t g_trackLatchMs[CCROS_AXIS_COUNT];
static float g_velGoal[CCROS_AXIS_COUNT];
static bool g_velLatched[CCROS_AXIS_COUNT];
static int32_t g_velCmd[CCROS_AXIS_COUNT];
static uint32_t g_retryMs[CCROS_AXIS_COUNT];

static int32_t g_lastPosSteps[CCROS_AXIS_COUNT];
static uint32_t g_lastVelMs = 0;
static float g_velRos[CCROS_AXIS_COUNT];

static MotorDriver *MotorFor(uint8_t axis) {
    switch (axis) {
        case CCROS_AXIS_X: return &ConnectorM0;
        case CCROS_AXIS_Y: return &ConnectorM1;
        case CCROS_AXIS_Z: return &ConnectorM2;
        case CCROS_AXIS_A: return &ConnectorM3;
        default: return nullptr;
    }
}

static bool AxisOn(uint8_t axis) {
    return (g_axisMask & (1u << axis)) != 0;
}

static int32_t IAbs32(int32_t v) {
    return (v < 0) ? -v : v;
}

static int32_t RoundToI32(double v) {
    if (v >= 2147483646.0) {
        return 2147483646;
    }
    if (v <= -2147483646.0) {
        return -2147483646;
    }
    if (v >= 0.0) {
        return (int32_t)(v + 0.5);
    }
    return (int32_t)(v - 0.5);
}

static double StepsPerUnit(uint8_t axis) {
    if (g_stepsPerRev[axis] == 0) {
        return 0.0;
    }
    if (axis == CCROS_AXIS_A) {
        return (double)g_stepsPerRev[axis] / (2.0 * 3.14159265358979323846);
    }
    if (g_pitchMm[axis] <= 0.0) {
        return 0.0;
    }
    return (double)g_stepsPerRev[axis] * 1000.0 / g_pitchMm[axis];
}

static void ClearGoals() {
    for (uint8_t a = 0; a < CCROS_AXIS_COUNT; a++) {
        g_goalValid[a] = false;
        g_absIssued[a] = false;
        g_absRetried[a] = false;
        g_axisVelMode[a] = false;
        g_axisTrack[a] = false;
        g_trackDirty[a] = false;
        g_trackPos[a] = 0.f;
        g_trackVel[a] = 0.f;
        g_velGoal[a] = 0.f;
        g_velLatched[a] = false;
        g_velCmd[a] = 0;
        g_retryMs[a] = 0;
    }
}

static void ApplyDynamics() {
    for (uint8_t a = 0; a < CCROS_AXIS_COUNT; a++) {
        MotorDriver *m = MotorFor(a);
        if (!m) {
            continue;
        }
        m->VelMax(g_vel);
        m->AccelMax(g_accel);
        m->EStopDecelMax(g_decel);
    }
}

static void ApplyMechanics() {
    for (uint8_t a = 0; a < CCROS_AXIS_COUNT; a++) {
        MotorDriver *m = MotorFor(a);
        if (!m) {
            continue;
        }
        m->HlfbMode(MotorDriver::HLFB_MODE_HAS_BIPOLAR_PWM);
        m->HlfbCarrier(MotorDriver::HLFB_CARRIER_482_HZ);
        if (a == CCROS_AXIS_A) {
            m->SetMechanicalParams(g_stepsPerRev[a], 360.0, UNIT_DEGREES, 1.0);
        } else {
            m->SetMechanicalParams(g_stepsPerRev[a], g_pitchMm[a], UNIT_MM, 1.0);
        }
    }
    ApplyDynamics();
}

static void StopDecelAll() {
    for (uint8_t a = 0; a < CCROS_AXIS_COUNT; a++) {
        MotorDriver *m = MotorFor(a);
        if (m) {
            m->MoveStopDecel(g_decel);
        }
    }
    for (uint8_t a = 0; a < CCROS_AXIS_COUNT; a++) {
        g_velLatched[a] = false;
        g_velCmd[a] = 0;
        g_absIssued[a] = false;
    }
}

static void AbruptDisable() {
    for (uint8_t a = 0; a < CCROS_AXIS_COUNT; a++) {
        MotorDriver *m = MotorFor(a);
        if (!m) {
            continue;
        }
        m->MoveStopAbrupt();
        m->EnableRequest(false);
    }
    g_enabled = false;
    ClearGoals();
}

static bool HardwareEstop() {
    if (g_testMode || g_estopDi6 == 0) {
        return false;
    }
    const bool diOn = (ConnectorDI6.State() != 0);
    if (g_estopDi6 == 1) {
        return !diOn;
    }
    if (g_estopDi6 == 2) {
        return diOn;
    }
    return false;
}

static bool StepsActive(MotorDriver *m) {
    return m && m->StatusReg().bit.StepsActive;
}

static uint32_t AlertBits(uint8_t axis) {
    if (!AxisOn(axis)) {
        return 0;
    }
    MotorDriver *m = MotorFor(axis);
    if (!m) {
        return 0;
    }
    uint32_t bits = m->AlertReg().reg;
    /* Inactive or not-yet-enabled motors report motor_disabled. That bit is
     * MotionCanceledMotorDisabled (bit 4). Only an enabled axis counts. */
    if (!g_enabled) {
        bits &= ~(1u << 4);
    }
    return bits;
}

static bool AxisFaulted(uint8_t axis) {
    if (!AxisOn(axis)) {
        return false;
    }
    MotorDriver *m = MotorFor(axis);
    if (!m) {
        return false;
    }
    if (AlertBits(axis) != 0) {
        return true;
    }
    if (g_enabled && (m->StatusReg().bit.AlertsPresent || m->StatusReg().bit.MotorInFault)) {
        return true;
    }
    return false;
}

static bool AnyMoving() {
    for (uint8_t a = 0; a < CCROS_AXIS_COUNT; a++) {
        if (AxisOn(a) && StepsActive(MotorFor(a))) {
            return true;
        }
    }
    return false;
}

static bool MotionPending() {
    if (AnyMoving()) {
        return true;
    }
    for (uint8_t a = 0; a < CCROS_AXIS_COUNT; a++) {
        if (!AxisOn(a)) {
            continue;
        }
        MotorDriver *m = MotorFor(a);
        if (!m) {
            continue;
        }
        if ((g_axisVelMode[a] || g_axisTrack[a]) && g_velCmd[a] != 0) {
            return true;
        }
        if (g_axisTrack[a] && IAbs32(RoundToI32((double)g_trackPos[a] * StepsPerUnit(a)) -
                                    m->PositionRefCommanded()) > 1) {
            return true;
        }
        if (g_goalValid[a] && IAbs32(g_goalSteps[a] - m->PositionRefCommanded()) > 1) {
            return true;
        }
    }
    return false;
}

static bool RetryReady(uint8_t axis, uint32_t now) {
    return (now - g_retryMs[axis]) >= CCROS_MOVE_RETRY_MS;
}

static uint32_t BrakeSteps() {
    if (g_accel == 0) {
        return 0;
    }
    const double s = ((double)g_vel * (double)g_vel) / (2.0 * (double)g_accel);
    if (s > 2000000000.0) {
        return 2000000000u;
    }
    return (uint32_t)s;
}

static void LatchVelocity(uint8_t axis, MotorDriver *m, int32_t sps, uint32_t now) {
    if (sps == 0) {
        if (g_velLatched[axis] || StepsActive(m)) {
            m->MoveStopDecel(g_decel);
        }
        g_velLatched[axis] = false;
        g_velCmd[axis] = 0;
        return;
    }
    int32_t delta = sps - g_velCmd[axis];
    if (delta < 0) {
        delta = -delta;
    }
    const int32_t thresh = CcrosVelocityDeadband(sps, g_velCmd[axis]);
    const bool signChange =
        g_velLatched[axis] && g_velCmd[axis] != 0 && ((sps > 0) != (g_velCmd[axis] > 0));
    if (g_velLatched[axis] && !signChange && delta < thresh) {
        return;
    }
    if (!RetryReady(axis, now)) {
        return;
    }
    if (m->MoveVelocity(sps)) {
        g_velLatched[axis] = true;
        g_velCmd[axis] = sps;
    } else {
        g_retryMs[axis] = now;
    }
}

/* Position correction runs only when a new track frame sets g_trackDirty.
 * At that instant t - t_latch is ~0, so the error is versus q_latched, not
 * versus q_latched + v*(t - t_latch). Between frames the latched target is
 * held and this function does not run. */
static void ServiceTrack(uint8_t axis, uint32_t now) {
    if (!g_trackDirty[axis]) {
        return;
    }
    g_trackDirty[axis] = false;
    MotorDriver *m = MotorFor(axis);
    const double spu = StepsPerUnit(axis);
    if (!m || spu <= 0.0) {
        return;
    }
    const int32_t scheduled = RoundToI32((double)g_trackPos[axis] * spu);
    const int32_t ff = RoundToI32((double)g_trackVel[axis] * spu);
    const int32_t err = scheduled - m->PositionRefCommanded();
    int32_t corr_limit = (int32_t)(g_vel / 4u);
    if (corr_limit < 1) {
        corr_limit = 1;
    }
    const int32_t sps = CcrosTrackVelocity(ff, err, CCROS_TRACK_KP, corr_limit, (int32_t)g_vel);
    LatchVelocity(axis, m, sps, now);
}

static void ServiceVelocity(uint8_t axis, uint32_t now) {
    MotorDriver *m = MotorFor(axis);
    const double spu = StepsPerUnit(axis);
    if (!m || spu <= 0.0) {
        return;
    }
    int32_t sps = RoundToI32((double)g_velGoal[axis] * spu);
    const int32_t cap = (int32_t)g_vel;
    if (sps > cap) {
        sps = cap;
    }
    if (sps < -cap) {
        sps = -cap;
    }
    LatchVelocity(axis, m, sps, now);
}

static void ServicePosition(uint8_t axis, uint32_t now) {
    MotorDriver *m = MotorFor(axis);
    if (!m || !g_goalValid[axis]) {
        return;
    }
    const int32_t cur = m->PositionRefCommanded();
    const int32_t err = g_goalSteps[axis] - cur;
    const int32_t absErr = IAbs32(err);
    const bool stable = (now - g_goalChangedMs[axis]) >= CCROS_GOAL_STABLE_MS;

    if (stable) {
        if (absErr <= 1 && g_velLatched[axis]) {
            m->MoveStopDecel(g_decel);
            g_velLatched[axis] = false;
            g_velCmd[axis] = 0;
        }
        if (g_absIssued[axis] && absErr > 1 && !StepsActive(m) && !g_absRetried[axis]) {
            g_absIssued[axis] = false;
            g_absRetried[axis] = true;
        }
        if (!g_absIssued[axis] && absErr > 0 && RetryReady(axis, now)) {
            if (m->Move(g_goalSteps[axis], StepGenerator::MOVE_TARGET_ABSOLUTE)) {
                g_absIssued[axis] = true;
                g_velLatched[axis] = false;
                g_velCmd[axis] = 0;
            } else {
                g_retryMs[axis] = now;
            }
        }
        return;
    }

    if (absErr <= 2) {
        if (g_velLatched[axis]) {
            m->MoveStopDecel(g_decel);
            g_velLatched[axis] = false;
            g_velCmd[axis] = 0;
        }
        return;
    }

    int32_t sps;
    const uint32_t brake = BrakeSteps();
    if ((uint32_t)absErr > brake) {
        sps = (err > 0) ? (int32_t)g_vel : -(int32_t)g_vel;
    } else {
        const double scale = (brake > 0) ? ((double)absErr / (double)brake) : 1.0;
        int32_t mag = (int32_t)((double)g_vel * scale);
        if (mag < 50) {
            mag = 50;
        }
        sps = (err > 0) ? mag : -mag;
    }
    LatchVelocity(axis, m, sps, now);
}

static void UpdateVelocityEstimate() {
    const uint32_t now = Milliseconds();
    if (g_lastVelMs == 0 || (now - g_lastVelMs) < CCROS_STREAM_PERIOD_MS) {
        if (g_lastVelMs == 0) {
            for (uint8_t a = 0; a < CCROS_AXIS_COUNT; a++) {
                MotorDriver *m = MotorFor(a);
                g_lastPosSteps[a] = m ? m->PositionRefCommanded() : 0;
                g_velRos[a] = 0.f;
            }
            g_lastVelMs = now;
        }
        return;
    }
    const double dt = (double)(now - g_lastVelMs) / 1000.0;
    for (uint8_t a = 0; a < CCROS_AXIS_COUNT; a++) {
        MotorDriver *m = MotorFor(a);
        const int32_t pos = m ? m->PositionRefCommanded() : 0;
        const double spu = StepsPerUnit(a);
        if (spu > 0.0 && dt > 0.0) {
            g_velRos[a] = (float)(((double)(pos - g_lastPosSteps[a]) / spu) / dt);
        } else {
            g_velRos[a] = 0.f;
        }
        g_lastPosSteps[a] = pos;
    }
    g_lastVelMs = now;
}

static const char *StorePosition(uint8_t mask, const float q[4], uint16_t seq, bool fromSession) {
    if (g_interrupted) {
        return "estop active";
    }
    if (g_watchdogTripped) {
        return "watchdog tripped; call clear_alerts";
    }
    g_lastHostMs = Milliseconds();
    g_lastCmdSeq = seq;
    if (!g_enabled) {
        return fromSession ? "motor not enabled" : nullptr;
    }
    const uint32_t now = g_lastHostMs;
    if (fromSession && (mask & (uint8_t)g_axisMask) != mask) {
        return "joint is outside axis_mask";
    }
    const uint8_t use = (uint8_t)(mask & (uint8_t)g_axisMask);
    if (use == 0) {
        return fromSession ? "no joints in axis_mask" : nullptr;
    }
    for (uint8_t a = 0; a < CCROS_AXIS_COUNT; a++) {
        if ((use & (1u << a)) == 0) {
            continue;
        }
        if (!(q[a] > -1.0e9f && q[a] < 1.0e9f)) {
            if (fromSession) {
                return "joint value is not a number";
            }
            continue;
        }
        const double spu = StepsPerUnit(a);
        if (spu <= 0.0) {
            return "axis is not configured";
        }
        const int32_t steps = RoundToI32((double)q[a] * spu);
        if (!g_goalValid[a] || steps != g_goalSteps[a]) {
            g_goalSteps[a] = steps;
            g_goalValid[a] = true;
            g_goalChangedMs[a] = now;
            g_absIssued[a] = false;
            g_absRetried[a] = false;
            g_velLatched[a] = false;
        }
        g_axisVelMode[a] = false;
        g_axisTrack[a] = false;
        g_trackDirty[a] = false;
    }
    return nullptr;
}

bool MotionInit() {
    for (uint8_t a = 0; a < CCROS_AXIS_COUNT; a++) {
        g_stepsPerRev[a] = CCROS_DEFAULT_STEPS_PER_REV;
        g_pitchMm[a] = CCROS_DEFAULT_PITCH_MM;
        g_goalValid[a] = false;
        g_velRos[a] = 0.f;
    }
    g_axisMask = CCROS_DEFAULT_AXIS_MASK;
    g_vel = CCROS_DEFAULT_VEL_STEPS;
    g_accel = CCROS_DEFAULT_ACCEL_STEPS;
    g_decel = CCROS_DEFAULT_DECEL_STEPS;
    g_watchdogMs = CCROS_DEFAULT_WATCHDOG_MS;
    g_estopDi6 = CCROS_DEFAULT_ESTOP_DI6;
    g_testMode = false;
    g_enabled = false;
    g_interrupted = false;
    g_watchdogTripped = false;
    g_lastHostMs = 0;
    ClearGoals();

    MotorMgr.MotorModeSet(MotorManager::MOTOR_ALL, Connector::CPM_MODE_STEP_AND_DIR);
    Delay_ms(50);
    ConnectorDI6.Mode(Connector::INPUT_DIGITAL);

    ConnectorM0.EnableRequest(false);
    ConnectorM1.EnableRequest(false);
    ConnectorM2.EnableRequest(false);
    ConnectorM3.EnableRequest(false);
    ApplyMechanics();
    for (uint8_t a = 0; a < CCROS_AXIS_COUNT; a++) {
        MotorDriver *m = MotorFor(a);
        if (m) {
            m->PositionRefSet(0);
        }
    }
    return true;
}

void MotionPoll() {
    if (HardwareEstop()) {
        if (!g_interrupted) {
            AbruptDisable();
            g_interrupted = true;
        }
        return;
    }
    const uint32_t now = Milliseconds();
    if (g_watchdogMs != 0 && g_enabled && !g_watchdogTripped && g_lastHostMs != 0 &&
        MotionPending() && (now - g_lastHostMs) >= g_watchdogMs) {
        StopDecelAll();
        ClearGoals();
        g_watchdogTripped = true;
    }
    if (!g_enabled || g_interrupted || g_watchdogTripped) {
        return;
    }
    for (uint8_t a = 0; a < CCROS_AXIS_COUNT; a++) {
        if (!AxisOn(a)) {
            continue;
        }
        if (g_axisTrack[a]) {
            ServiceTrack(a, now);
        } else if (g_axisVelMode[a]) {
            ServiceVelocity(a, now);
        } else if (g_goalValid[a]) {
            ServicePosition(a, now);
        }
    }
}

bool MotionIsEnabled() {
    return g_enabled;
}

const char *MotionConfigure(const MotionConfigPatch *patch) {
    if (!patch) {
        return "missing params";
    }
    const bool mechanics = patch->hasAxisMask || patch->hasSteps || patch->hasPitch;
    if (mechanics && g_enabled) {
        return "disable before changing mechanics";
    }
    if (patch->hasAxisMask) {
        if (patch->axisMask == 0 || patch->axisMask > 0x0f) {
            return "axis_mask must be 1..15";
        }
        g_axisMask = patch->axisMask;
    }
    if (patch->hasSteps) {
        for (uint8_t a = 0; a < CCROS_AXIS_COUNT; a++) {
            if (patch->stepsPerRev[a] == 0 || patch->stepsPerRev[a] > 1000000u) {
                return "steps_per_rev out of range";
            }
            g_stepsPerRev[a] = patch->stepsPerRev[a];
        }
    }
    if (patch->hasPitch) {
        for (uint8_t a = 0; a < CCROS_AXIS_COUNT; a++) {
            if (a == CCROS_AXIS_A) {
                continue;
            }
            if (patch->pitchMm[a] < 0.01 || patch->pitchMm[a] > 1000.0) {
                return "pitch_mm out of range";
            }
            g_pitchMm[a] = patch->pitchMm[a];
        }
    }
    if (patch->hasVel) {
        if (patch->vel == 0 || patch->vel > 500000u) {
            return "vel_steps out of range";
        }
        g_vel = patch->vel;
    }
    if (patch->hasAccel) {
        if (patch->accel == 0 || patch->accel > 5000000u) {
            return "accel_steps out of range";
        }
        g_accel = patch->accel;
    }
    if (patch->hasDecel) {
        if (patch->decel == 0 || patch->decel > 5000000u) {
            return "decel_steps out of range";
        }
        g_decel = patch->decel;
    }
    if (patch->hasWatchdog) {
        if (patch->watchdogMs > 60000u) {
            return "watchdog_ms out of range";
        }
        g_watchdogMs = patch->watchdogMs;
    }
    if (patch->hasEstop) {
        if (patch->estopDi6 > 2) {
            return "estop_di6 must be 0, 1, or 2";
        }
        g_estopDi6 = patch->estopDi6;
    }
    if (mechanics) {
        ApplyMechanics();
    } else if (patch->hasVel || patch->hasAccel || patch->hasDecel) {
        ApplyDynamics();
    }
    return nullptr;
}

const char *MotionSetTestMode(bool on) {
    g_testMode = on;
    if (!on && HardwareEstop()) {
        AbruptDisable();
        g_interrupted = true;
    }
    return nullptr;
}

const char *MotionEnable() {
    if (!g_testMode && HardwareEstop()) {
        g_interrupted = true;
        return "hardware estop";
    }
    if (g_watchdogTripped) {
        return "watchdog tripped; call clear_alerts";
    }
    g_interrupted = false;
    for (uint8_t a = 0; a < CCROS_AXIS_COUNT; a++) {
        MotorDriver *m = MotorFor(a);
        if (!m) {
            continue;
        }
        m->EnableRequest(AxisOn(a));
    }
    if (!g_testMode) {
        const uint32_t start = Milliseconds();
        bool ready = false;
        while (Milliseconds() - start < CCROS_ENABLE_HLFB_WAIT_MS) {
            ready = true;
            for (uint8_t a = 0; a < CCROS_AXIS_COUNT; a++) {
                if (!AxisOn(a)) {
                    continue;
                }
                MotorDriver *m = MotorFor(a);
                if (m && m->HlfbState() != MotorDriver::HLFB_ASSERTED) {
                    ready = false;
                }
            }
            if (ready) {
                break;
            }
            Delay_ms(1);
        }
        if (!ready) {
            for (uint8_t a = 0; a < CCROS_AXIS_COUNT; a++) {
                MotorDriver *m = MotorFor(a);
                if (m) {
                    m->EnableRequest(false);
                }
            }
            g_enabled = false;
            return "HLFB not ready";
        }
    }
    g_enabled = true;
    g_lastHostMs = Milliseconds();
    return nullptr;
}

const char *MotionDisable() {
    AbruptDisable();
    return nullptr;
}

const char *MotionStop() {
    StopDecelAll();
    ClearGoals();
    return nullptr;
}

const char *MotionEstop() {
    AbruptDisable();
    g_interrupted = true;
    return nullptr;
}

const char *MotionClearAlerts() {
    for (uint8_t a = 0; a < CCROS_AXIS_COUNT; a++) {
        MotorDriver *m = MotorFor(a);
        if (m) {
            m->ClearAlerts();
        }
    }
    g_watchdogTripped = false;
    g_lastHostMs = Milliseconds();
    if (!HardwareEstop()) {
        g_interrupted = false;
    }
    return nullptr;
}

const char *MotionKeepalive() {
    if (g_watchdogTripped) {
        return "watchdog tripped; call clear_alerts";
    }
    g_lastHostMs = Milliseconds();
    return nullptr;
}

void MotionNoteHost() {
    if (!g_watchdogTripped && !g_interrupted) {
        g_lastHostMs = Milliseconds();
    }
}

void MotionStreamLost() {
    if (MotionPending()) {
        StopDecelAll();
        ClearGoals();
        g_watchdogTripped = true;
    }
}

const char *MotionSetJoints(uint8_t mask, const float q[4]) {
    return StorePosition(mask, q, g_lastCmdSeq, true);
}

void MotionNotePosition(uint16_t seq, uint8_t mask, const float q[4]) {
    (void)StorePosition(mask, q, seq, false);
}

void MotionNoteVelocity(uint16_t seq, uint8_t mask, const float v[4]) {
    if (g_interrupted || g_watchdogTripped) {
        return;
    }
    g_lastHostMs = Milliseconds();
    g_lastCmdSeq = seq;
    if (!g_enabled) {
        return;
    }
    const uint8_t use = (uint8_t)(mask & (uint8_t)g_axisMask);
    for (uint8_t a = 0; a < CCROS_AXIS_COUNT; a++) {
        if ((use & (1u << a)) == 0) {
            continue;
        }
        if (!(v[a] > -1.0e9f && v[a] < 1.0e9f)) {
            continue;
        }
        g_axisVelMode[a] = true;
        g_axisTrack[a] = false;
        g_trackDirty[a] = false;
        g_goalValid[a] = false;
        g_velGoal[a] = v[a];
    }
}

void MotionNoteTrack(uint16_t seq, uint8_t mask, const float q[4], const float v[4]) {
    if (g_interrupted || g_watchdogTripped) {
        return;
    }
    g_lastHostMs = Milliseconds();
    g_lastCmdSeq = seq;
    if (!g_enabled) {
        return;
    }
    const uint8_t use = (uint8_t)(mask & (uint8_t)g_axisMask);
    for (uint8_t a = 0; a < CCROS_AXIS_COUNT; a++) {
        if ((use & (1u << a)) == 0) {
            continue;
        }
        if (!(q[a] > -1.0e9f && q[a] < 1.0e9f)) {
            continue;
        }
        if (!(v[a] > -1.0e9f && v[a] < 1.0e9f)) {
            continue;
        }
        g_axisTrack[a] = true;
        g_axisVelMode[a] = false;
        g_goalValid[a] = false;
        g_trackPos[a] = q[a];
        g_trackVel[a] = v[a];
        g_trackLatchMs[a] = Milliseconds();
        g_trackDirty[a] = true;
    }
}

static void AppendAlertName(char *dst, uint16_t len, bool *first, const char *name) {
    const size_t used = strlen(dst);
    if (used + 1 >= len) {
        return;
    }
    snprintf(dst + used, len - used, "%s%s", (*first) ? "" : ",", name);
    *first = false;
}

static void FormatAlerts(char *dst, uint16_t len) {
    dst[0] = '\0';
    uint32_t bits = 0;
    for (uint8_t a = 0; a < CCROS_AXIS_COUNT; a++) {
        bits |= AlertBits(a);
    }
    if (bits == 0) {
        snprintf(dst, len, "none");
        return;
    }
    bool first = true;
    if (bits & (1u << 0)) AppendAlertName(dst, len, &first, "in_alert");
    if (bits & (1u << 1)) AppendAlertName(dst, len, &first, "pos_limit");
    if (bits & (1u << 2)) AppendAlertName(dst, len, &first, "neg_limit");
    if (bits & (1u << 3)) AppendAlertName(dst, len, &first, "sensor_estop");
    if (bits & (1u << 4)) AppendAlertName(dst, len, &first, "motor_disabled");
    if (bits & (1u << 5)) AppendAlertName(dst, len, &first, "motor_faulted");
    if (dst[0] == '\0') {
        snprintf(dst, len, "alert_0x%lx", (unsigned long)bits);
    }
}

void MotionFillState(CcrosState *out) {
    UpdateVelocityEstimate();
    memset(out, 0, sizeof(*out));
    out->time_ms = Milliseconds();
    out->axis_mask = (uint8_t)g_axisMask;
    if (g_enabled) out->flags |= CCROS_FLAG_ENABLED;
    if (AnyMoving()) out->flags |= CCROS_FLAG_MOVING;
    if (g_interrupted || HardwareEstop()) out->flags |= CCROS_FLAG_ESTOP;
    if (g_watchdogTripped) out->flags |= CCROS_FLAG_WATCHDOG;
    uint32_t alerts = 0;
    bool fault = false;
    for (uint8_t a = 0; a < CCROS_AXIS_COUNT; a++) {
        alerts |= AlertBits(a);
        if (AxisFaulted(a)) {
            fault = true;
        }
        MotorDriver *m = MotorFor(a);
        const double spu = StepsPerUnit(a);
        const int32_t steps = m ? m->PositionRefCommanded() : 0;
        out->position[a] = (spu > 0.0) ? (float)((double)steps / spu) : 0.f;
        out->velocity[a] = g_velRos[a];
        float duty = 0.f;
        if (m && AxisOn(a)) {
            duty = m->HlfbPercent();
        }
        if (duty >= -100.f && duty <= 100.f) {
            out->effort[a] = duty / 100.f;
        }
        if (g_axisTrack[a]) {
            out->track_mask = (uint8_t)(out->track_mask | (1u << a));
            out->target_position[a] = g_trackPos[a];
            out->target_velocity[a] = g_trackVel[a];
        } else if (g_goalValid[a] && spu > 0.0) {
            out->target_position[a] = (float)((double)g_goalSteps[a] / spu);
        } else {
            out->target_position[a] = out->position[a];
        }
        if (spu > 0.0) {
            out->command_velocity[a] = (float)((double)g_velCmd[a] / spu);
        }
        out->target_latch_ms[a] = g_trackLatchMs[a];
    }
    out->alert_reg = alerts;
    if (fault) {
        out->flags |= CCROS_FLAG_FAULT;
    }
}

void MotionFillCapabilitiesJson(char *buf, uint16_t len) {
    snprintf(buf, len,
             "{\"protocol\":\"%s\",\"firmware\":\"%s\",\"session_port\":%u,"
             "\"stream_port\":%u,\"discover_port\":%u,\"stream_hz\":%u,"
             "\"joints\":[\"joint_x\",\"joint_y\",\"joint_z\",\"joint_a\"],"
             "\"joint_types\":[\"prismatic\",\"prismatic\",\"prismatic\",\"revolute\"],"
             "\"units\":[\"m\",\"m\",\"m\",\"rad\"],\"axis_mask\":%lu}",
             CCROS_PROTOCOL_VERSION, CCROS_FIRMWARE_NAME,
             (unsigned)CCROS_TCP_SESSION_PORT, (unsigned)CCROS_TCP_STREAM_PORT,
             (unsigned)CCROS_UDP_DISCOVERY_PORT,
             (unsigned)(1000u / CCROS_STREAM_PERIOD_MS),
             (unsigned long)g_axisMask);
}

void MotionFillConfigJson(char *buf, uint16_t len) {
    snprintf(buf, len,
             "{\"axis_mask\":%lu,\"steps_per_rev\":[%lu,%lu,%lu,%lu],"
             "\"pitch_mm\":[%.4f,%.4f,%.4f,%.4f],\"vel_steps\":%lu,"
             "\"accel_steps\":%lu,\"decel_steps\":%lu,\"watchdog_ms\":%lu,"
             "\"estop_di6\":%u,\"test_mode\":%s,\"enabled\":%s}",
             (unsigned long)g_axisMask,
             (unsigned long)g_stepsPerRev[0], (unsigned long)g_stepsPerRev[1],
             (unsigned long)g_stepsPerRev[2], (unsigned long)g_stepsPerRev[3],
             g_pitchMm[0], g_pitchMm[1], g_pitchMm[2], g_pitchMm[3],
             (unsigned long)g_vel, (unsigned long)g_accel, (unsigned long)g_decel,
             (unsigned long)g_watchdogMs, (unsigned)g_estopDi6,
             g_testMode ? "true" : "false", g_enabled ? "true" : "false");
}

void MotionFillStatusJson(char *buf, uint16_t len) {
    CcrosState st;
    MotionFillState(&st);
    char alerts[96];
    FormatAlerts(alerts, sizeof(alerts));
    snprintf(buf, len,
             "{\"enabled\":%s,\"moving\":%s,\"estop\":%s,\"fault\":%s,"
             "\"watchdog\":%s,\"test_mode\":%s,\"axis_mask\":%u,\"alert_reg\":%lu,"
             "\"alerts\":\"%s\",\"last_cmd_seq\":%u,"
             "\"position\":[%.6f,%.6f,%.6f,%.6f],"
             "\"velocity\":[%.6f,%.6f,%.6f,%.6f],"
             "\"effort\":[%.4f,%.4f,%.4f,%.4f]}",
             (st.flags & CCROS_FLAG_ENABLED) ? "true" : "false",
             (st.flags & CCROS_FLAG_MOVING) ? "true" : "false",
             (st.flags & CCROS_FLAG_ESTOP) ? "true" : "false",
             (st.flags & CCROS_FLAG_FAULT) ? "true" : "false",
             (st.flags & CCROS_FLAG_WATCHDOG) ? "true" : "false",
             g_testMode ? "true" : "false",
             (unsigned)st.axis_mask, (unsigned long)st.alert_reg, alerts,
             (unsigned)g_lastCmdSeq,
             st.position[0], st.position[1], st.position[2], st.position[3],
             st.velocity[0], st.velocity[1], st.velocity[2], st.velocity[3],
             st.effort[0], st.effort[1], st.effort[2], st.effort[3]);
}
