/*
 * Copyright (c) 2026 Adam G. Sweeney <agsweeney@gmail.com>
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#include "MotionBridge.h"

#include "ClearCore.h"
#include "SysTiming.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

#define motorX ConnectorM0
#define motorY ConnectorM1
#define motorZ ConnectorM2
#define motorA ConnectorM3

static CoordinatedMotionController g_xy;
static bool g_enabled = false;
static bool g_interrupted = false;
static ClearAiUnits g_units = CLEARAI_UNITS_MM;
static ClearAiMode g_mode = CLEARAI_MODE_ABS;
static uint32_t g_axisMask = CLEARAI_DEFAULT_AXIS_MASK;
static uint32_t g_vel = CLEARAI_DEFAULT_VEL_STEPS;
static uint32_t g_accel = CLEARAI_DEFAULT_ACCEL_STEPS;
static uint32_t g_decel = CLEARAI_DEFAULT_DECEL_STEPS;
static uint8_t g_estopDi6 = 1;
static bool g_testMode = (CLEARAI_TEST_MODE_DEFAULT != 0);
static double g_feedMmPerMin = CLEARAI_DEFAULT_FEED_MM_PER_MIN;
static double g_workOrigin[CLEARAI_AXIS_COUNT];
static uint32_t g_stepsPerRev[CLEARAI_AXIS_COUNT];
static double g_pitchMm[CLEARAI_AXIS_COUNT];
static double g_gear[CLEARAI_AXIS_COUNT];
static double g_stepsPerMm[CLEARAI_AXIS_COUNT]; // A: steps per degree

static int32_t I32Round(double v) {
    return (int32_t)(v >= 0.0 ? v + 0.5 : v - 0.5);
}

static MotorDriver *MotorForAxis(uint8_t axis) {
    switch (axis) {
        case CLEARAI_AXIS_X: return &motorX;
        case CLEARAI_AXIS_Y: return &motorY;
        case CLEARAI_AXIS_Z: return &motorZ;
        case CLEARAI_AXIS_A: return &motorA;
        default: return nullptr;
    }
}

static bool AxisOn(uint8_t axis) {
    return (g_axisMask & (1u << axis)) != 0;
}

static void RecalcStepsPerUnit() {
    for (uint8_t a = 0; a < CLEARAI_AXIS_COUNT; a++) {
        double pitch = g_pitchMm[a];
        if (pitch <= 0.0) {
            pitch = CLEARAI_DEFAULT_PITCH_MM;
        }
        double gear = g_gear[a];
        if (gear <= 0.0) {
            gear = 1.0;
        }
        if (a == CLEARAI_AXIS_A) {
            g_stepsPerMm[a] = ((double)g_stepsPerRev[a] * gear) / 360.0;
        } else {
            g_stepsPerMm[a] = ((double)g_stepsPerRev[a] * gear) / pitch;
        }
    }
}

static double ToInternal(uint8_t axis, double user) {
    if (axis == CLEARAI_AXIS_A) {
        return user;
    }
    if (g_units == CLEARAI_UNITS_INCH) {
        return user * 25.4;
    }
    return user;
}

static double FromInternal(uint8_t axis, double internal) {
    if (axis == CLEARAI_AXIS_A) {
        return internal;
    }
    if (g_units == CLEARAI_UNITS_INCH) {
        return internal / 25.4;
    }
    return internal;
}

static int32_t MachineSteps(uint8_t axis) {
    MotorDriver *m = MotorForAxis(axis);
    return m ? m->PositionRefCommanded() : 0;
}

static double MachineInternal(uint8_t axis) {
    const double spu = g_stepsPerMm[axis];
    if (spu == 0.0) {
        return 0.0;
    }
    return (double)MachineSteps(axis) / spu;
}

static double WorkInternal(uint8_t axis) {
    return MachineInternal(axis) - g_workOrigin[axis];
}

static bool HardwareEstopFaultRaw() {
    if (g_estopDi6 == 0) {
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

static bool HardwareEstopFault() {
    if (g_testMode) {
        return false;
    }
    return HardwareEstopFaultRaw();
}

static bool AxisHealthy(uint8_t axis) {
    if (!AxisOn(axis)) {
        return false;
    }
    MotorDriver *m = MotorForAxis(axis);
    if (!m) {
        return false;
    }
    if (!m->EnableRequest()) {
        return false;
    }
    if (m->StatusReg().bit.AlertsPresent || m->StatusReg().bit.MotorInFault) {
        return false;
    }
    return true;
}

static bool AnyAlert() {
    for (uint8_t a = 0; a < CLEARAI_AXIS_COUNT; a++) {
        if (!AxisOn(a)) {
            continue;
        }
        MotorDriver *m = MotorForAxis(a);
        if (!m) {
            continue;
        }
        if (m->StatusReg().bit.AlertsPresent || m->StatusReg().bit.MotorInFault) {
            return true;
        }
    }
    return false;
}

static bool AnyHealthyAxis() {
    for (uint8_t a = 0; a < CLEARAI_AXIS_COUNT; a++) {
        if (AxisHealthy(a)) {
            return true;
        }
    }
    return false;
}

static bool HlfbOk() {
    for (uint8_t a = 0; a < CLEARAI_AXIS_COUNT; a++) {
        if (!AxisOn(a)) {
            continue;
        }
        MotorDriver *m = MotorForAxis(a);
        if (!m) {
            continue;
        }
        if (m->HlfbState() != MotorDriver::HLFB_ASSERTED) {
            return false;
        }
    }
    return true;
}

static bool MotorsIdle() {
    if (g_xy.IsActive() || g_xy.MotionQueueCount() != 0) {
        return false;
    }
    for (uint8_t a = 0; a < CLEARAI_AXIS_COUNT; a++) {
        if (!AxisOn(a)) {
            continue;
        }
        MotorDriver *m = MotorForAxis(a);
        if (!m) {
            continue;
        }
        if (!m->StepsComplete()) {
            return false;
        }
        if (!m->QuadratureOutputIdle()) {
            return false;
        }
    }
    return true;
}

static void ApplyLimits() {
    for (uint8_t a = 0; a < CLEARAI_AXIS_COUNT; a++) {
        MotorDriver *m = MotorForAxis(a);
        if (!m) {
            continue;
        }
        m->HlfbMode(MotorDriver::HLFB_MODE_HAS_BIPOLAR_PWM);
        m->HlfbCarrier(MotorDriver::HLFB_CARRIER_482_HZ);
        m->VelMax(g_vel);
        m->AccelMax(g_accel);
        m->EStopDecelMax(g_decel);
    }
    g_xy.ArcVelMax(g_vel);
    g_xy.ArcAccelMax(g_accel);
}

static const char *GateMotion() {
    if (g_testMode) {
        if (g_interrupted) {
            return "estop active";
        }
        if (!g_enabled) {
            MotionEnable();
        }
        return nullptr;
    }
    if (!g_enabled) {
        return "motor not enabled";
    }
    /* Empty/faulted partner axes (one-motor bench, default XY mask) must not
     * block a healthy installed motor. */
    if (AnyAlert() && !AnyHealthyAxis()) {
        return "motor in alert";
    }
    if (g_interrupted) {
        return "estop active";
    }
    return nullptr;
}

static void DisableAbrupt() {
    g_xy.Stop();
    for (uint8_t a = 0; a < CLEARAI_AXIS_COUNT; a++) {
        MotorDriver *m = MotorForAxis(a);
        if (!m) {
            continue;
        }
        m->MoveStopAbrupt();
        m->EnableRequest(false);
    }
    g_enabled = false;
}

void MotionPollEstop() {
    if (g_testMode) {
        return;
    }
    if (HardwareEstopFault()) {
        DisableAbrupt();
        g_interrupted = true;
    }
}

bool MotionInterrupted() {
    return g_interrupted;
}

void MotionClearInterrupt() {
    if (!HardwareEstopFault()) {
        g_interrupted = false;
    }
}

bool MotionIsEnabled() {
    return g_enabled;
}

bool MotionIsMoving() {
    return !MotorsIdle();
}

bool MotionInit() {
    for (uint8_t a = 0; a < CLEARAI_AXIS_COUNT; a++) {
        g_stepsPerRev[a] = CLEARAI_DEFAULT_STEPS_PER_REV;
        g_pitchMm[a] = CLEARAI_DEFAULT_PITCH_MM;
        g_gear[a] = 1.0;
        g_workOrigin[a] = 0.0;
    }
    RecalcStepsPerUnit();
    g_axisMask = CLEARAI_DEFAULT_AXIS_MASK;
    g_units = CLEARAI_UNITS_MM;
    g_mode = CLEARAI_MODE_ABS;
    g_vel = CLEARAI_DEFAULT_VEL_STEPS;
    g_accel = CLEARAI_DEFAULT_ACCEL_STEPS;
    g_decel = CLEARAI_DEFAULT_DECEL_STEPS;
    g_feedMmPerMin = CLEARAI_DEFAULT_FEED_MM_PER_MIN;
    g_estopDi6 = 1;
    g_testMode = (CLEARAI_TEST_MODE_DEFAULT != 0);
    g_enabled = false;
    g_interrupted = false;

    MotorMgr.MotorModeSet(MotorManager::MOTOR_ALL, Connector::CPM_MODE_STEP_AND_DIR);
    Delay_ms(50);
    ConnectorDI6.Mode(Connector::INPUT_DIGITAL);

    motorX.EnableRequest(false);
    motorY.EnableRequest(false);
    motorZ.EnableRequest(false);
    motorA.EnableRequest(false);

    if (!g_xy.Initialize(&motorX, &motorY)) {
        return false;
    }
    g_xy.SetMechanicalParamsX(g_stepsPerRev[0], g_pitchMm[0], UNIT_MM, g_gear[0]);
    g_xy.SetMechanicalParamsY(g_stepsPerRev[1], g_pitchMm[1], UNIT_MM, g_gear[1]);
    motorZ.SetMechanicalParams(g_stepsPerRev[2], g_pitchMm[2], UNIT_MM, g_gear[2]);
    motorA.SetMechanicalParams(g_stepsPerRev[3], 360.0, UNIT_DEGREES, g_gear[3]);
    g_xy.SetPosition(0, 0);
    motorZ.PositionRefSet(0);
    motorA.PositionRefSet(0);
    g_xy.StopAtQueueEnd(true);
    ApplyLimits();
    return true;
}

const char *MotionEnable() {
    MotionClearInterrupt();
    if (!g_testMode && HardwareEstopFault()) {
        g_interrupted = true;
        return "hardware estop";
    }
    for (uint8_t a = 0; a < CLEARAI_AXIS_COUNT; a++) {
        if (!AxisOn(a)) {
            continue;
        }
        MotorDriver *m = MotorForAxis(a);
        if (m) {
            m->EnableRequest(true);
        }
    }
    if (!g_testMode) {
        const uint32_t start = Milliseconds();
        while (Milliseconds() - start < CLEARAI_ENABLE_HLFB_WAIT_MS) {
            if (HlfbOk()) {
                break;
            }
            Delay_ms(1);
        }
    }
    g_enabled = true;
    return nullptr;
}

const char *MotionDisable() {
    DisableAbrupt();
    return nullptr;
}

const char *MotionClearAlerts() {
    for (uint8_t a = 0; a < CLEARAI_AXIS_COUNT; a++) {
        MotorDriver *m = MotorForAxis(a);
        if (m) {
            m->ClearAlerts();
        }
    }
    MotionClearInterrupt();
    return nullptr;
}

const char *MotionStop() {
    g_xy.StopDecel();
    for (uint8_t a = 0; a < CLEARAI_AXIS_COUNT; a++) {
        MotorDriver *m = MotorForAxis(a);
        if (m) {
            m->MoveStopDecel(g_decel);
        }
    }
    return nullptr;
}

const char *MotionEstop() {
    DisableAbrupt();
    g_interrupted = true;
    return nullptr;
}

static void ApplyTestMode(bool on) {
    g_testMode = on;
    if (g_testMode) {
        g_interrupted = false;
    }
}

const char *MotionSetTestMode(const RpcParams *p) {
    bool on = true;
    if (p->hasTestMode) {
        on = p->testMode;
    } else if (p->hasEnabled) {
        on = p->enabled;
    }
    ApplyTestMode(on);
    return nullptr;
}

const char *MotionConfigure(const RpcParams *p) {
    if (p->hasTestMode) {
        ApplyTestMode(p->testMode);
    }
    const bool needsDisable = p->hasStepsX || p->hasStepsY || p->hasStepsZ || p->hasStepsA ||
        p->hasPitchX || p->hasPitchY || p->hasPitchZ || p->hasPitchA ||
        p->hasGearX || p->hasGearY || p->hasGearZ || p->hasGearA ||
        p->hasVel || p->hasAccel || p->hasDecel || p->hasAxisMask || p->hasEstopDi6;
    if (needsDisable && g_enabled) {
        return "disable before configure";
    }
    if (!needsDisable && p->hasTestMode) {
        return nullptr;
    }
    if (g_enabled) {
        return "disable before configure";
    }
    if (p->hasStepsX) {
        g_stepsPerRev[0] = p->stepsX;
    }
    if (p->hasStepsY) {
        g_stepsPerRev[1] = p->stepsY;
    }
    if (p->hasStepsZ) {
        g_stepsPerRev[2] = p->stepsZ;
    }
    if (p->hasStepsA) {
        g_stepsPerRev[3] = p->stepsA;
    }
    if (p->hasPitchX && p->pitchX > 0.0) {
        g_pitchMm[0] = p->pitchX;
    }
    if (p->hasPitchY && p->pitchY > 0.0) {
        g_pitchMm[1] = p->pitchY;
    }
    if (p->hasPitchZ && p->pitchZ > 0.0) {
        g_pitchMm[2] = p->pitchZ;
    }
    if (p->hasPitchA && p->pitchA > 0.0) {
        g_pitchMm[3] = p->pitchA;
    }
    if (p->hasGearX && p->gearX > 0.0) {
        g_gear[0] = p->gearX;
    }
    if (p->hasGearY && p->gearY > 0.0) {
        g_gear[1] = p->gearY;
    }
    if (p->hasGearZ && p->gearZ > 0.0) {
        g_gear[2] = p->gearZ;
    }
    if (p->hasGearA && p->gearA > 0.0) {
        g_gear[3] = p->gearA;
    }
    if (p->hasVel) {
        g_vel = p->vel;
    }
    if (p->hasAccel) {
        g_accel = p->accel;
    }
    if (p->hasDecel) {
        g_decel = p->decel;
    }
    if (p->hasAxisMask) {
        uint32_t m = p->axisMask;
        if (m > 15u) {
            m = 15u;
        }
        g_axisMask = m;
    }
    if (p->hasEstopDi6) {
        uint32_t e = p->estopDi6;
        if (e > 2u) {
            e = 1u;
        }
        g_estopDi6 = (uint8_t)e;
    }
    RecalcStepsPerUnit();
    g_xy.SetMechanicalParamsX(g_stepsPerRev[0], g_pitchMm[0], UNIT_MM, g_gear[0]);
    g_xy.SetMechanicalParamsY(g_stepsPerRev[1], g_pitchMm[1], UNIT_MM, g_gear[1]);
    motorZ.SetMechanicalParams(g_stepsPerRev[2], g_pitchMm[2], UNIT_MM, g_gear[2]);
    motorA.SetMechanicalParams(g_stepsPerRev[3], 360.0, UNIT_DEGREES, g_gear[3]);
    ApplyLimits();
    return nullptr;
}

const char *MotionSetUnits(const RpcParams *p) {
    if (!p->hasUnits) {
        return "units required";
    }
    if (strcmp(p->units, "mm") == 0) {
        g_units = CLEARAI_UNITS_MM;
        return nullptr;
    }
    if (strcmp(p->units, "inch") == 0 || strcmp(p->units, "in") == 0) {
        g_units = CLEARAI_UNITS_INCH;
        return nullptr;
    }
    return "units must be mm or inch";
}

const char *MotionSetMode(const RpcParams *p) {
    if (!p->hasMode) {
        return "mode required";
    }
    if (strcmp(p->mode, "abs") == 0) {
        g_mode = CLEARAI_MODE_ABS;
        return nullptr;
    }
    if (strcmp(p->mode, "rel") == 0) {
        g_mode = CLEARAI_MODE_REL;
        return nullptr;
    }
    return "mode must be abs or rel";
}

const char *MotionSetWorkOrigin(const RpcParams *p) {
    const bool any = p->hasX || p->hasY || p->hasZ || p->hasA;
    for (uint8_t a = 0; a < CLEARAI_AXIS_COUNT; a++) {
        bool setThis = false;
        double user = 0.0;
        if (a == CLEARAI_AXIS_X && p->hasX) {
            setThis = true;
            user = p->x;
        } else if (a == CLEARAI_AXIS_Y && p->hasY) {
            setThis = true;
            user = p->y;
        } else if (a == CLEARAI_AXIS_Z && p->hasZ) {
            setThis = true;
            user = p->z;
        } else if (a == CLEARAI_AXIS_A && p->hasA) {
            setThis = true;
            user = p->a;
        } else if (!any) {
            setThis = true;
            user = 0.0;
        }
        if (setThis) {
            g_workOrigin[a] = MachineInternal(a) - ToInternal(a, user);
        }
    }
    return nullptr;
}

static int32_t TargetSteps(uint8_t axis, bool hasUser, double user, bool relative) {
    const int32_t cur = MachineSteps(axis);
    if (!hasUser || !AxisOn(axis)) {
        return cur;
    }
    const double internal = ToInternal(axis, user);
    if (relative) {
        return cur + I32Round(internal * g_stepsPerMm[axis]);
    }
    const double machine = internal + g_workOrigin[axis];
    return I32Round(machine * g_stepsPerMm[axis]);
}

static void ApplyFeed(bool rapid, bool hasFeed, double feedUser) {
    double feedMm = g_feedMmPerMin;
    if (hasFeed && feedUser > 0.0) {
        feedMm = (g_units == CLEARAI_UNITS_INCH) ? (feedUser * 25.4) : feedUser;
        g_feedMmPerMin = feedMm;
    }
    if (rapid) {
        g_xy.ArcVelMax(g_vel);
        g_xy.FeedRateMMPerMin((double)g_vel / (g_stepsPerMm[0] > 0.0 ? g_stepsPerMm[0] : 1.0) * 60.0);
        return;
    }
    g_xy.FeedRateMMPerMin(feedMm);
    const double stepsPerSec = (feedMm / 60.0) * g_stepsPerMm[0];
    uint32_t vel = (stepsPerSec < 1.0) ? 1u : (uint32_t)stepsPerSec;
    if (vel > g_vel) {
        vel = g_vel;
    }
    for (uint8_t a = 0; a < CLEARAI_AXIS_COUNT; a++) {
        MotorDriver *m = MotorForAxis(a);
        if (m) {
            m->VelMax(vel);
        }
    }
}

static void MoveZa(int32_t tz, int32_t ta, bool hasZ, bool hasA) {
    if (hasZ && AxisOn(CLEARAI_AXIS_Z)) {
        motorZ.Move(tz, StepGenerator::MOVE_TARGET_ABSOLUTE);
    }
    if (hasA && AxisOn(CLEARAI_AXIS_A)) {
        motorA.Move(ta, StepGenerator::MOVE_TARGET_ABSOLUTE);
    }
}

static const char *MoveAxisAbs(uint8_t axis, int32_t steps) {
    MotorDriver *m = MotorForAxis(axis);
    if (!m) {
        return "axis missing";
    }
    if (g_testMode) {
        m->ClearAlerts();
        if (!m->EnableRequest()) {
            m->EnableRequest(true);
        }
    }
    if (!m->Move(steps, StepGenerator::MOVE_TARGET_ABSOLUTE)) {
        if (!m->EnableRequest()) {
            return "motor not enabled";
        }
        if (m->StatusReg().bit.AlertsPresent || m->StatusReg().bit.MotorInFault) {
            return "motor in alert";
        }
        return "move rejected";
    }
    return nullptr;
}

/* Coordinated XY only when both motors are actually driveable. A missing
 * partner (typical one-motor bench) falls back to independent Move. */
static const char *IssueXyLinear(int32_t tx, int32_t ty, bool hasX, bool hasY) {
    if (AxisHealthy(CLEARAI_AXIS_X) && AxisHealthy(CLEARAI_AXIS_Y)) {
        if (!g_xy.QueueLinear(tx, ty)) {
            return "xy queue rejected";
        }
        return nullptr;
    }
    if (hasX && AxisOn(CLEARAI_AXIS_X)) {
        const char *err = MoveAxisAbs(CLEARAI_AXIS_X, tx);
        if (err) {
            return err;
        }
    }
    if (hasY && AxisOn(CLEARAI_AXIS_Y)) {
        const char *err = MoveAxisAbs(CLEARAI_AXIS_Y, ty);
        if (err) {
            if (g_testMode) {
                return nullptr;
            }
            return err;
        }
    }
    return nullptr;
}

const char *MotionMoveLinear(const RpcParams *p) {
    const char *err = GateMotion();
    if (err) {
        return err;
    }
    const bool rel = (g_mode == CLEARAI_MODE_REL);
    const int32_t tx = TargetSteps(CLEARAI_AXIS_X, p->hasX, p->x, rel);
    const int32_t ty = TargetSteps(CLEARAI_AXIS_Y, p->hasY, p->y, rel);
    const int32_t tz = TargetSteps(CLEARAI_AXIS_Z, p->hasZ, p->z, rel);
    const int32_t ta = TargetSteps(CLEARAI_AXIS_A, p->hasA, p->a, rel);
    const bool rapid = p->hasRapid && p->rapid;
    ApplyFeed(rapid, p->hasFeed, p->feed);

    if (p->hasX || p->hasY) {
        const char *xyErr = IssueXyLinear(tx, ty, p->hasX, p->hasY);
        if (xyErr) {
            return xyErr;
        }
    }
    MoveZa(tz, ta, p->hasZ, p->hasA);
    return nullptr;
}

const char *MotionMoveArc(const RpcParams *p) {
    const char *err = GateMotion();
    if (err) {
        return err;
    }
    if (!AxisOn(CLEARAI_AXIS_X) || !AxisOn(CLEARAI_AXIS_Y)) {
        return "arc requires XY axes";
    }
    if (p->hasZ || p->hasA) {
        return "arc does not accept z or a";
    }
    if (!p->hasI || !p->hasJ) {
        return "arc requires i and j";
    }
    const bool rel = (g_mode == CLEARAI_MODE_REL);
    if (!rel && !p->hasX && !p->hasY) {
        return "arc requires x and y in abs mode";
    }
    const int32_t sx = MachineSteps(CLEARAI_AXIS_X);
    const int32_t sy = MachineSteps(CLEARAI_AXIS_Y);
    const int32_t ex = TargetSteps(CLEARAI_AXIS_X, p->hasX, p->x, rel);
    const int32_t ey = TargetSteps(CLEARAI_AXIS_Y, p->hasY, p->y, rel);
    const int32_t iSteps = I32Round(ToInternal(CLEARAI_AXIS_X, p->i) * g_stepsPerMm[0]);
    const int32_t jSteps = I32Round(ToInternal(CLEARAI_AXIS_Y, p->j) * g_stepsPerMm[1]);
    const int32_t cx = sx + iSteps;
    const int32_t cy = sy + jSteps;
    const double dx = (double)sx - (double)cx;
    const double dy = (double)sy - (double)cy;
    const double radius = sqrt(dx * dx + dy * dy);
    if (radius < 1.0) {
        return "arc radius too small";
    }
    const double startAngle = atan2(dy, dx);
    const double endAngle = atan2((double)ey - (double)cy, (double)ex - (double)cx);
    const bool clockwise = p->hasClockwise ? p->clockwise : false;
    ApplyFeed(false, p->hasFeed, p->feed);
    if (!g_xy.QueueArc(cx, cy, I32Round(radius), startAngle, endAngle, clockwise)) {
        return "arc queue rejected";
    }
    return nullptr;
}

const char *MotionJog(const RpcParams *p) {
    const char *err = GateMotion();
    if (err) {
        return err;
    }
    const int32_t tx = TargetSteps(CLEARAI_AXIS_X, p->hasX, p->x, true);
    const int32_t ty = TargetSteps(CLEARAI_AXIS_Y, p->hasY, p->y, true);
    const int32_t tz = TargetSteps(CLEARAI_AXIS_Z, p->hasZ, p->z, true);
    const int32_t ta = TargetSteps(CLEARAI_AXIS_A, p->hasA, p->a, true);
    ApplyFeed(false, p->hasFeed, p->feed);
    if (p->hasX || p->hasY) {
        const char *xyErr = IssueXyLinear(tx, ty, p->hasX, p->hasY);
        if (xyErr) {
            return xyErr;
        }
    }
    MoveZa(tz, ta, p->hasZ, p->hasA);
    return nullptr;
}

static const char *WaitLoop(uint32_t timeoutMs, bool waitMotionIdle, ClearAiUrgentFn urgent) {
    const uint32_t start = Milliseconds();
    for (;;) {
        MotionPollEstop();
        if (g_interrupted) {
            return "interrupted by estop";
        }
        if (urgent && urgent()) {
            return "interrupted by safety command";
        }
        if (waitMotionIdle) {
            if (MotorsIdle()) {
                return nullptr;
            }
        } else {
            if ((Milliseconds() - start) >= timeoutMs) {
                return nullptr;
            }
        }
        if (waitMotionIdle && timeoutMs > 0 && (Milliseconds() - start) >= timeoutMs) {
            return "wait_idle timeout";
        }
        Delay_ms(1);
    }
}

const char *MotionDwell(const RpcParams *p, ClearAiUrgentFn urgent) {
    if (!p->hasSeconds || p->seconds < 0.0) {
        return "seconds required";
    }
    double ms = p->seconds * 1000.0;
    if (ms > (double)CLEARAI_DWELL_MS_MAX) {
        ms = (double)CLEARAI_DWELL_MS_MAX;
    }
    return WaitLoop((uint32_t)ms, false, urgent);
}

const char *MotionWaitIdle(const RpcParams *p, ClearAiUrgentFn urgent) {
    uint32_t timeout = 60000u;
    if (p->hasTimeoutMs) {
        timeout = p->timeoutMs;
        if (timeout == 0) {
            timeout = 1;
        }
    }
    return WaitLoop(timeout, true, urgent);
}

void MotionGetStatus(MotionStatus *out) {
    memset(out, 0, sizeof(*out));
    out->enabled = g_enabled;
    out->moving = !MotorsIdle();
    out->hlfb = HlfbOk();
    out->estop = !g_testMode && (g_interrupted || HardwareEstopFault());
    out->hwEstop = HardwareEstopFaultRaw();
    out->testMode = g_testMode;
    out->alerts = AnyAlert();
    out->queue = g_xy.MotionQueueCount();
    out->units = g_units;
    out->mode = g_mode;
    out->axisMask = g_axisMask;
    out->vel = g_vel;
    out->accel = g_accel;
    for (uint8_t a = 0; a < CLEARAI_AXIS_COUNT; a++) {
        MotorDriver *m = MotorForAxis(a);
        if (m) {
            out->alertReg |= m->AlertReg().reg;
        }
        out->work[a] = FromInternal(a, WorkInternal(a));
    }
}

void MotionFillCapabilitiesJson(char *buf, uint16_t bufLen) {
    snprintf(buf, bufLen,
             "{\"protocol\":\"clearai\",\"version\":\"%s\",\"axes\":[\"x\",\"y\",\"z\",\"a\"],"
             "\"axis_mask\":%lu,\"units\":\"%s\",\"mode\":\"%s\","
             "\"test_mode\":%s,\"methods\":[\"get_capabilities\",\"get_status\",\"get_pose\",\"enable\","
             "\"disable\",\"clear_alerts\",\"stop\",\"estop\",\"wait_idle\",\"configure\","
             "\"set_test_mode\",\"set_units\",\"set_mode\",\"set_work_origin\",\"move_linear\","
             "\"move_arc\",\"jog\",\"dwell\"],\"tcp\":%u,\"tel\":%u,\"discover\":%u}",
             CLEARAI_PROTOCOL_VERSION,
             (unsigned long)g_axisMask,
             g_units == CLEARAI_UNITS_MM ? "mm" : "inch",
             g_mode == CLEARAI_MODE_ABS ? "abs" : "rel",
             g_testMode ? "true" : "false",
             CLEARAI_TCP_CONTROL_PORT, CLEARAI_TCP_TELEMETRY_PORT,
             CLEARAI_UDP_DISCOVERY_PORT);
}

static void FillPoseObject(char *buf, uint16_t bufLen, bool withFlags) {
    MotionStatus st;
    MotionGetStatus(&st);
    if (withFlags) {
        snprintf(buf, bufLen,
                 "{\"enabled\":%s,\"moving\":%s,\"hlfb\":%s,\"estop\":%s,\"hw_estop\":%s,"
                 "\"test_mode\":%s,\"alerts\":%s,"
                 "\"queue\":%u,\"alert_reg\":%lu,\"units\":\"%s\",\"mode\":\"%s\","
                 "\"axis_mask\":%lu,\"x\":%.4f,\"y\":%.4f,\"z\":%.4f,\"a\":%.4f}",
                 st.enabled ? "true" : "false",
                 st.moving ? "true" : "false",
                 st.hlfb ? "true" : "false",
                 st.estop ? "true" : "false",
                 st.hwEstop ? "true" : "false",
                 st.testMode ? "true" : "false",
                 st.alerts ? "true" : "false",
                 (unsigned)st.queue,
                 (unsigned long)st.alertReg,
                 st.units == CLEARAI_UNITS_MM ? "mm" : "inch",
                 st.mode == CLEARAI_MODE_ABS ? "abs" : "rel",
                 (unsigned long)st.axisMask,
                 st.work[0], st.work[1], st.work[2], st.work[3]);
    } else {
        snprintf(buf, bufLen,
                 "{\"units\":\"%s\",\"mode\":\"%s\",\"x\":%.4f,\"y\":%.4f,\"z\":%.4f,\"a\":%.4f}",
                 st.units == CLEARAI_UNITS_MM ? "mm" : "inch",
                 st.mode == CLEARAI_MODE_ABS ? "abs" : "rel",
                 st.work[0], st.work[1], st.work[2], st.work[3]);
    }
}

void MotionFillStatusJson(char *buf, uint16_t bufLen) {
    FillPoseObject(buf, bufLen, true);
}

void MotionFillPoseJson(char *buf, uint16_t bufLen) {
    FillPoseObject(buf, bufLen, false);
}

void MotionFillTelemetryJson(char *buf, uint16_t bufLen) {
    FillPoseObject(buf, bufLen, true);
}
