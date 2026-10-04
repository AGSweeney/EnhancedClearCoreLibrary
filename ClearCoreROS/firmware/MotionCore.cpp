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
#include "NvmManager.h"
#include "SysTiming.h"
#include "XrceClient.h"

#include <math.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>

static uint32_t g_stepsPerRev[CCROS_AXIS_COUNT];
static double g_pitchMm[CCROS_AXIS_COUNT];
static char g_jointName[CCROS_AXIS_COUNT][16];
static uint8_t g_rotaryMask;
static int8_t g_direction[CCROS_AXIS_COUNT];
static double g_gear[CCROS_AXIS_COUNT];
static double g_offset[CCROS_AXIS_COUNT];
static uint32_t g_axisMask = CCROS_DEFAULT_AXIS_MASK;
static uint32_t g_vel = CCROS_DEFAULT_VEL_STEPS;
static uint32_t g_accel = CCROS_DEFAULT_ACCEL_STEPS;
static uint32_t g_decel = CCROS_DEFAULT_DECEL_STEPS;
static uint32_t g_watchdogMs = CCROS_DEFAULT_WATCHDOG_MS;
static uint8_t g_estopDi6 = CCROS_DEFAULT_ESTOP_DI6;
static bool g_testMode = false;
static bool g_nvmLoaded = false;
static uint8_t g_netMode = 0;
static uint8_t g_ipOctets[4] = {0, 0, 0, 0};
static uint8_t g_netmaskOctets[4] = {0, 0, 0, 0};
static uint8_t g_gatewayOctets[4] = {0, 0, 0, 0};
static uint8_t g_limitFlags = 0;
static double g_limitMin[CCROS_AXIS_COUNT];
static double g_limitMax[CCROS_AXIS_COUNT];
static uint8_t g_posLimDi[CCROS_AXIS_COUNT];
static uint8_t g_negLimDi[CCROS_AXIS_COUNT];
static char g_limitErr[40];
static char g_travelLimit[48];
static CoordinatedMotionController g_xy;
static bool g_xyReady = false;
static bool g_seekActive = false;
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

static bool AxisIsRotary(uint8_t axis) {
    return (g_rotaryMask & (1u << axis)) != 0;
}

static double StepsPerUnit(uint8_t axis) {
    if (g_stepsPerRev[axis] == 0) {
        return 0.0;
    }
    if (AxisIsRotary(axis)) {
        return (double)g_stepsPerRev[axis] / (2.0 * 3.14159265358979323846);
    }
    if (g_pitchMm[axis] <= 0.0) {
        return 0.0;
    }
    return (double)g_stepsPerRev[axis] * 1000.0 / g_pitchMm[axis];
}

/* Joint units are what the host sends and what status reports.
 * motor = direction * (joint - offset) / gear, with direction ±1 and gear > 0. */
static double JointToMotor(uint8_t axis, double joint) {
    return (double)g_direction[axis] * (joint - g_offset[axis]) / g_gear[axis];
}

static double MotorToJoint(uint8_t axis, double motor) {
    return (double)g_direction[axis] * g_gear[axis] * motor + g_offset[axis];
}

static double JointVelToMotor(uint8_t axis, double jointVel) {
    return (double)g_direction[axis] * jointVel / g_gear[axis];
}

static double JointDeltaToMotor(uint8_t axis, double delta) {
    return (double)g_direction[axis] * delta / g_gear[axis];
}

static double MotorVelToJoint(uint8_t axis, double motorVel) {
    return (double)g_direction[axis] * g_gear[axis] * motorVel;
}

const char *MotionJointName(uint8_t axis) {
    if (axis >= CCROS_AXIS_COUNT) {
        return "";
    }
    return g_jointName[axis];
}

bool MotionAxisRotary(uint8_t axis) {
    return axis < CCROS_AXIS_COUNT && AxisIsRotary(axis);
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
        if (AxisIsRotary(a)) {
            m->SetMechanicalParams(g_stepsPerRev[a], 360.0, UNIT_DEGREES, 1.0);
        } else {
            m->SetMechanicalParams(g_stepsPerRev[a], g_pitchMm[a], UNIT_MM, 1.0);
        }
    }
    ApplyDynamics();
    if (g_xyReady) {
        g_xy.SetMechanicalParamsX(g_stepsPerRev[CCROS_AXIS_X], g_pitchMm[CCROS_AXIS_X], UNIT_MM, 1.0);
        g_xy.SetMechanicalParamsY(g_stepsPerRev[CCROS_AXIS_Y], g_pitchMm[CCROS_AXIS_Y], UNIT_MM, 1.0);
        g_xy.ArcVelMax(g_vel);
        g_xy.ArcAccelMax(g_accel);
    }
}

/* User-page blob. Magic differs from ClearAI ('CAIC') so that blob is not applied.
 * Version 1 is mechanics and network. Version 2 appends soft limits and DI pins.
 * Version 3 appends joint name, rotary flag, direction, gear, and offset. */
static const uint32_t CCROS_NVM_MAGIC = 0x534F5243u; /* 'CROS' */
static const uint16_t CCROS_NVM_VERSION_V1 = 1;
static const uint16_t CCROS_NVM_VERSION_V2 = 2;
static const uint16_t CCROS_NVM_VERSION = 3;

#pragma pack(push, 1)
struct CcrosNvmConfigV1 {
    uint32_t magic;
    uint16_t version;
    uint16_t size;
    uint32_t axisMask;
    uint32_t stepsPerRev[CCROS_AXIS_COUNT];
    float pitchMm[CCROS_AXIS_COUNT];
    uint32_t vel;
    uint32_t accel;
    uint32_t decel;
    uint32_t watchdogMs;
    uint8_t estopDi6;
    uint8_t testMode;
    uint8_t netMode;
    uint8_t ipOctets[4];
    uint8_t netmaskOctets[4];
    uint8_t gatewayOctets[4];
};

struct CcrosNvmConfigV2 {
    CcrosNvmConfigV1 v1;
    uint8_t limitFlags;
    uint8_t posLimDi[CCROS_AXIS_COUNT];
    uint8_t negLimDi[CCROS_AXIS_COUNT];
    float limitMin[CCROS_AXIS_COUNT];
    float limitMax[CCROS_AXIS_COUNT];
};

struct CcrosNvmConfig {
    CcrosNvmConfigV2 v2;
    char jointName[CCROS_AXIS_COUNT][16];
    uint8_t rotaryMask;
    int8_t direction[CCROS_AXIS_COUNT];
    float gear[CCROS_AXIS_COUNT];
    float offset[CCROS_AXIS_COUNT];
};
#pragma pack(pop)

static_assert(sizeof(CcrosNvmConfig) <= 416, "ROS NVM blob exceeds the user page");
static_assert(offsetof(CcrosNvmConfigV2, limitFlags) == sizeof(CcrosNvmConfigV1), "v1 prefix");
static_assert(offsetof(CcrosNvmConfig, jointName) == sizeof(CcrosNvmConfigV2), "v2 prefix");

static ClearCore::NvmManager &Nvm() {
    return ClearCore::NvmManager::Instance();
}

static bool OctetsZero(const uint8_t o[4]) {
    return (o[0] | o[1] | o[2] | o[3]) == 0;
}

static void ApplyMapDefaults() {
    static const char *kName[CCROS_AXIS_COUNT] = {"joint_x", "joint_y", "joint_z", "joint_a"};
    g_rotaryMask = (uint8_t)(1u << CCROS_AXIS_A);
    for (uint8_t a = 0; a < CCROS_AXIS_COUNT; a++) {
        memset(g_jointName[a], 0, sizeof(g_jointName[a]));
        strncpy(g_jointName[a], kName[a], sizeof(g_jointName[a]) - 1u);
        g_direction[a] = 1;
        g_gear[a] = 1.0;
        g_offset[a] = 0.0;
    }
}

static void ApplyCompileDefaults() {
    for (uint8_t a = 0; a < CCROS_AXIS_COUNT; a++) {
        g_stepsPerRev[a] = CCROS_DEFAULT_STEPS_PER_REV;
        g_pitchMm[a] = CCROS_DEFAULT_PITCH_MM;
    }
    ApplyMapDefaults();
    g_axisMask = CCROS_DEFAULT_AXIS_MASK;
    g_vel = CCROS_DEFAULT_VEL_STEPS;
    g_accel = CCROS_DEFAULT_ACCEL_STEPS;
    g_decel = CCROS_DEFAULT_DECEL_STEPS;
    g_watchdogMs = CCROS_DEFAULT_WATCHDOG_MS;
    g_estopDi6 = CCROS_DEFAULT_ESTOP_DI6;
    g_testMode = false;
    g_netMode = 0;
    memset(g_ipOctets, 0, sizeof(g_ipOctets));
    memset(g_netmaskOctets, 0, sizeof(g_netmaskOctets));
    memset(g_gatewayOctets, 0, sizeof(g_gatewayOctets));
    g_limitFlags = 0;
    memset(g_limitMin, 0, sizeof(g_limitMin));
    memset(g_limitMax, 0, sizeof(g_limitMax));
    memset(g_posLimDi, 0, sizeof(g_posLimDi));
    memset(g_negLimDi, 0, sizeof(g_negLimDi));
    g_travelLimit[0] = '\0';
}

static bool LimitDiOk(uint8_t di) {
    return di == 0 || di == 255 || di <= 12;
}

static bool ConfigPrefixOk(const CcrosNvmConfigV1 *cfg) {
    if (cfg->axisMask == 0 || cfg->axisMask > 0x0f) {
        return false;
    }
    if (cfg->vel == 0 || cfg->vel > 500000u || cfg->accel == 0 || cfg->accel > 5000000u ||
        cfg->decel == 0 || cfg->decel > 5000000u || cfg->watchdogMs > 60000u) {
        return false;
    }
    if (cfg->estopDi6 > 2 || cfg->testMode > 1 || cfg->netMode > 1) {
        return false;
    }
    for (uint8_t a = 0; a < CCROS_AXIS_COUNT; a++) {
        if (cfg->stepsPerRev[a] == 0 || cfg->stepsPerRev[a] > 1000000u) {
            return false;
        }
        if (a != CCROS_AXIS_A && (cfg->pitchMm[a] < 0.01f || cfg->pitchMm[a] > 1000.f)) {
            return false;
        }
    }
    if (cfg->netMode == 1 && (OctetsZero(cfg->ipOctets) || OctetsZero(cfg->netmaskOctets))) {
        return false;
    }
    return true;
}

static bool LimitsOk(const CcrosNvmConfigV2 *cfg) {
    for (uint8_t a = 0; a < CCROS_AXIS_COUNT; a++) {
        if (!LimitDiOk(cfg->posLimDi[a]) || !LimitDiOk(cfg->negLimDi[a])) {
            return false;
        }
        const bool minEn = (cfg->limitFlags & (1u << (a * 2u))) != 0;
        const bool maxEn = (cfg->limitFlags & (1u << (a * 2u + 1u))) != 0;
        if (minEn && !(cfg->limitMin[a] > -1.0e6f && cfg->limitMin[a] < 1.0e6f)) {
            return false;
        }
        if (maxEn && !(cfg->limitMax[a] > -1.0e6f && cfg->limitMax[a] < 1.0e6f)) {
            return false;
        }
        if (minEn && maxEn && cfg->limitMin[a] > cfg->limitMax[a] + 1.0e-4f) {
            return false;
        }
    }
    return true;
}

static bool NameOk(const char *name) {
    size_t n = 0;
    while (n < 16 && name[n] != '\0') {
        n++;
    }
    if (n == 0 || n >= 16) {
        return false;
    }
    for (size_t i = 0; i < n; i++) {
        const char c = name[i];
        const bool ok = (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') ||
                        (c >= '0' && c <= '9') || c == '_';
        if (!ok) {
            return false;
        }
    }
    return true;
}

static bool ConfigBlobOk(const CcrosNvmConfig *cfg) {
    if (cfg->v2.v1.magic != CCROS_NVM_MAGIC) {
        return false;
    }
    if (cfg->v2.v1.version == CCROS_NVM_VERSION_V1 && cfg->v2.v1.size == sizeof(CcrosNvmConfigV1)) {
        return ConfigPrefixOk(&cfg->v2.v1);
    }
    if (cfg->v2.v1.version == CCROS_NVM_VERSION_V2 && cfg->v2.v1.size == sizeof(CcrosNvmConfigV2)) {
        return ConfigPrefixOk(&cfg->v2.v1) && LimitsOk(&cfg->v2);
    }
    if (cfg->v2.v1.version != CCROS_NVM_VERSION || cfg->v2.v1.size != sizeof(CcrosNvmConfig)) {
        return false;
    }
    if (!ConfigPrefixOk(&cfg->v2.v1) || !LimitsOk(&cfg->v2)) {
        return false;
    }
    if (cfg->rotaryMask > 0x0f) {
        return false;
    }
    for (uint8_t a = 0; a < CCROS_AXIS_COUNT; a++) {
        if (!NameOk(cfg->jointName[a])) {
            return false;
        }
        if (cfg->direction[a] != 1 && cfg->direction[a] != -1) {
            return false;
        }
        if (!(cfg->gear[a] > 1.0e-6f && cfg->gear[a] < 1.0e6f)) {
            return false;
        }
        if (!(cfg->offset[a] > -1.0e6f && cfg->offset[a] < 1.0e6f)) {
            return false;
        }
        for (uint8_t b = (uint8_t)(a + 1u); b < CCROS_AXIS_COUNT; b++) {
            if (strcmp(cfg->jointName[a], cfg->jointName[b]) == 0) {
                return false;
            }
        }
    }
    return true;
}

static void ConfigApplyPrefix(const CcrosNvmConfigV1 *cfg) {
    g_axisMask = cfg->axisMask;
    for (uint8_t a = 0; a < CCROS_AXIS_COUNT; a++) {
        g_stepsPerRev[a] = cfg->stepsPerRev[a];
        g_pitchMm[a] = cfg->pitchMm[a];
    }
    g_vel = cfg->vel;
    g_accel = cfg->accel;
    g_decel = cfg->decel;
    g_watchdogMs = cfg->watchdogMs;
    g_estopDi6 = cfg->estopDi6;
    g_testMode = cfg->testMode != 0;
    g_netMode = cfg->netMode;
    memcpy(g_ipOctets, cfg->ipOctets, 4);
    memcpy(g_netmaskOctets, cfg->netmaskOctets, 4);
    memcpy(g_gatewayOctets, cfg->gatewayOctets, 4);
}

static uint8_t LimitDiNorm(uint8_t di) {
    if (di == 0 || di == 255 || di > 12) {
        return 0;
    }
    return di;
}

static void ConfigApply(const CcrosNvmConfig *cfg) {
    ConfigApplyPrefix(&cfg->v2.v1);
    ApplyMapDefaults();
    g_limitFlags = 0;
    memset(g_limitMin, 0, sizeof(g_limitMin));
    memset(g_limitMax, 0, sizeof(g_limitMax));
    memset(g_posLimDi, 0, sizeof(g_posLimDi));
    memset(g_negLimDi, 0, sizeof(g_negLimDi));
    if (cfg->v2.v1.version < CCROS_NVM_VERSION_V2) {
        return;
    }
    g_limitFlags = cfg->v2.limitFlags;
    for (uint8_t a = 0; a < CCROS_AXIS_COUNT; a++) {
        g_limitMin[a] = cfg->v2.limitMin[a];
        g_limitMax[a] = cfg->v2.limitMax[a];
        g_posLimDi[a] = LimitDiNorm(cfg->v2.posLimDi[a]);
        g_negLimDi[a] = LimitDiNorm(cfg->v2.negLimDi[a]);
    }
    if (cfg->v2.v1.version < CCROS_NVM_VERSION) {
        return;
    }
    g_rotaryMask = cfg->rotaryMask;
    for (uint8_t a = 0; a < CCROS_AXIS_COUNT; a++) {
        memset(g_jointName[a], 0, sizeof(g_jointName[a]));
        strncpy(g_jointName[a], cfg->jointName[a], sizeof(g_jointName[a]) - 1u);
        g_direction[a] = cfg->direction[a];
        g_gear[a] = cfg->gear[a];
        g_offset[a] = cfg->offset[a];
    }
}

static void ConfigFill(CcrosNvmConfig *cfg) {
    memset(cfg, 0, sizeof(*cfg));
    cfg->v2.v1.magic = CCROS_NVM_MAGIC;
    cfg->v2.v1.version = CCROS_NVM_VERSION;
    cfg->v2.v1.size = (uint16_t)sizeof(CcrosNvmConfig);
    cfg->v2.v1.axisMask = g_axisMask;
    for (uint8_t a = 0; a < CCROS_AXIS_COUNT; a++) {
        cfg->v2.v1.stepsPerRev[a] = g_stepsPerRev[a];
        cfg->v2.v1.pitchMm[a] = (float)g_pitchMm[a];
    }
    cfg->v2.v1.vel = g_vel;
    cfg->v2.v1.accel = g_accel;
    cfg->v2.v1.decel = g_decel;
    cfg->v2.v1.watchdogMs = g_watchdogMs;
    cfg->v2.v1.estopDi6 = g_estopDi6;
    cfg->v2.v1.testMode = g_testMode ? 1u : 0u;
    cfg->v2.v1.netMode = g_netMode;
    memcpy(cfg->v2.v1.ipOctets, g_ipOctets, 4);
    memcpy(cfg->v2.v1.netmaskOctets, g_netmaskOctets, 4);
    memcpy(cfg->v2.v1.gatewayOctets, g_gatewayOctets, 4);
    cfg->v2.limitFlags = g_limitFlags;
    for (uint8_t a = 0; a < CCROS_AXIS_COUNT; a++) {
        cfg->v2.limitMin[a] = (float)g_limitMin[a];
        cfg->v2.limitMax[a] = (float)g_limitMax[a];
        cfg->v2.posLimDi[a] = g_posLimDi[a];
        cfg->v2.negLimDi[a] = g_negLimDi[a];
        strncpy(cfg->jointName[a], g_jointName[a], sizeof(cfg->jointName[a]) - 1u);
        cfg->direction[a] = g_direction[a];
        cfg->gear[a] = (float)g_gear[a];
        cfg->offset[a] = (float)g_offset[a];
    }
    cfg->rotaryMask = g_rotaryMask;
}

static bool ConfigWrite(const CcrosNvmConfig *cfg) {
    CcrosNvmConfig existing;
    memset(&existing, 0, sizeof(existing));
    Nvm().BlockRead(ClearCore::NvmManager::NVM_LOC_USER_START, (int)sizeof(existing),
                    (uint8_t *)&existing);
    /* BlockWrite reports "unchanged" and "write failed" the same way. A failed
     * write can leave the RAM cache matching cfg while the page is still dirty,
     * so an unchanged cache is success only when the page write has finished. */
    if (memcmp(&existing, cfg, sizeof(*cfg)) == 0 && Nvm().Synchonized()) {
        return true;
    }
    if (memcmp(&existing, cfg, sizeof(*cfg)) == 0) {
        CcrosNvmConfig nudge;
        memset(&nudge, 0, sizeof(nudge));
        nudge.v2.v1.magic = 1;
        if (!Nvm().BlockWrite(ClearCore::NvmManager::NVM_LOC_USER_START, (int)sizeof(nudge),
                              (const uint8_t *)&nudge)) {
            return false;
        }
    }
    if (!Nvm().BlockWrite(ClearCore::NvmManager::NVM_LOC_USER_START, (int)sizeof(*cfg),
                          (const uint8_t *)cfg)) {
        return false;
    }
    return Nvm().Synchonized();
}

static bool ConfigSave() {
    CcrosNvmConfig cfg;
    ConfigFill(&cfg);
    if (!ConfigWrite(&cfg)) {
        g_nvmLoaded = false;
        return false;
    }
    g_nvmLoaded = true;
    return true;
}

static bool ConfigClear() {
    CcrosNvmConfig cfg;
    memset(&cfg, 0, sizeof(cfg));
    if (!ConfigWrite(&cfg)) {
        return false;
    }
    g_nvmLoaded = false;
    return true;
}

static void ConfigLoad() {
    CcrosNvmConfig cfg;
    memset(&cfg, 0, sizeof(cfg));
    Nvm().BlockRead(ClearCore::NvmManager::NVM_LOC_USER_START, (int)sizeof(cfg), (uint8_t *)&cfg);
    if (!ConfigBlobOk(&cfg)) {
        g_nvmLoaded = false;
        return;
    }
    ConfigApply(&cfg);
    g_nvmLoaded = true;
}

static bool ParseIpOctets(const char *str, uint8_t out[4]) {
    if (!str) {
        return false;
    }
    uint8_t parts = 0;
    uint16_t acc = 0;
    bool any = false;
    for (const char *s = str;; s++) {
        const char c = *s;
        if (c >= '0' && c <= '9') {
            acc = (uint16_t)(acc * 10u + (uint16_t)(c - '0'));
            any = true;
            if (acc > 255) {
                return false;
            }
        } else if (c == '.' || c == '\0') {
            if (!any || parts >= 4) {
                return false;
            }
            out[parts++] = (uint8_t)acc;
            acc = 0;
            any = false;
            if (c == '\0') {
                break;
            }
        } else {
            return false;
        }
    }
    return parts == 4;
}

static bool StepsActive(MotorDriver *m);

static bool PlannerBusy() {
    return g_xyReady && (g_xy.IsActive() || g_xy.MotionQueueCount() != 0);
}

static void StopDecelAll() {
    if (g_xyReady) {
        g_xy.StopDecel();
    }
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

static const char *AxisName(uint8_t axis) {
    switch (axis) {
        case CCROS_AXIS_X: return "x";
        case CCROS_AXIS_Y: return "y";
        case CCROS_AXIS_Z: return "z";
        case CCROS_AXIS_A: return "a";
        default: return "?";
    }
}

static bool LimitMinEn(uint8_t axis) {
    return (g_limitFlags & (1u << (axis * 2u))) != 0;
}

static bool LimitMaxEn(uint8_t axis) {
    return (g_limitFlags & (1u << (axis * 2u + 1u))) != 0;
}

static void HaltAxis(uint8_t axis) {
    if (g_xyReady && (axis == CCROS_AXIS_X || axis == CCROS_AXIS_Y)) {
        g_xy.StopDecel();
    }
    MotorDriver *m = MotorFor(axis);
    if (m) {
        m->MoveStopDecel(g_decel);
    }
    g_goalValid[axis] = false;
    g_absIssued[axis] = false;
    g_absRetried[axis] = false;
    g_axisVelMode[axis] = false;
    g_axisTrack[axis] = false;
    g_trackDirty[axis] = false;
    g_velGoal[axis] = 0.f;
    g_velLatched[axis] = false;
    g_velCmd[axis] = 0;
}

static void NoteTravelLimit(const char *msg) {
    snprintf(g_travelLimit, sizeof(g_travelLimit), "%s", msg);
}

static Connector *LimitConnector(uint8_t pin) {
    if (pin == 0 || pin > 12) {
        return nullptr;
    }
    return SysMgr.ConnectorByIndex((ClearCorePins)pin);
}

static void ApplyHwLimitInputs() {
    for (uint8_t a = 0; a < CCROS_AXIS_COUNT; a++) {
        Connector *pos = LimitConnector(g_posLimDi[a]);
        Connector *neg = LimitConnector(g_negLimDi[a]);
        if (pos) {
            pos->Mode(Connector::INPUT_DIGITAL);
        }
        if (neg) {
            neg->Mode(Connector::INPUT_DIGITAL);
        }
    }
    ConnectorDI6.Mode(Connector::INPUT_DIGITAL);
}

static bool HwLimitOn(uint8_t axis, bool positive) {
    if (g_testMode || !AxisOn(axis)) {
        return false;
    }
    const uint8_t di = positive ? g_posLimDi[axis] : g_negLimDi[axis];
    Connector *input = LimitConnector(di);
    return input && input->State() != 0;
}

static const char *SoftReject(uint8_t axis, double q) {
    const double spu = StepsPerUnit(axis);
    const double eps = (spu > 0.0) ? ((0.5 / spu) * g_gear[axis]) : 1.0e-6;
    if (LimitMinEn(axis) && q < g_limitMin[axis] - eps) {
        snprintf(g_limitErr, sizeof(g_limitErr), "%s below min limit", AxisName(axis));
        return g_limitErr;
    }
    if (LimitMaxEn(axis) && q > g_limitMax[axis] + eps) {
        snprintf(g_limitErr, sizeof(g_limitErr), "%s above max limit", AxisName(axis));
        return g_limitErr;
    }
    return nullptr;
}

static const char *RejectSteps(uint8_t axis, int32_t target) {
    const double spu = StepsPerUnit(axis);
    const double motor = (spu > 0.0) ? ((double)target / spu) : 0.0;
    const char *err = SoftReject(axis, MotorToJoint(axis, motor));
    if (err) {
        return err;
    }
    MotorDriver *m = MotorFor(axis);
    const int32_t cur = m ? m->PositionRefCommanded() : 0;
    if (target > cur && HwLimitOn(axis, true)) {
        snprintf(g_limitErr, sizeof(g_limitErr), "%s pos limit active", AxisName(axis));
        return g_limitErr;
    }
    if (target < cur && HwLimitOn(axis, false)) {
        snprintf(g_limitErr, sizeof(g_limitErr), "%s neg limit active", AxisName(axis));
        return g_limitErr;
    }
    return nullptr;
}

static const char *RejectVelocity(uint8_t axis, float vel) {
    MotorDriver *m = MotorFor(axis);
    const double spu = StepsPerUnit(axis);
    const double motor = (m && spu > 0.0) ? ((double)m->PositionRefCommanded() / spu) : 0.0;
    const double q = MotorToJoint(axis, motor);
    const double eps = (spu > 0.0) ? (0.5 / spu) * g_gear[axis] : 1.0e-6;
    const float jointVel = (float)MotorVelToJoint(axis, vel);
    if (jointVel > 0.f) {
        if (HwLimitOn(axis, g_direction[axis] > 0)) {
            snprintf(g_limitErr, sizeof(g_limitErr), "%s pos limit active", AxisName(axis));
            return g_limitErr;
        }
        if (LimitMaxEn(axis) && q > g_limitMax[axis] + eps) {
            snprintf(g_limitErr, sizeof(g_limitErr), "%s above max limit", AxisName(axis));
            return g_limitErr;
        }
    }
    if (jointVel < 0.f) {
        if (HwLimitOn(axis, g_direction[axis] < 0)) {
            snprintf(g_limitErr, sizeof(g_limitErr), "%s neg limit active", AxisName(axis));
            return g_limitErr;
        }
        if (LimitMinEn(axis) && q < g_limitMin[axis] - eps) {
            snprintf(g_limitErr, sizeof(g_limitErr), "%s below min limit", AxisName(axis));
            return g_limitErr;
        }
    }
    return nullptr;
}

/* Stop this axis when its generated position has crossed a soft limit in the
 * direction of travel, or a hardware switch is active in that direction. */
static bool PollTravel(uint8_t axis) {
    if (g_seekActive) {
        return false;
    }
    MotorDriver *m = MotorFor(axis);
    if (!m) {
        return false;
    }
    const bool moving = StepsActive(m);
    const bool posDir = m->StatusReg().bit.MoveDirection != 0;
    if (moving && ((posDir && HwLimitOn(axis, true)) || (!posDir && HwLimitOn(axis, false)))) {
        HaltAxis(axis);
        snprintf(g_limitErr, sizeof(g_limitErr), "%s %s limit active", AxisName(axis), posDir ? "pos" : "neg");
        NoteTravelLimit(g_limitErr);
        return true;
    }
    const double spu = StepsPerUnit(axis);
    if (spu <= 0.0) {
        return false;
    }
    const double q = MotorToJoint(axis, (double)m->PositionRefCommanded() / spu);
    const double eps = (0.5 / spu) * g_gear[axis];
    const int32_t cur = m->PositionRefCommanded();
    const bool cmdPos = g_velCmd[axis] > 0 || (g_goalValid[axis] && g_goalSteps[axis] > cur) ||
                        (g_axisTrack[axis] && g_trackVel[axis] > 0.f);
    const bool cmdNeg = g_velCmd[axis] < 0 || (g_goalValid[axis] && g_goalSteps[axis] < cur) ||
                        (g_axisTrack[axis] && g_trackVel[axis] < 0.f);
    const bool towardMax = g_direction[axis] > 0 ? ((moving && posDir) || cmdPos)
                                                 : ((moving && !posDir) || cmdNeg);
    const bool towardMin = g_direction[axis] > 0 ? ((moving && !posDir) || cmdNeg)
                                                 : ((moving && posDir) || cmdPos);
    if (LimitMaxEn(axis) && q > g_limitMax[axis] + eps && towardMax) {
        HaltAxis(axis);
        snprintf(g_limitErr, sizeof(g_limitErr), "%s above max limit", AxisName(axis));
        NoteTravelLimit(g_limitErr);
        return true;
    }
    if (LimitMinEn(axis) && q < g_limitMin[axis] - eps && towardMin) {
        HaltAxis(axis);
        snprintf(g_limitErr, sizeof(g_limitErr), "%s below min limit", AxisName(axis));
        NoteTravelLimit(g_limitErr);
        return true;
    }
    return false;
}

static void AbruptDisable() {
    if (g_xyReady) {
        g_xy.Stop();
    }
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
    if (PlannerBusy()) {
        return true;
    }
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
    const char *blocked = RejectSteps(axis, scheduled);
    if (!blocked && ff > 0 && HwLimitOn(axis, true)) {
        snprintf(g_limitErr, sizeof(g_limitErr), "%s pos limit active", AxisName(axis));
        blocked = g_limitErr;
    }
    if (!blocked && ff < 0 && HwLimitOn(axis, false)) {
        snprintf(g_limitErr, sizeof(g_limitErr), "%s neg limit active", AxisName(axis));
        blocked = g_limitErr;
    }
    if (blocked) {
        HaltAxis(axis);
        NoteTravelLimit(blocked);
        return;
    }
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
    const char *blocked = RejectVelocity(axis, (float)sps / (float)spu);
    if (blocked) {
        HaltAxis(axis);
        NoteTravelLimit(blocked);
        return;
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
            const char *blocked = RejectSteps(axis, g_goalSteps[axis]);
            if (blocked) {
                HaltAxis(axis);
                NoteTravelLimit(blocked);
                return;
            }
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
        const int32_t steps = RoundToI32(JointToMotor(a, q[a]) * spu);
        const char *blocked = RejectSteps(a, steps);
        if (blocked) {
            if (fromSession) {
                return blocked;
            }
            HaltAxis(a);
            NoteTravelLimit(blocked);
            continue;
        }
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
    ApplyCompileDefaults();
    for (uint8_t a = 0; a < CCROS_AXIS_COUNT; a++) {
        g_goalValid[a] = false;
        g_velRos[a] = 0.f;
    }
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
    ConfigLoad();
    ApplyHwLimitInputs();
    if (!g_xy.Initialize(&ConnectorM0, &ConnectorM1)) {
        return false;
    }
    g_xy.StopAtQueueEnd(true);
    g_xyReady = true;
    ApplyMechanics();
    for (uint8_t a = 0; a < CCROS_AXIS_COUNT; a++) {
        MotorDriver *m = MotorFor(a);
        if (m) {
            m->PositionRefSet(0);
        }
    }
    g_xy.SetPosition(0, 0);
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
        if (PollTravel(a)) {
            continue;
        }
        if (PlannerBusy() && (a == CCROS_AXIS_X || a == CCROS_AXIS_Y)) {
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

static const char *ApplyLimitPatch(const MotionConfigPatch *patch) {
    bool any = patch->hasClearLimits && patch->clearLimits;
    for (uint8_t a = 0; a < CCROS_AXIS_COUNT; a++) {
        any = any || patch->hasLimitMin[a] || patch->hasLimitMax[a] || patch->hasClearMin[a] ||
              patch->hasClearMax[a] || patch->hasPosLim[a] || patch->hasNegLim[a];
    }
    if (!any) {
        return nullptr;
    }
    uint8_t flags = g_limitFlags;
    double mn[CCROS_AXIS_COUNT];
    double mx[CCROS_AXIS_COUNT];
    uint8_t pos[CCROS_AXIS_COUNT];
    uint8_t neg[CCROS_AXIS_COUNT];
    memcpy(mn, g_limitMin, sizeof(mn));
    memcpy(mx, g_limitMax, sizeof(mx));
    memcpy(pos, g_posLimDi, sizeof(pos));
    memcpy(neg, g_negLimDi, sizeof(neg));
    if (patch->hasClearLimits && patch->clearLimits) {
        flags = 0;
        memset(pos, 0, sizeof(pos));
        memset(neg, 0, sizeof(neg));
    }
    for (uint8_t a = 0; a < CCROS_AXIS_COUNT; a++) {
        if (patch->hasClearMin[a]) {
            flags = (uint8_t)(flags & ~(1u << (a * 2u)));
        }
        if (patch->hasClearMax[a]) {
            flags = (uint8_t)(flags & ~(1u << (a * 2u + 1u)));
        }
        if (patch->hasLimitMin[a]) {
            if (!(patch->limitMin[a] > -1.0e6 && patch->limitMin[a] < 1.0e6)) {
                return "soft limit out of range";
            }
            mn[a] = patch->limitMin[a];
            flags = (uint8_t)(flags | (1u << (a * 2u)));
        }
        if (patch->hasLimitMax[a]) {
            if (!(patch->limitMax[a] > -1.0e6 && patch->limitMax[a] < 1.0e6)) {
                return "soft limit out of range";
            }
            mx[a] = patch->limitMax[a];
            flags = (uint8_t)(flags | (1u << (a * 2u + 1u)));
        }
        if (patch->hasPosLim[a]) {
            if (!LimitDiOk(patch->posLim[a])) {
                return "invalid limit di";
            }
            pos[a] = LimitDiNorm(patch->posLim[a]);
        }
        if (patch->hasNegLim[a]) {
            if (!LimitDiOk(patch->negLim[a])) {
                return "invalid limit di";
            }
            neg[a] = LimitDiNorm(patch->negLim[a]);
        }
        const bool minEn = (flags & (1u << (a * 2u))) != 0;
        const bool maxEn = (flags & (1u << (a * 2u + 1u))) != 0;
        if (minEn && maxEn && mn[a] > mx[a] + 1.0e-9) {
            snprintf(g_limitErr, sizeof(g_limitErr), "%s min above max", AxisName(a));
            return g_limitErr;
        }
    }
    g_limitFlags = flags;
    memcpy(g_limitMin, mn, sizeof(mn));
    memcpy(g_limitMax, mx, sizeof(mx));
    memcpy(g_posLimDi, pos, sizeof(pos));
    memcpy(g_negLimDi, neg, sizeof(neg));
    ApplyHwLimitInputs();
    return nullptr;
}

const char *MotionConfigure(const MotionConfigPatch *patch) {
    if (!patch) {
        return "missing params";
    }
    bool mapScale = false;
    for (uint8_t a = 0; a < CCROS_AXIS_COUNT; a++) {
        if (patch->hasRotary[a] || patch->hasDirection[a] || patch->hasGear[a] || patch->hasOffset[a]) {
            mapScale = true;
        }
    }
    const bool mechanics = patch->hasAxisMask || patch->hasSteps || patch->hasPitch || mapScale;
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
    for (uint8_t a = 0; a < CCROS_AXIS_COUNT; a++) {
        if (patch->hasName[a]) {
            if (!NameOk(patch->name[a])) {
                return "joint name must be 1..15 letters, digits, or _";
            }
            memset(g_jointName[a], 0, sizeof(g_jointName[a]));
            strncpy(g_jointName[a], patch->name[a], sizeof(g_jointName[a]) - 1u);
        }
        if (patch->hasRotary[a]) {
            if (patch->rotary[a]) {
                g_rotaryMask = (uint8_t)(g_rotaryMask | (1u << a));
            } else {
                g_rotaryMask = (uint8_t)(g_rotaryMask & ~(1u << a));
            }
        }
        if (patch->hasDirection[a]) {
            if (patch->direction[a] != 1 && patch->direction[a] != -1) {
                return "direction must be -1 or 1";
            }
            g_direction[a] = patch->direction[a];
        }
        if (patch->hasGear[a]) {
            if (!(patch->gear[a] > 1.0e-6 && patch->gear[a] < 1.0e6)) {
                return "gear out of range";
            }
            g_gear[a] = patch->gear[a];
        }
        if (patch->hasOffset[a]) {
            if (!(patch->offset[a] > -1.0e6 && patch->offset[a] < 1.0e6)) {
                return "offset out of range";
            }
            g_offset[a] = patch->offset[a];
        }
    }
    for (uint8_t a = 0; a < CCROS_AXIS_COUNT; a++) {
        for (uint8_t b = (uint8_t)(a + 1u); b < CCROS_AXIS_COUNT; b++) {
            if (strcmp(g_jointName[a], g_jointName[b]) == 0) {
                return "joint names must be unique";
            }
        }
    }
    if (mechanics) {
        ApplyMechanics();
    } else if (patch->hasVel || patch->hasAccel || patch->hasDecel) {
        ApplyDynamics();
    }
    {
        const char *limitErr = ApplyLimitPatch(patch);
        if (limitErr) {
            return limitErr;
        }
    }
    if (!ConfigSave()) {
        return "nvm write failed";
    }
    return nullptr;
}

const char *MotionResetConfig() {
    if (g_enabled) {
        return "disable before reset_config";
    }
    if (!ConfigClear()) {
        return "nvm write failed";
    }
    ApplyCompileDefaults();
    ApplyMechanics();
    return nullptr;
}

const char *MotionConfigureNetwork(const char *mode, bool hasMode, const char *ipAddress, bool hasIp,
                                   const char *netmask, bool hasNetmask, const char *gateway,
                                   bool hasGateway, char *buf, uint16_t bufLen) {
    uint8_t netMode = g_netMode;
    uint8_t ip[4];
    uint8_t nm[4];
    uint8_t gw[4];
    memcpy(ip, g_ipOctets, 4);
    memcpy(nm, g_netmaskOctets, 4);
    memcpy(gw, g_gatewayOctets, 4);
    if (hasMode) {
        if (mode && strcmp(mode, "dhcp") == 0) {
            netMode = 0;
        } else if (mode && strcmp(mode, "static") == 0) {
            netMode = 1;
        } else {
            return "mode must be dhcp or static";
        }
    }
    if (hasIp && !ParseIpOctets(ipAddress, ip)) {
        return "invalid ip_address";
    }
    if (hasNetmask && !ParseIpOctets(netmask, nm)) {
        return "invalid netmask";
    }
    if (hasGateway && !ParseIpOctets(gateway, gw)) {
        return "invalid gateway";
    }
    if (netMode == 1 && OctetsZero(ip)) {
        return "static mode requires ip_address";
    }
    if (netMode == 1 && OctetsZero(nm)) {
        return "static mode requires netmask";
    }
    g_netMode = netMode;
    memcpy(g_ipOctets, ip, 4);
    memcpy(g_netmaskOctets, nm, 4);
    memcpy(g_gatewayOctets, gw, 4);
    if (!ConfigSave()) {
        return "nvm write failed";
    }
    snprintf(buf, bufLen,
             "{\"network_mode\":\"%s\",\"ip_address\":\"%u.%u.%u.%u\","
             "\"netmask\":\"%u.%u.%u.%u\",\"gateway\":\"%u.%u.%u.%u\","
             "\"applies_on\":\"restart\"}",
             g_netMode == 1 ? "static" : "dhcp",
             (unsigned)g_ipOctets[0], (unsigned)g_ipOctets[1], (unsigned)g_ipOctets[2],
             (unsigned)g_ipOctets[3], (unsigned)g_netmaskOctets[0], (unsigned)g_netmaskOctets[1],
             (unsigned)g_netmaskOctets[2], (unsigned)g_netmaskOctets[3],
             (unsigned)g_gatewayOctets[0], (unsigned)g_gatewayOctets[1],
             (unsigned)g_gatewayOctets[2], (unsigned)g_gatewayOctets[3]);
    return nullptr;
}

void MotionRestart() {
    for (int i = 0; i < 10; i++) {
        EthernetMgr.Refresh();
        Delay_ms(5);
    }
    SysMgr.ResetBoard();
}

void MotionGetNetworkConfig(uint8_t *mode, uint8_t ip[4], uint8_t netmask[4], uint8_t gateway[4]) {
    if (mode) {
        *mode = g_netMode;
    }
    if (ip) {
        memcpy(ip, g_ipOctets, 4);
    }
    if (netmask) {
        memcpy(netmask, g_netmaskOctets, 4);
    }
    if (gateway) {
        memcpy(gateway, g_gatewayOctets, 4);
    }
}

const char *MotionSetTestMode(bool on) {
    g_testMode = on;
    if (!on && HardwareEstop()) {
        AbruptDisable();
        g_interrupted = true;
    }
    if (!ConfigSave()) {
        return "nvm write failed";
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
    g_travelLimit[0] = '\0';
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

static const char *GateMotion() {
    if (g_interrupted) {
        return "estop active";
    }
    if (g_watchdogTripped) {
        return "watchdog tripped; call clear_alerts";
    }
    if (!g_enabled) {
        return "motor not enabled";
    }
    return nullptr;
}

static void ReleaseAxis(uint8_t axis) {
    g_goalValid[axis] = false;
    g_absIssued[axis] = false;
    g_absRetried[axis] = false;
    g_axisVelMode[axis] = false;
    g_axisTrack[axis] = false;
    g_trackDirty[axis] = false;
    g_velGoal[axis] = 0.f;
    g_velLatched[axis] = false;
    g_velCmd[axis] = 0;
}

static bool AxisFromName(const char *name, uint8_t *axis) {
    if (!name || name[0] == '\0' || name[1] != '\0') {
        return false;
    }
    if (name[0] == 'x') *axis = CCROS_AXIS_X;
    else if (name[0] == 'y') *axis = CCROS_AXIS_Y;
    else if (name[0] == 'z') *axis = CCROS_AXIS_Z;
    else if (name[0] == 'a') *axis = CCROS_AXIS_A;
    else return false;
    return true;
}

/* The XY planner is a millimetre path. A rotary axis, or unequal gears, is not one path speed. */
static bool XyCoordinated() {
    if (!g_xyReady || AxisIsRotary(CCROS_AXIS_X) || AxisIsRotary(CCROS_AXIS_Y)) {
        return false;
    }
    const double diff = g_gear[CCROS_AXIS_X] - g_gear[CCROS_AXIS_Y];
    return diff > -1.0e-4 && diff < 1.0e-4;
}

static void ApplyFeed(uint8_t mask, bool hasFeed, double feedMps) {
    if (g_xyReady) {
        g_xy.ArcAccelMax(g_accel);
    }
    if (!hasFeed || !(feedMps > 0.0)) {
        if (g_xyReady) {
            g_xy.ArcVelMax(g_vel);
        }
        return;
    }
    for (uint8_t a = 0; a < CCROS_AXIS_COUNT; a++) {
        if ((mask & (1u << a)) == 0) {
            continue;
        }
        MotorDriver *m = MotorFor(a);
        const double spu = StepsPerUnit(a);
        if (!m || spu <= 0.0) {
            continue;
        }
        double sps = (feedMps / g_gear[a]) * spu;
        if (sps > (double)g_vel) {
            sps = (double)g_vel;
        }
        if (sps < 1.0) {
            sps = 1.0;
        }
        m->VelMax((uint32_t)sps);
    }
    if (XyCoordinated() && (mask & 0x3u) != 0) {
        const double motorMps = feedMps / g_gear[CCROS_AXIS_X];
        g_xy.FeedRateMMPerMin(motorMps * 60000.0);
        const double spu = StepsPerUnit(CCROS_AXIS_X);
        double sps = (spu > 0.0) ? motorMps * spu : (double)g_vel;
        if (sps > (double)g_vel) {
            sps = (double)g_vel;
        }
        if (sps < 1.0) {
            sps = 1.0;
        }
        g_xy.ArcVelMax((uint32_t)sps);
    }
}

static const char *MoveAxisAbs(uint8_t axis, int32_t steps) {
    MotorDriver *m = MotorFor(axis);
    if (!m) {
        return "axis missing";
    }
    ReleaseAxis(axis);
    if (!m->Move(steps, StepGenerator::MOVE_TARGET_ABSOLUTE)) {
        return "move rejected";
    }
    return nullptr;
}

static const char *IssueXyOrIndependent(int32_t tx, int32_t ty, bool hasX, bool hasY) {
    if (hasX && hasY && AxisOn(CCROS_AXIS_X) && AxisOn(CCROS_AXIS_Y) && XyCoordinated()) {
        ReleaseAxis(CCROS_AXIS_X);
        ReleaseAxis(CCROS_AXIS_Y);
        g_xy.SetPosition(ConnectorM0.PositionRefCommanded(), ConnectorM1.PositionRefCommanded());
        if (!g_xy.QueueLinear(tx, ty)) {
            return "xy queue rejected";
        }
        return nullptr;
    }
    if (hasX && AxisOn(CCROS_AXIS_X)) {
        const char *err = MoveAxisAbs(CCROS_AXIS_X, tx);
        if (err) {
            return err;
        }
    }
    if (hasY && AxisOn(CCROS_AXIS_Y)) {
        const char *err = MoveAxisAbs(CCROS_AXIS_Y, ty);
        if (err) {
            return err;
        }
    }
    return nullptr;
}

static const char *WaitIdleMs(uint32_t timeoutMs) {
    const uint32_t start = Milliseconds();
    uint32_t idleSince = 0;
    for (;;) {
        /* The host is blocked inside this call, so the silence timer stays armed. */
        g_lastHostMs = Milliseconds();
        MotionPoll();
        if (g_interrupted) {
            return "estop active";
        }
        if (!AnyMoving()) {
            if (idleSince == 0) {
                idleSince = Milliseconds();
            }
            if ((Milliseconds() - idleSince) >= 20u) {
                return nullptr;
            }
        } else {
            idleSince = 0;
        }
        if (timeoutMs != 0 && (Milliseconds() - start) >= timeoutMs) {
            return "wait_idle timeout";
        }
        Delay_ms(1);
    }
}

const char *MotionMoveLinear(uint8_t mask, const float q[4], bool hasFeed, double feedMps, char *buf, uint16_t len) {
    const char *err = GateMotion();
    if (err) {
        return err;
    }
    const uint8_t use = (uint8_t)(mask & (uint8_t)g_axisMask);
    if (use == 0) {
        return "no joints in axis_mask";
    }
    int32_t target[CCROS_AXIS_COUNT];
    int32_t start[CCROS_AXIS_COUNT];
    for (uint8_t a = 0; a < CCROS_AXIS_COUNT; a++) {
        MotorDriver *m = MotorFor(a);
        start[a] = m ? m->PositionRefCommanded() : 0;
        target[a] = start[a];
        if ((use & (1u << a)) == 0) {
            continue;
        }
        const double spu = StepsPerUnit(a);
        if (spu <= 0.0) {
            return "axis is not configured";
        }
        target[a] = RoundToI32(JointToMotor(a, q[a]) * spu);
        err = RejectSteps(a, target[a]);
        if (err) {
            return err;
        }
    }
    ApplyFeed(use, hasFeed, feedMps);
    err = IssueXyOrIndependent(target[CCROS_AXIS_X], target[CCROS_AXIS_Y],
                              (use & 0x1u) != 0, (use & 0x2u) != 0);
    if (err) {
        return err;
    }
    if ((use & 0x4u) != 0) {
        err = MoveAxisAbs(CCROS_AXIS_Z, target[CCROS_AXIS_Z]);
        if (err) {
            return err;
        }
    }
    if ((use & 0x8u) != 0) {
        err = MoveAxisAbs(CCROS_AXIS_A, target[CCROS_AXIS_A]);
        if (err) {
            return err;
        }
    }
    double seconds = 0.0;
    const double speed = (hasFeed && feedMps > 0.0) ? feedMps : 0.0;
    for (uint8_t a = 0; a < CCROS_AXIS_COUNT; a++) {
        if ((use & (1u << a)) == 0) {
            continue;
        }
        const double spu = StepsPerUnit(a);
        if (spu <= 0.0) {
            continue;
        }
        const double dist = fabs((double)(target[a] - start[a]) / spu);
        const double sps = (speed > 0.0) ? (speed / g_gear[a]) : ((double)g_vel / spu);
        if (sps > 0.0 && dist / sps > seconds) {
            seconds = dist / sps;
        }
    }
    snprintf(buf, len, "{\"ok\":true,\"coordinated\":%s,\"est_ms\":%lu}",
             ((use & 0x3u) == 0x3u && XyCoordinated()) ? "true" : "false",
             (unsigned long)(seconds * 1000.0));
    return nullptr;
}

const char *MotionMoveArc(bool hasX, float x, bool hasY, float y, float iOff, float jOff, bool clockwise,
                          bool hasFeed, double feedMps, char *buf, uint16_t len) {
    const char *err = GateMotion();
    if (err) {
        return err;
    }
    if (!AxisOn(CCROS_AXIS_X) || !AxisOn(CCROS_AXIS_Y) || !XyCoordinated()) {
        return "arc requires linear x and y with the same gear";
    }
    if (!hasX && !hasY) {
        return "arc requires x or y";
    }
    const double spuX = StepsPerUnit(CCROS_AXIS_X);
    const double spuY = StepsPerUnit(CCROS_AXIS_Y);
    if (spuX <= 0.0 || spuY <= 0.0) {
        return "axis is not configured";
    }
    const int32_t sx = ConnectorM0.PositionRefCommanded();
    const int32_t sy = ConnectorM1.PositionRefCommanded();
    const int32_t ex = hasX ? RoundToI32(JointToMotor(CCROS_AXIS_X, x) * spuX) : sx;
    const int32_t ey = hasY ? RoundToI32(JointToMotor(CCROS_AXIS_Y, y) * spuY) : sy;
    err = RejectSteps(CCROS_AXIS_X, ex);
    if (err) {
        return err;
    }
    err = RejectSteps(CCROS_AXIS_Y, ey);
    if (err) {
        return err;
    }
    const int32_t cx = sx + RoundToI32(JointDeltaToMotor(CCROS_AXIS_X, iOff) * spuX);
    const int32_t cy = sy + RoundToI32(JointDeltaToMotor(CCROS_AXIS_Y, jOff) * spuY);
    const double dx = (double)sx - (double)cx;
    const double dy = (double)sy - (double)cy;
    const double radius = sqrt(dx * dx + dy * dy);
    if (radius < 1.0) {
        return "arc radius too small";
    }
    const double startAngle = atan2(dy, dx);
    const double endAngle = atan2((double)ey - (double)cy, (double)ex - (double)cx);
    ApplyFeed(0x3u, hasFeed, feedMps);
    ReleaseAxis(CCROS_AXIS_X);
    ReleaseAxis(CCROS_AXIS_Y);
    g_xy.SetPosition(sx, sy);
    if (!g_xy.QueueArc(cx, cy, RoundToI32(radius), startAngle, endAngle, clockwise)) {
        return "arc queue rejected";
    }
    double swept = endAngle - startAngle;
    const double twoPi = 2.0 * 3.14159265358979323846;
    if (swept < 0.0) {
        swept += twoPi;
    }
    swept = clockwise ? (twoPi - swept) : swept;
    if (swept < 1.0e-6) {
        swept = twoPi;
    }
    const double meters = (radius / spuX) * swept;
    const double speed = (hasFeed && feedMps > 0.0) ? feedMps : ((double)g_vel / spuX);
    const unsigned long est = (speed > 0.0) ? (unsigned long)(meters / speed * 1000.0) : 0ul;
    snprintf(buf, len, "{\"ok\":true,\"coordinated\":true,\"est_ms\":%lu}", est);
    return nullptr;
}

const char *MotionWaitIdle(uint32_t timeoutMs, char *buf, uint16_t len) {
    const uint32_t start = Milliseconds();
    const char *err = WaitIdleMs(timeoutMs == 0 ? 60000u : timeoutMs);
    if (err) {
        return err;
    }
    snprintf(buf, len, "{\"ok\":true,\"elapsed_ms\":%lu}",
             (unsigned long)(Milliseconds() - start));
    return nullptr;
}

static bool SwitchIsHigh(uint8_t pin) {
    Connector *input = LimitConnector(pin);
    return input && input->State() != 0;
}

static const char *SeekAxis(uint8_t axis, bool positive, int32_t seekSteps) {
    if (seekSteps <= 0) {
        return "seek invalid";
    }
    MotorDriver *m = MotorFor(axis);
    if (!m) {
        return "axis missing";
    }
    const int32_t cur = m->PositionRefCommanded();
    const int32_t far = cur + (positive ? seekSteps : -seekSteps);
    if ((axis == CCROS_AXIS_X || axis == CCROS_AXIS_Y) && AxisOn(CCROS_AXIS_X) && AxisOn(CCROS_AXIS_Y)) {
        const int32_t tx = (axis == CCROS_AXIS_X) ? far : ConnectorM0.PositionRefCommanded();
        const int32_t ty = (axis == CCROS_AXIS_Y) ? far : ConnectorM1.PositionRefCommanded();
        return IssueXyOrIndependent(tx, ty, true, true);
    }
    return MoveAxisAbs(axis, far);
}

static int SeekUntil(uint8_t axis, bool positive, bool useLimit, uint8_t pin, bool activeHigh, uint32_t timeoutMs) {
    const uint32_t start = Milliseconds();
    for (;;) {
        g_lastHostMs = Milliseconds();
        MotionPoll();
        if (g_interrupted) {
            return -1;
        }
        bool tripped = false;
        if (useLimit) {
            tripped = SwitchIsHigh(positive ? g_posLimDi[axis] : g_negLimDi[axis]);
        } else {
            const bool high = SwitchIsHigh(pin);
            tripped = activeHigh ? high : !high;
        }
        if (tripped) {
            StopDecelAll();
            const uint32_t stopStart = Milliseconds();
            while (AnyMoving() && (Milliseconds() - stopStart) < 5000u) {
                MotionPoll();
                Delay_ms(1);
            }
            return 0;
        }
        if (!AnyMoving()) {
            if ((Milliseconds() - start) < 30u) {
                Delay_ms(1);
                continue;
            }
            return 1;
        }
        if (timeoutMs != 0 && (Milliseconds() - start) >= timeoutMs) {
            StopDecelAll();
            return -2;
        }
        Delay_ms(1);
    }
}

static void ZeroAxis(uint8_t axis) {
    MotorDriver *m = MotorFor(axis);
    if (m) {
        m->PositionRefSet(0);
    }
    if (g_xyReady) {
        g_xy.SetPosition(ConnectorM0.PositionRefCommanded(), ConnectorM1.PositionRefCommanded());
    }
}

static const char *RunSeek(uint8_t axis, bool positive, bool useLimit, uint8_t pin, bool activeHigh,
                           bool hasSeek, double seek, bool hasBackoff, double backoff,
                           bool hasTimeout, uint32_t timeoutMs, bool zero, char *buf, uint16_t len,
                           bool homing) {
    const double spu = StepsPerUnit(axis);
    if (spu <= 0.0) {
        return "axis is not configured";
    }
    const double seekUnits = hasSeek ? seek : 1.0;
    if (!(seekUnits > 0.0)) {
        return "seek invalid";
    }
    const int32_t seekSteps = RoundToI32((seekUnits / g_gear[axis]) * spu);
    const bool motorPos = g_direction[axis] > 0 ? positive : !positive;
    ApplyFeed(1u << axis, false, 0.0);
    g_seekActive = true;
    const char *err = SeekAxis(axis, motorPos, seekSteps);
    if (err) {
        g_seekActive = false;
        return err;
    }
    const uint32_t timeout = hasTimeout ? timeoutMs : 30000u;
    const int rc = SeekUntil(axis, motorPos, useLimit, pin, activeHigh, timeout);
    if (rc != 0) {
        g_seekActive = false;
        if (rc == -1) {
            return homing ? "estop during home" : "estop during probe";
        }
        if (rc == -2) {
            return homing ? "home timeout" : "probe timeout";
        }
        return homing ? "limit not reached" : "probe not reached";
    }
    const double backUnits = hasBackoff ? backoff : 0.0;
    if (backUnits > 0.0) {
        const int32_t backSteps = RoundToI32((backUnits / g_gear[axis]) * spu);
        err = SeekAxis(axis, !motorPos, backSteps);
        if (err) {
            g_seekActive = false;
            return err;
        }
        err = WaitIdleMs(5000u);
        if (err) {
            g_seekActive = false;
            return err;
        }
    }
    g_seekActive = false;
    if (zero) {
        ZeroAxis(axis);
    }
    MotorDriver *m = MotorFor(axis);
    const double motor = (m && spu > 0.0) ? ((double)m->PositionRefCommanded() / spu) : 0.0;
    const double pos = MotorToJoint(axis, motor);
    if (homing) {
        const uint8_t lim = positive ? g_posLimDi[axis] : g_negLimDi[axis];
        snprintf(buf, len, "{\"homed\":true,\"axis\":\"%s\",\"dir\":\"%s\",\"pos\":%.6f,\"limit_pin\":%u}",
                 AxisName(axis), positive ? "pos" : "neg", pos, (unsigned)lim);
    } else {
        snprintf(buf, len, "{\"probed\":true,\"axis\":\"%s\",\"dir\":\"%s\",\"pos\":%.6f,\"pin\":%u}",
                 AxisName(axis), positive ? "pos" : "neg", pos, (unsigned)pin);
    }
    return nullptr;
}

const char *MotionHome(const char *axisName, const char *dir, bool hasSeek, double seek, bool hasBackoff,
                       double backoff, bool hasTimeout, uint32_t timeoutMs, bool hasZero, bool zeroOn,
                       char *buf, uint16_t len) {
    const char *err = GateMotion();
    if (err) {
        return err;
    }
    uint8_t axis = 0;
    if (!AxisFromName(axisName, &axis)) {
        return "axis must be x, y, z, or a";
    }
    if (!AxisOn(axis)) {
        return "axis not in axis_mask";
    }
    bool positive = false;
    if (dir && strcmp(dir, "pos") == 0) {
        positive = true;
    } else if (dir && strcmp(dir, "neg") == 0) {
        positive = false;
    } else {
        return "dir must be pos or neg";
    }
    const uint8_t lim = positive ? g_posLimDi[axis] : g_negLimDi[axis];
    if (lim == 0) {
        return "limit not configured for this axis/dir";
    }
    if (SwitchIsHigh(lim)) {
        return "limit already active; back off first";
    }
    const bool zero = hasZero ? zeroOn : true;
    return RunSeek(axis, positive, true, 0, true, hasSeek, seek, hasBackoff, backoff, hasTimeout, timeoutMs,
                   zero, buf, len, true);
}

const char *MotionProbe(const char *axisName, const char *dir, uint8_t pin, bool activeHigh, bool hasSeek,
                        double seek, bool hasBackoff, double backoff, bool hasTimeout, uint32_t timeoutMs,
                        bool hasZero, bool zeroOn, char *buf, uint16_t len) {
    const char *err = GateMotion();
    if (err) {
        return err;
    }
    uint8_t axis = 0;
    if (!AxisFromName(axisName, &axis)) {
        return "axis must be x, y, z, or a";
    }
    if (!AxisOn(axis)) {
        return "axis not in axis_mask";
    }
    bool positive = false;
    if (dir && strcmp(dir, "pos") == 0) {
        positive = true;
    } else if (dir && strcmp(dir, "neg") == 0) {
        positive = false;
    } else {
        return "dir must be pos or neg";
    }
    if (pin == 0 || pin > 12) {
        return "pin must be 1-12";
    }
    for (uint8_t a = 0; a < CCROS_AXIS_COUNT; a++) {
        if (g_posLimDi[a] == pin || g_negLimDi[a] == pin) {
            return "pin reserved for limit";
        }
    }
    Connector *probe = LimitConnector(pin);
    if (!probe) {
        return "pin missing";
    }
    probe->Mode(Connector::INPUT_DIGITAL);
    const bool already = activeHigh ? (probe->State() != 0) : (probe->State() == 0);
    if (already) {
        return "probe already active";
    }
    const bool zero = hasZero ? zeroOn : false;
    return RunSeek(axis, positive, false, pin, activeHigh, hasSeek, seek, hasBackoff, backoff, hasTimeout,
                   timeoutMs, zero, buf, len, false);
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
        g_velGoal[a] = (float)JointVelToMotor(a, v[a]);
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
        g_trackPos[a] = (float)JointToMotor(a, q[a]);
        g_trackVel[a] = (float)JointVelToMotor(a, v[a]);
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
        const double motor = (spu > 0.0) ? ((double)steps / spu) : 0.0;
        out->position[a] = (float)MotorToJoint(a, motor);
        out->velocity[a] = (float)MotorVelToJoint(a, g_velRos[a]);
        float duty = 0.f;
        if (m && AxisOn(a)) {
            duty = m->HlfbPercent();
        }
        if (duty >= -100.f && duty <= 100.f) {
            out->effort[a] = duty / 100.f;
        }
        if (g_axisTrack[a]) {
            out->track_mask = (uint8_t)(out->track_mask | (1u << a));
            out->target_position[a] = (float)MotorToJoint(a, g_trackPos[a]);
            out->target_velocity[a] = (float)MotorVelToJoint(a, g_trackVel[a]);
        } else if (g_goalValid[a] && spu > 0.0) {
            out->target_position[a] = (float)MotorToJoint(a, (double)g_goalSteps[a] / spu);
        } else {
            out->target_position[a] = out->position[a];
        }
        if (spu > 0.0) {
            out->command_velocity[a] = (float)MotorVelToJoint(a, (double)g_velCmd[a] / spu);
        }
        out->target_latch_ms[a] = g_trackLatchMs[a];
    }
    out->alert_reg = alerts;
    if (fault) {
        out->flags |= CCROS_FLAG_FAULT;
    }
}

void MotionFillCapabilitiesJson(char *buf, uint16_t len) {
    const char *types[CCROS_AXIS_COUNT];
    const char *units[CCROS_AXIS_COUNT];
    for (uint8_t a = 0; a < CCROS_AXIS_COUNT; a++) {
        types[a] = AxisIsRotary(a) ? "revolute" : "prismatic";
        units[a] = AxisIsRotary(a) ? "rad" : "m";
    }
    snprintf(buf, len,
             "{\"protocol\":\"%s\",\"firmware\":\"%s\",\"session_port\":%u,"
             "\"stream_port\":%u,\"discover_port\":%u,\"stream_hz\":%u,"
             "\"joints\":[\"%s\",\"%s\",\"%s\",\"%s\"],"
             "\"joint_types\":[\"%s\",\"%s\",\"%s\",\"%s\"],"
             "\"units\":[\"%s\",\"%s\",\"%s\",\"%s\"],\"axis_mask\":%lu,\"nvm\":%s}",
             CCROS_PROTOCOL_VERSION, CCROS_FIRMWARE_NAME,
             (unsigned)CCROS_TCP_SESSION_PORT, (unsigned)CCROS_TCP_STREAM_PORT,
             (unsigned)CCROS_UDP_DISCOVERY_PORT,
             (unsigned)(1000u / CCROS_STREAM_PERIOD_MS),
             g_jointName[0], g_jointName[1], g_jointName[2], g_jointName[3],
             types[0], types[1], types[2], types[3],
             units[0], units[1], units[2], units[3],
             (unsigned long)g_axisMask, g_nvmLoaded ? "true" : "false");
}

void MotionFillConfigJson(char *buf, uint16_t len) {
    CcrosNvmConfig stored;
    memset(&stored, 0, sizeof(stored));
    Nvm().BlockRead(ClearCore::NvmManager::NVM_LOC_USER_START, (int)sizeof(stored),
                    (uint8_t *)&stored);
    const bool storedOk = ConfigBlobOk(&stored);
    snprintf(buf, len,
             "{\"nvm\":%s,\"nvm_valid\":%s,\"nvm_version\":%u,"
             "\"axis_mask\":%lu,\"steps_per_rev\":[%lu,%lu,%lu,%lu],"
             "\"pitch_mm\":[%.4f,%.4f,%.4f,%.4f],\"vel_steps\":%lu,"
             "\"accel_steps\":%lu,\"decel_steps\":%lu,\"watchdog_ms\":%lu,"
             "\"estop_di6\":%u,\"test_mode\":%s,\"enabled\":%s,"
             "\"network_mode\":\"%s\",\"ip_address\":\"%u.%u.%u.%u\","
             "\"netmask\":\"%u.%u.%u.%u\",\"gateway\":\"%u.%u.%u.%u\","
             "\"limit_flags\":%u,"
             "\"limits_min\":[%.6f,%.6f,%.6f,%.6f],"
             "\"limits_max\":[%.6f,%.6f,%.6f,%.6f],"
             "\"pos_lim_di\":[%u,%u,%u,%u],"
             "\"neg_lim_di\":[%u,%u,%u,%u],"
             "\"names\":[\"%s\",\"%s\",\"%s\",\"%s\"],"
             "\"rotary\":[%u,%u,%u,%u],"
             "\"direction\":[%d,%d,%d,%d],"
             "\"gear\":[%.6f,%.6f,%.6f,%.6f],"
             "\"offset\":[%.6f,%.6f,%.6f,%.6f]}",
             g_nvmLoaded ? "true" : "false", storedOk ? "true" : "false",
             storedOk ? (unsigned)stored.v2.v1.version : 0u,
             (unsigned long)g_axisMask,
             (unsigned long)g_stepsPerRev[0], (unsigned long)g_stepsPerRev[1],
             (unsigned long)g_stepsPerRev[2], (unsigned long)g_stepsPerRev[3],
             g_pitchMm[0], g_pitchMm[1], g_pitchMm[2], g_pitchMm[3],
             (unsigned long)g_vel, (unsigned long)g_accel, (unsigned long)g_decel,
             (unsigned long)g_watchdogMs, (unsigned)g_estopDi6,
             g_testMode ? "true" : "false", g_enabled ? "true" : "false",
             g_netMode == 1 ? "static" : "dhcp",
             (unsigned)g_ipOctets[0], (unsigned)g_ipOctets[1], (unsigned)g_ipOctets[2],
             (unsigned)g_ipOctets[3], (unsigned)g_netmaskOctets[0], (unsigned)g_netmaskOctets[1],
             (unsigned)g_netmaskOctets[2], (unsigned)g_netmaskOctets[3],
             (unsigned)g_gatewayOctets[0], (unsigned)g_gatewayOctets[1],
             (unsigned)g_gatewayOctets[2], (unsigned)g_gatewayOctets[3],
             (unsigned)g_limitFlags,
             g_limitMin[0], g_limitMin[1], g_limitMin[2], g_limitMin[3],
             g_limitMax[0], g_limitMax[1], g_limitMax[2], g_limitMax[3],
             (unsigned)g_posLimDi[0], (unsigned)g_posLimDi[1], (unsigned)g_posLimDi[2],
             (unsigned)g_posLimDi[3],
             (unsigned)g_negLimDi[0], (unsigned)g_negLimDi[1], (unsigned)g_negLimDi[2],
             (unsigned)g_negLimDi[3],
             g_jointName[0], g_jointName[1], g_jointName[2], g_jointName[3],
             (unsigned)AxisIsRotary(0), (unsigned)AxisIsRotary(1),
             (unsigned)AxisIsRotary(2), (unsigned)AxisIsRotary(3),
             (int)g_direction[0], (int)g_direction[1], (int)g_direction[2], (int)g_direction[3],
             g_gear[0], g_gear[1], g_gear[2], g_gear[3],
             g_offset[0], g_offset[1], g_offset[2], g_offset[3]);
}

void MotionFillStatusJson(char *buf, uint16_t len) {
    CcrosState st;
    MotionFillState(&st);
    char alerts[96];
    FormatAlerts(alerts, sizeof(alerts));
    snprintf(buf, len,
             "{\"enabled\":%s,\"moving\":%s,\"estop\":%s,\"fault\":%s,"
             "\"watchdog\":%s,\"test_mode\":%s,\"axis_mask\":%u,\"alert_reg\":%lu,"
             "\"alerts\":\"%s\",\"travel_limit\":\"%s\",\"xrce\":\"%s\",\"xrce_time\":\"%s\",\"last_cmd_seq\":%u,"
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
             g_travelLimit[0] ? g_travelLimit : "none",
             XrceStateName(),
             XrceTimeSynced() ? "synced" : "unsync",
             (unsigned)g_lastCmdSeq,
             st.position[0], st.position[1], st.position[2], st.position[3],
             st.velocity[0], st.velocity[1], st.velocity[2], st.velocity[3],
             st.effort[0], st.effort[1], st.effort[2], st.effort[3]);
}
