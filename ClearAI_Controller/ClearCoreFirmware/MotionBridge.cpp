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
#include "NvmManager.h"
#include "SysTiming.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

using ClearCore::NvmManager;

static NvmManager &Nvm() {
    return NvmManager::Instance();
}

/* User-page blob at NVM_LOC_USER_START (416 bytes available before Teknic reserved). */
static const uint32_t CLEARAI_NVM_MAGIC = 0x43414943u; /* 'CAIC' */
static const uint16_t CLEARAI_NVM_VERSION = 4;
static const uint16_t CLEARAI_NVM_VERSION_V1 = 1;
static const uint16_t CLEARAI_NVM_VERSION_V2 = 2;
static const uint16_t CLEARAI_NVM_VERSION_V3 = 3;

#pragma pack(push, 1)
struct ClearAiNvmConfigV1 {
    uint32_t magic;
    uint16_t version;
    uint16_t size;
    uint32_t axisMask;
    uint8_t testMode;
    uint8_t units;
    uint8_t mode;
    uint8_t estopDi6;
    uint32_t stepsPerRev[CLEARAI_AXIS_COUNT];
    float pitchMm[CLEARAI_AXIS_COUNT];
    float gear[CLEARAI_AXIS_COUNT];
    uint32_t vel;
    uint32_t accel;
    uint32_t decel;
    float feedMmPerMin;
};

struct ClearAiNvmConfigV2 {
    uint32_t magic;
    uint16_t version;
    uint16_t size;
    uint32_t axisMask;
    uint8_t testMode;
    uint8_t units;
    uint8_t mode;
    uint8_t estopDi6;
    uint32_t stepsPerRev[CLEARAI_AXIS_COUNT];
    float pitchMm[CLEARAI_AXIS_COUNT];
    float gear[CLEARAI_AXIS_COUNT];
    uint32_t vel;
    uint32_t accel;
    uint32_t decel;
    float feedMmPerMin;
    uint8_t limitFlags;
    uint8_t limitPad[3];
    float limitMin[CLEARAI_AXIS_COUNT];
    float limitMax[CLEARAI_AXIS_COUNT];
};

struct ClearAiNvmConfigV3 {
    uint32_t magic;
    uint16_t version;
    uint16_t size;
    uint32_t axisMask;
    uint8_t testMode;
    uint8_t units;
    uint8_t mode;
    uint8_t estopDi6;
    uint32_t stepsPerRev[CLEARAI_AXIS_COUNT];
    float pitchMm[CLEARAI_AXIS_COUNT];
    float gear[CLEARAI_AXIS_COUNT];
    uint32_t vel;
    uint32_t accel;
    uint32_t decel;
    float feedMmPerMin;
    uint8_t limitFlags;
    uint8_t limitPad[3];
    float limitMin[CLEARAI_AXIS_COUNT];
    float limitMax[CLEARAI_AXIS_COUNT];
    uint8_t posLimDi[CLEARAI_AXIS_COUNT];
    uint8_t negLimDi[CLEARAI_AXIS_COUNT];
};

struct ClearAiNvmConfig {
    uint32_t magic;
    uint16_t version;
    uint16_t size;
    uint32_t axisMask;
    uint8_t testMode;
    uint8_t units;
    uint8_t mode;
    uint8_t estopDi6;
    uint32_t stepsPerRev[CLEARAI_AXIS_COUNT];
    float pitchMm[CLEARAI_AXIS_COUNT];
    float gear[CLEARAI_AXIS_COUNT];
    uint32_t vel;
    uint32_t accel;
    uint32_t decel;
    float feedMmPerMin;
    uint8_t limitFlags;
    uint8_t limitPad[3];
    float limitMin[CLEARAI_AXIS_COUNT];
    float limitMax[CLEARAI_AXIS_COUNT];
    uint8_t posLimDi[CLEARAI_AXIS_COUNT];
    uint8_t negLimDi[CLEARAI_AXIS_COUNT];
    uint8_t unitsA;
    uint8_t nvmPad[3];
};
#pragma pack(pop)

static bool g_nvmLoaded = false;

#define motorX ConnectorM0
#define motorY ConnectorM1
#define motorZ ConnectorM2
#define motorA ConnectorM3

static CoordinatedMotionController g_xy;
static bool g_enabled = false;
static bool g_interrupted = false;
static ClearAiUnits g_units = CLEARAI_UNITS_MM;
static ClearAiUnitsA g_unitsA = CLEARAI_UNITS_A_DEG;
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
static uint8_t g_limitFlags = 0;
static double g_limitMin[CLEARAI_AXIS_COUNT];
static double g_limitMax[CLEARAI_AXIS_COUNT];
static uint8_t g_posLimDi[CLEARAI_AXIS_COUNT];
static uint8_t g_negLimDi[CLEARAI_AXIS_COUNT];
static char g_limitErrBuf[32];

static void ApplyLimits();

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

static bool LimitMinEn(uint8_t axis) {
    return (g_limitFlags & (1u << (axis * 2u))) != 0;
}

static bool LimitMaxEn(uint8_t axis) {
    return (g_limitFlags & (1u << (axis * 2u + 1u))) != 0;
}

static void LimitClearAll() {
    g_limitFlags = 0;
}

static void LimitSetMin(uint8_t axis, double internal) {
    g_limitMin[axis] = internal;
    g_limitFlags |= (uint8_t)(1u << (axis * 2u));
}

static void LimitSetMax(uint8_t axis, double internal) {
    g_limitMax[axis] = internal;
    g_limitFlags |= (uint8_t)(1u << (axis * 2u + 1u));
}

static void LimitClearMin(uint8_t axis) {
    g_limitFlags &= (uint8_t)~(1u << (axis * 2u));
}

static void LimitClearMax(uint8_t axis) {
    g_limitFlags &= (uint8_t)~(1u << (axis * 2u + 1u));
}

static const char *AxisName(uint8_t axis) {
    switch (axis) {
        case CLEARAI_AXIS_X: return "x";
        case CLEARAI_AXIS_Y: return "y";
        case CLEARAI_AXIS_Z: return "z";
        case CLEARAI_AXIS_A: return "a";
        default: return "?";
    }
}

static bool AxisFromStr(const char *s, uint8_t *out) {
    if (!s) {
        return false;
    }
    if (strcmp(s, "x") == 0) { *out = CLEARAI_AXIS_X; return true; }
    if (strcmp(s, "y") == 0) { *out = CLEARAI_AXIS_Y; return true; }
    if (strcmp(s, "z") == 0) { *out = CLEARAI_AXIS_Z; return true; }
    if (strcmp(s, "a") == 0) { *out = CLEARAI_AXIS_A; return true; }
    return false;
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

static void ApplyMechanicalParams() {
    RecalcStepsPerUnit();
    g_xy.SetMechanicalParamsX(g_stepsPerRev[0], g_pitchMm[0], UNIT_MM, g_gear[0]);
    g_xy.SetMechanicalParamsY(g_stepsPerRev[1], g_pitchMm[1], UNIT_MM, g_gear[1]);
    motorZ.SetMechanicalParams(g_stepsPerRev[2], g_pitchMm[2], UNIT_MM, g_gear[2]);
    motorA.SetMechanicalParams(g_stepsPerRev[3], 360.0, UNIT_DEGREES, g_gear[3]);
    ApplyLimits();
}

static void ConfigFillFromLive(ClearAiNvmConfig *cfg) {
    memset(cfg, 0, sizeof(*cfg));
    cfg->magic = CLEARAI_NVM_MAGIC;
    cfg->version = CLEARAI_NVM_VERSION;
    cfg->size = (uint16_t)sizeof(ClearAiNvmConfig);
    cfg->axisMask = g_axisMask;
    cfg->testMode = g_testMode ? 1u : 0u;
    cfg->units = (uint8_t)g_units;
    cfg->mode = (uint8_t)g_mode;
    cfg->estopDi6 = g_estopDi6;
    for (uint8_t a = 0; a < CLEARAI_AXIS_COUNT; a++) {
        cfg->stepsPerRev[a] = g_stepsPerRev[a];
        cfg->pitchMm[a] = (float)g_pitchMm[a];
        cfg->gear[a] = (float)g_gear[a];
    }
    cfg->vel = g_vel;
    cfg->accel = g_accel;
    cfg->decel = g_decel;
    cfg->feedMmPerMin = (float)g_feedMmPerMin;
    cfg->limitFlags = g_limitFlags;
    cfg->limitPad[0] = 0;
    cfg->limitPad[1] = 0;
    cfg->limitPad[2] = 0;
    for (uint8_t a = 0; a < CLEARAI_AXIS_COUNT; a++) {
        cfg->limitMin[a] = (float)g_limitMin[a];
        cfg->limitMax[a] = (float)g_limitMax[a];
        cfg->posLimDi[a] = g_posLimDi[a];
        cfg->negLimDi[a] = g_negLimDi[a];
    }
    cfg->unitsA = (uint8_t)g_unitsA;
    cfg->nvmPad[0] = 0;
    cfg->nvmPad[1] = 0;
    cfg->nvmPad[2] = 0;
}

static bool ConfigApplyCommon(const ClearAiNvmConfigV1 *cfg) {
    g_axisMask = cfg->axisMask & 15u;
    g_testMode = (cfg->testMode != 0);
    g_units = (cfg->units == (uint8_t)CLEARAI_UNITS_INCH) ? CLEARAI_UNITS_INCH : CLEARAI_UNITS_MM;
    g_mode = (cfg->mode == (uint8_t)CLEARAI_MODE_REL) ? CLEARAI_MODE_REL : CLEARAI_MODE_ABS;
    g_estopDi6 = (cfg->estopDi6 <= 2u) ? cfg->estopDi6 : 1u;
    for (uint8_t a = 0; a < CLEARAI_AXIS_COUNT; a++) {
        g_stepsPerRev[a] = cfg->stepsPerRev[a] ? cfg->stepsPerRev[a] : CLEARAI_DEFAULT_STEPS_PER_REV;
        g_pitchMm[a] = (cfg->pitchMm[a] > 0.0f) ? (double)cfg->pitchMm[a] : CLEARAI_DEFAULT_PITCH_MM;
        g_gear[a] = (cfg->gear[a] > 0.0f) ? (double)cfg->gear[a] : 1.0;
    }
    g_vel = cfg->vel ? cfg->vel : CLEARAI_DEFAULT_VEL_STEPS;
    g_accel = cfg->accel ? cfg->accel : CLEARAI_DEFAULT_ACCEL_STEPS;
    g_decel = cfg->decel ? cfg->decel : CLEARAI_DEFAULT_DECEL_STEPS;
    g_feedMmPerMin = (cfg->feedMmPerMin > 0.0f) ? (double)cfg->feedMmPerMin : CLEARAI_DEFAULT_FEED_MM_PER_MIN;
    if (g_testMode) {
        g_interrupted = false;
    }
    return true;
}

static void HwLimClearAll() {
    for (uint8_t a = 0; a < CLEARAI_AXIS_COUNT; a++) {
        g_posLimDi[a] = 0;
        g_negLimDi[a] = 0;
    }
}

static bool HwLimDiDisabled(uint8_t di) {
    return di == 0 || di == 255;
}

static bool HwLimDiValid(uint32_t di) {
    return di == 0 || di == 255 || di <= 12;
}

static uint8_t HwLimDiNormalize(uint32_t di) {
    if (di == 0 || di == 255) {
        return 0;
    }
    if (di <= 12) {
        return (uint8_t)di;
    }
    return 255;
}

static Connector *LimitInputConnector(uint8_t pinIndex) {
    if (pinIndex > 12) {
        return nullptr;
    }
    return SysMgr.ConnectorByIndex((ClearCorePins)pinIndex);
}

static const char *PinModeTag(Connector::ConnectorModes mode) {
    switch (mode) {
        case Connector::INPUT_DIGITAL:
            return "in";
        case Connector::OUTPUT_DIGITAL:
            return "out";
        default:
            return "other";
    }
}

static bool PinReservedForLimit(uint8_t pinIndex) {
    for (uint8_t a = 0; a < CLEARAI_AXIS_COUNT; a++) {
        if (!HwLimDiDisabled(g_posLimDi[a]) && g_posLimDi[a] == pinIndex) {
            return true;
        }
        if (!HwLimDiDisabled(g_negLimDi[a]) && g_negLimDi[a] == pinIndex) {
            return true;
        }
    }
    return false;
}

static void EnsureLimitInputMode(uint8_t pinIndex) {
    Connector *input = LimitInputConnector(pinIndex);
    if (input) {
        /* IO-0..IO-5 are in/out; DI-6..A-12 are input-only. Limits always input. */
        input->Mode(Connector::INPUT_DIGITAL);
    }
}

static void ApplyHwLimitDiModes() {
    for (uint8_t a = 0; a < CLEARAI_AXIS_COUNT; a++) {
        if (!HwLimDiDisabled(g_posLimDi[a])) {
            EnsureLimitInputMode(g_posLimDi[a]);
        }
        if (!HwLimDiDisabled(g_negLimDi[a])) {
            EnsureLimitInputMode(g_negLimDi[a]);
        }
    }
    /* DI-6 may also be estop; keep it configured as digital input. */
    ConnectorDI6.Mode(Connector::INPUT_DIGITAL);
}

static bool HwLimitSwitchActive(uint8_t di) {
    if (HwLimDiDisabled(di)) {
        return false;
    }
    Connector *input = LimitInputConnector(di);
    return input && (input->State() != 0);
}

static bool HwLimitActive(uint8_t axis, bool positive) {
    if (!AxisOn(axis)) {
        return false;
    }
    const uint8_t di = positive ? g_posLimDi[axis] : g_negLimDi[axis];
    return HwLimitSwitchActive(di);
}

static bool ConfigApplyBlob(const ClearAiNvmConfig *cfg) {
    if (cfg->magic != CLEARAI_NVM_MAGIC) {
        return false;
    }
    if (cfg->version == CLEARAI_NVM_VERSION_V1) {
        if (cfg->size < sizeof(ClearAiNvmConfigV1)) {
            return false;
        }
        if (!ConfigApplyCommon((const ClearAiNvmConfigV1 *)cfg)) {
            return false;
        }
        LimitClearAll();
        HwLimClearAll();
        return true;
    }
    if (cfg->version == CLEARAI_NVM_VERSION_V2) {
        if (cfg->size < sizeof(ClearAiNvmConfigV2)) {
            return false;
        }
        const ClearAiNvmConfigV2 *v2 = (const ClearAiNvmConfigV2 *)cfg;
        if (!ConfigApplyCommon((const ClearAiNvmConfigV1 *)v2)) {
            return false;
        }
        g_limitFlags = v2->limitFlags;
        for (uint8_t a = 0; a < CLEARAI_AXIS_COUNT; a++) {
            g_limitMin[a] = (double)v2->limitMin[a];
            g_limitMax[a] = (double)v2->limitMax[a];
        }
        HwLimClearAll();
        g_unitsA = CLEARAI_UNITS_A_DEG;
        return true;
    }
    if (cfg->version == CLEARAI_NVM_VERSION_V3) {
        if (cfg->size < sizeof(ClearAiNvmConfigV3)) {
            return false;
        }
        const ClearAiNvmConfigV3 *v3 = (const ClearAiNvmConfigV3 *)cfg;
        if (!ConfigApplyCommon((const ClearAiNvmConfigV1 *)v3)) {
            return false;
        }
        g_limitFlags = v3->limitFlags;
        for (uint8_t a = 0; a < CLEARAI_AXIS_COUNT; a++) {
            g_limitMin[a] = (double)v3->limitMin[a];
            g_limitMax[a] = (double)v3->limitMax[a];
            g_posLimDi[a] = v3->posLimDi[a];
            g_negLimDi[a] = v3->negLimDi[a];
        }
        g_unitsA = CLEARAI_UNITS_A_DEG;
        return true;
    }
    if (cfg->version != CLEARAI_NVM_VERSION) {
        return false;
    }
    if (cfg->size < sizeof(ClearAiNvmConfig)) {
        return false;
    }
    if (!ConfigApplyCommon((const ClearAiNvmConfigV1 *)cfg)) {
        return false;
    }
    g_limitFlags = cfg->limitFlags;
    for (uint8_t a = 0; a < CLEARAI_AXIS_COUNT; a++) {
        g_limitMin[a] = (double)cfg->limitMin[a];
        g_limitMax[a] = (double)cfg->limitMax[a];
        g_posLimDi[a] = cfg->posLimDi[a];
        g_negLimDi[a] = cfg->negLimDi[a];
    }
    g_unitsA = (cfg->unitsA == (uint8_t)CLEARAI_UNITS_A_REV) ? CLEARAI_UNITS_A_REV : CLEARAI_UNITS_A_DEG;
    return true;
}

static bool ConfigLoadNvm() {
    ClearAiNvmConfig cfg;
    memset(&cfg, 0, sizeof(cfg));
    Nvm().BlockRead(NvmManager::NVM_LOC_USER_START, (int)sizeof(cfg), (uint8_t *)&cfg);
    if (!ConfigApplyBlob(&cfg)) {
        g_nvmLoaded = false;
        return false;
    }
    g_nvmLoaded = true;
    return true;
}

static void ConfigSaveNvm() {
    ClearAiNvmConfig cfg;
    ClearAiNvmConfig existing;
    ConfigFillFromLive(&cfg);
    memset(&existing, 0, sizeof(existing));
    Nvm().BlockRead(NvmManager::NVM_LOC_USER_START, (int)sizeof(existing), (uint8_t *)&existing);
    if (memcmp(&existing, &cfg, sizeof(cfg)) == 0) {
        g_nvmLoaded = true;
        return;
    }
    (void)Nvm().BlockWrite(NvmManager::NVM_LOC_USER_START, (int)sizeof(cfg), (const uint8_t *)&cfg);
    g_nvmLoaded = true;
}

static void ConfigClearNvm() {
    ClearAiNvmConfig cfg;
    memset(&cfg, 0, sizeof(cfg));
    ClearAiNvmConfig existing;
    memset(&existing, 0, sizeof(existing));
    Nvm().BlockRead(NvmManager::NVM_LOC_USER_START, (int)sizeof(existing), (uint8_t *)&existing);
    if (memcmp(&existing, &cfg, sizeof(cfg)) == 0) {
        g_nvmLoaded = false;
        return;
    }
    (void)Nvm().BlockWrite(NvmManager::NVM_LOC_USER_START, (int)sizeof(cfg), (const uint8_t *)&cfg);
    g_nvmLoaded = false;
}

static double ToInternal(uint8_t axis, double user) {
    if (axis == CLEARAI_AXIS_A) {
        return (g_unitsA == CLEARAI_UNITS_A_REV) ? user * 360.0 : user;
    }
    if (g_units == CLEARAI_UNITS_INCH) {
        return user * 25.4;
    }
    return user;
}

static double FromInternal(uint8_t axis, double internal) {
    if (axis == CLEARAI_AXIS_A) {
        return (g_unitsA == CLEARAI_UNITS_A_REV) ? internal / 360.0 : internal;
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

static double WorkFromSteps(uint8_t axis, int32_t steps) {
    const double spu = g_stepsPerMm[axis];
    if (spu == 0.0) {
        return 0.0;
    }
    return (double)steps / spu - g_workOrigin[axis];
}

static const char *ValidateTargetSteps(uint8_t axis, int32_t steps, bool moving) {
    if (!moving || !AxisOn(axis) || g_limitFlags == 0) {
        return nullptr;
    }
    const double work = WorkFromSteps(axis, steps);
    if (LimitMinEn(axis) && work < g_limitMin[axis] - 1e-6) {
        snprintf(g_limitErrBuf, sizeof(g_limitErrBuf), "%s below min limit", AxisName(axis));
        return g_limitErrBuf;
    }
    if (LimitMaxEn(axis) && work > g_limitMax[axis] + 1e-6) {
        snprintf(g_limitErrBuf, sizeof(g_limitErrBuf), "%s above max limit", AxisName(axis));
        return g_limitErrBuf;
    }
    return nullptr;
}

static const char *ValidateConfiguredLimits() {
    for (uint8_t a = 0; a < CLEARAI_AXIS_COUNT; a++) {
        if (LimitMinEn(a) && LimitMaxEn(a) && g_limitMin[a] > g_limitMax[a] + 1e-6) {
            snprintf(g_limitErrBuf, sizeof(g_limitErrBuf), "%s min above max", AxisName(a));
            return g_limitErrBuf;
        }
    }
    return nullptr;
}

static const char *ValidateHwLimitMove(uint8_t axis, int32_t cur, int32_t target, bool moving) {
    if (!moving || !AxisOn(axis)) {
        return nullptr;
    }
    if (target > cur && HwLimitActive(axis, true)) {
        snprintf(g_limitErrBuf, sizeof(g_limitErrBuf), "%s pos limit active", AxisName(axis));
        return g_limitErrBuf;
    }
    if (target < cur && HwLimitActive(axis, false)) {
        snprintf(g_limitErrBuf, sizeof(g_limitErrBuf), "%s neg limit active", AxisName(axis));
        return g_limitErrBuf;
    }
    return nullptr;
}

static const char *ValidateMoveTargets(bool hasX, int32_t tx, bool hasY, int32_t ty,
                                         bool hasZ, int32_t tz, bool hasA, int32_t ta) {
    const char *err;
    if (hasX) {
        if ((err = ValidateTargetSteps(CLEARAI_AXIS_X, tx, true)) != nullptr) {
            return err;
        }
        if ((err = ValidateHwLimitMove(CLEARAI_AXIS_X, MachineSteps(CLEARAI_AXIS_X), tx, true)) != nullptr) {
            return err;
        }
    }
    if (hasY) {
        if ((err = ValidateTargetSteps(CLEARAI_AXIS_Y, ty, true)) != nullptr) {
            return err;
        }
        if ((err = ValidateHwLimitMove(CLEARAI_AXIS_Y, MachineSteps(CLEARAI_AXIS_Y), ty, true)) != nullptr) {
            return err;
        }
    }
    if (hasZ) {
        if ((err = ValidateTargetSteps(CLEARAI_AXIS_Z, tz, true)) != nullptr) {
            return err;
        }
        if ((err = ValidateHwLimitMove(CLEARAI_AXIS_Z, MachineSteps(CLEARAI_AXIS_Z), tz, true)) != nullptr) {
            return err;
        }
    }
    if (hasA) {
        if ((err = ValidateTargetSteps(CLEARAI_AXIS_A, ta, true)) != nullptr) {
            return err;
        }
        if ((err = ValidateHwLimitMove(CLEARAI_AXIS_A, MachineSteps(CLEARAI_AXIS_A), ta, true)) != nullptr) {
            return err;
        }
    }
    return nullptr;
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
        return;
    }
    if (!MotorsIdle()) {
        for (uint8_t a = 0; a < CLEARAI_AXIS_COUNT; a++) {
            if (!AxisOn(a)) {
                continue;
            }
            MotorDriver *m = MotorForAxis(a);
            if (!m || m->StepsComplete() || !m->StatusReg().bit.StepsActive) {
                continue;
            }
            const bool posDir = (m->StatusReg().bit.MoveDirection != 0);
            if ((posDir && HwLimitActive(a, true)) || (!posDir && HwLimitActive(a, false))) {
                g_xy.StopDecel();
                for (uint8_t b = 0; b < CLEARAI_AXIS_COUNT; b++) {
                    MotorDriver *mb = MotorForAxis(b);
                    if (mb) {
                        mb->MoveStopDecel(g_decel);
                    }
                }
                return;
            }
        }
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
    g_axisMask = CLEARAI_DEFAULT_AXIS_MASK;
    g_units = CLEARAI_UNITS_MM;
    g_unitsA = CLEARAI_UNITS_A_DEG;
    g_mode = CLEARAI_MODE_ABS;
    g_vel = CLEARAI_DEFAULT_VEL_STEPS;
    g_accel = CLEARAI_DEFAULT_ACCEL_STEPS;
    g_decel = CLEARAI_DEFAULT_DECEL_STEPS;
    g_feedMmPerMin = CLEARAI_DEFAULT_FEED_MM_PER_MIN;
    g_estopDi6 = 1;
    g_testMode = (CLEARAI_TEST_MODE_DEFAULT != 0);
    g_enabled = false;
    g_interrupted = false;
    g_nvmLoaded = false;
    LimitClearAll();
    HwLimClearAll();
    (void)ConfigLoadNvm();

    MotorMgr.MotorModeSet(MotorManager::MOTOR_ALL, Connector::CPM_MODE_STEP_AND_DIR);
    Delay_ms(50);
    ApplyHwLimitDiModes();

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
    RecalcStepsPerUnit();
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
    ConfigSaveNvm();
    return nullptr;
}

const char *MotionConfigure(const RpcParams *p) {
    if (p->hasTestMode) {
        ApplyTestMode(p->testMode);
    }
    const bool mechChange = p->hasStepsX || p->hasStepsY || p->hasStepsZ || p->hasStepsA ||
        p->hasPitchX || p->hasPitchY || p->hasPitchZ || p->hasPitchA ||
        p->hasGearX || p->hasGearY || p->hasGearZ || p->hasGearA ||
        p->hasVel || p->hasAccel || p->hasDecel || p->hasAxisMask || p->hasEstopDi6;
    const bool limitChange = p->hasMinX || p->hasMaxX || p->hasMinY || p->hasMaxY ||
        p->hasMinZ || p->hasMaxZ || p->hasMinA || p->hasMaxA || p->hasClearLimits ||
        p->hasClearMinX || p->hasClearMaxX || p->hasClearMinY || p->hasClearMaxY ||
        p->hasClearMinZ || p->hasClearMaxZ || p->hasClearMinA || p->hasClearMaxA ||
        p->hasPosLimX || p->hasNegLimX || p->hasPosLimY || p->hasNegLimY ||
        p->hasPosLimZ || p->hasNegLimZ || p->hasPosLimA || p->hasNegLimA;
    const bool testOnly = p->hasTestMode && !mechChange && !limitChange;

    if (mechChange && g_enabled) {
        return "disable before configure";
    }
    if (g_enabled && !testOnly && !limitChange) {
        return "disable before configure";
    }
    if (testOnly) {
        ConfigSaveNvm();
        return nullptr;
    }

    if (p->hasClearLimits && p->clearLimits) {
        LimitClearAll();
        HwLimClearAll();
    }
    if (p->hasClearMinX && p->clearMinX) {
        LimitClearMin(CLEARAI_AXIS_X);
    }
    if (p->hasClearMaxX && p->clearMaxX) {
        LimitClearMax(CLEARAI_AXIS_X);
    }
    if (p->hasClearMinY && p->clearMinY) {
        LimitClearMin(CLEARAI_AXIS_Y);
    }
    if (p->hasClearMaxY && p->clearMaxY) {
        LimitClearMax(CLEARAI_AXIS_Y);
    }
    if (p->hasClearMinZ && p->clearMinZ) {
        LimitClearMin(CLEARAI_AXIS_Z);
    }
    if (p->hasClearMaxZ && p->clearMaxZ) {
        LimitClearMax(CLEARAI_AXIS_Z);
    }
    if (p->hasClearMinA && p->clearMinA) {
        LimitClearMin(CLEARAI_AXIS_A);
    }
    if (p->hasClearMaxA && p->clearMaxA) {
        LimitClearMax(CLEARAI_AXIS_A);
    }
    if (p->hasMinX) {
        LimitSetMin(CLEARAI_AXIS_X, ToInternal(CLEARAI_AXIS_X, p->minX));
    }
    if (p->hasMaxX) {
        LimitSetMax(CLEARAI_AXIS_X, ToInternal(CLEARAI_AXIS_X, p->maxX));
    }
    if (p->hasMinY) {
        LimitSetMin(CLEARAI_AXIS_Y, ToInternal(CLEARAI_AXIS_Y, p->minY));
    }
    if (p->hasMaxY) {
        LimitSetMax(CLEARAI_AXIS_Y, ToInternal(CLEARAI_AXIS_Y, p->maxY));
    }
    if (p->hasMinZ) {
        LimitSetMin(CLEARAI_AXIS_Z, ToInternal(CLEARAI_AXIS_Z, p->minZ));
    }
    if (p->hasMaxZ) {
        LimitSetMax(CLEARAI_AXIS_Z, ToInternal(CLEARAI_AXIS_Z, p->maxZ));
    }
    if (p->hasMinA) {
        LimitSetMin(CLEARAI_AXIS_A, ToInternal(CLEARAI_AXIS_A, p->minA));
    }
    if (p->hasMaxA) {
        LimitSetMax(CLEARAI_AXIS_A, ToInternal(CLEARAI_AXIS_A, p->maxA));
    }
    if (p->hasPosLimX) {
        if (!HwLimDiValid(p->posLimX)) {
            return "invalid limit di";
        }
        g_posLimDi[CLEARAI_AXIS_X] = HwLimDiNormalize(p->posLimX);
    }
    if (p->hasNegLimX) {
        if (!HwLimDiValid(p->negLimX)) {
            return "invalid limit di";
        }
        g_negLimDi[CLEARAI_AXIS_X] = HwLimDiNormalize(p->negLimX);
    }
    if (p->hasPosLimY) {
        if (!HwLimDiValid(p->posLimY)) {
            return "invalid limit di";
        }
        g_posLimDi[CLEARAI_AXIS_Y] = HwLimDiNormalize(p->posLimY);
    }
    if (p->hasNegLimY) {
        if (!HwLimDiValid(p->negLimY)) {
            return "invalid limit di";
        }
        g_negLimDi[CLEARAI_AXIS_Y] = HwLimDiNormalize(p->negLimY);
    }
    if (p->hasPosLimZ) {
        if (!HwLimDiValid(p->posLimZ)) {
            return "invalid limit di";
        }
        g_posLimDi[CLEARAI_AXIS_Z] = HwLimDiNormalize(p->posLimZ);
    }
    if (p->hasNegLimZ) {
        if (!HwLimDiValid(p->negLimZ)) {
            return "invalid limit di";
        }
        g_negLimDi[CLEARAI_AXIS_Z] = HwLimDiNormalize(p->negLimZ);
    }
    if (p->hasPosLimA) {
        if (!HwLimDiValid(p->posLimA)) {
            return "invalid limit di";
        }
        g_posLimDi[CLEARAI_AXIS_A] = HwLimDiNormalize(p->posLimA);
    }
    if (p->hasNegLimA) {
        if (!HwLimDiValid(p->negLimA)) {
            return "invalid limit di";
        }
        g_negLimDi[CLEARAI_AXIS_A] = HwLimDiNormalize(p->negLimA);
    }
    ApplyHwLimitDiModes();
    {
        const char *limitErr = ValidateConfiguredLimits();
        if (limitErr) {
            return limitErr;
        }
    }

    if (!mechChange && limitChange) {
        ConfigSaveNvm();
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
    ApplyMechanicalParams();
    ConfigSaveNvm();
    return nullptr;
}

const char *MotionSetUnits(const RpcParams *p) {
    if (!p->hasUnits) {
        return "units required";
    }
    if (strcmp(p->units, "mm") == 0) {
        g_units = CLEARAI_UNITS_MM;
        ConfigSaveNvm();
        return nullptr;
    }
    if (strcmp(p->units, "inch") == 0 || strcmp(p->units, "in") == 0) {
        g_units = CLEARAI_UNITS_INCH;
        ConfigSaveNvm();
        return nullptr;
    }
    return "units must be mm or inch";
}

const char *MotionSetUnitsA(const RpcParams *p) {
    if (!p->hasUnits) {
        return "units required";
    }
    if (strcmp(p->units, "deg") == 0 || strcmp(p->units, "degree") == 0 ||
        strcmp(p->units, "degrees") == 0) {
        g_unitsA = CLEARAI_UNITS_A_DEG;
        ConfigSaveNvm();
        return nullptr;
    }
    if (strcmp(p->units, "rev") == 0 || strcmp(p->units, "revolution") == 0 ||
        strcmp(p->units, "revs") == 0) {
        g_unitsA = CLEARAI_UNITS_A_REV;
        ConfigSaveNvm();
        return nullptr;
    }
    return "units must be deg or rev";
}

const char *MotionSetMode(const RpcParams *p) {
    if (!p->hasMode) {
        return "mode required";
    }
    if (strcmp(p->mode, "abs") == 0) {
        g_mode = CLEARAI_MODE_ABS;
        ConfigSaveNvm();
        return nullptr;
    }
    if (strcmp(p->mode, "rel") == 0) {
        g_mode = CLEARAI_MODE_REL;
        ConfigSaveNvm();
        return nullptr;
    }
    return "mode must be abs or rel";
}

const char *MotionResetConfig() {
    if (g_enabled) {
        return "disable before reset_config";
    }
    for (uint8_t a = 0; a < CLEARAI_AXIS_COUNT; a++) {
        g_stepsPerRev[a] = CLEARAI_DEFAULT_STEPS_PER_REV;
        g_pitchMm[a] = CLEARAI_DEFAULT_PITCH_MM;
        g_gear[a] = 1.0;
        g_workOrigin[a] = 0.0;
    }
    g_axisMask = CLEARAI_DEFAULT_AXIS_MASK;
    g_units = CLEARAI_UNITS_MM;
    g_unitsA = CLEARAI_UNITS_A_DEG;
    g_mode = CLEARAI_MODE_ABS;
    g_vel = CLEARAI_DEFAULT_VEL_STEPS;
    g_accel = CLEARAI_DEFAULT_ACCEL_STEPS;
    g_decel = CLEARAI_DEFAULT_DECEL_STEPS;
    g_feedMmPerMin = CLEARAI_DEFAULT_FEED_MM_PER_MIN;
    g_estopDi6 = 1;
    g_testMode = (CLEARAI_TEST_MODE_DEFAULT != 0);
    LimitClearAll();
    HwLimClearAll();
    ApplyMechanicalParams();
    ConfigClearNvm();
    return nullptr;
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
    const char *limitErr = ValidateMoveTargets(p->hasX, tx, p->hasY, ty, p->hasZ, tz, p->hasA, ta);
    if (limitErr) {
        return limitErr;
    }
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
    const char *limitErr = ValidateMoveTargets(p->hasX, ex, p->hasY, ey, false, 0, false, 0);
    if (limitErr) {
        return limitErr;
    }
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
    const char *limitErr = ValidateMoveTargets(p->hasX, tx, p->hasY, ty, p->hasZ, tz, p->hasA, ta);
    if (limitErr) {
        return limitErr;
    }
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
    out->queueActive = g_xy.IsActive();
    out->units = g_units;
    out->unitsA = g_unitsA;
    out->mode = g_mode;
    out->axisMask = g_axisMask;
    out->vel = g_vel;
    out->accel = g_accel;
    for (uint8_t a = 0; a < CLEARAI_AXIS_COUNT; a++) {
        MotorDriver *m = MotorForAxis(a);
        if (m) {
            out->alertReg |= m->AlertReg().reg;
            out->hlfbPercent[a] = m->HlfbPercent();
        } else {
            out->hlfbPercent[a] = MotorDriver::HLFB_DUTY_UNKNOWN;
        }
        out->work[a] = FromInternal(a, WorkInternal(a));
    }
}

static void MotionFillInputsJson(char *buf, uint16_t bufLen, uint8_t startPin, uint8_t endPin) {
    int pos = snprintf(buf, bufLen, "{\"pins\":[");
    if (pos < 0 || (uint16_t)pos >= bufLen) {
        buf[0] = '\0';
        return;
    }
    for (uint8_t pin = startPin; pin <= endPin; pin++) {
        Connector *connector = LimitInputConnector(pin);
        const char *mode = "none";
        int state = 0;
        if (connector) {
            state = (connector->State() != 0) ? 1 : 0;
            mode = PinModeTag(connector->Mode());
        }
        const int n = snprintf(buf + pos, (size_t)(bufLen - (uint16_t)pos),
                               "%s{\"pin\":%u,\"state\":%d,\"mode\":\"%s\"}",
                               pin > startPin ? "," : "",
                               (unsigned)pin, state, mode);
        if (n < 0) {
            break;
        }
        pos += n;
        if ((uint16_t)pos >= bufLen) {
            break;
        }
    }
    if ((uint16_t)pos + 2 < bufLen) {
        snprintf(buf + pos, (size_t)(bufLen - (uint16_t)pos), "]}");
    }
}

const char *MotionReadInputs(const RpcParams *p, char *buf, uint16_t bufLen) {
    uint8_t startPin = 0;
    uint8_t endPin = 12;
    if (p->hasPin) {
        if (p->pin > 12) {
            return "invalid pin";
        }
        startPin = (uint8_t)p->pin;
        endPin = startPin;
    }
    MotionFillInputsJson(buf, bufLen, startPin, endPin);
    return nullptr;
}

const char *MotionWriteOutput(const RpcParams *p) {
    if (!p->hasPin) {
        return "pin required";
    }
    if (!p->hasIoState) {
        return "state required";
    }
    if (p->pin > 5) {
        return "pin not output capable";
    }
    if (PinReservedForLimit((uint8_t)p->pin)) {
        return "pin reserved for limit";
    }
    Connector *connector = LimitInputConnector((uint8_t)p->pin);
    if (!connector) {
        return "pin missing";
    }
    if (!connector->Mode(Connector::OUTPUT_DIGITAL)) {
        return "output mode rejected";
    }
    if (!connector->State(p->ioState ? 1 : 0)) {
        return "output rejected";
    }
    return nullptr;
}

const char *MotionQueueClear() {
    /* StopDecel() decelerates the active coordinated segment to a stop and
     * drops all pending queued segments (it zeros the planner queue counts). */
    g_xy.StopDecel();
    for (uint8_t a = 0; a < CLEARAI_AXIS_COUNT; a++) {
        MotorDriver *m = MotorForAxis(a);
        if (m) {
            m->MoveStopDecel(g_decel);
        }
    }
    return nullptr;
}

void MotionFillQueueStatusJson(char *buf, uint16_t bufLen) {
    snprintf(buf, bufLen, "{\"queue\":%u,\"active\":%s}",
             (unsigned)g_xy.MotionQueueCount(),
             g_xy.IsActive() ? "true" : "false");
}

/* ---- Homing / probing ----------------------------------------------------- */

struct SeekTrigger {
    bool checkLimit;   /* homing: trip on the configured hw limit for axis/dir */
    Connector *probe;  /* probing: trip on this digital input */
    bool probeActiveHigh;
};

static bool SeekTriggered(const SeekTrigger *t, uint8_t axis, bool positive) {
    if (t->checkLimit) {
        return HwLimitActive(axis, positive);
    }
    if (!t->probe) {
        return false;
    }
    return t->probeActiveHigh ? (t->probe->State() != 0) : (t->probe->State() == 0);
}

/* Command a single-axis seek move toward `positive` by `seekSteps`. Bypasses
 * soft limits (the hardware limit / probe input is expected to stop it). */
static const char *SeekMove(uint8_t axis, bool positive, int32_t seekSteps) {
    if (seekSteps <= 0) {
        return "seek invalid";
    }
    const int32_t cur = MachineSteps(axis);
    const int32_t far = cur + (positive ? seekSteps : -seekSteps);
    if (axis == CLEARAI_AXIS_X || axis == CLEARAI_AXIS_Y) {
        const int32_t tx = (axis == CLEARAI_AXIS_X) ? far : MachineSteps(CLEARAI_AXIS_X);
        const int32_t ty = (axis == CLEARAI_AXIS_Y) ? far : MachineSteps(CLEARAI_AXIS_Y);
        const bool hasX = (axis == CLEARAI_AXIS_X);
        const bool hasY = (axis == CLEARAI_AXIS_Y);
        return IssueXyLinear(tx, ty, hasX, hasY);
    }
    return MoveAxisAbs(axis, far);
}

/* Returns 0 on trigger, 1 if seek distance exhausted with no trigger,
 * -1 on estop/safety interrupt, -2 on timeout. */
static int SeekUntilTrigger(uint8_t axis, bool positive, const SeekTrigger *trig,
                             uint32_t timeoutMs, ClearAiUrgentFn urgent) {
    const uint32_t start = Milliseconds();
    for (;;) {
        MotionPollEstop();
        if (g_interrupted) {
            return -1;
        }
        if (urgent && urgent()) {
            return -1;
        }
        if (SeekTriggered(trig, axis, positive)) {
            /* Trip: decelerate everything to a stop. For hw limits MotionPollEstop
             * may have already started the stop; this is idempotent. */
            g_xy.StopDecel();
            for (uint8_t a = 0; a < CLEARAI_AXIS_COUNT; a++) {
                MotorDriver *m = MotorForAxis(a);
                if (m) {
                    m->MoveStopDecel(g_decel);
                }
            }
            while (!MotorsIdle()) {
                MotionPollEstop();
                if (g_interrupted) {
                    return -1;
                }
                Delay_ms(1);
            }
            return 0;
        }
        if (MotorsIdle()) {
            return 1;
        }
        if (timeoutMs > 0 && (Milliseconds() - start) >= timeoutMs) {
            return -2;
        }
        Delay_ms(1);
    }
}

const char *MotionHome(const RpcParams *p, char *body, uint16_t bufLen, ClearAiUrgentFn urgent) {
    const char *err = GateMotion();
    if (err) {
        return err;
    }
    if (!p->hasAxis) {
        return "axis required";
    }
    uint8_t axis;
    if (!AxisFromStr(p->axis, &axis)) {
        return "axis must be x, y, z, or a";
    }
    if (!AxisOn(axis)) {
        return "axis not in axis mask";
    }
    if (!p->hasDir) {
        return "dir required";
    }
    bool positive;
    if (strcmp(p->dir, "pos") == 0) {
        positive = true;
    } else if (strcmp(p->dir, "neg") == 0) {
        positive = false;
    } else {
        return "dir must be pos or neg";
    }
    const uint8_t limDi = positive ? g_posLimDi[axis] : g_negLimDi[axis];
    if (HwLimDiDisabled(limDi)) {
        return "limit not configured for this axis/dir";
    }
    if (!HwLimDiValid(limDi)) {
        return "limit pin invalid";
    }
    EnsureLimitInputMode(limDi);
    if (HwLimitSwitchActive(limDi)) {
        return "limit already active; back off first";
    }

    ApplyFeed(false, p->hasFeed, p->feed);
    const double seekUser = p->hasSeek ? p->seek : CLEARAI_HOME_SEEK_DEFAULT;
    const int32_t seekSteps = I32Round(ToInternal(axis, seekUser) * g_stepsPerMm[axis]);
    const char *merr = SeekMove(axis, positive, seekSteps);
    if (merr) {
        return merr;
    }

    SeekTrigger trig;
    trig.checkLimit = true;
    trig.probe = nullptr;
    trig.probeActiveHigh = true;
    const uint32_t timeoutMs = p->hasTimeoutMs ? p->timeoutMs : CLEARAI_HOME_TIMEOUT_MS;
    const int rc = SeekUntilTrigger(axis, positive, &trig, timeoutMs, urgent);
    if (rc == -1) {
        return g_interrupted ? "estop during home" : "interrupted by safety command";
    }
    if (rc == -2) {
        return "home timeout";
    }
    if (rc == 1) {
        return "limit not reached";
    }

    const double backoff = p->hasBackoff ? p->backoff : 0.0;
    if (backoff > 0.0) {
        const int32_t backSteps = I32Round(ToInternal(axis, backoff) * g_stepsPerMm[axis]);
        const char *berr = SeekMove(axis, !positive, backSteps);
        if (berr) {
            return berr;
        }
        const char *werr = WaitLoop(CLEARAI_SEEK_BACKOFF_MS, true, urgent);
        if (werr) {
            return werr;
        }
    }

    const bool zero = p->hasZero ? p->zero : true;
    if (zero) {
        g_workOrigin[axis] = MachineInternal(axis);
    }

    const double posUser = FromInternal(axis, WorkInternal(axis));
    snprintf(body, bufLen,
             "{\"homed\":true,\"axis\":\"%s\",\"dir\":\"%s\",\"pos\":%.4f,\"limit_pin\":%u}",
             AxisName(axis), positive ? "pos" : "neg", posUser, (unsigned)limDi);
    return nullptr;
}

const char *MotionProbe(const RpcParams *p, char *body, uint16_t bufLen, ClearAiUrgentFn urgent) {
    const char *err = GateMotion();
    if (err) {
        return err;
    }
    if (!p->hasAxis) {
        return "axis required";
    }
    uint8_t axis;
    if (!AxisFromStr(p->axis, &axis)) {
        return "axis must be x, y, z, or a";
    }
    if (!AxisOn(axis)) {
        return "axis not in axis mask";
    }
    if (!p->hasDir) {
        return "dir required";
    }
    bool positive;
    if (strcmp(p->dir, "pos") == 0) {
        positive = true;
    } else if (strcmp(p->dir, "neg") == 0) {
        positive = false;
    } else {
        return "dir must be pos or neg";
    }
    if (!p->hasPin) {
        return "pin required";
    }
    if (!HwLimDiValid(p->pin) || p->pin == 0 || p->pin == 255) {
        return "pin must be 1-12";
    }
    const uint8_t probePin = (uint8_t)p->pin;
    if (PinReservedForLimit(probePin)) {
        return "pin reserved for limit";
    }
    Connector *probe = LimitInputConnector(probePin);
    if (!probe) {
        return "pin missing";
    }
    EnsureLimitInputMode(probePin);
    bool activeHigh = true;
    if (p->hasActive) {
        if (strcmp(p->active, "low") == 0) {
            activeHigh = false;
        } else if (strcmp(p->active, "high") == 0) {
            activeHigh = true;
        } else {
            return "active must be high or low";
        }
    }
    if (activeHigh ? (probe->State() != 0) : (probe->State() == 0)) {
        return "probe already active";
    }

    ApplyFeed(false, p->hasFeed, p->feed);
    const double seekUser = p->hasSeek ? p->seek : CLEARAI_HOME_SEEK_DEFAULT;
    const int32_t seekSteps = I32Round(ToInternal(axis, seekUser) * g_stepsPerMm[axis]);
    const char *merr = SeekMove(axis, positive, seekSteps);
    if (merr) {
        return merr;
    }

    SeekTrigger trig;
    trig.checkLimit = false;
    trig.probe = probe;
    trig.probeActiveHigh = activeHigh;
    const uint32_t timeoutMs = p->hasTimeoutMs ? p->timeoutMs : CLEARAI_PROBE_TIMEOUT_MS;
    const int rc = SeekUntilTrigger(axis, positive, &trig, timeoutMs, urgent);
    if (rc == -1) {
        return g_interrupted ? "estop during probe" : "interrupted by safety command";
    }
    if (rc == -2) {
        return "probe timeout";
    }
    if (rc == 1) {
        return "probe not reached";
    }

    const double touchUser = FromInternal(axis, WorkInternal(axis));
    const double backoff = p->hasBackoff ? p->backoff : 0.0;
    if (backoff > 0.0) {
        const int32_t backSteps = I32Round(ToInternal(axis, backoff) * g_stepsPerMm[axis]);
        const char *berr = SeekMove(axis, !positive, backSteps);
        if (berr) {
            return berr;
        }
        const char *werr = WaitLoop(CLEARAI_SEEK_BACKOFF_MS, true, urgent);
        if (werr) {
            return werr;
        }
    }

    const bool zero = p->hasZero ? p->zero : false;
    if (zero) {
        g_workOrigin[axis] = MachineInternal(axis);
    }

    snprintf(body, bufLen,
             "{\"probed\":true,\"axis\":\"%s\",\"dir\":\"%s\",\"pos\":%.4f,\"pin\":%u}",
             AxisName(axis), positive ? "pos" : "neg", touchUser, (unsigned)probePin);
    return nullptr;
}

void MotionFillCapabilitiesJson(char *buf, uint16_t bufLen) {
    snprintf(buf, bufLen,
             "{\"protocol\":\"clearai\",\"version\":\"%s\",\"axes\":[\"x\",\"y\",\"z\",\"a\"],"
             "\"axis_mask\":%lu,\"units\":\"%s\",\"mode\":\"%s\","
             "\"test_mode\":%s,\"nvm\":%s,\"methods\":[\"get_capabilities\",\"get_status\",\"get_pose\","
             "\"get_config\",\"enable\",\"disable\",\"clear_alerts\",\"stop\",\"estop\",\"wait_idle\","
             "\"configure\",\"reset_config\",\"set_test_mode\",\"set_units\",\"set_units_a\",\"set_mode\","
             "\"set_work_origin\",\"move_linear\",\"move_arc\",\"jog\",\"dwell\","
             "\"read_inputs\",\"write_output\",\"queue_status\",\"queue_clear\","
             "\"home\",\"probe\"],"
             "\"tcp\":%u,\"tel\":%u,\"discover\":%u}",
             CLEARAI_PROTOCOL_VERSION,
             (unsigned long)g_axisMask,
             g_units == CLEARAI_UNITS_MM ? "mm" : "inch",
             g_mode == CLEARAI_MODE_ABS ? "abs" : "rel",
             g_testMode ? "true" : "false",
             g_nvmLoaded ? "true" : "false",
             CLEARAI_TCP_CONTROL_PORT, CLEARAI_TCP_TELEMETRY_PORT,
             CLEARAI_UDP_DISCOVERY_PORT);
}

void MotionFillConfigJson(char *buf, uint16_t bufLen) {
    /* Live config (matches NVM after boot load / successful save). */
    ClearAiNvmConfig stored;
    bool storedOk = false;
    memset(&stored, 0, sizeof(stored));
    Nvm().BlockRead(NvmManager::NVM_LOC_USER_START, (int)sizeof(stored), (uint8_t *)&stored);
    storedOk = (stored.magic == CLEARAI_NVM_MAGIC &&
                (stored.version == CLEARAI_NVM_VERSION ||
                 stored.version == CLEARAI_NVM_VERSION_V3 ||
                 stored.version == CLEARAI_NVM_VERSION_V2 ||
                 stored.version == CLEARAI_NVM_VERSION_V1) &&
                stored.size >= sizeof(ClearAiNvmConfigV1));

    double limMinUser[CLEARAI_AXIS_COUNT];
    double limMaxUser[CLEARAI_AXIS_COUNT];
    unsigned posLimDi[CLEARAI_AXIS_COUNT];
    unsigned negLimDi[CLEARAI_AXIS_COUNT];
    for (uint8_t a = 0; a < CLEARAI_AXIS_COUNT; a++) {
        limMinUser[a] = FromInternal(a, g_limitMin[a]);
        limMaxUser[a] = FromInternal(a, g_limitMax[a]);
        posLimDi[a] = g_posLimDi[a];
        negLimDi[a] = g_negLimDi[a];
    }

    snprintf(buf, bufLen,
             "{\"nvm\":%s,\"nvm_valid\":%s,\"nvm_version\":%u,"
             "\"axis_mask\":%lu,\"test_mode\":%s,\"units\":\"%s\",\"units_a\":\"%s\",\"mode\":\"%s\","
             "\"estop_di6\":%u,\"vel\":%lu,\"accel\":%lu,\"decel\":%lu,\"feed\":%.3f,"
             "\"steps_per_rev\":[%lu,%lu,%lu,%lu],"
             "\"pitch_mm\":[%.4f,%.4f,%.4f,%.4f],"
             "\"gear\":[%.4f,%.4f,%.4f,%.4f],"
             "\"limit_flags\":%u,"
             "\"limits_min\":[%.4f,%.4f,%.4f,%.4f],"
             "\"limits_max\":[%.4f,%.4f,%.4f,%.4f],"
             "\"pos_lim_di\":[%u,%u,%u,%u],"
             "\"neg_lim_di\":[%u,%u,%u,%u]}",
             g_nvmLoaded ? "true" : "false",
             storedOk ? "true" : "false",
             (unsigned)(storedOk ? stored.version : 0),
             (unsigned long)g_axisMask,
             g_testMode ? "true" : "false",
             g_units == CLEARAI_UNITS_MM ? "mm" : "inch",
             g_unitsA == CLEARAI_UNITS_A_REV ? "rev" : "deg",
             g_mode == CLEARAI_MODE_ABS ? "abs" : "rel",
             (unsigned)g_estopDi6,
             (unsigned long)g_vel,
             (unsigned long)g_accel,
             (unsigned long)g_decel,
             g_feedMmPerMin,
             (unsigned long)g_stepsPerRev[0], (unsigned long)g_stepsPerRev[1],
             (unsigned long)g_stepsPerRev[2], (unsigned long)g_stepsPerRev[3],
             g_pitchMm[0], g_pitchMm[1], g_pitchMm[2], g_pitchMm[3],
             g_gear[0], g_gear[1], g_gear[2], g_gear[3],
             (unsigned)g_limitFlags,
             limMinUser[0], limMinUser[1], limMinUser[2], limMinUser[3],
             limMaxUser[0], limMaxUser[1], limMaxUser[2], limMaxUser[3],
             posLimDi[0], posLimDi[1], posLimDi[2], posLimDi[3],
             negLimDi[0], negLimDi[1], negLimDi[2], negLimDi[3]);
}

static void FormatHlfbPercentArray(char *out, size_t outLen, const MotionStatus *st) {
    size_t pos = 0;
    if (pos + 1 < outLen) {
        out[pos++] = '[';
    }
    for (uint8_t a = 0; a < CLEARAI_AXIS_COUNT; a++) {
        const char *sep = (a > 0) ? "," : "";
        if (st->hlfbPercent[a] == MotorDriver::HLFB_DUTY_UNKNOWN) {
            pos += (size_t)snprintf(out + pos, outLen - pos, "%snull", sep);
        } else {
            pos += (size_t)snprintf(out + pos, outLen - pos, "%s%.2f", sep, st->hlfbPercent[a]);
        }
        if (pos >= outLen) {
            break;
        }
    }
    if (pos + 1 < outLen) {
        out[pos++] = ']';
        out[pos] = '\0';
    }
}

static void FillPoseObject(char *buf, uint16_t bufLen, bool withFlags) {
    MotionStatus st;
    MotionGetStatus(&st);
    if (withFlags) {
        char hlfbArr[64];
        FormatHlfbPercentArray(hlfbArr, sizeof(hlfbArr), &st);
        snprintf(buf, bufLen,
                 "{\"enabled\":%s,\"moving\":%s,\"hlfb\":%s,\"estop\":%s,\"hw_estop\":%s,"
                 "\"test_mode\":%s,\"alerts\":%s,"
                 "\"queue\":%u,\"queue_active\":%s,\"alert_reg\":%lu,"
                 "\"units\":\"%s\",\"units_a\":\"%s\",\"mode\":\"%s\","
                 "\"axis_mask\":%lu,\"x\":%.4f,\"y\":%.4f,\"z\":%.4f,\"a\":%.4f,"
                 "\"hlfb_percent\":%s}",
                 st.enabled ? "true" : "false",
                 st.moving ? "true" : "false",
                 st.hlfb ? "true" : "false",
                 st.estop ? "true" : "false",
                 st.hwEstop ? "true" : "false",
                 st.testMode ? "true" : "false",
                 st.alerts ? "true" : "false",
                 (unsigned)st.queue,
                 st.queueActive ? "true" : "false",
                 (unsigned long)st.alertReg,
                 st.units == CLEARAI_UNITS_MM ? "mm" : "inch",
                 st.unitsA == CLEARAI_UNITS_A_REV ? "rev" : "deg",
                 st.mode == CLEARAI_MODE_ABS ? "abs" : "rel",
                 (unsigned long)st.axisMask,
                 st.work[0], st.work[1], st.work[2], st.work[3],
                 hlfbArr);
    } else {
        snprintf(buf, bufLen,
                 "{\"units\":\"%s\",\"units_a\":\"%s\",\"mode\":\"%s\","
                 "\"x\":%.4f,\"y\":%.4f,\"z\":%.4f,\"a\":%.4f}",
                 st.units == CLEARAI_UNITS_MM ? "mm" : "inch",
                 st.unitsA == CLEARAI_UNITS_A_REV ? "rev" : "deg",
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
