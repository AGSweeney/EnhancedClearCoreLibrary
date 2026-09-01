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
#include "Transport.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

using ClearCore::NvmManager;

static NvmManager &Nvm() {
    return NvmManager::Instance();
}

/* User-page blob at NVM_LOC_USER_START (416 bytes available before Teknic reserved). */
static const uint32_t CLEARAI_NVM_MAGIC = 0x43414943u; /* 'CAIC' */
static const uint16_t CLEARAI_NVM_VERSION = 7;
static const uint16_t CLEARAI_NVM_VERSION_V1 = 1;
static const uint16_t CLEARAI_NVM_VERSION_V2 = 2;
static const uint16_t CLEARAI_NVM_VERSION_V3 = 3;
static const uint16_t CLEARAI_NVM_VERSION_V4 = 4;
static const uint16_t CLEARAI_NVM_VERSION_V5 = 5;
static const uint16_t CLEARAI_NVM_VERSION_V6 = 6;

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

struct ClearAiNvmConfigV4 {
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

struct ClearAiNvmConfigV5 {
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
    uint32_t velAxis[CLEARAI_AXIS_COUNT];
    uint32_t accelAxis[CLEARAI_AXIS_COUNT];
    uint32_t decelAxis[CLEARAI_AXIS_COUNT];
    uint32_t watchdogMs;
    uint8_t nvmPad2[4];
};

struct ClearAiNvmConfigV6 {
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
    uint32_t velAxis[CLEARAI_AXIS_COUNT];
    uint32_t accelAxis[CLEARAI_AXIS_COUNT];
    uint32_t decelAxis[CLEARAI_AXIS_COUNT];
    uint32_t watchdogMs;
    uint8_t nvmPad2[4];
    uint8_t outPowerOnState[6];
    uint8_t outPowerOnMask;
    uint8_t nvmPad3[3];
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
    uint32_t velAxis[CLEARAI_AXIS_COUNT];
    uint32_t accelAxis[CLEARAI_AXIS_COUNT];
    uint32_t decelAxis[CLEARAI_AXIS_COUNT];
    uint32_t watchdogMs;
    uint8_t nvmPad2[4];
    uint8_t outPowerOnState[6];
    uint8_t outPowerOnMask;
    uint8_t nvmPad3[3];
    uint8_t netMode;            /* 0 = DHCP, 1 = static */
    uint8_t ipOctets[4];
    uint8_t netmaskOctets[4];
    uint8_t gatewayOctets[4];
    uint8_t nvmPad4[3];
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
static uint32_t g_velAxis[CLEARAI_AXIS_COUNT];     /* 0 = inherit g_vel */
static uint32_t g_accelAxis[CLEARAI_AXIS_COUNT];  /* 0 = inherit g_accel */
static uint32_t g_decelAxis[CLEARAI_AXIS_COUNT];  /* 0 = inherit g_decel */
static uint32_t g_watchdogMs = 0;               /* 0 = disabled */
static uint32_t g_lastKeepaliveMs = 0;
static bool g_watchdogTripped = false;
static uint8_t g_limitTrippedAxis = 0xff;       /* 0xff = none */
static bool g_limitTrippedPos = false;
static uint8_t g_pwmDuty[6] = {0};            /* last commanded PWM duty per IO pin */
static uint8_t g_outPowerOnState[6] = {0};    /* boot state per IO pin (0/1) */
static uint8_t g_outPowerOnMask = 0;          /* bit i set = apply g_outPowerOnState[i] at boot */
static uint16_t g_inputSubMask = 0;          /* bits 0-12: subscribed input pins */
static uint16_t g_inputSubDebounceMs = CLEARAI_INPUT_DEBOUNCE_DEFAULT_MS;
static uint16_t g_inputState[13] = {0};       /* last reported digital state per pin */
static uint32_t g_inputLastEdgeMs[13] = {0}; /* last edge timestamp per pin */
static uint8_t g_netMode = 0;               /* 0 = DHCP, 1 = static */
static uint8_t g_ipOctets[4] = {0};
static uint8_t g_netmaskOctets[4] = {0};
static uint8_t g_gatewayOctets[4] = {0};

#define CLEARAI_LOG_SIZE 16
#define CLEARAI_LOG_METHOD_LEN 16
#define CLEARAI_LOG_REASON_LEN 48
struct ClearAiLogEntry {
    uint32_t ms;
    char method[CLEARAI_LOG_METHOD_LEN];
    char reason[CLEARAI_LOG_REASON_LEN];
    uint8_t kind; /* 0 = rejected move, 1 = info event */
};
static ClearAiLogEntry g_log[CLEARAI_LOG_SIZE];
static uint8_t g_logHead = 0;   /* next write slot */
static uint8_t g_logCount = 0;  /* entries present (<= CLEARAI_LOG_SIZE) */

static uint32_t g_bootMs = 0;
static uint32_t g_moveCount = 0;
static uint32_t g_movesRejected = 0;
static int32_t g_distanceSteps[CLEARAI_AXIS_COUNT] = {0};
static int32_t g_arcPathSteps = 0;

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
    for (uint8_t a = 0; a < CLEARAI_AXIS_COUNT; a++) {
        cfg->velAxis[a] = g_velAxis[a];
        cfg->accelAxis[a] = g_accelAxis[a];
        cfg->decelAxis[a] = g_decelAxis[a];
    }
    cfg->watchdogMs = g_watchdogMs;
    cfg->nvmPad2[0] = 0;
    cfg->nvmPad2[1] = 0;
    cfg->nvmPad2[2] = 0;
    cfg->nvmPad2[3] = 0;
    for (uint8_t i = 0; i < 6; i++) {
        cfg->outPowerOnState[i] = g_outPowerOnState[i];
    }
    cfg->outPowerOnMask = g_outPowerOnMask;
    cfg->nvmPad3[0] = 0;
    cfg->nvmPad3[1] = 0;
    cfg->nvmPad3[2] = 0;
    cfg->netMode = g_netMode;
    for (uint8_t i = 0; i < 4; i++) {
        cfg->ipOctets[i] = g_ipOctets[i];
        cfg->netmaskOctets[i] = g_netmaskOctets[i];
        cfg->gatewayOctets[i] = g_gatewayOctets[i];
    }
    cfg->nvmPad4[0] = 0;
    cfg->nvmPad4[1] = 0;
    cfg->nvmPad4[2] = 0;
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
        case Connector::OUTPUT_PWM:
            return "pwm";
        case Connector::INPUT_ANALOG:
            return "analog_in";
        case Connector::OUTPUT_ANALOG:
            return "analog_out";
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

static void ResetPerAxisDynamics() {
    for (uint8_t a = 0; a < CLEARAI_AXIS_COUNT; a++) {
        g_velAxis[a] = 0;
        g_accelAxis[a] = 0;
        g_decelAxis[a] = 0;
    }
    g_watchdogMs = 0;
}

static void ResetOutputDefaults() {
    for (uint8_t i = 0; i < 6; i++) {
        g_outPowerOnState[i] = 0;
    }
    g_outPowerOnMask = 0;
}

static void ResetNetworkDefaults() {
    g_netMode = 0;
    for (uint8_t i = 0; i < 4; i++) {
        g_ipOctets[i] = 0;
        g_netmaskOctets[i] = 0;
        g_gatewayOctets[i] = 0;
    }
}

static void ApplyOutputDefaults() {
    for (uint8_t i = 0; i < 6; i++) {
        if (!(g_outPowerOnMask & (1u << i))) {
            continue;
        }
        if (PinReservedForLimit(i)) {
            continue;
        }
        Connector *c = LimitInputConnector(i);
        if (c && c->Mode(Connector::OUTPUT_DIGITAL)) {
            c->State(g_outPowerOnState[i] ? 1 : 0);
        }
    }
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
        ResetPerAxisDynamics();
        ResetOutputDefaults();
        ResetNetworkDefaults();
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
        ResetPerAxisDynamics();
        ResetOutputDefaults();
        ResetNetworkDefaults();
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
        ResetPerAxisDynamics();
        ResetOutputDefaults();
        ResetNetworkDefaults();
        return true;
    }
    if (cfg->version == CLEARAI_NVM_VERSION_V4) {
        if (cfg->size < sizeof(ClearAiNvmConfigV4)) {
            return false;
        }
        const ClearAiNvmConfigV4 *v4 = (const ClearAiNvmConfigV4 *)cfg;
        if (!ConfigApplyCommon((const ClearAiNvmConfigV1 *)v4)) {
            return false;
        }
        g_limitFlags = v4->limitFlags;
        for (uint8_t a = 0; a < CLEARAI_AXIS_COUNT; a++) {
            g_limitMin[a] = (double)v4->limitMin[a];
            g_limitMax[a] = (double)v4->limitMax[a];
            g_posLimDi[a] = v4->posLimDi[a];
            g_negLimDi[a] = v4->negLimDi[a];
        }
        g_unitsA = (v4->unitsA == (uint8_t)CLEARAI_UNITS_A_REV) ? CLEARAI_UNITS_A_REV : CLEARAI_UNITS_A_DEG;
        ResetPerAxisDynamics();
        ResetOutputDefaults();
        ResetNetworkDefaults();
        return true;
    }
    if (cfg->version == CLEARAI_NVM_VERSION_V5) {
        if (cfg->size < sizeof(ClearAiNvmConfigV5)) {
            return false;
        }
        const ClearAiNvmConfigV5 *v5 = (const ClearAiNvmConfigV5 *)cfg;
        if (!ConfigApplyCommon((const ClearAiNvmConfigV1 *)v5)) {
            return false;
        }
        g_limitFlags = v5->limitFlags;
        for (uint8_t a = 0; a < CLEARAI_AXIS_COUNT; a++) {
            g_limitMin[a] = (double)v5->limitMin[a];
            g_limitMax[a] = (double)v5->limitMax[a];
            g_posLimDi[a] = v5->posLimDi[a];
            g_negLimDi[a] = v5->negLimDi[a];
        }
        g_unitsA = (v5->unitsA == (uint8_t)CLEARAI_UNITS_A_REV) ? CLEARAI_UNITS_A_REV : CLEARAI_UNITS_A_DEG;
        for (uint8_t a = 0; a < CLEARAI_AXIS_COUNT; a++) {
            g_velAxis[a] = v5->velAxis[a];
            g_accelAxis[a] = v5->accelAxis[a];
            g_decelAxis[a] = v5->decelAxis[a];
        }
        g_watchdogMs = v5->watchdogMs;
        g_lastKeepaliveMs = Milliseconds();
        ResetOutputDefaults();
        ResetNetworkDefaults();
        return true;
    }
    if (cfg->version == CLEARAI_NVM_VERSION_V6) {
        if (cfg->size < sizeof(ClearAiNvmConfigV6)) {
            return false;
        }
        const ClearAiNvmConfigV6 *v6 = (const ClearAiNvmConfigV6 *)cfg;
        if (!ConfigApplyCommon((const ClearAiNvmConfigV1 *)v6)) {
            return false;
        }
        g_limitFlags = v6->limitFlags;
        for (uint8_t a = 0; a < CLEARAI_AXIS_COUNT; a++) {
            g_limitMin[a] = (double)v6->limitMin[a];
            g_limitMax[a] = (double)v6->limitMax[a];
            g_posLimDi[a] = v6->posLimDi[a];
            g_negLimDi[a] = v6->negLimDi[a];
        }
        g_unitsA = (v6->unitsA == (uint8_t)CLEARAI_UNITS_A_REV) ? CLEARAI_UNITS_A_REV : CLEARAI_UNITS_A_DEG;
        for (uint8_t a = 0; a < CLEARAI_AXIS_COUNT; a++) {
            g_velAxis[a] = v6->velAxis[a];
            g_accelAxis[a] = v6->accelAxis[a];
            g_decelAxis[a] = v6->decelAxis[a];
        }
        g_watchdogMs = v6->watchdogMs;
        g_lastKeepaliveMs = Milliseconds();
        for (uint8_t i = 0; i < 6; i++) {
            g_outPowerOnState[i] = v6->outPowerOnState[i];
        }
        g_outPowerOnMask = v6->outPowerOnMask;
        ResetNetworkDefaults();
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
    for (uint8_t a = 0; a < CLEARAI_AXIS_COUNT; a++) {
        g_velAxis[a] = cfg->velAxis[a];
        g_accelAxis[a] = cfg->accelAxis[a];
        g_decelAxis[a] = cfg->decelAxis[a];
    }
    g_watchdogMs = cfg->watchdogMs;
    g_lastKeepaliveMs = Milliseconds();
    for (uint8_t i = 0; i < 6; i++) {
        g_outPowerOnState[i] = cfg->outPowerOnState[i];
    }
    g_outPowerOnMask = cfg->outPowerOnMask;
    g_netMode = cfg->netMode;
    for (uint8_t i = 0; i < 4; i++) {
        g_ipOctets[i] = cfg->ipOctets[i];
        g_netmaskOctets[i] = cfg->netmaskOctets[i];
        g_gatewayOctets[i] = cfg->gatewayOctets[i];
    }
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

/* Convert accumulated step counts to user units (steps -> mm/deg, then unit-convert). */
static double StepsToUserUnits(uint8_t axis, int32_t steps) {
    double spu = g_stepsPerMm[axis];
    if (spu <= 0.0) {
        return 0.0;
    }
    return FromInternal(axis, (double)steps / spu);
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

static const char *ValidateTargetSteps(uint8_t axis, int32_t steps, bool moving) {
    if (!moving || !AxisOn(axis) || g_limitFlags == 0) {
        return nullptr;
    }
    /* Soft limits are in machine (absolute) coordinates: they bound the
     * physical travel envelope and do NOT move with set_work_origin. */
    const double spu = g_stepsPerMm[axis];
    const double machine = (spu > 0.0) ? (double)steps / spu : 0.0;
    if (LimitMinEn(axis) && machine < g_limitMin[axis] - 1e-6) {
        snprintf(g_limitErrBuf, sizeof(g_limitErrBuf), "%s below min limit", AxisName(axis));
        return g_limitErrBuf;
    }
    if (LimitMaxEn(axis) && machine > g_limitMax[axis] + 1e-6) {
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

static uint32_t AxisVelCap(uint8_t a) {
    return g_velAxis[a] ? g_velAxis[a] : g_vel;
}

static uint32_t AxisAccelCap(uint8_t a) {
    return g_accelAxis[a] ? g_accelAxis[a] : g_accel;
}

static uint32_t AxisDecelCap(uint8_t a) {
    return g_decelAxis[a] ? g_decelAxis[a] : g_decel;
}

static void ApplyLimits() {
    for (uint8_t a = 0; a < CLEARAI_AXIS_COUNT; a++) {
        MotorDriver *m = MotorForAxis(a);
        if (!m) {
            continue;
        }
        m->HlfbMode(MotorDriver::HLFB_MODE_HAS_BIPOLAR_PWM);
        m->HlfbCarrier(MotorDriver::HLFB_CARRIER_482_HZ);
        m->VelMax(AxisVelCap(a));
        m->AccelMax(AxisAccelCap(a));
        m->EStopDecelMax(AxisDecelCap(a));
    }
    g_xy.ArcVelMax(g_vel);
    g_xy.ArcAccelMax(g_accel);
}

static const char *GateMotion() {
    if (g_watchdogTripped) {
        return "watchdog tripped; call keepalive";
    }
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
                /* Latch a limit-tripped fault for host diagnostics. Cleared by
                 * clear_alerts. */
                g_limitTrippedAxis = a;
                g_limitTrippedPos = posDir;
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

void MotionKeepalive() {
    g_lastKeepaliveMs = Milliseconds();
    g_watchdogTripped = false;
}

void MotionPollWatchdog() {
    if (g_watchdogMs == 0 || g_watchdogTripped) {
        return;
    }
    if ((Milliseconds() - g_lastKeepaliveMs) >= g_watchdogMs) {
        /* Host went silent mid-session: decelerate to a stop and latch. */
        g_xy.StopDecel();
        for (uint8_t a = 0; a < CLEARAI_AXIS_COUNT; a++) {
            MotorDriver *m = MotorForAxis(a);
            if (m) {
                m->MoveStopDecel(g_decel);
            }
        }
        g_watchdogTripped = true;
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
    ResetPerAxisDynamics();
    ResetOutputDefaults();
    ResetNetworkDefaults();
    g_bootMs = Milliseconds();
    g_moveCount = 0;
    g_movesRejected = 0;
    for (uint8_t a = 0; a < CLEARAI_AXIS_COUNT; a++) {
        g_distanceSteps[a] = 0;
    }
    g_arcPathSteps = 0;
    g_watchdogTripped = false;
    g_limitTrippedAxis = 0xff;
    g_limitTrippedPos = false;
    LimitClearAll();
    HwLimClearAll();
    (void)ConfigLoadNvm();

    MotorMgr.MotorModeSet(MotorManager::MOTOR_ALL, Connector::CPM_MODE_STEP_AND_DIR);
    Delay_ms(50);
    ApplyHwLimitDiModes();
    /* IO-0 DAC is initialized by the board setup; Mode(OUTPUT_ANALOG) in
     * write_analog enables it. Just apply output power-on defaults. */
    ApplyOutputDefaults();

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
    MotionKeepalive();
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
    g_limitTrippedAxis = 0xff;
    g_limitTrippedPos = false;
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
        p->hasVel || p->hasAccel || p->hasDecel || p->hasAxisMask || p->hasEstopDi6 ||
        p->hasVelX || p->hasVelY || p->hasVelZ || p->hasVelA ||
        p->hasAccelX || p->hasAccelY || p->hasAccelZ || p->hasAccelA ||
        p->hasDecelX || p->hasDecelY || p->hasDecelZ || p->hasDecelA;
    const bool limitChange = p->hasMinX || p->hasMaxX || p->hasMinY || p->hasMaxY ||
        p->hasMinZ || p->hasMaxZ || p->hasMinA || p->hasMaxA || p->hasClearLimits ||
        p->hasClearMinX || p->hasClearMaxX || p->hasClearMinY || p->hasClearMaxY ||
        p->hasClearMinZ || p->hasClearMaxZ || p->hasClearMinA || p->hasClearMaxA ||
        p->hasPosLimX || p->hasNegLimX || p->hasPosLimY || p->hasNegLimY ||
        p->hasPosLimZ || p->hasNegLimZ || p->hasPosLimA || p->hasNegLimA;
    const bool safetyChange = p->hasWatchdogMs ||
        p->hasOutPowerOn0 || p->hasOutPowerOn1 || p->hasOutPowerOn2 ||
        p->hasOutPowerOn3 || p->hasOutPowerOn4 || p->hasOutPowerOn5;
    const bool testOnly = p->hasTestMode && !mechChange && !limitChange && !safetyChange;

    if (mechChange && g_enabled) {
        return "disable before configure";
    }
    if (g_enabled && !testOnly && !limitChange && !safetyChange) {
        return "disable before configure";
    }
    if (testOnly) {
        ConfigSaveNvm();
        return nullptr;
    }
    if (p->hasWatchdogMs) {
        g_watchdogMs = p->watchdogMs;
        MotionKeepalive();  /* re-arm timer and clear any trip on change */
    }
    {
        const uint8_t *src[6] = {p->hasOutPowerOn0 ? &p->outPowerOn0 : nullptr,
                                 p->hasOutPowerOn1 ? &p->outPowerOn1 : nullptr,
                                 p->hasOutPowerOn2 ? &p->outPowerOn2 : nullptr,
                                 p->hasOutPowerOn3 ? &p->outPowerOn3 : nullptr,
                                 p->hasOutPowerOn4 ? &p->outPowerOn4 : nullptr,
                                 p->hasOutPowerOn5 ? &p->outPowerOn5 : nullptr};
        for (uint8_t i = 0; i < 6; i++) {
            if (!src[i]) {
                continue;
            }
            uint8_t v = *src[i];
            if (v == 255) {
                g_outPowerOnMask &= (uint8_t)~(1u << i);
            } else {
                g_outPowerOnState[i] = (v != 0) ? 1 : 0;
                g_outPowerOnMask |= (uint8_t)(1u << i);
            }
        }
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

    if (!mechChange && (limitChange || safetyChange)) {
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
    if (p->hasVelX) { g_velAxis[0] = p->velX; }
    if (p->hasVelY) { g_velAxis[1] = p->velY; }
    if (p->hasVelZ) { g_velAxis[2] = p->velZ; }
    if (p->hasVelA) { g_velAxis[3] = p->velA; }
    if (p->hasAccelX) { g_accelAxis[0] = p->accelX; }
    if (p->hasAccelY) { g_accelAxis[1] = p->accelY; }
    if (p->hasAccelZ) { g_accelAxis[2] = p->accelZ; }
    if (p->hasAccelA) { g_accelAxis[3] = p->accelA; }
    if (p->hasDecelX) { g_decelAxis[0] = p->decelX; }
    if (p->hasDecelY) { g_decelAxis[1] = p->decelY; }
    if (p->hasDecelZ) { g_decelAxis[2] = p->decelZ; }
    if (p->hasDecelA) { g_decelAxis[3] = p->decelA; }
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
    ResetPerAxisDynamics();
    ResetOutputDefaults();
    ResetNetworkDefaults();
    g_watchdogTripped = false;
    g_limitTrippedAxis = 0xff;
    g_limitTrippedPos = false;
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
        for (uint8_t a = 0; a < CLEARAI_AXIS_COUNT; a++) {
            MotorDriver *m = MotorForAxis(a);
            if (m) {
                m->VelMax(AxisVelCap(a));
            }
        }
        return;
    }
    g_xy.FeedRateMMPerMin(feedMm);
    for (uint8_t a = 0; a < CLEARAI_AXIS_COUNT; a++) {
        MotorDriver *m = MotorForAxis(a);
        if (!m) {
            continue;
        }
        const double spu = g_stepsPerMm[a] > 0.0 ? g_stepsPerMm[a] : 1.0;
        const double stepsPerSec = (feedMm / 60.0) * spu;
        uint32_t vel = (stepsPerSec < 1.0) ? 1u : (uint32_t)stepsPerSec;
        uint32_t cap = AxisVelCap(a);
        if (vel > cap) {
            vel = cap;
        }
        m->VelMax(vel);
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

static void AccumulateLinear(bool hasX, int32_t tx, bool hasY, int32_t ty,
                             bool hasZ, int32_t tz, bool hasA, int32_t ta) {
    if (hasX) {
        g_distanceSteps[0] += labs(tx - MachineSteps(CLEARAI_AXIS_X));
    }
    if (hasY) {
        g_distanceSteps[1] += labs(ty - MachineSteps(CLEARAI_AXIS_Y));
    }
    if (hasZ) {
        g_distanceSteps[2] += labs(tz - MachineSteps(CLEARAI_AXIS_Z));
    }
    if (hasA) {
        g_distanceSteps[3] += labs(ta - MachineSteps(CLEARAI_AXIS_A));
    }
}

static uint32_t EstimateLinearMs(bool hasX, int32_t dxSteps, bool hasY, int32_t dySteps,
                                bool hasZ, int32_t dzSteps, bool hasA, int32_t daSteps) {
    /* Rough cruise-time estimate (ignores accel/decel ramps). */
    uint32_t est = 0;
    if (hasX || hasY) {
        double dxmm = (g_stepsPerMm[0] > 0.0) ? (double)dxSteps / g_stepsPerMm[0] : 0.0;
        double dymm = (g_stepsPerMm[1] > 0.0) ? (double)dySteps / g_stepsPerMm[1] : 0.0;
        double path = sqrt(dxmm * dxmm + dymm * dymm);
        double feedMmPerSec = g_feedMmPerMin / 60.0;
        if (feedMmPerSec > 0.0 && path > 0.0) {
            uint32_t t = (uint32_t)(path / feedMmPerSec * 1000.0);
            if (t > est) { est = t; }
        }
    }
    if (hasZ && g_stepsPerMm[2] > 0.0) {
        double dzmm = fabs((double)dzSteps) / g_stepsPerMm[2];
        double vMmPerSec = (double)g_velAxis[2] / g_stepsPerMm[2];
        if (vMmPerSec > 0.0 && dzmm > 0.0) {
            uint32_t t = (uint32_t)(dzmm / vMmPerSec * 1000.0);
            if (t > est) { est = t; }
        }
    }
    if (hasA && g_stepsPerMm[3] > 0.0) {
        double dadeg = fabs((double)daSteps) / g_stepsPerMm[3];
        double vDegPerSec = (double)g_velAxis[3] / g_stepsPerMm[3];
        if (vDegPerSec > 0.0 && dadeg > 0.0) {
            uint32_t t = (uint32_t)(dadeg / vDegPerSec * 1000.0);
            if (t > est) { est = t; }
        }
    }
    return est;
}

const char *MotionMoveLinear(const RpcParams *p, uint32_t *estMsOut) {
    const char *err = GateMotion();
    if (err) {
        return err;
    }
    const bool rel = (g_mode == CLEARAI_MODE_REL);
    const int32_t sx = MachineSteps(CLEARAI_AXIS_X);
    const int32_t sy = MachineSteps(CLEARAI_AXIS_Y);
    const int32_t sz = MachineSteps(CLEARAI_AXIS_Z);
    const int32_t sa = MachineSteps(CLEARAI_AXIS_A);
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
    AccumulateLinear(p->hasX, tx, p->hasY, ty, p->hasZ, tz, p->hasA, ta);
    g_moveCount++;
    if (estMsOut) {
        *estMsOut = EstimateLinearMs(p->hasX, tx - sx, p->hasY, ty - sy,
                                    p->hasZ, tz - sz, p->hasA, ta - sa);
    }
    return nullptr;
}

const char *MotionMoveArc(const RpcParams *p, uint32_t *estMsOut) {
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
    double swept = endAngle - startAngle;
    if (swept < 0) {
        swept += 2.0 * 3.14159265358979323846;
    }
    swept = clockwise ? (2.0 * 3.14159265358979323846 - swept) : swept;
    if (swept < 1e-6) {
        swept = 2.0 * 3.14159265358979323846; /* full circle */
    }
    g_arcPathSteps += I32Round(radius * swept);
    g_moveCount++;
    if (estMsOut) {
        double radiusMm = (g_stepsPerMm[0] > 0.0) ? radius / g_stepsPerMm[0] : 0.0;
        double arcLenMm = radiusMm * swept;
        double feedMmPerSec = g_feedMmPerMin / 60.0;
        if (feedMmPerSec > 0.0 && arcLenMm > 0.0) {
            *estMsOut = (uint32_t)(arcLenMm / feedMmPerSec * 1000.0);
        }
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
    AccumulateLinear(p->hasX, tx, p->hasY, ty, p->hasZ, tz, p->hasA, ta);
    g_moveCount++;
    return nullptr;
}

const char *MotionJogVelocity(const RpcParams *p) {
    const char *err = GateMotion();
    if (err) {
        return err;
    }
    /* Per-axis continuous velocity. The coordinated XY controller has no
     * velocity mode, so X/Y are driven as independent motors. Hardware
     * limit switches auto-stop a velocity move; soft limits are NOT
     * enforced in velocity mode. */
    if (p->hasX && AxisOn(CLEARAI_AXIS_X)) {
        MotorDriver *m = MotorForAxis(CLEARAI_AXIS_X);
        if (m) {
            m->MoveVelocity(I32Round(ToInternal(CLEARAI_AXIS_X, p->x) * g_stepsPerMm[0]));
        }
    }
    if (p->hasY && AxisOn(CLEARAI_AXIS_Y)) {
        MotorDriver *m = MotorForAxis(CLEARAI_AXIS_Y);
        if (m) {
            m->MoveVelocity(I32Round(ToInternal(CLEARAI_AXIS_Y, p->y) * g_stepsPerMm[1]));
        }
    }
    if (p->hasZ && AxisOn(CLEARAI_AXIS_Z)) {
        MotorDriver *m = MotorForAxis(CLEARAI_AXIS_Z);
        if (m) {
            m->MoveVelocity(I32Round(ToInternal(CLEARAI_AXIS_Z, p->z) * g_stepsPerMm[2]));
        }
    }
    if (p->hasA && AxisOn(CLEARAI_AXIS_A)) {
        MotorDriver *m = MotorForAxis(CLEARAI_AXIS_A);
        if (m) {
            m->MoveVelocity(I32Round(ToInternal(CLEARAI_AXIS_A, p->a) * g_stepsPerMm[3]));
        }
    }
    return nullptr;
}

void MotionJogStop() {
    g_xy.StopDecel();
    for (uint8_t a = 0; a < CLEARAI_AXIS_COUNT; a++) {
        MotorDriver *m = MotorForAxis(a);
        if (m) {
            m->MoveStopDecel(g_decel);
        }
    }
}

void MotionLog(const char *method, const char *reason, uint8_t kind) {
    ClearAiLogEntry *e = &g_log[g_logHead];
    e->ms = Milliseconds();
    strncpy(e->method, method ? method : "", sizeof(e->method) - 1);
    e->method[sizeof(e->method) - 1] = '\0';
    strncpy(e->reason, reason ? reason : "", sizeof(e->reason) - 1);
    e->reason[sizeof(e->reason) - 1] = '\0';
    e->kind = kind;
    g_logHead = (g_logHead + 1) % CLEARAI_LOG_SIZE;
    if (g_logCount < CLEARAI_LOG_SIZE) {
        g_logCount++;
    }
}

void MotionGetLog(char *buf, uint16_t bufLen) {
    size_t pos = 0;
    if (pos + 1 < bufLen) {
        buf[pos++] = '[';
    }
    /* Most-recent first: walk backwards from the last written slot. */
    for (uint8_t i = 0; i < g_logCount; i++) {
        uint8_t idx = (g_logHead + CLEARAI_LOG_SIZE - 1 - i) % CLEARAI_LOG_SIZE;
        ClearAiLogEntry *e = &g_log[idx];
        char entry[96];
        int n = snprintf(entry, sizeof(entry),
                          "%s{\"ms\":%lu,\"method\":\"%s\",\"reason\":\"%s\",\"kind\":%u}",
                          (i == 0) ? "" : ",",
                          (unsigned long)e->ms, e->method, e->reason, (unsigned)e->kind);
        if (n < 0 || pos + (size_t)n + 1 >= bufLen) {
            break;
        }
        memcpy(buf + pos, entry, (size_t)n);
        pos += (size_t)n;
    }
    if (pos + 1 < bufLen) {
        buf[pos++] = ']';
    }
    if (pos < bufLen) {
        buf[pos] = '\0';
    } else {
        buf[bufLen - 1] = '\0';
    }
}

void MotionClearLog() {
    g_logHead = 0;
    g_logCount = 0;
}

void MotionIncMovesRejected() {
    g_movesRejected++;
}

const char *MotionMoveBatch(const RpcParams *p, uint16_t *acceptedOut) {
    if (!p->hasMoves || p->movesCount == 0) {
        return "moves array required";
    }
    for (uint8_t i = 0; i < p->movesCount; i++) {
        const RpcParams *sub = &p->moves[i];
        const char *err = nullptr;
        if (sub->hasI && sub->hasJ) {
            err = MotionMoveArc(sub, nullptr);
        } else if (sub->hasSeconds) {
            err = MotionDwell(sub, nullptr);
        } else if (sub->hasX || sub->hasY || sub->hasZ || sub->hasA) {
            err = MotionMoveLinear(sub, nullptr);
        } else {
            err = "empty move in batch";
        }
        if (err) {
            *acceptedOut = i;
            g_movesRejected++;
            return err;
        }
    }
    *acceptedOut = p->movesCount;
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

static void DecodeAlerts(uint32_t reg, char *out, size_t outLen) {
    /* AlertRegMotor bits (MotorDriver.h): step/direction mode has 6 bits. */
    static const char *const kNames[] = {
        "motion_canceled_in_alert",
        "motion_canceled_positive_limit",
        "motion_canceled_negative_limit",
        "motion_canceled_sensor_estop",
        "motion_canceled_motor_disabled",
        "motor_faulted",
    };
    size_t pos = 0;
    out[0] = '\0';
    for (uint8_t b = 0; b < 6; b++) {
        if (!(reg & (1u << b))) {
            continue;
        }
        const char *name = kNames[b];
        size_t need = (pos == 0) ? strlen(name) + 2 : strlen(name) + 4;
        if (pos + need + 1 >= outLen) {
            break;
        }
        if (pos > 0) {
            out[pos++] = ',';
        }
        out[pos++] = '"';
        size_t n = strlen(name);
        memcpy(out + pos, name, n);
        pos += n;
        out[pos++] = '"';
        out[pos] = '\0';
    }
}

void MotionGetNetworkConfig(uint8_t *mode, uint8_t ip[4], uint8_t netmask[4],
                            uint8_t gateway[4]) {
    if (mode) {
        *mode = g_netMode;
    }
    if (ip) {
        for (uint8_t i = 0; i < 4; i++) {
            ip[i] = g_ipOctets[i];
        }
    }
    if (netmask) {
        for (uint8_t i = 0; i < 4; i++) {
            netmask[i] = g_netmaskOctets[i];
        }
    }
    if (gateway) {
        for (uint8_t i = 0; i < 4; i++) {
            gateway[i] = g_gatewayOctets[i];
        }
    }
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
    out->watchdogMs = g_watchdogMs;
    out->watchdogTripped = g_watchdogTripped;
    out->limitTripped = (g_limitTrippedAxis != 0xff);
    out->limitTrippedAxis = g_limitTrippedAxis;
    out->limitTrippedPos = g_limitTrippedPos;
    for (uint8_t a = 0; a < CLEARAI_AXIS_COUNT; a++) {
        MotorDriver *m = MotorForAxis(a);
        bool active = (g_axisMask >> a) & 1u;
        if (m) {
            uint32_t reg = active ? m->AlertReg().reg : 0u;
            out->alertReg |= reg;
            out->alertRegAxis[a] = reg;
            out->hlfbPercent[a] = m->HlfbPercent();
        } else {
            out->hlfbPercent[a] = MotorDriver::HLFB_DUTY_UNKNOWN;
        }
        out->work[a] = FromInternal(a, WorkInternal(a));
    }
    DecodeAlerts(out->alertReg, out->alertsDecoded, sizeof(out->alertsDecoded));
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

static DigitalInAnalogIn *AnalogInConnector(uint8_t pinIndex) {
    switch (pinIndex) {
        case 9:  return &ConnectorA9;
        case 10: return &ConnectorA10;
        case 11: return &ConnectorA11;
        case 12: return &ConnectorA12;
        default: return nullptr;
    }
}

static DigitalInOut *PwmOutConnector(uint8_t pinIndex) {
    switch (pinIndex) {
        case 0: return &ConnectorIO0;
        case 1: return &ConnectorIO1;
        case 2: return &ConnectorIO2;
        case 3: return &ConnectorIO3;
        case 4: return &ConnectorIO4;
        case 5: return &ConnectorIO5;
        default: return nullptr;
    }
}

const char *MotionReadAnalog(const RpcParams *p, char *buf, uint16_t bufLen) {
    uint8_t startPin = 9;
    uint8_t endPin = 12;
    if (p->hasPin) {
        if (p->pin < 9 || p->pin > 12) {
            return "pin must be 9-12";
        }
        startPin = (uint8_t)p->pin;
        endPin = startPin;
    }
    int pos = snprintf(buf, bufLen, "{\"pins\":[");
    if (pos < 0 || (uint16_t)pos >= bufLen) {
        buf[0] = '\0';
        return nullptr;
    }
    bool first = true;
    for (uint8_t pin = startPin; pin <= endPin; pin++) {
        if (PinReservedForLimit(pin)) {
            continue;
        }
        DigitalInAnalogIn *a = AnalogInConnector(pin);
        if (!a) {
            continue;
        }
        a->Mode(Connector::INPUT_ANALOG);
        const float volts = a->AnalogVoltage();
        const int16_t raw = a->State();
        const int n = snprintf(buf + pos, (size_t)(bufLen - (uint16_t)pos),
                               "%s{\"pin\":%u,\"volts\":%.3f,\"raw\":%d}",
                               first ? "" : ",", (unsigned)pin, (double)volts, (int)raw);
        if (n < 0) {
            break;
        }
        pos += n;
        first = false;
        if ((uint16_t)pos >= bufLen) {
            break;
        }
    }
    if ((uint16_t)pos + 2 < bufLen) {
        snprintf(buf + pos, (size_t)(bufLen - (uint16_t)pos), "]}");
    }
    return nullptr;
}

const char *MotionWriteAnalog(const RpcParams *p) {
    if (!p->hasPin) {
        return "pin required";
    }
    if (p->pin != 0) {
        return "analog output is pin 0 only";
    }
    if (PinReservedForLimit(0)) {
        return "pin reserved for limit";
    }
    if (p->hasMicroamps) {
        if (!ConnectorIO0.Mode(Connector::OUTPUT_ANALOG)) {
            return "analog mode rejected";
        }
        ConnectorIO0.OutputCurrent((uint16_t)p->microamps);
        return nullptr;
    }
    if (!p->hasAnalogValue) {
        return "value or microamps required";
    }
    if (p->analogValue > 2047) {
        return "value must be 0-2047";
    }
    if (!ConnectorIO0.Mode(Connector::OUTPUT_ANALOG)) {
        return "analog mode rejected";
    }
    ConnectorIO0.AnalogWrite((uint16_t)p->analogValue);
    return nullptr;
}

const char *MotionWritePwm(const RpcParams *p) {
    if (!p->hasPin) {
        return "pin required";
    }
    if (!p->hasDuty) {
        return "duty required";
    }
    if (p->pin > 5) {
        return "pin not output capable";
    }
    if (PinReservedForLimit((uint8_t)p->pin)) {
        return "pin reserved for limit";
    }
    DigitalInOut *connector = PwmOutConnector((uint8_t)p->pin);
    if (!connector) {
        return "pin missing";
    }
    if (!connector->Mode(Connector::OUTPUT_PWM)) {
        return "pwm mode rejected";
    }
    if (!connector->PwmDuty((uint8_t)p->duty)) {
        return "pwm rejected";
    }
    g_pwmDuty[(uint8_t)p->pin] = (uint8_t)p->duty;
    return nullptr;
}

const char *MotionSubscribeInputs(const RpcParams *p, char *buf, uint16_t bufLen) {
    if (!p->hasPins || p->pinsCount == 0) {
        g_inputSubMask = 0;
        snprintf(buf, bufLen, "{\"subscribed\":[]}");
        return nullptr;
    }
    uint16_t mask = 0;
    int pos = snprintf(buf, bufLen, "{\"subscribed\":[");
    if (pos < 0 || (uint16_t)pos >= bufLen) {
        buf[0] = '\0';
        return nullptr;
    }
    bool first = true;
    for (uint8_t i = 0; i < p->pinsCount; i++) {
        uint8_t pin = p->pins[i];
        if (pin > 12) {
            return "pin must be 0-12";
        }
        mask |= (uint16_t)(1u << pin);
        const int n = snprintf(buf + pos, (size_t)(bufLen - (uint16_t)pos),
                               "%s%u", first ? "" : ",", (unsigned)pin);
        if (n < 0) {
            break;
        }
        pos += n;
        first = false;
        if ((uint16_t)pos >= bufLen) {
            break;
        }
    }
    if ((uint16_t)pos + 2 < bufLen) {
        snprintf(buf + pos, (size_t)(bufLen - (uint16_t)pos), "]}");
    }
    g_inputSubMask = mask;
    if (p->hasDebounceMs) {
        g_inputSubDebounceMs = p->debounceMs;
    }
    /* Seed current state so we only emit edges after subscription. */
    for (uint8_t pin = 0; pin <= 12; pin++) {
        if (mask & (1u << pin)) {
            Connector *c = LimitInputConnector(pin);
            g_inputState[pin] = c ? ((c->State() != 0) ? 1 : 0) : 0;
            g_inputLastEdgeMs[pin] = Milliseconds();
        }
    }
    return nullptr;
}

const char *MotionUnsubscribeInputs() {
    g_inputSubMask = 0;
    return nullptr;
}

void MotionPollInputs() {
    if (g_inputSubMask == 0) {
        return;
    }
    const uint32_t now = Milliseconds();
    for (uint8_t pin = 0; pin <= 12; pin++) {
        if (!(g_inputSubMask & (1u << pin))) {
            continue;
        }
        Connector *c = LimitInputConnector(pin);
        if (!c) {
            continue;
        }
        const uint16_t state = (c->State() != 0) ? 1 : 0;
        if (state == g_inputState[pin]) {
            continue;
        }
        if ((now - g_inputLastEdgeMs[pin]) < g_inputSubDebounceMs) {
            continue;
        }
        g_inputState[pin] = state;
        g_inputLastEdgeMs[pin] = now;
        char line[64];
        snprintf(line, sizeof(line),
                 "{\"pin\":%u,\"state\":%u,\"edge\":\"%s\"}",
                 (unsigned)pin, (unsigned)state, state ? "rising" : "falling");
        char notif[160];
        JsonRpcFormatNotification(notif, sizeof(notif), "input_changed", line);
        TransportSendTelemetryLine(notif);
    }
}

static bool ParseIpOctets(const char *str, uint8_t out[4]) {
    uint8_t parts = 0;
    uint16_t acc = 0;
    bool any = false;
    for (const char *s = str; ; s++) {
        char c = *s;
        if (c >= '0' && c <= '9') {
            acc = acc * 10 + (uint16_t)(c - '0');
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

const char *MotionConfigureNetwork(const RpcParams *p, char *buf, uint16_t bufLen) {
    if (p->hasMode) {
        if (strcmp(p->mode, "dhcp") == 0) {
            g_netMode = 0;
        } else if (strcmp(p->mode, "static") == 0) {
            g_netMode = 1;
        } else {
            return "mode must be 'dhcp' or 'static'";
        }
    }
    if (g_netMode == 1) {
        if (p->hasIpAddress) {
            if (!ParseIpOctets(p->ipAddress, g_ipOctets)) {
                return "invalid ip_address";
            }
        } else if (!(g_ipOctets[0] | g_ipOctets[1] | g_ipOctets[2] | g_ipOctets[3])) {
            return "static mode requires ip_address";
        }
        if (p->hasNetmask) {
            if (!ParseIpOctets(p->netmask, g_netmaskOctets)) {
                return "invalid netmask";
            }
        }
        if (p->hasGateway) {
            if (!ParseIpOctets(p->gateway, g_gatewayOctets)) {
                return "invalid gateway";
            }
        }
    }
    ConfigSaveNvm();
    snprintf(buf, bufLen,
             "{\"network_mode\":\"%s\",\"ip_address\":\"%u.%u.%u.%u\","
             "\"netmask\":\"%u.%u.%u.%u\",\"gateway\":\"%u.%u.%u.%u\","
             "\"applies_on\":\"restart\"}",
             g_netMode == 1 ? "static" : "dhcp",
             (unsigned)g_ipOctets[0], (unsigned)g_ipOctets[1],
             (unsigned)g_ipOctets[2], (unsigned)g_ipOctets[3],
             (unsigned)g_netmaskOctets[0], (unsigned)g_netmaskOctets[1],
             (unsigned)g_netmaskOctets[2], (unsigned)g_netmaskOctets[3],
             (unsigned)g_gatewayOctets[0], (unsigned)g_gatewayOctets[1],
             (unsigned)g_gatewayOctets[2], (unsigned)g_gatewayOctets[3]);
    return nullptr;
}

void MotionRestart() {
    SysMgr.ResetBoard();
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
             "\"home\",\"probe\",\"keepalive\","
             "\"read_analog\",\"write_analog\",\"write_pwm\","
             "\"subscribe_inputs\",\"unsubscribe_inputs\","
             "\"configure_network\",\"restart\","
             "\"jog_velocity\",\"jog_stop\",\"move_batch\","
             "\"get_log\",\"clear_log\"],"
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
                 stored.version == CLEARAI_NVM_VERSION_V6 ||
                 stored.version == CLEARAI_NVM_VERSION_V5 ||
                 stored.version == CLEARAI_NVM_VERSION_V4 ||
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
             "\"neg_lim_di\":[%u,%u,%u,%u],"
             "\"vel_axis\":[%lu,%lu,%lu,%lu],"
             "\"accel_axis\":[%lu,%lu,%lu,%lu],"
             "\"decel_axis\":[%lu,%lu,%lu,%lu],"
             "\"watchdog_ms\":%lu,"
             "\"out_power_on_state\":[%u,%u,%u,%u,%u,%u],"
             "\"out_power_on_mask\":%u,"
             "\"network_mode\":\"%s\","
             "\"ip_address\":\"%u.%u.%u.%u\","
             "\"netmask\":\"%u.%u.%u.%u\","
             "\"gateway\":\"%u.%u.%u.%u\"}",
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
             negLimDi[0], negLimDi[1], negLimDi[2], negLimDi[3],
             (unsigned long)g_velAxis[0], (unsigned long)g_velAxis[1],
             (unsigned long)g_velAxis[2], (unsigned long)g_velAxis[3],
             (unsigned long)g_accelAxis[0], (unsigned long)g_accelAxis[1],
             (unsigned long)g_accelAxis[2], (unsigned long)g_accelAxis[3],
             (unsigned long)g_decelAxis[0], (unsigned long)g_decelAxis[1],
             (unsigned long)g_decelAxis[2], (unsigned long)g_decelAxis[3],
             (unsigned long)g_watchdogMs,
             (unsigned)g_outPowerOnState[0], (unsigned)g_outPowerOnState[1],
             (unsigned)g_outPowerOnState[2], (unsigned)g_outPowerOnState[3],
             (unsigned)g_outPowerOnState[4], (unsigned)g_outPowerOnState[5],
             (unsigned)g_outPowerOnMask,
             g_netMode == 1 ? "static" : "dhcp",
             (unsigned)g_ipOctets[0], (unsigned)g_ipOctets[1],
             (unsigned)g_ipOctets[2], (unsigned)g_ipOctets[3],
             (unsigned)g_netmaskOctets[0], (unsigned)g_netmaskOctets[1],
             (unsigned)g_netmaskOctets[2], (unsigned)g_netmaskOctets[3],
             (unsigned)g_gatewayOctets[0], (unsigned)g_gatewayOctets[1],
             (unsigned)g_gatewayOctets[2], (unsigned)g_gatewayOctets[3]);
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
        char limitStatus[48];
        if (st.limitTripped) {
            snprintf(limitStatus, sizeof(limitStatus),
                      "{\"tripped\":true,\"axis\":\"%s\",\"dir\":\"%s\"}",
                      AxisName(st.limitTrippedAxis),
                      st.limitTrippedPos ? "pos" : "neg");
        } else {
            snprintf(limitStatus, sizeof(limitStatus),
                      "{\"tripped\":false}");
        }
        char pwmDuty[40];
        snprintf(pwmDuty, sizeof(pwmDuty), "[%u,%u,%u,%u,%u,%u]",
                 (unsigned)g_pwmDuty[0], (unsigned)g_pwmDuty[1],
                 (unsigned)g_pwmDuty[2], (unsigned)g_pwmDuty[3],
                 (unsigned)g_pwmDuty[4], (unsigned)g_pwmDuty[5]);
        snprintf(buf, bufLen,
                 "{\"enabled\":%s,\"moving\":%s,\"hlfb\":%s,\"estop\":%s,\"hw_estop\":%s,"
                 "\"test_mode\":%s,\"alerts\":%s,"
                 "\"queue\":%u,\"queue_active\":%s,\"alert_reg\":%lu,"
                 "\"alert_reg_axis\":[%lu,%lu,%lu,%lu],\"alerts_decoded\":[%s],"
                 "\"units\":\"%s\",\"units_a\":\"%s\",\"mode\":\"%s\","
                 "\"axis_mask\":%lu,\"x\":%.4f,\"y\":%.4f,\"z\":%.4f,\"a\":%.4f,"
                 "\"hlfb_percent\":%s,"
                 "\"watchdog_ms\":%lu,\"watchdog_tripped\":%s,\"limit_status\":%s,"
                 "\"pwm_duty\":%s,"
                 "\"uptime_ms\":%lu,\"moves\":%lu,\"moves_rejected\":%lu,"
                 "\"distance\":{\"x\":%.3f,\"y\":%.3f,\"z\":%.3f,\"a\":%.3f},"
                 "\"arc_path\":%.3f}",
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
                 (unsigned long)st.alertRegAxis[0], (unsigned long)st.alertRegAxis[1],
                 (unsigned long)st.alertRegAxis[2], (unsigned long)st.alertRegAxis[3],
                 st.alertsDecoded,
                 st.units == CLEARAI_UNITS_MM ? "mm" : "inch",
                 st.unitsA == CLEARAI_UNITS_A_REV ? "rev" : "deg",
                 st.mode == CLEARAI_MODE_ABS ? "abs" : "rel",
                 (unsigned long)st.axisMask,
                 st.work[0], st.work[1], st.work[2], st.work[3],
                 hlfbArr,
                 (unsigned long)st.watchdogMs,
                 st.watchdogTripped ? "true" : "false",
                 limitStatus,
                 pwmDuty,
                 (unsigned long)(Milliseconds() - g_bootMs),
                 (unsigned long)g_moveCount,
                 (unsigned long)g_movesRejected,
                 StepsToUserUnits(CLEARAI_AXIS_X, g_distanceSteps[0]),
                 StepsToUserUnits(CLEARAI_AXIS_Y, g_distanceSteps[1]),
                 StepsToUserUnits(CLEARAI_AXIS_Z, g_distanceSteps[2]),
                 StepsToUserUnits(CLEARAI_AXIS_A, g_distanceSteps[3]),
                 StepsToUserUnits(CLEARAI_AXIS_X, g_arcPathSteps));
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
