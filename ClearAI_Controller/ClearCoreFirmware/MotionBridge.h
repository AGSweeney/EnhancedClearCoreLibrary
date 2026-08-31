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

#ifndef __MOTION_BRIDGE_H__
#define __MOTION_BRIDGE_H__

#include <stdint.h>
#include "ClearAI_Config.h"
#include "JsonRpcLine.h"

typedef bool (*ClearAiUrgentFn)();

struct MotionStatus {
    bool enabled;
    bool moving;
    bool hlfb;
    bool estop;
    bool alerts;
    uint8_t queue;
    uint32_t alertReg;
    double work[CLEARAI_AXIS_COUNT];
    ClearAiUnits units;
    ClearAiUnitsA unitsA;
    ClearAiMode mode;
    uint32_t axisMask;
    bool testMode;
    bool hwEstop;
    uint32_t vel;
    uint32_t accel;
    float hlfbPercent[CLEARAI_AXIS_COUNT]; /* -100..100, or HLFB_DUTY_UNKNOWN */
    bool queueActive;
    uint32_t watchdogMs;
    bool watchdogTripped;
    bool limitTripped;
    uint8_t limitTrippedAxis; /* 0xff = none */
    bool limitTrippedPos;
};

bool MotionInit();
void MotionPollEstop();
bool MotionIsEnabled();
bool MotionIsMoving();
bool MotionInterrupted();
void MotionClearInterrupt();
void MotionKeepalive();
void MotionPollWatchdog();

const char *MotionEnable();
const char *MotionDisable();
const char *MotionClearAlerts();
const char *MotionStop();
const char *MotionEstop();
const char *MotionConfigure(const RpcParams *p);
const char *MotionSetTestMode(const RpcParams *p);
const char *MotionSetUnits(const RpcParams *p);
const char *MotionSetUnitsA(const RpcParams *p);
const char *MotionSetMode(const RpcParams *p);
const char *MotionSetWorkOrigin(const RpcParams *p);
const char *MotionResetConfig();
void MotionFillConfigJson(char *buf, uint16_t bufLen);
const char *MotionMoveLinear(const RpcParams *p);
const char *MotionMoveArc(const RpcParams *p);
const char *MotionJog(const RpcParams *p);
const char *MotionDwell(const RpcParams *p, ClearAiUrgentFn urgent);
const char *MotionWaitIdle(const RpcParams *p, ClearAiUrgentFn urgent);
const char *MotionHome(const RpcParams *p, char *body, uint16_t bufLen, ClearAiUrgentFn urgent);
const char *MotionProbe(const RpcParams *p, char *body, uint16_t bufLen, ClearAiUrgentFn urgent);
const char *MotionQueueClear();
void MotionFillQueueStatusJson(char *buf, uint16_t bufLen);

void MotionGetStatus(MotionStatus *out);
void MotionFillCapabilitiesJson(char *buf, uint16_t bufLen);
void MotionFillStatusJson(char *buf, uint16_t bufLen);
void MotionFillPoseJson(char *buf, uint16_t bufLen);
void MotionFillTelemetryJson(char *buf, uint16_t bufLen);

const char *MotionReadInputs(const RpcParams *p, char *buf, uint16_t bufLen);
const char *MotionWriteOutput(const RpcParams *p);
const char *MotionReadAnalog(const RpcParams *p, char *buf, uint16_t bufLen);
const char *MotionWriteAnalog(const RpcParams *p);
const char *MotionWritePwm(const RpcParams *p);
const char *MotionSubscribeInputs(const RpcParams *p, char *buf, uint16_t bufLen);
const char *MotionUnsubscribeInputs();
void MotionPollInputs();

#endif
