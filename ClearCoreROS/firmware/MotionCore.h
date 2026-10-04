/* MIT License. Copyright (c) 2026 Adam G. Sweeney <agsweeney@gmail.com> */

#ifndef __CCROS_MOTION_CORE_H__
#define __CCROS_MOTION_CORE_H__

#include <stdbool.h>
#include <stdint.h>
#include "RosConfig.h"
#include "RosProtocol.h"

struct MotionConfigPatch {
    bool hasAxisMask;
    uint32_t axisMask;
    bool hasSteps;
    uint32_t stepsPerRev[CCROS_AXIS_COUNT];
    bool hasPitch;
    double pitchMm[CCROS_AXIS_COUNT];
    bool hasVel;
    uint32_t vel;
    bool hasAccel;
    uint32_t accel;
    bool hasDecel;
    uint32_t decel;
    bool hasWatchdog;
    uint32_t watchdogMs;
    bool hasEstop;
    uint8_t estopDi6;
};

bool MotionInit();
void MotionPoll();

bool MotionIsEnabled();
const char *MotionConfigure(const MotionConfigPatch *patch);
const char *MotionSetTestMode(bool on);
const char *MotionEnable();
const char *MotionDisable();
const char *MotionStop();
const char *MotionEstop();
const char *MotionClearAlerts();
void MotionKeepalive();
void MotionNoteHost();
void MotionStreamLost();

/* Absolute joint targets. X/Y/Z are meters, A is radians. Ignored while disabled. */
const char *MotionSetJoints(uint8_t mask, const float q[4]);
void MotionNotePosition(uint16_t seq, uint8_t mask, const float q[4]);
void MotionNoteVelocity(uint16_t seq, uint8_t mask, const float v[4]);

void MotionFillState(CcrosState *out);
void MotionFillCapabilitiesJson(char *buf, uint16_t len);
void MotionFillConfigJson(char *buf, uint16_t len);
void MotionFillStatusJson(char *buf, uint16_t len);

#endif
