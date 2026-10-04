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
    bool hasLimitMin[CCROS_AXIS_COUNT];
    double limitMin[CCROS_AXIS_COUNT];
    bool hasLimitMax[CCROS_AXIS_COUNT];
    double limitMax[CCROS_AXIS_COUNT];
    bool hasClearMin[CCROS_AXIS_COUNT];
    bool hasClearMax[CCROS_AXIS_COUNT];
    bool hasClearLimits;
    bool clearLimits;
    bool hasPosLim[CCROS_AXIS_COUNT];
    uint8_t posLim[CCROS_AXIS_COUNT];
    bool hasNegLim[CCROS_AXIS_COUNT];
    uint8_t negLim[CCROS_AXIS_COUNT];
    bool hasName[CCROS_AXIS_COUNT];
    char name[CCROS_AXIS_COUNT][16];
    bool hasRotary[CCROS_AXIS_COUNT];
    bool rotary[CCROS_AXIS_COUNT];
    bool hasDirection[CCROS_AXIS_COUNT];
    int8_t direction[CCROS_AXIS_COUNT];
    bool hasGear[CCROS_AXIS_COUNT];
    double gear[CCROS_AXIS_COUNT];
    bool hasOffset[CCROS_AXIS_COUNT];
    double offset[CCROS_AXIS_COUNT];
};

const char *MotionJointName(uint8_t axis);
bool MotionAxisRotary(uint8_t axis);

bool MotionInit();
void MotionPoll();

bool MotionIsEnabled();
const char *MotionConfigure(const MotionConfigPatch *patch);
const char *MotionResetConfig();
const char *MotionConfigureNetwork(const char *mode, bool hasMode, const char *ipAddress,
                                   bool hasIp, const char *netmask, bool hasNetmask,
                                   const char *gateway, bool hasGateway, char *buf,
                                   uint16_t bufLen);
void MotionRestart();
void MotionGetNetworkConfig(uint8_t *mode, uint8_t ip[4], uint8_t netmask[4], uint8_t gateway[4]);
const char *MotionSetTestMode(bool on);
const char *MotionEnable();
const char *MotionDisable();
const char *MotionStop();
const char *MotionEstop();
const char *MotionClearAlerts();
const char *MotionKeepalive();
void MotionNoteHost();
void MotionStreamLost();

/* Absolute joint targets. X/Y/Z are meters, A is radians. Ignored while disabled. */
const char *MotionSetJoints(uint8_t mask, const float q[4]);
void MotionNotePosition(uint16_t seq, uint8_t mask, const float q[4]);
void MotionNoteVelocity(uint16_t seq, uint8_t mask, const float v[4]);
void MotionNoteTrack(uint16_t seq, uint8_t mask, const float q[4], const float v[4]);

const char *MotionMoveLinear(uint8_t mask, const float q[4], bool hasFeed, double feedMps, char *buf, uint16_t len);
const char *MotionMoveArc(bool hasX, float x, bool hasY, float y, float iOff, float jOff, bool clockwise,
                          bool hasFeed, double feedMps, char *buf, uint16_t len);
const char *MotionWaitIdle(uint32_t timeoutMs, char *buf, uint16_t len);
const char *MotionHome(const char *axisName, const char *dir, bool hasSeek, double seek, bool hasBackoff,
                       double backoff, bool hasTimeout, uint32_t timeoutMs, bool hasZero, bool zeroOn,
                       char *buf, uint16_t len);
const char *MotionProbe(const char *axisName, const char *dir, uint8_t pin, bool activeHigh, bool hasSeek,
                        double seek, bool hasBackoff, double backoff, bool hasTimeout, uint32_t timeoutMs,
                        bool hasZero, bool zeroOn, char *buf, uint16_t len);

void MotionFillState(CcrosState *out);
void MotionFillCapabilitiesJson(char *buf, uint16_t len);
void MotionFillConfigJson(char *buf, uint16_t len);
void MotionFillStatusJson(char *buf, uint16_t len);

#endif
