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

#ifndef __JSON_RPC_LINE_H__
#define __JSON_RPC_LINE_H__

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "ClearAI_Config.h"

#define JSONRPC_PARSE_ERROR -32700
#define JSONRPC_INVALID_REQUEST -32600
#define JSONRPC_METHOD_NOT_FOUND -32601
#define JSONRPC_INVALID_PARAMS -32602
#define JSONRPC_APP_ERROR -32000

struct RpcId {
    bool present;
    bool isString;
    char text[40];
};

struct RpcParams {
    bool hasX, hasY, hasZ, hasA;
    bool hasI, hasJ, hasFeed, hasTimeoutMs, hasSeconds;
    bool hasClockwise, hasRapid, hasUnits, hasMode;
    bool hasVel, hasAccel, hasDecel, hasAxisMask, hasEstopDi6;
    bool hasTestMode, hasEnabled;
    bool hasStepsX, hasStepsY, hasStepsZ, hasStepsA;
    bool hasPitchX, hasPitchY, hasPitchZ, hasPitchA;
    bool hasGearX, hasGearY, hasGearZ, hasGearA;
    bool hasMinX, hasMaxX, hasMinY, hasMaxY, hasMinZ, hasMaxZ, hasMinA, hasMaxA;
    bool hasClearLimits;
    bool hasClearMinX, hasClearMaxX, hasClearMinY, hasClearMaxY;
    bool hasClearMinZ, hasClearMaxZ, hasClearMinA, hasClearMaxA;
    bool hasPosLimX, hasNegLimX, hasPosLimY, hasNegLimY;
    bool hasPosLimZ, hasNegLimZ, hasPosLimA, hasNegLimA;
    bool hasPin, hasIoState;
    bool hasAxis, hasDir, hasSeek, hasBackoff, hasZero, hasActive;
    double x, y, z, a;
    double i, j, feed, seconds;
    uint32_t timeoutMs;
    bool clockwise;
    bool rapid;
    char units[8];
    char mode[8];
    uint32_t vel, accel, decel;
    uint32_t axisMask;
    uint32_t estopDi6;
    bool testMode;
    bool enabled;
    uint32_t stepsX, stepsY, stepsZ, stepsA;
    double pitchX, pitchY, pitchZ, pitchA;
    double gearX, gearY, gearZ, gearA;
    double minX, maxX, minY, maxY, minZ, maxZ, minA, maxA;
    bool clearLimits;
    bool clearMinX, clearMaxX, clearMinY, clearMaxY;
    bool clearMinZ, clearMaxZ, clearMinA, clearMaxA;
    uint32_t posLimX, negLimX, posLimY, negLimY, posLimZ, negLimZ, posLimA, negLimA;
    uint32_t pin;
    bool ioState;
    char axis[8];
    char dir[8];
    double seek;
    double backoff;
    bool zero;
    char active[8];
};

struct RpcRequest {
    bool ok;
    int errorCode;
    const char *errorMessage;
    RpcId id;
    char method[32];
    RpcParams params;
};

bool JsonRpcParseLine(const char *line, RpcRequest *out);
int JsonRpcFormatResult(char *buf, size_t bufLen, const RpcId *id, const char *resultObject);
int JsonRpcFormatError(char *buf, size_t bufLen, const RpcId *id, int code, const char *message);
int JsonRpcFormatNotification(char *buf, size_t bufLen, const char *method,
                              const char *paramsObject);

#endif
