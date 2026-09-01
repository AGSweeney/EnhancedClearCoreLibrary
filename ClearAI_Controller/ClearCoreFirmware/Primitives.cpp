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

#include "Primitives.h"

#include "JsonRpcLine.h"
#include "MotionBridge.h"
#include "Transport.h"

#include "ClearCore.h"
#include <stdio.h>
#include <string.h>

static bool g_inWait = false;
static bool g_safetyHit = false;

static void ReplyResult(const RpcId *id, const char *resultObject) {
    char line[CLEARAI_MAX_REPLY];
    JsonRpcFormatResult(line, sizeof(line), id, resultObject);
    TransportSendLine(line);
}

static void ReplyError(const RpcId *id, int code, const char *message) {
    char line[CLEARAI_MAX_REPLY];
    JsonRpcFormatError(line, sizeof(line), id, code, message);
    TransportSendLine(line);
}

static void ReplyOkQueued(const RpcId *id) {
    ReplyResult(id, "{\"ok\":true,\"queued\":true}");
}

static void ReplyOkQueuedEst(const RpcId *id, uint32_t estMs) {
    char body[64];
    snprintf(body, sizeof(body), "{\"ok\":true,\"queued\":true,\"est_ms\":%lu}",
             (unsigned long)estMs);
    ReplyResult(id, body);
}

static void ReplyOkElapsed(const RpcId *id, uint32_t elapsedMs) {
    char body[64];
    snprintf(body, sizeof(body), "{\"ok\":true,\"elapsed_ms\":%lu}",
             (unsigned long)elapsedMs);
    ReplyResult(id, body);
}

static void ReplyOk(const RpcId *id) {
    ReplyResult(id, "{\"ok\":true}");
}

static bool IsSafetyMethod(const char *method) {
    return strcmp(method, "estop") == 0 ||
           strcmp(method, "stop") == 0 ||
           strcmp(method, "jog_stop") == 0 ||
           strcmp(method, "disable") == 0 ||
           strcmp(method, "queue_clear") == 0 ||
           strcmp(method, "keepalive") == 0 ||
           strcmp(method, "subscribe_inputs") == 0 ||
           strcmp(method, "unsubscribe_inputs") == 0;
}

static void DispatchParsed(const RpcRequest *req) {
    const char *method = req->method;
    const RpcParams *p = &req->params;
    const RpcId *id = &req->id;

    if (strcmp(method, "get_capabilities") == 0) {
        char body[CLEARAI_MAX_REPLY];
        MotionFillCapabilitiesJson(body, sizeof(body));
        ReplyResult(id, body);
        return;
    }
    if (strcmp(method, "get_status") == 0) {
        char body[CLEARAI_MAX_REPLY];
        MotionFillStatusJson(body, sizeof(body));
        ReplyResult(id, body);
        return;
    }
    if (strcmp(method, "get_pose") == 0) {
        char body[CLEARAI_MAX_REPLY];
        MotionFillPoseJson(body, sizeof(body));
        ReplyResult(id, body);
        return;
    }
    if (strcmp(method, "get_config") == 0) {
        char body[CLEARAI_MAX_REPLY];
        MotionFillConfigJson(body, sizeof(body));
        ReplyResult(id, body);
        return;
    }
    if (strcmp(method, "read_inputs") == 0) {
        char body[CLEARAI_MAX_REPLY];
        const char *err = MotionReadInputs(p, body, sizeof(body));
        if (err) {
            ReplyError(id, JSONRPC_INVALID_PARAMS, err);
        } else {
            ReplyResult(id, body);
        }
        return;
    }
    if (strcmp(method, "write_output") == 0) {
        const char *err = MotionWriteOutput(p);
        if (err) {
            ReplyError(id, JSONRPC_APP_ERROR, err);
        } else {
            ReplyOk(id);
        }
        return;
    }
    if (strcmp(method, "read_analog") == 0) {
        char body[CLEARAI_MAX_REPLY];
        const char *err = MotionReadAnalog(p, body, sizeof(body));
        if (err) {
            ReplyError(id, JSONRPC_INVALID_PARAMS, err);
        } else {
            ReplyResult(id, body);
        }
        return;
    }
    if (strcmp(method, "write_analog") == 0) {
        const char *err = MotionWriteAnalog(p);
        if (err) {
            ReplyError(id, JSONRPC_APP_ERROR, err);
        } else {
            ReplyOk(id);
        }
        return;
    }
    if (strcmp(method, "write_pwm") == 0) {
        const char *err = MotionWritePwm(p);
        if (err) {
            ReplyError(id, JSONRPC_APP_ERROR, err);
        } else {
            ReplyOk(id);
        }
        return;
    }
    if (strcmp(method, "subscribe_inputs") == 0) {
        char body[CLEARAI_MAX_REPLY];
        const char *err = MotionSubscribeInputs(p, body, sizeof(body));
        if (err) {
            ReplyError(id, JSONRPC_APP_ERROR, err);
        } else {
            ReplyResult(id, body);
        }
        return;
    }
    if (strcmp(method, "unsubscribe_inputs") == 0) {
        MotionUnsubscribeInputs();
        ReplyOk(id);
        return;
    }
    if (strcmp(method, "configure_network") == 0) {
        char body[CLEARAI_MAX_REPLY];
        const char *err = MotionConfigureNetwork(p, body, sizeof(body));
        if (err) {
            ReplyError(id, JSONRPC_APP_ERROR, err);
        } else {
            ReplyResult(id, body);
        }
        return;
    }
    if (strcmp(method, "restart") == 0) {
        ReplyOk(id);
        MotionRestart();  /* does not return; board resets */
        return;
    }
    if (strcmp(method, "enable") == 0) {
        const char *err = MotionEnable();
        if (err) {
            ReplyError(id, JSONRPC_APP_ERROR, err);
        } else {
            ReplyOk(id);
        }
        return;
    }
    if (strcmp(method, "disable") == 0) {
        MotionDisable();
        MotionLog("disable", "disabled", 1);
        ReplyOk(id);
        return;
    }
    if (strcmp(method, "clear_alerts") == 0) {
        MotionClearAlerts();
        ReplyOk(id);
        return;
    }
    if (strcmp(method, "stop") == 0) {
        MotionStop();
        MotionLog("stop", "stopped", 1);
        ReplyOk(id);
        return;
    }
    if (strcmp(method, "estop") == 0) {
        MotionEstop();
        MotionLog("estop", "estop", 1);
        ReplyOk(id);
        return;
    }
    if (strcmp(method, "configure") == 0) {
        const char *err = MotionConfigure(p);
        if (err) {
            MotionLog("configure", err, 0);
            ReplyError(id, JSONRPC_APP_ERROR, err);
        } else {
            ReplyOk(id);
        }
        return;
    }
    if (strcmp(method, "reset_config") == 0) {
        const char *err = MotionResetConfig();
        if (err) {
            ReplyError(id, JSONRPC_APP_ERROR, err);
        } else {
            ReplyOk(id);
        }
        return;
    }
    if (strcmp(method, "set_test_mode") == 0) {
        MotionSetTestMode(p);
        ReplyOk(id);
        return;
    }
    if (strcmp(method, "set_units") == 0) {
        const char *err = MotionSetUnits(p);
        if (err) {
            ReplyError(id, JSONRPC_INVALID_PARAMS, err);
        } else {
            ReplyOk(id);
        }
        return;
    }
    if (strcmp(method, "set_units_a") == 0) {
        const char *err = MotionSetUnitsA(p);
        if (err) {
            ReplyError(id, JSONRPC_INVALID_PARAMS, err);
        } else {
            ReplyOk(id);
        }
        return;
    }
    if (strcmp(method, "set_mode") == 0) {
        const char *err = MotionSetMode(p);
        if (err) {
            ReplyError(id, JSONRPC_INVALID_PARAMS, err);
        } else {
            ReplyOk(id);
        }
        return;
    }
    if (strcmp(method, "set_work_origin") == 0) {
        MotionSetWorkOrigin(p);
        ReplyOk(id);
        return;
    }
    if (strcmp(method, "move_linear") == 0) {
        uint32_t estMs = 0;
        const char *err = MotionMoveLinear(p, &estMs);
        if (err) {
            MotionIncMovesRejected();
            MotionLog("move_linear", err, 0);
            ReplyError(id, JSONRPC_APP_ERROR, err);
        } else {
            ReplyOkQueuedEst(id, estMs);
        }
        return;
    }
    if (strcmp(method, "move_arc") == 0) {
        uint32_t estMs = 0;
        const char *err = MotionMoveArc(p, &estMs);
        if (err) {
            MotionIncMovesRejected();
            MotionLog("move_arc", err, 0);
            ReplyError(id, JSONRPC_APP_ERROR, err);
        } else {
            ReplyOkQueuedEst(id, estMs);
        }
        return;
    }
    if (strcmp(method, "jog") == 0) {
        const char *err = MotionJog(p);
        if (err) {
            MotionIncMovesRejected();
            MotionLog("jog", err, 0);
            ReplyError(id, JSONRPC_APP_ERROR, err);
        } else {
            ReplyOkQueued(id);
        }
        return;
    }
    if (strcmp(method, "jog_velocity") == 0) {
        const char *err = MotionJogVelocity(p);
        if (err) {
            MotionIncMovesRejected();
            MotionLog("jog_velocity", err, 0);
            ReplyError(id, JSONRPC_APP_ERROR, err);
        } else {
            ReplyOk(id);
        }
        return;
    }
    if (strcmp(method, "jog_stop") == 0) {
        MotionJogStop();
        ReplyOk(id);
        return;
    }
    if (strcmp(method, "move_batch") == 0) {
        uint16_t accepted = 0;
        const char *err = MotionMoveBatch(p, &accepted);
        if (err) {
            MotionIncMovesRejected();
            MotionLog("move_batch", err, 0);
            char body[CLEARAI_MAX_REPLY];
            snprintf(body, sizeof(body),
                      "{\"ok\":false,\"accepted\":%u,\"failed_at\":%u,\"error\":\"%s\"}",
                      (unsigned)accepted, (unsigned)accepted, err);
            ReplyResult(id, body);
        } else {
            char body[64];
            snprintf(body, sizeof(body), "{\"ok\":true,\"queued\":true,\"accepted\":%u}",
                     (unsigned)accepted);
            ReplyResult(id, body);
        }
        return;
    }
    if (strcmp(method, "dwell") == 0) {
        g_inWait = true;
        g_safetyHit = false;
        const char *err = MotionDwell(p, PrimitivesPollSafetyDuringWait);
        g_inWait = false;
        if (err) {
            ReplyError(id, JSONRPC_APP_ERROR, err);
        } else {
            ReplyOk(id);
        }
        return;
    }
    if (strcmp(method, "wait_idle") == 0) {
        g_inWait = true;
        g_safetyHit = false;
        const uint32_t startMs = Milliseconds();
        const char *err = MotionWaitIdle(p, PrimitivesPollSafetyDuringWait);
        g_inWait = false;
        if (err) {
            MotionLog("wait_idle", err, 0);
            ReplyError(id, JSONRPC_APP_ERROR, err);
        } else {
            ReplyOkElapsed(id, Milliseconds() - startMs);
        }
        return;
    }
    if (strcmp(method, "queue_status") == 0) {
        char body[CLEARAI_MAX_REPLY];
        MotionFillQueueStatusJson(body, sizeof(body));
        ReplyResult(id, body);
        return;
    }
    if (strcmp(method, "get_log") == 0) {
        char body[CLEARAI_MAX_REPLY];
        MotionGetLog(body, sizeof(body));
        ReplyResult(id, body);
        return;
    }
    if (strcmp(method, "clear_log") == 0) {
        MotionClearLog();
        ReplyOk(id);
        return;
    }
    if (strcmp(method, "queue_clear") == 0) {
        MotionQueueClear();
        ReplyOk(id);
        return;
    }
    if (strcmp(method, "home") == 0) {
        g_inWait = true;
        g_safetyHit = false;
        char body[CLEARAI_MAX_REPLY];
        const char *err = MotionHome(p, body, sizeof(body), PrimitivesPollSafetyDuringWait);
        g_inWait = false;
        if (err) {
            MotionLog("home", err, 0);
            ReplyError(id, JSONRPC_APP_ERROR, err);
        } else {
            ReplyResult(id, body);
        }
        return;
    }
    if (strcmp(method, "probe") == 0) {
        g_inWait = true;
        g_safetyHit = false;
        char body[CLEARAI_MAX_REPLY];
        const char *err = MotionProbe(p, body, sizeof(body), PrimitivesPollSafetyDuringWait);
        g_inWait = false;
        if (err) {
            MotionLog("probe", err, 0);
            ReplyError(id, JSONRPC_APP_ERROR, err);
        } else {
            ReplyResult(id, body);
        }
        return;
    }
    if (strcmp(method, "keepalive") == 0) {
        MotionKeepalive();
        ReplyOk(id);
        return;
    }

    ReplyError(id, JSONRPC_METHOD_NOT_FOUND, "method not found");
}

void PrimitivesDispatchLine(const char *line) {
    RpcRequest req;
    if (!JsonRpcParseLine(line, &req)) {
        ReplyError(&req.id, req.errorCode, req.errorMessage);
        return;
    }
    DispatchParsed(&req);
}

bool PrimitivesPollSafetyDuringWait() {
    if (!g_inWait) {
        return false;
    }
    char line[CLEARAI_MAX_LINE];
    if (!TransportReadLine(line, sizeof(line))) {
        return false;
    }
    RpcRequest req;
    if (!JsonRpcParseLine(line, &req)) {
        ReplyError(&req.id, req.errorCode, req.errorMessage);
        return g_safetyHit;
    }
    if (IsSafetyMethod(req.method)) {
        DispatchParsed(&req);
        g_safetyHit = true;
        return true;
    }
    if (        strcmp(req.method, "get_status") == 0 ||
        strcmp(req.method, "get_pose") == 0 ||
        strcmp(req.method, "get_capabilities") == 0 ||
        strcmp(req.method, "get_config") == 0 ||
        strcmp(req.method, "read_inputs") == 0 ||
        strcmp(req.method, "read_analog") == 0 ||
        strcmp(req.method, "configure_network") == 0 ||
        strcmp(req.method, "queue_status") == 0 ||
        strcmp(req.method, "get_log") == 0 ||
        strcmp(req.method, "clear_log") == 0 ||
        strcmp(req.method, "set_test_mode") == 0) {
        DispatchParsed(&req);
        return MotionInterrupted();
    }
    ReplyError(&req.id, JSONRPC_APP_ERROR, "busy waiting");
    return MotionInterrupted();
}
