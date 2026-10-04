/* MIT License. Copyright (c) 2026 Adam G. Sweeney <agsweeney@gmail.com> */

#include "Session.h"

#include "MotionCore.h"
#include "Transport.h"

#define JSMN_STATIC
#include "jsmn.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

struct SessionReq {
    const char *error;
    bool hasId;
    int id;
    char method[32];
    MotionConfigPatch cfg;
    bool testOn;
    bool hasTest;
    bool hasJoint[CCROS_AXIS_COUNT];
    float joint[CCROS_AXIS_COUNT];
};

static bool TokEq(const char *js, const jsmntok_t *t, const char *s) {
    const int n = t->end - t->start;
    const int m = (int)strlen(s);
    return t->type == JSMN_STRING && n == m && memcmp(js + t->start, s, (size_t)n) == 0;
}

static int SkipToken(const jsmntok_t *toks, int ntok, int i) {
    if (i < 0 || i >= ntok) {
        return ntok;
    }
    if (toks[i].type == JSMN_ARRAY) {
        const int n = toks[i].size;
        i++;
        for (int k = 0; k < n; k++) {
            i = SkipToken(toks, ntok, i);
        }
        return i;
    }
    if (toks[i].type == JSMN_OBJECT) {
        const int n = toks[i].size;
        i++;
        for (int k = 0; k < n; k++) {
            i = SkipToken(toks, ntok, i);
            i = SkipToken(toks, ntok, i);
        }
        return i;
    }
    return i + 1;
}

static bool ParseDouble(const char *js, const jsmntok_t *t, double *out) {
    if (t->type != JSMN_PRIMITIVE && t->type != JSMN_STRING) {
        return false;
    }
    char tmp[48];
    const int n = t->end - t->start;
    if (n <= 0 || n >= (int)sizeof(tmp)) {
        return false;
    }
    memcpy(tmp, js + t->start, (size_t)n);
    tmp[n] = '\0';
    if (strcmp(tmp, "true") == 0 || strcmp(tmp, "false") == 0 || strcmp(tmp, "null") == 0) {
        return false;
    }
    char *end = nullptr;
    *out = strtod(tmp, &end);
    return end != tmp;
}

static bool ParseBool(const char *js, const jsmntok_t *t, bool *out) {
    char tmp[16];
    const int n = t->end - t->start;
    if (n <= 0 || n >= (int)sizeof(tmp)) {
        return false;
    }
    memcpy(tmp, js + t->start, (size_t)n);
    tmp[n] = '\0';
    if (strcmp(tmp, "true") == 0) {
        *out = true;
        return true;
    }
    if (strcmp(tmp, "false") == 0) {
        *out = false;
        return true;
    }
    double d = 0;
    if (!ParseDouble(js, t, &d)) {
        return false;
    }
    *out = (d != 0.0);
    return true;
}

static const char *ReadNumberArray(const char *js, const jsmntok_t *toks, int ntok,
                                   int valIndex, double *dst, int *count) {
    const jsmntok_t *val = &toks[valIndex];
    if (val->type == JSMN_PRIMITIVE) {
        double d = 0;
        if (!ParseDouble(js, val, &d)) {
            return "expected number";
        }
        for (int i = 0; i < CCROS_AXIS_COUNT; i++) {
            dst[i] = d;
        }
        *count = 1;
        return nullptr;
    }
    if (val->type != JSMN_ARRAY || (val->size != 1 && val->size != CCROS_AXIS_COUNT)) {
        return "expected a number or an array of 4";
    }
    int child = valIndex + 1;
    for (int k = 0; k < val->size; k++) {
        if (child >= ntok) {
            return "truncated array";
        }
        double d = 0;
        if (!ParseDouble(js, &toks[child], &d)) {
            return "expected number";
        }
        if (val->size == 1) {
            for (int i = 0; i < CCROS_AXIS_COUNT; i++) {
                dst[i] = d;
            }
        } else {
            dst[k] = d;
        }
        child = SkipToken(toks, ntok, child);
    }
    *count = val->size;
    return nullptr;
}

static void ApplyKey(const char *js, const jsmntok_t *toks, int ntok, int keyIndex,
                     SessionReq *req) {
    if (req->error) {
        return;
    }
    const jsmntok_t *key = &toks[keyIndex];
    const int vi = keyIndex + 1;
    if (vi >= ntok) {
        req->error = "missing value";
        return;
    }
    const jsmntok_t *val = &toks[vi];
    if (TokEq(js, key, "axis_mask")) {
        double d = 0;
        if (!ParseDouble(js, val, &d)) {
            req->error = "axis_mask must be a number";
            return;
        }
        req->cfg.hasAxisMask = true;
        req->cfg.axisMask = (uint32_t)d;
    } else if (TokEq(js, key, "steps_per_rev")) {
        double d[CCROS_AXIS_COUNT] = {0, 0, 0, 0};
        int n = 0;
        req->error = ReadNumberArray(js, toks, ntok, vi, d, &n);
        if (req->error) {
            return;
        }
        req->cfg.hasSteps = true;
        for (int i = 0; i < CCROS_AXIS_COUNT; i++) {
            req->cfg.stepsPerRev[i] = (uint32_t)d[i];
        }
    } else if (TokEq(js, key, "pitch_mm")) {
        double d[CCROS_AXIS_COUNT] = {0, 0, 0, 0};
        int n = 0;
        req->error = ReadNumberArray(js, toks, ntok, vi, d, &n);
        if (req->error) {
            return;
        }
        req->cfg.hasPitch = true;
        for (int i = 0; i < CCROS_AXIS_COUNT; i++) {
            req->cfg.pitchMm[i] = d[i];
        }
    } else if (TokEq(js, key, "vel_steps") || TokEq(js, key, "accel_steps") ||
               TokEq(js, key, "decel_steps") || TokEq(js, key, "watchdog_ms") ||
               TokEq(js, key, "estop_di6")) {
        double d = 0;
        if (!ParseDouble(js, val, &d) || d < 0) {
            req->error = "expected a non-negative number";
            return;
        }
        if (TokEq(js, key, "vel_steps")) {
            req->cfg.hasVel = true;
            req->cfg.vel = (uint32_t)d;
        } else if (TokEq(js, key, "accel_steps")) {
            req->cfg.hasAccel = true;
            req->cfg.accel = (uint32_t)d;
        } else if (TokEq(js, key, "decel_steps")) {
            req->cfg.hasDecel = true;
            req->cfg.decel = (uint32_t)d;
        } else if (TokEq(js, key, "watchdog_ms")) {
            req->cfg.hasWatchdog = true;
            req->cfg.watchdogMs = (uint32_t)d;
        } else {
            req->cfg.hasEstop = true;
            req->cfg.estopDi6 = (uint8_t)d;
        }
    } else if (TokEq(js, key, "on")) {
        if (!ParseBool(js, val, &req->testOn)) {
            req->error = "on must be a boolean";
            return;
        }
        req->hasTest = true;
    } else if (TokEq(js, key, "x") || TokEq(js, key, "y") || TokEq(js, key, "z") ||
               TokEq(js, key, "a")) {
        double d = 0;
        if (!ParseDouble(js, val, &d)) {
            req->error = "joint value must be a number";
            return;
        }
        int axis = 0;
        if (TokEq(js, key, "y")) axis = 1;
        else if (TokEq(js, key, "z")) axis = 2;
        else if (TokEq(js, key, "a")) axis = 3;
        req->hasJoint[axis] = true;
        req->joint[axis] = (float)d;
    }
}

static void ReplyErr(char *buf, uint16_t len, const SessionReq *req, int code, const char *msg) {
    if (req->hasId) {
        snprintf(buf, len,
                 "{\"jsonrpc\":\"2.0\",\"id\":%d,\"error\":{\"code\":%d,\"message\":\"%s\"}}",
                 req->id, code, msg);
    } else {
        snprintf(buf, len,
                 "{\"jsonrpc\":\"2.0\",\"id\":null,\"error\":{\"code\":%d,\"message\":\"%s\"}}",
                 code, msg);
    }
}

static void ReplyResult(char *buf, uint16_t len, const SessionReq *req, const char *resultJson) {
    if (req->hasId) {
        snprintf(buf, len, "{\"jsonrpc\":\"2.0\",\"id\":%d,\"result\":%s}", req->id, resultJson);
    } else {
        snprintf(buf, len, "{\"jsonrpc\":\"2.0\",\"id\":null,\"result\":%s}", resultJson);
    }
}

static bool ParseLine(const char *js, SessionReq *req) {
    memset(req, 0, sizeof(*req));
    jsmn_parser parser;
    jsmntok_t toks[96];
    jsmn_init(&parser);
    const int n = jsmn_parse(&parser, js, strlen(js), toks, 96);
    if (n < 1 || toks[0].type != JSMN_OBJECT) {
        req->error = "parse error";
        return false;
    }
    int i = 1;
    for (int k = 0; k < toks[0].size; k++) {
        if (i >= n) {
            req->error = "parse error";
            return false;
        }
        if (TokEq(js, &toks[i], "id")) {
            double d = 0;
            if (i + 1 < n && ParseDouble(js, &toks[i + 1], &d)) {
                req->hasId = true;
                req->id = (int)d;
            }
        } else if (TokEq(js, &toks[i], "method")) {
            if (i + 1 >= n || toks[i + 1].type != JSMN_STRING) {
                req->error = "method must be a string";
                return false;
            }
            const int len = toks[i + 1].end - toks[i + 1].start;
            if (len <= 0 || len >= (int)sizeof(req->method)) {
                req->error = "method name too long";
                return false;
            }
            memcpy(req->method, js + toks[i + 1].start, (size_t)len);
            req->method[len] = '\0';
        } else if (TokEq(js, &toks[i], "params")) {
            if (i + 1 >= n || toks[i + 1].type != JSMN_OBJECT) {
                req->error = "params must be an object";
                return false;
            }
            const int paramObj = i + 1;
            int child = paramObj + 1;
            for (int p = 0; p < toks[paramObj].size; p++) {
                ApplyKey(js, toks, n, child, req);
                if (req->error) {
                    return false;
                }
                child = SkipToken(toks, n, child + 1);
            }
        }
        i = SkipToken(toks, n, i + 1);
    }
    if (req->method[0] == '\0') {
        req->error = "missing method";
        return false;
    }
    return true;
}

void SessionDispatch(const char *line) {
    char reply[CCROS_MAX_REPLY];
    SessionReq req;
    if (!ParseLine(line, &req)) {
        ReplyErr(reply, sizeof(reply), &req, -32700, req.error ? req.error : "parse error");
        TransportSendLine(reply);
        return;
    }
    if (strcmp(req.method, "keepalive") != 0) {
        MotionNoteHost();
    }

    const char *err = nullptr;
    char body[CCROS_MAX_REPLY];
    body[0] = '\0';

    if (strcmp(req.method, "get_capabilities") == 0) {
        MotionFillCapabilitiesJson(body, sizeof(body));
    } else if (strcmp(req.method, "get_config") == 0) {
        MotionFillConfigJson(body, sizeof(body));
    } else if (strcmp(req.method, "get_status") == 0) {
        MotionFillStatusJson(body, sizeof(body));
    } else if (strcmp(req.method, "configure") == 0) {
        err = MotionConfigure(&req.cfg);
        if (!err) {
            snprintf(body, sizeof(body), "{\"ok\":true}");
        }
    } else if (strcmp(req.method, "set_test_mode") == 0) {
        if (!req.hasTest) {
            err = "missing on";
        } else {
            err = MotionSetTestMode(req.testOn);
            if (!err) {
                snprintf(body, sizeof(body), "{\"test_mode\":%s}", req.testOn ? "true" : "false");
            }
        }
    } else if (strcmp(req.method, "enable") == 0) {
        err = MotionEnable();
        if (!err) {
            snprintf(body, sizeof(body), "{\"enabled\":true}");
        }
    } else if (strcmp(req.method, "disable") == 0) {
        err = MotionDisable();
        if (!err) {
            snprintf(body, sizeof(body), "{\"enabled\":false}");
        }
    } else if (strcmp(req.method, "stop") == 0) {
        err = MotionStop();
        if (!err) {
            snprintf(body, sizeof(body), "{\"ok\":true}");
        }
    } else if (strcmp(req.method, "estop") == 0) {
        err = MotionEstop();
        if (!err) {
            snprintf(body, sizeof(body), "{\"estop\":true}");
        }
    } else if (strcmp(req.method, "clear_alerts") == 0) {
        err = MotionClearAlerts();
        if (!err) {
            snprintf(body, sizeof(body), "{\"ok\":true}");
        }
    } else if (strcmp(req.method, "keepalive") == 0) {
        err = MotionKeepalive();
        if (!err) {
            snprintf(body, sizeof(body), "{\"ok\":true}");
        }
    } else if (strcmp(req.method, "set_joints") == 0) {
        uint8_t mask = 0;
        float q[CCROS_AXIS_COUNT] = {0, 0, 0, 0};
        for (uint8_t a = 0; a < CCROS_AXIS_COUNT; a++) {
            if (req.hasJoint[a]) {
                mask = (uint8_t)(mask | (1u << a));
                q[a] = req.joint[a];
            }
        }
        if (mask == 0) {
            err = "no joints";
        } else {
            err = MotionSetJoints(mask, q);
            if (!err) {
                snprintf(body, sizeof(body), "{\"ok\":true}");
            }
        }
    } else {
        ReplyErr(reply, sizeof(reply), &req, -32601, "method not found");
        TransportSendLine(reply);
        return;
    }

    if (err) {
        ReplyErr(reply, sizeof(reply), &req, -32000, err);
    } else {
        ReplyResult(reply, sizeof(reply), &req, body);
    }
    TransportSendLine(reply);
}
