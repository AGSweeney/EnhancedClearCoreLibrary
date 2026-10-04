/* MIT License. Copyright (c) 2026 Adam G. Sweeney <agsweeney@gmail.com> */

#include "Session.h"

#include "MotionCore.h"
#include "Transport.h"
#include "XrceClient.h"

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
    bool hasNetMode;
    char netMode[8];
    bool hasIp;
    char ipAddress[16];
    bool hasNetmask;
    char netmask[16];
    bool hasGateway;
    char gateway[16];
    bool hasI;
    bool hasJ;
    float arcI;
    float arcJ;
    bool hasCw;
    bool cw;
    bool hasAxisName;
    char axisName[8];
    bool hasDirName;
    char dirName[8];
    bool hasPin;
    uint8_t pin;
    bool hasActive;
    bool activeHigh;
    bool hasSeek;
    double seek;
    bool hasBackoff;
    double backoff;
    bool hasTimeout;
    uint32_t timeoutMs;
    bool hasZero;
    bool zeroOn;
    bool hasFeed;
    double feedMps;
    bool hasPort;
    uint16_t port;
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

static bool KeyAxis(const char *js, const jsmntok_t *key, const char *prefix, int *axis) {
    const int plen = (int)strlen(prefix);
    const int n = key->end - key->start;
    if (key->type != JSMN_STRING || n != plen + 1 || memcmp(js + key->start, prefix, (size_t)plen) != 0) {
        return false;
    }
    const char suffix = js[key->start + plen];
    if (suffix == 'x') *axis = 0;
    else if (suffix == 'y') *axis = 1;
    else if (suffix == 'z') *axis = 2;
    else if (suffix == 'a') *axis = 3;
    else return false;
    return true;
}

static void ApplyKey(const char *js, const jsmntok_t *toks, int ntok, int keyIndex,
                     SessionReq *req) {
    int axis = 0;
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
    } else if (TokEq(js, key, "clear_limits")) {
        if (!ParseBool(js, val, &req->cfg.clearLimits)) {
            req->error = "clear_limits must be a boolean";
            return;
        }
        req->cfg.hasClearLimits = true;
    } else if (KeyAxis(js, key, "min_", &axis) || KeyAxis(js, key, "max_", &axis)) {
        double d = 0;
        if (!ParseDouble(js, val, &d)) {
            req->error = "soft limit must be a number";
            return;
        }
        const bool isMin = js[key->start] == 'm' && js[key->start + 1] == 'i';
        if (isMin) {
            req->cfg.hasLimitMin[axis] = true;
            req->cfg.limitMin[axis] = d;
        } else {
            req->cfg.hasLimitMax[axis] = true;
            req->cfg.limitMax[axis] = d;
        }
    } else if (KeyAxis(js, key, "clear_min_", &axis) || KeyAxis(js, key, "clear_max_", &axis)) {
        bool on = false;
        if (!ParseBool(js, val, &on)) {
            req->error = "clear limit must be a boolean";
            return;
        }
        if (!on) {
            return;
        }
        if (memcmp(js + key->start, "clear_min_", 10) == 0) {
            req->cfg.hasClearMin[axis] = true;
        } else {
            req->cfg.hasClearMax[axis] = true;
        }
    } else if (KeyAxis(js, key, "pos_lim_", &axis) || KeyAxis(js, key, "neg_lim_", &axis)) {
        double d = 0;
        if (!ParseDouble(js, val, &d) || d < 0.0 || d > 255.0) {
            req->error = "limit di must be 0..12 or 255";
            return;
        }
        if (js[key->start] == 'p') {
            req->cfg.hasPosLim[axis] = true;
            req->cfg.posLim[axis] = (uint8_t)d;
        } else {
            req->cfg.hasNegLim[axis] = true;
            req->cfg.negLim[axis] = (uint8_t)d;
        }
    } else if (TokEq(js, key, "i") || TokEq(js, key, "j")) {
        double d = 0;
        if (!ParseDouble(js, val, &d)) {
            req->error = "arc offset must be a number";
            return;
        }
        if (TokEq(js, key, "i")) {
            req->hasI = true;
            req->arcI = (float)d;
        } else {
            req->hasJ = true;
            req->arcJ = (float)d;
        }
    } else if (TokEq(js, key, "cw") || TokEq(js, key, "zero")) {
        bool on = false;
        if (!ParseBool(js, val, &on)) {
            req->error = "expected a boolean";
            return;
        }
        if (TokEq(js, key, "cw")) {
            req->hasCw = true;
            req->cw = on;
        } else {
            req->hasZero = true;
            req->zeroOn = on;
        }
    } else if (TokEq(js, key, "axis") || TokEq(js, key, "dir") || TokEq(js, key, "active")) {
        if (val->type != JSMN_STRING) {
            req->error = "expected a string";
            return;
        }
        const int n = val->end - val->start;
        char *dst = req->axisName;
        int cap = (int)sizeof(req->axisName);
        if (TokEq(js, key, "dir")) {
            dst = req->dirName;
            cap = (int)sizeof(req->dirName);
        } else if (TokEq(js, key, "active")) {
            dst = req->netMode;
            cap = (int)sizeof(req->netMode);
        }
        if (n <= 0 || n >= cap) {
            req->error = "string too long";
            return;
        }
        memcpy(dst, js + val->start, (size_t)n);
        dst[n] = '\0';
        if (TokEq(js, key, "axis")) {
            req->hasAxisName = true;
        } else if (TokEq(js, key, "dir")) {
            req->hasDirName = true;
        } else if (strcmp(dst, "high") == 0) {
            req->hasActive = true;
            req->activeHigh = true;
        } else if (strcmp(dst, "low") == 0) {
            req->hasActive = true;
            req->activeHigh = false;
        } else {
            req->error = "active must be high or low";
            return;
        }
    } else if (TokEq(js, key, "pin") || TokEq(js, key, "seek") || TokEq(js, key, "backoff") ||
               TokEq(js, key, "port") || TokEq(js, key, "timeout_ms") || TokEq(js, key, "feed_mps")) {
        double d = 0;
        if (!ParseDouble(js, val, &d)) {
            req->error = "expected a number";
            return;
        }
        if (TokEq(js, key, "pin")) {
            if (d < 0.0 || d > 255.0) {
                req->error = "pin must be 1-12";
                return;
            }
            req->hasPin = true;
            req->pin = (uint8_t)d;
        } else if (TokEq(js, key, "seek")) {
            req->hasSeek = true;
            req->seek = d;
        } else if (TokEq(js, key, "backoff")) {
            req->hasBackoff = true;
            req->backoff = d;
        } else if (TokEq(js, key, "port")) {
            if (d < 1.0 || d > 65535.0) {
                req->error = "port out of range";
                return;
            }
            req->hasPort = true;
            req->port = (uint16_t)d;
        } else if (TokEq(js, key, "timeout_ms")) {
            if (d < 0.0) {
                req->error = "timeout_ms out of range";
                return;
            }
            req->hasTimeout = true;
            req->timeoutMs = (uint32_t)d;
        } else {
            req->hasFeed = true;
            req->feedMps = d;
        }
    } else if (TokEq(js, key, "mode") || TokEq(js, key, "ip_address") ||
               TokEq(js, key, "netmask") || TokEq(js, key, "gateway")) {
        if (val->type != JSMN_STRING) {
            req->error = "network fields must be strings";
            return;
        }
        const int n = val->end - val->start;
        char *dst = req->netMode;
        int cap = (int)sizeof(req->netMode);
        bool *flag = &req->hasNetMode;
        if (TokEq(js, key, "ip_address")) {
            dst = req->ipAddress;
            cap = (int)sizeof(req->ipAddress);
            flag = &req->hasIp;
        } else if (TokEq(js, key, "netmask")) {
            dst = req->netmask;
            cap = (int)sizeof(req->netmask);
            flag = &req->hasNetmask;
        } else if (TokEq(js, key, "gateway")) {
            dst = req->gateway;
            cap = (int)sizeof(req->gateway);
            flag = &req->hasGateway;
        }
        if (n <= 0 || n >= cap) {
            req->error = "network field too long";
            return;
        }
        memcpy(dst, js + val->start, (size_t)n);
        dst[n] = '\0';
        *flag = true;
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
    } else if (strcmp(req.method, "reset_config") == 0) {
        err = MotionResetConfig();
        if (!err) {
            snprintf(body, sizeof(body), "{\"ok\":true}");
        }
    } else if (strcmp(req.method, "configure_network") == 0) {
        err = MotionConfigureNetwork(req.netMode, req.hasNetMode, req.ipAddress, req.hasIp,
                                     req.netmask, req.hasNetmask, req.gateway, req.hasGateway, body,
                                     sizeof(body));
    } else if (strcmp(req.method, "restart") == 0) {
        snprintf(body, sizeof(body), "{\"ok\":true}");
        ReplyResult(reply, sizeof(reply), &req, body);
        TransportSendLine(reply);
        MotionRestart();
        return;
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
    } else if (strcmp(req.method, "move_linear") == 0) {
        uint8_t mask = 0;
        float q[CCROS_AXIS_COUNT] = {0, 0, 0, 0};
        for (uint8_t a = 0; a < CCROS_AXIS_COUNT; a++) {
            if (req.hasJoint[a]) {
                mask = (uint8_t)(mask | (1u << a));
                q[a] = req.joint[a];
            }
        }
        err = MotionMoveLinear(mask, q, req.hasFeed, req.feedMps, body, sizeof(body));
    } else if (strcmp(req.method, "move_arc") == 0) {
        if (!req.hasI || !req.hasJ) {
            err = "arc requires i and j";
        } else {
            err = MotionMoveArc(req.hasJoint[0], req.joint[0], req.hasJoint[1], req.joint[1], req.arcI,
                                req.arcJ, req.hasCw && req.cw, req.hasFeed, req.feedMps, body, sizeof(body));
        }
    } else if (strcmp(req.method, "wait_idle") == 0) {
        err = MotionWaitIdle(req.hasTimeout ? req.timeoutMs : 60000u, body, sizeof(body));
    } else if (strcmp(req.method, "home") == 0) {
        err = MotionHome(req.hasAxisName ? req.axisName : nullptr, req.hasDirName ? req.dirName : nullptr,
                         req.hasSeek, req.seek, req.hasBackoff, req.backoff, req.hasTimeout, req.timeoutMs,
                         req.hasZero, req.zeroOn, body, sizeof(body));
    } else if (strcmp(req.method, "xrce_connect") == 0) {
        if (!req.hasIp) {
            err = "ip_address required";
        } else {
            uint8_t ip[4];
            uint8_t parts = 0;
            uint16_t acc = 0;
            bool any = false;
            bool ok = true;
            for (const char *s = req.ipAddress;; s++) {
                const char c = *s;
                if (c >= '0' && c <= '9') {
                    acc = (uint16_t)(acc * 10u + (uint16_t)(c - '0'));
                    any = true;
                    if (acc > 255) {
                        ok = false;
                        break;
                    }
                } else if (c == '.' || c == '\0') {
                    if (!any || parts >= 4) {
                        ok = false;
                        break;
                    }
                    ip[parts++] = (uint8_t)acc;
                    acc = 0;
                    any = false;
                    if (c == '\0') {
                        break;
                    }
                } else {
                    ok = false;
                    break;
                }
            }
            if (!ok || parts != 4) {
                err = "invalid ip_address";
            } else {
                err = XrceConnect(ip, req.hasPort ? req.port : CCROS_XRCE_AGENT_PORT);
                if (!err) {
                    snprintf(body, sizeof(body),
                             "{\"state\":\"%s\",\"local_port\":%u,\"agent_port\":%u}",
                             XrceStateName(), (unsigned)CCROS_XRCE_LOCAL_PORT,
                             (unsigned)(req.hasPort ? req.port : CCROS_XRCE_AGENT_PORT));
                }
            }
        }
    } else if (strcmp(req.method, "xrce_disconnect") == 0) {
        XrceDisconnect();
        snprintf(body, sizeof(body), "{\"state\":\"off\"}");
    } else if (strcmp(req.method, "probe") == 0) {
        if (!req.hasPin) {
            err = "pin required";
        } else {
            err = MotionProbe(req.hasAxisName ? req.axisName : nullptr, req.hasDirName ? req.dirName : nullptr,
                              req.pin, req.hasActive ? req.activeHigh : true, req.hasSeek, req.seek,
                              req.hasBackoff, req.backoff, req.hasTimeout, req.timeoutMs, req.hasZero,
                              req.zeroOn, body, sizeof(body));
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
