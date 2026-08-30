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

#define JSMN_STATIC
#include "third_party/jsmn/jsmn.h"

#include "JsonRpcLine.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static bool TokenEq(const char *js, const jsmntok_t *tok, const char *key) {
    const int len = tok->end - tok->start;
    const int keyLen = (int)strlen(key);
    return tok->type == JSMN_STRING && len == keyLen &&
           strncmp(js + tok->start, key, (size_t)len) == 0;
}

static void TokenCopy(const char *js, const jsmntok_t *tok, char *dst, size_t dstLen) {
    int len = tok->end - tok->start;
    if (len < 0) {
        len = 0;
    }
    if ((size_t)len >= dstLen) {
        len = (int)dstLen - 1;
    }
    memcpy(dst, js + tok->start, (size_t)len);
    dst[len] = '\0';
}

static bool TokenBool(const char *js, const jsmntok_t *tok, bool *out) {
    if (tok->type != JSMN_PRIMITIVE) {
        return false;
    }
    if (js[tok->start] == 't') {
        *out = true;
        return true;
    }
    if (js[tok->start] == 'f') {
        *out = false;
        return true;
    }
    return false;
}

static bool TokenNumber(const char *js, const jsmntok_t *tok, double *out) {
    if (tok->type != JSMN_PRIMITIVE && tok->type != JSMN_STRING) {
        return false;
    }
    char tmp[48];
    TokenCopy(js, tok, tmp, sizeof(tmp));
    char *end = nullptr;
    const double v = strtod(tmp, &end);
    if (end == tmp) {
        return false;
    }
    *out = v;
    return true;
}

static void ApplyParam(const char *js, const jsmntok_t *key, const jsmntok_t *val,
                       RpcParams *p) {
    double n = 0.0;
    bool b = false;
    if (TokenEq(js, key, "x") && TokenNumber(js, val, &n)) {
        p->hasX = true;
        p->x = n;
    } else if (TokenEq(js, key, "y") && TokenNumber(js, val, &n)) {
        p->hasY = true;
        p->y = n;
    } else if (TokenEq(js, key, "z") && TokenNumber(js, val, &n)) {
        p->hasZ = true;
        p->z = n;
    } else if (TokenEq(js, key, "a") && TokenNumber(js, val, &n)) {
        p->hasA = true;
        p->a = n;
    } else if (TokenEq(js, key, "i") && TokenNumber(js, val, &n)) {
        p->hasI = true;
        p->i = n;
    } else if (TokenEq(js, key, "j") && TokenNumber(js, val, &n)) {
        p->hasJ = true;
        p->j = n;
    } else if (TokenEq(js, key, "feed") && TokenNumber(js, val, &n)) {
        p->hasFeed = true;
        p->feed = n;
    } else if (TokenEq(js, key, "timeout_ms") && TokenNumber(js, val, &n)) {
        p->hasTimeoutMs = true;
        p->timeoutMs = (n < 0.0) ? 0u : (uint32_t)n;
    } else if (TokenEq(js, key, "seconds") && TokenNumber(js, val, &n)) {
        p->hasSeconds = true;
        p->seconds = n;
    } else if (TokenEq(js, key, "clockwise") && TokenBool(js, val, &b)) {
        p->hasClockwise = true;
        p->clockwise = b;
    } else if (TokenEq(js, key, "rapid") && TokenBool(js, val, &b)) {
        p->hasRapid = true;
        p->rapid = b;
    } else if (TokenEq(js, key, "test_mode")) {
        if (TokenBool(js, val, &b)) {
            p->hasTestMode = true;
            p->testMode = b;
        } else if (TokenNumber(js, val, &n)) {
            p->hasTestMode = true;
            p->testMode = (n != 0.0);
        }
    } else if (TokenEq(js, key, "enabled")) {
        if (TokenBool(js, val, &b)) {
            p->hasEnabled = true;
            p->enabled = b;
        } else if (TokenNumber(js, val, &n)) {
            p->hasEnabled = true;
            p->enabled = (n != 0.0);
        }
    } else if (TokenEq(js, key, "units") && val->type == JSMN_STRING) {
        p->hasUnits = true;
        TokenCopy(js, val, p->units, sizeof(p->units));
    } else if (TokenEq(js, key, "mode") && val->type == JSMN_STRING) {
        p->hasMode = true;
        TokenCopy(js, val, p->mode, sizeof(p->mode));
    } else if (TokenEq(js, key, "vel") && TokenNumber(js, val, &n)) {
        p->hasVel = true;
        p->vel = (n < 0.0) ? 0u : (uint32_t)n;
    } else if (TokenEq(js, key, "accel") && TokenNumber(js, val, &n)) {
        p->hasAccel = true;
        p->accel = (n < 0.0) ? 0u : (uint32_t)n;
    } else if (TokenEq(js, key, "decel") && TokenNumber(js, val, &n)) {
        p->hasDecel = true;
        p->decel = (n < 0.0) ? 0u : (uint32_t)n;
    } else if (TokenEq(js, key, "axis_mask") && TokenNumber(js, val, &n)) {
        p->hasAxisMask = true;
        p->axisMask = (n < 0.0) ? 0u : (uint32_t)n;
    } else if (TokenEq(js, key, "estop_di6") && TokenNumber(js, val, &n)) {
        p->hasEstopDi6 = true;
        p->estopDi6 = (n < 0.0) ? 0u : (uint32_t)n;
    } else if (TokenEq(js, key, "steps_per_rev_x") && TokenNumber(js, val, &n)) {
        p->hasStepsX = true;
        p->stepsX = (n < 1.0) ? 1u : (uint32_t)n;
    } else if (TokenEq(js, key, "steps_per_rev_y") && TokenNumber(js, val, &n)) {
        p->hasStepsY = true;
        p->stepsY = (n < 1.0) ? 1u : (uint32_t)n;
    } else if (TokenEq(js, key, "steps_per_rev_z") && TokenNumber(js, val, &n)) {
        p->hasStepsZ = true;
        p->stepsZ = (n < 1.0) ? 1u : (uint32_t)n;
    } else if (TokenEq(js, key, "steps_per_rev_a") && TokenNumber(js, val, &n)) {
        p->hasStepsA = true;
        p->stepsA = (n < 1.0) ? 1u : (uint32_t)n;
    } else if (TokenEq(js, key, "pitch_x") && TokenNumber(js, val, &n)) {
        p->hasPitchX = true;
        p->pitchX = n;
    } else if (TokenEq(js, key, "pitch_y") && TokenNumber(js, val, &n)) {
        p->hasPitchY = true;
        p->pitchY = n;
    } else if (TokenEq(js, key, "pitch_z") && TokenNumber(js, val, &n)) {
        p->hasPitchZ = true;
        p->pitchZ = n;
    } else if (TokenEq(js, key, "pitch_a") && TokenNumber(js, val, &n)) {
        p->hasPitchA = true;
        p->pitchA = n;
    } else if (TokenEq(js, key, "gear_x") && TokenNumber(js, val, &n)) {
        p->hasGearX = true;
        p->gearX = n;
    } else if (TokenEq(js, key, "gear_y") && TokenNumber(js, val, &n)) {
        p->hasGearY = true;
        p->gearY = n;
    } else if (TokenEq(js, key, "gear_z") && TokenNumber(js, val, &n)) {
        p->hasGearZ = true;
        p->gearZ = n;
    } else if (TokenEq(js, key, "gear_a") && TokenNumber(js, val, &n)) {
        p->hasGearA = true;
        p->gearA = n;
    }
}

bool JsonRpcParseLine(const char *line, RpcRequest *out) {
    memset(out, 0, sizeof(*out));
    out->ok = false;
    out->errorCode = JSONRPC_PARSE_ERROR;
    out->errorMessage = "parse error";

    if (!line || line[0] == '\0') {
        return false;
    }

    jsmn_parser parser;
    jsmntok_t tokens[64];
    jsmn_init(&parser);
    const int ntok = jsmn_parse(&parser, line, strlen(line), tokens, 64);
    if (ntok < 1 || tokens[0].type != JSMN_OBJECT) {
        return false;
    }

    int paramsTok = -1;
    for (int i = 1; i < ntok; i++) {
        if (tokens[i].type != JSMN_STRING) {
            continue;
        }
        if (TokenEq(line, &tokens[i], "method") && i + 1 < ntok) {
            if (tokens[i + 1].type != JSMN_STRING) {
                out->errorCode = JSONRPC_INVALID_REQUEST;
                out->errorMessage = "method must be a string";
                return false;
            }
            TokenCopy(line, &tokens[i + 1], out->method, sizeof(out->method));
        } else if (TokenEq(line, &tokens[i], "id") && i + 1 < ntok) {
            out->id.present = true;
            out->id.isString = (tokens[i + 1].type == JSMN_STRING);
            TokenCopy(line, &tokens[i + 1], out->id.text, sizeof(out->id.text));
        } else if (TokenEq(line, &tokens[i], "params") && i + 1 < ntok) {
            if (tokens[i + 1].type != JSMN_OBJECT && tokens[i + 1].type != JSMN_ARRAY) {
                out->errorCode = JSONRPC_INVALID_PARAMS;
                out->errorMessage = "params must be an object";
                return false;
            }
            if (tokens[i + 1].type == JSMN_ARRAY) {
                out->errorCode = JSONRPC_INVALID_PARAMS;
                out->errorMessage = "params must be an object";
                return false;
            }
            paramsTok = i + 1;
        }
    }

    if (out->method[0] == '\0') {
        out->errorCode = JSONRPC_INVALID_REQUEST;
        out->errorMessage = "missing method";
        return false;
    }
    if (!out->id.present) {
        out->errorCode = JSONRPC_INVALID_REQUEST;
        out->errorMessage = "id required";
        return false;
    }

    if (paramsTok >= 0) {
        const jsmntok_t *obj = &tokens[paramsTok];
        int idx = paramsTok + 1;
        for (int p = 0; p < obj->size && idx + 1 < ntok; p++) {
            const jsmntok_t *key = &tokens[idx];
            const jsmntok_t *val = &tokens[idx + 1];
            ApplyParam(line, key, val, &out->params);
            idx += 2;
            if (val->type == JSMN_OBJECT || val->type == JSMN_ARRAY) {
                // Skip nested tokens; v1 params are scalars only.
                int remaining = val->size;
                while (remaining > 0 && idx < ntok) {
                    remaining += tokens[idx].size;
                    remaining--;
                    idx++;
                }
            }
        }
    }

    out->ok = true;
    out->errorCode = 0;
    out->errorMessage = nullptr;
    return true;
}

static int WriteId(char *buf, size_t bufLen, const RpcId *id) {
    if (!id || !id->present) {
        return snprintf(buf, bufLen, "null");
    }
    if (id->isString) {
        return snprintf(buf, bufLen, "\"%s\"", id->text);
    }
    return snprintf(buf, bufLen, "%s", id->text);
}

int JsonRpcFormatResult(char *buf, size_t bufLen, const RpcId *id, const char *resultObject) {
    char idBuf[48];
    WriteId(idBuf, sizeof(idBuf), id);
    return snprintf(buf, bufLen, "{\"jsonrpc\":\"2.0\",\"id\":%s,\"result\":%s}",
                    idBuf, resultObject ? resultObject : "{\"ok\":true}");
}

int JsonRpcFormatError(char *buf, size_t bufLen, const RpcId *id, int code, const char *message) {
    char idBuf[48];
    WriteId(idBuf, sizeof(idBuf), id);
    const char *msg = message ? message : "error";
    return snprintf(buf, bufLen,
                    "{\"jsonrpc\":\"2.0\",\"id\":%s,\"error\":{\"code\":%d,\"message\":\"%s\"}}",
                    idBuf, code, msg);
}

int JsonRpcFormatNotification(char *buf, size_t bufLen, const char *method,
                              const char *paramsObject) {
    return snprintf(buf, bufLen, "{\"jsonrpc\":\"2.0\",\"method\":\"%s\",\"params\":%s}",
                    method, paramsObject ? paramsObject : "{}");
}
