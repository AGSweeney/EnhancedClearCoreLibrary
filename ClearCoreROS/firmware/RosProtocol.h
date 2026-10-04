/* MIT License. Copyright (c) 2026 Adam G. Sweeney <agsweeney@gmail.com> */
/*
 * Little-endian stream frames shared by the firmware and host/test_protocol.cpp.
 * Layout is specified in ../PROTOCOL.md. Do not put a packed struct on the wire.
 */

#ifndef __CCROS_PROTOCOL_H__
#define __CCROS_PROTOCOL_H__

#include <stdint.h>
#include <string.h>

#define CCROS_MAGIC 0xC5
#define CCROS_WIRE_VERSION 1

#define CCROS_TYPE_STATE 1
#define CCROS_TYPE_POSITION 2
#define CCROS_TYPE_VELOCITY 3
#define CCROS_TYPE_HEARTBEAT 4
#define CCROS_TYPE_TRACK 5

#define CCROS_FLAG_ENABLED 0x01u
#define CCROS_FLAG_MOVING 0x02u
#define CCROS_FLAG_ESTOP 0x04u
#define CCROS_FLAG_FAULT 0x08u
#define CCROS_FLAG_WATCHDOG 0x10u

#define CCROS_HDR_SIZE 6
#define CCROS_STATE_PAYLOAD 128
#define CCROS_POSITION_PAYLOAD 20
#define CCROS_VELOCITY_PAYLOAD 20
#define CCROS_HEARTBEAT_PAYLOAD 2
#define CCROS_TRACK_PAYLOAD 36
#define CCROS_MAX_FRAME (CCROS_HDR_SIZE + CCROS_STATE_PAYLOAD)

#define CCROS_DECODE_NEED_MORE 0
#define CCROS_DECODE_OK 1
#define CCROS_DECODE_RESYNC (-1)

#define CCROS_STREAM_POSITION 1
#define CCROS_STREAM_VELOCITY 2
#define CCROS_STREAM_HEARTBEAT 3

/* Any integer steps/s change is applied. A wider deadband drops the small
 * velocity updates a timed trajectory needs. Identical commands are not reissued.
 */
static inline int32_t CcrosVelocityDeadband(int32_t commanded, int32_t latched) {
    (void)commanded;
    (void)latched;
    return 1;
}

/* Feedforward velocity plus a correction from scheduled-position error.
 * corr_limit and vel_cap are steps/s, both >= 0.
 */
static inline int32_t CcrosTrackVelocity(int32_t ff_sps, int32_t pos_error_steps,
                                        int32_t kp, int32_t corr_limit, int32_t vel_cap) {
    int64_t corr = (int64_t)kp * (int64_t)pos_error_steps;
    if (corr_limit < 0) {
        corr_limit = 0;
    }
    if (corr > corr_limit) {
        corr = corr_limit;
    }
    if (corr < -(int64_t)corr_limit) {
        corr = -(int64_t)corr_limit;
    }
    if (vel_cap < 0) {
        vel_cap = 0;
    }
    int64_t sps = (int64_t)ff_sps + corr;
    if (sps > vel_cap) {
        sps = vel_cap;
    }
    if (sps < -(int64_t)vel_cap) {
        sps = -(int64_t)vel_cap;
    }
    return (int32_t)sps;
}

/* Velocity streaming holds on the first unchanged cycle. A heartbeat leaves
 * the previous velocity running, so the transition is an explicit position hold.
 */
static inline int CcrosNextStream(int velocity_mode, int command_changed, int *hold_pending) {
    if (velocity_mode && command_changed) {
        *hold_pending = 1;
        return CCROS_STREAM_VELOCITY;
    }
    if (velocity_mode && *hold_pending) {
        *hold_pending = 0;
        return CCROS_STREAM_POSITION;
    }
    if (!velocity_mode) {
        return CCROS_STREAM_POSITION;
    }
    return CCROS_STREAM_HEARTBEAT;
}

typedef struct CcrosState {
    uint32_t time_ms;
    uint16_t seq;
    uint8_t flags;
    uint8_t axis_mask;
    uint32_t alert_reg;
    float position[4];
    float velocity[4];
    float effort[4];
    /* Latched track target and the velocity actually passed to the step
     * generator, sampled at the same time_ms as position. */
    uint8_t track_mask;
    float target_position[4];
    float target_velocity[4];
    float command_velocity[4];
    uint32_t target_latch_ms[4];
} CcrosState;

typedef struct CcrosDecoded {
    uint8_t type;
    uint16_t seq;
    uint8_t mask;
    uint8_t flags;
    uint32_t time_ms;
    uint32_t alert_reg;
    float a[4];
    float b[4];
    float c[4];
    uint8_t track_mask;
    float target_position[4];
    float target_velocity[4];
    float command_velocity[4];
    uint32_t target_latch_ms[4];
} CcrosDecoded;

static inline void CcrosPutU16(uint8_t *p, uint16_t v) {
    p[0] = (uint8_t)(v & 0xffu);
    p[1] = (uint8_t)((v >> 8) & 0xffu);
}

static inline void CcrosPutU32(uint8_t *p, uint32_t v) {
    p[0] = (uint8_t)(v & 0xffu);
    p[1] = (uint8_t)((v >> 8) & 0xffu);
    p[2] = (uint8_t)((v >> 16) & 0xffu);
    p[3] = (uint8_t)((v >> 24) & 0xffu);
}

static inline void CcrosPutF32(uint8_t *p, float v) {
    uint32_t u = 0;
    memcpy(&u, &v, sizeof(u));
    CcrosPutU32(p, u);
}

static inline uint16_t CcrosGetU16(const uint8_t *p) {
    return (uint16_t)p[0] | (uint16_t)((uint16_t)p[1] << 8);
}

static inline uint32_t CcrosGetU32(const uint8_t *p) {
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) |
           ((uint32_t)p[3] << 24);
}

static inline float CcrosGetF32(const uint8_t *p) {
    uint32_t u = CcrosGetU32(p);
    float v = 0.f;
    memcpy(&v, &u, sizeof(v));
    return v;
}

static inline void CcrosPutHdr(uint8_t *p, uint8_t type, uint16_t plen) {
    p[0] = CCROS_MAGIC;
    p[1] = CCROS_WIRE_VERSION;
    p[2] = type;
    p[3] = 0;
    CcrosPutU16(p + 4, plen);
}

static inline void CcrosPutFloats(uint8_t *p, const float *v) {
    for (int i = 0; i < 4; i++) {
        CcrosPutF32(p + (i * 4), v[i]);
    }
}

static inline void CcrosGetFloats(const uint8_t *p, float *v) {
    for (int i = 0; i < 4; i++) {
        v[i] = CcrosGetF32(p + (i * 4));
    }
}

static inline int CcrosEncodeState(uint8_t *buf, uint16_t cap, const CcrosState *s) {
    if (cap < CCROS_HDR_SIZE + CCROS_STATE_PAYLOAD) {
        return -1;
    }
    CcrosPutHdr(buf, CCROS_TYPE_STATE, CCROS_STATE_PAYLOAD);
    uint8_t *p = buf + CCROS_HDR_SIZE;
    CcrosPutU32(p + 0, s->time_ms);
    CcrosPutU16(p + 4, s->seq);
    p[6] = s->flags;
    p[7] = s->axis_mask;
    CcrosPutU32(p + 8, s->alert_reg);
    CcrosPutFloats(p + 12, s->position);
    CcrosPutFloats(p + 28, s->velocity);
    CcrosPutFloats(p + 44, s->effort);
    p[60] = s->track_mask;
    p[61] = 0;
    CcrosPutU16(p + 62, 0);
    CcrosPutFloats(p + 64, s->target_position);
    CcrosPutFloats(p + 80, s->target_velocity);
    CcrosPutFloats(p + 96, s->command_velocity);
    for (int i = 0; i < 4; i++) {
        CcrosPutU32(p + 112 + (i * 4), s->target_latch_ms[i]);
    }
    return CCROS_HDR_SIZE + CCROS_STATE_PAYLOAD;
}

static inline int CcrosEncodePosition(uint8_t *buf, uint16_t cap, uint16_t seq,
                                      uint8_t mask, const float q[4]) {
    if (cap < CCROS_HDR_SIZE + CCROS_POSITION_PAYLOAD) {
        return -1;
    }
    CcrosPutHdr(buf, CCROS_TYPE_POSITION, CCROS_POSITION_PAYLOAD);
    uint8_t *p = buf + CCROS_HDR_SIZE;
    CcrosPutU16(p + 0, seq);
    p[2] = mask;
    p[3] = 0;
    CcrosPutFloats(p + 4, q);
    return CCROS_HDR_SIZE + CCROS_POSITION_PAYLOAD;
}

static inline int CcrosEncodeVelocity(uint8_t *buf, uint16_t cap, uint16_t seq,
                                      uint8_t mask, const float v[4]) {
    if (cap < CCROS_HDR_SIZE + CCROS_VELOCITY_PAYLOAD) {
        return -1;
    }
    CcrosPutHdr(buf, CCROS_TYPE_VELOCITY, CCROS_VELOCITY_PAYLOAD);
    uint8_t *p = buf + CCROS_HDR_SIZE;
    CcrosPutU16(p + 0, seq);
    p[2] = mask;
    p[3] = 0;
    CcrosPutFloats(p + 4, v);
    return CCROS_HDR_SIZE + CCROS_VELOCITY_PAYLOAD;
}

static inline int CcrosEncodeTrack(uint8_t *buf, uint16_t cap, uint16_t seq,
                                   uint8_t mask, const float q[4], const float v[4]) {
    if (cap < CCROS_HDR_SIZE + CCROS_TRACK_PAYLOAD) {
        return -1;
    }
    CcrosPutHdr(buf, CCROS_TYPE_TRACK, CCROS_TRACK_PAYLOAD);
    uint8_t *p = buf + CCROS_HDR_SIZE;
    CcrosPutU16(p + 0, seq);
    p[2] = mask;
    p[3] = 0;
    CcrosPutFloats(p + 4, q);
    CcrosPutFloats(p + 20, v);
    return CCROS_HDR_SIZE + CCROS_TRACK_PAYLOAD;
}

static inline int CcrosEncodeHeartbeat(uint8_t *buf, uint16_t cap, uint16_t seq) {
    if (cap < CCROS_HDR_SIZE + CCROS_HEARTBEAT_PAYLOAD) {
        return -1;
    }
    CcrosPutHdr(buf, CCROS_TYPE_HEARTBEAT, CCROS_HEARTBEAT_PAYLOAD);
    CcrosPutU16(buf + CCROS_HDR_SIZE, seq);
    return CCROS_HDR_SIZE + CCROS_HEARTBEAT_PAYLOAD;
}

/* OK writes *consumed. RESYNC asks the caller to drop one byte. NEED_MORE waits. */
static inline int CcrosDecode(const uint8_t *buf, uint16_t len, CcrosDecoded *out,
                              uint16_t *consumed) {
    *consumed = 0;
    if (len == 0) {
        return CCROS_DECODE_NEED_MORE;
    }
    if (buf[0] != CCROS_MAGIC) {
        return CCROS_DECODE_RESYNC;
    }
    if (len < CCROS_HDR_SIZE) {
        return CCROS_DECODE_NEED_MORE;
    }
    if (buf[1] != CCROS_WIRE_VERSION) {
        return CCROS_DECODE_RESYNC;
    }
    const uint8_t type = buf[2];
    const uint16_t plen = CcrosGetU16(buf + 4);
    if (plen > CCROS_STATE_PAYLOAD) {
        return CCROS_DECODE_RESYNC;
    }
    if ((uint16_t)(len - CCROS_HDR_SIZE) < plen) {
        return CCROS_DECODE_NEED_MORE;
    }
    if (type == CCROS_TYPE_STATE && plen != CCROS_STATE_PAYLOAD) {
        return CCROS_DECODE_RESYNC;
    }
    if ((type == CCROS_TYPE_POSITION || type == CCROS_TYPE_VELOCITY) &&
        plen != CCROS_POSITION_PAYLOAD) {
        return CCROS_DECODE_RESYNC;
    }
    if (type == CCROS_TYPE_HEARTBEAT && plen != CCROS_HEARTBEAT_PAYLOAD) {
        return CCROS_DECODE_RESYNC;
    }
    if (type == CCROS_TYPE_TRACK && plen != CCROS_TRACK_PAYLOAD) {
        return CCROS_DECODE_RESYNC;
    }
    memset(out, 0, sizeof(*out));
    out->type = type;
    const uint8_t *p = buf + CCROS_HDR_SIZE;
    if (type == CCROS_TYPE_STATE) {
        out->time_ms = CcrosGetU32(p + 0);
        out->seq = CcrosGetU16(p + 4);
        out->flags = p[6];
        out->mask = p[7];
        out->alert_reg = CcrosGetU32(p + 8);
        CcrosGetFloats(p + 12, out->a);
        CcrosGetFloats(p + 28, out->b);
        CcrosGetFloats(p + 44, out->c);
        out->track_mask = p[60];
        CcrosGetFloats(p + 64, out->target_position);
        CcrosGetFloats(p + 80, out->target_velocity);
        CcrosGetFloats(p + 96, out->command_velocity);
        for (int i = 0; i < 4; i++) {
            out->target_latch_ms[i] = CcrosGetU32(p + 112 + (i * 4));
        }
    } else if (type == CCROS_TYPE_POSITION || type == CCROS_TYPE_VELOCITY) {
        out->seq = CcrosGetU16(p + 0);
        out->mask = p[2];
        CcrosGetFloats(p + 4, out->a);
    } else if (type == CCROS_TYPE_TRACK) {
        out->seq = CcrosGetU16(p + 0);
        out->mask = p[2];
        CcrosGetFloats(p + 4, out->a);
        CcrosGetFloats(p + 20, out->b);
    } else if (type == CCROS_TYPE_HEARTBEAT) {
        out->seq = CcrosGetU16(p);
    }
    *consumed = (uint16_t)(CCROS_HDR_SIZE + plen);
    return CCROS_DECODE_OK;
}

#endif
