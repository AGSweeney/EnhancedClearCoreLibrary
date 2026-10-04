/* MIT License. Copyright (c) 2026 Adam G. Sweeney <agsweeney@gmail.com> */
/* Host-side check that firmware/RosProtocol.h matches host/test_wire.py. */

#include <stdio.h>
#include <string.h>

#include "../firmware/RosProtocol.h"

static void PrintHex(const uint8_t *buf, int n) {
    for (int i = 0; i < n; i++) {
        printf("%02x", buf[i]);
    }
    printf("\n");
}

static int Fail(const char *msg) {
    fprintf(stderr, "%s\n", msg);
    return 1;
}

int main() {
    const float q[4] = {0.01f, -0.02f, 0.0f, 1.0f};
    uint8_t frame[CCROS_MAX_FRAME];
    const int pn = CcrosEncodePosition(frame, sizeof(frame), 7, 0x03, q);
    if (pn != CCROS_HDR_SIZE + CCROS_POSITION_PAYLOAD) {
        return Fail("position length");
    }
    PrintHex(frame, pn);

    CcrosDecoded decoded;
    uint16_t consumed = 0;
    if (CcrosDecode(frame, (uint16_t)pn, &decoded, &consumed) != CCROS_DECODE_OK) {
        return Fail("position decode");
    }
    if (consumed != (uint16_t)pn || decoded.type != CCROS_TYPE_POSITION ||
        decoded.seq != 7 || decoded.mask != 0x03 || decoded.a[0] != q[0] ||
        decoded.a[1] != q[1] || decoded.a[3] != q[3]) {
        return Fail("position fields");
    }

    CcrosState state;
    memset(&state, 0, sizeof(state));
    state.time_ms = 1000;
    state.seq = 3;
    state.flags = CCROS_FLAG_ENABLED | CCROS_FLAG_MOVING;
    state.axis_mask = 0x01;
    state.alert_reg = 0;
    state.position[0] = 0.01f;
    state.effort[0] = 0.5f;
    const int sn = CcrosEncodeState(frame, sizeof(frame), &state);
    if (sn != CCROS_HDR_SIZE + CCROS_STATE_PAYLOAD) {
        return Fail("state length");
    }
    PrintHex(frame, sn);
    if (CcrosDecode(frame, (uint16_t)sn, &decoded, &consumed) != CCROS_DECODE_OK ||
        decoded.time_ms != 1000 || decoded.seq != 3 || decoded.flags != 0x03 ||
        decoded.mask != 0x01 || decoded.a[0] != 0.01f || decoded.c[0] != 0.5f) {
        return Fail("state fields");
    }

    uint8_t junked[80];
    junked[0] = 0x00;
    memcpy(junked + 1, frame, (size_t)sn);
    uint16_t off = 0;
    const uint16_t total = (uint16_t)(sn + 1);
    int saw = 0;
    while (off < total) {
        uint16_t used = 0;
        const int st = CcrosDecode(junked + off, (uint16_t)(total - off), &decoded, &used);
        if (st == CCROS_DECODE_NEED_MORE) {
            return Fail("resync stalled");
        }
        if (st == CCROS_DECODE_RESYNC) {
            off += 1;
            continue;
        }
        if (decoded.type == CCROS_TYPE_STATE && decoded.seq == 3) {
            saw = 1;
        }
        off = (uint16_t)(off + used);
    }
    if (!saw) {
        return Fail("resync missed state");
    }
    printf("ok\n");
    return 0;
}
