/* MIT License. Copyright (c) 2026 Adam G. Sweeney <agsweeney@gmail.com> */
/*
 * Minimal XRCE-DDS 1.0 client. It speaks the same messages as
 * Micro-XRCE-DDS-Client v2.4.3: CREATE_CLIENT, CREATE with XML, WRITE_DATA.
 * The payload is sensor_msgs/JointState. Joint names come from the motor map.
 */

#include "XrceClient.h"

#include "ClearCore.h"
#include "MotionCore.h"
#include "RosConfig.h"

#include "lwip/udp.h"

#include <stdio.h>
#include <string.h>

static const uint8_t XRCE_SESSION = 0x81;
static const uint8_t XRCE_STREAM_RELIABLE = 0x80;
static const uint8_t XRCE_STREAM_BEST_EFFORT = 0x01;
static const uint8_t XRCE_KIND_PARTICIPANT = 0x01;
static const uint8_t XRCE_KIND_TOPIC = 0x02;
static const uint8_t XRCE_KIND_PUBLISHER = 0x03;
static const uint8_t XRCE_KIND_DATAWRITER = 0x05;
static const uint16_t XRCE_MTU = 512;
static const uint32_t XRCE_RETRY_MS = 1000;
static const uint32_t XRCE_PUBLISH_MS = 50;
static const uint32_t XRCE_HEARTBEAT_MS = 1000;
static const uint32_t XRCE_AGENT_LOST_MS = 3000;
static const uint32_t XRCE_TIME_SYNC_MS = 1000;

static const char kParticipantXml[] =
    "<dds><participant><rtps><name>clearcore_ros</name></rtps></participant></dds>";
static const char kTopicXml[] =
    "<dds><topic><name>rt/joint_states</name>"
    "<dataType>sensor_msgs::msg::dds_::JointState_</dataType></topic></dds>";
static const char kPublisherXml[] =
    "<dds><publisher name=\"clearcore_pub\"></publisher></dds>";
static const char kWriterXml[] =
    "<dds><data_writer><topic><kind>NO_KEY</kind><name>rt/joint_states</name>"
    "<dataType>sensor_msgs::msg::dds_::JointState_</dataType></topic></data_writer></dds>";

enum XrceState {
    XRCE_OFF = 0,
    XRCE_WAIT_AGENT,
    XRCE_CREATE_PART,
    XRCE_CREATE_TOPIC,
    XRCE_CREATE_PUB,
    XRCE_CREATE_WRITER,
    XRCE_STREAM
};

struct XrceBuf {
    uint8_t *data;
    uint16_t n;
    uint16_t cap;
};

/* A micro-ROS agent answers one request with several UDP datagrams. The
 * ClearCore UDP port keeps a single packet, so the client copies arrivals
 * into this queue from the lwIP callback. */
static const uint8_t XRCE_RX_SLOTS = 12;
static const uint16_t XRCE_RX_MAX = 768;
static struct udp_pcb *g_pcb = nullptr;
static uint8_t g_rxBuf[XRCE_RX_SLOTS][XRCE_RX_MAX];
static uint16_t g_rxLen[XRCE_RX_SLOTS];
static volatile uint8_t g_rxHead = 0;
static volatile uint8_t g_rxTail = 0;
static volatile uint8_t g_rxCount = 0;
static bool g_udpOpen = false;
static XrceState g_state = XRCE_OFF;
static uint8_t g_agent[4];
static uint16_t g_agentPort = CCROS_XRCE_AGENT_PORT;
static uint16_t g_request = 9;
static uint16_t g_reliableSeq = 0;
static uint16_t g_bestEffortSeq = 0;
static uint16_t g_waitRequest = 0;
static uint32_t g_lastSendMs = 0;
static uint32_t g_lastPubMs = 0;
static uint32_t g_lastRxMs = 0;
static uint32_t g_lastHeartbeatMs = 0;
static uint32_t g_lastTimeSyncMs = 0;
static uint32_t g_timeSyncT1Ms = 0;
static bool g_timeSynced = false;
static int64_t g_agentOffsetNs = 0;
static bool g_waiting = false;
static uint8_t g_clientKey[4] = {0x43, 0x52, 0x4F, 0x53};

static void Align(XrceBuf *b, uint16_t a) {
    while ((b->n % a) != 0 && b->n < b->cap) {
        b->data[b->n++] = 0;
    }
}

static void PutU8(XrceBuf *b, uint8_t v) {
    if (b->n < b->cap) {
        b->data[b->n++] = v;
    }
}

static void PutU16(XrceBuf *b, uint16_t v) {
    Align(b, 2);
    PutU8(b, (uint8_t)(v & 0xffu));
    PutU8(b, (uint8_t)(v >> 8));
}

static void PutU32(XrceBuf *b, uint32_t v) {
    Align(b, 4);
    PutU8(b, (uint8_t)(v & 0xffu));
    PutU8(b, (uint8_t)((v >> 8) & 0xffu));
    PutU8(b, (uint8_t)((v >> 16) & 0xffu));
    PutU8(b, (uint8_t)(v >> 24));
}

static void PutI16(XrceBuf *b, int16_t v) {
    PutU16(b, (uint16_t)v);
}

static void PutTime(XrceBuf *b, uint32_t ms) {
    PutU32(b, ms / 1000u);
    PutU32(b, (ms % 1000u) * 1000000u);
}

static int64_t TimeToNs(const uint8_t *p) {
    const int32_t sec = (int32_t)(p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) |
                                  ((uint32_t)p[3] << 24));
    const uint32_t nsec = (uint32_t)p[4] | ((uint32_t)p[5] << 8) | ((uint32_t)p[6] << 16) |
                          ((uint32_t)p[7] << 24);
    return (int64_t)sec * 1000000000LL + (int64_t)nsec;
}

static int64_t BoardNs(uint32_t ms) {
    return (int64_t)ms * 1000000LL;
}

static void ResetTimeSync() {
    g_timeSynced = false;
    g_agentOffsetNs = 0;
    g_timeSyncT1Ms = 0;
    g_lastTimeSyncMs = 0;
}

static void PutF64(XrceBuf *b, double v) {
    Align(b, 8);
    if (b->n + 8 <= b->cap) {
        memcpy(b->data + b->n, &v, 8);
        b->n = (uint16_t)(b->n + 8);
    }
}

static void PutStr(XrceBuf *b, const char *s) {
    const uint32_t len = (uint32_t)strlen(s) + 1u;
    PutU32(b, len);
    if (b->n + len <= b->cap) {
        memcpy(b->data + b->n, s, len);
        b->n = (uint16_t)(b->n + len);
    }
}

static void PutRaw(XrceBuf *b, const void *src, uint16_t len) {
    if (b->n + len <= b->cap) {
        memcpy(b->data + b->n, src, len);
        b->n = (uint16_t)(b->n + len);
    }
}

static void ObjectRaw(uint16_t id, uint8_t kind, uint8_t raw[2]) {
    raw[0] = (uint8_t)(id >> 4);
    raw[1] = (uint8_t)(((uint8_t)id << 4) | (kind & 0x0fu));
}

static void Header(XrceBuf *b, uint8_t stream, uint16_t seq) {
    PutU8(b, XRCE_SESSION);
    PutU8(b, stream);
    PutU16(b, seq);
}

static uint16_t BeginSub(XrceBuf *b, uint8_t id, uint8_t flags) {
    Align(b, 4);
    const uint16_t at = b->n;
    PutU8(b, id);
    PutU8(b, (uint8_t)(flags | 0x01u));
    PutU16(b, 0);
    return at;
}

static void EndSub(XrceBuf *b, uint16_t at) {
    const uint16_t len = (uint16_t)(b->n - (at + 4u));
    b->data[at + 2] = (uint8_t)(len & 0xffu);
    b->data[at + 3] = (uint8_t)(len >> 8);
    Align(b, 4);
}

static void XrceRecv(void *arg, struct udp_pcb *pcb, struct pbuf *p, const ip_addr_t *addr, u16_t port) {
    (void)arg;
    (void)pcb;
    (void)addr;
    (void)port;
    if (p == nullptr) {
        return;
    }
    if (g_rxCount < XRCE_RX_SLOTS && p->tot_len > 0 && p->tot_len <= XRCE_RX_MAX) {
        __disable_irq();
        pbuf_copy_partial(p, g_rxBuf[g_rxTail], p->tot_len, 0);
        g_rxLen[g_rxTail] = (uint16_t)p->tot_len;
        g_rxTail = (uint8_t)((g_rxTail + 1u) % XRCE_RX_SLOTS);
        g_rxCount++;
        __enable_irq();
    }
    pbuf_free(p);
}

static bool UdpSend(const uint8_t *data, uint16_t len) {
    if (!g_udpOpen || g_pcb == nullptr) {
        return false;
    }
    struct pbuf *p = pbuf_alloc(PBUF_TRANSPORT, len, PBUF_RAM);
    if (p == nullptr) {
        return false;
    }
    memcpy(p->payload, data, len);
    ip_addr_t dest;
    IP4_ADDR(&dest, g_agent[0], g_agent[1], g_agent[2], g_agent[3]);
    const err_t err = udp_sendto(g_pcb, p, &dest, g_agentPort);
    pbuf_free(p);
    EthernetMgr.Refresh();
    return err == ERR_OK;
}

static void SendCreateClient() {
    ResetTimeSync();
    uint8_t raw[64];
    XrceBuf b = {raw, 0, sizeof(raw)};
    /* CREATE_CLIENT is stamped with session_id & 0x80 (0x80 for this client).
     * That is the none-session without a header key. The real session id and
     * the client key are in the payload. A header of 0x01 is ignored by the agent. */
    PutU8(&b, (uint8_t)(XRCE_SESSION & 0x80u));
    PutU8(&b, 0);
    PutU16(&b, 0);
    const uint16_t sub = BeginSub(&b, 0, 0);
    PutRaw(&b, "XRCE", 4);
    PutU8(&b, 1);
    PutU8(&b, 0);
    PutU8(&b, 0x01);
    PutU8(&b, 0x0F);
    PutRaw(&b, g_clientKey, 4);
    PutU8(&b, XRCE_SESSION);
    PutU8(&b, 0);
    PutU16(&b, XRCE_MTU);
    EndSub(&b, sub);
    UdpSend(raw, b.n);
    g_lastSendMs = Milliseconds();
}

static uint16_t NextRequest() {
    g_request = (uint16_t)(g_request + 1u);
    return g_request;
}

static void SendCreate(uint8_t kind, const uint8_t selfRaw[2], const uint8_t refRaw[2], int16_t domain,
                       const char *xml) {
    uint8_t raw[640];
    XrceBuf b = {raw, 0, sizeof(raw)};
    Header(&b, XRCE_STREAM_RELIABLE, g_reliableSeq);
    const uint16_t sub = BeginSub(&b, 1, 0x04);
    PutU8(&b, (uint8_t)(g_waitRequest >> 8));
    PutU8(&b, (uint8_t)g_waitRequest);
    PutRaw(&b, selfRaw, 2);
    PutU8(&b, kind);
    PutU8(&b, 0x02);
    PutStr(&b, xml);
    if (kind == XRCE_KIND_PARTICIPANT) {
        PutI16(&b, domain);
    } else if (refRaw) {
        PutRaw(&b, refRaw, 2);
    }
    EndSub(&b, sub);
    const uint16_t beat = BeginSub(&b, 11, 0);
    PutU16(&b, 0);
    PutU16(&b, g_reliableSeq);
    PutU8(&b, XRCE_STREAM_RELIABLE);
    EndSub(&b, beat);
    UdpSend(raw, b.n);
    g_lastSendMs = Milliseconds();
}

static void SendReliableHeartbeat() {
    uint8_t raw[32];
    XrceBuf b = {raw, 0, sizeof(raw)};
    Header(&b, XRCE_STREAM_RELIABLE, g_reliableSeq);
    const uint16_t beat = BeginSub(&b, 11, 0);
    PutU16(&b, 0);
    PutU16(&b, g_reliableSeq);
    PutU8(&b, XRCE_STREAM_RELIABLE);
    EndSub(&b, beat);
    if (UdpSend(raw, b.n)) {
        g_reliableSeq = (uint16_t)(g_reliableSeq + 1u);
        g_lastHeartbeatMs = Milliseconds();
    }
}

static void SendJointState() {
    if (!g_timeSynced) {
        return;
    }
    CcrosState st;
    MotionFillState(&st);
    uint8_t raw[512];
    XrceBuf body = {raw, 0, sizeof(raw)};
    /* Fast DDS adds the CDR encapsulation. Alignment is from the start of this body.
     * Stamps are agent system time: agent_epoch_ns + (board_ms - t1_ms) * 1e6.
     * They do not follow ROS /clock. */
    int64_t stampNs = g_agentOffsetNs + BoardNs(st.time_ms);
    if (stampNs < 0) {
        stampNs = 0;
    }
    PutU32(&body, (uint32_t)(stampNs / 1000000000LL));
    PutU32(&body, (uint32_t)(stampNs % 1000000000LL));
    PutStr(&body, "");
    PutU32(&body, 4);
    PutStr(&body, MotionJointName(0));
    PutStr(&body, MotionJointName(1));
    PutStr(&body, MotionJointName(2));
    PutStr(&body, MotionJointName(3));
    PutU32(&body, 4);
    for (uint8_t i = 0; i < 4; i++) {
        PutF64(&body, st.position[i]);
    }
    PutU32(&body, 4);
    for (uint8_t i = 0; i < 4; i++) {
        PutF64(&body, st.velocity[i]);
    }
    PutU32(&body, 4);
    for (uint8_t i = 0; i < 4; i++) {
        PutF64(&body, st.effort[i]);
    }

    uint8_t frame[640];
    XrceBuf b = {frame, 0, sizeof(frame)};
    Header(&b, XRCE_STREAM_BEST_EFFORT, g_bestEffortSeq);
    const uint16_t sub = BeginSub(&b, 7, 0);
    const uint16_t req = NextRequest();
    PutU8(&b, (uint8_t)(req >> 8));
    PutU8(&b, (uint8_t)req);
    uint8_t writer[2];
    ObjectRaw(1, XRCE_KIND_DATAWRITER, writer);
    PutRaw(&b, writer, 2);
    PutRaw(&b, body.data, body.n);
    EndSub(&b, sub);
    if (UdpSend(frame, b.n)) {
        g_bestEffortSeq = (uint16_t)(g_bestEffortSeq + 1u);
        g_lastPubMs = Milliseconds();
    }
}

static void SendTimeSync() {
    uint8_t raw[48];
    XrceBuf b = {raw, 0, sizeof(raw)};
    Header(&b, XRCE_STREAM_RELIABLE, g_reliableSeq);
    const uint16_t sub = BeginSub(&b, 14, 0);
    const uint32_t t1 = Milliseconds();
    PutTime(&b, t1);
    EndSub(&b, sub);
    if (UdpSend(raw, b.n)) {
        g_timeSyncT1Ms = t1;
        g_lastTimeSyncMs = t1;
        /* CREATE already shares this sequence with HEARTBEAT. TIMESTAMP does
         * the same so a later heartbeat is not treated as a gap. */
    }
}

static void OnTimestampReply(const uint8_t *p, uint16_t len) {
    if (len < 24 || g_timeSyncT1Ms == 0) {
        return;
    }
    const int64_t t1 = TimeToNs(p);
    const int64_t t2 = TimeToNs(p + 8);
    const int64_t t3 = TimeToNs(p + 16);
    const int64_t t4 = BoardNs(Milliseconds());
    g_agentOffsetNs = ((t2 - t1) + (t3 - t4)) / 2;
    g_timeSynced = true;
    g_timeSyncT1Ms = 0;
}

static void StartCreate(XrceState next, bool newRequest) {
    uint8_t self[2];
    uint8_t part[2];
    uint8_t pub[2];
    ObjectRaw(1, XRCE_KIND_PARTICIPANT, part);
    ObjectRaw(1, XRCE_KIND_PUBLISHER, pub);
    if (newRequest) {
        g_waitRequest = NextRequest();
    }
    g_waiting = true;
    if (next == XRCE_CREATE_PART) {
        ObjectRaw(1, XRCE_KIND_PARTICIPANT, self);
        SendCreate(XRCE_KIND_PARTICIPANT, self, nullptr, 0, kParticipantXml);
    } else if (next == XRCE_CREATE_TOPIC) {
        ObjectRaw(1, XRCE_KIND_TOPIC, self);
        SendCreate(XRCE_KIND_TOPIC, self, part, 0, kTopicXml);
    } else if (next == XRCE_CREATE_PUB) {
        ObjectRaw(1, XRCE_KIND_PUBLISHER, self);
        SendCreate(XRCE_KIND_PUBLISHER, self, part, 0, kPublisherXml);
    } else if (next == XRCE_CREATE_WRITER) {
        ObjectRaw(1, XRCE_KIND_DATAWRITER, self);
        SendCreate(XRCE_KIND_DATAWRITER, self, pub, 0, kWriterXml);
    }
    g_state = next;
}

static void Advance() {
    g_waiting = false;
    g_reliableSeq = (uint16_t)(g_reliableSeq + 1u);
    if (g_state == XRCE_CREATE_PART) {
        StartCreate(XRCE_CREATE_TOPIC, true);
    } else if (g_state == XRCE_CREATE_TOPIC) {
        StartCreate(XRCE_CREATE_PUB, true);
    } else if (g_state == XRCE_CREATE_PUB) {
        StartCreate(XRCE_CREATE_WRITER, true);
    } else if (g_state == XRCE_CREATE_WRITER) {
        g_state = XRCE_STREAM;
        g_lastPubMs = 0;
        g_lastHeartbeatMs = Milliseconds();
        g_lastRxMs = Milliseconds();
    }
}

static void OnPayload(uint8_t id, const uint8_t *p, uint16_t len) {
    if (id == 4 && len >= 1 && g_state == XRCE_WAIT_AGENT) {
        if (p[0] == 0x00 || p[0] == 0x01) {
            g_reliableSeq = 0;
            g_request = 9;
            StartCreate(XRCE_CREATE_PART, true);
        }
        return;
    }
    if (id == 5 && len >= 5 && g_waiting) {
        const uint16_t req = (uint16_t)(((uint16_t)p[0] << 8) | p[1]);
        const uint8_t status = p[4];
        if (req == g_waitRequest && (status == 0x00 || status == 0x01 || status == 0x82)) {
            Advance();
        }
        return;
    }
    if (id == 15) {
        OnTimestampReply(p, len);
    }
}

static void ReadReplies() {
    if (!g_udpOpen) {
        return;
    }
    EthernetMgr.Refresh();
    while (g_rxCount > 0) {
        uint8_t buf[XRCE_RX_MAX];
        __disable_irq();
        const uint16_t take = g_rxLen[g_rxHead];
        memcpy(buf, g_rxBuf[g_rxHead], take);
        g_rxHead = (uint8_t)((g_rxHead + 1u) % XRCE_RX_SLOTS);
        g_rxCount--;
        __enable_irq();
        if (take < 8) {
            continue;
        }
        g_lastRxMs = Milliseconds();
        uint16_t i = 4;
        if (buf[0] < 0x80) {
            i = 8;
        }
        while (i + 4 <= take) {
            while ((i % 4u) != 0 && i < take) {
                i++;
            }
            if (i + 4 > take) {
                break;
            }
            const uint8_t mid = buf[i];
            const uint16_t len = (uint16_t)(buf[i + 2] | ((uint16_t)buf[i + 3] << 8));
            i = (uint16_t)(i + 4);
            if (i + len > take) {
                break;
            }
            OnPayload(mid, buf + i, len);
            i = (uint16_t)(i + len);
        }
    }
}

const char *XrceConnect(const uint8_t ip[4], uint16_t port) {
    if (port == 0) {
        return "agent port required";
    }
    if (g_udpOpen && g_pcb != nullptr) {
        udp_remove(g_pcb);
        g_pcb = nullptr;
        g_udpOpen = false;
    }
    memcpy(g_agent, ip, 4);
    g_agentPort = port;
    g_rxHead = 0;
    g_rxTail = 0;
    g_rxCount = 0;
    g_pcb = udp_new();
    if (g_pcb == nullptr) {
        g_state = XRCE_OFF;
        return "xrce udp bind failed";
    }
    ip_addr_t local = IPADDR4_INIT(uint32_t(EthernetMgr.LocalIp()));
    if (udp_bind(g_pcb, &local, CCROS_XRCE_LOCAL_PORT) != ERR_OK) {
        udp_remove(g_pcb);
        g_pcb = nullptr;
        g_state = XRCE_OFF;
        return "xrce udp bind failed";
    }
    udp_recv(g_pcb, XrceRecv, nullptr);
    g_udpOpen = true;
    g_state = XRCE_WAIT_AGENT;
    g_waiting = false;
    g_bestEffortSeq = 0;
    g_lastSendMs = 0;
    g_lastRxMs = 0;
    g_lastHeartbeatMs = 0;
    ResetTimeSync();
    SendCreateClient();
    return nullptr;
}

const char *XrceDisconnect() {
    if (g_udpOpen && g_pcb != nullptr) {
        udp_remove(g_pcb);
        g_pcb = nullptr;
        g_udpOpen = false;
    }
    g_state = XRCE_OFF;
    g_waiting = false;
    ResetTimeSync();
    return nullptr;
}

void XrcePoll() {
    if (g_state == XRCE_OFF) {
        return;
    }
    ReadReplies();
    const uint32_t now = Milliseconds();
    if (g_state == XRCE_WAIT_AGENT) {
        if ((now - g_lastSendMs) >= XRCE_RETRY_MS) {
            SendCreateClient();
        }
        return;
    }
    if (g_state == XRCE_STREAM) {
        if (g_lastRxMs != 0 && (now - g_lastRxMs) >= XRCE_AGENT_LOST_MS) {
            g_state = XRCE_WAIT_AGENT;
            g_waiting = false;
            SendCreateClient();
            return;
        }
        if (g_lastHeartbeatMs == 0 || (now - g_lastHeartbeatMs) >= XRCE_HEARTBEAT_MS) {
            SendReliableHeartbeat();
        }
        if (g_lastTimeSyncMs == 0 || (now - g_lastTimeSyncMs) >= XRCE_TIME_SYNC_MS) {
            SendTimeSync();
        }
        if (g_lastPubMs == 0 || (now - g_lastPubMs) >= XRCE_PUBLISH_MS) {
            SendJointState();
        }
        return;
    }
    if (g_waiting && (now - g_lastSendMs) >= XRCE_RETRY_MS) {
        StartCreate(g_state, false);
    }
}

const char *XrceStateName() {
    switch (g_state) {
        case XRCE_WAIT_AGENT: return "connecting";
        case XRCE_CREATE_PART:
        case XRCE_CREATE_TOPIC:
        case XRCE_CREATE_PUB:
        case XRCE_CREATE_WRITER: return "creating";
        case XRCE_STREAM: return "streaming";
        case XRCE_OFF:
        default: return "off";
    }
}

bool XrceTimeSynced() {
    return g_timeSynced;
}
