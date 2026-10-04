/* MIT License. Copyright (c) 2026 Adam G. Sweeney <agsweeney@gmail.com> */

#include "Transport.h"

#include "ClearCore.h"
#include "EthernetTcpClient.h"
#include "EthernetTcpServer.h"
#include "EthernetUdp.h"
#include "MotionCore.h"
#include "RosConfig.h"
#include "XrceClient.h"
#include "RosProtocol.h"
#include "SysTiming.h"

#include <stdio.h>
#include <string.h>

#define SerialPort ConnectorUsb

static EthernetTcpServer g_sessionServer(CCROS_TCP_SESSION_PORT);
static EthernetTcpServer g_streamServer(CCROS_TCP_STREAM_PORT);
static EthernetTcpClient g_sessionClient;
static EthernetTcpClient g_streamClient;
static EthernetUdp g_discoveryUdp;
static bool g_ethernetReady = false;
static bool g_sessionConnected = false;
static bool g_streamConnected = false;

static char g_usbLine[CCROS_MAX_LINE];
static uint16_t g_usbIndex = 0;
static char g_tcpLine[CCROS_MAX_LINE];
static uint16_t g_tcpIndex = 0;

static uint8_t g_rx[256];
static uint16_t g_rxLen = 0;
static uint32_t g_lastStateMs = 0;
static uint16_t g_stateSeq = 0;

static bool ReadFromPort(bool usb, char *outLine, uint16_t maxLen) {
    char *buf = usb ? g_usbLine : g_tcpLine;
    uint16_t *idx = usb ? &g_usbIndex : &g_tcpIndex;
    while (true) {
        int16_t ch = -1;
        if (usb) {
            ch = SerialPort.CharGet();
            if (ch < 0) {
                return false;
            }
        } else {
            if (!g_sessionConnected || !g_sessionClient.Connected()) {
                return false;
            }
            if (g_sessionClient.BytesAvailable() <= 0) {
                return false;
            }
            ch = g_sessionClient.Read();
        }
        if (ch < 0) {
            return false;
        }
        if (ch == '\n' || ch == '\r') {
            if (*idx == 0) {
                continue;
            }
            buf[*idx] = '\0';
            *idx = 0;
            strncpy(outLine, buf, maxLen - 1);
            outLine[maxLen - 1] = '\0';
            return true;
        }
        if (*idx < CCROS_MAX_LINE - 1) {
            buf[(*idx)++] = (char)ch;
        } else {
            *idx = 0;
        }
    }
}

void TransportInitUsb() {
    SerialPort.Mode(Connector::USB_CDC);
    SerialPort.Speed(CCROS_SERIAL_BAUD);
    SerialPort.PortOpen();
    const uint32_t start = Milliseconds();
    while (!SerialPort && Milliseconds() - start < CCROS_WAIT_USB_MS) {
        continue;
    }
    Delay_ms(100);
}

void TransportInitEthernet() {
    if (g_ethernetReady || !EthernetMgr.PhyLinkActive()) {
        return;
    }
    EthernetMgr.Setup();
    uint8_t netMode = 0;
    uint8_t ip[4] = {0, 0, 0, 0};
    uint8_t nm[4] = {0, 0, 0, 0};
    uint8_t gw[4] = {0, 0, 0, 0};
    MotionGetNetworkConfig(&netMode, ip, nm, gw);
    if (netMode == 1) {
        EthernetMgr.LocalIp(IpAddress(ip[0], ip[1], ip[2], ip[3]));
        EthernetMgr.NetmaskIp(IpAddress(nm[0], nm[1], nm[2], nm[3]));
        EthernetMgr.GatewayIp(IpAddress(gw[0], gw[1], gw[2], gw[3]));
    } else if (!EthernetMgr.DhcpBegin()) {
        EthernetMgr.LocalIp(IpAddress(192, 168, 0, 109));
    }
    g_sessionServer.Begin();
    g_streamServer.Begin();
    g_discoveryUdp.Begin(CCROS_UDP_DISCOVERY_PORT);
    g_ethernetReady = true;
    SerialPort.Send("ClearCoreROS IP=");
    SerialPort.SendLine(EthernetMgr.LocalIp().StringValue());
}

static void PollSessionAccept() {
    if (!g_sessionConnected || !g_sessionClient.Connected()) {
        EthernetTcpClient next = g_sessionServer.Accept();
        if (next.Connected()) {
            g_sessionClient = next;
            g_sessionConnected = true;
            g_tcpIndex = 0;
        } else {
            g_sessionConnected = false;
        }
    }
}

static void PollStream() {
    const bool was = g_streamConnected && g_streamClient.Connected();
    if (!was) {
        EthernetTcpClient next = g_streamServer.Accept();
        if (next.Connected()) {
            g_streamClient = next;
            g_streamConnected = true;
            g_rxLen = 0;
            g_lastStateMs = 0;
        } else {
            if (g_streamConnected) {
                MotionStreamLost();
            }
            g_streamConnected = false;
            return;
        }
    }

    while (g_streamClient.BytesAvailable() > 0 && g_rxLen < sizeof(g_rx)) {
        const int16_t ch = g_streamClient.Read();
        if (ch < 0) {
            break;
        }
        g_rx[g_rxLen++] = (uint8_t)ch;
    }
    while (g_rxLen > 0) {
        CcrosDecoded decoded;
        uint16_t consumed = 0;
        const int st = CcrosDecode(g_rx, g_rxLen, &decoded, &consumed);
        if (st == CCROS_DECODE_NEED_MORE) {
            break;
        }
        if (st != CCROS_DECODE_OK || consumed == 0 || consumed > g_rxLen) {
            memmove(g_rx, g_rx + 1, g_rxLen - 1);
            g_rxLen--;
            continue;
        }
        memmove(g_rx, g_rx + consumed, g_rxLen - consumed);
        g_rxLen = (uint16_t)(g_rxLen - consumed);
        if (decoded.type == CCROS_TYPE_POSITION) {
            MotionNotePosition(decoded.seq, decoded.mask, decoded.a);
        } else if (decoded.type == CCROS_TYPE_VELOCITY) {
            MotionNoteVelocity(decoded.seq, decoded.mask, decoded.a);
        } else if (decoded.type == CCROS_TYPE_TRACK) {
            MotionNoteTrack(decoded.seq, decoded.mask, decoded.a, decoded.b);
        } else if (decoded.type == CCROS_TYPE_HEARTBEAT) {
            MotionNoteHost();
        }
    }

    const uint32_t now = Milliseconds();
    if (g_lastStateMs != 0 && (now - g_lastStateMs) < CCROS_STREAM_PERIOD_MS) {
        return;
    }
    g_lastStateMs = now;
    CcrosState state;
    MotionFillState(&state);
    state.seq = g_stateSeq++;
    uint8_t frame[CCROS_MAX_FRAME];
    const int n = CcrosEncodeState(frame, sizeof(frame), &state);
    if (n > 0) {
        g_streamClient.Send(frame, (uint32_t)n);
    }
}

static void PollDiscovery() {
    const uint16_t packetSize = g_discoveryUdp.PacketParse();
    if (packetSize == 0) {
        return;
    }
    unsigned char packet[96];
    const int32_t n = g_discoveryUdp.PacketRead(packet, sizeof(packet) - 1);
    if (n <= 0) {
        return;
    }
    packet[n] = '\0';
    if (strncmp((const char *)packet, CCROS_DISCOVERY_REQUEST,
                strlen(CCROS_DISCOVERY_REQUEST)) != 0) {
        return;
    }
    char response[160];
    snprintf(response, sizeof(response),
             "CLEARCORE_ROS %s IP=%s TCP=%u STREAM=%u FW=%s",
             CCROS_FIRMWARE_NAME, EthernetMgr.LocalIp().StringValue(),
             (unsigned)CCROS_TCP_SESSION_PORT, (unsigned)CCROS_TCP_STREAM_PORT,
             CCROS_PROTOCOL_VERSION);
    g_discoveryUdp.Connect(g_discoveryUdp.RemoteIp(), g_discoveryUdp.RemotePort());
    g_discoveryUdp.PacketWrite(response);
    g_discoveryUdp.PacketSend();
}

void TransportPoll() {
    if (!g_ethernetReady) {
        TransportInitEthernet();
        return;
    }
    PollSessionAccept();
    PollStream();
    PollDiscovery();
    XrcePoll();
}

void TransportSendLine(const char *line) {
    SerialPort.SendLine(line);
    if (g_sessionConnected && g_sessionClient.Connected()) {
        g_sessionClient.Send(line);
        g_sessionClient.Send("\r\n");
    }
}

bool TransportReadLine(char *outLine, uint16_t maxLen) {
    if (ReadFromPort(true, outLine, maxLen)) {
        return true;
    }
    if (ReadFromPort(false, outLine, maxLen)) {
        return true;
    }
    return false;
}
