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

#include "Transport.h"

#include "ClearCore.h"
#include "EthernetTcpClient.h"
#include "EthernetTcpServer.h"
#include "EthernetUdp.h"
#include "JsonRpcLine.h"
#include "MotionBridge.h"
#include "SysTiming.h"

#include <stdio.h>
#include <string.h>

#define SerialPort ConnectorUsb

static EthernetTcpServer g_controlServer(CLEARAI_TCP_CONTROL_PORT);
static EthernetTcpServer g_telemetryServer(CLEARAI_TCP_TELEMETRY_PORT);
static EthernetTcpClient g_controlClient;
static EthernetTcpClient g_telemetryClient;
static EthernetUdp g_discoveryUdp;
static bool g_ethernetReady = false;
static bool g_controlConnected = false;
static bool g_telemetryConnected = false;
static char g_usbLine[CLEARAI_MAX_LINE];
static uint16_t g_usbIndex = 0;
static char g_tcpLine[CLEARAI_MAX_LINE];
static uint16_t g_tcpIndex = 0;
static uint32_t g_lastTelemetryMs = 0;

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
            if (!g_controlConnected || !g_controlClient.Connected()) {
                return false;
            }
            if (g_controlClient.BytesAvailable() <= 0) {
                return false;
            }
            ch = g_controlClient.Read();
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
        if (*idx < CLEARAI_MAX_LINE - 1) {
            buf[(*idx)++] = (char)ch;
        } else {
            *idx = 0;
        }
    }
}

void TransportInitUsb() {
    SerialPort.Mode(Connector::USB_CDC);
    SerialPort.Speed(CLEARAI_SERIAL_BAUD);
    SerialPort.PortOpen();
    const uint32_t start = Milliseconds();
    while (!SerialPort && Milliseconds() - start < CLEARAI_WAIT_USB_MS) {
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
    uint8_t ip[4] = {0};
    uint8_t nm[4] = {0};
    uint8_t gw[4] = {0};
    MotionGetNetworkConfig(&netMode, ip, nm, gw);
    if (netMode == 1) {
        /* Static IP: set addresses before Setup-driven bring-up; skip DHCP. */
        EthernetMgr.LocalIp(IpAddress(ip[0], ip[1], ip[2], ip[3]));
        EthernetMgr.NetmaskIp(IpAddress(nm[0], nm[1], nm[2], nm[3]));
        EthernetMgr.GatewayIp(IpAddress(gw[0], gw[1], gw[2], gw[3]));
    } else if (!EthernetMgr.DhcpBegin()) {
        EthernetMgr.LocalIp(IpAddress(192, 168, 0, 109));
    }
    g_controlServer.Begin();
    g_telemetryServer.Begin();
    g_discoveryUdp.Begin(CLEARAI_UDP_DISCOVERY_PORT);
    g_ethernetReady = true;
    SerialPort.Send("ClearAI Ethernet IP=");
    SerialPort.SendLine(EthernetMgr.LocalIp().StringValue());
}

void TransportPollEthernet() {
    if (!g_ethernetReady) {
        TransportInitEthernet();
        return;
    }
    if (!g_controlConnected || !g_controlClient.Connected()) {
        EthernetTcpClient next = g_controlServer.Accept();
        if (next.Connected()) {
            g_controlClient = next;
            g_controlConnected = true;
            g_tcpIndex = 0;
        } else {
            g_controlConnected = false;
        }
    }
}

void TransportPollDiscovery() {
    if (!g_ethernetReady) {
        return;
    }
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
    if (strncmp((const char *)packet, CLEARAI_DISCOVERY_REQUEST,
                strlen(CLEARAI_DISCOVERY_REQUEST)) != 0) {
        return;
    }
    char response[160];
    snprintf(response, sizeof(response),
             "CLEARAI_DISCOVER ClearAI_Firmware IP=%s TCP=%u TEL=%u FW=%s",
             EthernetMgr.LocalIp().StringValue(),
             CLEARAI_TCP_CONTROL_PORT, CLEARAI_TCP_TELEMETRY_PORT,
             CLEARAI_PROTOCOL_VERSION);
    g_discoveryUdp.Connect(g_discoveryUdp.RemoteIp(), g_discoveryUdp.RemotePort());
    g_discoveryUdp.PacketWrite(response);
    g_discoveryUdp.PacketSend();
}

void TransportPollTelemetry() {
    if (!g_ethernetReady) {
        return;
    }
    if (!g_telemetryConnected || !g_telemetryClient.Connected()) {
        EthernetTcpClient next = g_telemetryServer.Accept();
        if (next.Connected()) {
            g_telemetryClient = next;
            g_telemetryConnected = true;
            g_lastTelemetryMs = 0;
        } else {
            g_telemetryConnected = false;
            return;
        }
    }
    const uint32_t now = Milliseconds();
    if (now - g_lastTelemetryMs < CLEARAI_TELEMETRY_INTERVAL_MS) {
        return;
    }
    g_lastTelemetryMs = now;
    char pose[512];
    MotionFillTelemetryJson(pose, sizeof(pose));
    char line[CLEARAI_MAX_REPLY];
    JsonRpcFormatNotification(line, sizeof(line), "status", pose);
    TransportSendTelemetryLine(line);
}

void TransportSendLine(const char *line) {
    SerialPort.SendLine(line);
    if (g_controlConnected && g_controlClient.Connected()) {
        g_controlClient.Send(line);
        g_controlClient.Send("\r\n");
    }
}

void TransportSendTelemetryLine(const char *line) {
    if (g_telemetryConnected && g_telemetryClient.Connected()) {
        g_telemetryClient.Send(line);
        g_telemetryClient.Send("\r\n");
    }
}

bool TransportReadLine(char *outLine, uint16_t maxLen) {
    if (ReadFromPort(true, outLine, maxLen)) {
        return true;
    }
    TransportPollEthernet();
    if (ReadFromPort(false, outLine, maxLen)) {
        return true;
    }
    return false;
}

bool TransportEthernetReady() {
    return g_ethernetReady;
}
