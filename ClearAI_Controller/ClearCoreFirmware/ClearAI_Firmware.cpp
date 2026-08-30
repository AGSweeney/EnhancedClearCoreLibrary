/*
 * ClearAI_Firmware
 *
 * JSON-RPC Lines command firmware so a host LLM can drive a ClearPath XY gantry
 * (optional Z/A) through Code as Policies primitives. Not G-code.
 *
 * Transport:
 *   USB CDC 115200
 *   TCP control 9100
 *   TCP telemetry 9101 (status notifications)
 *   UDP discovery 9102  ("CLEARAI_DISCOVER?")
 *
 * Motor setup (ClearPath MSP):
 *   Step and Direction (or QUAD_AB if using that library mode)
 *   HLFB: ASG-Position w/Measured Torque, 482 Hz
 *   Input Format: Step + Direction
 *
 * See ClearAI_Controller/PROTOCOL.md
 *
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

#include "ClearCore.h"
#include "MotionBridge.h"
#include "Primitives.h"
#include "SysTiming.h"
#include "Transport.h"

int main(void) {
    Delay_ms(300);
    TransportInitUsb();

    if (!MotionInit()) {
        TransportSendLine(
            "{\"jsonrpc\":\"2.0\",\"method\":\"fault\",\"params\":{\"message\":\"motion init failed\"}}");
    } else {
        TransportSendLine(
            "{\"jsonrpc\":\"2.0\",\"method\":\"status\",\"params\":{\"ready\":true,\"hint\":\"call get_capabilities\"}}");
    }

    TransportInitEthernet();

    char line[CLEARAI_MAX_LINE];
    while (true) {
        TransportPollEthernet();
        TransportPollDiscovery();
        TransportPollTelemetry();
        MotionPollEstop();
        if (TransportReadLine(line, sizeof(line))) {
            PrimitivesDispatchLine(line);
        }
        Delay_ms(1);
    }
}
