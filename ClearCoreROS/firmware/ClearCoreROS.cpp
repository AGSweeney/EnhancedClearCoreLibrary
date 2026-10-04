/* MIT License. Copyright (c) 2026 Adam G. Sweeney <agsweeney@gmail.com> */
/*
 * ClearCoreROS
 *
 * ClearCore firmware that exposes ClearPath M0..M3 as ROS 2 joints.
 * The board does not run DDS. A host bridge or ros2_control plugin speaks:
 *   USB CDC 115200     JSON-RPC Lines (same methods as TCP)
 *   TCP 9200           JSON-RPC session (one client)
 *   TCP 9201           little-endian joint state / command stream
 *   UDP 9202           "CLEARCORE_ROS_DISCOVER?"
 *
 * ClearPath MSP: Step and Direction, HLFB ASG-Position w/Measured Torque, 482 Hz.
 * See ClearCoreROS/PROTOCOL.md.
 */

#include "ClearCore.h"
#include "MotionCore.h"
#include "RosConfig.h"
#include "Session.h"
#include "Transport.h"

int main(void) {
    Delay_ms(300);
    TransportInitUsb();
    if (!MotionInit()) {
        TransportSendLine(
            "{\"jsonrpc\":\"2.0\",\"method\":\"fault\",\"params\":{\"message\":\"motion init failed\"}}");
    } else {
        TransportSendLine(
            "{\"jsonrpc\":\"2.0\",\"method\":\"status\",\"params\":{\"ready\":true,"
            "\"firmware\":\"ClearCoreROS\",\"hint\":\"call get_capabilities\"}}");
    }
    TransportInitEthernet();

    char line[CCROS_MAX_LINE];
    while (true) {
        TransportPoll();
        MotionPoll();
        if (TransportReadLine(line, sizeof(line))) {
            SessionDispatch(line);
        }
        Delay_ms(1);
    }
}
