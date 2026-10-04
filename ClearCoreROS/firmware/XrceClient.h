/* MIT License. Copyright (c) 2026 Adam G. Sweeney <agsweeney@gmail.com> */

#ifndef __CCROS_XRCE_CLIENT_H__
#define __CCROS_XRCE_CLIENT_H__

#include <stdint.h>

/* Publish joint_x/y/z/a as sensor_msgs/JointState to a micro-ROS agent. */
const char *XrceConnect(const uint8_t ip[4], uint16_t port);
const char *XrceDisconnect();
void XrcePoll();
const char *XrceStateName();
bool XrceTimeSynced();

#endif
