/* MIT License. Copyright (c) 2026 Adam G. Sweeney <agsweeney@gmail.com> */

#ifndef __CCROS_CONFIG_H__
#define __CCROS_CONFIG_H__

#include <stdint.h>

#define CCROS_PROTOCOL_VERSION "1.0"
#define CCROS_FIRMWARE_NAME "ClearCoreROS"
#define CCROS_SERIAL_BAUD 115200
#define CCROS_MAX_LINE 512
#define CCROS_MAX_REPLY 1024
#define CCROS_AXIS_COUNT 4

/* Clear of ClearAI 9100-9102 and ClearCNC 8888/8889/10040. */
#define CCROS_TCP_SESSION_PORT 9200
#define CCROS_TCP_STREAM_PORT 9201
#define CCROS_UDP_DISCOVERY_PORT 9202
#define CCROS_DISCOVERY_REQUEST "CLEARCORE_ROS_DISCOVER?"

#define CCROS_STREAM_PERIOD_MS 20u
#define CCROS_GOAL_STABLE_MS 40u
/* Timed tracking: steps/s of correction per step of schedule error. */
#define CCROS_TRACK_KP 8
#define CCROS_MOVE_RETRY_MS 20u
#define CCROS_WAIT_USB_MS 5000u
#define CCROS_ENABLE_HLFB_WAIT_MS 500u
#define CCROS_DEFAULT_WATCHDOG_MS 500u

#define CCROS_DEFAULT_STEPS_PER_REV 800u
#define CCROS_DEFAULT_PITCH_MM 5.0
#define CCROS_DEFAULT_VEL_STEPS 27000u
#define CCROS_DEFAULT_ACCEL_STEPS 250000u
#define CCROS_DEFAULT_DECEL_STEPS 250000u
#define CCROS_DEFAULT_AXIS_MASK 0x3u

/* 0 = estop input ignored, 1 = fault when DI-6 is low, 2 = fault when DI-6 is high. */
#define CCROS_DEFAULT_ESTOP_DI6 1u

#define CCROS_AXIS_X 0
#define CCROS_AXIS_Y 1
#define CCROS_AXIS_Z 2
#define CCROS_AXIS_A 3

#endif
