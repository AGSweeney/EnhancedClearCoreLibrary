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

#ifndef __CLEARAI_CONFIG_H__
#define __CLEARAI_CONFIG_H__

#include <stdint.h>

#define CLEARAI_PROTOCOL_VERSION "1.0"
#define CLEARAI_SERIAL_BAUD 115200
#define CLEARAI_MAX_LINE 512
#define CLEARAI_MAX_REPLY 1024
#define CLEARAI_AXIS_COUNT 4
#define CLEARAI_TCP_CONTROL_PORT 9100
#define CLEARAI_TCP_TELEMETRY_PORT 9101
#define CLEARAI_UDP_DISCOVERY_PORT 9102
#define CLEARAI_DISCOVERY_REQUEST "CLEARAI_DISCOVER?"
#define CLEARAI_TELEMETRY_INTERVAL_MS 50
#define CLEARAI_WAIT_USB_MS 5000
#define CLEARAI_DWELL_MS_MAX 600000u
/* Homing/probing defaults (work units and ms). */
#define CLEARAI_HOME_SEEK_DEFAULT 1000.0
#define CLEARAI_HOME_TIMEOUT_MS 30000u
#define CLEARAI_PROBE_TIMEOUT_MS 30000u
#define CLEARAI_SEEK_BACKOFF_MS 60000u
#define CLEARAI_INPUT_DEBOUNCE_DEFAULT_MS 5u
#define CLEARAI_DEFAULT_STEPS_PER_REV 800u
#define CLEARAI_DEFAULT_PITCH_MM 5.0
/* 800 ppr × 2025 rpm ≈ 27000 steps/s (motor rated 2000+ rpm, unloaded bench). */
#define CLEARAI_DEFAULT_VEL_STEPS 27000u
#define CLEARAI_DEFAULT_ACCEL_STEPS 250000u
#define CLEARAI_DEFAULT_DECEL_STEPS 250000u
#define CLEARAI_DEFAULT_FEED_MM_PER_MIN 500.0
#define CLEARAI_DEFAULT_AXIS_MASK 0x3u
#define CLEARAI_ENABLE_HLFB_WAIT_MS 500u
/* 1 = boot with all motion gates off (bench). 0 = normal (default). */
#ifndef CLEARAI_TEST_MODE_DEFAULT
#define CLEARAI_TEST_MODE_DEFAULT 0
#endif

enum ClearAiAxis {
    CLEARAI_AXIS_X = 0,
    CLEARAI_AXIS_Y = 1,
    CLEARAI_AXIS_Z = 2,
    CLEARAI_AXIS_A = 3
};

enum ClearAiUnits {
    CLEARAI_UNITS_MM = 0,
    CLEARAI_UNITS_INCH = 1
};

/* Axis A is rotary; user-facing unit is degrees or revolutions. Internal stays degrees. */
enum ClearAiUnitsA {
    CLEARAI_UNITS_A_DEG = 0,
    CLEARAI_UNITS_A_REV = 1
};

enum ClearAiMode {
    CLEARAI_MODE_ABS = 0,
    CLEARAI_MODE_REL = 1
};

#endif
