// EXCERPT — source: ClearCNC_Controller/ClearCoreFirmware/ClearCNC_Firmware.cpp
// EVIDENCE: E1 | symbol: MOTION_QUEUE_SIZE / ETHERNET ports | lines: 64-96

#define SERIAL_BAUD_RATE 115200
// Long enough for CONFIG with SPMMX/Y, VEL, ACCEL/DECEL, DVMAX, SINGLE, AX, LASER, RVELX/Y/Z/A (~160+ chars).
#define MAX_LINE_LENGTH 256

#define MOTOR_X_STEPS_PER_REV 800
#define MOTOR_X_PITCH_MM 5.0
#define MOTOR_Y_STEPS_PER_REV 800
#define MOTOR_Y_PITCH_MM 5.0

#define DEFAULT_FEED_MM_PER_MIN 500.0
#define DEFAULT_FEED_IN_PER_MIN 20.0
#define DEFAULT_VELOCITY_STEPS_PER_SEC 5000
#define DEFAULT_ACCEL_STEPS_PER_SEC2 85000
#define DEFAULT_STOP_DECEL_STEPS_PER_SEC2 85000
#define WAIT_FOR_USB_CONNECTION 1
#define AXIS_COUNT 4
#define MOTION_QUEUE_SIZE 16
// Minimum firmware blocks queued before starting motion (gives lookahead a head-start).
// Streaming refill (TryExtendCoordinatedBatch) handles mid-program continuity, so this
// only needs to be large enough for a meaningful initial lookahead.
#define QUEUE_START_DEPTH 4
// After this many ms, start even if the depth hasn't been reached (handles short programs
// or hosts that are slow to respond).
#define QUEUE_START_TIMEOUT_MS 100
#define ETHERNET_CONTROL_PORT 8888
#define ETHERNET_TELEMETRY_PORT 8889
#define TELEMETRY_INTERVAL_MS 50
#define DISCOVERY_PORT 10040
#define DISCOVERY_REQUEST "CLEARCNC_DISCOVER?"
// Fanuc G4 P is ms; cap blocking dwell to avoid locking the link indefinitely.
#define G4_DWELL_MS_MAX 600000u
// ClearCore OutputCurrent() accepts 0..20000 µA (0..20 mA) per library.
#define SPINDLE_OUTPUT_MAX_UA 20000u
