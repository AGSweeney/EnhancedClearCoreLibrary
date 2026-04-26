/*
 * ClearCNC_Firmware
 *
 * Simple serial command firmware for ClearCore motion control.
 * Command protocol is line-based over USB CDC and designed to pair with
 * ClearCNC_Controller/QtController.
 *
 * Supported commands:
 *   HELP
 *   ENABLE | M202
 *   DISABLE | M203
 *   STOP | M201
 *   ESTOP | M200
 *   ABS | G90
 *   REL | G91
 *   UNITS MM | G21
 *   UNITS IN | G20
 *   ZERO | G92 [X..] [Y..] [Z..] [A..]  (work offset only; does not rewrite motor PositionRef)
 *   SYNCREF X=.. Y=.. Z=.. A=..  (DISABLE motors; set pulse refs to match ClearPath MSP counts after homing)
 *   POS | M114
 *   STATUS | M115
 *   MOVE X<num> Y<num> [F<num>] | G00/G0 (rapid, CONFIG Vel) | G01 X<num> Y<num> [F<num>]
 *   G02/G2 | G03/G3 X<num> Y<num> I<num> J<num> [F<num>]  (XY arc, center = start + I,J in active units)
 *   JOG X<num> Y<num> [F<num>] | G91 G01 X<num> Y<num> [F<num>]
 *
 * G01/G02/G03 in the X/Y plane: if CONFIG SINGLE=0 (default from host) uses CoordinatedMotionController
 * (vector feed / MoveArc). CONFIG SINGLE=1 = single-motor / independent M0 and M1 — G2/G3 are rejected.
 * Z/A always use per-stepper Move() on the remaining connectors. G2/G3 do not support Z/A on the same line.
 *
 * CONFIG AX=<0-15> bitmask: bit0=X,1=Y,2=Z,3=A. Inbound G0/G1 (and JOG) axis words for disabled
 * axes are ignored; only enabled axes are queued and no coordinate drift is applied for stripped words.
 */

#include "ClearCore.h"
#include "EthernetTcpServer.h"
#include "EthernetUdp.h"
#include "SysTiming.h"

#include <ctype.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define SerialPort ConnectorUsb
#define motorX ConnectorM0
#define motorY ConnectorM1

#define SERIAL_BAUD_RATE 115200
#define MAX_LINE_LENGTH 128

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

enum CoordinateMode {
    MODE_ABS,
    MODE_REL
};

enum UnitMode {
    UNITS_MM,
    UNITS_IN
};

enum AxisIndex {
    AXIS_X = 0,
    AXIS_Y = 1,
    AXIS_Z = 2,
    AXIS_A = 3
};

struct AxisConfig {
    bool enabled;
    uint8_t port;
    uint8_t axis;
    bool rotary;
    double stepsPerUnit; // steps/mm for linear, steps/degree for rotary
    uint32_t velocityMax;
    uint32_t accelMax;
    uint32_t decelMax;
};

enum MotionBlockKind : uint8_t {
    MOTION_BLOCK_LINEAR = 0,
    MOTION_BLOCK_ARC_XY = 1,
};

struct MotionBlock {
    bool valid;
    uint8_t axisMask;
    int32_t programLine;
    MotionBlockKind kind;
    int32_t target[AXIS_COUNT];
    int32_t delta[AXIS_COUNT];
    double unitDir[AXIS_COUNT];
    double lengthSteps;
    uint32_t nominalSpeed;
    uint32_t entrySpeed;
    uint32_t exitSpeed;
    // MOTION_BLOCK_ARC_XY: center/radius in steps; angles in radians.
    int32_t arcCenterX;
    int32_t arcCenterY;
    int32_t arcRadius;
    double arcStartAngle;
    double arcEndAngle;
    bool arcClockwise;
};

CoordinatedMotionController motionController;
MotorMechanicalConfig mechanicalConfigX;
MotorMechanicalConfig mechanicalConfigY;
EthernetTcpServer controlServer(ETHERNET_CONTROL_PORT);
EthernetTcpClient controlClient;
EthernetTcpServer telemetryServer(ETHERNET_TELEMETRY_PORT);
EthernetTcpClient telemetryClient;
EthernetUdp discoveryUdp;

static CoordinateMode coordinateMode = MODE_ABS;
static UnitMode unitMode = UNITS_MM;
static double feedRate = DEFAULT_FEED_MM_PER_MIN;
static double stepsPerMmX = (double)MOTOR_X_STEPS_PER_REV / MOTOR_X_PITCH_MM;
static double stepsPerMmY = (double)MOTOR_Y_STEPS_PER_REV / MOTOR_Y_PITCH_MM;
static uint32_t velocityMaxStepsPerSec = DEFAULT_VELOCITY_STEPS_PER_SEC;
static uint32_t accelMaxStepsPerSec2 = DEFAULT_ACCEL_STEPS_PER_SEC2;
static uint32_t stopDecelStepsPerSec2 = DEFAULT_STOP_DECEL_STEPS_PER_SEC2;
// dVmax: per-axis max velocity delta at junctions (Centroid-style corner speed cap).
// 0 = disabled (falls back to GRBL angle-based junction deviation).
// Suggested starting value: ~20% of velocityMax gives noticeable smoothing.
static uint32_t junctionDVmaxStepsPerSec = 0;
static bool motorsInitialized = false;
static bool motorsEnabled = false;
static bool ethernetReady = false;
static bool tcpClientConnected = false;
static bool telemetryClientConnected = false;
static AxisConfig axisConfig[AXIS_COUNT];
static int32_t currentSteps[AXIS_COUNT] = {0, 0, 0, 0};
static int32_t commandedSteps[AXIS_COUNT] = {0, 0, 0, 0};
static int32_t activeTargetSteps[AXIS_COUNT] = {0, 0, 0, 0};
// Work coordinates: machineSteps = workSteps + wcsOriginSteps (integer step space per axis).
static int32_t wcsOriginSteps[AXIS_COUNT] = {0, 0, 0, 0};
static uint8_t activeAxisMask = 0;
static MotionBlock motionQueue[MOTION_QUEUE_SIZE];
static uint8_t queueHead = 0;
static uint8_t queueTail = 0;
static uint8_t queueCount = 0;
static bool motionRunning = false;
static bool motionHold = false;
static bool queueStartPending = false;
static uint32_t queueFirstBlockTime = 0;
static bool motionActivityPendingDone = false;
static int32_t activeProgramLine = 0;
static uint32_t lastTelemetryTime = 0;
// Host: CONFIG SINGLE=0 → coordinated G01 in XY. CONFIG SINGLE=1 → independent M0/M1 (e.g. one motor on X).
static bool g_xyCoordinated = false;
// True for the in-flight block when XY was started with MoveLinear (not per-axis Move on M0/M1).
static bool s_blockUsedCoordinatedXy = false;
// Firmware blocks consumed in one StartMotionBlock (coordinated arc chains pop >1).
static uint8_t s_mergedBlocksToPop = 1;
// Streaming refill: endpoint and kind of the last firmware block submitted to the planner.
// Set by StartMotionBlock (last block of the initial batch) so TryExtendCoordinatedBatch
// can check contiguity correctly even after PopMergedBlocks has already advanced queueHead.
static int32_t s_streamLastEndX = 0;
static int32_t s_streamLastEndY = 0;
static MotionBlock s_streamLastBlock = {};

// Last committed arc fingerprint: allows streaming the next quadrant via QueueArc while
// the previous arc is still executing (host gets ok before motion finishes).
static bool s_coordArcStreamOpen = false;
static int32_t s_coordArcCx = 0;
static int32_t s_coordArcCy = 0;
static int32_t s_coordArcR = 0;
static bool s_coordArcCw = false;
static uint32_t s_coordArcNominal = 0;

static void InvalidateCoordinatedArcStream() {
    s_coordArcStreamOpen = false;
}

static bool CoordinatedArcMatchesStream(int32_t cx, int32_t cy, int32_t r, bool cw, uint32_t nom) {
    if (!s_coordArcStreamOpen || s_coordArcCw != cw || s_coordArcNominal != nom) {
        return false;
    }
    // Match committed fingerprint with small step tolerance (per-line I/J integer recompute).
    if (labs((long)(cx - s_coordArcCx)) > 2 || labs((long)(cy - s_coordArcCy)) > 2 ||
        labs((long)(r - s_coordArcR)) > 2) {
        return false;
    }
    return true;
}

static void CommitCoordinatedArcStream(int32_t cx, int32_t cy, int32_t r, bool cw, uint32_t nom) {
    s_coordArcStreamOpen = true;
    s_coordArcCx = cx;
    s_coordArcCy = cy;
    s_coordArcR = r;
    s_coordArcCw = cw;
    s_coordArcNominal = nom;
}

static char lineBuffer[MAX_LINE_LENGTH];
static uint16_t lineIndex = 0;
static char tcpLineBuffer[MAX_LINE_LENGTH];
static uint16_t tcpLineIndex = 0;

static MotorDriver *MotorForPort(uint8_t port) {
    switch (port) {
        case 0: return &ConnectorM0;
        case 1: return &ConnectorM1;
        case 2: return &ConnectorM2;
        case 3: return &ConnectorM3;
        default: return nullptr;
    }
}

static void SendLine(const char *msg) {
    if (tcpClientConnected && controlClient.Connected()) {
        controlClient.Send(msg);
        controlClient.Send("\r\n");
    }
    SerialPort.SendLine(msg);
}

static void SendTelemetryLine(const char *msg) {
    if (telemetryClientConnected && telemetryClient.Connected()) {
        telemetryClient.Send(msg);
        telemetryClient.Send("\r\n");
    }
}

static bool ParseValue(const char *line, char key, double &value) {
    const char keyUpper = (char)toupper((unsigned char)key);
    for (const char *p = line; *p != '\0'; p++) {
        if (toupper((unsigned char)*p) == keyUpper) {
            if (sscanf(p + 1, "%lf", &value) == 1) {
                return true;
            }
        }
    }
    return false;
}

static bool ParseNamedValue(const char *lineUpper, const char *name, double &value) {
    const char *p = strstr(lineUpper, name);
    if (!p) {
        return false;
    }
    p += strlen(name);
    if (*p == '=') {
        p++;
    }
    return sscanf(p, "%lf", &value) == 1;
}

static void PublishTelemetry(bool force);
static bool ReapplyKinematicsMode();

static void ToUpperInPlace(char *text) {
    for (char *p = text; *p != '\0'; p++) {
        *p = (char)toupper((unsigned char)*p);
    }
}

// True if the line requests a G0 rapid (G00 or G0 not G01/G02/...), anywhere in the line.
static bool MotionLineIsRapidG0(const char *lineUpper) {
    for (const char *p = lineUpper; *p != '\0';) {
        while (*p == ' ') {
            p++;
        }
        if (*p == '\0') {
            break;
        }
        if (p[0] == 'G' && p[1] == '0') {
            if (p[2] == '0') {
                if (p[3] == '\0' || isspace((unsigned char)p[3]) || !isdigit((unsigned char)p[3])) {
                    return true;
                }
            } else if (p[2] == '\0' || isspace((unsigned char)p[2]) || !isdigit((unsigned char)p[2])) {
                return true;
            }
        }
        while (*p != '\0' && *p != ' ') {
            p++;
        }
    }
    return false;
}

// Leading G17/G18/G19 only (XY/YZ/ZX plane select). Postprocessors emit G17 before arcs; the
// motion parser also matches strncmp(..., "G1", 2) which wrongly treats "G17" as G1 with no axes.
static int LeadingPlaneSelectFromLine(const char *lineUpper) {
    const char *p = lineUpper;
    while (*p == ' ') {
        p++;
    }
    if (p[0] != 'G' || p[1] != '1') {
        return 0;
    }
    if (p[2] == '7' && (p[3] == '\0' || isspace((unsigned char)p[3]) || p[3] == '.')) {
        return 17;
    }
    if (p[2] == '8' && (p[3] == '\0' || isspace((unsigned char)p[3]) || p[3] == '.')) {
        return 18;
    }
    if (p[2] == '9' && (p[3] == '\0' || isspace((unsigned char)p[3]) || p[3] == '.')) {
        return 19;
    }
    return 0;
}

// Returns: -1 = no arc, 0 = G2/G02 clockwise, 1 = G3/G03 counterclockwise.
// Skips G20/G21/G22/G23 by requiring G2/G3 not be followed by a digit.
static int MotionLineArcDirection(const char *lineUpper) {
    for (const char *p = lineUpper; *p != '\0';) {
        while (*p == ' ') {
            p++;
        }
        if (*p == '\0') {
            break;
        }
        if (p[0] == 'G') {
            if (p[1] == '0' && p[2] == '2' &&
                (p[3] == '\0' || isspace((unsigned char)p[3]) || p[3] == '.')) {
                return 0;
            }
            if (p[1] == '0' && p[2] == '3' &&
                (p[3] == '\0' || isspace((unsigned char)p[3]) || p[3] == '.')) {
                return 1;
            }
            if (p[1] == '2' && !isdigit((unsigned char)p[2])) {
                return 0;
            }
            if (p[1] == '3' && !isdigit((unsigned char)p[2])) {
                return 1;
            }
        }
        while (*p != '\0' && *p != ' ') {
            p++;
        }
    }
    return -1;
}

static void NormalizeAngle0TwoPi(double *a) {
    while (*a < 0.0) {
        *a += 2.0 * M_PI;
    }
    while (*a >= 2.0 * M_PI) {
        *a -= 2.0 * M_PI;
    }
}

// Span matching ArcInterpolator CCW/CW cases; angles in [0, 2*pi).
static double ArcSpanRadians(double startNorm, double endNorm, bool clockwise) {
    if (!clockwise) {
        if (endNorm > startNorm) {
            return endNorm - startNorm;
        }
        if (endNorm < startNorm) {
            return 2.0 * M_PI - (startNorm - endNorm);
        }
        return 2.0 * M_PI;
    }
    if (endNorm < startNorm) {
        return startNorm - endNorm;
    }
    if (endNorm > startNorm) {
        return 2.0 * M_PI - (endNorm - startNorm);
    }
    return 2.0 * M_PI;
}

static void SyncPlannerPositionFromMotors() {
    for (uint8_t axis = 0; axis < AXIS_COUNT; axis++) {
        if (!axisConfig[axis].enabled) {
            continue;
        }
        MotorDriver *m = MotorForPort(axisConfig[axis].port);
        int32_t s = currentSteps[axis];
        if (m) {
            s = (int32_t)m->PositionRefCommanded();
        }
        currentSteps[axis] = s;
        commandedSteps[axis] = s;
    }
    if (g_xyCoordinated && motorsInitialized) {
        motionController.SetPosition(currentSteps[AXIS_X], currentSteps[AXIS_Y]);
    }
}

static void ClearMotionQueueEx(bool syncPlannerFromMotors) {
    s_blockUsedCoordinatedXy = false;
    InvalidateCoordinatedArcStream();
    if (g_xyCoordinated && motorsInitialized) {
        motionController.Stop();
    }
    for (uint8_t i = 0; i < MOTION_QUEUE_SIZE; i++) {
        motionQueue[i].valid = false;
    }
    queueHead = 0;
    queueTail = 0;
    queueCount = 0;
    queueStartPending = false;
    motionRunning = false;
    motionHold = false;
    activeAxisMask = 0;
    motionActivityPendingDone = false;
    activeProgramLine = 0;
    if (syncPlannerFromMotors) {
        SyncPlannerPositionFromMotors();
    }
}

static void ClearMotionQueue() {
    ClearMotionQueueEx(false);
}

static void DisableMotorsAbrupt() {
    ClearMotionQueue();
    for (uint8_t axis = 0; axis < AXIS_COUNT; axis++) {
        if (MotorDriver *motor = MotorForPort(axisConfig[axis].port)) {
            motor->MoveStopAbrupt();
            motor->EnableRequest(false);
        }
    }
    motorsEnabled = false;
    PublishTelemetry(true);
}

static bool MotionQueueFull() {
    return queueCount >= MOTION_QUEUE_SIZE;
}

static bool MotionQueueEmpty() {
    return queueCount == 0;
}

static MotionBlock *QueueTailBlock() {
    if (MotionQueueFull()) {
        return nullptr;
    }
    return &motionQueue[queueTail];
}

static void CommitQueueTail() {
    motionQueue[queueTail].valid = true;
    queueTail = (queueTail + 1) % MOTION_QUEUE_SIZE;
    queueCount++;
    motionActivityPendingDone = true;
    if (!queueStartPending && !motionRunning) {
        queueStartPending = true;
        queueFirstBlockTime = Milliseconds();
    }
}

static void PopQueueHead() {
    if (queueCount == 0) {
        return;
    }
    motionQueue[queueHead].valid = false;
    queueHead = (queueHead + 1) % MOTION_QUEUE_SIZE;
    queueCount--;
    if (queueCount == 0) {
        queueStartPending = false;
    }
}

// Remove one logical block (offset 0 = queue head). Compacts the ring so order is preserved.
static void EraseQueueBlockAtOffset(uint8_t offsetFromHead) {
    if (offsetFromHead >= queueCount) {
        return;
    }
    const uint8_t n = queueCount;
    for (uint8_t j = offsetFromHead; j + 1 < n; j++) {
        uint8_t dst = (queueHead + j) % MOTION_QUEUE_SIZE;
        uint8_t src = (queueHead + j + 1) % MOTION_QUEUE_SIZE;
        motionQueue[dst] = motionQueue[src];
    }
    const uint8_t last = (queueHead + n - 1) % MOTION_QUEUE_SIZE;
    motionQueue[last].valid = false;
    queueCount--;
    if (queueCount == 0) {
        queueStartPending = false;
        queueHead = 0;
        queueTail = 0;
    } else {
        queueTail = (queueHead + queueCount) % MOTION_QUEUE_SIZE;
    }
}

static int32_t AxisValueToSteps(uint8_t axis, double value) {
    if (axis >= AXIS_COUNT) {
        return 0;
    }
    double valueInAxisUnits = value;
    if (!axisConfig[axis].rotary && unitMode == UNITS_IN) {
        valueInAxisUnits *= 25.4;
    }
    return (int32_t)((valueInAxisUnits * axisConfig[axis].stepsPerUnit) +
                     (valueInAxisUnits >= 0.0 ? 0.5 : -0.5));
}

// Increment in jog/G91/mm or inch user units → step delta (matches AxisValueToSteps scaling).
static int32_t UserDeltaToSteps(uint8_t axis, double deltaUserUnits) {
    if (axis >= AXIS_COUNT) {
        return 0;
    }
    double d = deltaUserUnits;
    if (!axisConfig[axis].rotary && unitMode == UNITS_IN) {
        d *= 25.4;
    }
    return (int32_t)((d * axisConfig[axis].stepsPerUnit) + (d >= 0.0 ? 0.5 : -0.5));
}

static double AxisStepsToDisplay(uint8_t axis, int32_t steps) {
    if (axis >= AXIS_COUNT || axisConfig[axis].stepsPerUnit <= 0.0) {
        return 0.0;
    }
    double axisUnits = (double)steps / axisConfig[axis].stepsPerUnit;
    if (!axisConfig[axis].rotary && unitMode == UNITS_IN) {
        axisUnits /= 25.4;
    }
    return axisUnits;
}

static int32_t AxisCurrentSteps(uint8_t axis) {
    if (axis >= AXIS_COUNT) {
        return 0;
    }
    if (g_xyCoordinated && motorsInitialized) {
        if (axis == AXIS_X) {
            return motionController.CurrentX();
        }
        if (axis == AXIS_Y) {
            return motionController.CurrentY();
        }
    }
    MotorDriver *motor = MotorForPort(axisConfig[axis].port);
    if (motor && axisConfig[axis].enabled) {
        return motor->PositionRefCommanded();
    }
    return currentSteps[axis];
}

static uint32_t FeedRateToStepsPerSec(double feedUnitsPerMin) {
    if (feedUnitsPerMin <= 0.0 || axisConfig[AXIS_X].stepsPerUnit <= 0.0) {
        return axisConfig[AXIS_X].velocityMax;
    }
    const double feedMmPerMin = (unitMode == UNITS_IN) ? (feedUnitsPerMin * 25.4) : feedUnitsPerMin;
    double stepsPerSec = (feedMmPerMin * axisConfig[AXIS_X].stepsPerUnit) / 60.0;
    if (stepsPerSec < 1.0) {
        stepsPerSec = 1.0;
    }
    if (stepsPerSec > (double)axisConfig[AXIS_X].velocityMax) {
        stepsPerSec = (double)axisConfig[AXIS_X].velocityMax;
    }
    return (uint32_t)(stepsPerSec + 0.5);
}

static double CurrentXInActiveUnits() {
    return AxisStepsToDisplay(AXIS_X, AxisCurrentSteps(AXIS_X) - wcsOriginSteps[AXIS_X]);
}

static double CurrentYInActiveUnits() {
    return AxisStepsToDisplay(AXIS_Y, AxisCurrentSteps(AXIS_Y) - wcsOriginSteps[AXIS_Y]);
}

static void ApplyFeedRate() {
    for (uint8_t axis = 0; axis < AXIS_COUNT; axis++) {
        MotorDriver *motor = MotorForPort(axisConfig[axis].port);
        if (!motor) {
            continue;
        }
        motor->VelMax(axisConfig[axis].velocityMax);
        motor->AccelMax(axisConfig[axis].accelMax);
        motor->EStopDecelMax(axisConfig[axis].decelMax);
    }
}

static void ReportPosition() {
    char msg[128];
    const char *units = (unitMode == UNITS_MM) ? "MM" : "IN";
    sprintf(msg, "POS X=%.3f Y=%.3f Z=%.3f A=%.3f Units=%s",
            CurrentXInActiveUnits(),
            CurrentYInActiveUnits(),
            AxisStepsToDisplay(AXIS_Z, AxisCurrentSteps(AXIS_Z)),
            AxisStepsToDisplay(AXIS_A, AxisCurrentSteps(AXIS_A)),
            units);
    SendLine(msg);
}

static void ReportStatus() {
    char msg[320];
    const char *mode = (coordinateMode == MODE_ABS) ? "ABS" : "REL";
    const char *units = (unitMode == UNITS_MM) ? "MM" : "IN";
    sprintf(msg, "STATUS Active=%d ActiveLine=%ld Queue=%u Mode=%s Units=%s Enabled=%d Feed=%.3f FeedSteps=%lu StepsPerMmX=%.6f Vel=%lu Accel=%lu Decel=%lu X=%.3f Y=%.3f Z=%.3f A=%.3f",
            motionRunning ? 1 : 0,
            (long)activeProgramLine,
            queueCount,
            mode, units, motorsEnabled ? 1 : 0,
            feedRate,
            (unsigned long)FeedRateToStepsPerSec(feedRate),
            axisConfig[AXIS_X].stepsPerUnit,
            (unsigned long)axisConfig[AXIS_X].velocityMax,
            (unsigned long)axisConfig[AXIS_X].accelMax,
            (unsigned long)axisConfig[AXIS_X].decelMax,
            CurrentXInActiveUnits(),
            CurrentYInActiveUnits(),
            AxisStepsToDisplay(AXIS_Z, AxisCurrentSteps(AXIS_Z) - wcsOriginSteps[AXIS_Z]),
            AxisStepsToDisplay(AXIS_A, AxisCurrentSteps(AXIS_A) - wcsOriginSteps[AXIS_A]));
    SendLine(msg);
}

static void FormatTelemetry(char *msg, size_t msgSize) {
    const char *mode = (coordinateMode == MODE_ABS) ? "ABS" : "REL";
    const char *units = (unitMode == UNITS_MM) ? "MM" : "IN";
    snprintf(msg, msgSize,
             "TEL Active=%d Hold=%d ActiveLine=%ld Queue=%u Mode=%s Units=%s Enabled=%d "
             "             Feed=%.3f X=%.3f Y=%.3f Z=%.3f A=%.3f",
             motionRunning ? 1 : 0,
             motionHold ? 1 : 0,
             (long)activeProgramLine,
             queueCount,
             mode,
             units,
             motorsEnabled ? 1 : 0,
             feedRate,
             CurrentXInActiveUnits(),
             CurrentYInActiveUnits(),
             AxisStepsToDisplay(AXIS_Z, AxisCurrentSteps(AXIS_Z) - wcsOriginSteps[AXIS_Z]),
             AxisStepsToDisplay(AXIS_A, AxisCurrentSteps(AXIS_A) - wcsOriginSteps[AXIS_A]));
}

static void PublishTelemetry(bool force = false) {
    if (!telemetryClientConnected || !telemetryClient.Connected()) {
        return;
    }
    const uint32_t now = Milliseconds();
    if (!force && (now - lastTelemetryTime) < TELEMETRY_INTERVAL_MS) {
        return;
    }
    lastTelemetryTime = now;

    char msg[240];
    FormatTelemetry(msg, sizeof(msg));
    SendTelemetryLine(msg);
}

static void SetPositionActiveUnits(double x, double y) {
    currentSteps[AXIS_X] = AxisValueToSteps(AXIS_X, x);
    currentSteps[AXIS_Y] = AxisValueToSteps(AXIS_Y, y);
    commandedSteps[AXIS_X] = currentSteps[AXIS_X];
    commandedSteps[AXIS_Y] = currentSteps[AXIS_Y];
    if (g_xyCoordinated && motorsInitialized) {
        motionController.SetPosition(currentSteps[AXIS_X], currentSteps[AXIS_Y]);
    }
    if (MotorDriver *motor = MotorForPort(axisConfig[AXIS_X].port)) {
        motor->PositionRefSet(currentSteps[AXIS_X]);
    }
    if (MotorDriver *motor = MotorForPort(axisConfig[AXIS_Y].port)) {
        motor->PositionRefSet(currentSteps[AXIS_Y]);
    }
}

static bool QueueMotionBlock(const int32_t target[AXIS_COUNT], uint8_t axisMask,
                             uint32_t nominalSpeed, int32_t programLine) {
    MotionBlock *block = QueueTailBlock();
    if (!block || axisMask == 0) {
        return false;
    }

    memset(block, 0, sizeof(MotionBlock));
    block->kind = MOTION_BLOCK_LINEAR;
    block->axisMask = axisMask;
    block->programLine = programLine;
    block->nominalSpeed = nominalSpeed;
    block->entrySpeed = 0;
    block->exitSpeed = 0;

    double lengthSqr = 0.0;
    for (uint8_t axis = 0; axis < AXIS_COUNT; axis++) {
        block->target[axis] = target[axis];
        block->delta[axis] = target[axis] - commandedSteps[axis];
        if (axisMask & (1u << axis)) {
            lengthSqr += (double)block->delta[axis] * (double)block->delta[axis];
        }
    }
    block->lengthSteps = sqrt(lengthSqr);
    if (block->lengthSteps <= 0.0) {
        return true;
    }

    for (uint8_t axis = 0; axis < AXIS_COUNT; axis++) {
        if (axisMask & (1u << axis)) {
            block->unitDir[axis] = (double)block->delta[axis] / block->lengthSteps;
            commandedSteps[axis] = target[axis];
        }
    }

    CommitQueueTail();
    return true;
}

static bool QueueArcXYBlock(const int32_t target[AXIS_COUNT], uint8_t axisMask,
                            int32_t centerX, int32_t centerY, int32_t radiusSteps,
                            double startAngle, double endAngle, bool clockwise,
                            uint32_t nominalSpeed, int32_t programLine) {
    MotionBlock *block = QueueTailBlock();
    if (!block || axisMask == 0 || radiusSteps <= 0) {
        return false;
    }

    memset(block, 0, sizeof(MotionBlock));
    block->kind = MOTION_BLOCK_ARC_XY;
    block->axisMask = axisMask;
    block->programLine = programLine;
    block->nominalSpeed = nominalSpeed;
    block->entrySpeed = 0;
    block->exitSpeed = 0;
    block->arcCenterX = centerX;
    block->arcCenterY = centerY;
    block->arcRadius = radiusSteps;
    block->arcStartAngle = startAngle;
    block->arcEndAngle = endAngle;
    block->arcClockwise = clockwise;

    double sn = startAngle;
    double en = endAngle;
    NormalizeAngle0TwoPi(&sn);
    NormalizeAngle0TwoPi(&en);
    block->lengthSteps = (double)radiusSteps * ArcSpanRadians(sn, en, clockwise);
    if (block->lengthSteps <= 0.0) {
        return false;
    }

    for (uint8_t axis = 0; axis < AXIS_COUNT; axis++) {
        block->target[axis] = target[axis];
        block->delta[axis] = target[axis] - commandedSteps[axis];
    }
    for (uint8_t axis = 0; axis < AXIS_COUNT; axis++) {
        if (axisMask & (1u << axis)) {
            commandedSteps[axis] = target[axis];
        }
    }

    CommitQueueTail();
    return true;
}

static MotionBlock BuildMergedHeadBlock() {
    return motionQueue[queueHead];
}


static bool MotionBlockHasXyMovement(const MotionBlock &b) {
    if (b.kind != MOTION_BLOCK_LINEAR || !b.valid) {
        return false;
    }
    if (!(b.axisMask & ((1u << AXIS_X) | (1u << AXIS_Y)))) {
        return false;
    }
    return b.delta[AXIS_X] != 0 || b.delta[AXIS_Y] != 0;
}

// Coordinated planner batch (QueueLinear/QueueArc) cannot carry Z/A on the same line.
static bool MotionBlockLinearIsXyPlannerSafe(const MotionBlock &b) {
    if (b.kind != MOTION_BLOCK_LINEAR || !b.valid) {
        return false;
    }
    if (axisConfig[AXIS_Z].enabled && (b.axisMask & (1u << AXIS_Z)) && b.delta[AXIS_Z] != 0) {
        return false;
    }
    if (axisConfig[AXIS_A].enabled && (b.axisMask & (1u << AXIS_A)) && b.delta[AXIS_A] != 0) {
        return false;
    }
    return true;
}

static bool MotionBlocksContiguousXY(const MotionBlock &prev, const MotionBlock &next) {
    const int32_t nsx = next.target[AXIS_X] - next.delta[AXIS_X];
    const int32_t nsy = next.target[AXIS_Y] - next.delta[AXIS_Y];
    if (labs((long)(prev.target[AXIS_X] - nsx)) > 3 || labs((long)(prev.target[AXIS_Y] - nsy)) > 3) {
        return false;
    }
    return true;
}

// Linear segments (including pure X or Y) + arcs in one deferred planner batch so line→arc
// junctions get GRBL-style corner speed instead of per-axis Move then a fresh coordinated arc.
static uint8_t CountConsecutiveXyPlannerChainAtHead() {
    if (queueCount == 0) {
        return 0;
    }
    MotionBlock &headb = motionQueue[queueHead];
    if (!headb.valid) {
        return 0;
    }
    if (headb.kind == MOTION_BLOCK_LINEAR) {
        if (!MotionBlockHasXyMovement(headb) || !MotionBlockLinearIsXyPlannerSafe(headb)) {
            return 0;
        }
    } else if (headb.kind != MOTION_BLOCK_ARC_XY) {
        return 0;
    }

    uint8_t n = 1;
    uint8_t idx = (queueHead + 1) % MOTION_QUEUE_SIZE;
    while (n < queueCount) {
        MotionBlock &prev = motionQueue[(queueHead + n - 1) % MOTION_QUEUE_SIZE];
        MotionBlock &cur = motionQueue[idx];
        if (!cur.valid) {
            break;
        }
        if (!MotionBlocksContiguousXY(prev, cur)) {
            break;
        }
        if (cur.kind == MOTION_BLOCK_LINEAR) {
            if (!MotionBlockHasXyMovement(cur) || !MotionBlockLinearIsXyPlannerSafe(cur)) {
                break;
            }
        } else if (cur.kind != MOTION_BLOCK_ARC_XY) {
            break;
        }
        n++;
        idx = (idx + 1) % MOTION_QUEUE_SIZE;
    }
    return n;
}

// True if this firmware arc is the same segment we just QueueArc'd. Match the *committed*
// stream fingerprint (first line of the circle), not this line's recomputed cx/cy/r — those can
// drift vs stored MotionBlock fields on anisotropic machines and Pop would miss → ~1.75 turns.
static bool MotionBlockMatchesStreamArcRelaxed(const MotionBlock &h, int32_t ex, int32_t ey,
                                               bool cw, uint32_t nominalSpeed) {
    if (!s_coordArcStreamOpen || h.kind != MOTION_BLOCK_ARC_XY || !h.valid) {
        return false;
    }
    if (h.arcClockwise != cw) {
        return false;
    }
    if (labs((long)(h.target[AXIS_X] - ex)) > 12 || labs((long)(h.target[AXIS_Y] - ey)) > 12) {
        return false;
    }
    const int32_t ctol = 16 + (int32_t)(s_coordArcR / 64);
    if (labs((long)(h.arcCenterX - s_coordArcCx)) > ctol ||
        labs((long)(h.arcCenterY - s_coordArcCy)) > ctol ||
        labs((long)(h.arcRadius - s_coordArcR)) > ctol) {
        return false;
    }
    const uint32_t nmin = h.nominalSpeed < nominalSpeed ? h.nominalSpeed : nominalSpeed;
    const uint32_t nmax = h.nominalSpeed < nominalSpeed ? nominalSpeed : h.nominalSpeed;
    if (nmax - nmin > 5) {
        return false;
    }
    return true;
}

// Remove a matching arc anywhere in the queue (duplicate is not always at head — e.g. linear
// lead-in block or ordering quirks). One match per streamed G2/G3 line.
static void PopOrEraseStreamDuplicateFirmwareArc(int32_t ex, int32_t ey, bool cw,
                                                 uint32_t nominalSpeed) {
    for (uint8_t i = 0; i < queueCount; i++) {
        const uint8_t phys = (queueHead + i) % MOTION_QUEUE_SIZE;
        if (MotionBlockMatchesStreamArcRelaxed(motionQueue[phys], ex, ey, cw, nominalSpeed)) {
            EraseQueueBlockAtOffset(i);
            return;
        }
    }
}

static void PopMergedBlocks(const MotionBlock &) {
    for (uint8_t i = 0; i < s_mergedBlocksToPop; i++) {
        PopQueueHead();
    }
    s_mergedBlocksToPop = 1;
}

static void RecalculateLookahead() {
    if (queueCount == 0) {
        return;
    }
    uint8_t idx = queueHead;
    for (uint8_t i = 0; i < queueCount; i++) {
        MotionBlock &cur = motionQueue[idx];
        cur.entrySpeed = cur.nominalSpeed;
        cur.exitSpeed = cur.nominalSpeed;
        if (i == 0 && !motionRunning) {
            cur.entrySpeed = 0;
        }
        if (i + 1 == queueCount) {
            cur.exitSpeed = 0;
        }
        idx = (idx + 1) % MOTION_QUEUE_SIZE;
    }
}

static bool StartMotionBlock(const MotionBlock &block) {
    bool anyMove = false;
    activeAxisMask = 0;
    s_blockUsedCoordinatedXy = false;
    s_mergedBlocksToPop = 1;
    if (g_xyCoordinated) {
        uint8_t xyChain = CountConsecutiveXyPlannerChainAtHead();
        // Never submit more segments than the planner queue can hold in one batch.
        // Overflow would cause QueueArc/QueueLinear to fail → StartMotionBlock returns false
        // → ClearMotionQueue() wipes everything. The cap is safe; the leftover segments stay
        // in the firmware queue and form the next batch (which will restart from 0 speed —
        // so the real fix is keeping ARC_QUEUE_SIZE ≥ MOTION_QUEUE_SIZE).
        if (xyChain > motionController.PlannerQueueCapacity()) {
            xyChain = motionController.PlannerQueueCapacity();
        }
        if (xyChain >= 1) {
            motionController.SetPosition(currentSteps[AXIS_X], currentSteps[AXIS_Y]);
            motionController.ArcAccelMax(axisConfig[AXIS_X].accelMax);
            motionController.SetDeferMotionQueueStart(xyChain > 1);
            uint8_t idx = queueHead;
            for (uint8_t k = 0; k < xyChain; k++) {
                MotionBlock &seg = motionQueue[idx];
                uint32_t pathSpeed = seg.nominalSpeed;
                if (pathSpeed < 1) {
                    pathSpeed = 1;
                }
                if (pathSpeed > axisConfig[AXIS_X].velocityMax) {
                    pathSpeed = axisConfig[AXIS_X].velocityMax;
                }
                motionController.ArcVelMax(pathSpeed);
                if (seg.kind == MOTION_BLOCK_LINEAR) {
                    if (!motionController.QueueLinear(seg.target[AXIS_X], seg.target[AXIS_Y])) {
                        motionController.SetDeferMotionQueueStart(false);
                        motionController.Stop();
                        return false;
                    }
                } else {
                    if (!motionController.QueueArc(seg.arcCenterX, seg.arcCenterY, seg.arcRadius,
                                                   seg.arcStartAngle, seg.arcEndAngle,
                                                   seg.arcClockwise)) {
                        motionController.SetDeferMotionQueueStart(false);
                        motionController.Stop();
                        return false;
                    }
                }
                idx = (idx + 1) % MOTION_QUEUE_SIZE;
            }
            motionController.SetDeferMotionQueueStart(false);
            if (!motionController.IsActive()) {
                if (!motionController.StartDeferredMotionIfPending()) {
                    motionController.Stop();
                    return false;
                }
            }
            s_mergedBlocksToPop = xyChain;
            s_blockUsedCoordinatedXy = true;
            const uint8_t lastIdx =
                (uint8_t)((queueHead + xyChain - 1) % MOTION_QUEUE_SIZE);
            activeTargetSteps[AXIS_X] = motionQueue[lastIdx].target[AXIS_X];
            activeTargetSteps[AXIS_Y] = motionQueue[lastIdx].target[AXIS_Y];
            // Save for streaming refill: must capture BEFORE PopMergedBlocks advances queueHead.
            s_streamLastEndX = motionQueue[lastIdx].target[AXIS_X];
            s_streamLastEndY = motionQueue[lastIdx].target[AXIS_Y];
            s_streamLastBlock = motionQueue[lastIdx];
            activeAxisMask |= (1u << AXIS_X) | (1u << AXIS_Y);
            anyMove = true;
        }
    }
    for (uint8_t axis = 0; axis < AXIS_COUNT; axis++) {
        if (!(block.axisMask & (1u << axis))) {
            continue;
        }
        if (s_blockUsedCoordinatedXy && (axis == AXIS_X || axis == AXIS_Y)) {
            continue;
        }
        MotorDriver *motor = MotorForPort(axisConfig[axis].port);
        if (!motor || !axisConfig[axis].enabled) {
            continue;
        }
        const int32_t delta = block.target[axis] - currentSteps[axis];
        if (delta == 0) {
            continue;
        }
        uint32_t axisSpeed = (uint32_t)fabs((double)block.nominalSpeed * block.unitDir[axis]);
        if (axisSpeed < 1) {
            axisSpeed = 1;
        }
        if (axisSpeed > axisConfig[axis].velocityMax) {
            axisSpeed = axisConfig[axis].velocityMax;
        }
        motor->VelMax(axisSpeed);
        motor->AccelMax(axisConfig[axis].accelMax);
        motor->EStopDecelMax(axisConfig[axis].decelMax);
        if (!motor->Move(delta)) {
            return false;
        }
        activeTargetSteps[axis] = block.target[axis];
        activeAxisMask |= (1u << axis);
        anyMove = true;
    }
    if (anyMove) {
        activeProgramLine = block.programLine;
        if (activeProgramLine > 0) {
            char msg[48];
            sprintf(msg, "ACTIVE Line=%ld", (long)activeProgramLine);
            SendLine(msg);
        }
        PublishTelemetry(true);
    }
    return anyMove;
}

static bool ActiveMotionComplete() {
    if (s_blockUsedCoordinatedXy) {
        if (MotorDriver *m = MotorForPort(axisConfig[AXIS_X].port)) {
            if (m->StatusReg().bit.AlertsPresent) {
                SendLine("error: motor alert on X (e.g. tracking), queue cleared");
                ClearMotionQueueEx(true);
                return true;
            }
        }
        if (MotorDriver *m = MotorForPort(axisConfig[AXIS_Y].port)) {
            if (m->StatusReg().bit.AlertsPresent) {
                SendLine("error: motor alert on Y (e.g. tracking), queue cleared");
                ClearMotionQueueEx(true);
                return true;
            }
        }
        if (motionController.IsActive() || motionController.MotionQueueCount() > 0) {
            return false;
        }
    }
    for (uint8_t axis = 0; axis < AXIS_COUNT; axis++) {
        if (s_blockUsedCoordinatedXy && (axis == AXIS_X || axis == AXIS_Y)) {
            continue;
        }
        MotorDriver *motor = MotorForPort(axisConfig[axis].port);
        if (motor && axisConfig[axis].enabled && motor->StatusReg().bit.AlertsPresent) {
            ClearMotionQueueEx(true);
            return true;
        }
        if (motor && axisConfig[axis].enabled && !motor->StepsComplete()) {
            return false;
        }
    }
    return true;
}

// Streaming refill: while the planner is executing the current batch, pop the next
// contiguous XY blocks from the FIRMWARE queue and feed them directly into the planner.
// Each block is popped immediately (so the firmware queue slot is freed) — the planner
// already has a copy.  s_streamLastBlock/EndX/Y track the last-submitted endpoint so
// contiguity can be checked even after the original batch blocks were popped.
static void TryExtendCoordinatedBatch() {
    if (!s_blockUsedCoordinatedXy) {
        return;
    }
    if (!motionController.IsActive() && motionController.MotionQueueCount() == 0) {
        return;  // Planner drained; UpdateMotionExecutor handles restart.
    }
    const uint8_t capacity = motionController.PlannerQueueCapacity();
    while (motionController.MotionQueueCount() + 1 < capacity) {
        if (queueCount == 0) {
            break;  // No more firmware blocks available yet.
        }
        const MotionBlock &next = motionQueue[queueHead];
        if (!next.valid) {
            break;
        }
        // Check that this block starts where the previous submission ended.
        const int32_t nsx = next.target[AXIS_X] - next.delta[AXIS_X];
        const int32_t nsy = next.target[AXIS_Y] - next.delta[AXIS_Y];
        if (labs((long)(s_streamLastEndX - nsx)) > 3 ||
            labs((long)(s_streamLastEndY - nsy)) > 3) {
            break;  // Not contiguous — different path segment (e.g. Z move).
        }
        if (next.kind == MOTION_BLOCK_LINEAR) {
            if (!MotionBlockHasXyMovement(next) || !MotionBlockLinearIsXyPlannerSafe(next)) {
                break;
            }
        } else if (next.kind != MOTION_BLOCK_ARC_XY) {
            break;
        }
        uint32_t pathSpeed = next.nominalSpeed;
        if (pathSpeed < 1) {
            pathSpeed = 1;
        }
        if (pathSpeed > axisConfig[AXIS_X].velocityMax) {
            pathSpeed = axisConfig[AXIS_X].velocityMax;
        }
        motionController.ArcVelMax(pathSpeed);
        bool ok;
        if (next.kind == MOTION_BLOCK_LINEAR) {
            ok = motionController.QueueLinear(next.target[AXIS_X], next.target[AXIS_Y]);
        } else {
            ok = motionController.QueueArc(next.arcCenterX, next.arcCenterY, next.arcRadius,
                                           next.arcStartAngle, next.arcEndAngle,
                                           next.arcClockwise);
        }
        if (!ok) {
            break;
        }
        // Update tracking, pop from firmware queue, update active target.
        s_streamLastEndX = next.target[AXIS_X];
        s_streamLastEndY = next.target[AXIS_Y];
        s_streamLastBlock = next;
        activeTargetSteps[AXIS_X] = next.target[AXIS_X];
        activeTargetSteps[AXIS_Y] = next.target[AXIS_Y];
        PopQueueHead();  // Firmware no longer needs this slot; planner owns the data.
    }
}

static void UpdateMotionExecutor() {
    if (motionRunning) {
        // While coordinated XY is executing, stream the next contiguous firmware blocks
        // into the planner before its last segment starts decelerating to zero.
        TryExtendCoordinatedBatch();

        if (!ActiveMotionComplete()) {
            return;
        }
        if (s_blockUsedCoordinatedXy) {
            currentSteps[AXIS_X] = motionController.CurrentX();
            currentSteps[AXIS_Y] = motionController.CurrentY();
        }
        for (uint8_t axis = 0; axis < AXIS_COUNT; axis++) {
            if (activeAxisMask & (1u << axis)) {
                if (s_blockUsedCoordinatedXy && (axis == AXIS_X || axis == AXIS_Y)) {
                    continue;
                }
                currentSteps[axis] = activeTargetSteps[axis];
            }
        }
        s_blockUsedCoordinatedXy = false;
        InvalidateCoordinatedArcStream();
        activeAxisMask = 0;
        activeProgramLine = 0;
        motionRunning = false;
    }

    if (MotionQueueEmpty()) {
        if (motionActivityPendingDone) {
            motionActivityPendingDone = false;
            SendLine("DONE");
        }
        return;
    }
    if (motionHold) {
        return;
    }
    if (queueCount < QUEUE_START_DEPTH &&
        (Milliseconds() - queueFirstBlockTime) < QUEUE_START_TIMEOUT_MS) {
        return;
    }

    RecalculateLookahead();
    MotionBlock block = BuildMergedHeadBlock();
    if (StartMotionBlock(block)) {
        PopMergedBlocks(block);
        motionRunning = true;
    } else {
        ClearMotionQueue();
    }
}

// R in user units (signed: negative = major arc). Returns false if geometry is invalid.
static bool ArcCenterFromSignedRadius(int32_t sx, int32_t sy, int32_t ex, int32_t ey,
                                      int32_t rSignedSteps, bool clockwise,
                                      int32_t *cx, int32_t *cy, int32_t *rNom) {
    const double dx = (double)(ex - sx);
    const double dy = (double)(ey - sy);
    const double chord = hypot(dx, dy);
    if (chord < 1.0) {
        return false;
    }
    const double absR = fabs((double)rSignedSteps);
    const double hSq = absR * absR - (chord * 0.5) * (chord * 0.5);
    if (hSq < 0.0) {
        return false;
    }
    const double h = sqrt(hSq);
    const double mx = -dy / chord;
    const double my = dx / chord;
    const double pmx = ((double)sx + (double)ex) * 0.5;
    const double pmy = ((double)sy + (double)ey) * 0.5;
    const double cxMinorCCW = pmx + mx * h;
    const double cyMinorCCW = pmy + my * h;
    const double cxMinorCW = pmx - mx * h;
    const double cyMinorCW = pmy - my * h;

    double tcx, tcy;
    if (rSignedSteps >= 0) {
        if (clockwise) {
            tcx = cxMinorCW;
            tcy = cyMinorCW;
        } else {
            tcx = cxMinorCCW;
            tcy = cyMinorCCW;
        }
    } else {
        if (clockwise) {
            tcx = cxMinorCCW;
            tcy = cyMinorCCW;
        } else {
            tcx = cxMinorCW;
            tcy = cyMinorCW;
        }
    }
    *cx = (int32_t)llround(tcx);
    *cy = (int32_t)llround(tcy);
    const double rr = hypot((double)sx - tcx, (double)sy - tcy);
    *rNom = (int32_t)llround(rr);
    if (*rNom <= 0) {
        return false;
    }
    return true;
}

static void HandleArcCommand(bool inlineRelative, int arcDir, const char *line, const char *lineUpper) {
    if (!motorsInitialized || !motorsEnabled) {
        SendLine("error: motors not enabled");
        return;
    }
    if (!g_xyCoordinated) {
        SendLine("error: G2/G3 need CONFIG SINGLE=0 (coordinated XY)");
        return;
    }
    if (!axisConfig[AXIS_X].enabled || !axisConfig[AXIS_Y].enabled) {
        SendLine("error: G2/G3 need X and Y enabled (CONFIG AX)");
        return;
    }

    const bool clockwise = (arcDir == 0);

    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
    double a = 0.0;
    double iu = 0.0;
    double ju = 0.0;
    double ru = 0.0;
    bool hasX = ParseValue(line, 'X', x);
    bool hasY = ParseValue(line, 'Y', y);
    bool hasZ = ParseValue(line, 'Z', z);
    bool hasA = ParseValue(line, 'A', a);
    bool hasI = ParseValue(line, 'I', iu);
    bool hasJ = ParseValue(line, 'J', ju);
    bool hasR = ParseValue(line, 'R', ru);

    if (hasX && !axisConfig[AXIS_X].enabled) {
        hasX = false;
    }
    if (hasY && !axisConfig[AXIS_Y].enabled) {
        hasY = false;
    }
    if (hasZ && !axisConfig[AXIS_Z].enabled) {
        hasZ = false;
    }
    if (hasA && !axisConfig[AXIS_A].enabled) {
        hasA = false;
    }

    double f = 0.0;
    if (ParseValue(line, 'F', f) && f > 0.0) {
        feedRate = f;
    }
    ApplyFeedRate();

    int32_t targets[AXIS_COUNT];
    uint8_t axisMask = 0;
    const bool incremental = inlineRelative || (coordinateMode == MODE_REL);

    if (incremental) {
        for (uint8_t axis = 0; axis < AXIS_COUNT; axis++) {
            targets[axis] = commandedSteps[axis];
        }
        if (hasX) {
            targets[AXIS_X] += UserDeltaToSteps(AXIS_X, x);
            axisMask |= (1u << AXIS_X);
        }
        if (hasY) {
            targets[AXIS_Y] += UserDeltaToSteps(AXIS_Y, y);
            axisMask |= (1u << AXIS_Y);
        }
        if (hasZ) {
            targets[AXIS_Z] += UserDeltaToSteps(AXIS_Z, z);
            axisMask |= (1u << AXIS_Z);
        }
        if (hasA) {
            targets[AXIS_A] += UserDeltaToSteps(AXIS_A, a);
            axisMask |= (1u << AXIS_A);
        }
    } else {
        double targetX = CurrentXInActiveUnits();
        double targetY = CurrentYInActiveUnits();
        double targetZ = AxisStepsToDisplay(AXIS_Z, currentSteps[AXIS_Z]);
        double targetA = AxisStepsToDisplay(AXIS_A, currentSteps[AXIS_A]);
        if (hasX) {
            targetX = x;
        }
        if (hasY) {
            targetY = y;
        }
        if (hasZ) {
            targetZ = z;
        }
        if (hasA) {
            targetA = a;
        }
        targets[AXIS_X] = AxisValueToSteps(AXIS_X, targetX);
        targets[AXIS_Y] = AxisValueToSteps(AXIS_Y, targetY);
        targets[AXIS_Z] = AxisValueToSteps(AXIS_Z, targetZ);
        targets[AXIS_A] = AxisValueToSteps(AXIS_A, targetA);
        if (!hasX) {
            targets[AXIS_X] = commandedSteps[AXIS_X];
        }
        if (!hasY) {
            targets[AXIS_Y] = commandedSteps[AXIS_Y];
        }
        if (!hasZ) {
            targets[AXIS_Z] = commandedSteps[AXIS_Z];
        }
        if (!hasA) {
            targets[AXIS_A] = commandedSteps[AXIS_A];
        }
        if (hasX) {
            axisMask |= (1u << AXIS_X);
        }
        if (hasY) {
            axisMask |= (1u << AXIS_Y);
        }
        if (hasZ) {
            axisMask |= (1u << AXIS_Z);
        }
        if (hasA) {
            axisMask |= (1u << AXIS_A);
        }
    }

    if ((hasZ && axisConfig[AXIS_Z].enabled &&
         targets[AXIS_Z] != commandedSteps[AXIS_Z]) ||
        (hasA && axisConfig[AXIS_A].enabled &&
         targets[AXIS_A] != commandedSteps[AXIS_A])) {
        SendLine("error: G2/G3 cannot move Z or A on the same line");
        return;
    }

    axisMask |= (1u << AXIS_X) | (1u << AXIS_Y);

    const int32_t sx = commandedSteps[AXIS_X];
    const int32_t sy = commandedSteps[AXIS_Y];
    const int32_t ex = targets[AXIS_X];
    const int32_t ey = targets[AXIS_Y];

    int32_t cx = 0;
    int32_t cy = 0;
    int32_t rSteps = 0;

    if (hasR) {
        const int32_t rSigned = UserDeltaToSteps(AXIS_X, ru);
        if (!ArcCenterFromSignedRadius(sx, sy, ex, ey, rSigned, clockwise, &cx, &cy, &rSteps)) {
            SendLine("error: arc R invalid (chord vs radius)");
            return;
        }
    } else {
        if (!hasI && !hasJ) {
            SendLine("error: G2/G3 needs I J or R");
            return;
        }
        const int32_t iSteps = UserDeltaToSteps(AXIS_X, iu);
        const int32_t jSteps = UserDeltaToSteps(AXIS_Y, ju);
        cx = sx + iSteps;
        cy = sy + jSteps;
        rSteps = (int32_t)llround(hypot((double)(sx - cx), (double)(sy - cy)));
        if (rSteps <= 0) {
            SendLine("error: arc I J yield zero radius");
            return;
        }
    }

    {
        const int32_t rEnd =
            (int32_t)llround(hypot((double)(ex - cx), (double)(ey - cy)));
        if ((int32_t)labs((long)(rEnd - rSteps)) > 2) {
            SendLine("error: arc end point not on circle (check X Y I J or R)");
            return;
        }
    }

    const double sa = atan2((double)(sy - cy), (double)(sx - cx));
    const double ea = atan2((double)(ey - cy), (double)(ex - cx));

    double lineId = 0.0;
    const int32_t programLine = ParseNamedValue(lineUpper, "LN", lineId)
        ? (int32_t)(lineId + 0.5)
        : 0;

    const uint32_t nominalSpeed = FeedRateToStepsPerSec(feedRate);
    uint32_t pathSpeed = nominalSpeed;
    if (pathSpeed < 1) {
        pathSpeed = 1;
    }
    if (pathSpeed > axisConfig[AXIS_X].velocityMax) {
        pathSpeed = axisConfig[AXIS_X].velocityMax;
    }

    // Treat as "in motion" for arc streaming if the controller is busy OR our executor is
    // still running a coordinated XY block (covers gaps where IsActive is briefly false).
    const bool coordBusyForArcStream =
        g_xyCoordinated &&
        (motionController.IsActive() || motionController.MotionQueueCount() > 0 ||
         (motionRunning && s_blockUsedCoordinatedXy));

    // Planner streaming only for continuations of the *same* circle (s_coordArcStreamOpen).
    // G0/G1 call InvalidateCoordinatedArcStream(), but motion may still be finishing the last
    // arc — coordBusy stays true while IsActive/MotionQueueCount/motionRunning. Without this
    // split, the next G3 (new corner, new I J) wrongly gets "busy" and the program stalls
    // (e.g. rounded-rectangle after a vertical G1).
    if (coordBusyForArcStream) {
        if (CoordinatedArcMatchesStream(cx, cy, rSteps, clockwise, nominalSpeed)) {
            motionController.ArcVelMax(pathSpeed);
            motionController.ArcAccelMax(axisConfig[AXIS_X].accelMax);
            if (!motionController.QueueArc(cx, cy, rSteps, sa, ea, clockwise)) {
                InvalidateCoordinatedArcStream();
                SendLine("error: arc planner queue full or invalid");
                return;
            }
            commandedSteps[AXIS_X] = ex;
            commandedSteps[AXIS_Y] = ey;
            // Keep streaming-refill tracker in sync so TryExtendCoordinatedBatch can check
            // contiguity correctly for the arc that follows this streamed one.
            s_streamLastEndX = ex;
            s_streamLastEndY = ey;
            PopOrEraseStreamDuplicateFirmwareArc(ex, ey, clockwise, nominalSpeed);
            SendLine("ok");
            return;
        }
        // Arc doesn't match the current stream (different circle / different path).
        // Fall through to the firmware queue — TryExtendCoordinatedBatch will pick it up
        // from there and add it to the planner at the right moment.  No "busy" here;
        // the queue-full guard below handles back-pressure.
    }

    if (MotionQueueFull()) {
        SendLine("busy: queue full");
        return;
    }

    if (!QueueArcXYBlock(targets, axisMask, cx, cy, rSteps, sa, ea, clockwise, nominalSpeed,
                         programLine)) {
        SendLine("error: arc queue rejected (degenerate arc?)");
        return;
    }
    CommitCoordinatedArcStream(cx, cy, rSteps, clockwise, nominalSpeed);
    SendLine("ok");
}

static void HandleMotionCommand(bool jog, bool rapid, const char *line, const char *lineUpper) {
    if (!motorsInitialized || !motorsEnabled) {
        SendLine("error: motors not enabled");
        return;
    }
    InvalidateCoordinatedArcStream();

    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
    double a = 0.0;
    bool hasX = ParseValue(line, 'X', x);
    bool hasY = ParseValue(line, 'Y', y);
    bool hasZ = ParseValue(line, 'Z', z);
    bool hasA = ParseValue(line, 'A', a);
    // Ignore moves on axes not present in CONFIG (AX bitmask / per-axis enabled).
    if (hasX && !axisConfig[AXIS_X].enabled) {
        hasX = false;
    }
    if (hasY && !axisConfig[AXIS_Y].enabled) {
        hasY = false;
    }
    if (hasZ && !axisConfig[AXIS_Z].enabled) {
        hasZ = false;
    }
    if (hasA && !axisConfig[AXIS_A].enabled) {
        hasA = false;
    }
    if (!hasX && !hasY && !hasZ && !hasA) {
        SendLine("error: no move on an enabled axis (check CONFIG AX=)");
        return;
    }

    double f = 0.0;
    if (!rapid && ParseValue(line, 'F', f) && f > 0.0) {
        feedRate = f;
    }
    ApplyFeedRate();

    int32_t targets[AXIS_COUNT];
    uint8_t axisMask = 0;
    const bool incremental = jog || (coordinateMode == MODE_REL);

    if (incremental) {
        // Jog, inline G91, or G91 modal (REL): add deltas in step space from commandedSteps.
        // "Display position + delta" + AxisValueToSteps() desyncs from the pulse reference
        // and can produce huge XY moves (Y jog "never stops") after a fault or coordinated
        // step drift vs inch round-trip.
        for (uint8_t axis = 0; axis < AXIS_COUNT; axis++) {
            targets[axis] = commandedSteps[axis];
        }
        if (hasX) {
            targets[AXIS_X] += UserDeltaToSteps(AXIS_X, x);
            axisMask |= (1u << AXIS_X);
        }
        if (hasY) {
            targets[AXIS_Y] += UserDeltaToSteps(AXIS_Y, y);
            axisMask |= (1u << AXIS_Y);
        }
        if (hasZ) {
            targets[AXIS_Z] += UserDeltaToSteps(AXIS_Z, z);
            axisMask |= (1u << AXIS_Z);
        }
        if (hasA) {
            targets[AXIS_A] += UserDeltaToSteps(AXIS_A, a);
            axisMask |= (1u << AXIS_A);
        }
    } else {
        double targetX = CurrentXInActiveUnits();
        double targetY = CurrentYInActiveUnits();
        double targetZ = AxisStepsToDisplay(AXIS_Z, currentSteps[AXIS_Z]);
        double targetA = AxisStepsToDisplay(AXIS_A, currentSteps[AXIS_A]);
        if (hasX) {
            targetX = x;
        }
        if (hasY) {
            targetY = y;
        }
        if (hasZ) {
            targetZ = z;
        }
        if (hasA) {
            targetA = a;
        }
        // Omitted axis words: keep planner step position exactly (no inch round-trip).
        targets[AXIS_X] = AxisValueToSteps(AXIS_X, targetX);
        targets[AXIS_Y] = AxisValueToSteps(AXIS_Y, targetY);
        targets[AXIS_Z] = AxisValueToSteps(AXIS_Z, targetZ);
        targets[AXIS_A] = AxisValueToSteps(AXIS_A, targetA);
        if (!hasX) {
            targets[AXIS_X] = commandedSteps[AXIS_X];
        }
        if (!hasY) {
            targets[AXIS_Y] = commandedSteps[AXIS_Y];
        }
        if (!hasZ) {
            targets[AXIS_Z] = commandedSteps[AXIS_Z];
        }
        if (!hasA) {
            targets[AXIS_A] = commandedSteps[AXIS_A];
        }
        if (hasX) {
            axisMask |= (1u << AXIS_X);
        }
        if (hasY) {
            axisMask |= (1u << AXIS_Y);
        }
        if (hasZ) {
            axisMask |= (1u << AXIS_Z);
        }
        if (hasA) {
            axisMask |= (1u << AXIS_A);
        }
    }

    double lineId = 0.0;
    const int32_t programLine = ParseNamedValue(lineUpper, "LN", lineId)
        ? (int32_t)(lineId + 0.5)
        : 0;

    if (MotionQueueFull()) {
        // Backpressure, not a fault: host should retry the same line after a short delay.
        SendLine("busy: queue full");
        return;
    }

    const uint32_t nominalSpeed =
        rapid ? velocityMaxStepsPerSec : FeedRateToStepsPerSec(feedRate);
    if (!QueueMotionBlock(targets, axisMask, nominalSpeed, programLine)) {
        SendLine("busy: queue full");
        return;
    }
    SendLine("ok");
}

static void ReportConfig() {
    char msg[220];
    // SINGLE=1: one-motor / independent M0+M1. SINGLE=0: coordinated G01 in XY.
    const int singleReport = g_xyCoordinated ? 0 : 1;
    const int axMask = (axisConfig[AXIS_X].enabled ? 1 : 0) | (axisConfig[AXIS_Y].enabled ? 2 : 0) |
                       (axisConfig[AXIS_Z].enabled ? 4 : 0) | (axisConfig[AXIS_A].enabled ? 8 : 0);
    sprintf(msg,
            "CONFIG StepsPerMmX=%.6f StepsPerMmY=%.6f Vel=%lu Accel=%lu Decel=%lu"
            " DVmax=%lu SINGLE=%d AX=%d",
            stepsPerMmX,
            stepsPerMmY,
            (unsigned long)velocityMaxStepsPerSec,
            (unsigned long)accelMaxStepsPerSec2,
            (unsigned long)stopDecelStepsPerSec2,
            (unsigned long)junctionDVmaxStepsPerSec,
            singleReport,
            axMask);
    SendLine(msg);
}

static void HandleConfigCommand(const char *lineUpper) {
    double value = 0.0;
    if (ParseNamedValue(lineUpper, "SPMMX", value) ||
        ParseNamedValue(lineUpper, "STEPSPERMMX", value)) {
        if (value > 0.0) {
            stepsPerMmX = value;
        }
    }
    if (ParseNamedValue(lineUpper, "SPMMY", value) ||
        ParseNamedValue(lineUpper, "STEPSPERMMY", value)) {
        if (value > 0.0) {
            stepsPerMmY = value;
        }
    }
    if (ParseNamedValue(lineUpper, "VEL", value) ||
        ParseNamedValue(lineUpper, "VELOCITY", value)) {
        if (value > 0.0) {
            velocityMaxStepsPerSec = (uint32_t)(value + 0.5);
        }
    }
    if (ParseNamedValue(lineUpper, "ACCEL", value)) {
        if (value > 0.0) {
            accelMaxStepsPerSec2 = (uint32_t)(value + 0.5);
        }
    }
    if (ParseNamedValue(lineUpper, "DECEL", value)) {
        if (value > 0.0) {
            stopDecelStepsPerSec2 = (uint32_t)(value + 0.5);
        }
    }
    if (ParseNamedValue(lineUpper, "DVMAX", value)) {
        // Per-axis max velocity delta at junctions (steps/sec).
        // 0 disables the cap and falls back to the GRBL angle-based formula.
        junctionDVmaxStepsPerSec = (uint32_t)(value + 0.5);
        if (g_xyCoordinated && motorsInitialized) {
            motionController.JunctionDVmax(junctionDVmaxStepsPerSec);
        }
    }
    axisConfig[AXIS_X].stepsPerUnit = stepsPerMmX;
    axisConfig[AXIS_Y].stepsPerUnit = stepsPerMmY;
    axisConfig[AXIS_X].velocityMax = velocityMaxStepsPerSec;
    axisConfig[AXIS_X].accelMax = accelMaxStepsPerSec2;
    axisConfig[AXIS_X].decelMax = stopDecelStepsPerSec2;
    axisConfig[AXIS_Y].velocityMax = velocityMaxStepsPerSec;
    axisConfig[AXIS_Y].accelMax = accelMaxStepsPerSec2;
    axisConfig[AXIS_Y].decelMax = stopDecelStepsPerSec2;
    {
        double singleIn = -1.0;
        if (ParseNamedValue(lineUpper, "SINGLE", singleIn)) {
            if (motorsEnabled) {
                SendLine("error: DISABLE (M203) before changing CONFIG SINGLE");
                return;
            }
            const bool wantCoordinatedXy = (singleIn < 0.5);
            if (wantCoordinatedXy != g_xyCoordinated) {
                g_xyCoordinated = wantCoordinatedXy;
                if (!ReapplyKinematicsMode()) {
                    SendLine("error: failed to init XY coordinated mode; keeping SINGLE=1 (independent)");
                }
            }
        }
    }
    // Bitmask: bit0=X, bit1=Y, bit2=Z, bit3=A. G01/jog words for disabled axes are ignored.
    if (ParseNamedValue(lineUpper, "AX", value)) {
        int m = (int)(value + 0.5);
        if (m < 0) {
            m = 0;
        }
        if (m > 15) {
            m = 15;
        }
        axisConfig[AXIS_X].enabled = (m & 1) != 0;
        axisConfig[AXIS_Y].enabled = (m & 2) != 0;
        axisConfig[AXIS_Z].enabled = (m & 4) != 0;
        axisConfig[AXIS_A].enabled = (m & 8) != 0;
    }
    ApplyFeedRate();
    ReportConfig();
}

static void ProcessCommand(char *lineRaw) {
    // Preserve original numeric formatting while matching keywords in uppercase.
    char lineUpper[MAX_LINE_LENGTH];
    strncpy(lineUpper, lineRaw, sizeof(lineUpper) - 1);
    lineUpper[sizeof(lineUpper) - 1] = '\0';
    ToUpperInPlace(lineUpper);

    if (strcmp(lineUpper, "HELP") == 0) {
        SendLine("ok: CONFIG (SPMMX,SPMMY,VEL,ACCEL,DECEL,DVMAX,SINGLE,AX) GETCFG M202 M203 M201 M200 G90 G91 G92 M114 M115 G17 G0 G01 G2 G3 JOG");
    } else if (strcmp(lineUpper, "ENABLE") == 0 || strcmp(lineUpper, "M202") == 0) {
        for (uint8_t axis = 0; axis < AXIS_COUNT; axis++) {
            if (axisConfig[axis].enabled) {
                if (MotorDriver *motor = MotorForPort(axisConfig[axis].port)) {
                    motor->EnableRequest(true);
                }
            }
        }
        motorsEnabled = true;
        SendLine("ok");
    } else if (strcmp(lineUpper, "DISABLE") == 0 || strcmp(lineUpper, "M203") == 0) {
        DisableMotorsAbrupt();
        SendLine("ok");
    } else if (strcmp(lineUpper, "STOP") == 0 || strcmp(lineUpper, "M201") == 0) {
        motionHold = false;
        ClearMotionQueue();
        for (uint8_t axis = 0; axis < AXIS_COUNT; axis++) {
            if (MotorDriver *motor = MotorForPort(axisConfig[axis].port)) {
                motor->MoveStopDecel(axisConfig[axis].decelMax);
            }
        }
        PublishTelemetry(true);
        SendLine("ok");
    } else if (strcmp(lineUpper, "HOLD") == 0 || strcmp(lineUpper, "PAUSE") == 0) {
        motionHold = true;
        PublishTelemetry(true);
        SendLine("ok");
    } else if (strcmp(lineUpper, "RESUME") == 0 || strcmp(lineUpper, "CYCLE START") == 0) {
        motionHold = false;
        if (!MotionQueueEmpty() && !motionRunning) {
            queueStartPending = true;
            queueFirstBlockTime = Milliseconds() - QUEUE_START_TIMEOUT_MS;
        }
        PublishTelemetry(true);
        SendLine("ok");
    } else if (strcmp(lineUpper, "ESTOP") == 0 || strcmp(lineUpper, "M200") == 0) {
        DisableMotorsAbrupt();
        SendLine("ok");
    } else if (strcmp(lineUpper, "ABS") == 0 || strcmp(lineUpper, "G90") == 0) {
        coordinateMode = MODE_ABS;
        SendLine("ok");
    } else if (strcmp(lineUpper, "REL") == 0 || strcmp(lineUpper, "G91") == 0) {
        coordinateMode = MODE_REL;
        SendLine("ok");
    } else if (strcmp(lineUpper, "UNITS MM") == 0 || strcmp(lineUpper, "G21") == 0 ||
               strstr(lineUpper, "G21 ") == lineUpper) {
        unitMode = UNITS_MM;
        if (feedRate <= 0.0) {
            feedRate = DEFAULT_FEED_MM_PER_MIN;
        }
        ApplyFeedRate();
        SendLine("ok");
    } else if (strcmp(lineUpper, "UNITS IN") == 0 || strcmp(lineUpper, "G20") == 0 ||
               strstr(lineUpper, "G20 ") == lineUpper) {
        unitMode = UNITS_IN;
        if (feedRate <= 0.0) {
            feedRate = DEFAULT_FEED_IN_PER_MIN;
        }
        ApplyFeedRate();
        SendLine("ok");
    } else if (strcmp(lineUpper, "ZERO") == 0 || strncmp(lineUpper, "G92", 3) == 0) {
        SetPositionActiveUnits(0.0, 0.0);
        SendLine("ok");
    } else if (strcmp(lineUpper, "POS") == 0 || strcmp(lineUpper, "M114") == 0) {
        ReportPosition();
    } else if (strcmp(lineUpper, "STATUS") == 0 || strcmp(lineUpper, "M115") == 0) {
        ReportStatus();
    } else if (strcmp(lineUpper, "GETCFG") == 0) {
        ReportConfig();
    } else if (strncmp(lineUpper, "CONFIG", 6) == 0) {
        HandleConfigCommand(lineUpper);
    } else if (LeadingPlaneSelectFromLine(lineUpper) != 0) {
        // G17/G18/G19: no-op (motion is XY only); must precede loose G1 match.
        SendLine("ok");
    } else if (MotionLineArcDirection(lineUpper) >= 0) {
        const int arcDir = MotionLineArcDirection(lineUpper);
        const bool inlineRelative = (strstr(lineUpper, "G91") != nullptr);
        HandleArcCommand(inlineRelative, arcDir, lineRaw, lineUpper);
    } else if (strncmp(lineUpper, "MOVE", 4) == 0 || strncmp(lineUpper, "G00", 3) == 0 ||
               strncmp(lineUpper, "G0", 2) == 0 || strncmp(lineUpper, "G01", 3) == 0 ||
               strncmp(lineUpper, "G1", 2) == 0 || strstr(lineUpper, " G00 ") != nullptr ||
               strstr(lineUpper, " G0 ") != nullptr || strstr(lineUpper, " G01 ") != nullptr ||
               strstr(lineUpper, " G1 ") != nullptr) {
        const bool inlineRelative = (strstr(lineUpper, "G91") != nullptr);
        const bool rapid = MotionLineIsRapidG0(lineUpper);
        HandleMotionCommand(inlineRelative, rapid, lineRaw, lineUpper);
    } else if (strncmp(lineUpper, "JOG", 3) == 0) {
        HandleMotionCommand(true, false, lineRaw, lineUpper);
    } else if (strcmp(lineUpper, "M30") == 0 || strcmp(lineUpper, "M02") == 0) {
        // Program end: acknowledge so the host G-code runner can finish (no special motion here).
        SendLine("ok");
    } else {
        SendLine("error: unknown command");
    }
}

static bool ReadLine(char *outLine, uint16_t maxLen) {
    const int16_t ch = SerialPort.CharGet();
    if (ch < 0) {
        return false;
    }

    if (ch == '\n' || ch == '\r') {
        if (lineIndex == 0) {
            return false;
        }
        outLine[lineIndex] = '\0';
        lineIndex = 0;
        return true;
    }

    if (lineIndex < maxLen - 1) {
        outLine[lineIndex++] = (char)ch;
    } else {
        lineIndex = 0;
        SendLine("error: line too long");
    }
    return false;
}

static bool ReadTcpLine(char *outLine, uint16_t maxLen) {
    if (!ethernetReady) {
        return false;
    }

    if (tcpClientConnected && !controlClient.Connected()) {
        DisableMotorsAbrupt();
        tcpClientConnected = false;
        tcpLineIndex = 0;
        ConnectorUsb.SendLine("Control TCP disconnected; motors disabled");
    }

    if (!tcpClientConnected || !controlClient.Connected()) {
        EthernetTcpClient newClient = controlServer.Accept();
        if (newClient.Connected()) {
            controlClient = newClient;
            tcpClientConnected = true;
            tcpLineIndex = 0;
            SendLine("ClearCNC TCP connected");
        } else {
            tcpClientConnected = false;
            return false;
        }
    }

    while (controlClient.BytesAvailable() > 0) {
        const int16_t ch = controlClient.Read();
        if (ch < 0) {
            return false;
        }
        if (ch == '\n' || ch == '\r') {
            if (tcpLineIndex == 0) {
                continue;
            }
            outLine[tcpLineIndex] = '\0';
            tcpLineIndex = 0;
            return true;
        }
        if (tcpLineIndex < maxLen - 1) {
            outLine[tcpLineIndex++] = (char)ch;
        } else {
            tcpLineIndex = 0;
            SendLine("error: line too long");
        }
    }
    return false;
}

static void PollTelemetryClient() {
    if (!ethernetReady) {
        return;
    }

    if (!telemetryClientConnected || !telemetryClient.Connected()) {
        EthernetTcpClient newClient = telemetryServer.Accept();
        if (newClient.Connected()) {
            telemetryClient = newClient;
            telemetryClientConnected = true;
            lastTelemetryTime = 0;
            PublishTelemetry(true);
        } else {
            telemetryClientConnected = false;
        }
    }
}

static bool InitializeEthernet() {
    if (ethernetReady || !EthernetMgr.PhyLinkActive()) {
        return ethernetReady;
    }

    EthernetMgr.Setup();
    if (!EthernetMgr.DhcpBegin()) {
        EthernetMgr.LocalIp(IpAddress(192, 168, 0, 109));
    }

    controlServer.Begin();
    telemetryServer.Begin();
    discoveryUdp.Begin(DISCOVERY_PORT);
    ethernetReady = true;

    ConnectorUsb.Send("Ethernet ready IP=");
    ConnectorUsb.SendLine(EthernetMgr.LocalIp().StringValue());
    return true;
}

static void PollDiscovery() {
    if (!ethernetReady) {
        return;
    }

    uint16_t packetSize = discoveryUdp.PacketParse();
    if (packetSize == 0) {
        return;
    }

    unsigned char packet[96];
    int32_t bytesRead = discoveryUdp.PacketRead(packet, sizeof(packet) - 1);
    if (bytesRead <= 0) {
        return;
    }
    packet[bytesRead] = '\0';
    if (strncmp((const char *)packet, DISCOVERY_REQUEST, strlen(DISCOVERY_REQUEST)) != 0) {
        return;
    }

    char response[128];
    sprintf(response, "CLEARCNC_DISCOVER ClearCNC_Firmware IP=%s TCP=%u TEL=%u FW=1",
            EthernetMgr.LocalIp().StringValue(),
            ETHERNET_CONTROL_PORT,
            ETHERNET_TELEMETRY_PORT);
    discoveryUdp.Connect(discoveryUdp.RemoteIp(), discoveryUdp.RemotePort());
    discoveryUdp.PacketWrite(response);
    discoveryUdp.PacketSend();
}

static void HlfbAndLimitsForMotor(MotorDriver *motor, const AxisConfig &cfg) {
    if (!motor) {
        return;
    }
    motor->HlfbMode(MotorDriver::HLFB_MODE_HAS_BIPOLAR_PWM);
    motor->HlfbCarrier(MotorDriver::HLFB_CARRIER_482_HZ);
    motor->VelMax(cfg.velocityMax);
    motor->AccelMax(cfg.accelMax);
    motor->EStopDecelMax(cfg.decelMax);
    motor->EnableRequest(false);
}

// Independent step/dir for all four motor connectors (G01 on X/Y = separate Move on each).
static void SetupMotorsIndependentAllAxes() {
    for (uint8_t axis = 0; axis < AXIS_COUNT; axis++) {
        if (MotorDriver *m = MotorForPort(axisConfig[axis].port)) {
            HlfbAndLimitsForMotor(m, axisConfig[axis]);
        }
    }
}

// M0 + M1 on CoordinatedMotionController; M2, M3 remain independent.
static bool SetupMotorsCoordinatedXy() {
    motorX.EnableRequest(false);
    motorY.EnableRequest(false);
    if (!motionController.Initialize(&motorX, &motorY)) {
        return false;
    }
    for (uint8_t axis = 2; axis < AXIS_COUNT; axis++) {
        if (MotorDriver *m = MotorForPort(axisConfig[axis].port)) {
            HlfbAndLimitsForMotor(m, axisConfig[axis]);
        }
    }
    motionController.SetMechanicalParamsX(MOTOR_X_STEPS_PER_REV, MOTOR_X_PITCH_MM, UNIT_MM);
    motionController.SetMechanicalParamsY(MOTOR_Y_STEPS_PER_REV, MOTOR_Y_PITCH_MM, UNIT_MM);
    motionController.ArcVelMax(velocityMaxStepsPerSec);
    motionController.ArcAccelMax(accelMaxStepsPerSec2);
    motionController.JunctionDVmax(junctionDVmaxStepsPerSec);
    motionController.StopAtQueueEnd(false);
    motionController.SetPosition(currentSteps[AXIS_X], currentSteps[AXIS_Y]);
    return true;
}

static void ReleaseCoordinatedModeOnM0M1() {
    motionController.Stop();
    motorX.CoordinatedMotionMode(false, nullptr);
    motorY.CoordinatedMotionMode(false, nullptr);
}

// Rebuild M0–M3 step/dir from axisConfig; call when motors are disabled and SINGLE changes.
static bool ReapplyKinematicsMode() {
    motorX.EnableRequest(false);
    motorY.EnableRequest(false);
    ReleaseCoordinatedModeOnM0M1();
    s_blockUsedCoordinatedXy = false;
    if (g_xyCoordinated) {
        if (!SetupMotorsCoordinatedXy()) {
            g_xyCoordinated = false;
            SetupMotorsIndependentAllAxes();
            return false;
        }
    } else {
        SetupMotorsIndependentAllAxes();
    }
    return true;
}

static void InitializeController() {
    MotorMgr.MotorModeSet(MotorManager::MOTOR_ALL, Connector::CPM_MODE_STEP_AND_DIR);
    Delay_ms(50);

    for (uint8_t axis = 0; axis < AXIS_COUNT; axis++) {
        axisConfig[axis].enabled = false;
        axisConfig[axis].port = axis;
        axisConfig[axis].axis = axis;
        axisConfig[axis].rotary = axis == AXIS_A;
        axisConfig[axis].stepsPerUnit = (axis == AXIS_Y) ? stepsPerMmY : stepsPerMmX;
        axisConfig[axis].velocityMax = velocityMaxStepsPerSec;
        axisConfig[axis].accelMax = accelMaxStepsPerSec2;
        axisConfig[axis].decelMax = stopDecelStepsPerSec2;
    }
    axisConfig[AXIS_X].enabled = true;

    motorX.EnableRequest(false);
    motorY.EnableRequest(false);
    g_xyCoordinated = false;
    SetupMotorsIndependentAllAxes();

    mechanicalConfigX.stepsPerRevolution = MOTOR_X_STEPS_PER_REV;
    mechanicalConfigX.pitch = MOTOR_X_PITCH_MM;
    mechanicalConfigX.pitchUnit = UNIT_MM;
    mechanicalConfigX.gearRatio = 1.0;
    UnitConverter::CalculateConversionFactors(mechanicalConfigX);

    mechanicalConfigY.stepsPerRevolution = MOTOR_Y_STEPS_PER_REV;
    mechanicalConfigY.pitch = MOTOR_Y_PITCH_MM;
    mechanicalConfigY.pitchUnit = UNIT_MM;
    mechanicalConfigY.gearRatio = 1.0;
    UnitConverter::CalculateConversionFactors(mechanicalConfigY);

    feedRate = DEFAULT_FEED_MM_PER_MIN;
    stepsPerMmX = (double)MOTOR_X_STEPS_PER_REV / MOTOR_X_PITCH_MM;
    stepsPerMmY = (double)MOTOR_Y_STEPS_PER_REV / MOTOR_Y_PITCH_MM;
    velocityMaxStepsPerSec = DEFAULT_VELOCITY_STEPS_PER_SEC;
    accelMaxStepsPerSec2 = DEFAULT_ACCEL_STEPS_PER_SEC2;
    stopDecelStepsPerSec2 = DEFAULT_STOP_DECEL_STEPS_PER_SEC2;
    unitMode = UNITS_MM;
    coordinateMode = MODE_ABS;
    for (uint8_t axis = 0; axis < AXIS_COUNT; axis++) {
        currentSteps[axis] = 0;
        commandedSteps[axis] = 0;
        activeTargetSteps[axis] = 0;
    }
    activeAxisMask = 0;
    ClearMotionQueue();
    ApplyFeedRate();
    motorsInitialized = true;
}

int main() {
    Delay_ms(300);

    SerialPort.Mode(Connector::USB_CDC);
    SerialPort.Speed(SERIAL_BAUD_RATE);
    SerialPort.PortOpen();
    uint32_t startTime = Milliseconds();
    while (WAIT_FOR_USB_CONNECTION && !SerialPort && Milliseconds() - startTime < 5000) {
        continue;
    }
    Delay_ms(100);

    InitializeController();
    if (!motorsInitialized) {
        SendLine("error: motion controller init failed");
    } else {
        SendLine("ClearCNC firmware ready");
        SendLine("Use ENABLE before motion commands");
    }
    InitializeEthernet();

    while (true) {
        if (!ethernetReady) {
            InitializeEthernet();
        }
        PollDiscovery();
        PollTelemetryClient();
        if (ReadLine(lineBuffer, MAX_LINE_LENGTH)) {
            ProcessCommand(lineBuffer);
        }
        if (ReadTcpLine(tcpLineBuffer, MAX_LINE_LENGTH)) {
            ProcessCommand(tcpLineBuffer);
        }
        UpdateMotionExecutor();
        PublishTelemetry();
        Delay_ms(1);
    }
}

