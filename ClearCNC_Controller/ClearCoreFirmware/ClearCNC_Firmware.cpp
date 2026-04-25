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
 *   ZERO | G92 X0 Y0
 *   POS | M114
 *   STATUS | M115
 *   MOVE X<num> Y<num> [F<num>] | G01 X<num> Y<num> [F<num>]
 *   JOG X<num> Y<num> [F<num>] | G91 G01 X<num> Y<num> [F<num>]
 *
 * G01 in the X/Y plane: if CONFIG SINGLE=0 (default from host) uses CoordinatedMotionController
 * (vector feed). CONFIG SINGLE=1 = single-motor / independent M0 and M1 (no Y coordination).
 * Z/A always use per-stepper Move() on the remaining connectors.
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
#define QUEUE_START_DEPTH 4
#define QUEUE_START_TIMEOUT_MS 75
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

struct MotionBlock {
    bool valid;
    uint8_t axisMask;
    int32_t programLine;
    int32_t target[AXIS_COUNT];
    int32_t delta[AXIS_COUNT];
    double unitDir[AXIS_COUNT];
    double lengthSteps;
    uint32_t nominalSpeed;
    uint32_t entrySpeed;
    uint32_t exitSpeed;
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
static bool motorsInitialized = false;
static bool motorsEnabled = false;
static bool ethernetReady = false;
static bool tcpClientConnected = false;
static bool telemetryClientConnected = false;
static AxisConfig axisConfig[AXIS_COUNT];
static int32_t currentSteps[AXIS_COUNT] = {0, 0, 0, 0};
static int32_t commandedSteps[AXIS_COUNT] = {0, 0, 0, 0};
static int32_t activeTargetSteps[AXIS_COUNT] = {0, 0, 0, 0};
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

static void ClearMotionQueue() {
    s_blockUsedCoordinatedXy = false;
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
    return AxisStepsToDisplay(AXIS_X, AxisCurrentSteps(AXIS_X));
}

static double CurrentYInActiveUnits() {
    return AxisStepsToDisplay(AXIS_Y, AxisCurrentSteps(AXIS_Y));
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
            AxisStepsToDisplay(AXIS_X, AxisCurrentSteps(AXIS_X)),
            AxisStepsToDisplay(AXIS_Y, AxisCurrentSteps(AXIS_Y)),
            AxisStepsToDisplay(AXIS_Z, AxisCurrentSteps(AXIS_Z)),
            AxisStepsToDisplay(AXIS_A, AxisCurrentSteps(AXIS_A)));
    SendLine(msg);
}

static void FormatTelemetry(char *msg, size_t msgSize) {
    const char *mode = (coordinateMode == MODE_ABS) ? "ABS" : "REL";
    const char *units = (unitMode == UNITS_MM) ? "MM" : "IN";
    snprintf(msg, msgSize,
             "TEL Active=%d Hold=%d ActiveLine=%ld Queue=%u Mode=%s Units=%s Enabled=%d "
             "Feed=%.3f X=%.3f Y=%.3f Z=%.3f A=%.3f",
             motionRunning ? 1 : 0,
             motionHold ? 1 : 0,
             (long)activeProgramLine,
             queueCount,
             mode,
             units,
             motorsEnabled ? 1 : 0,
             feedRate,
             AxisStepsToDisplay(AXIS_X, AxisCurrentSteps(AXIS_X)),
             AxisStepsToDisplay(AXIS_Y, AxisCurrentSteps(AXIS_Y)),
             AxisStepsToDisplay(AXIS_Z, AxisCurrentSteps(AXIS_Z)),
             AxisStepsToDisplay(AXIS_A, AxisCurrentSteps(AXIS_A)));
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

static MotionBlock BuildMergedHeadBlock() {
    return motionQueue[queueHead];
}

static void PopMergedBlocks(const MotionBlock &) {
    PopQueueHead();
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
    if (g_xyCoordinated) {
        const bool hasXyIntent = (block.axisMask & ((1u << AXIS_X) | (1u << AXIS_Y))) != 0;
        if (hasXyIntent) {
            const int32_t ddx = block.target[AXIS_X] - currentSteps[AXIS_X];
            const int32_t ddy = block.target[AXIS_Y] - currentSteps[AXIS_Y];
            if (ddx != 0 || ddy != 0) {
                uint32_t pathSpeed = block.nominalSpeed;
                if (pathSpeed < 1) {
                    pathSpeed = 1;
                }
                if (pathSpeed > axisConfig[AXIS_X].velocityMax) {
                    pathSpeed = axisConfig[AXIS_X].velocityMax;
                }
                motionController.ArcVelMax(pathSpeed);
                motionController.ArcAccelMax(axisConfig[AXIS_X].accelMax);
                if (!motionController.MoveLinear(block.target[AXIS_X], block.target[AXIS_Y])) {
                    return false;
                }
                s_blockUsedCoordinatedXy = true;
                activeTargetSteps[AXIS_X] = block.target[AXIS_X];
                activeTargetSteps[AXIS_Y] = block.target[AXIS_Y];
                activeAxisMask |= (1u << AXIS_X) | (1u << AXIS_Y);
                anyMove = true;
            }
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
                ClearMotionQueue();
                return true;
            }
        }
        if (MotorDriver *m = MotorForPort(axisConfig[AXIS_Y].port)) {
            if (m->StatusReg().bit.AlertsPresent) {
                ClearMotionQueue();
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
            ClearMotionQueue();
            return true;
        }
        if (motor && axisConfig[axis].enabled && !motor->StepsComplete()) {
            return false;
        }
    }
    return true;
}

static void UpdateMotionExecutor() {
    if (motionRunning) {
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

static void HandleMotionCommand(bool jog, const char *line, const char *lineUpper) {
    if (!motorsInitialized || !motorsEnabled) {
        SendLine("error: motors not enabled");
        return;
    }

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
    if (ParseValue(line, 'F', f) && f > 0.0) {
        feedRate = f;
    }
    ApplyFeedRate();

    double targetX = CurrentXInActiveUnits();
    double targetY = CurrentYInActiveUnits();
    double targetZ = AxisStepsToDisplay(AXIS_Z, currentSteps[AXIS_Z]);
    double targetA = AxisStepsToDisplay(AXIS_A, currentSteps[AXIS_A]);
    if (jog) {
        targetX += hasX ? x : 0.0;
        targetY += hasY ? y : 0.0;
        targetZ += hasZ ? z : 0.0;
        targetA += hasA ? a : 0.0;
    } else if (coordinateMode == MODE_REL) {
        targetX += hasX ? x : 0.0;
        targetY += hasY ? y : 0.0;
        targetZ += hasZ ? z : 0.0;
        targetA += hasA ? a : 0.0;
    } else {
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
    }

    int32_t targets[AXIS_COUNT] = {
        AxisValueToSteps(AXIS_X, targetX),
        AxisValueToSteps(AXIS_Y, targetY),
        AxisValueToSteps(AXIS_Z, targetZ),
        AxisValueToSteps(AXIS_A, targetA)
    };
    uint8_t axisMask = 0;
    if (hasX) axisMask |= (1u << AXIS_X);
    if (hasY) axisMask |= (1u << AXIS_Y);
    if (hasZ) axisMask |= (1u << AXIS_Z);
    if (hasA) axisMask |= (1u << AXIS_A);

    double lineId = 0.0;
    const int32_t programLine = ParseNamedValue(lineUpper, "LN", lineId)
        ? (int32_t)(lineId + 0.5)
        : 0;

    if (MotionQueueFull()) {
        // Backpressure, not a fault: host should retry the same line after a short delay.
        SendLine("busy: queue full");
        return;
    }

    if (!QueueMotionBlock(targets, axisMask, FeedRateToStepsPerSec(feedRate), programLine)) {
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
    sprintf(msg, "CONFIG StepsPerMmX=%.6f StepsPerMmY=%.6f Vel=%lu Accel=%lu Decel=%lu SINGLE=%d AX=%d",
            stepsPerMmX,
            stepsPerMmY,
            (unsigned long)velocityMaxStepsPerSec,
            (unsigned long)accelMaxStepsPerSec2,
            (unsigned long)stopDecelStepsPerSec2,
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
        SendLine("ok: CONFIG (SPMMX,SPMMY,VEL,ACCEL,DECEL,SINGLE,AX) GETCFG M202 M203 M201 M200 G90 G91 G92 M114 M115 G01 JOG");
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
    } else if (strncmp(lineUpper, "MOVE", 4) == 0 || strncmp(lineUpper, "G00", 3) == 0 ||
               strncmp(lineUpper, "G0", 2) == 0 || strncmp(lineUpper, "G01", 3) == 0 ||
               strncmp(lineUpper, "G1", 2) == 0 || strstr(lineUpper, " G00 ") != nullptr ||
               strstr(lineUpper, " G0 ") != nullptr || strstr(lineUpper, " G01 ") != nullptr ||
               strstr(lineUpper, " G1 ") != nullptr) {
        const bool inlineRelative = (strstr(lineUpper, "G91") != nullptr);
        HandleMotionCommand(inlineRelative, lineRaw, lineUpper);
    } else if (strncmp(lineUpper, "JOG", 3) == 0) {
        HandleMotionCommand(true, lineRaw, lineUpper);
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

