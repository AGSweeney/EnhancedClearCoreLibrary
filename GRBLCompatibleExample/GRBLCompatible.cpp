/*
 * Title: GRBLCompatible
 *
 * Objective:
 *    This example provides a GRBL-compatible interface for ClearCore, allowing
 *    use with GRBL-compatible software like Universal G-code Sender, bCNC, CNCjs, etc.
 *
 * Description:
 *    This example implements the GRBL protocol (v1.1) for 3-axis motion.
 *    X and Y axes are coordinated (for arcs and linear moves), while Z axis
 *    moves independently. It supports:
 *    1. GRBL status reporting format
 *    2. Real-time commands (? for status, ! for feed hold, ~ for resume)
 *    3. GRBL error codes
 *    4. Standard GRBL G-codes (G00, G01, G02, G03, G20, G21, G28, G38, G90, G91, G92)
 *       G02/G03: I,J (center offset) or R (radius). Use one, not both. Negative R = arc > 180 deg.
 *    5. GRBL startup message and help commands
 *    6. Home switches, probe input, and limit switches (all optional)
 *
 * Requirements:
 * 1. Three ClearPath motors must be connected: Connector M-0 (X), M-1 (Y), M-2 (Z).
 * 2. The connected ClearPath motors must be configured through the MSP software
 *    for Step and Direction mode (In MSP select Mode>>Step and Direction).
 * 3. The ClearPath motors must be set to use the HLFB mode "ASG-Position
 *    w/Measured Torque" with a PWM carrier frequency of 482 Hz through the MSP
 *    software (select Advanced>>High Level Feedback [Mode]... then choose
 *    "ASG-Position w/Measured Torque" from the dropdown, make sure that 482 Hz
 *    is selected in the "PWM Carrier Frequency" dropdown, and hit the OK
 *    button).
 *
 * GRBL Protocol:
 *    - Status format: <Idle|Run|Hold|Alarm>,MPos:X.xxx,Y.yyy,Z.zzz,FS:F,SS:0>
 *    - Real-time commands: ? (status), ! (feed hold), ~ (resume)
 *    - Error codes: error:1, error:2, etc.
 *    - Startup: "Grbl 1.1f ['$' for help]"
 *
 * Links:
 * ** ClearCore Documentation: https://teknic-inc.github.io/ClearCore-library/
 * ** GRBL Documentation: https://github.com/gnea/grbl/wiki
 *
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
#include <math.h>
#include <string.h>
#include <stdio.h>

// ========== Configuration ==========

// Serial configuration
#define SERIAL_BAUD_RATE 115200
#define SerialPort ConnectorUsb  // Options: ConnectorUsb, ConnectorCOM0, ConnectorCOM1

// Motor configuration
#define motorX ConnectorM0
#define motorY ConnectorM1
#define motorZ ConnectorM2  // Z-axis motor (3rd axis)

// Set to 1 to enable Z-axis (3-axis), 0 to run XY-only (2-axis). When 0, Z is reported as 0 and ignored in moves.
#define USE_Z_AXIS 0

// Home switch configuration (optional - comment out to disable)
// These switches are triggered when the axis reaches its home position
//#define HOME_X_PIN ConnectorIO0  // Comment out this line to disable X home switch
//#define HOME_Y_PIN ConnectorIO1  // Comment out this line to disable Y home switch
//#define HOME_Z_PIN ConnectorDI7  // Comment out this line to disable Z home switch
#define HOME_SWITCH_ACTIVE_LOW true  // true if switch is active low (closed = 0), false if active high

// Probe input configuration (optional - comment out to disable)
// Probe is triggered when probe touches workpiece
//#define PROBE_PIN ConnectorIO2    // Comment out this line to disable probe
#define PROBE_ACTIVE_LOW true     // true if probe is active low (touching = 0), false if active high

// Limit switch configuration (optional - comment out to disable)
// Limit switches prevent motion beyond safe travel limits
//#define LIMIT_X_MIN_PIN ConnectorIO3  // Comment out this line to disable X min limit
//#define LIMIT_X_MAX_PIN ConnectorIO4  // Comment out this line to disable X max limit
//#define LIMIT_Y_MIN_PIN ConnectorIO5  // Comment out this line to disable Y min limit
//#define LIMIT_Y_MAX_PIN ConnectorDI6  // Comment out this line to disable Y max limit
#define LIMIT_SWITCH_ACTIVE_LOW true  // true if switches are active low (triggered = 0), false if active high

// Mechanical configuration
#define MOTOR_X_STEPS_PER_REV 800
#define MOTOR_X_PITCH_MM 5.0
#define MOTOR_Y_STEPS_PER_REV 800
#define MOTOR_Y_PITCH_MM 5.0
#define MOTOR_Z_STEPS_PER_REV 800
#define MOTOR_Z_PITCH_MM 5.0

// Default motion parameters
#define DEFAULT_FEED_RATE_MM_PER_MIN 2540.0  // 100 inches/min
#define DEFAULT_VELOCITY_STEPS_PER_SEC 5000
#define DEFAULT_ACCELERATION_STEPS_PER_SEC2 85000
// Junction deviation in steps (cornering tolerance)
#define DEFAULT_JUNCTION_DEVIATION_STEPS 40

// GRBL status reporting interval (ms)
#define STATUS_REPORT_INTERVAL 250

// ========== Global Variables ==========

CoordinatedMotionController motionController;

// Store mechanical config for unit conversion
MotorMechanicalConfig mechanicalConfigX;
MotorMechanicalConfig mechanicalConfigY;
MotorMechanicalConfig mechanicalConfigZ;

// GRBL state machine
enum GrblState {
    STATE_IDLE,
    STATE_RUN,
    STATE_HOLD,
    STATE_ALARM,
    STATE_DOOR,
    STATE_CHECK,
    STATE_HOME,
    STATE_SLEEP
};

GrblState grblState = STATE_IDLE;
bool feedHoldActive = false;
bool motorsEnabled = false;
bool homed = false;  // True if homing cycle has been completed
bool probeTriggered = false;  // True if probe is currently triggered

// Command parsing state
enum CoordinateMode {
    COORD_ABSOLUTE,  // G90
    COORD_INCREMENTAL  // G91
};

enum UnitMode {
    UNIT_MODE_INCHES,  // G20
    UNIT_MODE_MM  // G21
};

CoordinateMode coordinateMode = COORD_ABSOLUTE;
UnitMode unitMode = UNIT_MODE_MM;
double currentFeedRate = DEFAULT_FEED_RATE_MM_PER_MIN;
bool feedRateSet = false;
uint32_t savedVelocityMax = DEFAULT_VELOCITY_STEPS_PER_SEC;  // Store velocity for rapid moves

// Command buffer
#define MAX_LINE_LENGTH 256
char commandBuffer[MAX_LINE_LENGTH];
uint16_t commandBufferIndex = 0;

// Status reporting
uint32_t lastStatusReportTime = 0;
bool statusReportRequested = false;

// Z-axis position tracking (in steps)
int32_t currentZSteps = 0;

// Motor initialization status
bool motorsInitialized = false;

// ========== Function Prototypes ==========

void InitializeMotors();
void InitializeCommunication();
void ProcessCommand(const char* command);
void ParseGCode(const char* line);
void ExecuteG00(double x, double y, double z = 0.0);  // Rapid move
void ExecuteG01(double x, double y, double z = 0.0, double f = -1.0);
void ExecuteG02(double x, double y, double z, double i, double j, double f = -1.0);
void ExecuteG03(double x, double y, double z, double i, double j, double f = -1.0);
void ExecuteG28();  // Home
void ExecuteG38(double x, double y, double z, double f);  // Probe
bool CheckHomeSwitches();
bool CheckLimitSwitches();
bool CheckProbe();
void InitializeInputs();
void MoveZAxis(int32_t targetZSteps, bool rapid = false);  // Move Z axis independently
double GetCurrentZPosition();  // Get current Z position in current units
static void ExecuteJog(const char* rest);  // $J= jog (GRBL v1.1)
bool ArcRToIJ(double startX, double startY, double endX, double endY, double r,
              bool clockwise, double& outI, double& outJ);  // GRBL-style R-to-IJ for G2/G3
void SendResponse(const char* message);
void SendResponseLine(const char* message);
void SendError(uint8_t errorCode);
void SendStatusReport();
bool ReadCommand(char* buffer, uint16_t maxLen);
void ProcessRealtimeCommand(char cmd);

// ========== Main Function ==========

int main() {
    // Small delay to ensure system is fully initialized
    Delay_ms(100);
    
    // Initialize inputs (switches, probe)
    InitializeInputs();
    
    // Initialize motors
    InitializeMotors();
    
    // Initialize communication
    InitializeCommunication();
    
    // Small delay before sending startup message
    Delay_ms(100);
    
    // Send GRBL startup message
    SendResponseLine("Grbl 1.1f ['$' for help]");
    if (!motorsInitialized) {
        SendResponseLine("ALARM: Motors not initialized (motors may not be attached)");
    }
    
    // Main loop
    while (true) {
        // Check for incoming commands
        if (ReadCommand(commandBuffer, MAX_LINE_LENGTH)) {
            ProcessCommand(commandBuffer);
            // Clear buffer
            memset(commandBuffer, 0, MAX_LINE_LENGTH);
            commandBufferIndex = 0;
        }
        
        // Check limit switches - trigger alarm if activated
        if (CheckLimitSwitches()) {
            grblState = STATE_ALARM;
            if (motorsInitialized) {
                motionController.Stop();
                #if USE_Z_AXIS
                motorZ.MoveStopAbrupt();
                #endif
            }
        }
        
        // Check probe state
        probeTriggered = CheckProbe();
        
        #if USE_Z_AXIS
        // Update Z position from motor (if not moving and motors initialized)
        if (motorsInitialized && !motorZ.StatusReg().bit.StepsActive) {
            currentZSteps = motorZ.PositionRefCommanded();
        }
        #endif
        
        // Update GRBL state based on motion controller
        if (grblState != STATE_ALARM && motorsInitialized) {
            if (motionController.IsActive() && !feedHoldActive) {
                grblState = STATE_RUN;
            } else if (feedHoldActive && motionController.IsActive()) {
                grblState = STATE_HOLD;
            } else {
                grblState = STATE_IDLE;
            }
        }
        
        // Send status report periodically or on request
        uint32_t currentTime = Milliseconds();
        if (statusReportRequested || (currentTime - lastStatusReportTime >= STATUS_REPORT_INTERVAL)) {
            SendStatusReport();
            lastStatusReportTime = currentTime;
            statusReportRequested = false;
        }
        
        // Small delay to prevent CPU spinning
        Delay_ms(1);
    }
    
    return 0;
}

// ========== Initialization Functions ==========

void InitializeMotors() {
    motorsInitialized = false;
    
    // Set motors to Step and Direction mode (XY only when Z disabled)
    #if USE_Z_AXIS
    MotorMgr.MotorModeSet(MotorManager::MOTOR_ALL, Connector::CPM_MODE_STEP_AND_DIR);
    #else
    MotorMgr.MotorModeSet(MotorManager::MOTOR_M0M1, Connector::CPM_MODE_STEP_AND_DIR);
    #endif
    
    // Try to enable motors (non-blocking - motors may not be attached)
    motorX.EnableRequest(true);
    motorY.EnableRequest(true);
    #if USE_Z_AXIS
    motorZ.EnableRequest(true);
    #endif
    motorsEnabled = true;
    
    // Short delay to allow enable signal to propagate (non-blocking)
    Delay_ms(100);
    
    // Initialize coordinated motion controller
    // This may fail if motors are not present or not properly configured
    if (!motionController.Initialize(&motorX, &motorY)) {
        // Motors not available - continue without motors
        // Communication and other features will still work
        grblState = STATE_ALARM;
        return;
    }
    
    // Configure mechanical parameters for unit conversion
    motionController.SetMechanicalParamsX(MOTOR_X_STEPS_PER_REV, MOTOR_X_PITCH_MM, ClearCore::UNIT_MM);
    motionController.SetMechanicalParamsY(MOTOR_Y_STEPS_PER_REV, MOTOR_Y_PITCH_MM, ClearCore::UNIT_MM);
    
    #if USE_Z_AXIS
    // Configure Z-axis motor mechanical parameters
    motorZ.SetMechanicalParams(MOTOR_Z_STEPS_PER_REV, MOTOR_Z_PITCH_MM, ClearCore::UNIT_MM, 1.0);
    mechanicalConfigZ.stepsPerRevolution = MOTOR_Z_STEPS_PER_REV;
    mechanicalConfigZ.pitch = MOTOR_Z_PITCH_MM;
    mechanicalConfigZ.pitchUnit = ClearCore::UNIT_MM;
    mechanicalConfigZ.gearRatio = 1.0;
    UnitConverter::CalculateConversionFactors(mechanicalConfigZ);
    #endif
    
    // Store config locally for unit conversion
    mechanicalConfigX.stepsPerRevolution = MOTOR_X_STEPS_PER_REV;
    mechanicalConfigX.pitch = MOTOR_X_PITCH_MM;
    mechanicalConfigX.pitchUnit = ClearCore::UNIT_MM;
    mechanicalConfigX.gearRatio = 1.0;
    UnitConverter::CalculateConversionFactors(mechanicalConfigX);
    
    mechanicalConfigY.stepsPerRevolution = MOTOR_Y_STEPS_PER_REV;
    mechanicalConfigY.pitch = MOTOR_Y_PITCH_MM;
    mechanicalConfigY.pitchUnit = ClearCore::UNIT_MM;
    mechanicalConfigY.gearRatio = 1.0;
    UnitConverter::CalculateConversionFactors(mechanicalConfigY);
    
    // Set default feed rate
    motionController.FeedRateMMPerMin(DEFAULT_FEED_RATE_MM_PER_MIN);
    currentFeedRate = DEFAULT_FEED_RATE_MM_PER_MIN;
    feedRateSet = true;
    
    // Set motion parameters
    motionController.ArcVelMax(DEFAULT_VELOCITY_STEPS_PER_SEC);
    savedVelocityMax = DEFAULT_VELOCITY_STEPS_PER_SEC;  // Store for rapid move restore
    motionController.ArcAccelMax(DEFAULT_ACCELERATION_STEPS_PER_SEC2);
    motionController.JunctionDeviationSteps(DEFAULT_JUNCTION_DEVIATION_STEPS);
    
    // Set initial position
    motionController.SetPosition(0, 0);
    currentZSteps = 0;
    
    // Mark motors as successfully initialized
    motorsInitialized = true;
    grblState = STATE_IDLE;  // Clear alarm state if initialization succeeded
}

void InitializeCommunication() {
    // Initialize serial port
    SerialPort.Mode(Connector::USB_CDC);
    SerialPort.Speed(SERIAL_BAUD_RATE);
    SerialPort.PortOpen();
    
    // Wait for serial port to be ready (non-blocking with timeout)
    uint32_t timeout = 2000;  // Reduced timeout to 2 seconds
    uint32_t startTime = Milliseconds();
    while (!SerialPort && Milliseconds() - startTime < timeout) {
        Delay_ms(10);  // Small delay to prevent tight loop
    }
    
    // Continue even if serial port isn't ready - device will still function
    // Serial port will become available when USB is connected
}

// ========== Command Processing Functions ==========

bool ReadCommand(char* buffer, uint16_t maxLen) {
    int16_t ch = SerialPort.CharGet();
    if (ch == -1) {
        return false;
    }
    
    // Handle real-time commands immediately (GRBL protocol)
    if (ch == '?' || ch == '!' || ch == '~' || ch == 0x18 || ch == 0x85) {  // ?, !, ~, Ctrl-X, jog cancel
        ProcessRealtimeCommand((char)ch);
        return false;
    }
    
    // Handle newline (command complete)
    if (ch == '\n' || ch == '\r') {
        if (commandBufferIndex > 0) {
            buffer[commandBufferIndex] = '\0';
            return true;
        }
        return false;
    }
    
    // Add character to buffer
    if (commandBufferIndex < maxLen - 1) {
        buffer[commandBufferIndex++] = (char)ch;
    } else {
        // Buffer overflow - reset
        commandBufferIndex = 0;
        memset(buffer, 0, maxLen);
        SendError(23);  // Buffer overflow
    }
    
    return false;
}

void ProcessRealtimeCommand(char cmd) {
    switch (cmd) {
        case '?':  // Status report request
            statusReportRequested = true;
            break;
            
        case '!':  // Feed hold
            if (grblState == STATE_RUN && motorsInitialized) {
                feedHoldActive = true;
                motionController.StopDecel();
                #if USE_Z_AXIS
                motorZ.MoveStopDecel();
                #endif
                grblState = STATE_HOLD;
            }
            break;
            
        case '~':  // Resume
            if (grblState == STATE_HOLD && motorsInitialized) {
                feedHoldActive = false;
                grblState = STATE_RUN;
                // Motion will resume automatically from queue
            }
            break;
            
        case 0x18:  // Ctrl-X (soft reset)
            if (motorsInitialized) {
                motionController.Stop();
                #if USE_Z_AXIS
                motorZ.MoveStopAbrupt();
                #endif
            }
            feedHoldActive = false;
            grblState = STATE_IDLE;
            SendResponseLine("ok");
            SendResponseLine("Grbl 1.1f ['$' for help]");
            break;
            
        case 0x85:  // Jog cancel (GRBL v1.1) – stop on release
            if (motorsInitialized && motionController.IsActive()) {
                motionController.Stop();
                #if USE_Z_AXIS
                motorZ.MoveStopAbrupt();
                #endif
            }
            feedHoldActive = false;
            if (grblState == STATE_RUN || grblState == STATE_HOLD) {
                grblState = STATE_IDLE;
            }
            break;
    }
}

static void TrimTrailingWhitespace(char* s) {
    size_t len = strlen(s);
    while (len > 0 && (s[len - 1] == ' ' || s[len - 1] == '\t' || s[len - 1] == '\r')) {
        s[--len] = '\0';
    }
}

// Wait for XY coordinated motion to finish and queue empty.
// Returns true when motion completes.
static bool WaitForMotionComplete(void) {
    while (motionController.IsActive() || motionController.MotionQueueCount() != 0) {
        uint32_t currentTime = Milliseconds();
        if (currentTime - lastStatusReportTime >= STATUS_REPORT_INTERVAL) {
            SendStatusReport();
            lastStatusReportTime = currentTime;
        }
        Delay_ms(1);
    }
    return true;
}

// Jog wait: allow 0x85 cancel during jog (safe to consume input here).
// Returns true if motion completed, false if cancelled.
static bool WaitForJogComplete(void) {
    while (motionController.IsActive() || motionController.MotionQueueCount() != 0) {
        uint32_t currentTime = Milliseconds();
        if (currentTime - lastStatusReportTime >= STATUS_REPORT_INTERVAL) {
            SendStatusReport();
            lastStatusReportTime = currentTime;
        }
        int16_t ch = SerialPort.CharGet();
        if (ch >= 0 && ch == 0x85) {
            motionController.Stop();
            #if USE_Z_AXIS
            motorZ.MoveStopAbrupt();
            #endif
            return false;
        }
        Delay_ms(1);
    }
    return true;
}

void ProcessCommand(const char* command) {
    // Skip empty commands
    if (strlen(command) == 0) {
        return;
    }
    
    // Convert to uppercase for parsing
    char upperCommand[MAX_LINE_LENGTH];
    strncpy(upperCommand, command, MAX_LINE_LENGTH - 1);
    upperCommand[MAX_LINE_LENGTH - 1] = '\0';
    
    for (uint16_t i = 0; i < strlen(upperCommand); i++) {
        if (upperCommand[i] >= 'a' && upperCommand[i] <= 'z') {
            upperCommand[i] = upperCommand[i] - 'a' + 'A';
        }
    }
    TrimTrailingWhitespace(upperCommand);

    // Ignore sender metadata lines (gSender "feeder ..." or status echoes)
    if (strncmp(upperCommand, "FEEDER ", 7) == 0 ||
        strncmp(upperCommand, "SG=", 3) == 0 ||
        strncmp(upperCommand, "$G=", 3) == 0) {
        SendResponseLine("ok");
        return;
    }
    
    // Handle $ commands (GRBL configuration)
    if (upperCommand[0] == '$') {
        if (strcmp(upperCommand, "$") == 0 || strcmp(upperCommand, "$G") == 0) {
            const char* coordStr = (coordinateMode == COORD_INCREMENTAL) ? "G91" : "G90";
            const char* unitStr = (unitMode == UNIT_MODE_INCHES) ? "G20" : "G21";
            char msg[32];
            snprintf(msg, sizeof(msg), "$G=%s %s", coordStr, unitStr);
            SendResponseLine(msg);
        } else if (strcmp(upperCommand, "$I") == 0) {
            SendResponseLine("ClearCore GRBL-Compatible v1.0");
        } else if (strcmp(upperCommand, "$N") == 0) {
            SendResponseLine("$N=");
        } else if (strcmp(upperCommand, "$$") == 0) {
            // Minimal $$ response for gSender compatibility (GRBL settings)
            SendResponseLine("$0=10");
            SendResponseLine("$1=25");
            SendResponseLine("$10=1");
            SendResponseLine("$100=250.000");
            SendResponseLine("$101=250.000");
            SendResponseLine("$102=250.000");
            SendResponseLine("$110=500.000");
            SendResponseLine("$111=500.000");
            SendResponseLine("$112=500.000");
            SendResponseLine("$120=10.000");
            SendResponseLine("$121=10.000");
            SendResponseLine("$122=10.000");
            SendResponseLine("$130=200.000");
            SendResponseLine("$131=200.000");
            SendResponseLine("$132=200.000");
            SendResponseLine("ok");
        } else if (strcmp(upperCommand, "$#") == 0) {
            // Parameters (WCS offsets) for gSender
            SendResponseLine("[G54:0.000,0.000,0.000]");
            SendResponseLine("[G55:0.000,0.000,0.000]");
            SendResponseLine("[G56:0.000,0.000,0.000]");
            SendResponseLine("[G57:0.000,0.000,0.000]");
            SendResponseLine("[G58:0.000,0.000,0.000]");
            SendResponseLine("[G59:0.000,0.000,0.000]");
            SendResponseLine("[G28:0.000,0.000,0.000]");
            SendResponseLine("[G30:0.000,0.000,0.000]");
            SendResponseLine("ok");
        } else if (strcmp(upperCommand, "$H") == 0) {
            // Homing cycle (gSender Home button)
            ExecuteG28();
        } else if (strncmp(upperCommand, "$J=", 3) == 0) {
            // Jog (GRBL v1.1) - minimal support for gSender
            ExecuteJog(upperCommand + 3);
        } else {
            SendError(20);  // Unsupported command
        }
        return;
    }

    // Ignore non-command lines (sender metadata or empty noise)
    if (upperCommand[0] != 'G' && upperCommand[0] != 'M') {
        SendResponseLine("ok");
        return;
    }

    // Parse G-code command
    ParseGCode(upperCommand);
}

void ParseGCode(const char* line) {
    // Parse command line (e.g., "G01 X10.5 Y20.3 Z5.0 F100")
    double x = 0, y = 0, z = 0, i = 0, j = 0, r = 0, f = -1.0, p = 0;
    bool hasX = false, hasY = false, hasZ = false, hasI = false, hasJ = false, hasR = false, hasP = false;
    
    // Find command code (G## or M##)
    if (line[0] == 'G' || line[0] == 'M') {
        int code = 0;
        if (sscanf(line, "%*[GM]%d", &code) != 1) {
            // Ignore malformed command stubs like "G" or "M"
            SendResponseLine("ok");
            return;
        }
        
        // Parse parameters
        const char* ptr = line;
        while (*ptr) {
            if (*ptr == 'X' || *ptr == 'x') {
                sscanf(ptr, "X%lf", &x);
                hasX = true;
            } else if (*ptr == 'Y' || *ptr == 'y') {
                sscanf(ptr, "Y%lf", &y);
                hasY = true;
            } else if (*ptr == 'Z' || *ptr == 'z') {
                sscanf(ptr, "Z%lf", &z);
                hasZ = true;
            } else if (*ptr == 'I' || *ptr == 'i') {
                sscanf(ptr, "I%lf", &i);
                hasI = true;
            } else if (*ptr == 'J' || *ptr == 'j') {
                sscanf(ptr, "J%lf", &j);
                hasJ = true;
            } else if (*ptr == 'R' || *ptr == 'r') {
                sscanf(ptr, "R%lf", &r);
                hasR = true;
            } else if (*ptr == 'F' || *ptr == 'f') {
                sscanf(ptr, "F%lf", &f);
            } else if (*ptr == 'P' || *ptr == 'p') {
                sscanf(ptr, "P%lf", &p);
                hasP = true;
            }
            ptr++;
        }
        
        // Execute command
        if (line[0] == 'G') {
            switch (code) {
                case 0:  // G00 - Rapid move
                    if (hasX || hasY || hasZ) {
                        ExecuteG00(x, y, z);
                    } else {
                        // No axis words; treat as a no-op modal update
                        SendResponseLine("ok");
                    }
                    break;
                    
                case 1:  // G01 - Linear interpolation
                    if (hasX || hasY || hasZ) {
                        ExecuteG01(x, y, z, f);
                    } else {
                        // No axis words; treat as a no-op modal update
                        SendResponseLine("ok");
                    }
                    break;
                    
                case 2:  // G02 - Circular interpolation CW
                    if (hasX && hasY) {
                        if ((hasI || hasJ) && hasR) {
                            SendError(20);  // Use I,J or R, not both
                        } else if (hasR) {
                            double startX = (unitMode == UNIT_MODE_INCHES)
                                ? motionController.CurrentXInches() : motionController.CurrentXMM();
                            double startY = (unitMode == UNIT_MODE_INCHES)
                                ? motionController.CurrentYInches() : motionController.CurrentYMM();
                            double endX = (coordinateMode == COORD_ABSOLUTE) ? x : startX + x;
                            double endY = (coordinateMode == COORD_ABSOLUTE) ? y : startY + y;
                            double iVal, jVal;
                            if (!ArcRToIJ(startX, startY, endX, endY, r, true, iVal, jVal)) {
                                SendError(2);  // Arc radius error or target same as current
                            } else {
                                ExecuteG02(x, y, z, iVal, jVal, f);
                            }
                        } else if (hasI && hasJ) {
                            ExecuteG02(x, y, z, i, j, f);
                        } else {
                            SendError(1);  // Missing I,J or R
                        }
                    } else {
                        SendError(1);  // Missing X,Y
                    }
                    break;
                    
                case 3:  // G03 - Circular interpolation CCW
                    if (hasX && hasY) {
                        if ((hasI || hasJ) && hasR) {
                            SendError(20);  // Use I,J or R, not both
                        } else if (hasR) {
                            double startX = (unitMode == UNIT_MODE_INCHES)
                                ? motionController.CurrentXInches() : motionController.CurrentXMM();
                            double startY = (unitMode == UNIT_MODE_INCHES)
                                ? motionController.CurrentYInches() : motionController.CurrentYMM();
                            double endX = (coordinateMode == COORD_ABSOLUTE) ? x : startX + x;
                            double endY = (coordinateMode == COORD_ABSOLUTE) ? y : startY + y;
                            double iVal, jVal;
                            if (!ArcRToIJ(startX, startY, endX, endY, r, false, iVal, jVal)) {
                                SendError(2);  // Arc radius error or target same as current
                            } else {
                                ExecuteG03(x, y, z, iVal, jVal, f);
                            }
                        } else if (hasI && hasJ) {
                            ExecuteG03(x, y, z, i, j, f);
                        } else {
                            SendError(1);  // Missing I,J or R
                        }
                    } else {
                        SendError(1);  // Missing X,Y
                    }
                    break;
                    
                case 4:  // G4 - Dwell
                    if (hasP && p >= 0) {
                        uint32_t ms = (uint32_t)(p * 1000.0 + 0.5);
                        if (ms > 60000) ms = 60000;  // cap 60 s
                        Delay_ms(ms);
                        SendResponseLine("ok");
                    } else {
                        SendError(1);  // Missing or invalid P
                    }
                    break;
                    
                case 20:  // G20 - Units in inches
                    unitMode = UNIT_MODE_INCHES;
                    SendResponseLine("ok");
                    break;
                    
                case 21:  // G21 - Units in millimeters
                    unitMode = UNIT_MODE_MM;
                    SendResponseLine("ok");
                    break;

                case 54:  // G54 - Work coordinate system (acknowledge)
                case 55:  // G55
                case 56:  // G56
                case 57:  // G57
                case 58:  // G58
                case 59:  // G59
                    SendResponseLine("ok");
                    break;
                    
                case 28:  // G28 - Home
                    ExecuteG28();
                    // ExecuteG28 sends ok internally
                    break;
                    
                case 38:  // G38 - Probe (G38.2)
                    if (hasX || hasY || hasZ) {
                        ExecuteG38(x, y, z, f);
                        // ExecuteG38 sends ok internally
                    } else {
                        SendError(1);
                    }
                    break;
                    
                case 90:  // G90 - Absolute coordinates
                    coordinateMode = COORD_ABSOLUTE;
                    SendResponseLine("ok");
                    break;
                    
                case 91:  // G91 - Incremental coordinates
                    coordinateMode = COORD_INCREMENTAL;
                    SendResponseLine("ok");
                    break;
                    
                case 92:  // G92 - Set coordinate system offset
                    if (!motorsInitialized) {
                        SendError(9);  // Motors not initialized
                        break;
                    }
                    if (hasX || hasY) {
                        int32_t newXSteps, newYSteps;
                        if (unitMode == UNIT_MODE_INCHES) {
                            newXSteps = hasX ? UnitConverter::DistanceToSteps(x, ClearCore::UNIT_INCHES, mechanicalConfigX) : motionController.CurrentX();
                            newYSteps = hasY ? UnitConverter::DistanceToSteps(y, ClearCore::UNIT_INCHES, mechanicalConfigY) : motionController.CurrentY();
                        } else {
                            newXSteps = hasX ? UnitConverter::DistanceToSteps(x, ClearCore::UNIT_MM, mechanicalConfigX) : motionController.CurrentX();
                            newYSteps = hasY ? UnitConverter::DistanceToSteps(y, ClearCore::UNIT_MM, mechanicalConfigY) : motionController.CurrentY();
                        }
                        motionController.SetPosition(newXSteps, newYSteps);
                        SendResponseLine("ok");
                    } else {
                        motionController.SetPosition(0, 0);
                        SendResponseLine("ok");
                    }
                    break;
                    
                default:
                    SendError(2);  // Numeric value format is not valid or missing an expected value
                    break;
            }
        } else if (line[0] == 'M') {
            switch (code) {
                case 0:  // M0 - Program stop
                case 1:  // M1 - Optional stop
                    if (motorsInitialized) {
                        motionController.StopDecel();
                        #if USE_Z_AXIS
                        motorZ.MoveStopDecel();
                        #endif
                    }
                    grblState = STATE_IDLE;
                    SendResponseLine("ok");
                    break;
                    
                case 3:  // M3 - Spindle CW (not applicable, but acknowledge)
                case 4:  // M4 - Spindle CCW (not applicable, but acknowledge)
                case 5:  // M5 - Spindle stop (not applicable, but acknowledge)
                    SendResponseLine("ok");
                    break;
                    
                case 30:  // M30 - Program end
                    if (motorsInitialized) {
                        // Wait for queued motion to finish before stopping
                        WaitForMotionComplete();
                        motionController.Stop();
                        #if USE_Z_AXIS
                        motorZ.MoveStopAbrupt();
                        #endif
                    }
                    grblState = STATE_IDLE;
                    SendResponseLine("ok");
                    break;
                    
                default:
                    SendError(2);
                    break;
            }
        }
    } else {
        SendError(20);  // Unsupported command
    }
}

void ExecuteG00(double x, double y, double z) {
    // Check if motors are initialized
    if (!motorsInitialized) {
        SendError(9);  // Homing cycle not started (or motors not initialized)
        return;
    }
    
    // Rapid move - use maximum velocity
    if (!motorsEnabled) {
        SendError(9);  // Homing cycle not started
        return;
    }
    
    // Check limit switches before moving
    if (CheckLimitSwitches()) {
        grblState = STATE_ALARM;
        SendError(9);
        return;
    }
    
    // Calculate target position for X/Y
    double targetX = x;
    double targetY = y;
    #if USE_Z_AXIS
    double targetZ = z;
    #endif
    
    if (coordinateMode == COORD_INCREMENTAL) {
        if (unitMode == UNIT_MODE_INCHES) {
            targetX = motionController.CurrentXInches() + x;
            targetY = motionController.CurrentYInches() + y;
            #if USE_Z_AXIS
            targetZ = GetCurrentZPosition() + z;
            #endif
        } else {
            targetX = motionController.CurrentXMM() + x;
            targetY = motionController.CurrentYMM() + y;
            #if USE_Z_AXIS
            targetZ = GetCurrentZPosition() + z;
            #endif
        }
    }
    
    // Convert to steps
    int32_t endXSteps, endYSteps;
    #if USE_Z_AXIS
    int32_t endZSteps;
    #endif
    if (unitMode == UNIT_MODE_INCHES) {
        endXSteps = UnitConverter::DistanceToSteps(targetX, ClearCore::UNIT_INCHES, mechanicalConfigX);
        endYSteps = UnitConverter::DistanceToSteps(targetY, ClearCore::UNIT_INCHES, mechanicalConfigY);
        #if USE_Z_AXIS
        endZSteps = UnitConverter::DistanceToSteps(targetZ, ClearCore::UNIT_INCHES, mechanicalConfigZ);
        #endif
    } else {
        endXSteps = UnitConverter::DistanceToSteps(targetX, ClearCore::UNIT_MM, mechanicalConfigX);
        endYSteps = UnitConverter::DistanceToSteps(targetY, ClearCore::UNIT_MM, mechanicalConfigY);
        #if USE_Z_AXIS
        endZSteps = UnitConverter::DistanceToSteps(targetZ, ClearCore::UNIT_MM, mechanicalConfigZ);
        #endif
    }

    // No XY move needed; handle Z-only or acknowledge.
    if (endXSteps == motionController.CurrentX() && endYSteps == motionController.CurrentY()) {
        #if USE_Z_AXIS
        if (endZSteps != currentZSteps) {
            MoveZAxis(endZSteps, true);
        }
        #endif
        SendResponseLine("ok");
        return;
    }
    
    // Use high velocity for rapid move
    motionController.ArcVelMax(DEFAULT_VELOCITY_STEPS_PER_SEC * 2);  // 2x normal speed
    
    // Queue X/Y coordinated move (wait for planner space)
    bool success = false;
    uint32_t startWait = Milliseconds();
    while (!success) {
        success = motionController.QueueLinear(endXSteps, endYSteps);
        if (success) {
            break;
        }
        if (Milliseconds() - startWait > 5000) {
            break;
        }
        Delay_ms(1);
    }
    if (!success) {
        motionController.ArcVelMax(savedVelocityMax);
        SendError(24);  // Queue full or reject
        return;
    }
    
    #if USE_Z_AXIS
    // Move Z axis independently (rapid)
    MoveZAxis(endZSteps, true);
    #endif
    
    WaitForMotionComplete();
    motionController.ArcVelMax(savedVelocityMax);
    SendResponseLine("ok");
}

void ExecuteG01(double x, double y, double z, double f) {
    // Check if motors are initialized
    if (!motorsInitialized) {
        SendError(9);  // Homing cycle not started (or motors not initialized)
        return;
    }
    
    // Set feed rate if specified
    if (f >= 0.0) {
        if (unitMode == UNIT_MODE_INCHES) {
            motionController.FeedRateInchesPerMin(f);
            currentFeedRate = f;
        } else {
            motionController.FeedRateMMPerMin(f);
            currentFeedRate = f;
        }
        feedRateSet = true;
    } else if (feedRateSet) {
        if (unitMode == UNIT_MODE_INCHES) {
            motionController.FeedRateInchesPerMin(currentFeedRate);
        } else {
            motionController.FeedRateMMPerMin(currentFeedRate);
        }
    }
    
    if (!motorsEnabled) {
        SendError(9);
        return;
    }
    
    // Check limit switches before moving
    if (CheckLimitSwitches()) {
        grblState = STATE_ALARM;
        SendError(9);
        return;
    }
    
    // Calculate target position
    double targetX = x;
    double targetY = y;
    
    if (coordinateMode == COORD_INCREMENTAL) {
        if (unitMode == UNIT_MODE_INCHES) {
            targetX = motionController.CurrentXInches() + x;
            targetY = motionController.CurrentYInches() + y;
        } else {
            targetX = motionController.CurrentXMM() + x;
            targetY = motionController.CurrentYMM() + y;
        }
    }
    
    // Convert to steps
    int32_t endXSteps, endYSteps;
    if (unitMode == UNIT_MODE_INCHES) {
        endXSteps = UnitConverter::DistanceToSteps(targetX, ClearCore::UNIT_INCHES, mechanicalConfigX);
        endYSteps = UnitConverter::DistanceToSteps(targetY, ClearCore::UNIT_INCHES, mechanicalConfigY);
    } else {
        endXSteps = UnitConverter::DistanceToSteps(targetX, ClearCore::UNIT_MM, mechanicalConfigX);
        endYSteps = UnitConverter::DistanceToSteps(targetY, ClearCore::UNIT_MM, mechanicalConfigY);
    }

    // No XY move needed; handle Z-only or acknowledge.
    if (endXSteps == motionController.CurrentX() && endYSteps == motionController.CurrentY()) {
        #if USE_Z_AXIS
        if (z != 0.0 || coordinateMode == COORD_ABSOLUTE) {
            double targetZ = z;
            if (coordinateMode == COORD_INCREMENTAL) {
                targetZ = GetCurrentZPosition() + z;
            }
            int32_t targetZSteps;
            if (unitMode == UNIT_MODE_INCHES) {
                targetZSteps = UnitConverter::DistanceToSteps(targetZ, ClearCore::UNIT_INCHES, mechanicalConfigZ);
            } else {
                targetZSteps = UnitConverter::DistanceToSteps(targetZ, ClearCore::UNIT_MM, mechanicalConfigZ);
            }
            MoveZAxis(targetZSteps, false);
        }
        #endif
        SendResponseLine("ok");
        return;
    }

    bool success = false;
    uint32_t startWait = Milliseconds();
    while (!success) {
        success = motionController.QueueLinear(endXSteps, endYSteps);
        if (success) {
            break;
        }
        if (Milliseconds() - startWait > 5000) {
            break;
        }
        Delay_ms(1);
    }
    if (!success) {
        SendError(24);
        return;
    }
    
    #if USE_Z_AXIS
    // Handle Z-axis movement independently
    if (z != 0.0 || coordinateMode == COORD_ABSOLUTE) {
        double targetZ = z;
        if (coordinateMode == COORD_INCREMENTAL) {
            targetZ = GetCurrentZPosition() + z;
        }
        
        int32_t targetZSteps;
        if (unitMode == UNIT_MODE_INCHES) {
            targetZSteps = UnitConverter::DistanceToSteps(targetZ, ClearCore::UNIT_INCHES, mechanicalConfigZ);
        } else {
            targetZSteps = UnitConverter::DistanceToSteps(targetZ, ClearCore::UNIT_MM, mechanicalConfigZ);
        }
        
        MoveZAxis(targetZSteps, false);
    }
    #endif
    
    WaitForMotionComplete();
    SendResponseLine("ok");
}

// Minimal $J= jog (GRBL v1.1). Parses "G91 G20 X0.5 F10" etc. Does not alter parser state.
static void ExecuteJog(const char* rest) {
    if (!motorsInitialized) {
        SendError(9);
        return;
    }
    if (!motorsEnabled) {
        SendError(9);
        return;
    }
    if (CheckLimitSwitches()) {
        grblState = STATE_ALARM;
        SendError(9);
        return;
    }

    bool relative = (strstr(rest, "G91") != nullptr);
    bool inches = (strstr(rest, "G20") != nullptr);
    if (strstr(rest, "G90") != nullptr) {
        relative = false;
    }
    if (strstr(rest, "G21") != nullptr) {
        inches = false;
    }

    double x = 0, y = 0, z = 0, f = -1;
    bool hasX = false, hasY = false, hasZ = false, hasF = false;
    const char* ptr = rest;
    while (*ptr) {
        if (*ptr == 'X') {
            sscanf(ptr, "X%lf", &x);
            hasX = true;
        } else if (*ptr == 'Y') {
            sscanf(ptr, "Y%lf", &y);
            hasY = true;
        } else if (*ptr == 'Z') {
            sscanf(ptr, "Z%lf", &z);
            hasZ = true;
        } else if (*ptr == 'F') {
            sscanf(ptr, "F%lf", &f);
            hasF = true;
        }
        ptr++;
    }

    if (!hasF || (!hasX && !hasY && !hasZ)) {
        SendError(1);
        return;
    }

    double curX = inches ? motionController.CurrentXInches() : motionController.CurrentXMM();
    double curY = inches ? motionController.CurrentYInches() : motionController.CurrentYMM();
    double targetX = relative ? (curX + x) : x;
    double targetY = relative ? (curY + y) : y;

    ClearCore::UnitType u = inches ? ClearCore::UNIT_INCHES : ClearCore::UNIT_MM;
    int32_t endXSteps = UnitConverter::DistanceToSteps(targetX, u, mechanicalConfigX);
    int32_t endYSteps = UnitConverter::DistanceToSteps(targetY, u, mechanicalConfigY);

    if (inches) {
        motionController.FeedRateInchesPerMin(f);
    } else {
        motionController.FeedRateMMPerMin(f);
    }

    bool success = false;
    uint32_t startWait = Milliseconds();
    while (!success) {
        success = motionController.QueueLinear(endXSteps, endYSteps);
        if (success) {
            break;
        }
        if (Milliseconds() - startWait > 5000) {
            break;
        }
        Delay_ms(1);
    }
    if (!success) {
        SendError(24);
        return;
    }

    bool completed = WaitForJogComplete();

    #if USE_Z_AXIS
    if (hasZ && completed) {
        double curZ = UnitConverter::StepsToDistance(currentZSteps, u, mechanicalConfigZ);
        double targetZ = relative ? (curZ + z) : z;
        int32_t endZSteps = UnitConverter::DistanceToSteps(targetZ, u, mechanicalConfigZ);
        MoveZAxis(endZSteps, false);
    }
    #else
    (void)z;
    #endif

    WaitForMotionComplete();
    SendResponseLine("ok");
}

void ExecuteG02(double x, double y, double z, double i, double j, double f) {
    // Check if motors are initialized
    if (!motorsInitialized) {
        SendError(9);  // Homing cycle not started (or motors not initialized)
        return;
    }
    
    // Set feed rate if specified
    if (f >= 0.0) {
        if (unitMode == UNIT_MODE_INCHES) {
            motionController.FeedRateInchesPerMin(f);
            currentFeedRate = f;
        } else {
            motionController.FeedRateMMPerMin(f);
            currentFeedRate = f;
        }
        feedRateSet = true;
    } else if (feedRateSet) {
        if (unitMode == UNIT_MODE_INCHES) {
            motionController.FeedRateInchesPerMin(currentFeedRate);
        } else {
            motionController.FeedRateMMPerMin(currentFeedRate);
        }
    }
    
    if (!motorsEnabled) {
        SendError(9);
        return;
    }
    
    // Calculate arc center and parameters
    double startX, startY;
    if (unitMode == UNIT_MODE_INCHES) {
        startX = motionController.CurrentXInches();
        startY = motionController.CurrentYInches();
    } else {
        startX = motionController.CurrentXMM();
        startY = motionController.CurrentYMM();
    }
    
    double endX = x;
    double endY = y;
    if (coordinateMode == COORD_INCREMENTAL) {
        endX = startX + x;
        endY = startY + y;
    }
    
    double centerX = startX + i;
    double centerY = startY + j;
    double radius = sqrt(i * i + j * j);
    
    if (radius <= 0) {
        SendError(2);
        return;
    }
    
    double endAngle = atan2(endY - centerY, endX - centerX);
    if (endAngle < 0) endAngle += 2 * M_PI;
    
    // Convert to steps
    int32_t centerXSteps, centerYSteps, radiusSteps;
    if (unitMode == UNIT_MODE_INCHES) {
        centerXSteps = UnitConverter::DistanceToSteps(centerX, ClearCore::UNIT_INCHES, mechanicalConfigX);
        centerYSteps = UnitConverter::DistanceToSteps(centerY, ClearCore::UNIT_INCHES, mechanicalConfigY);
        radiusSteps = UnitConverter::DistanceToSteps(radius, ClearCore::UNIT_INCHES, mechanicalConfigX);
    } else {
        centerXSteps = UnitConverter::DistanceToSteps(centerX, ClearCore::UNIT_MM, mechanicalConfigX);
        centerYSteps = UnitConverter::DistanceToSteps(centerY, ClearCore::UNIT_MM, mechanicalConfigY);
        radiusSteps = UnitConverter::DistanceToSteps(radius, ClearCore::UNIT_MM, mechanicalConfigX);
    }
    
    bool success = false;
    uint32_t startWait = Milliseconds();
    while (!success) {
        success = motionController.QueueArc(centerXSteps, centerYSteps, radiusSteps, endAngle, true);
        if (success) {
            break;
        }
        if (Milliseconds() - startWait > 5000) {
            break;
        }
        Delay_ms(1);
    }
    if (!success) {
        SendError(24);
        return;
    }
    
    WaitForMotionComplete();
    SendResponseLine("ok");
}

void ExecuteG03(double x, double y, double z, double i, double j, double f) {
    // Check if motors are initialized
    if (!motorsInitialized) {
        SendError(9);  // Homing cycle not started (or motors not initialized)
        return;
    }
    
    // Set feed rate if specified
    if (f >= 0.0) {
        if (unitMode == UNIT_MODE_INCHES) {
            motionController.FeedRateInchesPerMin(f);
            currentFeedRate = f;
        } else {
            motionController.FeedRateMMPerMin(f);
            currentFeedRate = f;
        }
        feedRateSet = true;
    } else if (feedRateSet) {
        if (unitMode == UNIT_MODE_INCHES) {
            motionController.FeedRateInchesPerMin(currentFeedRate);
        } else {
            motionController.FeedRateMMPerMin(currentFeedRate);
        }
    }
    
    if (!motorsEnabled) {
        SendError(9);
        return;
    }
    
    // Calculate arc center and parameters
    double startX, startY;
    if (unitMode == UNIT_MODE_INCHES) {
        startX = motionController.CurrentXInches();
        startY = motionController.CurrentYInches();
    } else {
        startX = motionController.CurrentXMM();
        startY = motionController.CurrentYMM();
    }
    
    double endX = x;
    double endY = y;
    if (coordinateMode == COORD_INCREMENTAL) {
        endX = startX + x;
        endY = startY + y;
    }
    
    double centerX = startX + i;
    double centerY = startY + j;
    double radius = sqrt(i * i + j * j);
    
    if (radius <= 0) {
        SendError(2);
        return;
    }
    
    double endAngle = atan2(endY - centerY, endX - centerX);
    if (endAngle < 0) endAngle += 2 * M_PI;
    
    // Convert to steps
    int32_t centerXSteps, centerYSteps, radiusSteps;
    if (unitMode == UNIT_MODE_INCHES) {
        centerXSteps = UnitConverter::DistanceToSteps(centerX, ClearCore::UNIT_INCHES, mechanicalConfigX);
        centerYSteps = UnitConverter::DistanceToSteps(centerY, ClearCore::UNIT_INCHES, mechanicalConfigY);
        radiusSteps = UnitConverter::DistanceToSteps(radius, ClearCore::UNIT_INCHES, mechanicalConfigX);
    } else {
        centerXSteps = UnitConverter::DistanceToSteps(centerX, ClearCore::UNIT_MM, mechanicalConfigX);
        centerYSteps = UnitConverter::DistanceToSteps(centerY, ClearCore::UNIT_MM, mechanicalConfigY);
        radiusSteps = UnitConverter::DistanceToSteps(radius, ClearCore::UNIT_MM, mechanicalConfigX);
    }
    
    bool success = false;
    uint32_t startWait = Milliseconds();
    while (!success) {
        success = motionController.QueueArc(centerXSteps, centerYSteps, radiusSteps, endAngle, false);
        if (success) {
            break;
        }
        if (Milliseconds() - startWait > 5000) {
            break;
        }
        Delay_ms(1);
    }
    if (!success) {
        SendError(24);
        return;
    }
    
    #if USE_Z_AXIS
    // Handle Z-axis movement independently (linear interpolation during arc)
    if (z != 0.0 || coordinateMode == COORD_ABSOLUTE) {
        double targetZ = z;
        if (coordinateMode == COORD_INCREMENTAL) {
            targetZ = GetCurrentZPosition() + z;
        }
        
        int32_t endZSteps;
        if (unitMode == UNIT_MODE_INCHES) {
            endZSteps = UnitConverter::DistanceToSteps(targetZ, ClearCore::UNIT_INCHES, mechanicalConfigZ);
        } else {
            endZSteps = UnitConverter::DistanceToSteps(targetZ, ClearCore::UNIT_MM, mechanicalConfigZ);
        }
        
        MoveZAxis(endZSteps, false);
    }
    #endif
    
    SendResponseLine("ok");
}

void ExecuteG28() {
    // Check if motors are initialized
    if (!motorsInitialized) {
        SendError(9);  // Homing cycle not started (or motors not initialized)
        return;
    }
    
    // Home - move to home switches, then set origin
    if (!motorsEnabled) {
        SendError(9);
        return;
    }
    
    grblState = STATE_HOME;
    
    // Homing sequence: move in negative direction until home switches are triggered
    // This is a simplified homing - in a full implementation, you'd move away first,
    // then approach slowly
    
    bool xHomed = false;
    bool yHomed = false;
    bool zHomed = false;
    
    // Use slow feed rate for homing
    double homeFeedRate = unitMode == UNIT_MODE_INCHES ? 10.0 : 254.0;  // 10 in/min or 254 mm/min
    motionController.FeedRateMMPerMin(homeFeedRate);
    
    #ifdef HOME_X_PIN
    // Move X axis negative until home switch is triggered
    while (!xHomed) {
        // Move small increment in negative X
        int32_t currentX = motionController.CurrentX();
        int32_t targetX = currentX - UnitConverter::DistanceToSteps(1.0, unitMode == UNIT_MODE_INCHES ? ClearCore::UNIT_INCHES : ClearCore::UNIT_MM, mechanicalConfigX);
        motionController.QueueLinear(targetX, motionController.CurrentY());
        
        // Wait for move to complete or home switch to trigger
        while (motionController.IsActive()) {
            Delay_ms(10);
            int16_t state = HOME_X_PIN.State();
            if (HOME_SWITCH_ACTIVE_LOW ? (state == 0) : (state != 0)) {
                motionController.Stop();
                xHomed = true;
                break;
            }
        }
        
        if (xHomed) break;
    }
    #else
    xHomed = true;  // No home switch configured
    #endif
    
    #ifdef HOME_Y_PIN
    // Move Y axis negative until home switch is triggered
    while (!yHomed) {
        int32_t currentY = motionController.CurrentY();
        int32_t targetY = currentY - UnitConverter::DistanceToSteps(1.0, unitMode == UNIT_MODE_INCHES ? ClearCore::UNIT_INCHES : ClearCore::UNIT_MM, mechanicalConfigY);
        motionController.QueueLinear(motionController.CurrentX(), targetY);
        
        while (motionController.IsActive()) {
            Delay_ms(10);
            int16_t state = HOME_Y_PIN.State();
            if (HOME_SWITCH_ACTIVE_LOW ? (state == 0) : (state != 0)) {
                motionController.Stop();
                yHomed = true;
                break;
            }
        }
        
        if (yHomed) break;
    }
    #else
    yHomed = true;
    #endif
    
    #if defined(HOME_Z_PIN) && USE_Z_AXIS
    // Move Z axis negative until home switch is triggered
    while (!zHomed) {
        int32_t currentZ = currentZSteps;
        int32_t targetZ = currentZ - UnitConverter::DistanceToSteps(1.0, unitMode == UNIT_MODE_INCHES ? ClearCore::UNIT_INCHES : ClearCore::UNIT_MM, mechanicalConfigZ);
        motorZ.Move(targetZ, MotorDriver::MOVE_TARGET_ABSOLUTE);
        
        while (motorZ.StatusReg().bit.StepsActive) {
            Delay_ms(10);
            int16_t state = HOME_Z_PIN.State();
            if (HOME_SWITCH_ACTIVE_LOW ? (state == 0) : (state != 0)) {
                motorZ.MoveStopAbrupt();
                zHomed = true;
                break;
            }
        }
        
        if (zHomed) break;
        currentZSteps = motorZ.PositionRefCommanded();
    }
    #else
    zHomed = true;
    #endif
    
    (void)xHomed;
    (void)yHomed;
    (void)zHomed;
    
    // Set position to origin after homing
    motionController.SetPosition(0, 0);
    currentZSteps = 0;
    homed = true;
    grblState = STATE_IDLE;
    SendResponseLine("ok");
}

void ExecuteG38(double x, double y, double z, double f) {
    // Check if motors are initialized
    if (!motorsInitialized) {
        SendError(9);  // Homing cycle not started (or motors not initialized)
        return;
    }
    
    // Probe - move until probe triggers
    if (!motorsEnabled) {
        SendError(9);
        return;
    }
    
    #ifndef PROBE_PIN
    SendError(9);  // Probe not configured
    return;
    #endif
    
    // Set feed rate if specified
    if (f >= 0.0) {
        if (unitMode == UNIT_MODE_INCHES) {
            motionController.FeedRateInchesPerMin(f);
        } else {
            motionController.FeedRateMMPerMin(f);
        }
    }
    
    // Calculate target position
    double targetX = x;
    double targetY = y;
    
    if (coordinateMode == COORD_INCREMENTAL) {
        if (unitMode == UNIT_MODE_INCHES) {
            targetX = motionController.CurrentXInches() + x;
            targetY = motionController.CurrentYInches() + y;
        } else {
            targetX = motionController.CurrentXMM() + x;
            targetY = motionController.CurrentYMM() + y;
        }
    }
    
    // Convert to steps
    int32_t endXSteps, endYSteps;
    if (unitMode == UNIT_MODE_INCHES) {
        endXSteps = UnitConverter::DistanceToSteps(targetX, ClearCore::UNIT_INCHES, mechanicalConfigX);
        endYSteps = UnitConverter::DistanceToSteps(targetY, ClearCore::UNIT_INCHES, mechanicalConfigY);
    } else {
        endXSteps = UnitConverter::DistanceToSteps(targetX, ClearCore::UNIT_MM, mechanicalConfigX);
        endYSteps = UnitConverter::DistanceToSteps(targetY, ClearCore::UNIT_MM, mechanicalConfigY);
    }
    
    // Move toward target, checking probe
    grblState = STATE_RUN;
    
    // Simplified probe: move in small increments checking probe
    int32_t currentX = motionController.CurrentX();
    int32_t currentY = motionController.CurrentY();
    int32_t dx = endXSteps - currentX;
    int32_t dy = endYSteps - currentY;
    
    // Move incrementally, checking probe
    while ((currentX != endXSteps || currentY != endYSteps) && !CheckProbe()) {
        // Calculate next step (simplified - move toward target)
        int32_t stepX = 0, stepY = 0;
        if (abs(dx) > abs(dy)) {
            stepX = (dx > 0) ? 1 : -1;
        } else {
            stepY = (dy > 0) ? 1 : -1;
        }
        
        currentX += stepX;
        currentY += stepY;
        dx = endXSteps - currentX;
        dy = endYSteps - currentY;
        
        motionController.QueueLinear(currentX, currentY);
        
        // Wait briefly, checking probe
        uint32_t startTime = Milliseconds();
        while (motionController.IsActive() && (Milliseconds() - startTime < 100)) {
            Delay_ms(1);
            if (CheckProbe()) {
                motionController.Stop();
                // Report probe position
                double probeX, probeY;
                if (unitMode == UNIT_MODE_INCHES) {
                    probeX = motionController.CurrentXInches();
                    probeY = motionController.CurrentYInches();
                } else {
                    probeX = motionController.CurrentXMM();
                    probeY = motionController.CurrentYMM();
                }
                char probeMsg[100];
                sprintf(probeMsg, "[PRB:%.3f,%.3f,0.000:1]", probeX, probeY);
                SendResponseLine(probeMsg);
                SendResponseLine("ok");
                grblState = STATE_IDLE;
                return;
            }
        }
    }
    
    // Probe didn't trigger - error
    SendError(9);
    grblState = STATE_IDLE;
}

void SendStatusReport() {
    // GRBL status format: <Idle|Run|Hold|Alarm>,MPos:X.xxx,Y.yyy,Z.zzz,FS:F,SS:0>
    char status[200];
    const char* stateStr;
    
    switch (grblState) {
        case STATE_IDLE:
            stateStr = "Idle";
            break;
        case STATE_RUN:
            stateStr = "Run";
            break;
        case STATE_HOLD:
            stateStr = "Hold";
            break;
        case STATE_ALARM:
            stateStr = "Alarm";
            break;
        case STATE_HOME:
            stateStr = "Home";
            break;
        default:
            stateStr = "Idle";
            break;
    }
    
    // Get current position
    double xPos, yPos, zPos;
    if (motorsInitialized) {
        if (unitMode == UNIT_MODE_INCHES) {
            xPos = motionController.CurrentXInches();
            yPos = motionController.CurrentYInches();
            zPos = GetCurrentZPosition();
        } else {
            xPos = motionController.CurrentXMM();
            yPos = motionController.CurrentYMM();
            zPos = GetCurrentZPosition();
        }
    } else {
        // Motors not initialized - report zero position
        xPos = 0.0;
        yPos = 0.0;
        zPos = 0.0;
    }
    
    // Format status report (GRBL format: FS is feed rate, SS is spindle speed)
    // Include probe state if probe is configured
    #ifdef PROBE_PIN
    if (CheckProbe()) {
        sprintf(status, "<%s,MPos:%.3f,%.3f,%.3f,FS:%.0f,0|Pn:XYZ>",
                stateStr, xPos, yPos, zPos, currentFeedRate);
    } else {
        sprintf(status, "<%s,MPos:%.3f,%.3f,%.3f,FS:%.0f,0>",
                stateStr, xPos, yPos, zPos, currentFeedRate);
    }
    #else
    sprintf(status, "<%s,MPos:%.3f,%.3f,%.3f,FS:%.0f,0>",
            stateStr, xPos, yPos, zPos, currentFeedRate);
    #endif
    
    SendResponseLine(status);
}

void SendError(uint8_t errorCode) {
    char errorMsg[20];
    sprintf(errorMsg, "error:%d", errorCode);
    SendResponseLine(errorMsg);
}

void SendResponse(const char* message) {
    SerialPort.Send(message);
}

void SendResponseLine(const char* message) {
    SendResponse(message);
    SendResponse("\n");
}

// ========== Arc R-to-IJ (GRBL-style) ==========

bool ArcRToIJ(double startX, double startY, double endX, double endY, double r,
              bool clockwise, double& outI, double& outJ) {
    double x = endX - startX;
    double y = endY - startY;
    if (x == 0.0 && y == 0.0) {
        return false;  // target same as current, invalid
    }
    double d = hypot(x, y);
    double h2 = 4.0 * r * r - x * x - y * y;
    // Allow tolerance for semicircle (e.g. R=0.707 for (0,0)->(1,1)) vs fp roundoff
    const double eps = 1e-3;
    if (h2 < -eps) {
        return false;  // arc radius error
    }
    if (h2 < 0.0) {
        h2 = 0.0;
    }
    double h_x2_div_d = -sqrt(h2) / d;
    if (!clockwise) {
        h_x2_div_d = -h_x2_div_d;
    }
    if (r < 0.0) {
        h_x2_div_d = -h_x2_div_d;
    }
    outI = 0.5 * (x - y * h_x2_div_d);
    outJ = 0.5 * (y + x * h_x2_div_d);
    return true;
}

// ========== Z-Axis Helper Functions ==========

void MoveZAxis(int32_t targetZSteps, bool rapid) {
    #if !USE_Z_AXIS
    (void)targetZSteps;
    (void)rapid;
    return;
    #else
    // Check if motors are initialized
    if (!motorsInitialized) {
        return;  // Cannot move without motors
    }
    
    // Move Z axis independently (not coordinated with X/Y)
    if (targetZSteps == currentZSteps) {
        return;  // Already at target
    }
    
    if (rapid) {
        // Rapid move - use high velocity
        motorZ.VelMax(DEFAULT_VELOCITY_STEPS_PER_SEC * 2);
    } else {
        // Feed rate move - convert feed rate to velocity
        // Feed rate is in distance per minute, velocity is steps per second
        double feedRateStepsPerSec = currentFeedRate;
        if (unitMode == UNIT_MODE_INCHES) {
            // Convert inches/min to steps/sec
            feedRateStepsPerSec = (currentFeedRate / 60.0) * (mechanicalConfigZ.stepsPerRevolution / mechanicalConfigZ.pitch);
        } else {
            // Convert mm/min to steps/sec
            feedRateStepsPerSec = (currentFeedRate / 60.0) * (mechanicalConfigZ.stepsPerRevolution / mechanicalConfigZ.pitch);
        }
        motorZ.VelMax((int32_t)feedRateStepsPerSec);
    }
    
    motorZ.Move(targetZSteps, MotorDriver::MOVE_TARGET_ABSOLUTE);
    
    // Wait for move to complete
    while (motorZ.StatusReg().bit.StepsActive) {
        Delay_ms(1);
    }
    
    currentZSteps = motorZ.PositionRefCommanded();
    #endif
}

double GetCurrentZPosition() {
    #if !USE_Z_AXIS
    return 0.0;
    #else
    // Get current Z position in current units
    if (unitMode == UNIT_MODE_INCHES) {
        return UnitConverter::StepsToDistance(currentZSteps, ClearCore::UNIT_INCHES, mechanicalConfigZ);
    } else {
        return UnitConverter::StepsToDistance(currentZSteps, ClearCore::UNIT_MM, mechanicalConfigZ);
    }
    #endif
}

// ========== Input Checking Functions ==========

void InitializeInputs() {
    // Initialize home switches
    #ifdef HOME_X_PIN
    HOME_X_PIN.Mode(Connector::INPUT_DIGITAL);
    #endif
    
    #ifdef HOME_Y_PIN
    HOME_Y_PIN.Mode(Connector::INPUT_DIGITAL);
    #endif
    
    #if defined(HOME_Z_PIN) && USE_Z_AXIS
    HOME_Z_PIN.Mode(Connector::INPUT_DIGITAL);
    #endif
    
    // Initialize probe input
    #ifdef PROBE_PIN
    PROBE_PIN.Mode(Connector::INPUT_DIGITAL);
    #endif
    
    // Initialize limit switches
    #ifdef LIMIT_X_MIN_PIN
    LIMIT_X_MIN_PIN.Mode(Connector::INPUT_DIGITAL);
    #endif
    
    #ifdef LIMIT_X_MAX_PIN
    LIMIT_X_MAX_PIN.Mode(Connector::INPUT_DIGITAL);
    #endif
    
    #ifdef LIMIT_Y_MIN_PIN
    LIMIT_Y_MIN_PIN.Mode(Connector::INPUT_DIGITAL);
    #endif
    
    #ifdef LIMIT_Y_MAX_PIN
    LIMIT_Y_MAX_PIN.Mode(Connector::INPUT_DIGITAL);
    #endif
}

bool CheckHomeSwitches() {
    // Check if any home switch is triggered
    #ifdef HOME_X_PIN
    if (HOME_SWITCH_ACTIVE_LOW ? !HOME_X_PIN.State() : HOME_X_PIN.State()) {
        return true;
    }
    #endif
    
    #ifdef HOME_Y_PIN
    if (HOME_SWITCH_ACTIVE_LOW ? !HOME_Y_PIN.State() : HOME_Y_PIN.State()) {
        return true;
    }
    #endif
    
    #if defined(HOME_Z_PIN) && USE_Z_AXIS
    if (HOME_SWITCH_ACTIVE_LOW ? !HOME_Z_PIN.State() : HOME_Z_PIN.State()) {
        return true;
    }
    #endif
    
    return false;
}

bool CheckLimitSwitches() {
    // Check if any limit switch is triggered
    #ifdef LIMIT_X_MIN_PIN
    if (LIMIT_SWITCH_ACTIVE_LOW ? !LIMIT_X_MIN_PIN.State() : LIMIT_X_MIN_PIN.State()) {
        return true;
    }
    #endif
    
    #ifdef LIMIT_X_MAX_PIN
    if (LIMIT_SWITCH_ACTIVE_LOW ? !LIMIT_X_MAX_PIN.State() : LIMIT_X_MAX_PIN.State()) {
        return true;
    }
    #endif
    
    #ifdef LIMIT_Y_MIN_PIN
    if (LIMIT_SWITCH_ACTIVE_LOW ? !LIMIT_Y_MIN_PIN.State() : LIMIT_Y_MIN_PIN.State()) {
        return true;
    }
    #endif
    
    #ifdef LIMIT_Y_MAX_PIN
    if (LIMIT_SWITCH_ACTIVE_LOW ? !LIMIT_Y_MAX_PIN.State() : LIMIT_Y_MAX_PIN.State()) {
        return true;
    }
    #endif
    
    return false;
}

bool CheckProbe() {
    // Check if probe is triggered
    #ifdef PROBE_PIN
    return PROBE_ACTIVE_LOW ? !PROBE_PIN.State() : PROBE_PIN.State();
    #else
    return false;
    #endif
}
