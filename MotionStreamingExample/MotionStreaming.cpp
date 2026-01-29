/*
 * Title: MotionStreaming
 *
 * Objective:
 *    This example demonstrates streaming motion commands to ClearCore over
 *    serial (USB CDC) or Ethernet (TCP server). Commands can be sent in real-time
 *    to execute coordinated motion moves, configure parameters, and query status.
 *
 * Description:
 *    This example sets up either a serial port (USB CDC) or Ethernet TCP server
 *    to receive motion commands. It supports:
 *    1. Coordinated linear moves (G01)
 *    2. Coordinated arc moves (G02/G03)
 *    3. Unit-based commands (inches, mm)
 *    4. Feed rate control
 *    5. Status queries
 *    6. Configuration commands
 *
 * Requirements:
 * 1. Two ClearPath motors must be connected, one to Connector M-0 the other to
 *    Connector M-1.
 * 2. The connected ClearPath motors must be configured through the MSP software
 *    for Step and Direction mode (In MSP select Mode>>Step and Direction).
 * 3. The ClearPath motors must be set to use the HLFB mode "ASG-Position
 *    w/Measured Torque" with a PWM carrier frequency of 482 Hz through the MSP
 *    software (select Advanced>>High Level Feedback [Mode]... then choose
 *    "ASG-Position w/Measured Torque" from the dropdown, make sure that 482 Hz
 *    is selected in the "PWM Carrier Frequency" dropdown, and hit the OK
 *    button).
 * 4. For Ethernet mode: Ethernet cable connected and network configured
 *
 * ** Note: Homing is optional, and not required in this operational mode or in
 *    this example. This example assumes motors start at position (0, 0).
 *
 * Command Protocol:
 *    Commands are sent as text lines, terminated with newline (\n).
 *    Supported commands:
 *    - G01 X<value> Y<value> [F<feedrate>]  : Linear move (feed rate optional)
 *    - G02 X<value> Y<value> I<value> J<value> [F<feedrate>]  : Arc CW (center offset)
 *    - G02 X<value> Y<value> R<value> [F<feedrate>]           : Arc CW (radius)
 *    - G03 X<value> Y<value> I<value> J<value> [F<feedrate>]  : Arc CCW (center offset)
 *    - G03 X<value> Y<value> R<value> [F<feedrate>]           : Arc CCW (radius)
 *      Use I,J or R, not both. Negative R = arc > 180 deg (GRBL-style).
 *    - G90  : Absolute coordinates
 *    - G91  : Incremental coordinates
 *    - G92  : Set coordinate system offset (set origin at current position)
 *    - G20  : Units in inches
 *    - G21  : Units in millimeters
 *    - G4 P<seconds> : Dwell
 *    - M200 : Emergency stop (immediate stop, clear queue)
 *    - M201 : Stop with deceleration (smooth stop, clear queue)
 *    - M202 : Enable motors
 *    - M203 : Disable motors (stops motion and disables)
 *    - M114 : Get current position
 *    - M115 : Get status
 *    - M500 [F<feedrate>] : Run 180° arc out/back test (defaults to 10 ipm)
 *    - M501 [F<feedrate>] : Run random move test sequence, auto-enable motors,
 *                            returns to origin (defaults to 10 ipm)
 * 
 * Notes:
 *    - Default units: inches (G20). Use G21 to switch to mm.
 *    - Motors start disabled on boot; use M202 to enable before motion.
 *
 * Links:
 * ** ClearCore Documentation: https://teknic-inc.github.io/ClearCore-library/
 * ** ClearCore Manual: https://www.teknic.com/files/downloads/clearcore_user_manual.pdf
 * ** ClearPath Manual (DC Power): https://www.teknic.com/files/downloads/clearpath_user_manual.pdf
 * ** ClearPath Manual (AC Power): https://www.teknic.com/files/downloads/ac_clearpath-mc-sd_manual.pdf
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
#include "EthernetTcpServer.h"
#include "SysTiming.h"
#include <math.h>
#include <string.h>
#include <stdio.h>

// Simple PRNG for random moves
static uint32_t randomSeed = 1;
static uint32_t randomNext() {
    randomSeed = randomSeed * 1103515245 + 12345;
    return (randomSeed >> 16) & 0x7FFF;
}
static void randomSeedInit() {
    // Seed from system time (milliseconds)
    randomSeed = Milliseconds();
    if (randomSeed == 0) randomSeed = 1;
}
static double randomDouble(double min, double max) {
    return min + (randomNext() % 10000) * (max - min) / 10000.0;
}

// ========== Configuration ==========

// Select communication mode: SERIAL_MODE or ETHERNET_MODE
#define SERIAL_MODE 1
#define ETHERNET_MODE 2
#define COMM_MODE SERIAL_MODE  // Change to ETHERNET_MODE for Ethernet

// Serial configuration (if using SERIAL_MODE)
#define SERIAL_BAUD_RATE 115200
#define SerialPort ConnectorUsb  // Options: ConnectorUsb, ConnectorCOM0, ConnectorCOM1

// Ethernet configuration (if using ETHERNET_MODE)
#define TCP_PORT 8888
#define MAX_PACKET_LENGTH 256
bool usingDhcp = true;  // Set to false for manual IP configuration
IpAddress manualIp = IpAddress(192, 168, 0, 109);  // Only used if usingDhcp = false

// Motor configuration
#define motorX ConnectorM0
#define motorY ConnectorM1

// Mechanical configuration
#define MOTOR_X_STEPS_PER_REV 800
#define MOTOR_X_PITCH_MM 5.0
#define MOTOR_Y_STEPS_PER_REV 800
#define MOTOR_Y_PITCH_MM 5.0

// Default motion parameters
#define DEFAULT_FEED_RATE_MM_PER_MIN 254.0  // 10 inches/min
#define DEFAULT_FEED_RATE_INCHES_PER_MIN 10.0
#define DEFAULT_VELOCITY_STEPS_PER_SEC 5000
#define DEFAULT_ACCELERATION_STEPS_PER_SEC2 85000

// ========== Global Variables ==========

// Global objects - constructors run before main()
CoordinatedMotionController motionController;
EthernetTcpServer* tcpServer = nullptr;
EthernetTcpClient tcpClient;  // Store by value, not pointer

// Store mechanical config for unit conversion (needed for queuing)
MotorMechanicalConfig mechanicalConfigX;
MotorMechanicalConfig mechanicalConfigY;

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
UnitMode unitMode = UNIT_MODE_INCHES;
double currentFeedRate = DEFAULT_FEED_RATE_INCHES_PER_MIN;
bool feedRateSet = false;

// Command buffer
char commandBuffer[MAX_PACKET_LENGTH];
uint16_t commandBufferIndex = 0;

// Motor initialization status
bool motorsInitialized = false;
static bool suppressAutoPosition = false;

// ========== Function Prototypes ==========

void InitializeMotors();
void InitializeCommunication();
void ProcessCommand(const char* command);
void ParseGCode(const char* line);
void ExecuteG01(double x, double y, double f = -1.0);
void ExecuteG02(double x, double y, double i, double j, double f = -1.0);
void ExecuteG03(double x, double y, double i, double j, double f = -1.0);
void SendResponse(const char* message);
void SendResponseLine(const char* message);
void PrintCurrentPosition();
bool ReadCommandSerial(char* buffer, uint16_t maxLen);
bool ReadCommandEthernet(char* buffer, uint16_t maxLen);

// GRBL-style R-to-IJ conversion for G2/G3 arc radius mode
static bool ArcRToIJ(double startX, double startY, double endX, double endY, double r,
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

// Wait helpers for test sequences
static uint32_t CalculateMoveTimeoutMs(double distance, double feedRate) {
    double moveTimeMs = 0.0;
    if (unitMode == UNIT_MODE_INCHES) {
        // Distance in inches, feed rate in ipm
        moveTimeMs = (distance / feedRate) * 60.0 * 1000.0;
    } else {
        // Distance in mm, feed rate in mm/min (feedRate * 25.4)
        moveTimeMs = (distance / (feedRate * 25.4)) * 60.0 * 1000.0;
    }
    // Add 3 second safety margin and minimum 2 second timeout
    uint32_t timeoutMs = (uint32_t)(moveTimeMs + 3000.0);
    if (timeoutMs < 2000) {
        timeoutMs = 2000;
    }
    return timeoutMs;
}

static void WaitForMotionComplete(uint32_t timeoutMs) {
    uint32_t startTime = Milliseconds();
    while (motionController.MotionQueueCount() > 0 || motionController.IsActive()) {
        Delay_ms(50);
        uint32_t currentTime = Milliseconds();
        if (currentTime - startTime > timeoutMs) {
            break;
        }
    }
}

// ========== Main Function ==========

int main() {
    // SysManager is initialized in Reset_Handler before main() is called
    // Just add a simple delay to allow everything to stabilize
    Delay_ms(500);
    
    // Initialize motors (may fail if motors not attached - that's OK)
    InitializeMotors();
    
    // Initialize communication (safe even if USB not connected)
    InitializeCommunication();
    
    // Small delay before sending startup message
    Delay_ms(200);
    
    // Only send startup messages if communication is ready
    #if COMM_MODE == SERIAL_MODE
    if (SerialPort) {
    #elif COMM_MODE == ETHERNET_MODE
    if (ConnectorUsb) {
    #endif
        SendResponseLine("Motion Streaming Example Ready");
        if (!motorsInitialized) {
            SendResponseLine("Note: Motors not initialized - continuing without motors");
        }
    #if COMM_MODE == SERIAL_MODE || COMM_MODE == ETHERNET_MODE
    }
    #endif
    
    // Main loop
    while (true) {
        bool commandReceived = false;
        
        #if COMM_MODE == SERIAL_MODE
        commandReceived = ReadCommandSerial(commandBuffer, MAX_PACKET_LENGTH);
        #elif COMM_MODE == ETHERNET_MODE
        commandReceived = ReadCommandEthernet(commandBuffer, MAX_PACKET_LENGTH);
        // Refresh Ethernet stack
        EthernetMgr.Refresh();
        #endif
        
        if (commandReceived) {
            ProcessCommand(commandBuffer);
            // Clear buffer
            memset(commandBuffer, 0, MAX_PACKET_LENGTH);
            commandBufferIndex = 0;
        }
        
        // Small delay to prevent CPU spinning
        Delay_ms(1);
    }
    
    return 0;
}

// ========== Initialization Functions ==========

void InitializeMotors() {
    motorsInitialized = false;
    
    // Simple delay to ensure system is stable before accessing hardware
    Delay_ms(50);
    
    // Set motors to Step and Direction mode
    // This is safe even if motors aren't attached - it just configures the connectors
    // Don't check SysManager.Ready() as it should already be ready from Reset_Handler
    MotorMgr.MotorModeSet(MotorManager::MOTOR_ALL, Connector::CPM_MODE_STEP_AND_DIR);
    
    // Small delay to allow mode change to propagate
    Delay_ms(50);
    
    // Motors start DISABLED for safety - user must explicitly enable with M202
    // This prevents unexpected motion on boot
    motorX.EnableRequest(false);
    motorY.EnableRequest(false);
    
    // Short delay to allow enable signal to propagate
    Delay_ms(50);
    
    // Initialize coordinated motion controller
    // This may fail if motors are not present or not properly configured
    // Attempt initialization - this can fail safely if motors aren't attached
    if (!motionController.Initialize(&motorX, &motorY)) {
        // Motors not available - continue without motors
        // Communication and other features will still work
        // Don't try to configure motion controller if initialization failed
        return;
    }
    
    // Configure mechanical parameters for unit conversion
    motionController.SetMechanicalParamsX(MOTOR_X_STEPS_PER_REV, MOTOR_X_PITCH_MM, ClearCore::UNIT_MM);
    motionController.SetMechanicalParamsY(MOTOR_Y_STEPS_PER_REV, MOTOR_Y_PITCH_MM, ClearCore::UNIT_MM);
    
    // Store config locally for unit conversion in command processing
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
    motionController.ArcAccelMax(DEFAULT_ACCELERATION_STEPS_PER_SEC2);
    
    // Set initial position
    motionController.SetPosition(0, 0);
    
    // Mark motors as successfully initialized
    motorsInitialized = true;
}

void InitializeCommunication() {
    // Simple delay to ensure system is stable
    Delay_ms(50);
    
    #if COMM_MODE == SERIAL_MODE
    // Initialize serial port
    SerialPort.Mode(Connector::USB_CDC);
    SerialPort.Speed(SERIAL_BAUD_RATE);
    SerialPort.PortOpen();
    
    // Simple delay - USB CDC requires host connection
    // Don't wait indefinitely - just delay and continue
    // Serial port will become available when USB connects
    Delay_ms(100);
    
    #elif COMM_MODE == ETHERNET_MODE
    // Initialize serial port for status messages (USB CDC)
    ConnectorUsb.Mode(Connector::USB_CDC);
    ConnectorUsb.Speed(9600);
    ConnectorUsb.PortOpen();
    
    // Simple delay - USB CDC requires host connection
    // Don't wait indefinitely - just delay and continue
    Delay_ms(100);
    
    // Wait for Ethernet link
    while (!EthernetMgr.PhyLinkActive()) {
        if (ConnectorUsb) {  // Only send if USB is ready
            ConnectorUsb.SendLine("Waiting for Ethernet link...");
        }
        Delay_ms(1000);
    }
    
    // Configure Ethernet
    EthernetMgr.Setup();
    
    if (usingDhcp) {
        bool dhcpSuccess = EthernetMgr.DhcpBegin();
        if (ConnectorUsb) {  // Only send if USB is ready
            if (dhcpSuccess) {
                ConnectorUsb.Send("DHCP IP: ");
                ConnectorUsb.SendLine(EthernetMgr.LocalIp().StringValue());
            } else {
                ConnectorUsb.SendLine("DHCP failed! Continuing with manual IP...");
                EthernetMgr.LocalIp(manualIp);
                ConnectorUsb.Send("Manual IP: ");
                ConnectorUsb.SendLine(EthernetMgr.LocalIp().StringValue());
            }
        } else {
            // USB not ready, but still configure IP
            if (!dhcpSuccess) {
                EthernetMgr.LocalIp(manualIp);
            }
        }
    } else {
        EthernetMgr.LocalIp(manualIp);
        if (ConnectorUsb) {  // Only send if USB is ready
            ConnectorUsb.Send("Manual IP: ");
            ConnectorUsb.SendLine(EthernetMgr.LocalIp().StringValue());
        }
    }
    
    // Create TCP server
    tcpServer = new EthernetTcpServer(TCP_PORT);
    if (!tcpServer) {
        if (ConnectorUsb) {  // Only send if USB is ready
            ConnectorUsb.SendLine("Failed to allocate TCP server!");
        }
    } else if (!tcpServer->Begin()) {
        if (ConnectorUsb) {  // Only send if USB is ready
            ConnectorUsb.SendLine("Failed to start TCP server! Will retry in main loop.");
        }
    }
    
    if (ConnectorUsb) {  // Only send if USB is ready
        ConnectorUsb.Send("TCP server listening on port ");
        char portStr[10];
        sprintf(portStr, "%d", TCP_PORT);
        ConnectorUsb.SendLine(portStr);
    }
    #endif
}

// ========== Command Processing Functions ==========

bool ReadCommandSerial(char* buffer, uint16_t maxLen) {
    int16_t ch = SerialPort.CharGet();
    if (ch == -1) {
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
    }
    
    return false;
}

bool ReadCommandEthernet(char* buffer, uint16_t maxLen) {
    if (!tcpServer) {
        return false;
    }
    
    // Check for new client connection
    if (!tcpClient.Connected() && tcpClient.BytesAvailable() == 0) {
        if (tcpServer) {  // Double-check tcpServer is valid
            tcpClient = tcpServer->Available();
            if (tcpClient.Connected() || tcpClient.BytesAvailable() > 0) {
                if (ConnectorUsb) {  // Only send if USB is ready
                    ConnectorUsb.SendLine("Client connected");
                }
                commandBufferIndex = 0;
                memset(buffer, 0, maxLen);
            }
        }
        return false;
    }
    
    // Read data from client
    int16_t ch = tcpClient.Read();
    if (ch == -1) {
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
    }
    
    return false;
}

void ProcessCommand(const char* command) {
    // Skip empty commands
    if (strlen(command) == 0) {
        return;
    }
    
    // Convert to uppercase for parsing
    char upperCommand[MAX_PACKET_LENGTH];
    strncpy(upperCommand, command, MAX_PACKET_LENGTH - 1);
    upperCommand[MAX_PACKET_LENGTH - 1] = '\0';
    
    for (uint16_t i = 0; i < strlen(upperCommand); i++) {
        if (upperCommand[i] >= 'a' && upperCommand[i] <= 'z') {
            upperCommand[i] = upperCommand[i] - 'a' + 'A';
        }
    }
    
    // Parse G-code command
    ParseGCode(upperCommand);
}

void ParseGCode(const char* line) {
    // Parse command line (e.g., "G01 X10.5 Y20.3 F100")
    double x = 0, y = 0, i = 0, j = 0, r = 0, f = -1.0;
    bool hasX = false, hasY = false, hasI = false, hasJ = false, hasR = false;
    
    // Find command code (G## or M##)
    if (line[0] == 'G' || line[0] == 'M') {
        int code = 0;
        sscanf(line, "%*[GM]%d", &code);
        
        // Parse parameters
        const char* ptr = line;
        while (*ptr) {
            if (*ptr == 'X' || *ptr == 'x') {
                sscanf(ptr, "X%lf", &x);
                hasX = true;
            } else if (*ptr == 'Y' || *ptr == 'y') {
                sscanf(ptr, "Y%lf", &y);
                hasY = true;
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
            }
            ptr++;
        }
        
        // Execute command
        if (line[0] == 'G') {
            switch (code) {
                case 0:  // G00 - Rapid move (not implemented, use G01)
                    SendResponseLine("G00 not implemented, use G01");
                    break;
                    
                case 1:  // G01 - Linear interpolation
                    if (hasX || hasY) {
                        ExecuteG01(x, y, f);
                    } else {
                        SendResponseLine("G01 requires X and/or Y");
                    }
                    break;
                    
                case 2:  // G02 - Circular interpolation CW
                    if (hasX && hasY) {
                        if ((hasI || hasJ) && hasR) {
                            SendResponseLine("G02: use I,J or R, not both");
                        } else if (hasR) {
                            double startX = (unitMode == UNIT_MODE_INCHES)
                                ? motionController.CurrentXInches() : motionController.CurrentXMM();
                            double startY = (unitMode == UNIT_MODE_INCHES)
                                ? motionController.CurrentYInches() : motionController.CurrentYMM();
                            double endX = (coordinateMode == COORD_ABSOLUTE) ? x : startX + x;
                            double endY = (coordinateMode == COORD_ABSOLUTE) ? y : startY + y;
                            double iVal, jVal;
                            if (!ArcRToIJ(startX, startY, endX, endY, r, true, iVal, jVal)) {
                                SendResponseLine("G02: arc radius error or target same as current");
                            } else {
                                ExecuteG02(x, y, iVal, jVal, f);
                            }
                        } else if (hasI && hasJ) {
                            ExecuteG02(x, y, i, j, f);
                        } else {
                            SendResponseLine("G02 requires X, Y, and either I,J or R");
                        }
                    } else {
                        SendResponseLine("G02 requires X, Y, and either I,J or R");
                    }
                    break;
                    
                case 3:  // G03 - Circular interpolation CCW
                    if (hasX && hasY) {
                        if ((hasI || hasJ) && hasR) {
                            SendResponseLine("G03: use I,J or R, not both");
                        } else if (hasR) {
                            double startX = (unitMode == UNIT_MODE_INCHES)
                                ? motionController.CurrentXInches() : motionController.CurrentXMM();
                            double startY = (unitMode == UNIT_MODE_INCHES)
                                ? motionController.CurrentYInches() : motionController.CurrentYMM();
                            double endX = (coordinateMode == COORD_ABSOLUTE) ? x : startX + x;
                            double endY = (coordinateMode == COORD_ABSOLUTE) ? y : startY + y;
                            double iVal, jVal;
                            if (!ArcRToIJ(startX, startY, endX, endY, r, false, iVal, jVal)) {
                                SendResponseLine("G03: arc radius error or target same as current");
                            } else {
                                ExecuteG03(x, y, iVal, jVal, f);
                            }
                        } else if (hasI && hasJ) {
                            ExecuteG03(x, y, i, j, f);
                        } else {
                            SendResponseLine("G03 requires X, Y, and either I,J or R");
                        }
                    } else {
                        SendResponseLine("G03 requires X, Y, and either I,J or R");
                    }
                    break;
                    
                case 20:  // G20 - Units in inches
                    unitMode = UNIT_MODE_INCHES;
                    SendResponseLine("Units: inches");
                    break;
                    
                case 21:  // G21 - Units in millimeters
                    unitMode = UNIT_MODE_MM;
                    SendResponseLine("Units: millimeters");
                    break;
                    
                case 90:  // G90 - Absolute coordinates
                    coordinateMode = COORD_ABSOLUTE;
                    SendResponseLine("Coordinates: absolute");
                    break;
                    
                case 91:  // G91 - Incremental coordinates
                    coordinateMode = COORD_INCREMENTAL;
                    SendResponseLine("Coordinates: incremental");
                    break;
                    
                case 92:  // G92 - Set coordinate system offset (set origin)
                    // G92 X<value> Y<value> sets current position to specified values
                    // This effectively shifts the coordinate system origin
                    if (!motorsInitialized) {
                        SendResponseLine("error: Motors not initialized");
                        break;
                    }
                    if (hasX || hasY) {
                        // Convert specified coordinates to steps
                        int32_t newXSteps, newYSteps;
                        if (unitMode == UNIT_MODE_INCHES) {
                            newXSteps = hasX ? UnitConverter::DistanceToSteps(x, ClearCore::UNIT_INCHES, mechanicalConfigX) : motionController.CurrentX();
                            newYSteps = hasY ? UnitConverter::DistanceToSteps(y, ClearCore::UNIT_INCHES, mechanicalConfigY) : motionController.CurrentY();
                        } else {
                            newXSteps = hasX ? UnitConverter::DistanceToSteps(x, ClearCore::UNIT_MM, mechanicalConfigX) : motionController.CurrentX();
                            newYSteps = hasY ? UnitConverter::DistanceToSteps(y, ClearCore::UNIT_MM, mechanicalConfigY) : motionController.CurrentY();
                        }
                        
                        // Set new position (this shifts the coordinate system)
                        motionController.SetPosition(newXSteps, newYSteps);
                        
                        SendResponseLine("Coordinate system offset set");
                    } else {
                        // G92 without parameters - reset offset (set current position to 0,0)
                        if (motorsInitialized) {
                            motionController.SetPosition(0, 0);
                            SendResponseLine("Coordinate system offset reset to origin");
                        } else {
                            SendResponseLine("error: Motors not initialized");
                        }
                    }
                    break;
                    
                default:
                    SendResponse("Unknown G-code: G");
                    char codeStr[10];
                    sprintf(codeStr, "%d", code);
                    SendResponseLine(codeStr);
                    break;
            }
        } else if (line[0] == 'M') {
            switch (code) {
                case 200:  // M200 - Emergency stop (immediate stop, clear queue)
                    if (motorsInitialized) {
                        motionController.Stop();
                        SendResponseLine("EMERGENCY STOP - queue cleared");
                    } else {
                        SendResponseLine("error: Motors not initialized");
                    }
                    break;
                    
                case 201:  // M201 - Stop with deceleration (smooth stop, clear queue)
                    if (motorsInitialized) {
                        motionController.StopDecel();
                        SendResponseLine("Stop with deceleration - queue cleared");
                    } else {
                        SendResponseLine("error: Motors not initialized");
                    }
                    break;
                    
                case 202:  // M202 - Enable motors
                    motorX.EnableRequest(true);
                    motorY.EnableRequest(true);
                    // Try to reinitialize if not already initialized
                    if (!motorsInitialized) {
                        Delay_ms(100);
                        if (motionController.Initialize(&motorX, &motorY)) {
                            motorsInitialized = true;
                            motionController.SetMechanicalParamsX(MOTOR_X_STEPS_PER_REV, MOTOR_X_PITCH_MM, ClearCore::UNIT_MM);
                            motionController.SetMechanicalParamsY(MOTOR_Y_STEPS_PER_REV, MOTOR_Y_PITCH_MM, ClearCore::UNIT_MM);
                            motionController.FeedRateMMPerMin(currentFeedRate);
                            motionController.ArcVelMax(DEFAULT_VELOCITY_STEPS_PER_SEC);
                            motionController.ArcAccelMax(DEFAULT_ACCELERATION_STEPS_PER_SEC2);
                            motionController.SetPosition(0, 0);
                            SendResponseLine("Motors enabled and initialized");
                        } else {
                            SendResponseLine("Motors enabled but initialization failed (motors may not be attached)");
                        }
                    } else {
                        SendResponseLine("Motors enabled");
                    }
                    break;
                    
                case 203:  // M203 - Disable motors (stops motion and disables)
                    if (motorsInitialized) {
                        motionController.Stop();
                    }
                    motorX.EnableRequest(false);
                    motorY.EnableRequest(false);
                    SendResponseLine("Motors disabled");
                    break;
                    
                case 114:  // M114 - Get current position
                    if (motorsInitialized) {
                        char response[100];
                        if (unitMode == UNIT_MODE_INCHES) {
                            double xInches = motionController.CurrentXInches();
                            double yInches = motionController.CurrentYInches();
                            sprintf(response, "X:%.3f Y:%.3f", xInches, yInches);
                        } else {
                            double xMM = motionController.CurrentXMM();
                            double yMM = motionController.CurrentYMM();
                            sprintf(response, "X:%.3f Y:%.3f", xMM, yMM);
                        }
                        SendResponseLine(response);
                    } else {
                        SendResponseLine("error: Motors not initialized");
                    }
                    break;
                    
                case 115:  // M115 - Get status
                    {
                        char response[200];
                        if (motorsInitialized) {
                            sprintf(response, "Status: Active=%d Queue=%d Units=%s Coords=%s Motors=OK",
                                    motionController.IsActive() ? 1 : 0,
                                    motionController.MotionQueueCount(),
                                    unitMode == UNIT_MODE_INCHES ? "inches" : "mm",
                                    coordinateMode == COORD_ABSOLUTE ? "abs" : "inc");
                        } else {
                            sprintf(response, "Status: Motors=NotInitialized Units=%s Coords=%s",
                                    unitMode == UNIT_MODE_INCHES ? "inches" : "mm",
                                    coordinateMode == COORD_ABSOLUTE ? "abs" : "inc");
                        }
                        SendResponseLine(response);
                    }
                    break;
                    
                case 116:  // M116 - Debug: Get motion debug info
                    if (motorsInitialized) {
                        uint32_t stepsRemaining;
                        int32_t currentXSteps, currentYSteps;
                        bool isActive = motionController.GetDebugInfo(stepsRemaining, currentXSteps, currentYSteps);
                        
                        char response[200];
                        if (unitMode == UNIT_MODE_INCHES) {
                            double xInches = motionController.CurrentXInches();
                            double yInches = motionController.CurrentYInches();
                            sprintf(response, "Debug: Active=%d StepsRem=%lu X=%.3f(step:%ld) Y=%.3f(step:%ld)",
                                    isActive ? 1 : 0,
                                    (unsigned long)stepsRemaining,
                                    xInches, (long)currentXSteps,
                                    yInches, (long)currentYSteps);
                        } else {
                            double xMM = motionController.CurrentXMM();
                            double yMM = motionController.CurrentYMM();
                            sprintf(response, "Debug: Active=%d StepsRem=%lu X=%.3f(step:%ld) Y=%.3f(step:%ld)",
                                    isActive ? 1 : 0,
                                    (unsigned long)stepsRemaining,
                                    xMM, (long)currentXSteps,
                                    yMM, (long)currentYSteps);
                        }
                        SendResponseLine(response);
                    } else {
                        SendResponseLine("error: Motors not initialized");
                    }
                    break;
                    
                case 500:  // M500 - Run automated test sequence
                    {
                        if (!motorsInitialized) {
                            SendResponseLine("error: Motors not initialized");
                            break;
                        }
                        
                        SendResponseLine("Starting automated test sequence...");
                        
                        // Enable motors
                        motorX.EnableRequest(true);
                        motorY.EnableRequest(true);
                        SendResponseLine("Motors enabled");
                        Delay_ms(100);
                        
                        // Set feed rate (use F parameter if provided, otherwise default to 10 ipm)
                        double feedRate = (f >= 0.0) ? f : 10.0;
                        if (unitMode == UNIT_MODE_INCHES) {
                            motionController.FeedRateInchesPerMin(feedRate);
                            currentFeedRate = feedRate;
                        } else {
                            motionController.FeedRateMMPerMin(feedRate * 25.4);  // Convert to mm/min
                            currentFeedRate = feedRate * 25.4;
                        }
                        feedRateSet = true;
                        char feedMsg[50];
                        sprintf(feedMsg, "Feed rate set to %.1f ipm", feedRate);
                        SendResponseLine(feedMsg);
                        
                        // Set current position to origin
                        motionController.SetPosition(0, 0);
                        SendResponseLine("Position set to origin");
                        Delay_ms(50);
                        
                        // Suppress immediate position prints; we'll print after moves complete
                        suppressAutoPosition = true;

                        // 180° arc out: (0,0) -> (4,0), center (2,0), radius 2"
                        SendResponseLine("Test 1: G02 X4 Y0 I2 J0 (180 deg arc out)");
                        ExecuteG02(4.0, 0.0, 2.0, 0.0);
                        const double arcLenHalf = M_PI * 2.0;  // pi * radius
                        WaitForMotionComplete(CalculateMoveTimeoutMs(arcLenHalf, feedRate));
                        SendResponseLine("Test 1 complete - Position:");
                        PrintCurrentPosition();
                        
                        // 180° arc back: (4,0) -> (0,0), center (2,0), radius 2"
                        SendResponseLine("Test 2: G02 X0 Y0 I-2 J0 (180 deg arc back)");
                        ExecuteG02(0.0, 0.0, -2.0, 0.0);
                        WaitForMotionComplete(CalculateMoveTimeoutMs(arcLenHalf, feedRate));
                        SendResponseLine("Test 2 complete - Position:");
                        PrintCurrentPosition();
                        suppressAutoPosition = false;
                        
                        SendResponseLine("Test complete - Final position:");
                        PrintCurrentPosition();
                    }
                    break;
                    
                case 501:  // M501 - Random move test sequence
                    {
                        if (!motorsInitialized) {
                            SendResponseLine("error: Motors not initialized");
                            break;
                        }
                        
                        SendResponseLine("Starting random move test sequence...");
                        
                        // Enable motors
                        motorX.EnableRequest(true);
                        motorY.EnableRequest(true);
                        SendResponseLine("Motors enabled");
                        Delay_ms(100);
                        
                        randomSeedInit();
                        
                        // Set feed rate (use F parameter if provided, otherwise default to 10 ipm)
                        double feedRate = (f >= 0.0) ? f : 10.0;
                        if (unitMode == UNIT_MODE_INCHES) {
                            motionController.FeedRateInchesPerMin(feedRate);
                            currentFeedRate = feedRate;
                        } else {
                            motionController.FeedRateMMPerMin(feedRate * 25.4);  // Convert to mm/min
                            currentFeedRate = feedRate * 25.4;
                        }
                        feedRateSet = true;
                        char feedMsg[50];
                        sprintf(feedMsg, "Feed rate set to %.1f ipm", feedRate);
                        SendResponseLine(feedMsg);
                        
                        // Suppress immediate position prints; we'll print after moves complete
                        suppressAutoPosition = true;

                        // Generate 10 random moves
                        const int numMoves = 10;
                        const double maxRange = 10.0;  // Max 10mm or 10 inches
                        
                        for (int i = 0; i < numMoves; i++) {
                            // Random move type: 0=X only, 1=Y only, 2=XY
                            int moveType = randomNext() % 3;
                            double x = 0.0, y = 0.0;
                            
                            switch (moveType) {
                                case 0:  // X only
                                    x = randomDouble(-maxRange, maxRange);
                                    y = 0.0;
                                    break;
                                case 1:  // Y only
                                    x = 0.0;
                                    y = randomDouble(-maxRange, maxRange);
                                    break;
                                case 2:  // XY
                                    x = randomDouble(-maxRange, maxRange);
                                    y = randomDouble(-maxRange, maxRange);
                                    break;
                            }
                            
                            char moveMsg[100];
                            sprintf(moveMsg, "Move %d/%d: G01 X%.3f Y%.3f", i+1, numMoves, x, y);
                            SendResponseLine(moveMsg);
                            
                            ExecuteG01(x, y);
                            
                            // Wait for move to complete with timeout based on move distance
                            double distance = sqrt(x*x + y*y);
                            uint32_t timeoutMs = CalculateMoveTimeoutMs(distance, feedRate);
                            WaitForMotionComplete(timeoutMs);
                            Delay_ms(100);  // Small delay between moves
                        }
                        
                        // Return to origin at end of test
                        SendResponseLine("Returning to origin: G01 X0 Y0");
                        double currentX = 0.0;
                        double currentY = 0.0;
                        if (unitMode == UNIT_MODE_INCHES) {
                            currentX = motionController.CurrentXInches();
                            currentY = motionController.CurrentYInches();
                        } else {
                            currentX = motionController.CurrentXMM();
                            currentY = motionController.CurrentYMM();
                        }
                        double returnDistance = sqrt(currentX * currentX + currentY * currentY);
                        ExecuteG01(0.0, 0.0);
                        WaitForMotionComplete(CalculateMoveTimeoutMs(returnDistance, feedRate));
                        suppressAutoPosition = false;
                        
                        SendResponseLine("Random move test complete - Final position:");
                        PrintCurrentPosition();
                    }
                    break;
                    
                default:
                    SendResponse("Unknown M-code: M");
                    char codeStr[10];
                    sprintf(codeStr, "%d", code);
                    SendResponseLine(codeStr);
                    break;
            }
        }
    } else {
        SendResponseLine("Invalid command format");
    }
}

void ExecuteG01(double x, double y, double f) {
    // Check if motors are initialized
    if (!motorsInitialized) {
        SendResponseLine("error: Motors not initialized (motors may not be attached, use M202 to enable)");
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
        // Use current feed rate
        if (unitMode == UNIT_MODE_INCHES) {
            motionController.FeedRateInchesPerMin(currentFeedRate);
        } else {
            motionController.FeedRateMMPerMin(currentFeedRate);
        }
    }
    
    // Calculate target position
    double targetX = x;
    double targetY = y;
    
    if (coordinateMode == COORD_INCREMENTAL) {
        // Incremental mode - add to current position
        if (unitMode == UNIT_MODE_INCHES) {
            targetX = motionController.CurrentXInches() + x;
            targetY = motionController.CurrentYInches() + y;
        } else {
            targetX = motionController.CurrentXMM() + x;
            targetY = motionController.CurrentYMM() + y;
        }
    }
    
    // Convert to steps for queuing
    int32_t endXSteps, endYSteps;
    if (unitMode == UNIT_MODE_INCHES) {
        endXSteps = UnitConverter::DistanceToSteps(targetX, ClearCore::UNIT_INCHES, mechanicalConfigX);
        endYSteps = UnitConverter::DistanceToSteps(targetY, ClearCore::UNIT_INCHES, mechanicalConfigY);
    } else {
        endXSteps = UnitConverter::DistanceToSteps(targetX, ClearCore::UNIT_MM, mechanicalConfigX);
        endYSteps = UnitConverter::DistanceToSteps(targetY, ClearCore::UNIT_MM, mechanicalConfigY);
    }
    
    // Check if motors are initialized
    if (!motorsInitialized) {
        SendResponseLine("error: Motors not initialized (motors may not be attached, use M202 to enable)");
        return;
    }
    
    // Check if motors are enabled before queuing
    if (!motorX.EnableRequest() || !motorY.EnableRequest()) {
        SendResponseLine("error: Motors not enabled (use M202 to enable)");
        return;
    }
    
    // Queue or execute move based on current motion state
    // QueueLinear automatically starts if no motion is active, or queues if motion is active
    bool success = motionController.QueueLinear(endXSteps, endYSteps);
    
    if (!success) {
        char errorMsg[100];
        uint8_t queueCount = motionController.MotionQueueCount();
        if (queueCount >= 8) {
            sprintf(errorMsg, "error: Queue full (current: %d/8) - wait for moves to complete", queueCount);
        } else {
            sprintf(errorMsg, "error: Invalid move (queue: %d/8) - check motors enabled and parameters", queueCount);
        }
        SendResponseLine(errorMsg);
        return;
    }
    
    SendResponseLine("ok");
    if (!suppressAutoPosition) {
        double startX = (unitMode == UNIT_MODE_INCHES) ? motionController.CurrentXInches() : motionController.CurrentXMM();
        double startY = (unitMode == UNIT_MODE_INCHES) ? motionController.CurrentYInches() : motionController.CurrentYMM();
        double dist = hypot(targetX - startX, targetY - startY);
        double feed = (currentFeedRate > 0.0) ? currentFeedRate : 10.0;
        WaitForMotionComplete(CalculateMoveTimeoutMs(dist, feed));
        PrintCurrentPosition();
    }
}

void ExecuteG02(double x, double y, double i, double j, double f) {
    // Check if motors are initialized
    if (!motorsInitialized) {
        SendResponseLine("error: Motors not initialized (motors may not be attached, use M202 to enable)");
        return;
    }
    
    // Debug: Print parsed values
    char debugMsg[200];
    sprintf(debugMsg, "G02 parsed: X=%.3f Y=%.3f I=%.3f J=%.3f", x, y, i, j);
    SendResponseLine(debugMsg);
    
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
    
    // Calculate arc center and parameters
    double startX, startY;
    if (unitMode == UNIT_MODE_INCHES) {
        startX = motionController.CurrentXInches();
        startY = motionController.CurrentYInches();
    } else {
        startX = motionController.CurrentXMM();
        startY = motionController.CurrentYMM();
    }
    
    // Calculate end position
    double endX = x;
    double endY = y;
    if (coordinateMode == COORD_INCREMENTAL) {
        endX = startX + x;
        endY = startY + y;
    }
    
    // Debug: Print calculated values
    sprintf(debugMsg, "G02 calc: Start(%.3f,%.3f) End(%.3f,%.3f) Center(%.3f,%.3f) Radius=%.3f",
            startX, startY, endX, endY, startX + i, startY + j, sqrt(i * i + j * j));
    SendResponseLine(debugMsg);
    
    // Calculate center (I and J are relative to start position)
    double centerX = startX + i;
    double centerY = startY + j;
    
    // Calculate radius
    double radius = sqrt(i * i + j * j);
    
    // Calculate end angle (start angle calculated automatically by QueueArc)
    double endAngle = atan2(endY - centerY, endX - centerX);
    if (endAngle < 0) endAngle += 2 * M_PI;
    
    // Debug: Print end angle
    sprintf(debugMsg, "G02 endAngle: %.6f rad (%.2f deg)", endAngle, endAngle * 180.0 / M_PI);
    SendResponseLine(debugMsg);
    
    // Convert to steps for queuing
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
    
    // Debug: Calculate and print start angle
    double startAngle = atan2(startY - centerY, startX - centerX);
    if (startAngle < 0) startAngle += 2 * M_PI;
    sprintf(debugMsg, "G02 startAngle: %.6f rad (%.2f deg)", startAngle, startAngle * 180.0 / M_PI);
    SendResponseLine(debugMsg);
    
    // Debug: Print step values
    sprintf(debugMsg, "G02 steps: Center(%ld,%ld) Radius=%ld StartAngle=%.6f EndAngle=%.6f",
            (long)centerXSteps, (long)centerYSteps, (long)radiusSteps, startAngle, endAngle);
    SendResponseLine(debugMsg);
    
    // Check if motors are initialized
    if (!motorsInitialized) {
        SendResponseLine("error: Motors not initialized (motors may not be attached, use M202 to enable)");
        return;
    }
    
    // Check if motors are enabled before queuing
    if (!motorX.EnableRequest() || !motorY.EnableRequest()) {
        SendResponseLine("error: Motors not enabled (use M202 to enable)");
        return;
    }
    
    // Queue arc move (clockwise)
    // QueueArc automatically starts if no motion is active, or queues if motion is active
    bool success = motionController.QueueArc(centerXSteps, centerYSteps, radiusSteps, endAngle, true);
    
    if (!success) {
        char errorMsg[100];
        uint8_t queueCount = motionController.MotionQueueCount();
        if (queueCount >= 8) {
            sprintf(errorMsg, "error: Queue full (current: %d/8) - wait for moves to complete", queueCount);
        } else {
            sprintf(errorMsg, "error: Invalid arc (queue: %d/8) - check motors enabled and parameters", queueCount);
        }
        SendResponseLine(errorMsg);
        return;
    }
    
    SendResponseLine("ok");
    if (!suppressAutoPosition) {
        double diff = (endAngle >= startAngle) ? (endAngle - startAngle) : (startAngle - endAngle);
        double angleSpan = (diff <= M_PI) ? diff : (2.0 * M_PI - diff);
        double arcLen = radius * angleSpan;
        double feed = (currentFeedRate > 0.0) ? currentFeedRate : 10.0;
        WaitForMotionComplete(CalculateMoveTimeoutMs(arcLen, feed));
        PrintCurrentPosition();
    }
}

void ExecuteG03(double x, double y, double i, double j, double f) {
    // Check if motors are initialized
    if (!motorsInitialized) {
        SendResponseLine("error: Motors not initialized (motors may not be attached, use M202 to enable)");
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
    
    // Calculate arc center and parameters
    double startX, startY;
    if (unitMode == UNIT_MODE_INCHES) {
        startX = motionController.CurrentXInches();
        startY = motionController.CurrentYInches();
    } else {
        startX = motionController.CurrentXMM();
        startY = motionController.CurrentYMM();
    }
    
    // Calculate end position
    double endX = x;
    double endY = y;
    if (coordinateMode == COORD_INCREMENTAL) {
        endX = startX + x;
        endY = startY + y;
    }
    
    // Calculate center (I and J are relative to start position)
    double centerX = startX + i;
    double centerY = startY + j;
    
    // Calculate radius
    double radius = sqrt(i * i + j * j);
    
    // Calculate end angle (start angle calculated automatically by QueueArc)
    double endAngle = atan2(endY - centerY, endX - centerX);
    if (endAngle < 0) endAngle += 2 * M_PI;
    double startAngle = atan2(startY - centerY, startX - centerX);
    if (startAngle < 0) startAngle += 2 * M_PI;
    
    // Convert to steps for queuing
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
    
    // Check if motors are enabled before queuing
    if (!motorX.EnableRequest() || !motorY.EnableRequest()) {
        SendResponseLine("error: Motors not enabled (use M202 to enable)");
        return;
    }
    
    // Queue arc move (counterclockwise)
    // QueueArc automatically starts if no motion is active, or queues if motion is active
    bool success = motionController.QueueArc(centerXSteps, centerYSteps, radiusSteps, endAngle, false);
    
    if (!success) {
        char errorMsg[100];
        uint8_t queueCount = motionController.MotionQueueCount();
        if (queueCount >= 8) {
            sprintf(errorMsg, "error: Queue full (current: %d/8) - wait for moves to complete", queueCount);
        } else {
            sprintf(errorMsg, "error: Invalid arc (queue: %d/8) - check motors enabled and parameters", queueCount);
        }
        SendResponseLine(errorMsg);
        return;
    }
    
    SendResponseLine("ok");
    if (!suppressAutoPosition) {
        double diff = (endAngle >= startAngle) ? (endAngle - startAngle) : (startAngle - endAngle);
        double angleSpan = (diff <= M_PI) ? diff : (2.0 * M_PI - diff);
        double arcLen = radius * angleSpan;
        double feed = (currentFeedRate > 0.0) ? currentFeedRate : 10.0;
        WaitForMotionComplete(CalculateMoveTimeoutMs(arcLen, feed));
        PrintCurrentPosition();
    }
}

void PrintCurrentPosition() {
    if (!motorsInitialized) {
        return;
    }
    
    char response[100];
    if (unitMode == UNIT_MODE_INCHES) {
        double xInches = motionController.CurrentXInches();
        double yInches = motionController.CurrentYInches();
        sprintf(response, "X:%.3f Y:%.3f", xInches, yInches);
    } else {
        double xMM = motionController.CurrentXMM();
        double yMM = motionController.CurrentYMM();
        sprintf(response, "X:%.3f Y:%.3f", xMM, yMM);
    }
    SendResponseLine(response);
}

void SendResponse(const char* message) {
    if (!message) {
        return;  // Safety check - don't send null pointers
    }
    
    #if COMM_MODE == SERIAL_MODE
    // Only send if serial port is ready
    if (SerialPort) {
        SerialPort.Send(message);
    }
    #elif COMM_MODE == ETHERNET_MODE
    if (tcpClient.Connected()) {
        tcpClient.Send(message);
    }
    // Only send to USB if it's ready
    if (ConnectorUsb) {
        ConnectorUsb.Send(message);  // Also send to USB for debugging
    }
    #endif
}

void SendResponseLine(const char* message) {
    SendResponse(message);
    SendResponse("\r\n");  // Use CRLF for proper terminal line endings (PuTTY, etc.)
}
