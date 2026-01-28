/*
 * Title: CoordinatedArcMoves
 *
 * Objective:
 *    This example demonstrates coordinated continuous arc moves using two
 *    ClearPath-SD motors. The motors move in synchronized circular arcs with
 *    support for chaining multiple arcs without stopping.
 *
 * Description:
 *    This example enables two motors then commands a series of coordinated arc
 *    moves. The arcs can be chained together for continuous motion. Move status
 *    is printed to the USB serial port.
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
 *
 * ** Note: Homing is optional, and not required in this operational mode or in
 *    this example. This example assumes motors start at position (0, 0).
 *
 * ** Note: Set the Input Resolution in MSP the same as your motor's Positioning
 *    Resolution spec if you'd like the pulses sent by ClearCore to command a
 *    move of the same number of Encoder Counts, a 1:1 ratio.
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
#include <math.h>

// Specify which motors to move.
// Options are: ConnectorM0, ConnectorM1, ConnectorM2, or ConnectorM3.
#define motorX ConnectorM0
#define motorY ConnectorM1

// Select the baud rate to match the target serial device
#define baudRate 9600

// Specify which serial connector to use: ConnectorUsb, ConnectorCOM0, or
// ConnectorCOM1
#define SerialPort ConnectorUsb

// This example has built-in functionality to automatically clear motor alerts, 
//	including motor shutdowns. Any uncleared alert will cancel and disallow motion.
// WARNING: enabling automatic alert handling will clear alerts immediately when 
//	encountered and return a motor to a state in which motion is allowed. Before 
//	enabling this functionality, be sure to understand this behavior and ensure 
//	your system will not enter an unsafe state. 
// To enable automatic alert handling, #define HANDLE_ALERTS (1)
// To disable automatic alert handling, #define HANDLE_ALERTS (0)
#define HANDLE_ALERTS (0)

// Define the velocity and acceleration limits for arc moves
uint32_t arcVelocityLimit = 5000;  // pulses per sec
uint32_t arcAccelerationLimit = 50000; // pulses per sec^2

// Arc parameters
int32_t arcRadius = 10000;  // Arc radius in steps
int32_t arcCenterX = 0;     // Arc center X position
int32_t arcCenterY = 0;     // Arc center Y position

// Coordinated motion controller instance
CoordinatedMotionController arcController;

// Declares user-defined helper functions.
bool ExecuteArcMove(double startAngle, double endAngle, bool clockwise);
bool ExecuteContinuousArcs();
void PrintAlerts();
void HandleAlerts();

int main() {
    // Sets the input clocking rate. This normal rate is ideal for ClearPath
    // step and direction applications.
    MotorMgr.MotorInputClocking(MotorManager::CLOCK_RATE_NORMAL);

    // Sets all motor connectors into step and direction mode.
    MotorMgr.MotorModeSet(MotorManager::MOTOR_ALL,
                          Connector::CPM_MODE_STEP_AND_DIR);

    // Put the motor connectors into the HLFB mode to read bipolar PWM (the
    // correct mode for ASG w/ Measured Torque)
    motorX.HlfbMode(MotorDriver::HLFB_MODE_HAS_BIPOLAR_PWM);
    motorY.HlfbMode(MotorDriver::HLFB_MODE_HAS_BIPOLAR_PWM);

    // Set the HFLB carrier frequencies to 482 Hz
    motorX.HlfbCarrier(MotorDriver::HLFB_CARRIER_482_HZ);
    motorY.HlfbCarrier(MotorDriver::HLFB_CARRIER_482_HZ);

    // Sets up serial communication and waits up to 5 seconds for a port to open.
    SerialPort.Mode(Connector::USB_CDC);
    SerialPort.Speed(baudRate);
    uint32_t timeout = 5000;
    uint32_t startTime = Milliseconds();
    SerialPort.PortOpen();
    while (!SerialPort && Milliseconds() - startTime < timeout) {
        continue;
    }

    SerialPort.SendLine("Coordinated Arc Moves Example");
    SerialPort.SendLine("=============================");

    // Enables the motors; homing will begin automatically if enabled in MSP.
    motorX.EnableRequest(true);
    SerialPort.SendLine("Motor X (M-0) Enabled");
    motorY.EnableRequest(true);
    SerialPort.SendLine("Motor Y (M-1) Enabled");

    // Waits for both motors to finish enabling.
    uint32_t lastStatusTime = Milliseconds();
    SerialPort.SendLine("Waiting for HLFB...");
    while ( (motorX.HlfbState() != MotorDriver::HLFB_ASSERTED ||
            motorY.HlfbState() != MotorDriver::HLFB_ASSERTED) &&
            !motorX.StatusReg().bit.AlertsPresent && 
            !motorY.StatusReg().bit.AlertsPresent) {
        // Periodically prints out why the application is waiting.
        if (Milliseconds() - lastStatusTime > 1000) {
            SerialPort.SendLine("Waiting for HLFB to assert on both motors");
            lastStatusTime = Milliseconds();
        }
    }
    
    // Check if motor alert occurred during enabling
    if (motorX.StatusReg().bit.AlertsPresent || 
        motorY.StatusReg().bit.AlertsPresent) {
        SerialPort.SendLine("Motor alert detected.");		
        PrintAlerts();
        if(HANDLE_ALERTS){
            HandleAlerts();
        } else {
            SerialPort.SendLine("Enable automatic alert handling by setting HANDLE_ALERTS to 1.");
        }
        SerialPort.SendLine("Enabling may not have completed as expected. Proceed with caution.");		
        SerialPort.SendLine();
    } else {
        SerialPort.SendLine("Motors Ready");	
    }

    // Initialize coordinated motion controller
    if (!arcController.Initialize(&motorX, &motorY)) {
        SerialPort.SendLine("ERROR: Failed to initialize coordinated motion controller");
        while (true) {
            continue;
        }
    }
    
    SerialPort.SendLine("Coordinated motion controller initialized");
    
    // Set arc motion parameters
    arcController.ArcVelMax(arcVelocityLimit);
    arcController.ArcAccelMax(arcAccelerationLimit);
    
    // Set initial position (assuming motors start at 0, 0)
    arcController.SetPosition(0, 0);
    
    SerialPort.SendLine();
    SerialPort.SendLine("Starting arc moves...");
    SerialPort.SendLine();

    while (true) {
        // Example 1: Single 90-degree arc
        SerialPort.SendLine("Example 1: Single 90-degree arc (clockwise)");
        if (ExecuteArcMove(0, M_PI / 2, true)) {
            SerialPort.SendLine("Arc completed successfully");
        } else {
            SerialPort.SendLine("Arc failed");
        }
        Delay_ms(2000);
        
        // Example 2: Continuous arcs forming a square pattern
        SerialPort.SendLine("Example 2: Continuous arcs forming a circle");
        if (ExecuteContinuousArcs()) {
            SerialPort.SendLine("All arcs completed successfully");
        } else {
            SerialPort.SendLine("Arcs failed");
        }
        Delay_ms(2000);
        
        // Example 3: Return to start with arcs
        SerialPort.SendLine("Example 3: Return to start");
        arcController.MoveArc(arcCenterX, arcCenterY, arcRadius,
                              3 * M_PI / 2, 2 * M_PI, true);
        while (!arcController.ArcComplete()) {
            Delay_ms(10);
        }
        SerialPort.SendLine("Returned to start");
        Delay_ms(2000);
    }
}

/*------------------------------------------------------------------------------
 * ExecuteArcMove
 *
 *    Executes a single arc move and waits for completion.
 *
 * Parameters:
 *    double startAngle - Start angle in radians
 *    double endAngle   - End angle in radians
 *    bool clockwise    - Direction (true = clockwise)
 *
 * Returns: True if arc completed successfully
 */
bool ExecuteArcMove(double startAngle, double endAngle, bool clockwise) {
    // Check for alerts
    if (motorX.StatusReg().bit.AlertsPresent || 
        motorY.StatusReg().bit.AlertsPresent) {
        SerialPort.SendLine("Motor alert detected.");
        PrintAlerts();
        if(HANDLE_ALERTS){
            HandleAlerts();
        }
        return false;
    }
    
    // Command the arc move
    arcController.MoveArc(arcCenterX, arcCenterY, arcRadius,
                          startAngle, endAngle, clockwise);
    
    // Wait for arc to complete
    uint32_t lastStatusTime = Milliseconds();
    while (!arcController.ArcComplete() &&
           !motorX.StatusReg().bit.AlertsPresent && 
           !motorY.StatusReg().bit.AlertsPresent) {
        if (Milliseconds() - lastStatusTime > 1000) {
            SerialPort.Send("Arc in progress... Queued arcs: ");
            SerialPort.SendLine(arcController.QueueCount());
            lastStatusTime = Milliseconds();
        }
        Delay_ms(10);
    }
    
    // Check for alerts
    if (motorX.StatusReg().bit.AlertsPresent || 
        motorY.StatusReg().bit.AlertsPresent) {
        SerialPort.SendLine("Motor alert detected during arc.");
        PrintAlerts();
        if(HANDLE_ALERTS){
            HandleAlerts();
        }
        return false;
    }
    
    return true;
}

/*------------------------------------------------------------------------------
 * ExecuteContinuousArcs
 *
 *    Executes multiple arcs chained together for continuous motion.
 *
 * Returns: True if all arcs completed successfully
 */
bool ExecuteContinuousArcs() {
    // Chain four 90-degree arcs to form a complete circle
    // Start from current position
    arcController.MoveArcContinuous(arcCenterX, arcCenterY, arcRadius,
                                    M_PI / 2, true);
    arcController.MoveArcContinuous(arcCenterX, arcCenterY, arcRadius,
                                    M_PI, true);
    arcController.MoveArcContinuous(arcCenterX, arcCenterY, arcRadius,
                                    3 * M_PI / 2, true);
    arcController.MoveArcContinuous(arcCenterX, arcCenterY, arcRadius,
                                    2 * M_PI, true);
    
    // Wait for all arcs to complete
    uint32_t lastStatusTime = Milliseconds();
    while (!arcController.ArcComplete() &&
           !motorX.StatusReg().bit.AlertsPresent && 
           !motorY.StatusReg().bit.AlertsPresent) {
        if (Milliseconds() - lastStatusTime > 1000) {
            SerialPort.Send("Arcs in progress... Queued arcs: ");
            SerialPort.SendLine(arcController.QueueCount());
            lastStatusTime = Milliseconds();
        }
        Delay_ms(10);
    }
    
    // Check for alerts
    if (motorX.StatusReg().bit.AlertsPresent || 
        motorY.StatusReg().bit.AlertsPresent) {
        SerialPort.SendLine("Motor alert detected during arcs.");
        PrintAlerts();
        if(HANDLE_ALERTS){
            HandleAlerts();
        }
        return false;
    }
    
    return true;
}

/*------------------------------------------------------------------------------
 * PrintAlerts
 *
 *    Prints active alerts.
 */
void PrintAlerts(){
    // report status of alerts on motorX
    SerialPort.SendLine("Alerts present on motorX: ");
    if(motorX.AlertReg().bit.MotionCanceledInAlert){
        SerialPort.SendLine("    MotionCanceledInAlert "); }
    if(motorX.AlertReg().bit.MotionCanceledPositiveLimit){
        SerialPort.SendLine("    MotionCanceledPositiveLimit "); }
    if(motorX.AlertReg().bit.MotionCanceledNegativeLimit){
        SerialPort.SendLine("    MotionCanceledNegativeLimit "); }
    if(motorX.AlertReg().bit.MotionCanceledSensorEStop){
        SerialPort.SendLine("    MotionCanceledSensorEStop "); }
    if(motorX.AlertReg().bit.MotionCanceledMotorDisabled){
        SerialPort.SendLine("    MotionCanceledMotorDisabled "); }
    if(motorX.AlertReg().bit.MotorFaulted){
        SerialPort.SendLine("    MotorFaulted ");
    }
    
    // report status of alerts on motorY
    SerialPort.SendLine("Alerts present on motorY: ");
    if(motorY.AlertReg().bit.MotionCanceledInAlert){
        SerialPort.SendLine("    MotionCanceledInAlert "); }
    if(motorY.AlertReg().bit.MotionCanceledPositiveLimit){
        SerialPort.SendLine("    MotionCanceledPositiveLimit "); }
    if(motorY.AlertReg().bit.MotionCanceledNegativeLimit){
        SerialPort.SendLine("    MotionCanceledNegativeLimit "); }
    if(motorY.AlertReg().bit.MotionCanceledSensorEStop){
        SerialPort.SendLine("    MotionCanceledSensorEStop "); }
    if(motorY.AlertReg().bit.MotionCanceledMotorDisabled){
        SerialPort.SendLine("    MotionCanceledMotorDisabled "); }
    if(motorY.AlertReg().bit.MotorFaulted){
        SerialPort.SendLine("    MotorFaulted ");
    }
}

/*------------------------------------------------------------------------------
 * HandleAlerts
 *
 *    Clears alerts, including motor faults. 
 */
void HandleAlerts(){
    // for each motor, if a motor fault is present, clear it by cycling enable
    if(motorX.AlertReg().bit.MotorFaulted){
        SerialPort.SendLine("Faults present on motorX. Cycling enable signal to motor to clear faults.");
        motorX.EnableRequest(false);
        Delay_ms(10);
        motorX.EnableRequest(true);
    }
    if(motorY.AlertReg().bit.MotorFaulted){
        SerialPort.SendLine("Faults present on motorY. Cycling enable signal to motor to clear faults.");
        motorY.EnableRequest(false);
        Delay_ms(10);
        motorY.EnableRequest(true);
    }
    // clear alerts
    SerialPort.SendLine("Clearing alerts on both motors.");
    motorX.ClearAlerts();
    motorY.ClearAlerts();
}
