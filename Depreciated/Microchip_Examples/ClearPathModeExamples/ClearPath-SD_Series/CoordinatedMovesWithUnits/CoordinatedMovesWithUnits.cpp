/*
 * Title: CoordinatedMovesWithUnits
 *
 * Objective:
 *    This example demonstrates coordinated motion (both linear and arc moves)
 *    with unit conversion support. Motors can be configured with mechanical
 *    parameters (steps/rev, pitch) and moves can be commanded in physical units
 *    (inches, mm) rather than raw step counts.
 *
 * Description:
 *    This example configures two motors with mechanical parameters, then
 *    demonstrates:
 *    1. Single motor moves in physical units (inches, mm)
 *    2. Coordinated linear moves in physical units
 *    3. Coordinated arc moves in physical units
 *    4. Feed rate programming in physical units (inches/min, mm/min)
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

// ========== Mechanical Configuration ==========
// Configure these values to match your mechanical system

// Motor X: 800 steps/rev, 5mm pitch ball screw
#define MOTOR_X_STEPS_PER_REV 800
#define MOTOR_X_PITCH_MM 5.0

// Motor Y: 800 steps/rev, 5mm pitch ball screw
#define MOTOR_Y_STEPS_PER_REV 800
#define MOTOR_Y_PITCH_MM 5.0

// ========== Motion Parameters ==========
// Feed rates in physical units
#define FEED_RATE_INCHES_PER_MIN 100.0
#define FEED_RATE_MM_PER_MIN 2540.0  // 100 inches/min = 2540 mm/min
#define FEED_RATE_MM_PER_SEC 42.33   // 100 inches/min = 42.33 mm/sec

// Acceleration limit (steps/sec^2)
// This will be converted from physical units if needed
#define ACCELERATION_LIMIT 50000

// Coordinated motion controller instance
CoordinatedMotionController motionController;

// Declares user-defined helper functions.
void PrintAlerts(MotorDriver &motor);
void HandleAlerts(MotorDriver &motor);
void WaitForMove(MotorDriver &motor);
void WaitForCoordinatedMove();

int main() {
    // Set up serial communication
    SerialPort.Mode(Connector::USB_CDC);
    SerialPort.Speed(baudRate);
    SerialPort.PortOpen();
    while (!SerialPort) {
        continue;
    }
    
    SerialPort.Send("Coordinated Moves with Units Example");
    SerialPort.SendLine();
    SerialPort.SendLine();
    
    // Configure input clocking rate
    MotorMgr.MotorInputClocking(MotorManager::CLOCK_RATE_NORMAL);
    
    // Set all motor connectors into step and direction mode
    MotorMgr.MotorModeSet(MotorManager::MOTOR_ALL, Connector::CPM_MODE_STEP_AND_DIR);
    
    // Configure motors for bipolar PWM HLFB mode at 482Hz
    motorX.HlfbMode(MotorDriver::HLFB_MODE_HAS_BIPOLAR_PWM);
    motorX.HlfbCarrier(MotorDriver::HLFB_CARRIER_482_HZ);
    motorY.HlfbMode(MotorDriver::HLFB_MODE_HAS_BIPOLAR_PWM);
    motorY.HlfbCarrier(MotorDriver::HLFB_CARRIER_482_HZ);
    
    // Enable motors
    motorX.EnableRequest(true);
    motorY.EnableRequest(true);
    
    // Wait for motors to be enabled
    int timeout = 0;
    while ((!motorX.StatusReg().bit.ReadyState || !motorY.StatusReg().bit.ReadyState) &&
           timeout++ < 100) {
        continue;
    }
    
    if (!motorX.StatusReg().bit.ReadyState || !motorY.StatusReg().bit.ReadyState) {
        SerialPort.Send("Motor enable failed!");
        SerialPort.SendLine();
        return -1;
    }
    
    SerialPort.Send("Motors enabled");
    SerialPort.SendLine();
    
    // ========== Configure Unit Conversion ==========
    SerialPort.Send("Configuring mechanical parameters...");
    SerialPort.SendLine();
    
    // Configure motor X: 800 steps/rev, 5mm pitch
    if (!motorX.SetMechanicalParams(MOTOR_X_STEPS_PER_REV, MOTOR_X_PITCH_MM, UNIT_MM)) {
        SerialPort.Send("Failed to configure motor X mechanical parameters!");
        SerialPort.SendLine();
        return -1;
    }
    
    // Configure motor Y: 800 steps/rev, 5mm pitch
    if (!motorY.SetMechanicalParams(MOTOR_Y_STEPS_PER_REV, MOTOR_Y_PITCH_MM, UNIT_MM)) {
        SerialPort.Send("Failed to configure motor Y mechanical parameters!");
        SerialPort.SendLine();
        return -1;
    }
    
    SerialPort.Send("Mechanical parameters configured:");
    SerialPort.SendLine();
    SerialPort.Send("  Motor X: ");
    SerialPort.Send(MOTOR_X_STEPS_PER_REV);
    SerialPort.Send(" steps/rev, ");
    SerialPort.Send(MOTOR_X_PITCH_MM);
    SerialPort.Send("mm pitch");
    SerialPort.SendLine();
    SerialPort.Send("  Motor Y: ");
    SerialPort.Send(MOTOR_Y_STEPS_PER_REV);
    SerialPort.Send(" steps/rev, ");
    SerialPort.Send(MOTOR_Y_PITCH_MM);
    SerialPort.Send("mm pitch");
    SerialPort.SendLine();
    SerialPort.SendLine();
    
    // ========== Example 1: Single Motor Move in Physical Units ==========
    SerialPort.Send("Example 1: Single motor move in inches");
    SerialPort.SendLine();
    
    // Set feed rate in inches per minute
    motorX.FeedRateInchesPerMin(FEED_RATE_INCHES_PER_MIN);
    
    // Move 11.25 inches (285.75 mm)
    SerialPort.Send("Moving motor X 11.25 inches...");
    SerialPort.SendLine();
    
    if (motorX.MoveInches(11.25)) {
        WaitForMove(motorX);
        SerialPort.Send("Move complete. Position: ");
        SerialPort.Send(motorX.PositionInches());
        SerialPort.Send(" inches (");
        SerialPort.Send(motorX.PositionMM());
        SerialPort.Send(" mm)");
        SerialPort.SendLine();
    } else {
        SerialPort.Send("Move failed!");
        SerialPort.SendLine();
        PrintAlerts(motorX);
    }
    
    SerialPort.SendLine();
    Delay_ms(1000);
    
    // ========== Example 2: Coordinated Linear Move ==========
    SerialPort.Send("Example 2: Coordinated linear move in millimeters");
    SerialPort.SendLine();
    
    // Initialize coordinated motion controller
    if (!motionController.Initialize(&motorX, &motorY)) {
        SerialPort.Send("Failed to initialize coordinated motion controller!");
        SerialPort.SendLine();
        return -1;
    }
    
    // Configure mechanical parameters for coordinated motion
    motionController.SetMechanicalParamsX(MOTOR_X_STEPS_PER_REV, MOTOR_X_PITCH_MM, UNIT_MM);
    motionController.SetMechanicalParamsY(MOTOR_Y_STEPS_PER_REV, MOTOR_Y_PITCH_MM, UNIT_MM);
    
    // Set feed rate for coordinated motion
    motionController.FeedRateMMPerMin(FEED_RATE_MM_PER_MIN);
    motionController.ArcAccelMax(ACCELERATION_LIMIT);
    
    // Set initial position
    motionController.SetPosition(0, 0);
    
    // Move to (100mm, 50mm) in a straight line
    SerialPort.Send("Moving to (100mm, 50mm) in a straight line...");
    SerialPort.SendLine();
    
    if (motionController.MoveLinearMM(100.0, 50.0)) {
        WaitForCoordinatedMove();
        SerialPort.Send("Move complete. Position: (");
        SerialPort.Send(motionController.CurrentXMM());
        SerialPort.Send("mm, ");
        SerialPort.Send(motionController.CurrentYMM());
        SerialPort.Send("mm)");
        SerialPort.SendLine();
    } else {
        SerialPort.Send("Move failed!");
        SerialPort.SendLine();
    }
    
    SerialPort.SendLine();
    Delay_ms(1000);
    
    // ========== Example 3: Coordinated Arc Move ==========
    SerialPort.Send("Example 3: Coordinated arc move in inches");
    SerialPort.SendLine();
    
    // Set feed rate in inches per minute
    motionController.FeedRateInchesPerMin(FEED_RATE_INCHES_PER_MIN);
    
    // Move in a 10-inch radius arc from 0 to 90 degrees
    SerialPort.Send("Moving in a 10-inch radius arc (0 to 90 degrees)...");
    SerialPort.SendLine();
    
    if (motionController.MoveArcInches(0.0, 0.0, 10.0, 0.0, M_PI / 2.0, true)) {
        WaitForCoordinatedMove();
        SerialPort.Send("Arc complete. Position: (");
        SerialPort.Send(motionController.CurrentXInches());
        SerialPort.Send("in, ");
        SerialPort.Send(motionController.CurrentYInches());
        SerialPort.Send("in)");
        SerialPort.SendLine();
    } else {
        SerialPort.Send("Arc move failed!");
        SerialPort.SendLine();
    }
    
    SerialPort.SendLine();
    Delay_ms(1000);
    
    // ========== Example 4: Continuous Linear Moves ==========
    SerialPort.Send("Example 4: Continuous linear moves (chaining)");
    SerialPort.SendLine();
    
    // Queue multiple linear moves
    SerialPort.Send("Queueing multiple linear moves...");
    SerialPort.SendLine();
    
    motionController.MoveLinearMM(50.0, 50.0);
    motionController.MoveLinearContinuous(100.0, 50.0);
    motionController.MoveLinearContinuous(100.0, 100.0);
    motionController.MoveLinearContinuous(50.0, 100.0);
    motionController.MoveLinearContinuous(50.0, 50.0);  // Return to start
    
    WaitForCoordinatedMove();
    SerialPort.Send("All moves complete!");
    SerialPort.SendLine();
    
    SerialPort.SendLine();
    Delay_ms(1000);
    
    // ========== Example 5: Mixed Units ==========
    SerialPort.Send("Example 5: Mixed units (inches and mm)");
    SerialPort.SendLine();
    
    // Move using inches
    SerialPort.Send("Moving to (5 inches, 127 mm)...");
    SerialPort.SendLine();
    
    // Convert 5 inches to mm for Y axis, or use inches for both
    motionController.MoveLinearMM(5.0 * 25.4, 127.0);  // 5 inches = 127 mm
    
    WaitForCoordinatedMove();
    SerialPort.Send("Move complete!");
    SerialPort.SendLine();
    
    SerialPort.SendLine();
    SerialPort.Send("Example complete!");
    SerialPort.SendLine();
    
    return 0;
}

void PrintAlerts(MotorDriver &motor) {
    if (motor.StatusReg().bit.AlertsPresent) {
        SerialPort.Send("Motor alerts: 0x");
        SerialPort.Send(motor.AlertReg().reg, 16);
        SerialPort.SendLine();
    }
}

void HandleAlerts(MotorDriver &motor) {
    if (motor.StatusReg().bit.AlertsPresent) {
        motor.ClearAlerts();
        PrintAlerts(motor);
    }
}

void WaitForMove(MotorDriver &motor) {
    while (motor.StepsActive()) {
        if (HANDLE_ALERTS) {
            HandleAlerts(motor);
        }
        continue;
    }
}

void WaitForCoordinatedMove() {
    while (motionController.IsActive()) {
        if (HANDLE_ALERTS) {
            HandleAlerts(motorX);
            HandleAlerts(motorY);
        }
        continue;
    }
}
