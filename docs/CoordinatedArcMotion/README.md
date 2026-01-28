<!--
Copyright (c) 2026 Adam G. Sweeney <agsweeney@gmail.com>

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
-->

# Coordinated Motion System for ClearCore

## Table of Contents

1. [Overview](#overview)
2. [Theory of Operation](#theory-of-operation)
3. [Architecture](#architecture)
4. [Mathematical Foundation](#mathematical-foundation)
5. [Unit Conversion System](#unit-conversion-system)
6. [Implementation Details](#implementation-details)
7. [Usage Guide](#usage-guide)
8. [API Reference](#api-reference)
9. [Examples](#examples)
10. [G-Code Compatibility](#g-code-compatibility)
11. [Performance Characteristics](#performance-characteristics)
12. [Troubleshooting](#troubleshooting)
13. [Advanced Topics](#advanced-topics)

---

## Overview

The Coordinated Motion System enables smooth, continuous coordinated movements using two motors in the ClearCore library. This system provides:

- **Coordinated Arc Motion**: Two motors move synchronously along circular arc paths
- **Coordinated Linear Motion**: Two motors move synchronously along straight-line paths
- **Mixed Motion Chaining**: Seamlessly chain arcs and linear moves together
- **Unit Conversion Support**: Program moves in physical units (inches, mm, degrees) instead of raw step counts
- **Constant Path Velocity**: Speed along the programmed path remains constant
- **Continuous Motion**: Multiple moves can be chained seamlessly without stopping
- **Real-Time Performance**: Operates at 5 kHz sample rate (200 µs per calculation)
- **High Precision**: Fixed-point Q15 arithmetic provides sub-step accuracy

### Key Features

**Arc Motion:**
- ✅ True arc path following (not chord-based approximation)
- ✅ Constant speed along arc circumference
- ✅ Automatic motor speed adaptation
- ✅ Smooth arc-to-arc transitions

**Linear Motion:**
- ✅ Straight-line coordinated motion
- ✅ Constant velocity along path
- ✅ Smooth linear-to-linear transitions

**Mixed Motion:**
- ✅ Chain linear moves into arc moves
- ✅ Chain arc moves into linear moves
- ✅ Seamless transitions between motion types

**Unit Conversion:**
- ✅ Physical units (inches, mm, cm, meters, degrees, revolutions)
- ✅ Feed rate units (inches/min, mm/min, mm/sec, RPM)
- ✅ Automatic conversion based on mechanical parameters
- ✅ Backward compatible with step-based APIs

**General:**
- ✅ Real-time calculation (no pre-computation)
- ✅ Support for arbitrary paths and angles
- ✅ G-code compatible programming style

---

## Theory of Operation

### Fundamental Principle

The system maintains **constant tangential velocity** along the arc path. This means:
- The speed along the arc circumference is constant (user-specified)
- Individual motor speeds adapt automatically based on their position on the arc
- The arc path is followed precisely using parametric equations

### Coordinate System

The system uses a standard X-Y coordinate system:
- **X-axis**: Typically horizontal (Motor 0)
- **Y-axis**: Typically vertical (Motor 1)
- **Origin**: User-defined (usually at start position)
- **Units**: Positions can be specified in steps or physical units (inches, mm, etc.)

### Arc Definition

An arc is defined by:
- **Center Point**: (centerX, centerY) in steps or physical units
- **Radius**: Distance from center to arc path in steps or physical units
- **Start Angle**: Initial angle in radians (0 = +X axis, π/2 = +Y axis)
- **End Angle**: Final angle in radians
- **Direction**: Clockwise (CW) or Counterclockwise (CCW)

### Linear Motion Definition

A linear move is defined by:
- **Start Point**: (startX, startY) in steps or physical units
- **End Point**: (endX, endY) in steps or physical units
- **Velocity**: Speed along the straight-line path

### Motion Generation Process

**Arc Motion** - Generated using a **parametric approach**:

1. **Angle Parameter**: The angle θ parameterizes position along the arc
2. **Continuous Increment**: Angle is incremented smoothly each sample period
3. **Position Calculation**: Position calculated directly from angle using trigonometry
4. **Step Generation**: Step deltas derived from position differences

**Linear Motion** - Generated using **linear interpolation**:

1. **Direction Vector**: Calculated from start to end point
2. **Normalized Direction**: Unit vector along the path
3. **Constant Velocity**: Steps per sample calculated to maintain constant speed
4. **Step Generation**: Step deltas distributed proportionally to X and Y components

### Velocity Control

**Tangential Velocity** (constant along arc):
```
v = tangential velocity (steps/sec) - USER SPECIFIED
```

**Angular Velocity** (calculated):
```
ω = v / r  (rad/sec)
```

**Angle Increment Per Sample**:
```
dθ = ω × dt = (v / r) × (1 / sampleRateHz)
```

**Individual Motor Speeds** (adaptive):
```
v_x = v × cos(θ)  (X motor speed)
v_y = v × sin(θ)  (Y motor speed)
```

### Path Development

The arc is **NOT** divided into chords. Instead:

- **Point-by-Point Calculation**: One point calculated per sample period (200 µs)
- **True Arc Path**: Each point lies exactly on the mathematical arc
- **Dynamic Point Count**: Number of points = `(arc_length / velocity) × sample_rate`
- **Step Discretization**: Steps derived from position differences

---

## Architecture

### System Components

#### 1. ArcInterpolator Class

**Purpose**: Core arc interpolation engine

**Responsibilities**:
- Convert arc parameters to step sequences
- Maintain arc state and progress tracking
- Generate next step pair for each ISR call
- Handle arc completion

**Key Methods**:
- `InitializeArc()`: Set up arc parameters
- `GenerateNextSteps()`: Calculate next step pair
- `IsArcComplete()`: Check completion status

#### 2. CoordinatedMotionController Class

**Purpose**: High-level coordinator managing arc queue

**Responsibilities**:
- Manage queue of arc segments (up to 8 arcs)
- Coordinate two motors (X and Y axes)
- Integrate with existing MotorDriver infrastructure
- Handle velocity/acceleration limits
- Provide public API for arc commands

**Key Methods**:
- `Initialize()`: Set up with two motor references
- `MoveArc()`: Execute single arc move
- `MoveArcContinuous()`: Queue arc for chaining
- `UpdateFast()`: ISR callback for step generation

#### 3. MotorDriver Integration

**Modifications**:
- Added coordinated motion mode flag
- Added `CoordinatedMotionController` reference
- Modified `Refresh()` to check for coordinated mode
- When in coordinated mode, calls controller's `UpdateFast()` instead of `StepGenerator::StepsCalculated()`

#### 4. Trigonometric Lookup Tables

**Purpose**: Fast sin/cos calculations

**Implementation**:
- 1024-entry lookup table
- Linear interpolation between entries
- Q15 fixed-point format
- ~5-10 µs calculation time

### Data Flow

**Arc Motion**:
```
User Code
    ↓
CoordinatedMotionController::MoveArc()
    ↓
ArcInterpolator::InitializeArc()
    ↓
[ISR: 5 kHz]
    ↓
CoordinatedMotionController::UpdateFast()
    ↓
ArcInterpolator::GenerateNextSteps()
    ↓
MotorDriver::SetCoordinatedSteps()
    ↓
Motors (X and Y)
```

**Linear Motion**:
```
User Code
    ↓
CoordinatedMotionController::MoveLinear()
    ↓
LinearInterpolator::InitializeLinear()
    ↓
[ISR: 5 kHz]
    ↓
CoordinatedMotionController::UpdateFast()
    ↓
LinearInterpolator::GenerateNextSteps()
    ↓
MotorDriver::SetCoordinatedSteps()
    ↓
Motors (X and Y)
```

**Unit Conversion**:
```
User Code (Physical Units)
    ↓
UnitConverter::DistanceToSteps()
    ↓
CoordinatedMotionController::MoveArc() / MoveLinear()
    ↓
[Standard motion flow...]
```

### State Management

**Arc State**:
- Current angle along arc (Q15 fixed-point)
- Current X/Y positions (Q15 fixed-point)
- Remaining steps estimate
- Arc completion status

**Linear State**:
- Current X/Y positions (Q15 fixed-point)
- Step increments per sample (Q15 fixed-point)
- Remaining steps estimate
- Linear completion status

**Queue State**:
- Unified queue: Up to 8 motions (arcs or linear) queued
- Legacy queues: Separate arc and linear queues (for backward compatibility)
- Head/tail pointers for circular buffers
- Queue count tracking
- Motion type tracking (ARC, LINEAR, NONE)

---

## Mathematical Foundation

### Arc Parametric Equations

**Position on Arc**:
```
X(θ) = centerX + radius × cos(θ)
Y(θ) = centerY + radius × sin(θ)
```

Where:
- `θ` = angle parameter (radians)
- `centerX, centerY` = arc center position (steps)
- `radius` = arc radius (steps)

### Velocity Relationships

**Tangential Velocity** (constant):
```
v_tangential = constant (user-specified, steps/sec)
```

**Angular Velocity**:
```
ω = v_tangential / radius  (rad/sec)
```

**Angle Increment**:
```
dθ = ω × dt = (v_tangential / radius) × (1 / sampleRateHz)
```

**Individual Motor Velocities**:
```
v_x = v_tangential × cos(θ)  (X motor)
v_y = v_tangential × sin(θ)  (Y motor)
```

### Fixed-Point Arithmetic

**Q15 Format**:
- 15 fractional bits
- 1 sign bit
- 16 integer bits
- Range: -32768 to +32767 (represents -1.0 to +0.99997)

**Angle Representation**:
- 0 to 32768 represents 0 to 2π radians
- 16384 represents π radians
- 8192 represents π/2 radians

**Position Representation**:
- Positions stored as Q15 fixed-point
- Provides 1/32768 step precision
- Sub-step accuracy maintained

### Arc Length Calculation

**Arc Length**:
```
arc_length = radius × angle_span  (in steps)
```

**Total Steps Estimate**:
```
totalSteps = radius × (endAngle - startAngle) × (32768 / TWO_PI_QX)
```

**Number of Points**:
```
num_points = (arc_length / velocity) × sample_rate
```

### Step Delta Calculation

**Position Difference**:
```
ΔX = X(θ + dθ) - X(θ)
ΔY = Y(θ + dθ) - Y(θ)
```

**Step Deltas**:
```
stepsX = round(ΔX)  (integer steps)
stepsY = round(ΔY)  (integer steps)
```

---

## Unit Conversion System

### Overview

The unit conversion system allows programming moves in physical units (inches, millimeters, degrees) instead of raw step counts. This makes programming more intuitive and reduces errors from manual conversions.

### Supported Units

**Distance Units**:
- `UNIT_STEPS` - Raw steps (default, no conversion)
- `UNIT_INCHES` - Inches
- `UNIT_MM` - Millimeters
- `UNIT_CM` - Centimeters
- `UNIT_METERS` - Meters
- `UNIT_REVOLUTIONS` - Motor revolutions
- `UNIT_DEGREES` - Degrees (for rotary axes)

**Feed Rate Units**:
- `FR_UNIT_STEPS_PER_SEC` - Steps per second (default)
- `FR_UNIT_INCHES_PER_MIN` - Inches per minute
- `FR_UNIT_INCHES_PER_SEC` - Inches per second
- `FR_UNIT_MM_PER_MIN` - Millimeters per minute
- `FR_UNIT_MM_PER_SEC` - Millimeters per second
- `FR_UNIT_RPM` - Revolutions per minute
- `FR_UNIT_RPS` - Revolutions per second

### Mechanical Configuration

Each motor must be configured with its mechanical parameters:

```cpp
motor.SetMechanicalParams(
    stepsPerRevolution,  // Motor steps per revolution
    pitch,               // Lead screw pitch (or gear ratio)
    pitchUnit,           // Units for pitch (UNIT_MM, UNIT_INCHES, etc.)
    gearRatio            // Gear ratio (default 1.0)
);
```

**Example**:
```cpp
// Motor: 800 steps/rev, 5mm pitch ball screw
motor.SetMechanicalParams(800, 5.0, UNIT_MM);
```

### Conversion Formulas

**Distance to Steps** (Linear Motion):
```
steps = (distance / pitch) × stepsPerRevolution × gearRatio
```

**Distance to Steps** (Rotary Motion):
```
steps = (angle / 360) × stepsPerRevolution × gearRatio
```

**Feed Rate to Steps/Second** (Linear):
```
stepsPerSec = (feedRate / pitch) × stepsPerRevolution × gearRatio
```

**Feed Rate to Steps/Second** (Rotary):
```
stepsPerSec = (rpm / 60) × stepsPerRevolution × gearRatio
```

### Usage Example

```cpp
// Configure motor mechanical parameters
motor.SetMechanicalParams(800, 5.0, UNIT_MM);  // 800 steps/rev, 5mm pitch

// Set feed rate in inches per minute
motor.FeedRateInchesPerMin(100.0);

// Move in physical units
motor.MoveInches(11.25);  // Move 11.25 inches
motor.MoveMM(285.75);     // Move 285.75 mm

// Get position in physical units
double posInches = motor.PositionInches();
double posMM = motor.PositionMM();
```

---

## Implementation Details

### ISR Integration

**Sample Rate**: 5 kHz (200 µs period)

**ISR Flow**:
```cpp
TCC0_0_Handler()  // Hardware timer interrupt
    ↓
SysMgr.FastUpdate()
    ↓
MotorDriver::Refresh()  // For each motor
    ↓
[If coordinated mode]
    CoordinatedMotionController::UpdateFast()
        ↓
    ArcInterpolator::GenerateNextSteps()
        ↓
    MotorDriver::SetCoordinatedSteps()
```

**Timing Budget**:
- Total ISR time: 200 µs
- Arc calculation: ~40-70 µs
- Well within budget

### Fixed-Point Calculations

**Angle Increment**:
```cpp
int64_t angleIncrementPerSampleQx = 
    ((int64_t)velocityMax * TWO_PI_QX) / 
    (radiusSteps * (int64_t)sampleRateHz);
```

**Position Calculation**:
```cpp
int32_t cosVal = CosQx(currentAngleQx);
int32_t sinVal = SinQx(currentAngleQx);

int64_t newXQx = centerXQx + 
                 ((radiusQx * cosVal) >> 15);
int64_t newYQx = centerYQx + 
                 ((radiusQx * sinVal) >> 15);
```

**Step Deltas**:
```cpp
int32_t newXSteps = (int32_t)(newXQx >> FRACT_BITS);
int32_t newYSteps = (int32_t)(newYQx >> FRACT_BITS);

stepsX = newXSteps - lastXSteps;
stepsY = newYSteps - lastYSteps;
```

### Trigonometric Lookup

**Lookup Table**:
- 1024 entries covering 0 to 2π
- Each entry: `sin(i × 2π / 1024) × 32768`
- Linear interpolation between entries

**Lookup Process**:
```cpp
1. Normalize angle to 0-32767 range
2. Convert to table index: index = angleQx / 32
3. Get fractional part: frac = angleQx & 0x1F
4. Linear interpolation: val = val1 + (val2 - val1) × frac / 32
```

### Arc Queue Management

**Queue Structure**:
- Circular buffer of 8 arc segments
- Each segment stores: center, radius, end angle, direction
- Head/tail pointers for FIFO operation

**Queue Operations**:
- `MoveArcContinuous()`: Add arc to queue
- `ProcessNextArc()`: Remove and initialize next arc
- Automatic chaining when current arc completes

---

## Usage Guide

### Basic Setup

#### 1. Include Headers

```cpp
#include "ClearCore.h"
```

#### 2. Initialize Motors

```cpp
// Set motors to Step and Direction mode
MotorMgr.MotorModeSet(MotorManager::MOTOR_ALL,
                      Connector::CPM_MODE_STEP_AND_DIR);

// Set clock rate
MotorMgr.MotorInputClocking(MotorManager::CLOCK_RATE_NORMAL);

// Enable motors
ConnectorM0.EnableRequest(true);
ConnectorM1.EnableRequest(true);
```

#### 3. Create Controller

```cpp
CoordinatedMotionController arcController;

// Initialize with two motors
if (!arcController.Initialize(&ConnectorM0, &ConnectorM1)) {
    // Handle error
}
```

#### 4. Configure Motion Parameters

```cpp
// Set velocity (tangential speed along path)
arcController.ArcVelMax(5000);  // 5000 steps/sec

// Set acceleration limit
arcController.ArcAccelMax(50000);  // 50000 steps/sec²

// Set initial position (if not starting at origin)
arcController.SetPosition(0, 0);
```

#### 5. (Optional) Configure Unit Conversion

```cpp
// Configure mechanical parameters for X-axis motor
// Motor: 800 steps/rev, 5mm pitch ball screw
arcController.SetMechanicalParamsX(800, 5.0, UNIT_MM);

// Configure mechanical parameters for Y-axis motor
arcController.SetMechanicalParamsY(800, 5.0, UNIT_MM);

// Set feed rate in physical units
arcController.ArcFeedRateInchesPerMin(100.0);
// or
arcController.ArcFeedRateMMPerMin(2540.0);
```

### Single Arc Move

#### Basic Arc

```cpp
// Move in a 90° arc, radius 10000 steps, clockwise
// Center at (0, 0), from 0° to 90°
arcController.MoveArc(
    0,              // centerX
    0,              // centerY
    10000,          // radius (steps)
    0,              // startAngle (radians)
    M_PI / 2,       // endAngle (radians)
    true            // clockwise
);

// Wait for completion
while (!arcController.ArcComplete()) {
    Delay_ms(10);
}
```

#### Arc with Different Center

```cpp
// Arc centered at (5000, 5000)
arcController.MoveArc(
    5000,           // centerX
    5000,           // centerY
    10000,          // radius
    0,              // startAngle
    M_PI,           // endAngle (180°)
    true            // clockwise
);
```

### Continuous Arc Chaining

#### Chain Multiple Arcs

```cpp
// Chain four 90° arcs to form a complete circle
arcController.MoveArcContinuous(0, 0, 10000, M_PI / 2, true);
arcController.MoveArcContinuous(0, 0, 10000, M_PI, true);
arcController.MoveArcContinuous(0, 0, 10000, 3 * M_PI / 2, true);
arcController.MoveArcContinuous(0, 0, 10000, 2 * M_PI, true);

// Wait for all arcs to complete
while (!arcController.ArcComplete()) {
    Delay_ms(10);
}
```

#### Chain with Different Radii

```cpp
// Start with large radius arc
arcController.MoveArc(0, 0, 20000, 0, M_PI / 2, true);

// Chain smaller radius arc (tangent continuity maintained)
arcController.MoveArcContinuous(0, 0, 10000, M_PI, true);
```

### Linear Coordinated Moves

#### Single Linear Move

```cpp
// Move in a straight line from current position to (10000, 5000)
arcController.MoveLinear(10000, 5000);

// Wait for completion
while (!arcController.ArcComplete()) {
    Delay_ms(10);
}
```

#### Linear Move with Absolute Start Position

```cpp
// Move from (0, 0) to (10000, 5000)
arcController.MoveLinearAbsolute(0, 0, 10000, 5000);
```

#### Continuous Linear Moves

```cpp
// Chain multiple linear moves
arcController.MoveLinear(10000, 0);
arcController.MoveLinearContinuous(10000, 10000);
arcController.MoveLinearContinuous(0, 10000);
arcController.MoveLinearContinuous(0, 0);  // Return to start

// Wait for all moves to complete
while (!arcController.ArcComplete()) {
    Delay_ms(10);
}
```

### Mixed Motion Chaining

#### Chain Linear into Arc

```cpp
// Start with a linear move
arcController.MoveLinear(10000, 0);

// Queue an arc move (will chain after linear completes)
arcController.QueueArc(0, 0, 10000, M_PI / 2, true);

// Queue another linear move (will chain after arc completes)
arcController.QueueLinear(20000, 10000);

// All motions execute continuously!
```

#### Chain Arc into Linear

```cpp
// Start with an arc move
arcController.MoveArc(0, 0, 10000, 0, M_PI / 2, true);

// Queue a linear move (will chain after arc completes)
arcController.QueueLinear(20000, 10000);

// Queue another arc (will chain after linear completes)
arcController.QueueArc(20000, 0, 10000, M_PI, false);
```

#### Complex Path Example

```cpp
// Create a complex path: linear -> arc -> linear -> arc
arcController.MoveLinear(10000, 0);                    // Move right
arcController.QueueArc(10000, 0, 5000, M_PI / 2, true); // Arc up
arcController.QueueLinear(10000, 10000);               // Move up
arcController.QueueArc(0, 10000, 10000, M_PI, true);   // Arc left
arcController.QueueLinear(0, 0);                       // Return to start
```

### Unit Conversion Examples

#### Configure Units for Coordinated Motion

```cpp
// Configure mechanical parameters
// Motor X: 800 steps/rev, 5mm pitch
arcController.SetMechanicalParamsX(800, 5.0, UNIT_MM);

// Motor Y: 800 steps/rev, 5mm pitch
arcController.SetMechanicalParamsY(800, 5.0, UNIT_MM);

// Set feed rate in inches per minute
arcController.ArcFeedRateInchesPerMin(100.0);
```

#### Arc Moves with Units

```cpp
// Move arc in inches
arcController.MoveArcInches(
    0.0,        // centerX (inches)
    0.0,        // centerY (inches)
    10.0,       // radius (inches)
    0.0,        // startAngle (radians)
    M_PI / 2,   // endAngle (radians)
    true        // clockwise
);

// Move arc in millimeters
arcController.MoveArcMM(
    0.0,        // centerX (mm)
    0.0,        // centerY (mm)
    254.0,      // radius (mm) = 10 inches
    0.0,        // startAngle
    M_PI / 2,   // endAngle
    true
);
```

#### Linear Moves with Units

```cpp
// Move linear in inches
arcController.MoveLinearInches(11.25, 5.0);

// Move linear in millimeters
arcController.MoveLinearMM(285.75, 127.0);  // 11.25" = 285.75mm, 5" = 127mm
```

#### Get Position in Units

```cpp
// Get current position in inches
double xInches = arcController.CurrentXInches();
double yInches = arcController.CurrentYInches();

// Get current position in millimeters
double xMM = arcController.CurrentXMM();
double yMM = arcController.CurrentYMM();
```

#### Single Motor Moves with Units

```cpp
// Configure motor mechanical parameters
ConnectorM0.SetMechanicalParams(800, 5.0, UNIT_MM);  // 800 steps/rev, 5mm pitch

// Set feed rate in inches per minute
ConnectorM0.FeedRateInchesPerMin(100.0);

// Move in physical units
ConnectorM0.MoveInches(11.25);  // Move 11.25 inches
ConnectorM0.MoveMM(285.75);     // Move 285.75 mm

// Get position in physical units
double posInches = ConnectorM0.PositionInches();
double posMM = ConnectorM0.PositionMM();
```

### Motion Control

#### Stop Motion

```cpp
// Stop immediately (abrupt)
arcController.Stop();

// Stop with deceleration
arcController.StopDecel();
```

#### Check Status

```cpp
// Check if motion is active
if (arcController.IsActive()) {
    // Motion in progress
}

// Check if all arcs complete
if (arcController.ArcComplete()) {
    // All motion finished
}

// Get current position
int32_t x = arcController.CurrentX();
int32_t y = arcController.CurrentY();

// Get queue status
uint8_t queued = arcController.QueueCount();
```

### Error Handling

#### Validate Before Move

```cpp
// Check motors are enabled
if (!ConnectorM0.EnableRequest() || !ConnectorM1.EnableRequest()) {
    // Enable motors first
}

// Check for alerts
if (ConnectorM0.StatusReg().bit.AlertsPresent ||
    ConnectorM1.StatusReg().bit.AlertsPresent) {
    // Handle alerts
}

// Check queue not full
if (arcController.QueueCount() >= 8) {
    // Wait for queue space
}
```

#### Handle Errors During Motion

```cpp
arcController.MoveArc(0, 0, 10000, 0, M_PI / 2, true);

while (!arcController.ArcComplete()) {
    // Check for alerts
    if (ConnectorM0.StatusReg().bit.AlertsPresent ||
        ConnectorM1.StatusReg().bit.AlertsPresent) {
        arcController.Stop();
        // Handle error
        break;
    }
    Delay_ms(10);
}
```

---

## API Reference

### CoordinatedMotionController Class

#### Constructor

```cpp
CoordinatedMotionController();
```

Creates a new controller instance.

#### Initialize

```cpp
bool Initialize(MotorDriver* motorX, MotorDriver* motorY);
```

**Parameters**:
- `motorX`: Pointer to X-axis motor (typically ConnectorM0)
- `motorY`: Pointer to Y-axis motor (typically ConnectorM1)

**Returns**: `true` if initialization successful

**Description**: Initializes the controller with two motor references and enables coordinated mode.

#### MoveArc

```cpp
bool MoveArc(int32_t centerX, int32_t centerY,
             int32_t radius,
             double startAngle, double endAngle,
             bool clockwise);
```

**Parameters**:
- `centerX`: Arc center X position (steps)
- `centerY`: Arc center Y position (steps)
- `radius`: Arc radius (steps)
- `startAngle`: Start angle (radians, 0 = +X axis)
- `endAngle`: End angle (radians)
- `clockwise`: `true` for clockwise, `false` for counterclockwise

**Returns**: `true` if arc command accepted

**Description**: Executes a single arc move. Stops any current motion and starts the new arc immediately.

#### MoveArcContinuous

```cpp
bool MoveArcContinuous(int32_t centerX, int32_t centerY,
                       int32_t radius,
                       double endAngle,
                       bool clockwise);
```

**Parameters**:
- `centerX`: Arc center X position (steps)
- `centerY`: Arc center Y position (steps)
- `radius`: Arc radius (steps)
- `endAngle`: End angle (radians)
- `clockwise`: `true` for clockwise, `false` for counterclockwise

**Returns**: `true` if arc queued successfully

**Description**: Queues an arc to be executed after the current arc completes. Start angle is automatically calculated for tangent continuity.

#### ArcVelMax

```cpp
void ArcVelMax(uint32_t velMax);
```

**Parameters**:
- `velMax`: Maximum tangential velocity (steps/sec)

**Description**: Sets the maximum velocity along the arc path. This is the speed the tool will travel along the arc circumference.

#### ArcAccelMax

```cpp
void ArcAccelMax(uint32_t accelMax);
```

**Parameters**:
- `accelMax`: Maximum acceleration (steps/sec²)

**Description**: Sets the maximum acceleration limit (currently used for reference, full acceleration profiling may be added in future).

#### Stop

```cpp
void Stop();
```

**Description**: Stops motion immediately. Clears arc queue and stops motors abruptly.

#### MoveLinear

```cpp
bool MoveLinear(int32_t endX, int32_t endY);
```

**Parameters**:
- `endX`: Ending X position (steps)
- `endY`: Ending Y position (steps)

**Returns**: `true` if move command accepted

**Description**: Executes a coordinated linear move from current position to the specified end point. Stops any current motion and starts the new linear move immediately.

#### MoveLinearAbsolute

```cpp
bool MoveLinearAbsolute(int32_t startX, int32_t startY,
                       int32_t endX, int32_t endY);
```

**Parameters**:
- `startX`: Starting X position (steps)
- `startY`: Starting Y position (steps)
- `endX`: Ending X position (steps)
- `endY`: Ending Y position (steps)

**Returns**: `true` if move command accepted

**Description**: Executes a coordinated linear move from the specified start position to the end position. Sets current position to start position.

#### MoveLinearContinuous

```cpp
bool MoveLinearContinuous(int32_t endX, int32_t endY);
```

**Parameters**:
- `endX`: Ending X position (steps)
- `endY`: Ending Y position (steps)

**Returns**: `true` if move queued successfully

**Description**: Queues a linear move to be executed after the current linear move completes.

#### QueueArc

```cpp
bool QueueArc(int32_t centerX, int32_t centerY,
             int32_t radius,
             double endAngle,
             bool clockwise);
```

**Parameters**:
- `centerX`: Arc center X position (steps)
- `centerY`: Arc center Y position (steps)
- `radius`: Arc radius (steps)
- `endAngle`: End angle (radians) - angle where arc should end
- `clockwise`: `true` for clockwise, `false` for counterclockwise

**Returns**: `true` if arc queued successfully

**Description**: Queues an arc move that can chain from any motion type (arc or linear). The arc will start smoothly from the current position when the previous motion completes. The start angle is automatically calculated using `CalculateStartAngle()` to ensure tangent continuity from the current position.

#### QueueLinear

```cpp
bool QueueLinear(int32_t endX, int32_t endY);
```

**Parameters**:
- `endX`: Ending X position (steps)
- `endY`: Ending Y position (steps)

**Returns**: `true` if move queued successfully

**Description**: Queues a linear move that can chain from any motion type (arc or linear). The linear move will start smoothly from the current position when the previous motion completes.

#### StopDecel

```cpp
void StopDecel();
```

**Description**: Stops motion with deceleration. Clears arc queue and stops motors smoothly.

#### IsActive

```cpp
bool IsActive() const;
```

**Returns**: `true` if coordinated motion is currently active

#### ArcComplete

```cpp
bool ArcComplete() const;
```

**Returns**: `true` if no motion is active and no arcs are queued in the legacy arc queue

**Note**: This method checks the legacy arc queue (`m_arcQueueCount`). To check the unified motion queue (which includes both arcs and linear moves), use `MotionQueueCount() == 0 && !IsActive()`.

#### CurrentX / CurrentY

```cpp
int32_t CurrentX() const;
int32_t CurrentY() const;
```

**Returns**: Current X or Y position (steps)

#### SetPosition

```cpp
void SetPosition(int32_t x, int32_t y);
```

**Parameters**:
- `x`: X position (steps)
- `y`: Y position (steps)

**Description**: Sets the current position (useful for homing or initialization).

#### QueueCount

```cpp
uint8_t QueueCount() const;
```

**Returns**: Number of arcs currently queued (0-8)

### MotorDriver Class - Unit Conversion Methods

#### SetMechanicalParams

```cpp
bool SetMechanicalParams(uint32_t stepsPerRev, double pitch,
                        UnitType pitchUnit, double gearRatio = 1.0);
```

**Parameters**:
- `stepsPerRev`: Motor steps per revolution
- `pitch`: Lead screw pitch (or gear ratio for rotary)
- `pitchUnit`: Units for pitch (UNIT_MM, UNIT_INCHES, etc.)
- `gearRatio`: Gear ratio (default 1.0)

**Returns**: `true` if configuration successful

**Description**: Configures mechanical parameters for unit conversion.

#### MoveInches / MoveMM / MoveCM / MoveMeters

```cpp
bool MoveInches(double distance, MoveTarget moveTarget = MOVE_TARGET_REL_END_POSN);
bool MoveMM(double distance, MoveTarget moveTarget = MOVE_TARGET_REL_END_POSN);
bool MoveCM(double distance, MoveTarget moveTarget = MOVE_TARGET_REL_END_POSN);
bool MoveMeters(double distance, MoveTarget moveTarget = MOVE_TARGET_REL_END_POSN);
```

**Parameters**:
- `distance`: Distance in specified unit
- `moveTarget`: Move target type (absolute or relative)

**Description**: Move motor in physical units.

#### MoveRevolutions / MoveDegrees

```cpp
bool MoveRevolutions(double revolutions, MoveTarget moveTarget = MOVE_TARGET_REL_END_POSN);
bool MoveDegrees(double degrees, MoveTarget moveTarget = MOVE_TARGET_REL_END_POSN);
```

**Description**: Move motor in rotary units (revolutions or degrees).

#### FeedRateInchesPerMin / FeedRateMMPerMin / FeedRateMMPerSec / FeedRateRPM

```cpp
void FeedRateInchesPerMin(double feedRate);
void FeedRateMMPerMin(double feedRate);
void FeedRateMMPerSec(double feedRate);
void FeedRateRPM(double rpm);
```

**Description**: Set feed rate in physical units.

#### PositionInches / PositionMM / PositionRevolutions / PositionDegrees

```cpp
double PositionInches() const;
double PositionMM() const;
double PositionRevolutions() const;
double PositionDegrees() const;
```

**Returns**: Current position in specified unit

**Description**: Get current position in physical units.

---

## Examples

### Example 1: Simple 90° Arc

```cpp
#include "ClearCore.h"

CoordinatedMotionController controller;

int main() {
    // Initialize motors
    MotorMgr.MotorModeSet(MotorManager::MOTOR_ALL,
                          Connector::CPM_MODE_STEP_AND_DIR);
    ConnectorM0.EnableRequest(true);
    ConnectorM1.EnableRequest(true);
    
    // Initialize controller
    controller.Initialize(&ConnectorM0, &ConnectorM1);
    controller.ArcVelMax(5000);
    controller.SetPosition(0, 0);
    
    // Execute 90° arc
    controller.MoveArc(0, 0, 10000, 0, M_PI / 2, true);
    
    // Wait for completion
    while (!controller.ArcComplete()) {
        Delay_ms(10);
    }
    
    return 0;
}
```

### Example 2: Continuous Circle

```cpp
// Chain four arcs to form a complete circle
controller.MoveArcContinuous(0, 0, 10000, M_PI / 2, true);
controller.MoveArcContinuous(0, 0, 10000, M_PI, true);
controller.MoveArcContinuous(0, 0, 10000, 3 * M_PI / 2, true);
controller.MoveArcContinuous(0, 0, 10000, 2 * M_PI, true);

while (!controller.ArcComplete()) {
    Delay_ms(10);
}
```

### Example 3: Complex Pattern

```cpp
// Create a figure-8 pattern
// First loop (clockwise)
controller.MoveArc(0, 0, 10000, 0, 2 * M_PI, true);

// Second loop (counterclockwise, offset center)
controller.MoveArcContinuous(20000, 0, 10000, 0, 2 * M_PI, false);
```

### Example 4: Error Handling

```cpp
bool ExecuteArcSafely(CoordinatedMotionController& controller,
                      int32_t centerX, int32_t centerY,
                      int32_t radius, double startAngle,
                      double endAngle, bool clockwise) {
    // Validate motors enabled
    if (!ConnectorM0.EnableRequest() || !ConnectorM1.EnableRequest()) {
        return false;
    }
    
    // Check for alerts
    if (ConnectorM0.StatusReg().bit.AlertsPresent ||
        ConnectorM1.StatusReg().bit.AlertsPresent) {
        return false;
    }
    
    // Check queue space
    if (controller.QueueCount() >= 8) {
        return false;
    }
    
    // Execute arc
    if (!controller.MoveArc(centerX, centerY, radius,
                            startAngle, endAngle, clockwise)) {
        return false;
    }
    
    // Monitor for errors
    while (!controller.ArcComplete()) {
        if (ConnectorM0.StatusReg().bit.AlertsPresent ||
            ConnectorM1.StatusReg().bit.AlertsPresent) {
            controller.Stop();
            return false;
        }
        Delay_ms(10);
    }
    
    return true;
}
```

### Example 5: Linear Coordinated Moves

```cpp
#include "ClearCore.h"

CoordinatedMotionController controller;

int main() {
    // Initialize motors and controller
    MotorMgr.MotorModeSet(MotorManager::MOTOR_ALL,
                          Connector::CPM_MODE_STEP_AND_DIR);
    ConnectorM0.EnableRequest(true);
    ConnectorM1.EnableRequest(true);
    
    controller.Initialize(&ConnectorM0, &ConnectorM1);
    controller.ArcVelMax(5000);
    controller.SetPosition(0, 0);
    
    // Move in a square pattern using linear moves
    controller.MoveLinear(10000, 0);      // Right
    controller.MoveLinearContinuous(10000, 10000);  // Up
    controller.MoveLinearContinuous(0, 10000);      // Left
    controller.MoveLinearContinuous(0, 0);          // Down
    
    // Wait for completion
    while (!controller.ArcComplete()) {
        Delay_ms(10);
    }
    
    return 0;
}
```

### Example 6: Mixed Motion Chaining

```cpp
// Create a complex path mixing linear and arc moves
controller.MoveLinear(10000, 0);                    // Start: move right
controller.QueueArc(10000, 0, 5000, M_PI / 2, true); // Arc up
controller.QueueLinear(10000, 10000);              // Move up
controller.QueueArc(0, 10000, 10000, M_PI, true);   // Arc left
controller.QueueLinear(0, 0);                       // Return to start

// All motions execute continuously!
while (!controller.ArcComplete()) {
    Delay_ms(10);
}
```

### Example 7: Unit Conversion

```cpp
#include "ClearCore.h"

CoordinatedMotionController controller;

int main() {
    // Initialize motors
    MotorMgr.MotorModeSet(MotorManager::MOTOR_ALL,
                          Connector::CPM_MODE_STEP_AND_DIR);
    ConnectorM0.EnableRequest(true);
    ConnectorM1.EnableRequest(true);
    
    // Configure unit conversion
    controller.Initialize(&ConnectorM0, &ConnectorM1);
    controller.SetMechanicalParamsX(800, 5.0, UNIT_MM);  // 800 steps/rev, 5mm pitch
    controller.SetMechanicalParamsY(800, 5.0, UNIT_MM);
    controller.ArcFeedRateInchesPerMin(100.0);
    
    // Move in physical units
    controller.MoveLinearInches(11.25, 5.0);  // Move to (11.25", 5")
    
    // Move arc in inches
    controller.MoveArcInches(0, 0, 10.0, 0, M_PI / 2, true);
    
    // Get position in units
    double xInches = controller.CurrentXInches();
    double yMM = controller.CurrentYMM();
    
    return 0;
}
```

---

## Performance Characteristics

### Timing

**ISR Execution Time**:
- Arc calculation: ~40-70 µs
- Linear calculation: ~20-40 µs
- Unit conversion: ~1-5 µs (done outside ISR)
- Total ISR budget: 200 µs
- Utilization: ~20-35%

**Sample Rate**:
- Fixed: 5000 Hz (200 µs period)
- One point calculated per sample period
- 5000 points per second

### Memory Usage

**Static Memory**:
- Unified motion queue: ~400-600 bytes (8 motions × 50-75 bytes)
- Legacy queues: ~200-400 bytes (arc + linear queues)
- Lookup tables: ~2-4 KB (1024-entry sin/cos)
- State variables: ~150 bytes
- Unit conversion config: ~100 bytes (per motor)
- **Total**: ~3-5 KB

**Dynamic Memory**:
- None (all static allocation)

### CPU Load

**ISR Overhead**:
- ~20-35% of 200 µs window
- Well within real-time constraints

**Background Processing**:
- Minimal (only on motion commands)
- Unit conversion done outside ISR

### Point Generation Rate

**Points Per Second**:
- Fixed: 5000 points/sec (sample rate)

**Points Per Arc**:
- Dynamic: `(arc_length / velocity) × 5000`
- Example: 15,708 step arc at 5000 steps/sec = 15,708 points

### Velocity Range

**Recommended Velocities**:
- Low: 1000-2000 steps/sec (very smooth)
- Medium: 5000-10000 steps/sec (optimal)
- High: 15000-20000 steps/sec (still smooth)

**Maximum Practical**:
- Limited by motor capabilities
- Typically 20000-50000 steps/sec depending on motor

---

## Troubleshooting

### Common Issues

#### Issue: Arc Not Starting

**Symptoms**: `MoveArc()` returns false, no motion

**Possible Causes**:
1. Motors not enabled
2. Alerts present
3. Invalid arc parameters (radius ≤ 0)
4. Controller not initialized

**Solutions**:
```cpp
// Check initialization
if (!controller.IsActive()) {
    controller.Initialize(&ConnectorM0, &ConnectorM1);
}

// Check motors enabled
if (!ConnectorM0.EnableRequest()) {
    ConnectorM0.EnableRequest(true);
}

// Check alerts
if (ConnectorM0.StatusReg().bit.AlertsPresent) {
    ConnectorM0.ClearAlerts();
}

// Validate parameters
if (radius <= 0) {
    // Use valid radius
}
```

#### Issue: Jerky Motion

**Symptoms**: Motion is not smooth, visible steps

**Possible Causes**:
1. Velocity too high for sample rate
2. Radius too small
3. Motor resolution mismatch

**Solutions**:
```cpp
// Reduce velocity
controller.ArcVelMax(2000);  // Lower velocity

// Increase radius
// Use larger radius for smoother motion

// Check motor resolution
// Ensure steps match motor resolution
```

#### Issue: Arc Not Completing

**Symptoms**: Motion stops before reaching end angle

**Possible Causes**:
1. Alerts during motion
2. Motor limits reached
3. Queue overflow

**Solutions**:
```cpp
// Monitor alerts
while (!controller.ArcComplete()) {
    if (ConnectorM0.StatusReg().bit.AlertsPresent) {
        // Handle alert
    }
    Delay_ms(10);
}

// Check limits
// Ensure arc doesn't exceed motor travel limits

// Check queue
if (controller.QueueCount() >= 8) {
    // Wait for queue space
}
```

#### Issue: Position Drift

**Symptoms**: End position doesn't match expected

**Possible Causes**:
1. Initial position not set correctly
2. Step rounding errors accumulating
3. Motor missed steps

**Solutions**:
```cpp
// Set initial position
controller.SetPosition(actualX, actualY);

// Use homing to establish known position
// Re-home periodically if needed

// Check for motor faults
if (ConnectorM0.StatusReg().bit.MotorInFault) {
    // Handle fault
}
```

### Debugging Tips

#### Enable Serial Output

```cpp
SerialPort.Mode(Connector::USB_CDC);
SerialPort.Speed(9600);
SerialPort.PortOpen();

// Log arc status
SerialPort.Send("Current X: ");
SerialPort.SendLine(controller.CurrentX());
SerialPort.Send("Current Y: ");
SerialPort.SendLine(controller.CurrentY());
SerialPort.Send("Queue Count: ");
SerialPort.SendLine(controller.QueueCount());
```

#### Monitor Step Generation

```cpp
// In GenerateNextSteps(), add logging:
SerialPort.Send("Steps X: ");
SerialPort.Send(stepsX);
SerialPort.Send(" Steps Y: ");
SerialPort.SendLine(stepsY);
```

#### Check Arc Parameters

```cpp
// Validate before move
if (radius <= 0) {
    SerialPort.SendLine("ERROR: Invalid radius");
    return;
}

if (startAngle < 0 || startAngle > 2 * M_PI) {
    SerialPort.SendLine("ERROR: Invalid start angle");
    return;
}
```

---

## G-Code Compatibility

### Overview

The coordinated motion system is designed to be compatible with G-code programming, making it easy to integrate with CNC controllers, CAM software, and G-code interpreters. The library provides a natural mapping between G-code commands and coordinated motion functions.

### G-Code Command Mapping

#### G00 - Rapid Positioning (Linear Move)

**G-Code Format**:
```
G00 X11.25 Y5.0
```

**Library Equivalent**:
```cpp
// Configure units first
controller.SetMechanicalParamsX(800, 5.0, UNIT_MM);
controller.SetMechanicalParamsY(800, 5.0, UNIT_MM);

// Rapid move (use maximum velocity)
controller.ArcVelMax(10000);  // High speed for rapid
controller.MoveLinearInches(11.25, 5.0);
```

**Notes**:
- G00 typically uses maximum velocity
- No feed rate specified (uses current max velocity)
- Absolute or incremental based on G90/G91 mode

#### G01 - Linear Interpolation (Coordinated Linear Move)

**G-Code Format**:
```
G01 X11.25 Y5.0 F100
```

**Library Equivalent**:
```cpp
// Set feed rate (F100 = 100 inches/min)
controller.ArcFeedRateInchesPerMin(100.0);

// Execute linear move
controller.MoveLinearInches(11.25, 5.0);
```

**With Incremental Mode (G91)**:
```cpp
// G91 G01 X11.25 Y5.0 F100
// Get current position
double currentX = controller.CurrentXInches();
double currentY = controller.CurrentYInches();

// Move relative to current position
controller.MoveLinearInches(currentX + 11.25, currentY + 5.0);
```

#### G02 - Circular Interpolation Clockwise (Arc Move)

**G-Code Format**:
```
G02 X10.0 Y10.0 I5.0 J0.0 F100
```

Where:
- `X, Y` = end point
- `I, J` = center offset from start point (I=X offset, J=Y offset)

**Library Equivalent**:
```cpp
// Calculate arc center
double startX = controller.CurrentXInches();
double startY = controller.CurrentYInches();
double centerX = startX + 5.0;  // I offset
double centerY = startY + 0.0;  // J offset

// Calculate radius
double radius = sqrt(5.0 * 5.0 + 0.0 * 0.0);  // sqrt(I² + J²)

// Calculate angles
double startAngle = atan2(startY - centerY, startX - centerX);
double endAngle = atan2(10.0 - centerY, 10.0 - centerX);

// Normalize angles
if (startAngle < 0) startAngle += 2 * M_PI;
if (endAngle < 0) endAngle += 2 * M_PI;

// Set feed rate
controller.ArcFeedRateInchesPerMin(100.0);

// Execute arc move
controller.MoveArcInches(centerX, centerY, radius, startAngle, endAngle, true);
```

#### G03 - Circular Interpolation Counterclockwise (Arc Move)

**G-Code Format**:
```
G03 X10.0 Y10.0 I5.0 J0.0 F100
```

**Library Equivalent**:
```cpp
// Same as G02, but clockwise = false
controller.MoveArcInches(centerX, centerY, radius, startAngle, endAngle, false);
```

### G-Code Helper Functions

Here's a complete example of G-code command parsing and execution:

```cpp
#include "ClearCore.h"
#include <math.h>
#include <string.h>

CoordinatedMotionController controller;

// G-code state
enum GCodeMode {
    G90_ABSOLUTE,
    G91_INCREMENTAL
};

GCodeMode coordinateMode = G90_ABSOLUTE;
double currentFeedRate = 100.0;  // inches/min
bool feedRateSet = false;

// Parse and execute G00 (Rapid)
void ExecuteG00(double x, double y) {
    if (coordinateMode == G90_ABSOLUTE) {
        controller.MoveLinearInches(x, y);
    } else {
        double currentX = controller.CurrentXInches();
        double currentY = controller.CurrentYInches();
        controller.MoveLinearInches(currentX + x, currentY + y);
    }
}

// Parse and execute G01 (Linear)
void ExecuteG01(double x, double y, double f = -1.0) {
    if (f >= 0.0) {
        controller.ArcFeedRateInchesPerMin(f);
        currentFeedRate = f;
        feedRateSet = true;
    } else if (feedRateSet) {
        controller.ArcFeedRateInchesPerMin(currentFeedRate);
    }
    
    if (coordinateMode == G90_ABSOLUTE) {
        controller.MoveLinearInches(x, y);
    } else {
        double currentX = controller.CurrentXInches();
        double currentY = controller.CurrentYInches();
        controller.MoveLinearInches(currentX + x, currentY + y);
    }
}

// Parse and execute G02/G03 (Arc)
void ExecuteG02G03(double x, double y, double i, double j, 
                   double f, bool clockwise) {
    // Get current position
    double startX = controller.CurrentXInches();
    double startY = controller.CurrentYInches();
    
    // Calculate end position
    double endX, endY;
    if (coordinateMode == G90_ABSOLUTE) {
        endX = x;
        endY = y;
    } else {
        endX = startX + x;
        endY = startY + y;
    }
    
    // Calculate arc center
    double centerX = startX + i;
    double centerY = startY + j;
    
    // Calculate radius
    double radius = sqrt(i * i + j * j);
    
    // Calculate angles
    double startAngle = atan2(startY - centerY, startX - centerX);
    double endAngle = atan2(endY - centerY, endX - centerX);
    
    // Normalize angles to 0-2π
    if (startAngle < 0) startAngle += 2 * M_PI;
    if (endAngle < 0) endAngle += 2 * M_PI;
    
    // Set feed rate
    if (f >= 0.0) {
        controller.ArcFeedRateInchesPerMin(f);
        currentFeedRate = f;
        feedRateSet = true;
    } else if (feedRateSet) {
        controller.ArcFeedRateInchesPerMin(currentFeedRate);
    }
    
    // Execute arc
    controller.MoveArcInches(centerX, centerY, radius, startAngle, endAngle, clockwise);
}

// Example G-code parser (simplified)
void ParseGCode(const char* line) {
    // Simple parser - in production, use a proper G-code parser library
    double x = NAN, y = NAN, i = NAN, j = NAN, f = -1.0;
    int gCode = -1;
    
    // Parse line (simplified - real parser would be more robust)
    char* token = strtok((char*)line, " ");
    while (token != NULL) {
        if (token[0] == 'G') {
            gCode = atoi(&token[1]);
        } else if (token[0] == 'X') {
            x = atof(&token[1]);
        } else if (token[0] == 'Y') {
            y = atof(&token[1]);
        } else if (token[0] == 'I') {
            i = atof(&token[1]);
        } else if (token[0] == 'J') {
            j = atof(&token[1]);
        } else if (token[0] == 'F') {
            f = atof(&token[1]);
        } else if (strcmp(token, "G90") == 0) {
            coordinateMode = G90_ABSOLUTE;
        } else if (strcmp(token, "G91") == 0) {
            coordinateMode = G91_INCREMENTAL;
        }
        token = strtok(NULL, " ");
    }
    
    // Execute command
    switch (gCode) {
        case 0:
            if (!isnan(x) && !isnan(y)) {
                ExecuteG00(x, y);
            }
            break;
        case 1:
            if (!isnan(x) && !isnan(y)) {
                ExecuteG01(x, y, f);
            }
            break;
        case 2:
            if (!isnan(x) && !isnan(y) && !isnan(i) && !isnan(j)) {
                ExecuteG02G03(x, y, i, j, f, true);  // Clockwise
            }
            break;
        case 3:
            if (!isnan(x) && !isnan(y) && !isnan(i) && !isnan(j)) {
                ExecuteG02G03(x, y, i, j, f, false);  // Counterclockwise
            }
            break;
    }
}

// Example usage
int main() {
    // Initialize motors
    MotorMgr.MotorModeSet(MotorManager::MOTOR_ALL, Connector::CPM_MODE_STEP_AND_DIR);
    ConnectorM0.EnableRequest(true);
    ConnectorM1.EnableRequest(true);
    
    // Initialize system
    controller.Initialize(&ConnectorM0, &ConnectorM1);
    controller.SetMechanicalParamsX(800, 5.0, UNIT_MM);
    controller.SetMechanicalParamsY(800, 5.0, UNIT_MM);
    controller.SetPosition(0, 0);
    
    // Parse G-code commands
    ParseGCode("G90");              // Absolute coordinates
    ParseGCode("G01 X11.25 Y5.0 F100");  // Linear move
    ParseGCode("G02 X10.0 Y10.0 I5.0 J0.0");  // Arc move
    
    // Wait for completion
    while (!controller.ArcComplete() && controller.MotionQueueCount() > 0) {
        Delay_ms(10);
    }
    
    return 0;
}
```

### Coordinate System Mapping

**G-Code Coordinate System**:
- X-axis: Horizontal (typically)
- Y-axis: Vertical (typically)
- Z-axis: Not used in 2D coordinated motion
- Origin: Set by G92 or homing

**Library Coordinate System**:
- X-axis: Motor X (typically ConnectorM0)
- Y-axis: Motor Y (typically ConnectorM1)
- Origin: Set by `SetPosition()`

**Mapping**:
```cpp
// G-code X → Library X
// G-code Y → Library Y
// G-code Z → Not applicable (2D only)
```

### Feed Rate Handling

**G-Code Feed Rate (F command)**:
- Units depend on G20/G21 (inches/mm)
- Persists until changed
- Applies to G01, G02, G03

**Library Feed Rate**:
```cpp
// Set feed rate based on G-code units
if (unitsInches) {
    controller.ArcFeedRateInchesPerMin(feedRate);
} else {
    controller.ArcFeedRateMMPerMin(feedRate);
}

// Feed rate persists until changed
// Can be overridden per move if needed
```

### Common G-Code Patterns

#### Square Pattern
```
G90 G01 F100
G01 X10.0 Y0.0
G01 X10.0 Y10.0
G01 X0.0 Y10.0
G01 X0.0 Y0.0
```

**Library Equivalent**:
```cpp
controller.ArcFeedRateInchesPerMin(100.0);
controller.MoveLinearInches(10.0, 0.0);
controller.MoveLinearContinuous(10.0, 10.0);
controller.MoveLinearContinuous(0.0, 10.0);
controller.MoveLinearContinuous(0.0, 0.0);
```

#### Circle Pattern
```
G90 G02 F100
G02 X0.0 Y0.0 I5.0 J0.0  ; Complete circle (assumes start at 0,0)
```

**Library Equivalent**:
```cpp
controller.ArcFeedRateInchesPerMin(100.0);
// Assuming start position is (0, 0)
double startX = controller.CurrentXInches();  // Should be 0.0
double startY = controller.CurrentYInches();  // Should be 0.0
double centerX = startX + 5.0;  // I offset = 5.0
double centerY = startY + 0.0;  // J offset = 0.0
double radius = 5.0;  // sqrt(I² + J²) = sqrt(5² + 0²) = 5.0
controller.MoveArcInches(centerX, centerY, radius, 0, 2 * M_PI, true);
```

#### Mixed Path
```
G90 G01 F100
G01 X10.0 Y0.0
G02 X10.0 Y10.0 I0.0 J5.0
G01 X0.0 Y10.0
G02 X0.0 Y0.0 I0.0 J-5.0
```

**Library Equivalent**:
```cpp
controller.ArcFeedRateInchesPerMin(100.0);
controller.MoveLinearInches(10.0, 0.0);

// G02 X10.0 Y10.0 I0.0 J5.0
// Start: (10, 0), End: (10, 10)
// Center offset: I=0, J=5 → Center = (10+0, 0+5) = (10, 5)
// Radius = sqrt(0² + 5²) = 5.0
// End angle = π/2 (pointing up from center)
// Start angle calculated automatically by QueueArc
controller.QueueArc(10.0, 5.0, 5.0, M_PI / 2, true);  // Arc up

controller.QueueLinear(0.0, 10.0);                     // Move left

// G02 X0.0 Y0.0 I0.0 J-5.0
// Start: (0, 10), End: (0, 0)
// Center offset: I=0, J=-5 → Center = (0+0, 10+(-5)) = (0, 5)
// Radius = sqrt(0² + (-5)²) = 5.0
// End angle = 3π/2 (pointing down from center)
// Start angle calculated automatically by QueueArc
controller.QueueArc(0.0, 5.0, 5.0, 3 * M_PI / 2, true);  // Arc down
```

### Integration with G-Code Interpreters

The library can be integrated with existing G-code interpreters:

1. **Parse G-code commands** using a G-code parser library
2. **Map to library functions** using the helper functions above
3. **Handle coordinate modes** (G90/G91)
4. **Manage feed rates** (F commands)
5. **Queue motions** for continuous execution

### Limitations and Considerations

**Not Supported (2D only)**:
- G-code Z-axis motion
- 3D arcs (helical interpolation)
- Tool length compensation
- Cutter radius compensation (G40/G41/G42)

**Supported**:
- G00, G01, G02, G03 (2D)
- Absolute and incremental coordinates (G90/G91)
- Feed rate programming (F)
- Continuous motion chaining
- Unit conversion (G20/G21 equivalent via unit methods)

**Recommended Approach**:
- Use a G-code parser library for robust parsing
- Implement helper functions to map G-code to library calls
- Handle coordinate system transformations if needed
- Manage feed rate state across commands

## Advanced Topics

### Coordinate System Transformations

If your physical system doesn't match the standard X-Y coordinate system:

```cpp
// Transform coordinates before calling MoveArc
int32_t transformedX = originalX * cos(rotation) - originalY * sin(rotation);
int32_t transformedY = originalX * sin(rotation) + originalY * cos(rotation);
```

### Velocity Profiling

Currently, the system uses constant velocity. For acceleration/deceleration profiling:

```cpp
// Future enhancement: Add velocity ramping
// For now, use lower velocities for smoother starts/stops
controller.ArcVelMax(2000);  // Lower velocity = smoother
```

### Custom Arc Shapes

To create non-circular arcs, you can approximate with multiple small arcs:

```cpp
// Approximate ellipse with multiple arcs
for (int i = 0; i < numSegments; i++) {
    double angle = (2 * M_PI * i) / numSegments;
    double nextAngle = (2 * M_PI * (i + 1)) / numSegments;
    // Calculate center and radius for this segment
    // Chain arcs together
}
```

### G-Code Integration

See the [G-Code Compatibility](#g-code-compatibility) section above for complete details on integrating with G-code interpreters.

### Multi-Axis Extension

To extend to 3 axes (X, Y, Z):

```cpp
// Would require:
// 1. Extended CoordinatedMotionController
// 2. 3D arc interpolation
// 3. Additional motor reference
// Future enhancement
```

---

## References

### Related Documentation

- `DESIGN_COORDINATED_ARCS.md`: Detailed design document
- `IMPLEMENTATION_SUMMARY.md`: Implementation overview
- `MOTION_SMOOTHNESS.md`: Smoothness analysis
- `CONSTANT_PATH_VELOCITY.md`: Velocity control details
- `ARC_PATH_DEVELOPMENT.md`: Path generation explanation
- `ARC_POINT_COUNT.md`: Point count calculations
- `BRESENHAM_COMPARISON.md`: Algorithm comparison

### ClearCore Documentation

- [ClearCore Library Documentation](https://teknic-inc.github.io/ClearCore-library/)
- [ClearCore User Manual](https://www.teknic.com/files/downloads/clearcore_user_manual.pdf)
- [ClearPath Motor Manual](https://www.teknic.com/files/downloads/clearpath_user_manual.pdf)

### Mathematical References

- **Parametric Equations**: Standard parametric form for circles
- **Fixed-Point Arithmetic**: Q15 format (15 fractional bits)
- **Trigonometric Functions**: sin/cos lookup tables with interpolation

---

## License

Copyright (c) 2026 Adam G. Sweeney <agsweeney@gmail.com>

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

---

## Version History

- **v1.0** (2025-01-28): Initial implementation
  - Basic arc interpolation
  - Continuous arc chaining
  - Constant tangential velocity
  - Real-time ISR integration

---

## Support

For questions, issues, or contributions, please contact:
**Adam G. Sweeney** <agsweeney@gmail.com>
