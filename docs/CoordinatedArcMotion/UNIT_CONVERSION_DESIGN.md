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

# Unit Conversion System Design for ClearCore

## Executive Summary

**Yes, the ClearCore library can be enhanced to support units.** This document outlines a comprehensive design for adding unit conversion capabilities, allowing users to program moves in physical units (inches, mm, degrees) rather than raw step counts.

## Overview

### Current State
- All moves specified in **step pulses**
- Velocities in **steps/second**
- No unit conversion or mechanical parameter tracking
- User must manually calculate conversions

### Proposed Enhancement
- Support for multiple units (steps, inches, mm, degrees, revolutions)
- Motor/mechanical configuration (steps/rev, pitch, gear ratios)
- Automatic unit conversion
- Feed rate support (inches/min, mm/min, mm/sec)
- Integration with existing Move() functions and coordinated arcs

## Requirements Analysis

### User Requirements

**Example Use Case**:
```cpp
// Motor: 800 steps/rev, 5mm pitch ball screw
// Desired: Move 11.25 inches at 100 inches/min

// Current (manual calculation):
// 11.25 inches = 285.75 mm
// 285.75 mm / 5mm per rev = 57.15 revolutions
// 57.15 rev × 800 steps/rev = 45,720 steps
// 100 in/min = 2540 mm/min = 42.33 mm/sec
// 42.33 mm/sec / 5mm per rev = 8.466 rev/sec
// 8.466 rev/sec × 800 steps/rev = 6,773 steps/sec

motor1.Move(45720);
motor1.VelMax(6773);

// Desired (with units):
motor1.SetMechanicalParams(800, 5.0, UNIT_MM);  // steps/rev, pitch, pitch units
motor1.MoveInches(11.25);
motor1.FeedRateInchesPerMin(100);
```

### Supported Units

**Distance Units**:
- Steps (raw, default)
- Inches
- Millimeters
- Centimeters
- Meters
- Revolutions
- Degrees (for rotary axes)

**Velocity/Feed Rate Units**:
- Steps/second (raw, default)
- Inches/minute
- Inches/second
- Millimeters/minute
- Millimeters/second
- Revolutions/minute (RPM)
- Revolutions/second

## Architecture Design

### 1. UnitConverter Class

**Purpose**: Core unit conversion engine

**Responsibilities**:
- Convert between physical units and steps
- Handle unit scaling and precision
- Support fixed-point arithmetic for performance

**Key Methods**:
```cpp
class UnitConverter {
public:
    // Convert distance to steps
    int32_t DistanceToSteps(double distance, UnitType unit);
    
    // Convert steps to distance
    double StepsToDistance(int32_t steps, UnitType unit);
    
    // Convert feed rate to steps/sec
    uint32_t FeedRateToStepsPerSec(double feedRate, FeedRateUnit unit);
    
    // Convert steps/sec to feed rate
    double StepsPerSecToFeedRate(uint32_t stepsPerSec, FeedRateUnit unit);
};
```

### 2. MotorMechanicalConfig Class

**Purpose**: Store motor/mechanical parameters

**Data Structure**:
```cpp
struct MotorMechanicalConfig {
    uint32_t stepsPerRevolution;    // Motor steps per revolution
    double pitch;                    // Lead screw pitch or gear ratio
    UnitType pitchUnit;              // Units for pitch (mm, inches, etc.)
    double gearRatio;                // Gear ratio (1.0 = direct drive)
    bool inverted;                   // Direction inversion
    
    // Calculated conversion factors
    double stepsPerUnit;             // Steps per unit (calculated)
    double unitsPerStep;             // Units per step (calculated)
};
```

### 3. MotorDriver Enhancement

**New Methods**:
```cpp
class MotorDriver {
public:
    // Configure mechanical parameters
    bool SetMechanicalParams(uint32_t stepsPerRev, double pitch,
                             UnitType pitchUnit, double gearRatio = 1.0);
    
    // Move with units
    bool MoveInches(double distance, MoveTarget target = MOVE_TARGET_REL_END_POSN);
    bool MoveMM(double distance, MoveTarget target = MOVE_TARGET_REL_END_POSN);
    bool MoveRevolutions(double revolutions, MoveTarget target = MOVE_TARGET_REL_END_POSN);
    bool MoveDegrees(double degrees, MoveTarget target = MOVE_TARGET_REL_END_POSN);
    
    // Generic move with unit specification
    bool MoveWithUnit(double distance, UnitType unit, MoveTarget target = MOVE_TARGET_REL_END_POSN);
    
    // Feed rate settings
    void FeedRateInchesPerMin(double feedRate);
    void FeedRateMMPerMin(double feedRate);
    void FeedRateMMPerSec(double feedRate);
    void FeedRateRPM(double rpm);
    
    // Generic feed rate with unit
    void FeedRateWithUnit(double feedRate, FeedRateUnit unit);
    
    // Get position in units
    double PositionInches() const;
    double PositionMM() const;
    double PositionRevolutions() const;
    double PositionDegrees() const;
    double PositionWithUnit(UnitType unit) const;
    
    // Velocity in units
    void VelMaxInchesPerMin(double velMax);
    void VelMaxMMPerMin(double velMax);
    void VelMaxRPM(double rpm);
    void VelMaxWithUnit(double velMax, FeedRateUnit unit);
};
```

### 4. CoordinatedMotionController Enhancement

**New Methods**:
```cpp
class CoordinatedMotionController {
public:
    // Configure units for coordinated motion
    void SetUnits(UnitType unit);
    
    // Arc moves with units
    bool MoveArcInches(double centerX, double centerY, double radius,
                      double startAngle, double endAngle, bool clockwise);
    bool MoveArcMM(double centerX, double centerY, double radius,
                  double startAngle, double endAngle, bool clockwise);
    
    // Feed rate with units
    void ArcFeedRateInchesPerMin(double feedRate);
    void ArcFeedRateMMPerMin(double feedRate);
    void ArcFeedRateMMPerSec(double feedRate);
    
    // Get position in units
    double CurrentXInches() const;
    double CurrentYInches() const;
    double CurrentXMM() const;
    double CurrentYMM() const;
};
```

## Implementation Details

### Unit Conversion Calculations

#### Distance to Steps

**Linear Motion (Lead Screw)**:
```
steps = (distance / pitch) × stepsPerRevolution × gearRatio

Example:
  distance = 11.25 inches = 285.75 mm
  pitch = 5 mm
  stepsPerRev = 800
  gearRatio = 1.0
  
  steps = (285.75 / 5) × 800 × 1.0 = 45,720 steps
```

**Rotary Motion**:
```
steps = (angle / 360) × stepsPerRevolution × gearRatio

Example:
  angle = 90 degrees
  stepsPerRev = 800
  gearRatio = 1.0
  
  steps = (90 / 360) × 800 × 1.0 = 200 steps
```

#### Feed Rate to Steps/Second

**Linear Feed Rate**:
```
stepsPerSec = (feedRate / pitch) × stepsPerRevolution × gearRatio

Example:
  feedRate = 100 inches/min = 2540 mm/min = 42.33 mm/sec
  pitch = 5 mm
  stepsPerRev = 800
  gearRatio = 1.0
  
  stepsPerSec = (42.33 / 5) × 800 × 1.0 = 6,773 steps/sec
```

**Rotary Feed Rate (RPM)**:
```
stepsPerSec = (rpm / 60) × stepsPerRevolution × gearRatio

Example:
  rpm = 100
  stepsPerRev = 800
  gearRatio = 1.0
  
  stepsPerSec = (100 / 60) × 800 × 1.0 = 1,333 steps/sec
```

### Fixed-Point Precision

**For Performance**:
- Use Q15 or Q31 fixed-point for conversions
- Maintain precision while avoiding floating-point in ISR
- Store conversion factors as fixed-point constants

**Conversion Factor Storage**:
```cpp
// Calculate once, store as fixed-point
int64_t stepsPerUnitQx = (stepsPerRev * pitchUnitsPerRev) / pitch;
// Use integer math for conversions
int32_t steps = (distanceQx * stepsPerUnitQx) >> FRACT_BITS;
```

### Integration Points

#### MotorDriver Integration

**Option 1: Wrapper Functions** (Recommended)
- Keep existing `Move(int32_t steps)` unchanged
- Add new unit-based functions that convert and call existing functions
- Backward compatible, no breaking changes

**Option 2: Overloaded Functions**
- Add overloaded `Move()` with unit parameters
- More C++-like but requires careful overload resolution

**Option 3: Unit Mode Flag**
- Add unit mode flag to MotorDriver
- When enabled, `Move()` interprets input as physical units
- Less intuitive, but minimal API changes

#### CoordinatedMotionController Integration

**Approach**:
- Add unit conversion layer before arc initialization
- Convert arc parameters (center, radius) from physical units to steps
- Convert feed rate from physical units to steps/sec
- Use existing step-based arc system internally

## API Design Examples

### Basic Setup

```cpp
// Configure motor mechanical parameters
ConnectorM0.SetMechanicalParams(
    800,        // steps per revolution
    5.0,        // pitch (5mm)
    UNIT_MM,    // pitch units
    1.0         // gear ratio (direct drive)
);

// Set feed rate in physical units
ConnectorM0.FeedRateInchesPerMin(100.0);

// Move in physical units
ConnectorM0.MoveInches(11.25, MOVE_TARGET_ABSOLUTE);
```

### Coordinated Arcs with Units

```cpp
// Configure units for coordinated motion
CoordinatedMotionController controller;
controller.Initialize(&ConnectorM0, &ConnectorM1);

// Set mechanical params for both motors
ConnectorM0.SetMechanicalParams(800, 5.0, UNIT_MM);
ConnectorM1.SetMechanicalParams(800, 5.0, UNIT_MM);

// Set feed rate
controller.ArcFeedRateInchesPerMin(100.0);

// Move arc in inches
controller.MoveArcInches(
    0.0,        // centerX (inches)
    0.0,        // centerY (inches)
    10.0,       // radius (inches)
    0.0,        // startAngle (radians)
    M_PI / 2,   // endAngle (radians)
    true        // clockwise
);
```

### Mixed Units

```cpp
// Motor 0: Linear axis (inches)
ConnectorM0.SetMechanicalParams(800, 0.2, UNIT_INCHES);  // 5mm = 0.2 inches

// Motor 1: Rotary axis (degrees)
ConnectorM1.SetMechanicalParams(800, 360.0, UNIT_DEGREES);  // Full rotation

// Move linear axis in inches
ConnectorM0.MoveInches(11.25);

// Move rotary axis in degrees
ConnectorM1.MoveDegrees(90.0);
```

## Implementation Complexity

### Low Complexity Components

1. **UnitConverter Class**: Straightforward conversion functions
2. **MotorMechanicalConfig**: Simple data structure
3. **Wrapper Functions**: Thin wrappers around existing Move()

### Medium Complexity Components

1. **Feed Rate Conversion**: Time unit conversions (min/sec)
2. **Position Tracking**: Convert step positions to physical units
3. **Error Handling**: Validate units and parameters

### High Complexity Components

1. **Coordinated Arc Units**: Convert arc parameters while maintaining precision
2. **Mixed Unit Systems**: Different units for X and Y axes
3. **Unit Persistence**: Store unit preferences across power cycles

## Performance Considerations

### Conversion Overhead

**Distance Conversion**:
- Single floating-point multiply/divide
- ~1-5 µs per conversion
- Negligible compared to move execution time

**Feed Rate Conversion**:
- Calculated once per move command
- No ISR overhead
- Acceptable performance impact

### Memory Requirements

**Per Motor**:
- MotorMechanicalConfig: ~32 bytes
- Conversion factors: ~16 bytes
- **Total**: ~48 bytes per motor

**System-Wide**:
- UnitConverter: ~100 bytes (static)
- **Total**: ~200-300 bytes

### Precision Considerations

**Fixed-Point vs. Floating-Point**:
- Floating-point: Easier, but slower
- Fixed-point: Faster, but more complex
- Recommendation: Use floating-point for conversions (not in ISR)

## Backward Compatibility

### Strategy

**Maintain Full Compatibility**:
- All existing `Move(int32_t steps)` functions unchanged
- New unit-based functions are additions, not replacements
- Users can mix step-based and unit-based APIs

**Example**:
```cpp
// Old code still works
motor.Move(5000);  // Steps

// New code can use units
motor.MoveInches(2.5);  // Inches

// Both work together
motor.Move(1000);  // Still in steps
motor.MoveMM(50.0);  // Now in mm
```

## Testing Requirements

### Unit Tests

1. **Conversion Accuracy**: Verify conversions are correct
2. **Precision**: Test with various step counts and units
3. **Edge Cases**: Zero, negative, very large values

### Integration Tests

1. **Move Execution**: Verify moves execute correctly with units
2. **Feed Rate**: Verify feed rates are applied correctly
3. **Position Tracking**: Verify position reporting in units

### System Tests

1. **Real Hardware**: Test with actual motors and mechanical systems
2. **Accuracy Validation**: Measure actual movement vs. commanded
3. **Performance**: Verify no significant performance degradation

## Example Implementation

### UnitConverter.h

```cpp
namespace ClearCore {

enum UnitType {
    UNIT_STEPS,      // Raw steps (default)
    UNIT_INCHES,     // Inches
    UNIT_MM,         // Millimeters
    UNIT_CM,         // Centimeters
    UNIT_METERS,     // Meters
    UNIT_REVOLUTIONS,// Motor revolutions
    UNIT_DEGREES     // Degrees
};

enum FeedRateUnit {
    FR_UNIT_STEPS_PER_SEC,    // Steps per second (default)
    FR_UNIT_INCHES_PER_MIN,   // Inches per minute
    FR_UNIT_INCHES_PER_SEC,   // Inches per second
    FR_UNIT_MM_PER_MIN,       // Millimeters per minute
    FR_UNIT_MM_PER_SEC,       // Millimeters per second
    FR_UNIT_RPM,              // Revolutions per minute
    FR_UNIT_RPS               // Revolutions per second
};

class UnitConverter {
public:
    // Convert distance to steps
    static int32_t DistanceToSteps(double distance, UnitType unit,
                                  const MotorMechanicalConfig& config);
    
    // Convert steps to distance
    static double StepsToDistance(int32_t steps, UnitType unit,
                                  const MotorMechanicalConfig& config);
    
    // Convert feed rate to steps/sec
    static uint32_t FeedRateToStepsPerSec(double feedRate, FeedRateUnit unit,
                                         const MotorMechanicalConfig& config);
    
    // Convert steps/sec to feed rate
    static double StepsPerSecToFeedRate(uint32_t stepsPerSec, FeedRateUnit unit,
                                       const MotorMechanicalConfig& config);
    
private:
    // Unit conversion constants
    static const double MM_PER_INCH;
    static const double CM_PER_INCH;
    static const double METERS_PER_INCH;
    static const double DEGREES_PER_REV;
};

} // ClearCore namespace
```

### MotorDriver Enhancement

```cpp
class MotorDriver {
private:
    MotorMechanicalConfig m_mechanicalConfig;
    bool m_unitsConfigured;
    UnitType m_defaultUnit;
    FeedRateUnit m_defaultFeedRateUnit;

public:
    // Configure mechanical parameters
    bool SetMechanicalParams(uint32_t stepsPerRev, double pitch,
                            UnitType pitchUnit, double gearRatio = 1.0) {
        m_mechanicalConfig.stepsPerRevolution = stepsPerRev;
        m_mechanicalConfig.pitch = pitch;
        m_mechanicalConfig.pitchUnit = pitchUnit;
        m_mechanicalConfig.gearRatio = gearRatio;
        
        // Calculate conversion factors
        CalculateConversionFactors();
        m_unitsConfigured = true;
        return true;
    }
    
    // Move with units
    bool MoveInches(double distance, MoveTarget target = MOVE_TARGET_REL_END_POSN) {
        if (!m_unitsConfigured) return false;
        int32_t steps = UnitConverter::DistanceToSteps(distance, UNIT_INCHES, m_mechanicalConfig);
        return Move(steps, target);
    }
    
    bool MoveMM(double distance, MoveTarget target = MOVE_TARGET_REL_END_POSN) {
        if (!m_unitsConfigured) return false;
        int32_t steps = UnitConverter::DistanceToSteps(distance, UNIT_MM, m_mechanicalConfig);
        return Move(steps, target);
    }
    
    // Feed rate with units
    void FeedRateInchesPerMin(double feedRate) {
        if (!m_unitsConfigured) return;
        uint32_t stepsPerSec = UnitConverter::FeedRateToStepsPerSec(
            feedRate, FR_UNIT_INCHES_PER_MIN, m_mechanicalConfig);
        VelMax(stepsPerSec);
    }
    
    // Position in units
    double PositionInches() const {
        if (!m_unitsConfigured) return 0.0;
        return UnitConverter::StepsToDistance(PositionRefCommanded(), UNIT_INCHES, m_mechanicalConfig);
    }
    
private:
    void CalculateConversionFactors() {
        // Calculate steps per unit based on configuration
        // Implementation details...
    }
};
```

## Benefits

### User Experience

1. **Intuitive**: Program in familiar units (inches, mm)
2. **Less Error-Prone**: No manual conversion calculations
3. **Easier Setup**: Configure once, use units throughout
4. **G-Code Compatible**: Natural fit for G-code interpreters

### Developer Benefits

1. **Backward Compatible**: Existing code continues to work
2. **Modular**: Unit system is optional, can be disabled
3. **Extensible**: Easy to add new units
4. **Well-Tested**: Conversion logic is isolated and testable

## Challenges and Solutions

### Challenge 1: Precision Loss

**Problem**: Floating-point to integer conversion may lose precision

**Solution**:
- Use fixed-point arithmetic for critical conversions
- Round to nearest step (acceptable for most applications)
- Provide precision warnings for very small moves

### Challenge 2: Performance

**Problem**: Unit conversion adds overhead

**Solution**:
- Conversions done outside ISR (in Move() calls)
- Cache conversion factors
- Use efficient fixed-point math where possible

### Challenge 3: Mixed Units

**Problem**: Different motors may use different units

**Solution**:
- Each motor has its own mechanical configuration
- Coordinated motion converts to common unit (steps) internally
- User can specify units per motor

### Challenge 4: Unit Persistence

**Problem**: Units should persist across power cycles

**Solution**:
- Store configuration in NVM (non-volatile memory)
- Provide save/load functions
- Default to steps if not configured

## Migration Path

### Phase 1: Core Unit System
- Implement UnitConverter class
- Add MotorMechanicalConfig
- Add basic MoveInches(), MoveMM() functions

### Phase 2: Feed Rate Support
- Add feed rate conversion
- Integrate with VelMax()
- Support multiple feed rate units

### Phase 3: Coordinated Motion Integration
- Add unit support to CoordinatedMotionController
- Convert arc parameters
- Test with real hardware

### Phase 4: Advanced Features
- Unit persistence (NVM)
- Mixed unit systems
- G-code integration helpers

## Conclusion

**Yes, the ClearCore library can be enhanced to support units.** The design is:

- ✅ **Feasible**: Straightforward implementation
- ✅ **Backward Compatible**: Existing code continues to work
- ✅ **Performant**: Minimal overhead
- ✅ **Extensible**: Easy to add new units
- ✅ **User-Friendly**: Intuitive API

The enhancement would significantly improve usability while maintaining the library's performance and compatibility characteristics.
