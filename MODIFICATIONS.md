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

# Modifications to Teknic ClearCore Library Files

This document lists all modifications made to files originally provided by Teknic, Inc.
All modifications are by **Adam G. Sweeney <agsweeney@gmail.com>** (2026).

## Modified Files

### 1. `libClearCore/inc/MotorDriver.h`

**Original Copyright**: Copyright (c) 2020 Teknic, Inc.

**Modifications Added**:

#### Coordinated Motion Support:
- Added `friend class CoordinatedMotionController;` declaration
- Added public methods:
  - `CoordinatedMotionMode(bool enable, CoordinatedMotionController* controller)`
  - `IsCoordinatedMode() const`
  - `SetCoordinatedSteps(int32_t steps)`
- Added protected member variables:
  - `bool m_coordinatedMode`
  - `CoordinatedMotionController* m_coordinatedController`
  - `MotorDriver* m_coordinatedMotorX`
  - `MotorDriver* m_coordinatedMotorY`

#### Unit Conversion Support:
- Added `#include "UnitConverter.h"`
- Added public methods:
  - `SetMechanicalParams(uint32_t stepsPerRev, double pitch, UnitType pitchUnit, double gearRatio = 1.0)`
  - `IsUnitsConfigured() const`
  - `MoveInches(double distance, MoveTarget moveTarget = MOVE_TARGET_REL_END_POSN)`
  - `MoveMM(double distance, MoveTarget moveTarget = MOVE_TARGET_REL_END_POSN)`
  - `MoveCM(double distance, MoveTarget moveTarget = MOVE_TARGET_REL_END_POSN)`
  - `MoveMeters(double distance, MoveTarget moveTarget = MOVE_TARGET_REL_END_POSN)`
  - `MoveRevolutions(double revolutions, MoveTarget moveTarget = MOVE_TARGET_REL_END_POSN)`
  - `MoveDegrees(double degrees, MoveTarget moveTarget = MOVE_TARGET_REL_END_POSN)`
  - `MoveWithUnit(double distance, UnitType unit, MoveTarget moveTarget = MOVE_TARGET_REL_END_POSN)`
  - `FeedRateInchesPerMin(double feedRate)`
  - `FeedRateMMPerMin(double feedRate)`
  - `FeedRateMMPerSec(double feedRate)`
  - `FeedRateRPM(double rpm)`
  - `FeedRateWithUnit(double feedRate, FeedRateUnit unit)`
  - `PositionInches() const`
  - `PositionMM() const`
  - `PositionRevolutions() const`
  - `PositionDegrees() const`
  - `PositionWithUnit(UnitType unit) const`
- Added protected member variables:
  - `MotorMechanicalConfig m_mechanicalConfig`
  - `bool m_unitsConfigured`

**Location of Modifications**:
- Lines ~80-86: Friend class declaration
- Lines ~432-472: Coordinated motion methods
- Lines ~474-600: Unit conversion methods
- Lines ~1558-1562: Protected coordinated motion members
- Lines ~1563-1564: Protected unit conversion members

### 2. `libClearCore/src/MotorDriver.cpp`

**Original Copyright**: Copyright (c) 2020 Teknic, Inc.

**Modifications Added**:

#### Coordinated Motion Integration:
- Added `#include "CoordinatedMotionController.h"`
- Modified `Refresh()` method (lines ~471-489):
  - Added check for `m_coordinatedMode`
  - When in coordinated mode, calls `CoordinatedMotionController::UpdateFast()` instead of `StepGenerator::StepsCalculated()`
  - Only executes on X-axis motor to avoid double execution
- Added implementation of `CoordinatedMotionMode()` method (lines ~551-579)
- Added implementation of `SetCoordinatedSteps()` method (lines ~581-620)
- Added constructor initialization of coordinated motion members (lines ~152-155)

#### Unit Conversion Implementation:
- Added `#include "UnitConverter.h"`
- Added constructor initialization of `m_unitsConfigured` (line ~156)
- Added implementation of `SetMechanicalParams()` method (lines ~1133-1149)
- Added implementations of all unit-based Move methods (lines ~1151-1205)
- Added implementations of all unit-based FeedRate methods (lines ~1207-1235)
- Added implementations of all unit-based Position query methods (lines ~1237-1275)

**Location of Modifications**:
- Lines ~24-41: Modification notice header
- Lines ~48: Include CoordinatedMotionController.h
- Lines ~49: Include UnitConverter.h
- Lines ~152-156: Constructor initialization
- Lines ~471-489: Refresh() method modification
- Lines ~551-620: Coordinated motion methods
- Lines ~1131-1275: Unit conversion methods

### 3. `libClearCore/inc/ClearCore.h`

**Original Copyright**: Copyright (c) 2020 Teknic, Inc.

**Modifications Added**:
- Added `#include "UnitConverter.h"` (line ~43)
- Added `#include "LinearInterpolator.h"` (line ~44)

**Note**: `ArcInterpolator.h`, `CoordinatedMotionController.h`, and `TrigLUT.h` were already added in previous modifications.

**Location of Modifications**:
- Lines ~20-28: Modification notice header
- Lines ~43-44: New header includes

## Summary

**Total Modified Files**: 3

**Modification Types**:
1. **Coordinated Motion Integration**: Added support for coordinated motion between two motors
2. **Unit Conversion Support**: Added physical unit conversion (inches, mm, degrees, etc.)
3. **Header Includes**: Added includes for new feature headers

**All Modifications Preserve**:
- Original Teknic copyright and license
- Original file structure and organization
- Backward compatibility with existing code
- Original functionality (additions only, no removals)

## License

All modifications are licensed under the MIT License with copyright attribution to:
**Adam G. Sweeney <agsweeney@gmail.com>**

The original Teknic files remain under their original copyright (Copyright (c) 2020 Teknic, Inc.).
