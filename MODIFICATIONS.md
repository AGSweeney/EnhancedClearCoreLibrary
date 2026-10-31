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

### 4. `CPM_MODE_QUAD_AB` Motion Mode

**Files**:
- `libClearCore/inc/Connector.h` — added `CPM_MODE_QUAD_AB` ([18]) at end of `ConnectorModes`
- `libClearCore/inc/MotorDriver.h` / `libClearCore/src/MotorDriver.cpp` — mode setup, `OutputQuadratureSteps()`, `UpdateFast()` / `SetCoordinatedSteps()` routing, polarity/enable/HLFB guards
- `libClearCore/inc/MotorManager.h` / `libClearCore/src/MotorManager.cpp` — `MotorModeSet()` whitelist and carrier-pin enable for QUAD_AB
- `libClearCore/inc-doc/MotorDriver.dox`, `libClearCore/inc-doc/MoveGen.dox` — documentation
- `Microchip_Examples/ClearPathModeExamples/ClearPath-SD_Series/MoveQuadrature/` — example sketch

**Behavior**:
- Same `Move()` / `MoveVelocity()` / coordinated APIs as step-and-dir
- Each commanded step emits one full **ABAB** (forward) or **BABA** (reverse) cycle, returning to idle
- Edges are paced by a state machine: **one edge per sample** (~200 µs at 5 kHz), no ISR busy-wait — avoids burst/gap trains that trip ClearPath "Step input timing error"
- Prefer `MotorManager::CLOCK_RATE_NORMAL`; practical max QUAD rate ≈ sample rate / 4 (~1250 counts/s). Wait on `QuadratureOutputIdle()` after `StepsComplete()`.

### 5. Coordinated step output and TCP receive fixes (from ClearHPGL, 2026-10-30)

**Files**:
- `libClearCore/inc/MotorDriver.h` / `libClearCore/src/MotorDriver.cpp` — `SetCoordinatedSteps()` carries steps above the per-sample limit into following samples (`m_coordCarry`) instead of discarding them; `CoordinatedExcessSteps()` reports the total.
- `libClearCore/inc/EthernetTcp.h` — `TCP_DATA_BUFFER_SIZE` 600 → 6144 (holds a full `TCP_WND`).
- `libClearCore/src/EthernetTcp.cpp` / `libClearCore/src/EthernetTcpClient.cpp` — `tcp_recved()` moved from the receive callback to `EthernetTcpClient::Read()`.
- New-file changes (see NEW_FILES.md): `CoordinatedMotionController` (`PlanQueue()`, burst log, timing probe) and `LinearInterpolator` (`DebugState()`).

**Why**:
- **Extra step bursts on small arcs.** `QueueLinear()` / `QueueArc()` ran the double-precision planner with interrupts disabled (measured up to 204 µs, longer than one 200 µs TCC0 period). The TCC0 period interrupt that writes each motor's step count (CCBUF) was held off, the write straddled the period boundary, the compare was missed and the step gate stayed open for a full period: ~100 pulses (1/8 rev) in 0.2 ms that the controller neither commanded nor counted. `PlanQueue()` now plans a snapshot with interrupts enabled and commits only the speeds with interrupts off (measured after: 40 µs max IRQ-off, 207 µs max update gap, no extra pulses; a 12-circle job returned to within 8 encoder counts, same as squares).
- **TCP connections stalling.** Each connection buffered 600 bytes and refused any segment that did not fit; a full 1460-byte segment never fits, so bulk transfers stalled permanently. The buffer now holds a full receive window and the window reopens only as the application reads, so TCP flow control applies.

## Summary

**Total Modified Files**: 3 (+ QUAD_AB additions across Connector/MotorDriver/MotorManager/docs/example)

**Modification Types**:
1. **Coordinated Motion Integration**: Added support for coordinated motion between two motors
2. **Unit Conversion Support**: Added physical unit conversion (inches, mm, degrees, etc.)
3. **Header Includes**: Added includes for new feature headers
4. **Quadrature AB Motion Mode**: Added `CPM_MODE_QUAD_AB` StepGenerator output mode

**All Modifications Preserve**:
- Original Teknic copyright and license
- Original file structure and organization
- Backward compatibility with existing code
- Original functionality (additions only, no removals)

## License

All modifications are licensed under the MIT License with copyright attribution to:
**Adam G. Sweeney <agsweeney@gmail.com>**

The original Teknic files remain under their original copyright (Copyright (c) 2020 Teknic, Inc.).
