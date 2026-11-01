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

### 6. USB serial receive and linear interpolation fixes (from ClearHPGL, 2026-10-31)

**Files**:
- `libClearCore/hpl/usb/hpl_usb.c` — `_usb_d_dev_ep_init()` records the endpoint direction (`flags.bits.dir`).
- `libClearCore/src/UsbManager.cpp` / `libClearCore/inc/UsbManager.h` — receive stays armed when DTR drops; a zero-length completion re-arms the read; all reads go through `ArmRead()`; receive counters and endpoint state in `RxDebug()`.
- `libClearCore/src/SerialUsb.cpp` / `libClearCore/inc/SerialUsb.h` — `RxDebugLine()` formats `RxDebug()` for a console.
- New-file change (see NEW_FILES.md): `LinearInterpolator` tracks progress along the segment exactly.

**Why**:
- **USB input stopping part-way through a transfer.** The CDC bulk IN (0x81) and bulk OUT (0x01) endpoints share endpoint number 1. Endpoint init cleared the direction bit, which only the first transfer set, so until the board transmitted, the IN endpoint looked like an OUT endpoint. The interrupt handler runs every endpoint instance on that number: when an OUT packet arrived with a stale TRFAIL0 pending (after NAKs during back-pressure), the real OUT endpoint handled TRFAIL0 only and the idle IN instance took the packet's TRCPT0, acknowledged it and dropped the data (not busy). The read never completed and every later packet was NAKed. Hosts that receive replies (DTR high) never hit it; `copy /b file COMn` (DTR low, nothing sent back) stalled at a different point each run. Found with a driver-level event trace.
- **No USB input at all with DTR low.** Receive was stopped whenever DTR dropped and restarted only on DTR high, so after any client closed the port, or for a host that writes without DTR (Windows `copy`), input stopped for good. Transmit still requires DTR; the 1200-baud bootloader touch is unchanged.
- **Zero-length packets.** The read buffer is one 64-byte packet; a host ends a transfer that is a multiple of 64 bytes with a zero-length packet, which completed the read with nothing to copy and never re-armed it.
- **Motor fault on slow, nearly axis-aligned moves.** Each axis advanced by a per-sample increment rounded to Q15 and a segment ended only when both axes passed their end. On the minor axis of a nearly axis-aligned move at low speed the increment is a few Q15 units (2 instead of 2.4 for a 2-step X over an 8800-step Y at 20 mm/s), so it lagged, the major axis overran its end (1685 steps), and the end clamp moved it back in one 200 µs sample — a step burst the ClearPath faulted on. Progress along the path is now accumulated in double precision and both axes are computed from the start point every sample; the segment ends exactly when the progress reaches its length. Checked with the failing segment and 20,000 random segments at 5–300 mm/s (old: 1685-step overshoot, 207 segments not ending on target; new: no overshoot, never above the commanded speed, all on target).

### 7. Arc timing in the step interrupt, queueing with interrupts enabled, TCP accept (from ClearHPGL, 2026-11-01)

**Files**:
- `libClearCore/src/EthernetTcpServer.cpp` — `Accept()` keeps a client that has closed but still holds unread data.
- New-file changes (see NEW_FILES.md): `ArcInterpolator` takes circle points and ramp speeds in single precision; `CoordinatedMotionController` builds queue blocks with interrupts enabled (`QueueStartPoint()` / `CommitQueuedMotion()`) and its timing probe measures each `UpdateFast()` (`MaxUpdateCycles()`, `MaxUpdateKind()`, `SlowUpdates()`).

**Why**:
- **Motor jumps on arcs.** `UpdateFast()` runs in the 5 kHz TCC0 interrupt (200 µs period). Starting an arc there took up to 185 µs: `ArcInterpolator` used double-precision `cos`/`sin`/`sqrt`/`fmod`, which are software on the Cortex-M4F. An overrun misses the step compare and the drive gets a burst of pulses that is neither commanded nor counted (the same mechanism as section 5). The circle point and the accel/decel limits now use `cosf`/`sinf`/`sqrtf` (hardware FPU); the sweep still accumulates in double and the last sample lands on the exact end point. Measured on a ClearPath plotter running a 545-arc job: longest `UpdateFast()` 93 µs (was 185 µs), none over 150 µs, position equal to pulses sent, no jumps. Host arc test unchanged (end error 0, worst radial error 1.69 steps, no bursts).
- **Interrupts off while queueing arcs.** `QueueArc()` computed its angles and tangents in double precision with interrupts disabled: up to 190 µs. Only the main thread adds to the motion queue and the step interrupt only takes from its head, so the block is now built with interrupts enabled; they are off only to read the start point and to publish the block (measured under 1 µs).
- **TCP jobs lost.** `Accept()` freed any client that was no longer connected, so a host that connected, sent a whole job and closed before the next `Accept()` (the application busy, e.g. writing NVM) lost the data. A closed client with unread data is now returned and can be read out.
- **Timing probe.** The gap probe counted pauses where the controller was not updated at all (e.g. stopped between moves); gaps now count only while motion runs on from the previous call. `UpdateFast()` records its longest run, whether it was an arc and whether it started the next move, and how many runs exceeded 150 µs.

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
