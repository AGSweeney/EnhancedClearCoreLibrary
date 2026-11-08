---
title: CoordinatedMotionController
component: coordinated-motion-controller
level: library
reuse: high
platforms:
  - ClearCore SAME53
topics:
  - coordinated-motion
  - QueueArc
  - QueueLinear
  - planner
  - UpdateFast
  - junction
source_paths:
  - libClearCore/inc/CoordinatedMotionController.h
  - libClearCore/src/CoordinatedMotionController.cpp
status: verified
retrieval:
  - How do I initialize CoordinatedMotionController with M0/M1?
  - Which context owns UpdateFast and the motion queue?
  - What breaks when QueueArc/QueueLinear fail or the planner queue is full?
---

# CoordinatedMotionController

High-reuse two-motor coordinated planner for chained linear/arc moves with constant path velocity and junction speed control. **Reuse:** `high`.

## Purpose and reuse

High-reuse two-motor coordinated planner for chained linear/arc moves with constant path velocity and junction speed control.

## Public API surface

| Method | Role |
|--------|------|
| `Initialize(motorX, motorY)` | Bind motors; enable coordinated mode |
| `QueueLinear` / `QueueArc` | Enqueue segments (preferred chaining) |
| `SetDeferMotionQueueStart` / `StartDeferredMotionIfPending` | Batch then start |
| `ArcVelMax` / `ArcAccelMax` | Path velocity / accel |
| `JunctionDVmax` / `JunctionDeviationSteps` | Cornering |
| `UpdateFast` | ISR step generator |
| `PlannerQueueCapacity` | Returns `ARC_QUEUE_SIZE` (16) |
| `SetKinematics` / `Kinematics` | `KINEMATICS_CARTESIAN` (default) or `KINEMATICS_COREXY` (motor 1 = A = X+Y, motor 2 = B = X−Y; also H-bot) |
| `MotorVelMax` | Per-motor speed cap (steps/s, 0 = none); lowers each segment's path speed |
| `SyncPositionFromMotors` | Set X/Y from the motors' commanded positions (after homing/jogging) |

## Kinematics (CoreXY)

Planning, interpolation, positions and units stay Cartesian. `OutputSteps()` maps each sample's X/Y steps through `Kinematics::ToMotor` (`CoordinatedKinematics.h`, no hardware dependencies) before `SetCoordinatedSteps`. Consequences in CoreXY:

- A motor runs up to √2 × path speed (45° moves). `MotorVelMax` caps each queued segment: linear factor `|ux|+|uy|`; arcs use the largest factor over the sweep (`ArcMotorSpeedFactor`). The interrupt-side legacy arc queue (`ProcessNextArc`) uses the worst case √2.
- `JunctionDVmax` is per motor: divisor `|ΔdirX|+|ΔdirY|` (`JunctionMotorDelta`).
- Motor acceleration reaches √2 × `ArcAccelMax`; not capped separately.
- Both axes need the same steps per unit; `SetKinematics(KINEMATICS_COREXY)` returns false if both mechanical configs are set and differ, or if motion is active/queued.
- Per-sample motor steps can be up to 2 × the larger axis step; excess over the per-sample limit is carried by `ApplySampleSteps` (`m_coordCarry`).

Artifact: [A-L01-if](../../artifacts/interfaces/CoordinatedMotionController.h).

Patterns: [A-L01-pat](../../artifacts/patterns/coordinated-initialize.cpp), [A-L01-def](../../artifacts/patterns/defer-motion-queue-start.h).

## Ownership and concurrency

App/firmware thread enqueues. Sample ISR (X motor `Refresh`) owns `UpdateFast` and step output.

## Runtime lifecycle

Construct → `Initialize` → set vel/accel/junction → `Queue*` (optionally deferred) → ISR runs until drain → `Stop`/`StopDecel`.

## Configuration

Velocity/accel, junction deviation/dVmax, `StopAtQueueEnd`, mechanical params X/Y, defer-start flag.

## Limits and capacities

| Limit | Value | Source |
|-------|-------|--------|
| Planner queue | 16 | `ARC_QUEUE_SIZE` |
| Default vel | 5000 steps/s | ctor |
| Default accel | 50000 steps/s² | ctor |
| Sample rate | 5 kHz | SysTiming |

## State and modes

Unified `QueuedMotion` queue; `m_active` / `MotionQueueCount()` for host gating.

## Error handling

`Initialize`/`Queue*`/`Move*` return `false` on failure. ClearCNC clears firmware queue if planner submit fails.

## Thread and ISR safety

Queue fields `volatile` + critical sections. `UpdateFast` is ISR-only.

## Memory footprint

Fixed 16-entry queue + embedded interpolators; no heap.

## Dependencies

`ArcInterpolator`, `LinearInterpolator`, `UnitConverter`, `MotorDriver` (friend).

## Integration points

ClearCNC firmware batches XY; examples under `Microchip_Examples/.../CoordinatedArcMoves`.

## Protocol and wire format

Not applicable — in-process C++ API (no network framing).

## Persistence

Not applicable — no NVM/config ownership in this library.

## Observability

`IsActive`, `MotionQueueCount`, `CurrentX/Y`, `GetDebugInfo`.

## Failure modes and pitfalls

- Cap batches at `PlannerQueueCapacity()` (match firmware `MOTION_QUEUE_SIZE`).
- Pass arc `startAngle` from parsed block (see header comment).
- Use deferred start when batching ([A-L01-def](../../artifacts/patterns/defer-motion-queue-start.h)).
- Idle motors fall back to per-motor step gen (see L06).

## Portability

ClearCore SAME53 / Teknic `libClearCore` build. Fixed-point and sample-ISR assumptions are MCU-specific.

## Testing and verification

Exercised via Microchip examples and ClearCNC firmware motion paths. The controller itself has no host test; its CoreXY maths does: `libClearCore/tests/host/corexy_kinematics_test.cpp` (transform round trip, speed factors, interpolated lines and arcs through the CoreXY output under a motor speed cap).

## Related components

[arc-interpolator](../arc-interpolator/README.md), [linear-interpolator](../linear-interpolator/README.md), [motor-driver](../motor-driver/README.md), [clearcnc-firmware](../../project/subsystems/clearcnc-firmware/README.md).

## Retrieval questions

- How do I initialize CoordinatedMotionController with M0/M1?
- Which context owns UpdateFast and the motion queue?
- What breaks when QueueArc/QueueLinear fail or the planner queue is full?

## Source evidence

| Claim | Evidence | Level |
|-------|----------|-------|
| Public queue/move API | `CoordinatedMotionController.h` L52–161 | E1 |
| `ARC_QUEUE_SIZE = 16` | header L508–511 | E1 |
| Initialize enables coordinated mode | [A-L01-pat](../../artifacts/patterns/coordinated-initialize.cpp) | E1 |
| Deferred batch start | header L134–147 | E1 |
