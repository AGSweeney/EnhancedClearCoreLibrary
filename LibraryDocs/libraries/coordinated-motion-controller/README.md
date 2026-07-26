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

Exercised via Microchip examples and ClearCNC firmware motion paths. No in-repo unit-test harness for these classes.

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
