---
title: ArcInterpolator
component: arc-interpolator
level: library
reuse: medium
platforms:
  - ClearCore SAME53
topics:
  - arc
  - Q15
  - interpolation
  - GenerateNextSteps
  - fixed-point
source_paths:
  - libClearCore/inc/ArcInterpolator.h
  - libClearCore/src/ArcInterpolator.cpp
status: verified
retrieval:
  - How do I initialize an arc segment for coordinated motors?
  - Who calls GenerateNextSteps?
  - What happens if remaining angle wraps past endAngle?
---

# ArcInterpolator

Medium-reuse fixed-point arc engine that emits per-sample XY step pairs. **Reuse:** `medium`.

## Purpose and reuse

Medium-reuse fixed-point arc engine that emits per-sample XY step pairs.

## Public API surface

`InitializeArc`, `GenerateNextSteps`, `SetExitSpeed`. `ArcSegment` holds Q15 geometry and `spanQx` clamp. [A-L02-if](../../artifacts/interfaces/ArcInterpolator.h).

Interface: [A-L02-if](../../artifacts/interfaces/ArcInterpolator.h).

## Ownership and concurrency

Owned by `CoordinatedMotionController` on the ISR path.

## Runtime lifecycle

`InitializeArc` → `GenerateNextSteps` until complete → controller advances queue.

## Configuration

velocityMax, accelMax, sampleRateHz, entry/exit speeds.

## Limits and capacities

Q15 angles 0–32768 ≡ 0–2π; `spanQx` prevents full-circle wrap on overshoot.

## State and modes

`ArcSegment` + ramp/position state.

## Error handling

`InitializeArc` false on bad params; `GenerateNextSteps` false when complete.

## Thread and ISR safety

ISR-only when driven by controller.

## Memory footprint

Single active segment; no dynamic allocation.

## Dependencies

`TrigLUT` (`SinQx`/`CosQx`).

## Integration points

`CoordinatedMotionController::UpdateFast` for arc motion type.

## Protocol and wire format

Not applicable — in-process C++ API (no network framing).

## Persistence

Not applicable — no NVM/config ownership in this library.

## Observability

Steps-remaining / position getters.

## Failure modes and pitfalls

Do not treat step-count estimates as completion — use geometry checks.

## Portability

ClearCore SAME53 / Teknic `libClearCore` build. Fixed-point and sample-ISR assumptions are MCU-specific.

## Testing and verification

Exercised via Microchip examples and ClearCNC firmware motion paths. No in-repo unit-test harness for these classes.

## Related components

[coordinated-motion-controller](../coordinated-motion-controller/README.md), [trig-lut](../trig-lut/README.md).

## Retrieval questions

- How do I initialize an arc segment for coordinated motors?
- Who calls GenerateNextSteps?
- What happens if remaining angle wraps past endAngle?

## Source evidence

| Claim | Evidence | Level |
|-------|----------|-------|
| ArcSegment Q15 + spanQx | `ArcInterpolator.h` L43–66 | E1 |
| InitializeArc / GenerateNextSteps | L96–120 | E1 |
| Interface excerpt | [A-L02-if](../../artifacts/interfaces/ArcInterpolator.h) | E1 |
