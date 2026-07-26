---
title: LinearInterpolator
component: linear-interpolator
level: library
reuse: medium
platforms:
  - ClearCore SAME53
topics:
  - linear
  - interpolation
  - coordinated
  - path-velocity
  - GenerateNextSteps
source_paths:
  - libClearCore/inc/LinearInterpolator.h
  - libClearCore/src/LinearInterpolator.cpp
status: verified
retrieval:
  - How do I start a coordinated linear segment?
  - When is a linear move considered complete?
  - What sample rate should I pass to InitializeLinear?
---

# LinearInterpolator

Medium-reuse straight-line interpolator with constant velocity along the XY path. **Reuse:** `medium`.

## Purpose and reuse

Medium-reuse straight-line interpolator with constant velocity along the XY path.

## Public API surface

`InitializeLinear`, `GenerateNextSteps`, `IsLinearComplete` (position-primary). [A-L03-if](../../artifacts/interfaces/LinearInterpolator.h).

Interface: [A-L03-if](../../artifacts/interfaces/LinearInterpolator.h).

## Ownership and concurrency

Owned by `CoordinatedMotionController` on ISR path.

## Runtime lifecycle

Initialize → GenerateNextSteps until at target → pop segment.

## Configuration

Path vel/accel, sample rate (typically 5000 Hz), entry/exit speeds.

## Limits and capacities

Integer step endpoints; completion requires current == end.

## State and modes

Current/end positions, remaining estimate, ramp.

## Error handling

Bool returns on init/generate.

## Thread and ISR safety

ISR via controller.

## Memory footprint

Fixed members only.

## Dependencies

ClearCore namespace / stdint.

## Integration points

Controller unified queue linear entries.

## Protocol and wire format

Not applicable — in-process C++ API (no network framing).

## Persistence

Not applicable — no NVM/config ownership in this library.

## Observability

Current position getters.

## Failure modes and pitfalls

`IsLinearComplete` trusts position equality over step-count estimate.

## Portability

ClearCore SAME53 / Teknic `libClearCore` build. Fixed-point and sample-ISR assumptions are MCU-specific.

## Testing and verification

Exercised via Microchip examples and ClearCNC firmware motion paths. No in-repo unit-test harness for these classes.

## Related components

[coordinated-motion-controller](../coordinated-motion-controller/README.md).

## Retrieval questions

- How do I start a coordinated linear segment?
- When is a linear move considered complete?
- What sample rate should I pass to InitializeLinear?

## Source evidence

| Claim | Evidence | Level |
|-------|----------|-------|
| InitializeLinear | `LinearInterpolator.h` L64–69 | E1 |
| Position-primary completion | `IsLinearComplete` L86–94 | E1 |
| Excerpt | [A-L03-if](../../artifacts/interfaces/LinearInterpolator.h) | E1 |
