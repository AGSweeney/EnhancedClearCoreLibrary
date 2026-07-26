---
title: MotorDriver enhancements
component: motor-driver
level: library
reuse: medium
platforms:
  - ClearCore SAME53
topics:
  - MotorDriver
  - CoordinatedMotionMode
  - SetMechanicalParams
  - Refresh
  - step-dir
source_paths:
  - libClearCore/inc/MotorDriver.h
  - libClearCore/src/MotorDriver.cpp
status: verified
retrieval:
  - How do I enable coordinated motion mode on ConnectorM0/M1?
  - When does Refresh call UpdateFast versus StepsCalculated?
  - How do I move in millimeters on a single motor?
---

# MotorDriver enhancements

Medium-reuse Teknic `MotorDriver` extensions: coordinated mode hooks and physical-unit move/feed APIs. **Reuse:** `medium`.

## Purpose and reuse

Medium-reuse Teknic `MotorDriver` extensions: coordinated mode hooks and physical-unit move/feed APIs.

## Public API surface

`CoordinatedMotionMode`, `SetCoordinatedSteps`, `SetMechanicalParams`, `MoveMM`/`MoveInches`/…, `FeedRate*`. [A-L06-if](../../artifacts/interfaces/MotorDriver_extensions.h).

[A-L06-if](../../artifacts/interfaces/MotorDriver_extensions.h), [A-L06-pat](../../artifacts/patterns/coordinated-refresh-fallback.cpp).

## Ownership and concurrency

MotorManager sample ISR calls `Refresh` per connector.

## Runtime lifecycle

Mode/unit setup at init; coordinated mode typically left enabled after controller `Initialize`.

## Configuration

Mechanical params; coordinated controller pointer.

## Limits and capacities

Unit APIs require `IsUnitsConfigured()`.

## State and modes

`m_coordinatedMode`, controller/peer pointers, `m_mechanicalConfig`.

## Error handling

Bool returns on mode/move methods.

## Thread and ISR safety

`Refresh` ISR; `UpdateFast` only on X when planner active/queued.

## Memory footprint

Small added members per instance.

## Dependencies

`CoordinatedMotionController`, `UnitConverter`, `StepGenerator`.

## Integration points

ClearCNC maps axes to `ConnectorM0`–`M3`.

## Protocol and wire format

Not applicable — in-process C++ API (no network framing).

## Persistence

Not applicable — no NVM/config ownership in this library.

## Observability

Teknic status/alert registers; `IsCoordinatedMode`.

## Failure modes and pitfalls

Idle coordinated connectors must fall back to `StepsCalculated` — [A-L06-pat](../../artifacts/patterns/coordinated-refresh-fallback.cpp).

## Portability

ClearCore SAME53 / Teknic `libClearCore` build. Fixed-point and sample-ISR assumptions are MCU-specific.

## Testing and verification

Exercised via Microchip examples and ClearCNC firmware motion paths. No in-repo unit-test harness for these classes.

## Related components

[coordinated-motion-controller](../coordinated-motion-controller/README.md), [unit-converter](../unit-converter/README.md), [clearcore-runtime](../../platform/clearcore-runtime/README.md).

## Retrieval questions

- How do I enable coordinated motion mode on ConnectorM0/M1?
- When does Refresh call UpdateFast versus StepsCalculated?
- How do I move in millimeters on a single motor?

## Source evidence

| Claim | Evidence | Level |
|-------|----------|-------|
| CoordinatedMotionMode API | `MotorDriver.h` L457–477 | E1 |
| SetMechanicalParams | L498–522 | E1 |
| coordDriving fallback | `MotorDriver.cpp` L485–499 | E1 |
