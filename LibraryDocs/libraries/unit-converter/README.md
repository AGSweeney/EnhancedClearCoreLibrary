---
title: UnitConverter
component: unit-converter
level: library
reuse: high
platforms:
  - ClearCore SAME53
topics:
  - units
  - mm
  - inches
  - feed-rate
  - MotorMechanicalConfig
  - steps
source_paths:
  - libClearCore/inc/UnitConverter.h
  - libClearCore/src/UnitConverter.cpp
status: verified
retrieval:
  - How do I convert millimeters to steps for a ball screw?
  - What fields must MotorMechanicalConfig have to be valid?
  - What breaks when DistanceToSteps is called with invalid config?
---

# UnitConverter

High-reuse static conversion between physical units and step counts via `MotorMechanicalConfig`. **Reuse:** `high`.

## Purpose and reuse

High-reuse static conversion between physical units and step counts via `MotorMechanicalConfig`.

## Public API surface

`UnitType`/`FeedRateUnit`, `MotorMechanicalConfig`, `DistanceToSteps`, `StepsToDistance`, `FeedRateToStepsPerSec`, `CalculateConversionFactors`. [A-L04-if](../../artifacts/interfaces/UnitConverter.h), [A-L04-pat](../../artifacts/patterns/distance-to-steps.cpp).

See [A-L04-if](../../artifacts/interfaces/UnitConverter.h), [A-L04-pat](../../artifacts/patterns/distance-to-steps.cpp).

## Ownership and concurrency

Stateless utility — caller’s thread.

## Runtime lifecycle

Configure mechanical params → call converters as needed.

## Configuration

`stepsPerRevolution`, `pitch`, `pitchUnit`, `gearRatio`, `inverted`.

## Limits and capacities

Invalid config → conversions return 0. Linear base unit is inches.

## State and modes

None (static methods).

## Error handling

Silent `0` on invalid config or negative feed.

## Thread and ISR safety

Pure functions if config not mutated mid-call.

## Memory footprint

Negligible.

## Dependencies

`<math.h>` for `round`.

## Integration points

`MotorDriver` unit APIs; coordinated unit helpers. ClearCNC often converts locally.

## Protocol and wire format

Not applicable — in-process C++ API (no network framing).

## Persistence

Not applicable — no NVM/config ownership in this library.

## Observability

Return values only.

## Failure modes and pitfalls

Do not assume ClearCNC G-code path always calls `DistanceToSteps`.

## Portability

ClearCore SAME53 / Teknic `libClearCore` build. Fixed-point and sample-ISR assumptions are MCU-specific.

## Testing and verification

Exercised via Microchip examples and ClearCNC firmware motion paths. No in-repo unit-test harness for these classes.

## Related components

[motor-driver](../motor-driver/README.md), [coordinated-motion-controller](../coordinated-motion-controller/README.md).

## Retrieval questions

- How do I convert millimeters to steps for a ball screw?
- What fields must MotorMechanicalConfig have to be valid?
- What breaks when DistanceToSteps is called with invalid config?

## Source evidence

| Claim | Evidence | Level |
|-------|----------|-------|
| Enums/config | `UnitConverter.h` L42–101 | E1 |
| DistanceToSteps math | `UnitConverter.cpp` L35–60 | E1 |
| MM_PER_INCH = 25.4 | `UnitConverter.cpp` L29 | E1 |
