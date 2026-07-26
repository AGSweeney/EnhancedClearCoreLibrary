---
title: TrigLUT
component: trig-lut
level: library
reuse: medium
platforms:
  - ClearCore SAME53
topics:
  - trigonometry
  - Q15
  - SinQx
  - CosQx
  - lookup-table
source_paths:
  - libClearCore/inc/TrigLUT.h
  - libClearCore/src/TrigLUT.cpp
  - libClearCore/src/TrigLUT_Data.h
status: verified
retrieval:
  - How do I compute sin/cos in Q15 for arc math?
  - What is TRIG_LUT_SIZE?
  - How is cosine derived from the sine table?
---

# TrigLUT

Medium-reuse Q15 sine LUT (1024 entries) with linear interpolation; cosine via π/2 shift. **Reuse:** `medium`.

## Purpose and reuse

Medium-reuse Q15 sine LUT (1024 entries) with linear interpolation; cosine via π/2 shift.

## Public API surface

`TRIG_LUT_SIZE` 1024, `SinQx`, `CosQx`. [A-L05-if](../../artifacts/interfaces/TrigLUT.h).

Interface: [A-L05-if](../../artifacts/interfaces/TrigLUT.h).

## Ownership and concurrency

Called from arc ISR; table read-only.

## Runtime lifecycle

Stateless calls; table linked at build.

## Configuration

Not applicable — fixed table.

## Limits and capacities

1024 entries; frac over 32 Q15 substeps.

## State and modes

None.

## Error handling

Angles masked to 15 bits; no error codes.

## Thread and ISR safety

Re-entrant read-only.

## Memory footprint

~1024×4 bytes ROM.

## Dependencies

Generated `sinLUT` data.

## Integration points

`ArcInterpolator` geometry.

## Protocol and wire format

Not applicable — in-process C++ API (no network framing).

## Persistence

Not applicable — no NVM/config ownership in this library.

## Observability

Not applicable.

## Failure modes and pitfalls

Q15 is not IEEE radians — convert before calling.

## Portability

ClearCore SAME53 / Teknic `libClearCore` build. Fixed-point and sample-ISR assumptions are MCU-specific.

## Testing and verification

Exercised via Microchip examples and ClearCNC firmware motion paths. No in-repo unit-test harness for these classes.

## Related components

[arc-interpolator](../arc-interpolator/README.md).

## Retrieval questions

- How do I compute sin/cos in Q15 for arc math?
- What is TRIG_LUT_SIZE?
- How is cosine derived from the sine table?

## Source evidence

| Claim | Evidence | Level |
|-------|----------|-------|
| TRIG_LUT_SIZE 1024 | `TrigLUT.h` L41 | E1 |
| SinQx interpolation | `TrigLUT.cpp` L39–57 | E1 |
| CosQx = SinQx(+8192) | `TrigLUT.cpp` L65–68 | E1 |
