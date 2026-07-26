---
title: ClearCore runtime
component: clearcore-runtime
level: platform
topics:
  - sample-rate
  - connectors
  - SysTiming
  - MotorManager
  - ClearCore.h
source_paths:
  - libClearCore/inc/SysTiming.h
  - libClearCore/inc/ClearCore.h
status: verified
---

# ClearCore runtime

Hardware/runtime profile for Teknic ClearCore under enhanced `libClearCore`.

## Sample timing

`_CLEARCORE_SAMPLE_RATE_HZ` = **5000**. Coordinated `UpdateFast` runs from X-motor `Refresh` when the planner is driving.

Artifact: [A-PL02-if](../../artifacts/interfaces/SysTiming_sample_rate.h).

## Connectors

`ClearCore.h` exposes `ConnectorM0`–`M3`, IO/DI, managers, and enhanced motion includes.

## Failure modes

| Symptom | Likely cause | Where documented |
|---------|--------------|------------------|
| Independent Move on M0/M1 stalls while coordinated mode on | Idle path must use StepGenerator fallback | [L06](../../libraries/motor-driver/README.md) |
| Visible hesitation between G-code batches | Planner queue < firmware batch / restart at 0 | [L01](../../libraries/coordinated-motion-controller/README.md) |

## Source evidence

| Claim | Evidence | Level |
|-------|----------|-------|
| 5000 Hz | `SysTiming.h` L42–43 | E1 |
| Enhanced includes | `ClearCore.h` L51–55 | E1 |
