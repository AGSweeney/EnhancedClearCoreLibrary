---
title: System overview
component: system-overview
level: project
topics:
  - architecture
  - ClearCNC
  - coordinated-motion
  - data-flow
  - startup
source_paths:
  - README.md
  - ClearCNC_Controller/README.md
  - libClearCore/inc/CoordinatedMotionController.h
status: verified
---

# System overview

Enhanced ClearCore Library: Teknic `libClearCore` (SAME53) plus coordinated XY / units, plus desktop CNC hosts.

## Subsystem map

| ID | Role |
|----|------|
| L01–L06 | Motion/unit libraries in `libClearCore` |
| P01 | ClearCNC firmware |
| P02 | ClearCNC Qt host |
| P03 | ClearHPGL (ClearCNC + DXF/HPGL) |
| P04 | NewGControl (GRBL-family; separate protocol) |
| PL01–PL03 | Microchip build, ClearCore runtime, Qt host build |

## Startup order (ClearCNC firmware)

1. Delay → USB CDC @ 115200 (optional 5 s wait).
2. `InitializeController` (motors, DI-6, spindle).
3. `InitializeEthernet` if link present.
4. Loop: discovery → telemetry accept → deferred cmds → USB/TCP commands → motion executor → estop → telemetry → 1 ms delay.

Canonical: [clearcnc-firmware](../subsystems/clearcnc-firmware/README.md).

## Data flows

```text
Qt host (P02/P03) --USB/TCP lines--> ClearCNC firmware (P01)
        ^                                |
        | TEL / status                   v
        +--------------------- CoordinatedMotionController (L01)
                                         |
                                         +--> MotorDriver M0/M1 (L06)
```

P04 uses GRBL realtime/`$` protocol to external controllers — not this ClearCNC path.

## Configuration persistence

| Store | Owner |
|-------|-------|
| `QSettings("ClearCNC", …)` | P02 |
| Runtime `CONFIG` only | P01 (reapplied on connect) |

## Source evidence

| Claim | Evidence | Level |
|-------|----------|-------|
| Product split | root `README.md` | E1 |
| Firmware main loop | `ClearCNC_Firmware.cpp` L3005–3043 | E1 |
| Matching queue sizes 16 | L01 `ARC_QUEUE_SIZE`; P01 `MOTION_QUEUE_SIZE` | E1 |
