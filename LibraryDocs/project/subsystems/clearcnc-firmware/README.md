---
title: ClearCNC Firmware
component: clearcnc-firmware
level: project
topics:
  - ClearCNC
  - G-code
  - telemetry
  - estop
  - spindle
source_paths:
  - ClearCNC_Controller/ClearCoreFirmware/ClearCNC_Firmware.cpp
status: verified
---

# ClearCNC Firmware

ClearCore embedded app: line-oriented CNC protocol over USB CDC and Ethernet; XY via `CoordinatedMotionController`.

## Role

Parses commands, maintains a 16-block firmware motion queue, bridges XY into the library planner, runs independent Z/A on `MotorDriver`, exposes spindle / DI-6 estop I/O.

## Transport

Artifacts: [A-P01-pat](../../../artifacts/patterns/clearcnc-ports-and-queues.cpp), [A-P01-data](../../../artifacts/data/clearcnc-protocol-commands.md).

| Channel | Detail |
|---------|--------|
| USB CDC | 115200 |
| Control TCP | **8888** |
| Telemetry TCP | **8889**, ≤50 ms `TEL` |
| Discovery UDP | **10040** |

## Startup and main loop

`main()` opens USB → `InitializeController()` → Ethernet → loop: discovery, telemetry accept, deferred cmds, USB/TCP lines, motion executor, estop poll, telemetry, 1 ms delay.

Artifact: [A-P01-main](../../../artifacts/patterns/clearcnc-main-loop.cpp).

## Motion bridge

When coordinated, `StartMotionBlock` batches consecutive XY segments (capped to `PlannerQueueCapacity()`), deferred-starts multi-segment chains, `QueueLinear`/`QueueArc`.

Artifact: [A-P01-batch](../../../artifacts/patterns/clearcnc-planner-batch.cpp).

## Safety and spindle

DI-6 estop (`CONFIG ESTOP_DI6`); fault runs `M200` path. Spindle: IO-1/IO-2 direction, IO-0 4–20 mA `OutputCurrent`.

## Persistence

Runtime config only — host reapplies `CONFIG` after connect.

## Related

[clearcnc-host](../clearcnc-host/README.md), [system-overview](../../architecture/system-overview.md), [L01](../../../libraries/coordinated-motion-controller/README.md).

## Source evidence

| Claim | Evidence | Level |
|-------|----------|-------|
| Ports / queue 16 | `ClearCNC_Firmware.cpp` L80–91 | E1 |
| Main loop | L3005–3043 | E1 |
| Planner batch | L1663–1711 | E1 |
