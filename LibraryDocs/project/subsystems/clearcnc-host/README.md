---
title: ClearCNC Qt Host
component: clearcnc-host
level: project
topics:
  - Qt
  - ClearCNC
  - streaming
  - QSettings
  - telemetry
source_paths:
  - ClearCNC_Controller/QtController/src/MainWindow.cpp
  - ClearCNC_Controller/QtController/src/main.cpp
  - ClearCNC_Controller/QtController/CMakeLists.txt
status: verified
---

# ClearCNC Qt Host

Desktop Qt 6 controller for ClearCNC firmware: connection, MDI/jog, G-code streaming, 3D DRO/path, settings.

## Connection flow

Serial or Ethernet control; optional telemetry; UDP discovery. On connect: units + `CONFIG …` + status/telemetry.

Artifact: [A-P02-pat](../../../artifacts/patterns/clearcnc-host-config-sync.cpp).

## Persistence

`QSettings("ClearCNC", "ClearCNC_Controller")` (`MainWindow.cpp` L59). Firmware is not the store.

## Streaming

Ack-driven send; retry on `busy: queue full`; `G4` treated as non-queued dwell.

## Related

[clearcnc-firmware](../clearcnc-firmware/README.md), [host-build](../../../platform/host-build/README.md).

## Source evidence

| Claim | Evidence | Level |
|-------|----------|-------|
| CONFIG SPMMX sync | `MainWindow.cpp` (~CONFIG SPMMX line) | E1 |
| QSettings org/app | `MainWindow.cpp` L59 | E1 |
| Feature/protocol overview | `ClearCNC_Controller/README.md` | E1 |
