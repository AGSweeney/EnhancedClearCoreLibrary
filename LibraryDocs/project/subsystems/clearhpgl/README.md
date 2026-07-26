---
title: ClearHPGL
component: clearhpgl
level: project
topics:
  - ClearHPGL
  - DXF
  - HPGL
  - ClearCNC
  - path-repair
source_paths:
  - ClearHPGL/src/MainWindow.cpp
  - ClearHPGL/src/DxfImport.h
  - ClearHPGL/CMakeLists.txt
status: verified
---

# ClearHPGL

Qt 6 ClearCNC-oriented host with DXF import, DXF→HPGL preview/export, and path-gap / issue repair.

**App-only** — not linked to `libClearCore`; speaks ClearCNC line protocol.

## Key modules

| Module | Role |
|--------|------|
| `MainWindow` | Transport, machine, program, DXF UI |
| `DxfImport` | ASCII DXF load |
| `PathGapBridge` / `PathVizIssues` | Repair / markers |
| Preview kinematics / 3D view | Visualization only |

## Related

[clearcnc-firmware](../clearcnc-firmware/README.md), [clearcnc-host](../clearcnc-host/README.md).

## Source evidence

| Claim | Evidence | Level |
|-------|----------|-------|
| Standalone Qt CMake target | `ClearHPGL/CMakeLists.txt` | E1 |
| DXF import | `ClearHPGL/src/DxfImport.h` | E1 |
| Host controls | `ClearHPGL/src/MainWindow.cpp` | E1 |
