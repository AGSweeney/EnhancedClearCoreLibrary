---
title: NewGControl
component: newgcontrol
level: project
topics:
  - GRBL
  - FluidNC
  - grblHAL
  - Qt
  - streaming
source_paths:
  - NewGControl/src/MainWindow.cpp
  - NewGControl/src/GrblProtocol.h
  - NewGControl/README.md
  - NewGControl/docs/GRBL_COMPATIBILITY_MATRIX.md
status: verified
---

# NewGControl

Standalone Qt 6 GRBL-family controller. **Not** coupled to ClearCNC’s custom protocol.

## Modules

| Module | Role |
|--------|------|
| `GrblProtocol` | Profile detect / normalize |
| `ProgramRunner` | Ack tracking / drain |
| `GrblSnapshotStore` | JSON snapshots |
| `MainWindow` | UI + transport |

Compatibility: `NewGControl/docs/GRBL_COMPATIBILITY_MATRIX.md`.

## Related

[system-overview](../../architecture/system-overview.md). Deprecated GRBL firmware example: `Depreciated/GRBLCompatibleExample/` (excluded).

## Source evidence

| Claim | Evidence | Level |
|-------|----------|-------|
| Purpose | `NewGControl/README.md` | E1 |
| Protocol component | `NewGControl/src/GrblProtocol.h` | E1 |
| Compatibility matrix | `NewGControl/docs/GRBL_COMPATIBILITY_MATRIX.md` | E1 |
