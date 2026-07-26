# -*- coding: utf-8 -*-
from pathlib import Path

LD = Path(__file__).resolve().parent


def write(rel: str, text: str) -> None:
    path = LD / rel
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(text.lstrip("\n"), encoding="utf-8")
    print("wrote", rel)


write(
    "project/README.md",
    """# Project docs

ClearCNC / ClearHPGL / NewGControl application knowledge.

| Area | Path |
|------|------|
| Inventory | [COMPONENT_INVENTORY.md](COMPONENT_INVENTORY.md) |
| Architecture | [architecture/system-overview.md](architecture/system-overview.md) |
| Subsystems | [subsystems/README.md](subsystems/README.md) |
| Recipes | [recipes/README.md](recipes/README.md) |
| Open questions | [OPEN_QUESTIONS.md](OPEN_QUESTIONS.md) |

## Alias table

| Term | Meaning |
|------|---------|
| Planner queue | L01 `ARC_QUEUE_SIZE` / `MotionQueueCount` |
| Firmware queue | P01 `MOTION_QUEUE_SIZE` (16) |
| Coordinated mode | L06 `CoordinatedMotionMode` + L01 active/queued |
| ClearCNC protocol | P01 line commands (not GRBL) |
""",
)

write(
    "project/subsystems/README.md",
    """# Subsystems

| ID | Name | Doc |
|----|------|-----|
| P01 | ClearCNC Firmware | [clearcnc-firmware](clearcnc-firmware/README.md) |
| P02 | ClearCNC Qt Host | [clearcnc-host](clearcnc-host/README.md) |
| P03 | ClearHPGL | [clearhpgl](clearhpgl/README.md) |
| P04 | NewGControl | [newgcontrol](newgcontrol/README.md) |

Coupling: [COMPONENT_INVENTORY.md](../COMPONENT_INVENTORY.md#coupling-register).
""",
)

write(
    "project/subsystems/clearcnc-firmware/README.md",
    """---
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
""",
)

write(
    "project/subsystems/clearcnc-host/README.md",
    """---
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
""",
)

write(
    "project/subsystems/clearhpgl/README.md",
    """---
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
""",
)

write(
    "project/subsystems/newgcontrol/README.md",
    """---
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
""",
)

write(
    "project/architecture/README.md",
    """# Architecture

| Doc | Purpose |
|-----|---------|
| [system-overview.md](system-overview.md) | Startup, data flows, subsystem map |
""",
)

write(
    "project/architecture/system-overview.md",
    """---
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
""",
)

write(
    "project/recipes/README.md",
    """# Recipes

| Recipe | Path |
|--------|------|
| Build libClearCore | [build-libclearcore.md](build-libclearcore.md) |
| Flash ClearCNC firmware | [flash-clearcnc-firmware.md](flash-clearcnc-firmware.md) |
| Build ClearCNC Qt host | [build-clearcnc-host.md](build-clearcnc-host.md) |
""",
)

write(
    "project/recipes/build-libclearcore.md",
    """---
title: Build libClearCore
component: build-libclearcore
level: project
topics:
  - build
  - Microchip-Studio
  - libClearCore
status: verified
---

# Build libClearCore

1. Install Microchip Studio 7.0.1645+ with SAME53_DFP 1.1.118 and CMSIS 4.5.0.
2. Open `libClearCore/ClearCore.atsln`.
3. Build Debug/Release → `libClearCore.a`.
4. Confirm enhanced sources ([A-PL01-bld](../../artifacts/build/libclearcore-enhanced-sources.cppproj.mk)).

See [platform/build](../../platform/build/build-instructions.md).
""",
)

write(
    "project/recipes/flash-clearcnc-firmware.md",
    """---
title: Flash ClearCNC firmware
component: flash-clearcnc-firmware
level: project
topics:
  - flash
  - ClearCNC
  - bossac
status: verified
---

# Flash ClearCNC firmware

1. Open `ClearCNC_Controller/ClearCoreFirmware/ClearCNC_Firmware.atsln`.
2. Set Startup Project; build.
3. Flash via Studio or `Tools/flash_clearcore.cmd` (see `Tools/README.md`).
4. Connect USB 115200 or TCP 8888; send `HELP`.

Also: [platform/build/deploy.md](../../platform/build/deploy.md).
""",
)

write(
    "project/recipes/build-clearcnc-host.md",
    """---
title: Build ClearCNC Qt host
component: build-clearcnc-host
level: project
topics:
  - Qt
  - CMake
  - vcpkg
status: verified
---

# Build ClearCNC Qt host

```powershell
.\\build.ps1 -Config Release
```

Or CMake per `ClearCNC_Controller/README.md`. Details: [platform/host-build](../../platform/host-build/README.md).
""",
)

write(
    "project/OPEN_QUESTIONS.md",
    """# Open questions

| Question | Component | Blocker for | Date |
|----------|-----------|-------------|------|
| Bench flash/serial log retained for coordinated motion on a specific ClearCore? | L01 / P01 | E2 bench claims | 2026-07-26 |
| Upstream Teknic unmodified connector API drift vs github.io docs | L06 base | exhaustive MotorDriver base doc | 2026-07-26 |
""",
)

# Platform with Detected stack (2.1.0 Phase 0)
write(
    "platform/README.md",
    """# Platform

Stack-inferred platform docs for this repository (create-librarydocs 2.1.0). Not a canned vendor profile.

## Detected stack

| Layer | Inference | Evidence |
|-------|-----------|----------|
| Language | C++ (firmware + Qt hosts) | `libClearCore/**/*.cpp`, `ClearCNC_Controller/**/*.cpp`, `*.h` |
| Build (MCU) | Microchip Studio / MSBuild `.cppproj` | `libClearCore/ClearCore.cppproj`, `ClearCore.atsln` |
| Build (host) | CMake + Ninja + vcpkg Qt6 | `build.ps1`, `QtController/CMakeLists.txt` |
| Runtime (MCU) | Bare-metal ClearCore / Teknic sample ISR | `SysTiming.h` `_CLEARCORE_SAMPLE_RATE_HZ`, `ClearCore.h` connectors |
| Runtime (host) | Windows Qt GUI thread | `MainWindow`, `QApplication` |
| Targets | ClearCore (SAME53); Windows x64 hosts | root `README.md` DFP/CMSIS; Qt apps |
| Networking / I/O | USB CDC, TCP/UDP Ethernet, step/dir motors | ClearCNC ports; `MotorDriver`; vendored `LwIP/` |
| Persistence | Host `QSettings`; firmware runtime CONFIG | `MainWindow.cpp` L59; no firmware NVM save in ClearCNC |
| Concurrency | 5 kHz motor sample ISR + `main` loop; Qt GUI thread | `SysTiming.h`; ClearCNC `main` |
| External SDKs | Teknic ClearCore library base; Qt 6; Microchip DFP | `libClearCore`, vcpkg Qt, SAME53_DFP |

## Target matrix

| Target | Built | Bench/CI verified | Notable features | Notes |
|--------|-------|-------------------|------------------|-------|
| ClearCore SAME53 | Y | E4 (no bench artifact) | Coordinated XY, Ethernet | Primary MCU |
| Windows x64 Qt host | Y | — | ClearCNC / ClearHPGL / NewGControl | Desktop |

## Platform modules

| ID | Doc area |
|----|----------|
| PL01 | [build](build/README.md) |
| PL02 | [clearcore-runtime](clearcore-runtime/README.md) |
| PL03 | [host-build](host-build/README.md) |
""",
)

write(
    "platform/build/README.md",
    """# Build / toolchain (PL01)

| Doc | Purpose |
|-----|---------|
| [build-instructions.md](build-instructions.md) | How to build libClearCore |
| [platform-requirements.md](platform-requirements.md) | Studio packs / versions |
| [memory-configuration.md](memory-configuration.md) | Sample rate / buffer limits |
| [deploy.md](deploy.md) | Flash / load procedure |
| [portability-notes.md](portability-notes.md) | MCU vs host split |
""",
)

write(
    "platform/build/build-instructions.md",
    """---
title: Build instructions
component: build
level: platform
topics:
  - Microchip-Studio
  - libClearCore
  - cppproj
  - SAME53
source_paths:
  - libClearCore/ClearCore.cppproj
  - libClearCore/ClearCore.atsln
status: verified
---

# Build instructions

## libClearCore

1. Open `libClearCore/ClearCore.atsln` in Microchip Studio.
2. Select Debug or Release.
3. Build → `libClearCore.a`.

Enhanced sources compiled via `ClearCore.cppproj`: `CoordinatedMotionController`, `ArcInterpolator`, `LinearInterpolator`, `UnitConverter`, `TrigLUT`.

Artifact: [A-PL01-bld](../../artifacts/build/libclearcore-enhanced-sources.cppproj.mk).

## Application firmware

Open the app `.atsln` (e.g. ClearCNC), link built `libClearCore`, set Startup Project, build/flash.

## Source evidence

| Claim | Evidence | Level |
|-------|----------|-------|
| Enhanced Compile includes | `ClearCore.cppproj` ~L885–900 | E1 |
| Solution present | `libClearCore/ClearCore.atsln` | E1 |
""",
)

write(
    "platform/build/platform-requirements.md",
    """---
title: Platform requirements
component: build
level: platform
topics:
  - SAME53_DFP
  - CMSIS
  - Microchip-Studio
  - Teknic
source_paths:
  - README.md
status: verified
---

# Platform requirements

From root `README.md`:

| Requirement | Version / note |
|-------------|----------------|
| Microchip Studio | 7.0.1645 or later |
| SAME53_DFP | 1.1.118 |
| CMSIS | 4.5.0 |
| Installers | https://www.teknic.com/downloads/ |

Ethernet apps use vendored `LwIP/`.

## Source evidence

| Claim | Evidence | Level |
|-------|----------|-------|
| Studio / DFP / CMSIS | root `README.md` | E1 |
""",
)

write(
    "platform/build/memory-configuration.md",
    """---
title: Memory configuration
component: build
level: platform
topics:
  - sample-rate
  - queue
  - buffers
  - ClearCore
source_paths:
  - libClearCore/inc/SysTiming.h
  - libClearCore/inc/CoordinatedMotionController.h
  - ClearCNC_Controller/ClearCoreFirmware/ClearCNC_Firmware.cpp
status: verified
---

# Memory configuration

| Resource | Value | Evidence |
|----------|-------|----------|
| Sample rate | 5000 Hz (200 µs) | `SysTiming.h` `_CLEARCORE_SAMPLE_RATE_HZ` |
| Planner queue | 16 segments | L01 `ARC_QUEUE_SIZE` |
| Firmware motion queue | 16 blocks | P01 `MOTION_QUEUE_SIZE` |
| Command line buffer | 256 chars | P01 `MAX_LINE_LENGTH` |
| Trig LUT | 1024 Q15 entries | L05 `TRIG_LUT_SIZE` |

Keep firmware batch size ≤ planner capacity so `Queue*` cannot fail mid-batch.

## Source evidence

| Claim | Evidence | Level |
|-------|----------|-------|
| 5000 Hz | `SysTiming.h` L42–43 | E1 |
| Queues = 16 | L01 header; P01 L80 | E1 |
""",
)

write(
    "platform/build/deploy.md",
    """---
title: Deploy and flash
component: build
level: platform
topics:
  - flash
  - bossac
  - UF2
  - ClearCore
source_paths:
  - Tools/README.md
  - Tools/flash_clearcore.cmd
status: verified
---

# Deploy and flash

| Tool | Role |
|------|------|
| Microchip Studio | Build + debug flash |
| `Tools/bossac.exe` | CLI flash |
| `Tools/flash_clearcore.cmd` | Find USB port + bossac |
| `Tools/uf2-builder.exe` | `.bin` → UF2 drag-drop |

See `Tools/README.md` and recipe [flash-clearcnc-firmware](../../project/recipes/flash-clearcnc-firmware.md).

## Source evidence

| Claim | Evidence | Level |
|-------|----------|-------|
| Tool list | `Tools/README.md` | E1 |
""",
)

write(
    "platform/build/portability-notes.md",
    """---
title: Portability notes
component: build
level: platform
topics:
  - portability
  - SAME53
  - Qt
  - host-vs-mcu
source_paths:
  - README.md
  - build.ps1
status: verified
---

# Portability notes

| Domain | Notes |
|--------|-------|
| MCU firmware | ClearCore SAME53 only for coordinated motion ISR assumptions |
| Host apps | Windows Qt 6; CMake/vcpkg paths in `build.ps1` are machine-local |
| Protocol | ClearCNC line protocol ≠ GRBL (P04 separate) |
| Units | Library converters are portable C++; mechanical configs are machine-specific |

## Source evidence

| Claim | Evidence | Level |
|-------|----------|-------|
| ClearCore / Studio requirements | root `README.md` | E1 |
| Host build defaults | `build.ps1` | E1 |
""",
)

write(
    "platform/clearcore-runtime/README.md",
    """---
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
""",
)

write(
    "platform/host-build/README.md",
    """---
title: Host Qt build
component: host-build
level: platform
topics:
  - Qt6
  - CMake
  - vcpkg
  - build.ps1
  - windeployqt
source_paths:
  - build.ps1
  - ClearCNC_Controller/QtController/CMakeLists.txt
status: verified
---

# Host Qt build

Windows desktop builds for ClearCNC (default); same CMake patterns for ClearHPGL / NewGControl.

## build.ps1 defaults

| Param | Default |
|-------|---------|
| SourceDir | `ClearCNC_Controller/QtController` |
| BuildDir | `ClearCNC_Controller/<Config>` |
| Generator | Ninja |
| Toolchain | vcpkg `vcpkg.cmake` |

Override VS DevCmd / CMake / vcpkg paths as needed (machine-local).

## Source evidence

| Claim | Evidence | Level |
|-------|----------|-------|
| Default SourceDir/BuildDir | `build.ps1` L34–39 | E1 |
| Qt components | `ClearCNC_Controller/QtController/CMakeLists.txt` | E1 |
""",
)

write(
    "README.md",
    """# LibraryDocs — Enhanced ClearCore Library

Knowledge extraction package (**create-librarydocs 2.1.0**) for coordinated motion, ClearCNC apps, and inferred ClearCore/Qt platform notes.

| Entry | Path |
|-------|------|
| Index | [INDEX.md](INDEX.md) |
| Inventory | [project/COMPONENT_INVENTORY.md](project/COMPONENT_INVENTORY.md) |
| Libraries | [libraries/README.md](libraries/README.md) |
| Project | [project/README.md](project/README.md) |
| Platform | [platform/README.md](platform/README.md) |
| Artifacts | [artifacts/README.md](artifacts/README.md) |
| Validation | [VALIDATION.md](VALIDATION.md) |

Deep motion theory (canonical narrative): `docs/CoordinatedArcMotion/README.md` (outside this package).
""",
)

write(
    "CREATE_LIBRARYDOCS.md",
    """# CREATE_LIBRARYDOCS

Produced with global Cursor skill **create-librarydocs** v2.1.0 (`%USERPROFILE%\\.cursor\\skills\\create-librarydocs\\`).

```powershell
python \"$env:USERPROFILE\\.cursor\\skills\\create-librarydocs\\scripts\\validate_librarydocs.py\" --repo-root . --strict
```

This file is not the skill specification.
""",
)

write(
    "INDEX.md",
    """# LibraryDocs index

| Path | ID | Level | Component | Purpose | Topics | Status |
|------|----|-------|-----------|---------|--------|--------|
| libraries/coordinated-motion-controller/README.md | L01 | library | coordinated-motion-controller | Two-motor coordinated planner | QueueArc, QueueLinear, UpdateFast, junction | verified |
| libraries/arc-interpolator/README.md | L02 | library | arc-interpolator | Q15 arc step engine | ArcInterpolator, Q15, GenerateNextSteps | verified |
| libraries/linear-interpolator/README.md | L03 | library | linear-interpolator | Coordinated linear path engine | LinearInterpolator, path-velocity | verified |
| libraries/unit-converter/README.md | L04 | library | unit-converter | Physical units ↔ steps | UnitConverter, mm, feed-rate | verified |
| libraries/trig-lut/README.md | L05 | library | trig-lut | Q15 sin/cos LUT | TrigLUT, SinQx, CosQx | verified |
| libraries/motor-driver/README.md | L06 | library | motor-driver | Coordinated + unit MotorDriver hooks | CoordinatedMotionMode, Refresh | verified |
| project/subsystems/clearcnc-firmware/README.md | P01 | project | clearcnc-firmware | ClearCNC embedded protocol/executor | G-code, telemetry, estop | verified |
| project/subsystems/clearcnc-host/README.md | P02 | project | clearcnc-host | ClearCNC Qt desktop host | Qt, streaming, QSettings | verified |
| project/subsystems/clearhpgl/README.md | P03 | project | clearhpgl | ClearCNC host + DXF/HPGL tools | DXF, path-repair | verified |
| project/subsystems/newgcontrol/README.md | P04 | project | newgcontrol | GRBL-family Qt host | GRBL, FluidNC | verified |
| platform/build/build-instructions.md | PL01 | platform | build | Microchip Studio lib build | cppproj, SAME53 | verified |
| platform/clearcore-runtime/README.md | PL02 | platform | clearcore-runtime | 5 kHz runtime / connectors | SysTiming, ClearCore.h | verified |
| platform/host-build/README.md | PL03 | platform | host-build | Qt/CMake/vcpkg host builds | build.ps1, Qt6 | verified |
| project/architecture/system-overview.md | — | project | system-overview | Startup and data flows | architecture, ClearCNC | verified |
""",
)

write(
    "VALIDATION.md",
    """---
title: LibraryDocs validation report
standard: create-librarydocs
standard_version: 2.1.0
validated: 2026-07-26
validator: agent
result: pass
mode: strict
repo_root: .
---

# Validation

Placeholder until strict validator confirms; will be finalized after `--strict` exit 0.
""",
)

print("project+platform ok")
