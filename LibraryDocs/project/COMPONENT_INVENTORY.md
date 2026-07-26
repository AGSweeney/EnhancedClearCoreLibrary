---
title: Component inventory
level: project
status: verified
inventory_version: 1
repo_root: .
last_updated: 2026-07-26
---

# Component Inventory

## Summary

| Metric | Count |
|--------|-------|
| Libraries | 6 |
| Project subsystems | 4 |
| Platform modules | 3 |
| Artifacts required | 17 |
| Verified (E1/E2) | 13 |
| Inferred/draft | 0 |

## Inventory table

| ID | Name | Level | Folder | Source paths | Reuse | Owner task | Socket/storage | Artifact IDs | Doc status | Evidence |
|----|------|-------|--------|--------------|-------|------------|----------------|--------------|------------|----------|
| L01 | CoordinatedMotionController | library | libraries/coordinated-motion-controller | libClearCore/inc/CoordinatedMotionController.h, libClearCore/src/CoordinatedMotionController.cpp | high | Motor sample ISR / app enqueue | planner queue RAM | A-L01-if, A-L01-pat, A-L01-def | verified | E1 |
| L02 | ArcInterpolator | library | libraries/arc-interpolator | libClearCore/inc/ArcInterpolator.h, libClearCore/src/ArcInterpolator.cpp | medium | CoordinatedMotionController ISR | Q15 segment state | A-L02-if | verified | E1 |
| L03 | LinearInterpolator | library | libraries/linear-interpolator | libClearCore/inc/LinearInterpolator.h, libClearCore/src/LinearInterpolator.cpp | medium | CoordinatedMotionController ISR | path state | A-L03-if | verified | E1 |
| L04 | UnitConverter | library | libraries/unit-converter | libClearCore/inc/UnitConverter.h, libClearCore/src/UnitConverter.cpp | high | caller thread | none | A-L04-if, A-L04-pat | verified | E1 |
| L05 | TrigLUT | library | libraries/trig-lut | libClearCore/inc/TrigLUT.h, libClearCore/src/TrigLUT.cpp, libClearCore/src/TrigLUT_Data.h | medium | ArcInterpolator | 1024-entry ROM table | A-L05-if | verified | E1 |
| L06 | MotorDriver enhancements | library | libraries/motor-driver | libClearCore/inc/MotorDriver.h, libClearCore/src/MotorDriver.cpp | medium | MotorManager sample ISR | step/dir pins | A-L06-if, A-L06-pat | verified | E1 |
| P01 | ClearCNC Firmware | project | project/subsystems/clearcnc-firmware | ClearCNC_Controller/ClearCoreFirmware/ClearCNC_Firmware.cpp | app-only | main loop | USB CDC, TCP 8888/8889, UDP 10040 | A-P01-pat, A-P01-main, A-P01-batch, A-P01-data | verified | E1 |
| P02 | ClearCNC Qt Host | project | project/subsystems/clearcnc-host | ClearCNC_Controller/QtController/src/MainWindow.cpp, ClearCNC_Controller/QtController/src/main.cpp | app-only | Qt GUI thread | serial/TCP; QSettings | A-P02-pat | verified | E1 |
| P03 | ClearHPGL | project | project/subsystems/clearhpgl | ClearHPGL/src/MainWindow.cpp, ClearHPGL/src/DxfImport.h | app-only | Qt GUI thread | serial/TCP; DXF files | — | verified | E1 |
| P04 | NewGControl | project | project/subsystems/newgcontrol | NewGControl/src/MainWindow.cpp, NewGControl/src/GrblProtocol.h | app-only | Qt GUI thread | serial/TCP; JSON snapshots | — | verified | E1 |
| PL01 | Microchip Studio build | platform | platform/build | libClearCore/ClearCore.cppproj, libClearCore/ClearCore.atsln | n/a | — | flash image | A-PL01-bld | verified | E1 |
| PL02 | ClearCore runtime | platform | platform/clearcore-runtime | libClearCore/inc/SysTiming.h, libClearCore/inc/ClearCore.h | n/a | SysManager UpdateFast | connectors M0–M3, I/O | A-PL02-if | verified | E1 |
| PL03 | Host Qt build | platform | platform/host-build | build.ps1, ClearCNC_Controller/QtController/CMakeLists.txt | n/a | — | desktop binary | — | verified | E1 |

### Artifact ID map

| ID | File |
|----|------|
| A-L01-if | interfaces/CoordinatedMotionController.h |
| A-L01-pat | patterns/coordinated-initialize.cpp |
| A-L01-def | patterns/defer-motion-queue-start.h |
| A-L02-if | interfaces/ArcInterpolator.h |
| A-L03-if | interfaces/LinearInterpolator.h |
| A-L04-if | interfaces/UnitConverter.h |
| A-L04-pat | patterns/distance-to-steps.cpp |
| A-L05-if | interfaces/TrigLUT.h |
| A-L06-if | interfaces/MotorDriver_extensions.h |
| A-L06-pat | patterns/coordinated-refresh-fallback.cpp |
| A-P01-pat | patterns/clearcnc-ports-and-queues.cpp |
| A-P01-main | patterns/clearcnc-main-loop.cpp |
| A-P01-batch | patterns/clearcnc-planner-batch.cpp |
| A-P01-data | data/clearcnc-protocol-commands.md |
| A-P02-pat | patterns/clearcnc-host-config-sync.cpp |
| A-PL01-bld | build/libclearcore-enhanced-sources.cppproj.mk |
| A-PL02-if | interfaces/SysTiming_sample_rate.h |

## Excluded (grouped under parent)

| Symbol/file | Parent ID | Reason |
|-------------|-----------|--------|
| TrigLUT_Data.h | L05 | Generated sine table data |
| StepGenerator base | L06 | Upstream Teknic; enhancements under L06 |
| LwIP/ | PL02 | Vendored Ethernet stack |
| Microchip_Examples/* | — | Example apps |
| Depreciated/* | — | Superseded streaming/GRBL examples |
| ProjectTemplate | — | Minimal starter |
| GCodeProgramKinematics | P02/P03 | Host visualization helper |

## Coupling register

| From ID | To ID | Coupling type | Notes |
|---------|-------|---------------|-------|
| L01 | L02 | calls API | Arc path generation in `UpdateFast` |
| L01 | L03 | calls API | Linear path generation in `UpdateFast` |
| L01 | L04 | calls API | Unit helpers when mechanical params set |
| L01 | L06 | calls API | `Initialize` enables coordinated mode |
| L02 | L05 | calls API | `SinQx`/`CosQx` |
| L06 | L01 | global state | `Refresh` → `UpdateFast` when planner driving |
| L06 | L04 | calls API | Unit move/feed helpers |
| P01 | L01 | calls API | XY batch via `QueueLinear`/`QueueArc` |
| P01 | L06 | calls API | Independent Z/A, enable/stop |
| P01 | L04 | include-only | Mechanical configs; most conversion local |
| P02 | P01 | op queue | Line protocol USB/TCP; CONFIG on connect |
| P03 | P01 | op queue | ClearCNC client + DXF/HPGL tooling |
| P04 | P02 | include-only | Shares Qt host patterns; GRBL protocol (not ClearCNC) |
| PL01 | L01 | include-only | Compiles enhanced sources into `libClearCore.a` |
| PL02 | L06 | global state | 5 kHz sample drives `MotorDriver::Refresh` |
| PL03 | P02 | config blob | CMake/vcpkg Qt host build |

## Retrieval keywords

| ID | keywords |
|----|----------|
| L01 | coordinated motion, QueueArc, QueueLinear, planner, UpdateFast, junction |
| L02 | ArcInterpolator, Q15, GenerateNextSteps |
| L03 | LinearInterpolator, path velocity |
| L04 | UnitConverter, MotorMechanicalConfig, DistanceToSteps |
| L05 | TrigLUT, SinQx, CosQx, 1024 |
| L06 | MotorDriver, CoordinatedMotionMode, SetMechanicalParams, Refresh |
| P01 | ClearCNC firmware, G-code, telemetry, DI-6, spindle |
| P02 | ClearCNC Qt, QSettings, streaming, CONFIG |
| P03 | ClearHPGL, DXF, path repair |
| P04 | NewGControl, GRBL, FluidNC, grblHAL |
| PL01 | Microchip Studio, ClearCore.cppproj, SAME53 |
| PL02 | sample rate 5000 Hz, connectors |
| PL03 | build.ps1, CMake, Qt6, vcpkg |
