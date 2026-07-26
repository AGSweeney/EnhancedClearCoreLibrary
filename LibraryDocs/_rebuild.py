# -*- coding: utf-8 -*-
"""Rebuild LibraryDocs for create-librarydocs 2.1.0."""
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
LD = ROOT / "LibraryDocs"


def write(rel: str, text: str) -> None:
    path = LD / rel
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(text.lstrip("\n"), encoding="utf-8")
    print("wrote", rel)


def excerpt(src: Path, start: int, end: int, out: Path, symbol: str) -> None:
    lines = src.read_text(encoding="utf-8", errors="replace").splitlines()
    chunk = lines[start - 1 : end]
    rel = src.relative_to(ROOT).as_posix()
    text = "\n".join(
        [
            f"// EXCERPT — source: {rel}",
            f"// EVIDENCE: E1 | symbol: {symbol} | lines: {start}-{end}",
            "",
            *chunk,
            "",
        ]
    )
    if text.count("\n") > 120:
        raise SystemExit(f"Too long: {out} ({text.count(chr(10))} lines)")
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(text, encoding="utf-8")
    print("artifact", out.relative_to(LD))


# --- Artifacts ---
art = LD / "artifacts"
excerpt(
    ROOT / "libClearCore/inc/CoordinatedMotionController.h",
    52,
    161,
    art / "interfaces/CoordinatedMotionController.h",
    "CoordinatedMotionController",
)
excerpt(
    ROOT / "libClearCore/src/CoordinatedMotionController.cpp",
    140,
    163,
    art / "patterns/coordinated-initialize.cpp",
    "CoordinatedMotionController::Initialize",
)
excerpt(
    ROOT / "libClearCore/inc/CoordinatedMotionController.h",
    134,
    147,
    art / "patterns/defer-motion-queue-start.h",
    "SetDeferMotionQueueStart",
)
excerpt(
    ROOT / "libClearCore/inc/ArcInterpolator.h",
    43,
    120,
    art / "interfaces/ArcInterpolator.h",
    "ArcInterpolator",
)
excerpt(
    ROOT / "libClearCore/inc/LinearInterpolator.h",
    45,
    94,
    art / "interfaces/LinearInterpolator.h",
    "LinearInterpolator",
)
excerpt(
    ROOT / "libClearCore/inc/UnitConverter.h",
    42,
    131,
    art / "interfaces/UnitConverter.h",
    "UnitConverter",
)
excerpt(
    ROOT / "libClearCore/src/UnitConverter.cpp",
    35,
    60,
    art / "patterns/distance-to-steps.cpp",
    "UnitConverter::DistanceToSteps",
)
excerpt(
    ROOT / "libClearCore/inc/TrigLUT.h",
    37,
    59,
    art / "interfaces/TrigLUT.h",
    "SinQx",
)
excerpt(
    ROOT / "libClearCore/inc/MotorDriver.h",
    457,
    522,
    art / "interfaces/MotorDriver_extensions.h",
    "CoordinatedMotionMode",
)
excerpt(
    ROOT / "libClearCore/src/MotorDriver.cpp",
    482,
    510,
    art / "patterns/coordinated-refresh-fallback.cpp",
    "MotorDriver::Refresh coordDriving",
)
excerpt(
    ROOT / "libClearCore/inc/SysTiming.h",
    42,
    57,
    art / "interfaces/SysTiming_sample_rate.h",
    "_CLEARCORE_SAMPLE_RATE_HZ",
)
excerpt(
    ROOT / "ClearCNC_Controller/ClearCoreFirmware/ClearCNC_Firmware.cpp",
    64,
    96,
    art / "patterns/clearcnc-ports-and-queues.cpp",
    "MOTION_QUEUE_SIZE / ETHERNET ports",
)
excerpt(
    ROOT / "ClearCNC_Controller/ClearCoreFirmware/ClearCNC_Firmware.cpp",
    3005,
    3043,
    art / "patterns/clearcnc-main-loop.cpp",
    "main",
)
excerpt(
    ROOT / "ClearCNC_Controller/ClearCoreFirmware/ClearCNC_Firmware.cpp",
    1663,
    1711,
    art / "patterns/clearcnc-planner-batch.cpp",
    "StartMotionBlock coordinated batch",
)

cppproj = ROOT / "libClearCore/ClearCore.cppproj"
cplines = cppproj.read_text(encoding="utf-8", errors="replace").splitlines()
block = []
for i, line in enumerate(cplines, 1):
    if any(
        x in line
        for x in (
            "CoordinatedMotionController.cpp",
            "UnitConverter.cpp",
            "LinearInterpolator.cpp",
            "ArcInterpolator.cpp",
            "TrigLUT.cpp",
        )
    ):
        block.extend(cplines[i - 1 : i + 2])
(art / "build/libclearcore-enhanced-sources.cppproj.mk").write_text(
    "\n".join(
        [
            "// EXCERPT — source: libClearCore/ClearCore.cppproj",
            "// EVIDENCE: E1 | symbol: Compile Include enhanced motion sources | lines: ~885-900",
            "",
            *block,
            "",
        ]
    ),
    encoding="utf-8",
)

mw = ROOT / "ClearCNC_Controller/QtController/src/MainWindow.cpp"
mw_lines = mw.read_text(encoding="utf-8", errors="replace").splitlines()
start = end = None
for i, line in enumerate(mw_lines, 1):
    if "CONFIG SPMMX" in line:
        start = max(1, i - 8)
        end = min(len(mw_lines), i + 20)
        break
if start:
    excerpt(mw, start, end, art / "patterns/clearcnc-host-config-sync.cpp", "host CONFIG sync")

write(
    "artifacts/data/clearcnc-protocol-commands.md",
    """---
title: ClearCNC protocol commands
component: clearcnc-firmware
level: project
topics:
  - ClearCNC
  - protocol
  - G-code
  - telemetry
status: verified
---

<!-- EXCERPT — source: ClearCNC_Controller/README.md + ClearCNC_Firmware.cpp -->
<!-- EVIDENCE: E1 | symbol: protocol command overview -->

# ClearCNC control protocol (quick reference)

| Channel | Detail |
|---------|--------|
| USB CDC | 115200 baud |
| Control TCP | 8888 |
| Telemetry TCP | 8889 (`TEL`, ≤50 ms) |
| Discovery UDP | 10040 / `CLEARCNC_DISCOVER?` |

Safety: `M202`/`ENABLE`, `M203`/`DISABLE`, `M201`/`STOP`, `M200`/`ESTOP`, `HOLD`/`RESUME`.

Motion: `G0`/`G1`, `G2`/`G3` (XY), `MOVE`, `JOG`, `G90`/`G91`, `G20`/`G21`, `G92`, `G4` dwell.

Status: `M114`/`POS`, `M115`/`STATUS`, `GETCFG`, `HELP`.

Config keys: `SPMMX`/`SPMMY`, `VEL`, `ACCEL`, `DECEL`, `DVMAX`, `SINGLE`, `AX`, `RVEL*`, spindle µA, `ESTOP_DI6`.

## Source evidence

| Claim | Evidence | Level |
|-------|----------|-------|
| Ports / queue defines | `ClearCNC_Firmware.cpp` L80–91 | E1 |
| Command overview | `ClearCNC_Controller/README.md` | E1 |
""",
)

write(
    "artifacts/README.md",
    """# Artifacts registry

| ID | File | Component | Usefulness | Description |
|----|------|-----------|------------|-------------|
| A-L01-if | [interfaces/CoordinatedMotionController.h](interfaces/CoordinatedMotionController.h) | L01 | U1–U6 | Public coordinated API |
| A-L01-pat | [patterns/coordinated-initialize.cpp](patterns/coordinated-initialize.cpp) | L01 | U2 | Initialize + coordinated mode |
| A-L01-def | [patterns/defer-motion-queue-start.h](patterns/defer-motion-queue-start.h) | L01 | U2 | Deferred batch start |
| A-L02-if | [interfaces/ArcInterpolator.h](interfaces/ArcInterpolator.h) | L02 | U1–U6 | ArcSegment + InitializeArc |
| A-L03-if | [interfaces/LinearInterpolator.h](interfaces/LinearInterpolator.h) | L03 | U1–U6 | Linear init + completion |
| A-L04-if | [interfaces/UnitConverter.h](interfaces/UnitConverter.h) | L04 | U1–U6 | Units API |
| A-L04-pat | [patterns/distance-to-steps.cpp](patterns/distance-to-steps.cpp) | L04 | U2 | DistanceToSteps math |
| A-L05-if | [interfaces/TrigLUT.h](interfaces/TrigLUT.h) | L05 | U1–U6 | SinQx/CosQx |
| A-L06-if | [interfaces/MotorDriver_extensions.h](interfaces/MotorDriver_extensions.h) | L06 | U1–U6 | Coordinated + unit APIs |
| A-L06-pat | [patterns/coordinated-refresh-fallback.cpp](patterns/coordinated-refresh-fallback.cpp) | L06 | U2 | Idle StepGenerator fallback |
| A-P01-pat | [patterns/clearcnc-ports-and-queues.cpp](patterns/clearcnc-ports-and-queues.cpp) | P01 | U2 | Ports + queue sizes |
| A-P01-main | [patterns/clearcnc-main-loop.cpp](patterns/clearcnc-main-loop.cpp) | P01 | U2 | Firmware main loop |
| A-P01-batch | [patterns/clearcnc-planner-batch.cpp](patterns/clearcnc-planner-batch.cpp) | P01 | U2 | XY planner batching |
| A-P01-data | [data/clearcnc-protocol-commands.md](data/clearcnc-protocol-commands.md) | P01 | U1 | Protocol quick reference |
| A-P02-pat | [patterns/clearcnc-host-config-sync.cpp](patterns/clearcnc-host-config-sync.cpp) | P02 | U2 | Host CONFIG sync |
| A-PL01-bld | [build/libclearcore-enhanced-sources.cppproj.mk](build/libclearcore-enhanced-sources.cppproj.mk) | PL01 | U2 | cppproj enhanced sources |
| A-PL02-if | [interfaces/SysTiming_sample_rate.h](interfaces/SysTiming_sample_rate.h) | PL02 | U2 | 5 kHz sample rate |

## Bench

No retained E2 flash/serial logs yet — see [OPEN_QUESTIONS.md](../project/OPEN_QUESTIONS.md).
""",
)

write(
    "artifacts/bench/README.md",
    """# Bench artifacts

Store ClearCore flash/serial captures here for E2 claims. None retained at package creation (2026-07-26).
""",
)

# --- Inventory ---
write(
    "project/COMPONENT_INVENTORY.md",
    """---
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
""",
)

print("inventory+artifacts ok")
