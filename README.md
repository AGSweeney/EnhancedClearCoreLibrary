# Enhanced ClearCore Library

**Teknic's ClearCore library with coordinated multi-axis motion, physical units, and the controllers built on it.**

The [Teknic ClearCore](https://teknic.com/products/io-motion-controller/) is an industrial I/O and motion controller for ClearPath servos and step-and-direction drives. Its stock library moves each motor on its own. This fork adds what a machine needs: two motors moving together along straight lines and arcs at a constant path speed, a queue with lookahead so moves flow into each other, and programming in millimetres, inches or degrees instead of step counts.

It also holds the applications built on top of it: a CNC controller, an LLM-driven gantry controller and a GRBL desktop app. Two larger projects that started here now have their own repositories (see [Projects](#projects)).

> [!WARNING]
> The additions and the controller projects are **experimental**. They are not certified for production or safety-critical use. Always keep a physical E-stop in the motor power circuit.

## What this fork adds to libClearCore

| Feature | What it does |
|---------|--------------|
| **Coordinated motion** | `CoordinatedMotionController` drives two motors as one machine: linear and circular-arc moves at a constant velocity along the path, with acceleration limits. |
| **Motion queue with lookahead** | Moves are queued and chained without stopping; the planner works out how fast the machine can pass through each corner. |
| **Physical units** | Configure steps per revolution, lead-screw pitch and gear ratio, then command positions and feed rates in mm, inches, revolutions or degrees (`UnitConverter`, unit methods on `MotorDriver` and the controller). |
| **Quadrature output mode** | `CPM_MODE_QUAD_AB`: drive ClearPath motors with A/B quadrature instead of step and direction. |
| **Sample callback and external step mode** | `SysMgr.SampleCallbackSet()` runs your function in the 5 kHz sample interrupt; `MotorDriver::ExternalStepMode()` / `SetExternalSteps()` let it set each motor's steps every sample, e.g. an electronic cam following an encoder or a crank motor. Used by the Electronic CAM Workbench (ECamUI, its own repository). |
| **Reliability fixes** | Fixes found while building the controllers: step bursts on small arcs, a motor fault on slow nearly-straight moves, stalled TCP transfers, USB serial input stopping mid-transfer, and per-sample step output flipping the direction between pulses (position drift on some drives). |

Everything Teknic's library already did keeps working; the additions are new classes and new methods. [NEW_FILES.md](NEW_FILES.md) lists the new files and [MODIFICATIONS.md](MODIFICATIONS.md) every change to Teknic's files, with the reasons.

### A taste of the API

```cpp
#include "ClearCore.h"

CoordinatedMotionController motion;

int main() {
    MotorMgr.MotorModeSet(MotorManager::MOTOR_ALL, Connector::CPM_MODE_STEP_AND_DIR);
    ConnectorM0.EnableRequest(true);
    ConnectorM1.EnableRequest(true);

    motion.Initialize(&ConnectorM0, &ConnectorM1);
    motion.SetMechanicalParamsX(800, 5.0, UNIT_MM);  // 800 steps/rev, 5 mm per rev
    motion.SetMechanicalParamsY(800, 5.0, UNIT_MM);
    motion.FeedRateMMPerMin(1200);

    // Queued moves run back to back without stopping at the joins.
    motion.MoveLinearMM(50, 0);
    motion.MoveArcMM(50, 25, 25, -M_PI / 2, M_PI / 2, false);  // half circle around (50, 25)
    motion.MoveLinearMM(0, 50);

    while (true) {
    }
}
```

The [coordinated motion guide](docs/CoordinatedArcMotion/README.md) covers the theory, the full API and more examples.

## Projects

Applications built on the library, in this repository:

| Project | What it is |
|---------|------------|
| [ClearCNC Controller](ClearCNC_Controller/README.md) | Qt desktop app and ClearCore firmware for a small CNC or bench motion setup: G-code streaming over USB or Ethernet, jogging, and a 3D tool-position view. |
| [ClearAI Controller](ClearAI_Controller/README.md) | Firmware and Python hosts that let an LLM command ClearPath servos through JSON-RPC primitives ("Code as Policies"), with MCP servers. |
| [NewGControl](NewGControl/README.md) | Standalone Qt desktop controller for GRBL-family firmware, aimed at gSender-style workflows. |

Moved to their own repositories (their history up to the split is still here):

| Project | Repository |
|---------|------------|
| ClearCoreROS: ClearCore firmware and ROS 2 packages that expose M0–M3 as ROS 2 joints | [ClearCore_ROS](https://github.com/AGSweeney/ClearCore_ROS) |
| ClearHPGL: firmware that makes ClearCore an HP-GL pen plotter, plus a desktop app | [ClearCore_HPGL](https://github.com/AGSweeney/ClearCore_HPGL) |

## Examples and templates

| Path | Contents |
|------|----------|
| `Microchip_Examples/` | Teknic's ClearCore examples (analog and digital I/O, CCIO-8, serial, Ethernet, ClearPath modes, step and direction), plus `MoveQuadrature` for the quadrature mode. |
| `ProjectTemplate/` | Minimal Microchip Studio project to start your own application: put your code in `main.cpp`. |
| `Depreciated/` | Older examples kept for reference: G-code motion streaming, a GRBL-compatible firmware, and the first coordinated arc and units examples. |

## Getting started

1. **Install Microchip Studio** 7.0.1645 or later, and from **Tools → Device Pack Manager** install **SAME53_DFP 1.1.118** and **CMSIS 4.5.0**. Teknic's installers and resources are at <https://www.teknic.com/downloads/>.
2. **Open a solution**: an example under `Microchip_Examples/`, `ProjectTemplate/ProjectTemplate.atsln`, or a project's firmware. The solutions include the `libClearCore` and `LwIP` projects, so the library builds with your application.
3. **Build and flash**: set your project as the startup project and choose **Start Without Debugging** (Ctrl+Alt+F5). From the command line, `Tools/flash_clearcore.cmd <file.bin>` finds the ClearCore's USB port and flashes it.

The ClearCNC desktop app builds with `build.ps1` (Visual Studio 2022, CMake and Qt 6); see its README.

## Documentation

| Document | Contents |
|----------|----------|
| [Teknic API reference](https://teknic-inc.github.io/ClearCore-library/) | The stock ClearCore library |
| [docs/CoordinatedArcMotion](docs/CoordinatedArcMotion/README.md) | Coordinated motion: theory, usage, API, examples |
| [NEW_FILES.md](NEW_FILES.md), [MODIFICATIONS.md](MODIFICATIONS.md) | What this fork adds and changes, and why |
| [LibraryDocs](LibraryDocs/README.md) | Knowledge base for the coordinated motion code and the controller apps |
| Project READMEs | [ClearCNC](ClearCNC_Controller/README.md), [ClearAI](ClearAI_Controller/README.md) (also its `PROTOCOL.md` and `docs/`), [NewGControl](NewGControl/README.md) |

## Repository layout

| Path | Contents |
|------|----------|
| `libClearCore/` | The ClearCore library with this fork's additions (Microchip Studio project included) |
| `libClearCore/tests/host/` | PC-side tests for hardware-independent code (`make` with g++); see its README |
| `LwIP/` | lwIP TCP/IP stack used by the library's Ethernet support |
| `ClearCNC_Controller/`, `ClearAI_Controller/`, `NewGControl/` | Projects built on the library |
| `Microchip_Examples/`, `ProjectTemplate/`, `Depreciated/` | Examples and the starter template |
| `docs/`, `LibraryDocs/` | Documentation |
| `Tools/` | `bossac`, `flash_clearcore.cmd` and `uf2-builder` for flashing over USB |
| `ClearCoreROS/`, `ClearHPGL/` | Pointers to the moved projects |

## License

The ClearCore library is provided under the MIT License (Copyright (c) 2020 Teknic, Inc.). The additions in this fork and the controller projects are also MIT licensed, Copyright (c) 2026 Adam G. Sweeney <agsweeney@gmail.com>. See [LICENSE](LICENSE). Microchip ASF, lwIP, jsmn and the bundled tools keep their own licences; see [THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md).
