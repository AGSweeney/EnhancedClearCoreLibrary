# Enhanced ClearCore Library

This repository contains the ClearCore Motion and I/O Library, enhanced with coordinated motion (arcs, linear) and unit conversion support. It provides a foundation to build ClearCore applications. Also included are Microchip Studio example programs—including G-code streaming and GRBL-compatible firmware—and a Microchip Studio template project for building your own application.

## Enhanced Features

This library includes enhanced motion control capabilities:

### Coordinated Motion
- **Coordinated Arc Moves**: Generate smooth circular arc paths with two motors moving synchronously
- **Coordinated Linear Moves**: Straight-line coordinated motion with constant velocity along the path
- **Continuous Motion Chaining**: Seamlessly chain multiple arcs or linear moves without stopping
- **Constant Path Velocity**: Maintains constant velocity along the programmed path (arc circumference or linear distance)

### Unit Conversion Support
- **Physical Units**: Program moves in inches, millimeters, centimeters, meters, revolutions, or degrees instead of raw step counts
- **Feed Rate Units**: Set velocities in inches/minute, mm/minute, mm/second, RPM, or steps/second
- **Mechanical Configuration**: Configure motors with steps per revolution, lead screw pitch, and gear ratios
- **Automatic Conversion**: Library automatically converts between physical units and step counts based on your mechanical setup

### Key Benefits
- **Intuitive Programming**: Work with familiar units (inches, mm) instead of calculating step counts manually
- **Reduced Errors**: Eliminate manual conversion calculations that can introduce mistakes
- **G-Code Compatible**: Natural fit for CNC applications and G-code interpreters
- **Backward Compatible**: All existing step-based APIs continue to work unchanged

See `NEW_FILES.md` for a complete list of new files added for these features.

#### Microchip Studio requirements

The included Microchip Studio projects require Microchip Studio 7.0.1645 or later (latest recommended).

From **Tools → Device Pack Manager**, ensure these packs are installed:
* **SAME53_DFP** 1.1.118
* **CMSIS** 4.5.0

**Installers and resources**: https://www.teknic.com/downloads/

### libClearCore

libClearCore provides a C++ object-oriented API for the ClearCore hardware. Each connector has an associated object. API reference: https://teknic-inc.github.io/ClearCore-library/.

A Microchip Studio project (*.cppproj) is included to build the library.

### LwIP

Ethernet support uses the LwIP stack. Use the ethernet API from libClearCore for Ethernet apps. LwIP source is included. A Microchip Studio project (*.cppproj) is provided to build LwIP.

### Root-level examples

| Example | Description |
|--------|-------------|
| **MotionStreamingExample** | Stream G-code over serial (USB CDC) or Ethernet TCP. Supports G01, G02/G03 (I,J or R), G20/G21, G90/G91, G92, G4, M200–M203, M114, M115, M500, M501. See `MotionStreamingExample/README.md` and `MotionStreamingExample/STREAMING_GUIDE.md`. |
| **GRBLCompatibleExample** | GRBL-style firmware: G0/G1/G2/G03, $ commands, jogging ($J=), homing ($H), gSender-friendly. Serial only. See `GRBLCompatibleExample/README.md`. |
| **ProjectTemplate** | Minimal Microchip Studio template. Put your code in `main.cpp`. |

Open the matching `.atsln`, set the project as **Startup Project**, then **Start Without Debugging (Ctrl+Alt+F5)** to build, flash, and run.

### Microchip_Examples

Official-style examples for ClearCore features (Analog I/O, CCIO, Serial, Ethernet, Step & Direction, etc.). Each subfolder has a solution (*.atsln). Load it, set the desired project as Startup Project, then build and run.

#### Coordinated motion (Microchip_Examples)

- **CoordinatedArcMoves** and **CoordinatedMovesWithUnits** live under `Microchip_Examples/ClearPathModeExamples/ClearPath-SD_Series/`. They demonstrate arc chaining, unit conversion, and feed rate in physical units.

See `docs/CoordinatedArcMotion/README.md` for coordinated motion theory and API details.

### ProjectTemplate

Minimal starter for new apps. Open `ProjectTemplate/ProjectTemplate.atsln`, add your code to `main.cpp`, then build and flash.

### Tools

Windows tools for flashing firmware via USB (see `Tools/README.md`):

- **bossac** — Command-line flasher
- **flash_clearcore.cmd** — Finds ClearCore USB port and flashes with bossac
- **uf2-builder** — Converts firmware to UF2 for drag-and-drop flashing via bootloader

## Documentation

- **API reference**: https://teknic-inc.github.io/ClearCore-library/
- **Coordinated motion**: `docs/CoordinatedArcMotion/README.md` — theory, usage, API
- **Motion streaming**: `MotionStreamingExample/README.md`, `MotionStreamingExample/STREAMING_GUIDE.md`
- **GRBL-compatible example**: `GRBLCompatibleExample/README.md`
- **New/modified files**: `NEW_FILES.md`, `MODIFICATIONS.md`

## License

The ClearCore library is provided under the MIT License. New coordinated motion and unit conversion features are also licensed under the MIT License with copyright attribution to Adam G. Sweeney <agsweeney@gmail.com>.
