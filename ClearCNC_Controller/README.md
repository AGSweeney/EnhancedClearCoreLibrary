# ClearCNC Controller

**Experimental.** This program and its firmware are under active development. They are **not** certified for production or safety-critical use. Use only in controlled conditions, with proper machine safeguards, and at your own risk.

Desktop host application and ClearCore firmware for driving a small CNC or bench motion setup. The controller connects over **USB serial** or **Ethernet** (control + optional telemetry), sends motion and setup commands, and can **stream a G-code file** with queue-aware program execution, MDI motion/jog, and a **3D tool-position view** with optional path preview.

![ClearCNC Controller main window (Program tab, Ethernet, 3D DRO)](images/Screenshot%202026-04-25%20180305.png)

## What’s in this folder

| Path | Role |
|------|------|
| `QtController/` | Qt Widgets GUI: `ClearCNC_Controller` |
| `ClearCoreFirmware/` | ClearCore project (Microchip Studio), line-based command protocol |
| `images/` | Documentation screenshots |

## Application features (Qt)

- **Connection:** Serial port or Ethernet (control TCP, optional telemetry TCP; discovery UDP for devices on the LAN).
- **Machine control:** Enable / disable, feed hold–style **Pause** / **Resume**, stop, and E-stop; absolute or relative mode; work zero (where supported by firmware).
- **MDI tab:** `Move` and jog controls for directed motion and feed rate.
- **Program tab:** Load `.nc` / `.tap` / `.gcode` (and friends), preview cleaned lines, run/pause/stop with buffer awareness when telemetry is available.
- **3D view:** Orbit and zoom, live DRO-style overlay, optional **Path** trace from parsed G-code.
- **Setup:** Motion parameters (steps, limits, per-port axis mapping) persisted via application settings; optional command log panel.

## Firmware (ClearCore)

**Project file:** `ClearCoreFirmware/ClearCNC_Firmware.atsln`

1. Set `ClearCNC_Firmware` as the startup project.
2. Build and flash using your usual ClearCore / Microchip Studio workflow.
3. Use **USB serial** and/or **Ethernet** per your board configuration.

The firmware speaks a **line-oriented** protocol. Examples (see source header and `HELP` for the live list): `M202` / `M203` (enable / disable), `G90` / `G91`, `G21` / `G20`, `G0` / `G1` moves, `JOG`, `HOLD` / `RESUME`, `STATUS` / `M115`, `CONFIG`, program flow (`M2` / `M30`), and more. Motion must be enabled before moves. Default modes are typically absolute and millimeters on boot; the GUI can set units and sync parameters.

## Building the Qt application

**Requirement:** Qt 6 with Widgets, Serial Port, and Network (and whatever your `CMakeLists.txt` lists).

### vcpkg Qt

```powershell
cmake -S .\ClearCNC_Controller\QtController -B .\build\ClearCNC_Controller\qt `
  -DCMAKE_TOOLCHAIN_FILE="C:\Users\Adam\vcpkg\scripts\buildsystems\vcpkg.cmake"
cmake --build .\build\ClearCNC_Controller\qt --config Release
```

### System Qt (example)

Point `CMAKE_PREFIX_PATH` at your kit, for example:

```powershell
cmake -S .\ClearCNC_Controller\QtController -B .\build\ClearCNC_Controller\qt `
  -DCMAKE_PREFIX_PATH="C:\Qt\6.8.0\msvc2022_64"
cmake --build .\build\ClearCNC_Controller\qt --config Release
```

The build output is `ClearCNC_Controller.exe` under your chosen build tree (e.g. `Release`).

## Typical bring-up

1. Flash the firmware to the ClearCore and confirm serial or Ethernet is reachable.
2. Launch `ClearCNC_Controller`, choose **Serial** or **Ethernet**, connect.
3. Open **Program → Open G-Code…** (or use **Load** on the Program tab), confirm the file appears in the preview; the work area can switch to the **Program** tab after a successful load from the menu.
4. **Enable** drives when it is safe to do so, choose **ABS** or **REL**, set feed as needed, then use **Jog** / **Move** on the **MDI** tab or **Run** on the **MDI** or **Program** path per your workflow.
5. Watch position and, if connected, **STATUS** / telemetry lines in the log; the 3D panel tracks reported position.

## Development notes

- The GUI `Enabled` state tracks the firmware’s commanded enable flag (`M202` / `M203` and related), not per-drive health diagnostics.
- For deeper integration (homing, tool tables, more alarms), extend both the firmware protocol and the `MainWindow` / streaming logic in lockstep.
