# ClearCNC Controller

**Experimental.** This program and its firmware are under active development. They are **not** certified for production or safety-critical use. Use only in controlled conditions, with proper machine safeguards, and at your own risk.

Desktop host application and ClearCore firmware for driving a small CNC or bench motion setup. The controller connects over **USB serial** or **Ethernet** (control + optional telemetry), sends motion and setup commands, and can **stream a G-code file** with queue-aware program execution, MDI motion/jog, and a **3D tool-position view** with optional path preview.

![ClearCNC Controller main window (Program tab, Ethernet, 3D DRO)](images/Screenshot%202026-04-29%20200411.png)

## What’s in this folder

| Path | Role |
|------|------|
| `QtController/` | Qt Widgets GUI: `ClearCNC_Controller` |
| `ClearCoreFirmware/` | ClearCore project (Microchip Studio), line-based command protocol |
| `TestGCodes/` | Sample programs (e.g. `SpindleControl.gcode` for `M3`/`M5`/`S`/`G4`) |
| `images/` | Documentation screenshots |

## Application features (Qt)

- **Connection:** Serial port or Ethernet (control TCP, optional telemetry TCP; discovery UDP for devices on the LAN).
- **Machine control:** Enable / disable, feed hold–style **Pause** / **Resume**, stop, and E-stop; absolute or relative mode; work zero (where supported by firmware). **E-Stop** button shows **hardware DI-6** status (yellow / red / amber) when `HwEstopOk` is present on telemetry or `M115`.
- **MDI tab:** `Move` and jog controls for directed motion and feed rate.
- **Program tab:** Load `.nc` / `.tap` / `.gcode` (and friends), preview cleaned lines, run/pause/stop with buffer awareness when telemetry is available. Programs may use **spindle** (`M3`/`M4`/`M5`, `S`) and **dwell** (`G4`/`G04`) lines supported by the firmware (see below).
- **3D view:** Orbit and zoom, live DRO-style overlay, optional **Path** trace from parsed G-code.
- **Setup:** Motion parameters (steps, limits, per-port axis mapping) persisted via application settings; optional command log panel.

## Firmware (ClearCore)

**Project file:** `ClearCoreFirmware/ClearCNC_Firmware.atsln`

1. Set `ClearCNC_Firmware` as the startup project.
2. Build and flash using your usual ClearCore / Microchip Studio workflow.
3. Use **USB serial** and/or **Ethernet** per your board configuration.

The firmware speaks a **line-oriented** protocol. Send `HELP` over the control link for the current keyword list. Motion must be enabled (`M202` / `ENABLE`) before moves. Default modes are typically absolute and millimeters on boot; the GUI can set units and sync parameters.

### Command overview

| Area | Notes |
|------|--------|
| Motion | `G0`/`G00`, `G1`/`G01`, `G2`/`G02`, `G3`/`G03` (XY), `JOG`, `MOVE`; `G90`/`G91`, `G20`/`G21`, `G92` work offset |
| Program / safety | `M201` stop decel, `M200`/`ESTOP` abrupt stop + disable + queue clear; `M2`/`M02`/`M30` program end (host + spindle I/O stop) |
| Status | `M114`/`POS`, `M115`/`STATUS`, `GETCFG`; telemetry `TEL …` on TCP 8889 when connected |
| Config | `CONFIG …` with `SPMMX`/`SPMMY`, `VEL`, `RVELX`/`RVELY`/`RVELZ`/`RVELA`, `ACCEL`, `DECEL`, `DVMAX`, `SINGLE`, `AX`, spindle, estop |

### Spindle (ClearCore I/O)

Spindle words may appear alone or on the same line as motion; the firmware parses `S` / spindle `M` codes on **every** line first.

| Code / word | Hardware |
|-------------|-----------|
| `M3` / `M03` | Spindle CW: **IO-1** high; **IO-2** is turned off first if it was on |
| `M4` / `M04` | Spindle CCW: **IO-2** high; **IO-1** off first if it was on |
| `M5` / `M05` | Stop: **IO-1** and **IO-2** low; **IO-0** analog current to *minimum* (see below) |
| `S…` | Speed reference while `M3`/`M4` active; `S` is ignored for analog scaling while spindle direction is off (value is still stored) |
| `M2` / `M02` / `M30` | Same spindle stop as `M5` on the I/O lines (program end for the host) |

**IO-0** is configured as **analog current output** (`ConnectorIO0.Mode(OUTPUT_ANALOG)`). Speed is commanded with **`OutputCurrent(microamps)`** (same API family as Teknic’s [WriteAnalogCurrentOutput](https://teknic-inc.github.io/ClearCore-library/_write_analog_current_output_8cpp-example.html)), not raw `AnalogWrite` counts.

Default loop is **4–20 mA**: at `S = 0` or spindle off → **4000 µA**; at `S = SPINDLE_SMAX` → **20000 µA**, linear in between.

| `CONFIG` key | Meaning |
|--------------|---------|
| `SPINDLE_SMAX` | `S` value treated as full speed (default 24000) |
| `SPINDLE_MIN_UA` | Current at `S = 0` / off (default **4000** = 4 mA) |
| `SPINDLE_MAX_UA` | Current at `S = SPINDLE_SMAX` (default **20000** = 20 mA; max 20000 per library) |

For a **0–20 mA** drive, use e.g. `CONFIG SPINDLE_MIN_UA=0 SPINDLE_MAX_UA=20000`. Invalid min/max pairs are clamped back to 4 mA / 20 mA defaults.

### Per-axis rapid (`CONFIG RVELX` / `RVELY` / `RVELZ` / `RVELA`)

- Values are **steps per second** on that axis (same unit family as `VEL`).
- **`CONFIG VEL=…`** still sets the global feed/axis `VelMax` and, when present on a line, **resets all four** `RVEL*` defaults to match `VEL`. **`RVELX`…`RVELA`** on the same line (or a follow-up `CONFIG`) override per axis.
- For a multi-axis **G0**, the planner uses the smallest vector speed that keeps every moving axis within its `RVEL*` cap.
- **`GETCFG`** reports `RVELX`…`RVELA`. The Qt **Motion Parameters → Rapids** tab edits these in mm/min or in/min (and **deg/min** for a rotary 4th axis).

### Laser mode

- Laser mode is **not supported**.
- `CONFIG LASER` is accepted only for backward compatibility and is ignored.
- `G0`/`G00` never alter spindle direction outputs for laser-style rapid blanking.
- `GETCFG` reports `LASER=0`.

### Dwell (Fanuc-style `G4` / `G04`)

- **`P`** = dwell in **milliseconds** (integer).
- **`X`** or **`U`** = dwell in **seconds** (floating-point).
- If **`P`** is present, it **wins** over `X`/`U` on the same line.
- **Max** dwell: 600000 ms (10 minutes). Execution **blocks** the command handler, but during the wait the firmware still **polls DI-6** and **reads USB / TCP control** (same control port as the Qt app, not telemetry). **Only** estop-class lines are executed **during** the dwell (`M200`, `ESTOP`, `M203`, `DISABLE`, `M5`/`M05`, `M2`/`M02`, `M30`); **all other lines are queued and run after the dwell returns** so buffered program lines (e.g. `M5` then `M4` after `G4`) are not applied early — which previously could turn on **M4** while still inside **G4**.
- **Not allowed** on the same block as `G0`/`G1`/`G2`/`G3`, `MOVE`, or `JOG`.

### Hardware estop (**DI-6**)

- Initialized like Teknic’s [ReadDigitalInput](https://teknic-inc.github.io/ClearCore-library/_read_digital_input_8cpp-example.html): `ConnectorDI6.Mode(INPUT_DIGITAL)`, read with `ConnectorDI6.State()`.
- **`CONFIG ESTOP_DI6`**: `0` = DI-6 ignored for estop (telemetry reports `HwEstopOk=1`). **`1` (default)** = estop **OK** when `State() != 0` (truthy / “ON”); **estop active** when `State() == 0`. **`2`** = inverted (OK when read is zero).
- When a fault is seen, the firmware runs the same path as **`M200`**: spindle outputs off, motion queue cleared, motors disabled.
- **`STATUS` / `M115`** and telemetry lines include **`HwEstopOk=`**, **`Di6=`**, **`Di6Mode=`** for the host and UI.

### Qt GUI notes

- The **E-Stop** button color reflects **`HwEstopOk`** from telemetry or `M115` when the control link is up: **yellow** = chain OK, **red** = hardware estop active, **amber** = state not yet received. The button still sends **`M200`** when clicked (software estop).
- Program streaming treats **`G4` / `G04`** as non-queued motion (regex word match so **`G40`** is not mistaken for dwell).

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
- **Authoritative protocol text** is the block comment at the top of `ClearCoreFirmware/ClearCNC_Firmware.cpp` plus the `HELP` reply from a running board.
- For deeper integration (homing, tool tables, more alarms), extend both the firmware protocol and the `MainWindow` / streaming logic in lockstep.
