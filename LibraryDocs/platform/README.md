# Platform

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
