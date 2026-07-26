# LibraryDocs index

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
