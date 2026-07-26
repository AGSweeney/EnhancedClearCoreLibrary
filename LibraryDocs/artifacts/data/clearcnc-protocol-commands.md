---
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
