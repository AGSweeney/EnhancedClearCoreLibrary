---
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
