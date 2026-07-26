---
title: Deploy and flash
component: build
level: platform
topics:
  - flash
  - bossac
  - UF2
  - ClearCore
source_paths:
  - Tools/README.md
  - Tools/flash_clearcore.cmd
status: verified
---

# Deploy and flash

| Tool | Role |
|------|------|
| Microchip Studio | Build + debug flash |
| `Tools/bossac.exe` | CLI flash |
| `Tools/flash_clearcore.cmd` | Find USB port + bossac |
| `Tools/uf2-builder.exe` | `.bin` → UF2 drag-drop |

See `Tools/README.md` and recipe [flash-clearcnc-firmware](../../project/recipes/flash-clearcnc-firmware.md).

## Source evidence

| Claim | Evidence | Level |
|-------|----------|-------|
| Tool list | `Tools/README.md` | E1 |
