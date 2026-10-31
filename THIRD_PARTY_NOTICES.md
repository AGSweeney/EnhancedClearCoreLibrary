# Third-party notices

EnhancedClearCoreLibrary is a fork of Teknic's ClearCore library. It is released under the
[MIT License](LICENSE): Copyright (c) 2020 Teknic, Inc., with additions Copyright (c) 2026
Adam G. Sweeney. Code and tools by others in this repository remain under their own copyrights and
licences, and changes made to them here stay under the licence of the file they were made in.

| Component | Path | Copyright | Licence |
|-----------|------|-----------|---------|
| ClearCore library | `libClearCore/` (files with a Teknic header) | Teknic, Inc. | MIT: [`libClearCore/license.txt`](libClearCore/license.txt) |
| ClearCore library additions | `libClearCore/` (files with an Adam G. Sweeney header; see `NEW_FILES.md`, `MODIFICATIONS.md`) | Adam G. Sweeney | MIT |
| ClearCore examples and project template | `Microchip_Examples/`, `Depreciated/Microchip_Examples/`, `ProjectTemplate/` | Teknic, Inc. | MIT |
| Microchip Advanced Software Framework (ASF) | `libClearCore/hal/`, `hpl/`, `hri/`, `usb/` | Microchip Technology Inc. and its subsidiaries | Microchip ASF licence, in each file's header (use with Microchip products) |
| Microchip device startup and linker scripts | `*/Device_Startup/` | Microchip Technology Inc. | Apache-2.0 (`SPDX-License-Identifier` in each file) |
| lwIP TCP/IP stack | `LwIP/` | Swedish Institute of Computer Science and contributors | BSD-style, in each file's header |
| jsmn JSON tokenizer | `ClearAI_Controller/ClearCoreFirmware/third_party/jsmn/`, `ClearCoreROS/firmware/third_party/jsmn/` | Serge Zaitsev | MIT, in `jsmn.h` |
| ClearCore flash scripts | `Tools/flash_clearcore.cmd`, `Tools/flash_clearcore_loop.cmd` | Teknic, Inc. (authors Zachary Lebold, Nick Dawson) | Distributed with Teknic's ClearCore library (MIT) |
| BOSSA flash utility | `Tools/bossac/bossac.exe` | BOSSA authors, <https://github.com/shumatech/BOSSA> | Upstream licence applies |
| UF2 conversion tool | `Tools/uf2-builder/` | Distributed with Teknic's ClearCore library; UF2 format by Microsoft, <https://github.com/microsoft/uf2> | Upstream licence applies |

The controller projects (`ClearCNC_Controller/`, `ClearAI_Controller/`, `ClearCoreROS/`,
`ClearHPGL/`, `NewGControl/`), `LibraryDocs/`, `docs/` and the build scripts are by Adam G. Sweeney
(MIT), apart from the third-party parts listed above.

Changes made here to third-party files are documented in `MODIFICATIONS.md`. They include changes to
Teknic files and one change to a Microchip ASF file (`libClearCore/hpl/usb/hpl_usb.c`, USB endpoint
direction), which remains under Microchip's licence.
