---
title: Portability notes
component: build
level: platform
topics:
  - portability
  - SAME53
  - Qt
  - host-vs-mcu
source_paths:
  - README.md
  - build.ps1
status: verified
---

# Portability notes

| Domain | Notes |
|--------|-------|
| MCU firmware | ClearCore SAME53 only for coordinated motion ISR assumptions |
| Host apps | Windows Qt 6; CMake/vcpkg paths in `build.ps1` are machine-local |
| Protocol | ClearCNC line protocol ≠ GRBL (P04 separate) |
| Units | Library converters are portable C++; mechanical configs are machine-specific |

## Source evidence

| Claim | Evidence | Level |
|-------|----------|-------|
| ClearCore / Studio requirements | root `README.md` | E1 |
| Host build defaults | `build.ps1` | E1 |
