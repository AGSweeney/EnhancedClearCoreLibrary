---
title: Host Qt build
component: host-build
level: platform
topics:
  - Qt6
  - CMake
  - vcpkg
  - build.ps1
  - windeployqt
source_paths:
  - build.ps1
  - ClearCNC_Controller/QtController/CMakeLists.txt
status: verified
---

# Host Qt build

Windows desktop builds for ClearCNC (default); same CMake patterns for ClearHPGL / NewGControl.

## build.ps1 defaults

| Param | Default |
|-------|---------|
| SourceDir | `ClearCNC_Controller/QtController` |
| BuildDir | `ClearCNC_Controller/<Config>` |
| Generator | Ninja |
| Toolchain | vcpkg `vcpkg.cmake` |

Override VS DevCmd / CMake / vcpkg paths as needed (machine-local).

## Source evidence

| Claim | Evidence | Level |
|-------|----------|-------|
| Default SourceDir/BuildDir | `build.ps1` L34–39 | E1 |
| Qt components | `ClearCNC_Controller/QtController/CMakeLists.txt` | E1 |
