---
title: Build instructions
component: build
level: platform
topics:
  - Microchip-Studio
  - libClearCore
  - cppproj
  - SAME53
source_paths:
  - libClearCore/ClearCore.cppproj
  - libClearCore/ClearCore.atsln
status: verified
---

# Build instructions

## libClearCore

1. Open `libClearCore/ClearCore.atsln` in Microchip Studio.
2. Select Debug or Release.
3. Build → `libClearCore.a`.

Enhanced sources compiled via `ClearCore.cppproj`: `CoordinatedMotionController`, `ArcInterpolator`, `LinearInterpolator`, `UnitConverter`, `TrigLUT`.

Artifact: [A-PL01-bld](../../artifacts/build/libclearcore-enhanced-sources.cppproj.mk).

## Application firmware

Open the app `.atsln` (e.g. ClearCNC), link built `libClearCore`, set Startup Project, build/flash.

## Source evidence

| Claim | Evidence | Level |
|-------|----------|-------|
| Enhanced Compile includes | `ClearCore.cppproj` ~L885–900 | E1 |
| Solution present | `libClearCore/ClearCore.atsln` | E1 |
