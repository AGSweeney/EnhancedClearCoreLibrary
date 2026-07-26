---
title: Build libClearCore
component: build-libclearcore
level: project
topics:
  - build
  - Microchip-Studio
  - libClearCore
status: verified
---

# Build libClearCore

1. Install Microchip Studio 7.0.1645+ with SAME53_DFP 1.1.118 and CMSIS 4.5.0.
2. Open `libClearCore/ClearCore.atsln`.
3. Build Debug/Release → `libClearCore.a`.
4. Confirm enhanced sources ([A-PL01-bld](../../artifacts/build/libclearcore-enhanced-sources.cppproj.mk)).

See [platform/build](../../platform/build/build-instructions.md).
