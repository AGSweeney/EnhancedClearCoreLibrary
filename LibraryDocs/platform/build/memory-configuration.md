---
title: Memory configuration
component: build
level: platform
topics:
  - sample-rate
  - queue
  - buffers
  - ClearCore
source_paths:
  - libClearCore/inc/SysTiming.h
  - libClearCore/inc/CoordinatedMotionController.h
  - ClearCNC_Controller/ClearCoreFirmware/ClearCNC_Firmware.cpp
status: verified
---

# Memory configuration

| Resource | Value | Evidence |
|----------|-------|----------|
| Sample rate | 5000 Hz (200 µs) | `SysTiming.h` `_CLEARCORE_SAMPLE_RATE_HZ` |
| Planner queue | 16 segments | L01 `ARC_QUEUE_SIZE` |
| Firmware motion queue | 16 blocks | P01 `MOTION_QUEUE_SIZE` |
| Command line buffer | 256 chars | P01 `MAX_LINE_LENGTH` |
| Trig LUT | 1024 Q15 entries | L05 `TRIG_LUT_SIZE` |

Keep firmware batch size ≤ planner capacity so `Queue*` cannot fail mid-batch.

## Source evidence

| Claim | Evidence | Level |
|-------|----------|-------|
| 5000 Hz | `SysTiming.h` L42–43 | E1 |
| Queues = 16 | L01 header; P01 L80 | E1 |
