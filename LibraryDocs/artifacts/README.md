# Artifacts registry

| ID | File | Component | Usefulness | Description |
|----|------|-----------|------------|-------------|
| A-L01-if | [interfaces/CoordinatedMotionController.h](interfaces/CoordinatedMotionController.h) | L01 | U1–U6 | Public coordinated API |
| A-L01-pat | [patterns/coordinated-initialize.cpp](patterns/coordinated-initialize.cpp) | L01 | U2 | Initialize + coordinated mode |
| A-L01-def | [patterns/defer-motion-queue-start.h](patterns/defer-motion-queue-start.h) | L01 | U2 | Deferred batch start |
| A-L02-if | [interfaces/ArcInterpolator.h](interfaces/ArcInterpolator.h) | L02 | U1–U6 | ArcSegment + InitializeArc |
| A-L03-if | [interfaces/LinearInterpolator.h](interfaces/LinearInterpolator.h) | L03 | U1–U6 | Linear init + completion |
| A-L04-if | [interfaces/UnitConverter.h](interfaces/UnitConverter.h) | L04 | U1–U6 | Units API |
| A-L04-pat | [patterns/distance-to-steps.cpp](patterns/distance-to-steps.cpp) | L04 | U2 | DistanceToSteps math |
| A-L05-if | [interfaces/TrigLUT.h](interfaces/TrigLUT.h) | L05 | U1–U6 | SinQx/CosQx |
| A-L06-if | [interfaces/MotorDriver_extensions.h](interfaces/MotorDriver_extensions.h) | L06 | U1–U6 | Coordinated + unit APIs |
| A-L06-pat | [patterns/coordinated-refresh-fallback.cpp](patterns/coordinated-refresh-fallback.cpp) | L06 | U2 | Idle StepGenerator fallback |
| A-P01-pat | [patterns/clearcnc-ports-and-queues.cpp](patterns/clearcnc-ports-and-queues.cpp) | P01 | U2 | Ports + queue sizes |
| A-P01-main | [patterns/clearcnc-main-loop.cpp](patterns/clearcnc-main-loop.cpp) | P01 | U2 | Firmware main loop |
| A-P01-batch | [patterns/clearcnc-planner-batch.cpp](patterns/clearcnc-planner-batch.cpp) | P01 | U2 | XY planner batching |
| A-P01-data | [data/clearcnc-protocol-commands.md](data/clearcnc-protocol-commands.md) | P01 | U1 | Protocol quick reference |
| A-P02-pat | [patterns/clearcnc-host-config-sync.cpp](patterns/clearcnc-host-config-sync.cpp) | P02 | U2 | Host CONFIG sync |
| A-PL01-bld | [build/libclearcore-enhanced-sources.cppproj.mk](build/libclearcore-enhanced-sources.cppproj.mk) | PL01 | U2 | cppproj enhanced sources |
| A-PL02-if | [interfaces/SysTiming_sample_rate.h](interfaces/SysTiming_sample_rate.h) | PL02 | U2 | 5 kHz sample rate |

## Bench

No retained E2 flash/serial logs yet — see [OPEN_QUESTIONS.md](../project/OPEN_QUESTIONS.md).
