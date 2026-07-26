# -*- coding: utf-8 -*-
from pathlib import Path

LD = Path(__file__).resolve().parent


def write(rel: str, text: str) -> None:
    path = LD / rel
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(text.lstrip("\n"), encoding="utf-8")
    print("wrote", rel)


LIB_NA_PROTOCOL = "Not applicable — in-process C++ API (no network framing)."
LIB_NA_PERSIST = "Not applicable — no NVM/config ownership in this library."


def lib_doc(
    component,
    title,
    reuse,
    platforms,
    topics,
    sources,
    questions,
    purpose,
    api,
    ownership,
    lifecycle,
    config,
    limits,
    state,
    errors,
    thread,
    memory,
    deps,
    integration,
    observability,
    pitfalls,
    related_body,
    evidence,
    artifacts_note,
):
    topics_y = "\n".join(f"  - {t}" for t in topics)
    src_y = "\n".join(f"  - {p}" for p in sources)
    # Validator: list items directly under retrieval: (2-space dashes)
    q_y = "\n".join(f"  - {q}" for q in questions)
    body_q = "\n".join(f"- {q}" for q in questions)
    return f"""---
title: {title}
component: {component}
level: library
reuse: {reuse}
platforms:
  - {platforms}
topics:
{topics_y}
source_paths:
{src_y}
status: verified
retrieval:
{q_y}
---

# {title}

{purpose} **Reuse:** `{reuse}`.

## Purpose and reuse

{purpose}

## Public API surface

{api}

{artifacts_note}

## Ownership and concurrency

{ownership}

## Runtime lifecycle

{lifecycle}

## Configuration

{config}

## Limits and capacities

{limits}

## State and modes

{state}

## Error handling

{errors}

## Thread and ISR safety

{thread}

## Memory footprint

{memory}

## Dependencies

{deps}

## Integration points

{integration}

## Protocol and wire format

{LIB_NA_PROTOCOL}

## Persistence

{LIB_NA_PERSIST}

## Observability

{observability}

## Failure modes and pitfalls

{pitfalls}

## Portability

ClearCore SAME53 / Teknic `libClearCore` build. Fixed-point and sample-ISR assumptions are MCU-specific.

## Testing and verification

Exercised via Microchip examples and ClearCNC firmware motion paths. No in-repo unit-test harness for these classes.

## Related components

{related_body}

## Retrieval questions

{body_q}

## Source evidence

{evidence}
"""


write(
    "libraries/coordinated-motion-controller/README.md",
    lib_doc(
        "coordinated-motion-controller",
        "CoordinatedMotionController",
        "high",
        "ClearCore SAME53",
        ["coordinated-motion", "QueueArc", "QueueLinear", "planner", "UpdateFast", "junction"],
        [
            "libClearCore/inc/CoordinatedMotionController.h",
            "libClearCore/src/CoordinatedMotionController.cpp",
        ],
        [
            "How do I initialize CoordinatedMotionController with M0/M1?",
            "Which context owns UpdateFast and the motion queue?",
            "What breaks when QueueArc/QueueLinear fail or the planner queue is full?",
        ],
        "High-reuse two-motor coordinated planner for chained linear/arc moves with constant path velocity and junction speed control.",
        """| Method | Role |
|--------|------|
| `Initialize(motorX, motorY)` | Bind motors; enable coordinated mode |
| `QueueLinear` / `QueueArc` | Enqueue segments (preferred chaining) |
| `SetDeferMotionQueueStart` / `StartDeferredMotionIfPending` | Batch then start |
| `ArcVelMax` / `ArcAccelMax` | Path velocity / accel |
| `JunctionDVmax` / `JunctionDeviationSteps` | Cornering |
| `UpdateFast` | ISR step generator |
| `PlannerQueueCapacity` | Returns `ARC_QUEUE_SIZE` (16) |

Artifact: [A-L01-if](../../artifacts/interfaces/CoordinatedMotionController.h).""",
        "App/firmware thread enqueues. Sample ISR (X motor `Refresh`) owns `UpdateFast` and step output.",
        "Construct → `Initialize` → set vel/accel/junction → `Queue*` (optionally deferred) → ISR runs until drain → `Stop`/`StopDecel`.",
        "Velocity/accel, junction deviation/dVmax, `StopAtQueueEnd`, mechanical params X/Y, defer-start flag.",
        """| Limit | Value | Source |
|-------|-------|--------|
| Planner queue | 16 | `ARC_QUEUE_SIZE` |
| Default vel | 5000 steps/s | ctor |
| Default accel | 50000 steps/s² | ctor |
| Sample rate | 5 kHz | SysTiming |""",
        "Unified `QueuedMotion` queue; `m_active` / `MotionQueueCount()` for host gating.",
        "`Initialize`/`Queue*`/`Move*` return `false` on failure. ClearCNC clears firmware queue if planner submit fails.",
        "Queue fields `volatile` + critical sections. `UpdateFast` is ISR-only.",
        "Fixed 16-entry queue + embedded interpolators; no heap.",
        "`ArcInterpolator`, `LinearInterpolator`, `UnitConverter`, `MotorDriver` (friend).",
        "ClearCNC firmware batches XY; examples under `Microchip_Examples/.../CoordinatedArcMoves`.",
        "`IsActive`, `MotionQueueCount`, `CurrentX/Y`, `GetDebugInfo`.",
        """- Cap batches at `PlannerQueueCapacity()` (match firmware `MOTION_QUEUE_SIZE`).
- Pass arc `startAngle` from parsed block (see header comment).
- Use deferred start when batching ([A-L01-def](../../artifacts/patterns/defer-motion-queue-start.h)).
- Idle motors fall back to per-motor step gen (see L06).""",
        "[arc-interpolator](../arc-interpolator/README.md), [linear-interpolator](../linear-interpolator/README.md), [motor-driver](../motor-driver/README.md), [clearcnc-firmware](../../project/subsystems/clearcnc-firmware/README.md).",
        """| Claim | Evidence | Level |
|-------|----------|-------|
| Public queue/move API | `CoordinatedMotionController.h` L52–161 | E1 |
| `ARC_QUEUE_SIZE = 16` | header L508–511 | E1 |
| Initialize enables coordinated mode | [A-L01-pat](../../artifacts/patterns/coordinated-initialize.cpp) | E1 |
| Deferred batch start | header L134–147 | E1 |""",
        "Patterns: [A-L01-pat](../../artifacts/patterns/coordinated-initialize.cpp), [A-L01-def](../../artifacts/patterns/defer-motion-queue-start.h).",
    ),
)

write(
    "libraries/arc-interpolator/README.md",
    lib_doc(
        "arc-interpolator",
        "ArcInterpolator",
        "medium",
        "ClearCore SAME53",
        ["arc", "Q15", "interpolation", "GenerateNextSteps", "fixed-point"],
        ["libClearCore/inc/ArcInterpolator.h", "libClearCore/src/ArcInterpolator.cpp"],
        [
            "How do I initialize an arc segment for coordinated motors?",
            "Who calls GenerateNextSteps?",
            "What happens if remaining angle wraps past endAngle?",
        ],
        "Medium-reuse fixed-point arc engine that emits per-sample XY step pairs.",
        "`InitializeArc`, `GenerateNextSteps`, `SetExitSpeed`. `ArcSegment` holds Q15 geometry and `spanQx` clamp. [A-L02-if](../../artifacts/interfaces/ArcInterpolator.h).",
        "Owned by `CoordinatedMotionController` on the ISR path.",
        "`InitializeArc` → `GenerateNextSteps` until complete → controller advances queue.",
        "velocityMax, accelMax, sampleRateHz, entry/exit speeds.",
        "Q15 angles 0–32768 ≡ 0–2π; `spanQx` prevents full-circle wrap on overshoot.",
        "`ArcSegment` + ramp/position state.",
        "`InitializeArc` false on bad params; `GenerateNextSteps` false when complete.",
        "ISR-only when driven by controller.",
        "Single active segment; no dynamic allocation.",
        "`TrigLUT` (`SinQx`/`CosQx`).",
        "`CoordinatedMotionController::UpdateFast` for arc motion type.",
        "Steps-remaining / position getters.",
        "Do not treat step-count estimates as completion — use geometry checks.",
        "[coordinated-motion-controller](../coordinated-motion-controller/README.md), [trig-lut](../trig-lut/README.md).",
        """| Claim | Evidence | Level |
|-------|----------|-------|
| ArcSegment Q15 + spanQx | `ArcInterpolator.h` L43–66 | E1 |
| InitializeArc / GenerateNextSteps | L96–120 | E1 |
| Interface excerpt | [A-L02-if](../../artifacts/interfaces/ArcInterpolator.h) | E1 |""",
        "Interface: [A-L02-if](../../artifacts/interfaces/ArcInterpolator.h).",
    ),
)

write(
    "libraries/linear-interpolator/README.md",
    lib_doc(
        "linear-interpolator",
        "LinearInterpolator",
        "medium",
        "ClearCore SAME53",
        ["linear", "interpolation", "coordinated", "path-velocity", "GenerateNextSteps"],
        ["libClearCore/inc/LinearInterpolator.h", "libClearCore/src/LinearInterpolator.cpp"],
        [
            "How do I start a coordinated linear segment?",
            "When is a linear move considered complete?",
            "What sample rate should I pass to InitializeLinear?",
        ],
        "Medium-reuse straight-line interpolator with constant velocity along the XY path.",
        "`InitializeLinear`, `GenerateNextSteps`, `IsLinearComplete` (position-primary). [A-L03-if](../../artifacts/interfaces/LinearInterpolator.h).",
        "Owned by `CoordinatedMotionController` on ISR path.",
        "Initialize → GenerateNextSteps until at target → pop segment.",
        "Path vel/accel, sample rate (typically 5000 Hz), entry/exit speeds.",
        "Integer step endpoints; completion requires current == end.",
        "Current/end positions, remaining estimate, ramp.",
        "Bool returns on init/generate.",
        "ISR via controller.",
        "Fixed members only.",
        "ClearCore namespace / stdint.",
        "Controller unified queue linear entries.",
        "Current position getters.",
        "`IsLinearComplete` trusts position equality over step-count estimate.",
        "[coordinated-motion-controller](../coordinated-motion-controller/README.md).",
        """| Claim | Evidence | Level |
|-------|----------|-------|
| InitializeLinear | `LinearInterpolator.h` L64–69 | E1 |
| Position-primary completion | `IsLinearComplete` L86–94 | E1 |
| Excerpt | [A-L03-if](../../artifacts/interfaces/LinearInterpolator.h) | E1 |""",
        "Interface: [A-L03-if](../../artifacts/interfaces/LinearInterpolator.h).",
    ),
)

write(
    "libraries/unit-converter/README.md",
    lib_doc(
        "unit-converter",
        "UnitConverter",
        "high",
        "ClearCore SAME53",
        ["units", "mm", "inches", "feed-rate", "MotorMechanicalConfig", "steps"],
        ["libClearCore/inc/UnitConverter.h", "libClearCore/src/UnitConverter.cpp"],
        [
            "How do I convert millimeters to steps for a ball screw?",
            "What fields must MotorMechanicalConfig have to be valid?",
            "What breaks when DistanceToSteps is called with invalid config?",
        ],
        "High-reuse static conversion between physical units and step counts via `MotorMechanicalConfig`.",
        "`UnitType`/`FeedRateUnit`, `MotorMechanicalConfig`, `DistanceToSteps`, `StepsToDistance`, `FeedRateToStepsPerSec`, `CalculateConversionFactors`. [A-L04-if](../../artifacts/interfaces/UnitConverter.h), [A-L04-pat](../../artifacts/patterns/distance-to-steps.cpp).",
        "Stateless utility — caller’s thread.",
        "Configure mechanical params → call converters as needed.",
        "`stepsPerRevolution`, `pitch`, `pitchUnit`, `gearRatio`, `inverted`.",
        "Invalid config → conversions return 0. Linear base unit is inches.",
        "None (static methods).",
        "Silent `0` on invalid config or negative feed.",
        "Pure functions if config not mutated mid-call.",
        "Negligible.",
        "`<math.h>` for `round`.",
        "`MotorDriver` unit APIs; coordinated unit helpers. ClearCNC often converts locally.",
        "Return values only.",
        "Do not assume ClearCNC G-code path always calls `DistanceToSteps`.",
        "[motor-driver](../motor-driver/README.md), [coordinated-motion-controller](../coordinated-motion-controller/README.md).",
        """| Claim | Evidence | Level |
|-------|----------|-------|
| Enums/config | `UnitConverter.h` L42–101 | E1 |
| DistanceToSteps math | `UnitConverter.cpp` L35–60 | E1 |
| MM_PER_INCH = 25.4 | `UnitConverter.cpp` L29 | E1 |""",
        "See [A-L04-if](../../artifacts/interfaces/UnitConverter.h), [A-L04-pat](../../artifacts/patterns/distance-to-steps.cpp).",
    ),
)

write(
    "libraries/trig-lut/README.md",
    lib_doc(
        "trig-lut",
        "TrigLUT",
        "medium",
        "ClearCore SAME53",
        ["trigonometry", "Q15", "SinQx", "CosQx", "lookup-table"],
        [
            "libClearCore/inc/TrigLUT.h",
            "libClearCore/src/TrigLUT.cpp",
            "libClearCore/src/TrigLUT_Data.h",
        ],
        [
            "How do I compute sin/cos in Q15 for arc math?",
            "What is TRIG_LUT_SIZE?",
            "How is cosine derived from the sine table?",
        ],
        "Medium-reuse Q15 sine LUT (1024 entries) with linear interpolation; cosine via π/2 shift.",
        "`TRIG_LUT_SIZE` 1024, `SinQx`, `CosQx`. [A-L05-if](../../artifacts/interfaces/TrigLUT.h).",
        "Called from arc ISR; table read-only.",
        "Stateless calls; table linked at build.",
        "Not applicable — fixed table.",
        "1024 entries; frac over 32 Q15 substeps.",
        "None.",
        "Angles masked to 15 bits; no error codes.",
        "Re-entrant read-only.",
        "~1024×4 bytes ROM.",
        "Generated `sinLUT` data.",
        "`ArcInterpolator` geometry.",
        "Not applicable.",
        "Q15 is not IEEE radians — convert before calling.",
        "[arc-interpolator](../arc-interpolator/README.md).",
        """| Claim | Evidence | Level |
|-------|----------|-------|
| TRIG_LUT_SIZE 1024 | `TrigLUT.h` L41 | E1 |
| SinQx interpolation | `TrigLUT.cpp` L39–57 | E1 |
| CosQx = SinQx(+8192) | `TrigLUT.cpp` L65–68 | E1 |""",
        "Interface: [A-L05-if](../../artifacts/interfaces/TrigLUT.h).",
    ),
)

write(
    "libraries/motor-driver/README.md",
    lib_doc(
        "motor-driver",
        "MotorDriver enhancements",
        "medium",
        "ClearCore SAME53",
        ["MotorDriver", "CoordinatedMotionMode", "SetMechanicalParams", "Refresh", "step-dir"],
        ["libClearCore/inc/MotorDriver.h", "libClearCore/src/MotorDriver.cpp"],
        [
            "How do I enable coordinated motion mode on ConnectorM0/M1?",
            "When does Refresh call UpdateFast versus StepsCalculated?",
            "How do I move in millimeters on a single motor?",
        ],
        "Medium-reuse Teknic `MotorDriver` extensions: coordinated mode hooks and physical-unit move/feed APIs.",
        "`CoordinatedMotionMode`, `SetCoordinatedSteps`, `SetMechanicalParams`, `MoveMM`/`MoveInches`/…, `FeedRate*`. [A-L06-if](../../artifacts/interfaces/MotorDriver_extensions.h).",
        "MotorManager sample ISR calls `Refresh` per connector.",
        "Mode/unit setup at init; coordinated mode typically left enabled after controller `Initialize`.",
        "Mechanical params; coordinated controller pointer.",
        "Unit APIs require `IsUnitsConfigured()`.",
        "`m_coordinatedMode`, controller/peer pointers, `m_mechanicalConfig`.",
        "Bool returns on mode/move methods.",
        "`Refresh` ISR; `UpdateFast` only on X when planner active/queued.",
        "Small added members per instance.",
        "`CoordinatedMotionController`, `UnitConverter`, `StepGenerator`.",
        "ClearCNC maps axes to `ConnectorM0`–`M3`.",
        "Teknic status/alert registers; `IsCoordinatedMode`.",
        "Idle coordinated connectors must fall back to `StepsCalculated` — [A-L06-pat](../../artifacts/patterns/coordinated-refresh-fallback.cpp).",
        "[coordinated-motion-controller](../coordinated-motion-controller/README.md), [unit-converter](../unit-converter/README.md), [clearcore-runtime](../../platform/clearcore-runtime/README.md).",
        """| Claim | Evidence | Level |
|-------|----------|-------|
| CoordinatedMotionMode API | `MotorDriver.h` L457–477 | E1 |
| SetMechanicalParams | L498–522 | E1 |
| coordDriving fallback | `MotorDriver.cpp` L485–499 | E1 |""",
        "[A-L06-if](../../artifacts/interfaces/MotorDriver_extensions.h), [A-L06-pat](../../artifacts/patterns/coordinated-refresh-fallback.cpp).",
    ),
)

write(
    "libraries/README.md",
    """# Libraries

Reusable motion and unit-conversion components in enhanced `libClearCore`.

| ID | Component | Path |
|----|-----------|------|
| L01 | CoordinatedMotionController | [coordinated-motion-controller](coordinated-motion-controller/README.md) |
| L02 | ArcInterpolator | [arc-interpolator](arc-interpolator/README.md) |
| L03 | LinearInterpolator | [linear-interpolator](linear-interpolator/README.md) |
| L04 | UnitConverter | [unit-converter](unit-converter/README.md) |
| L05 | TrigLUT | [trig-lut](trig-lut/README.md) |
| L06 | MotorDriver enhancements | [motor-driver](motor-driver/README.md) |

Inventory: [COMPONENT_INVENTORY.md](../project/COMPONENT_INVENTORY.md).
""",
)

print("libraries ok")
