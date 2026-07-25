# CPM_MODE_QUAD_AB Hardware Bring-up Checklist

Software compile of `MotorDriver.cpp` / `MotorManager.cpp` has been verified.
Complete the following on a ClearCore + ClearPath-SD bench before relying on the mode in production.

## MSP (ClearPath) setup

1. Mode: Step and Direction family
2. Input Format: **Quadrature**
3. Prefer **Has Detents** unchecked (each edge = one count = one ClearCore step)
4. HLFB: ASG-Position w/Measured Torque @ 482 Hz (matches the example)
5. Input Resolution: match motor positioning resolution for 1:1 counts

## ClearCore setup

1. `MotorMgr.MotorInputClocking(MotorManager::CLOCK_RATE_NORMAL)`
2. `MotorMgr.MotorModeSet(..., Connector::CPM_MODE_QUAD_AB)`
3. Flash / run `MoveQuadrature` example (or equivalent)

## Logic analyzer checks (Inputs A and B)

1. Idle: both channels inactive
2. Positive move: A rises before B (A leads B); Gray sequence `00 -> 10 -> 11 -> 01 -> 00`
3. Negative move: B leads A (reverse Gray sequence)
4. Edge dwell ~1 us (meets ClearPath >= 750 ns timing)
5. Count of edges equals commanded step distance (with Has Detents off)

## Motion checks

1. Absolute moves reach commanded positions (HLFB asserts)
2. `PolarityInvertSDDirection(true)` reverses lead sense / shaft direction
3. Disable during motion cancels and sets `MotionCanceledMotorDisabled`
4. Coordinated XY (if used) still emits quadrature on both axes
