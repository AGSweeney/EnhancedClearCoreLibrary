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

1. Idle: both channels inactive (`00`)
2. Positive move: full **ABAB** cycle per count: `A on → B on → A off → B off`
3. Negative move: full **BABA** cycle per count: `B on → A on → B off → A off`
4. Edges spaced across the sample (~50 µs at 5 kHz), not a tight burst; idle between counts at low speed
5. Every A assertion has a matching B; lines return to idle after each count
6. Count of complete cycles equals commanded step distance (Has Detents off)

## Motion checks

1. Absolute moves reach commanded positions (HLFB asserts)
2. `PolarityInvertSDDirection(true)` reverses lead sense / shaft direction
3. Disable during motion cancels and sets `MotionCanceledMotorDisabled`
4. Coordinated XY (if used) still emits quadrature on both axes
