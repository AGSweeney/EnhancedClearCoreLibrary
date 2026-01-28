<!--
Copyright (c) 2026 Adam G. Sweeney <agsweeney@gmail.com>

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
-->

# Design Guide: Coordinated Continuous Arc Moves for ClearCore

## Executive Summary

This document describes the design and implementation of coordinated continuous arc motion control for two motors in the ClearCore library. The system enables smooth, continuous circular arc movements with support for chaining multiple arcs without stopping.

## Architecture Overview

### Current System
- **Sample Rate**: 5 kHz (200 µs per sample)
- **Motion Generation**: Independent `StepGenerator` per motor
- **Motion Profile**: Trapezoidal (accel/cruise/decel) for linear moves
- **ISR**: `TCC0_0_Handler` → `SysMgr.FastUpdate()` → `MotorDriver::Refresh()` → `StepGenerator::StepsCalculated()`

### New System Requirements
- **Coordinated Motion**: Two motors move synchronously along an arc path
- **Arc Interpolation**: Generate X/Y step sequences from arc geometry
- **Continuous Arcs**: Chain arcs seamlessly without stopping
- **Velocity Control**: Maintain constant tangential velocity along arc
- **Real-time Performance**: Must complete calculations within 200 µs ISR window

## Design Components

### 1. ArcInterpolator Class

**Purpose**: Core arc interpolation engine that generates coordinated step sequences.

**Key Responsibilities**:
- Convert arc parameters (center, radius, angles) to X/Y step sequences
- Maintain arc state and progress tracking
- Generate next step pair for each ISR call
- Handle arc completion and transitions

**Key Data Structures**:
```cpp
struct ArcSegment {
    int64_t centerXQx;      // Arc center X position (Q15 fixed-point)
    int64_t centerYQx;      // Arc center Y position (Q15 fixed-point)
    int64_t radiusQx;       // Arc radius (Q15 fixed-point)
    int32_t startAngleQx;  // Start angle (Q15 fixed-point, 0-2π = 0-32768)
    int32_t endAngleQx;    // End angle (Q15 fixed-point, 0-2π = 0-32768)
    bool clockwise;         // Direction of rotation
    uint32_t totalSteps;   // Total steps in this arc segment
    uint32_t stepsRemaining; // Steps remaining in this arc
};
```

**Algorithms**:
- **Parametric Arc Algorithm**: Angle-based position calculation
- **Fixed-Point Trigonometry**: Lookup tables with linear interpolation
- **Velocity Profiling**: Constant tangential velocity control

### 2. CoordinatedMotionController Class

**Purpose**: High-level coordinator that manages arc queue and integrates with motor system.

**Key Responsibilities**:
- Manage queue of arc segments (up to 8 arcs)
- Coordinate two motors (X and Y axes)
- Integrate with existing `MotorDriver` infrastructure
- Handle velocity/acceleration limits
- Provide public API for arc commands

**Integration Points**:
- Bypasses normal `StepGenerator` motion when in coordinated mode
- Directly sets step counts for both motors in ISR
- Maintains position tracking for both motors

**API Design**:
```cpp
class CoordinatedMotionController {
public:
    // Initialize with two motor references
    bool Initialize(MotorDriver* motorX, MotorDriver* motorY);
    
    // Single arc move
    bool MoveArc(int32_t centerX, int32_t centerY, 
                 int32_t radius, double startAngle, double endAngle,
                 bool clockwise);
    
    // Continuous arc (chains to current position)
    bool MoveArcContinuous(int32_t centerX, int32_t centerY,
                          int32_t radius, double endAngle,
                          bool clockwise);
    
    // Set velocity/acceleration limits
    void ArcVelMax(uint32_t velMax);
    void ArcAccelMax(uint32_t accelMax);
    
    // Control
    void Stop();
    void StopDecel();
    bool IsActive();
    bool ArcComplete();
    
    // ISR callback (called from MotorDriver::Refresh)
    void UpdateFast();
};
```

### 3. MotorDriver Integration

**Modifications Required**:
- Add coordinated motion mode flag
- Add `CoordinatedMotionController` reference
- Modify `Refresh()` to check for coordinated mode
- When in coordinated mode, call controller's `UpdateFast()` instead of `StepGenerator::StepsCalculated()`

**New MotorDriver Methods**:
```cpp
// Enable/disable coordinated motion mode
bool CoordinatedMotionMode(bool enable, CoordinatedMotionController* controller);

// Check if in coordinated mode
bool IsCoordinatedMode() const;

// Set coordinated steps directly (internal use)
void SetCoordinatedSteps(int32_t steps);
```

### 4. Fixed-Point Mathematics

**Coordinate System**:
- All positions in step counts (integer)
- Angles in Q15 fixed-point format (15 fractional bits)
- 2π = 0x8000 in Q15 format (32768)

**Trigonometric Functions**:
- Lookup table (1024 entries) with linear interpolation
- Fast sin/cos calculations (~5-10 µs)
- Q15 format for precision

**Arc Length Calculation**:
```
arcLength = radius × angleSpan (in steps)
totalSteps = arcLength
```

### 5. Velocity Profiling

**Tangential Velocity**:
- Maintain constant speed along arc path
- Calculate velocity components: Vx = V × cos(θ), Vy = V × sin(θ)
- Angle increment: dθ = (v / r) × dt

**Velocity Profile States**:
- Constant velocity along arc (no accel/decel profiling currently)
- Future: Can add acceleration/deceleration ramps

### 6. Continuous Arc Chaining

**Arc Queue**:
- Circular buffer of arc segments (typically 8 arcs)
- Each arc stores: center, radius, end angle, direction
- Arcs chain automatically when current arc completes

**Transition Logic**:
1. When arc completes, check queue for next arc
2. Calculate exit angle from current arc
3. Calculate entry angle for next arc
4. Ensure tangent continuity (angles match)
5. Start next arc seamlessly

**State Machine**:
```
IDLE → ARC_ACTIVE → (next arc or IDLE)
```

## Implementation Details

### ISR Integration

**Modification to MotorDriver::Refresh()**:
```cpp
void MotorDriver::Refresh() {
    // ... existing code ...
    
    if (Connector::m_mode == Connector::CPM_MODE_STEP_AND_DIR) {
        if (m_coordinatedMode && m_coordinatedController) {
            // Coordinated motion mode
            if (this == m_coordinatedMotorX || this == m_coordinatedMotorY) {
                m_coordinatedController->UpdateFast();
            }
        } else {
            // Normal independent motion
            StepGenerator::StepsCalculated();
            StepGenerator::CheckTravelLimits();
            m_bDutyCnt = StepGenerator::m_stepsPrevious;
            UpdateBDuty();
        }
    }
}
```

**CoordinatedMotionController::UpdateFast()**:
```cpp
void CoordinatedMotionController::UpdateFast() {
    if (!m_active || !m_initialized) {
        return;
    }
    
    // Check if current arc is complete
    if (m_interpolator.IsArcComplete()) {
        // Process next arc from queue
        if (!ProcessNextArc()) {
            m_active = false;
            return;
        }
    }
    
    // Generate next step pair
    int32_t stepsX, stepsY;
    if (m_interpolator.GenerateNextSteps(stepsX, stepsY)) {
        // Apply steps to motors
        m_motorX->SetCoordinatedSteps(stepsX);
        m_motorY->SetCoordinatedSteps(stepsY);
        
        // Update current position
        m_currentX += stepsX;
        m_currentY += stepsY;
    }
}
```

### Arc Generation Algorithm

**Parametric Arc Algorithm**:
```
1. Calculate angle increment: dθ = (velocity / radius) × samplePeriod
2. Update angle: θ += dθ
3. Calculate position: X = centerX + radius × cos(θ)
                      Y = centerY + radius × sin(θ)
4. Calculate step deltas: ΔX = X_new - X_old, ΔY = Y_new - Y_old
5. Output steps
```

**Optimization**: Pre-computed lookup tables for sin/cos with linear interpolation.

### Error Handling

**Validation**:
- Check arc parameters (radius > 0, valid angles)
- Verify motors are enabled
- Check for limit switches
- Validate queue not full

**Error Recovery**:
- Stop motion on error
- Clear arc queue
- Set error flags
- Allow user to clear and restart

## Performance Considerations

### Timing Budget (200 µs ISR)
- Arc step generation: ~20-30 µs
- Trigonometric lookup: ~5-10 µs
- Motor step output: ~10-20 µs
- Queue management: ~5-10 µs
- **Total**: ~40-70 µs (well within budget)

### Memory Requirements
- Arc queue: ~200-400 bytes (8 arcs × 50 bytes)
- Lookup tables: ~2-4 KB (1024-entry sin/cos)
- State variables: ~100 bytes
- **Total**: ~2.5-4.5 KB

### CPU Load
- ISR overhead: ~20-35% of 200 µs window
- Background processing: Minimal (only on arc commands)

## Testing Strategy

### Unit Tests
1. Arc generation accuracy
2. Fixed-point math correctness
3. Queue management
4. Velocity profiling

### Integration Tests
1. Single arc execution
2. Continuous arc chaining
3. Velocity transitions
4. Error handling

### System Tests
1. Real motor movement verification
2. Position accuracy validation
3. Velocity smoothness measurement
4. Long-duration continuous arcs

## API Usage Examples

### Example 1: Single Arc
```cpp
CoordinatedMotionController arcController;
arcController.Initialize(&ConnectorM0, &ConnectorM1);
arcController.ArcVelMax(5000);  // 5000 steps/sec
arcController.ArcAccelMax(50000); // 50000 steps/sec²

// Move in a 90° arc, radius 10000 steps, clockwise
arcController.MoveArc(0, 0, 10000, 0, M_PI/2, true);
while (!arcController.ArcComplete()) {
    Delay_ms(10);
}
```

### Example 2: Continuous Arcs
```cpp
// Chain multiple arcs
arcController.MoveArcContinuous(0, 0, 10000, M_PI/2, true);
arcController.MoveArcContinuous(0, 0, 10000, M_PI, true);
arcController.MoveArcContinuous(0, 0, 10000, 3*M_PI/2, true);
arcController.MoveArcContinuous(0, 0, 10000, 2*M_PI, true); // Complete circle
```

## Future Enhancements

1. **3D Arc Support**: Extend to three motors
2. **Spline Interpolation**: Smooth curves beyond arcs
3. **Look-ahead**: Pre-calculate multiple arcs
4. **Adaptive Velocity**: Adjust speed based on curvature
5. **G-code Integration**: Parse G02/G03 commands
6. **Acceleration Profiling**: Add accel/decel ramps along arc

## Conclusion

This design provides a complete solution for coordinated continuous arc motion while maintaining compatibility with existing ClearCore motor control infrastructure. The implementation prioritizes real-time performance and smooth motion quality.
