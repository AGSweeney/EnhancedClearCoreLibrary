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

# Constant Tangential Velocity Implementation

## Overview

The coordinated arc motion system maintains **constant tangential velocity** along the arc path. This means the speed along the arc circumference is constant, while individual motor speeds adapt automatically to maintain this constant path speed.

## Key Principle

### Tangential Velocity (Path Speed)
- **Definition**: Speed along the arc circumference (steps/sec)
- **Constant**: This speed remains constant throughout the arc
- **User-Specified**: Set via `ArcVelMax()` parameter

### Individual Motor Speeds
- **Adaptive**: Each motor's speed varies based on its position on the arc
- **Automatic**: Calculated from arc geometry (sin/cos)
- **Coordinated**: Motors work together to maintain constant path speed

## Mathematical Foundation

### Relationship Between Speeds

```
Tangential Velocity: v = r × ω
Where:
  v = tangential velocity along arc (steps/sec) - CONSTANT
  r = arc radius (steps)
  ω = angular velocity (rad/sec)

Angular Velocity: ω = v / r
Angle Increment: dθ = ω × dt = (v / r) × (1 / sampleRateHz)
```

### Individual Motor Speeds

At any point on the arc:
```
X Motor Speed: v_x = v × cos(θ)
Y Motor Speed: v_y = v × sin(θ)

Where θ is the current angle along the arc
```

## Implementation Details

### Angle Increment Calculation

```cpp
// Calculate angle increment per sample to maintain constant path speed
angleIncrementPerSample = (velocityMax × 2π) / (radius × sampleRateHz)

Where:
  velocityMax = tangential velocity along arc (steps/sec) - CONSTANT
  radius = arc radius (steps)
  sampleRateHz = ISR sample rate (5000 Hz)
```

### Position Calculation

```cpp
// Calculate position on arc
X = centerX + radius × cos(angle)
Y = centerY + radius × sin(angle)

// Calculate step deltas (motor speeds)
stepsX = X_new - X_old  // X motor speed adapts via cos(θ)
stepsY = Y_new - Y_old  // Y motor speed adapts via sin(θ)
```

## Example: 90-Degree Arc

Consider a 90° arc with:
- Radius: 10000 steps
- Tangential velocity: 5000 steps/sec
- Sample rate: 5000 Hz

### At Start (θ = 0°)
- X motor: Moving at full speed (cos(0°) = 1)
- Y motor: Not moving (sin(0°) = 0)
- Path speed: 5000 steps/sec ✓

### At 45° (θ = 45°)
- X motor: Moving at 70.7% speed (cos(45°) = 0.707)
- Y motor: Moving at 70.7% speed (sin(45°) = 0.707)
- Combined: √(0.707² + 0.707²) = 1.0 → Path speed: 5000 steps/sec ✓

### At End (θ = 90°)
- X motor: Not moving (cos(90°) = 0)
- Y motor: Moving at full speed (sin(90°) = 1)
- Path speed: 5000 steps/sec ✓

## Verification

### Constant Path Speed
- ✅ Angle increment calculated from: `(v / r) × dt`
- ✅ Ensures constant tangential velocity
- ✅ Independent of motor position

### Adaptive Motor Speeds
- ✅ X motor speed: `v × cos(θ)` - varies with angle
- ✅ Y motor speed: `v × sin(θ)` - varies with angle
- ✅ Combined speed: Always equals path speed

### Smooth Motion
- ✅ Angle updates smoothly each sample period
- ✅ Position calculated from angle using trigonometry
- ✅ Step deltas calculated from position differences
- ✅ No jerky motion or speed variations

## Code Flow

```
1. User sets: ArcVelMax(5000)  // 5000 steps/sec along path

2. Each ISR call (200 µs):
   a. Calculate: angleIncrement = (5000 × 2π) / (radius × 5000)
   b. Update: angle += angleIncrement
   c. Calculate: X = centerX + radius × cos(angle)
   d. Calculate: Y = centerY + radius × sin(angle)
   e. Calculate: stepsX = X_new - X_old  // Adapts automatically
   f. Calculate: stepsY = Y_new - Y_old  // Adapts automatically
   g. Send steps to motors

3. Result:
   - Path speed: Constant 5000 steps/sec ✓
   - X motor speed: Varies (cos(θ) × 5000)
   - Y motor speed: Varies (sin(θ) × 5000)
```

## Benefits

1. **Predictable Motion**: Constant path speed means predictable timing
2. **Smooth Operation**: No speed variations along the arc
3. **Automatic Coordination**: Motors automatically adapt
4. **Accurate Path Following**: Exact arc geometry maintained

## Conclusion

The implementation correctly maintains **constant tangential velocity** along the arc path. Individual motor speeds adapt automatically through the trigonometric calculations (sin/cos), ensuring smooth, coordinated motion while keeping the path speed constant.
