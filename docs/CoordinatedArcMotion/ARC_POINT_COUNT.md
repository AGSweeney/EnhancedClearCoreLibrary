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

# Arc Point Count: How Many Points Are Calculated?

## Answer: Dynamic, Not Fixed

The arc is **NOT** divided into a fixed number of points. Instead, points are calculated **dynamically** based on:
- Arc length (radius × angle span)
- Velocity (tangential speed along path)
- Sample rate (5000 Hz = 200 µs per point)

## Calculation Method

### Points Are Calculated One Per Sample Period

Each ISR call (200 µs, 5 kHz):
- Calculates **one new point** on the arc
- Updates angle by velocity-based increment
- Computes position from angle using trigonometry

### Number of Points Formula

```
Number of Points = (Arc Length / Velocity) × Sample Rate

Where:
  Arc Length = radius × angle_span (in steps)
  Velocity = tangential velocity (steps/sec)
  Sample Rate = 5000 Hz
```

### Alternative Formula

```
Number of Points = (Arc Length × Sample Rate) / Velocity

Or:
Number of Points = (radius × angle_span × sample_rate) / velocity
```

## Examples

### Example 1: 90° Arc, Radius 10000, Velocity 5000 steps/sec

```
Arc Length = 10000 × (π/2) ≈ 15,708 steps
Velocity = 5000 steps/sec
Sample Rate = 5000 Hz

Number of Points = (15,708 / 5000) × 5000
                 = 3.14 seconds × 5000 points/sec
                 = 15,708 points
```

### Example 2: Full Circle, Radius 10000, Velocity 5000 steps/sec

```
Arc Length = 10000 × 2π ≈ 62,832 steps
Velocity = 5000 steps/sec
Sample Rate = 5000 Hz

Number of Points = (62,832 / 5000) × 5000
                 = 12.57 seconds × 5000 points/sec
                 = 62,832 points
```

### Example 3: Small Arc, Radius 1000, 45°, Velocity 10000 steps/sec

```
Arc Length = 1000 × (π/4) ≈ 785 steps
Velocity = 10000 steps/sec
Sample Rate = 5000 Hz

Number of Points = (785 / 10000) × 5000
                 = 0.0785 seconds × 5000 points/sec
                 = 393 points
```

## Key Characteristics

### ✅ **Dynamic Point Count**

- **Not pre-divided**: Points calculated in real-time
- **Velocity-dependent**: Higher velocity = fewer points (faster traversal)
- **Length-dependent**: Longer arcs = more points
- **Sample-rate dependent**: Fixed at 5000 Hz (200 µs per point)

### ✅ **One Point Per Sample Period**

- **Fixed rate**: One point every 200 µs
- **Real-time calculation**: No pre-computation
- **Continuous**: Points calculated until arc completes

### ✅ **Total Steps vs. Points**

- **totalSteps**: Estimated arc length in steps (for progress tracking)
- **Points**: Actual number of calculations = `(totalSteps / velocity) × sampleRate`
- **Relationship**: Points ≈ totalSteps when velocity ≈ sampleRate

## Code Implementation

### Arc Length Calculation
```cpp
// Calculate arc length in steps
arcLength = radius × angleSpan / 32768
totalSteps = arcLength  // Used for progress tracking
```

### Point Generation
```cpp
// Each sample period (200 µs):
1. Calculate angle increment: dθ = (velocity / radius) × samplePeriod
2. Update angle: θ += dθ
3. Calculate point: X = centerX + radius × cos(θ)
                    Y = centerY + radius × sin(θ)
4. Calculate step deltas: stepsX = X_new - X_old
                          stepsY = Y_new - Y_old
```

### Termination
```cpp
// Arc completes when:
angle reaches endAngle
OR
stepsRemaining reaches 0
```

## Point Density

### Points Per Second
- **Fixed**: 5000 points/sec (sample rate)
- **Independent of**: Arc parameters, velocity

### Points Per Step of Arc Length
- **Variable**: Depends on velocity
- **Formula**: `points_per_step = sampleRate / velocity`
- **Example**: At 5000 steps/sec → 1 point per step
- **Example**: At 10000 steps/sec → 0.5 points per step

### Angular Resolution

**Points per radian**:
```
points_per_radian = (sampleRate × radius) / velocity
```

**Points per degree**:
```
points_per_degree = (sampleRate × radius) / (velocity × 57.3)
```

### Example Angular Resolution

For radius = 10000, velocity = 5000:
- Points per radian: (5000 × 10000) / 5000 = 10,000 points/radian
- Points per degree: 10,000 / 57.3 ≈ 174 points/degree

## Comparison: Fixed vs. Dynamic

### Fixed Point Count (NOT used)
```
Pre-calculate: 100 points
Divide arc into: 100 equal segments
Result: Fixed resolution, independent of velocity
```

### Dynamic Point Count (ACTUALLY used)
```
Calculate: One point per sample period
Density: Depends on velocity
Result: Variable resolution, constant time spacing
```

## Practical Implications

### High Velocity
- **Fewer points**: Arc traversed faster
- **Lower resolution**: Fewer calculations per arc length
- **Still smooth**: 5000 Hz provides high resolution

### Low Velocity
- **More points**: Arc traversed slower
- **Higher resolution**: More calculations per arc length
- **Very smooth**: More points = smoother motion

### Constant Sample Rate
- **Fixed timing**: 200 µs between points
- **Predictable**: Always 5000 points/second
- **Real-time**: No pre-computation needed

## Summary

**Number of Points**: Dynamic, calculated as:
```
Points = (Arc Length / Velocity) × Sample Rate
       = (radius × angle_span / velocity) × 5000
```

**Key Points**:
- ✅ One point calculated per sample period (200 µs)
- ✅ Total points depend on arc length and velocity
- ✅ Not pre-divided into fixed number of segments
- ✅ Points calculated in real-time, on-the-fly
- ✅ Sample rate is constant (5000 Hz)

The arc is developed point-by-point in real-time, with the number of points determined dynamically by the arc's length and the programmed velocity.
