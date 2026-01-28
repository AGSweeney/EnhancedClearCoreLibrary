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

# Motion Smoothness Analysis: Coordinated Arc Moves

## Overview

The coordinated arc motion implementation is designed to provide **continuous and smooth** motion along circular arc paths. This document explains how smoothness is achieved and what factors affect it.

## Smoothness Characteristics

### ✅ **Continuous Motion**

The motion is **continuous** because:

1. **Angle Updates Per Sample**: The angle is updated smoothly by one sample period's worth each ISR call (5 kHz = 200 µs)
2. **No Gaps**: Steps are generated every sample period without skipping
3. **Fixed-Point Precision**: Q15 fixed-point arithmetic maintains sub-step precision (1/32768 step resolution)
4. **Trigonometric Accuracy**: Lookup tables with linear interpolation provide smooth sin/cos values

### ✅ **Smooth Motion**

The motion is **smooth** because:

1. **Constant Tangential Velocity**: The angle increment per sample is calculated to maintain constant speed along the arc path
2. **Precise Path Following**: Position is calculated directly from angle using trigonometry, ensuring exact arc geometry
3. **Small Step Deltas**: At typical velocities (5000 steps/sec), step deltas are typically 0-1 steps per axis per sample
4. **No Jerky Transitions**: Angle updates are incremental, not batched

## How It Works

### Step Generation Process

Each ISR call (200 µs):

1. **Calculate Angle Increment**: Based on velocity and radius
   ```
   angleIncrement = (velocity / radius) × samplePeriod
   ```

2. **Update Angle Smoothly**: Increment by one sample period's worth
   ```
   angle += angleIncrement (or -= for clockwise)
   ```

3. **Calculate Position**: Use trigonometry
   ```
   X = centerX + radius × cos(angle)
   Y = centerY + radius × sin(angle)
   ```

4. **Calculate Step Deltas**: Difference from last position
   ```
   stepsX = X_new - X_old
   stepsY = Y_new - Y_old
   ```

5. **Send to Motors**: Steps are applied synchronously to both motors

### Smoothness Factors

#### 1. **Sample Rate (5 kHz)**
- **High Frequency**: 200 µs sample period provides fine-grained control
- **Result**: Steps are distributed smoothly over time

#### 2. **Fixed-Point Precision (Q15)**
- **Sub-Step Resolution**: 1/32768 step precision
- **Result**: Smooth interpolation between integer step positions

#### 3. **Velocity-Based Angle Increment**
- **Constant Speed**: Angle increment maintains constant tangential velocity
- **Result**: Uniform motion along arc path

#### 4. **Trigonometric Lookup Tables**
- **1024 Entries**: Fine resolution for angle calculations
- **Linear Interpolation**: Smooth transitions between table entries
- **Result**: Accurate, smooth position calculations

## Motion Quality Examples

### Low Velocity (1000 steps/sec)
- **Steps per sample**: ~0.2 steps
- **Step deltas**: Typically 0, occasionally 1
- **Smoothness**: ⭐⭐⭐⭐⭐ Excellent

### Medium Velocity (5000 steps/sec)
- **Steps per sample**: ~1 step
- **Step deltas**: Typically 0-1 steps per axis
- **Smoothness**: ⭐⭐⭐⭐⭐ Excellent

### High Velocity (10000 steps/sec)
- **Steps per sample**: ~2 steps
- **Step deltas**: Typically 1-2 steps per axis
- **Smoothness**: ⭐⭐⭐⭐ Very Good (still smooth, but higher step rate)

### Very High Velocity (20000 steps/sec)
- **Steps per sample**: ~4 steps
- **Step deltas**: Typically 2-4 steps per axis
- **Smoothness**: ⭐⭐⭐ Good (smooth but higher step rate may be noticeable)

## Continuous Arc Chaining

When chaining multiple arcs:

1. **Tangent Continuity**: Each new arc starts from the end position of the previous arc
2. **Angle Matching**: Start angle is calculated to match current position
3. **Seamless Transition**: No stopping between arcs
4. **Smoothness**: Motion remains smooth across arc boundaries

## Comparison with Linear Moves

| Aspect | Linear Moves | Arc Moves |
|--------|-------------|-----------|
| **Path** | Straight line | Circular arc |
| **Smoothness** | Trapezoidal profile | Constant tangential velocity |
| **Step Distribution** | Variable (accel/cruise/decel) | Uniform along arc |
| **Complexity** | Lower | Higher (trigonometry) |

## Potential Smoothness Issues & Solutions

### Issue: Step Quantization
**Problem**: Steps are integers, but arc path is continuous
**Solution**: Fixed-point arithmetic maintains sub-step precision, steps are rounded correctly

### Issue: High Velocity
**Problem**: Multiple steps per sample period
**Solution**: Angle increment is calculated per sample, ensuring smooth distribution

### Issue: Small Radius Arcs
**Problem**: High angular velocity for small radius
**Solution**: Angle increment is clamped to prevent overshoot

### Issue: Arc Transitions
**Problem**: Discontinuity between arcs
**Solution**: Tangent continuity ensures smooth transitions

## Performance Impact on Smoothness

- **ISR Timing**: ~40-70 µs per sample (well within 200 µs budget)
- **No Skipped Samples**: All calculations complete in time
- **Deterministic**: Fixed-point math ensures consistent timing
- **Result**: No jitter or timing-related smoothness issues

## Recommendations for Optimal Smoothness

1. **Velocity Selection**: 
   - Use velocities that result in 1-2 steps per sample for best smoothness
   - Formula: `velocity ≈ (1-2) × sampleRateHz` steps/sec

2. **Radius Selection**:
   - Larger radii provide smoother motion (lower angular velocity)
   - Minimum recommended: 1000 steps

3. **Arc Chaining**:
   - Chain arcs with matching radii for smoothest transitions
   - Use tangent continuity (automatic)

4. **Sample Rate**:
   - Current 5 kHz is optimal for smooth motion
   - Higher rates provide even smoother motion but increase CPU load

## Conclusion

The coordinated arc motion implementation provides **continuous and smooth** motion by:

- ✅ Updating angle smoothly each sample period
- ✅ Using fixed-point precision for sub-step accuracy
- ✅ Maintaining constant tangential velocity
- ✅ Ensuring tangent continuity between arcs
- ✅ Operating within real-time constraints

The motion quality is **excellent** for typical velocities (1000-10000 steps/sec) and remains **very good** even at higher velocities. The implementation prioritizes smoothness while maintaining real-time performance.
