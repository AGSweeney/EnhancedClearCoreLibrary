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

# Arc Path Development: How the Arc is Actually Generated

## Overview

The arc is **NOT** divided into pre-calculated chords. Instead, it's developed as a **continuous parametric curve** calculated point-by-point in real-time. Each sample period, a new point on the true arc path is calculated, and step deltas are derived from the position differences.

## Method: Parametric Point-by-Point Calculation

### How It Works

The arc is developed using a **parametric approach** where:

1. **Angle Parameter**: The angle θ is the parameter that defines position along the arc
2. **Continuous Increment**: Angle is incremented smoothly each sample period
3. **Direct Calculation**: Position is calculated directly from angle using trigonometry
4. **Step Deltas**: Steps are derived from position differences

### Step-by-Step Process

Each ISR call (200 µs, 5 kHz):

```
1. Calculate angle increment:
   dθ = (velocity / radius) × samplePeriod
   
2. Update angle:
   θ_new = θ_old + dθ
   
3. Calculate position on true arc:
   X = centerX + radius × cos(θ_new)
   Y = centerY + radius × sin(θ_new)
   
4. Calculate step deltas:
   stepsX = X_new - X_old
   stepsY = Y_new - Y_old
   
5. Send steps to motors
```

## Key Characteristics

### ✅ **Not Chord-Based**

- **No pre-division**: Arc is not divided into chords beforehand
- **No straight segments**: Each point lies exactly on the true arc
- **Continuous calculation**: Points calculated on-the-fly

### ✅ **True Arc Path**

- **Mathematically exact**: Each position is calculated from the true arc equation
- **Parametric form**: Uses angle parameter θ
- **Trigonometric precision**: sin/cos lookup tables provide accurate positions

### ✅ **Step Discretization**

- **Natural discretization**: Steps occur naturally from position differences
- **High resolution**: Fixed-point Q15 provides 1/32768 step precision
- **Smooth approximation**: Step deltas closely follow the arc path

## Visual Representation

```
True Arc Path (mathematical):
    ╱╲
   ╱  ╲
  ╱    ╲
 ╱      ╲
╱        ╲

Calculated Points (each sample period):
    •─•─•
   •   •   •
  •     •     •
 •       •       •
•         •         •

Step Deltas (what motors receive):
    →↗→
   ↗  ↘  ↗
  ↗    ↘    ↗
 ↗      ↘      ↗
→         →         →
```

## Comparison: Chords vs. Parametric

### Chord-Based Approach (NOT used)
```
Pre-calculate chord endpoints:
  P0 → P1 → P2 → P3 → P4
  
Motors move in straight lines:
  ───  ───  ───  ───
  
Result: Approximated arc with straight segments
```

### Parametric Approach (ACTUALLY used)
```
Calculate points on-the-fly:
  P0 → P1 → P2 → P3 → P4
  
Each point on true arc:
  ╱╲
  
Step deltas follow arc:
  →↗→
  
Result: True arc path with step discretization
```

## Mathematical Foundation

### Arc Equation
```
X(θ) = centerX + radius × cos(θ)
Y(θ) = centerY + radius × sin(θ)
```

### Angle Increment
```
dθ = (tangential_velocity / radius) × dt
```

### Step Deltas
```
stepsX = X(θ + dθ) - X(θ)
stepsY = Y(θ + dθ) - Y(θ)
```

## Resolution and Precision

### Fixed-Point Precision
- **Q15 Format**: 15 fractional bits = 1/32768 step precision
- **Sub-step accuracy**: Positions calculated with sub-step precision
- **Rounding**: Final step counts rounded from fixed-point positions

### Sample Rate Impact
- **5 kHz sample rate**: 200 µs between calculations
- **High resolution**: ~5000 calculations per second
- **Smooth motion**: Small angle increments per sample

### Example Resolution

For a 10000-step radius arc at 5000 steps/sec:
- **Angle increment per sample**: ~0.0016 radians (0.09°)
- **Position precision**: ~0.03 steps (Q15 fixed-point)
- **Step deltas**: Typically 0-1 steps per axis per sample

## Why This Approach?

### Advantages

1. **True Arc Path**: Mathematically exact, not approximated
2. **Constant Velocity**: Natural support for constant tangential speed
3. **Flexibility**: Works for any angle range, any radius
4. **Real-time**: Calculated on-the-fly, no pre-computation needed
5. **Smooth**: Continuous angle increments provide smooth motion

### Trade-offs

- **Computational**: Requires trig calculations (mitigated by lookup tables)
- **Memory**: Needs lookup tables (2-4 KB)
- **Not Bresenham**: More complex than integer-only algorithms

## Step Delta Behavior

### Typical Step Patterns

**At 0° (right side of arc)**:
- X motor: Moving fast (cos(0°) = 1)
- Y motor: Not moving (sin(0°) = 0)
- Steps: (1, 0) or (1, 0) repeatedly

**At 45° (diagonal)**:
- X motor: Moving at 70.7% speed (cos(45°) = 0.707)
- Y motor: Moving at 70.7% speed (sin(45°) = 0.707)
- Steps: (1, 1) or (0, 1) alternating

**At 90° (top of arc)**:
- X motor: Not moving (cos(90°) = 0)
- Y motor: Moving fast (sin(90°) = 1)
- Steps: (0, 1) or (0, 1) repeatedly

### Step Distribution

Steps are distributed naturally based on:
- **Arc curvature**: Steeper curves = more steps on that axis
- **Velocity**: Higher velocity = more steps per sample
- **Radius**: Smaller radius = higher angular velocity = more steps

## Conclusion

The arc is developed as a **continuous parametric curve**, not divided into chords:

- ✅ **True arc path**: Each point calculated from arc equation
- ✅ **Point-by-point**: Calculated in real-time each sample period
- ✅ **Step discretization**: Steps derived from position differences
- ✅ **High precision**: Fixed-point arithmetic maintains accuracy
- ✅ **Smooth motion**: Continuous angle increments provide smoothness

The motors receive step deltas that naturally follow the true arc path, creating smooth, accurate motion without pre-dividing the arc into chords.
