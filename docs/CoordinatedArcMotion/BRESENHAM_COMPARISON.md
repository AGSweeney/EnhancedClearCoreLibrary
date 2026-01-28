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

# Bresenham vs. Current Implementation

## Current Implementation: Parametric/Trigonometric Approach

### How It Works
```cpp
1. Increment angle: angle += angleIncrement
2. Calculate position: X = centerX + radius × cos(angle)
                      Y = centerY + radius × sin(angle)
3. Calculate step deltas: stepsX = X_new - X_old
                          stepsY = Y_new - Y_old
```

### Characteristics
- ✅ Uses trigonometry (sin/cos lookup tables)
- ✅ Angle-based parameterization
- ✅ Supports variable velocity (constant tangential speed)
- ✅ Flexible for partial arcs and arbitrary start/end angles
- ❌ Requires trigonometric calculations
- ❌ More computationally intensive

## Bresenham's Circle Algorithm

### How It Works
```cpp
1. Initialize error term: error = 0
2. For each step:
   a. Decide next move (X+1, Y+1, or both) based on error
   b. Update error term
   c. Output step
```

### Characteristics
- ✅ Integer-only arithmetic (no trigonometry)
- ✅ Very efficient (minimal calculations)
- ✅ Deterministic step-by-step decisions
- ❌ Typically generates one octant at a time
- ❌ Less flexible for variable velocity
- ❌ Harder to maintain constant tangential speed

## Comparison

| Aspect | Current (Parametric) | Bresenham |
|--------|---------------------|-----------|
| **Method** | Angle increment + trig | Error-based decision |
| **Math** | sin/cos lookup tables | Integer arithmetic only |
| **Speed** | Moderate (trig lookup) | Very fast (no trig) |
| **Velocity Control** | Easy (angle increment) | Difficult (step-based) |
| **Flexibility** | High (any angle range) | Lower (octant-based) |
| **Accuracy** | High (trig precision) | High (integer precision) |
| **Constant Path Speed** | Natural fit | Requires adaptation |

## Why Current Approach Was Chosen

### 1. **Constant Tangential Velocity Requirement**
- Bresenham generates steps at a fixed rate (one per iteration)
- Our requirement: constant speed along arc path
- Parametric approach naturally supports this via angle increment

### 2. **Velocity Control**
- Angle increment = `(velocity / radius) × samplePeriod`
- Directly controls tangential speed
- Bresenham would require complex modifications

### 3. **Flexibility**
- Supports arbitrary start/end angles
- Supports partial arcs
- Bresenham typically works in octants (45° segments)

### 4. **Real-Time Performance**
- Trig lookup tables are fast (~5-10 µs)
- Well within ISR timing budget (200 µs)
- Bresenham would be faster but not necessary

## Could We Use Bresenham?

### Possible but Challenging

**Advantages**:
- Faster execution (no trig needed)
- Pure integer arithmetic
- Lower memory (no lookup tables)

**Challenges**:
1. **Velocity Control**: Bresenham generates one step per iteration. To maintain constant tangential speed, we'd need to:
   - Calculate how many Bresenham iterations per sample period
   - Batch steps or skip iterations
   - Complex velocity management

2. **Octant Handling**: Bresenham works best for 45° octants. For arbitrary arcs:
   - Need to handle octant transitions
   - More complex code

3. **Angle-Based Control**: Our system needs to:
   - Start/stop at specific angles
   - Chain arcs with tangent continuity
   - Easier with angle-based approach

## Hybrid Approach (Future Enhancement)

A hybrid could combine benefits:
```
1. Use Bresenham for step generation (efficient)
2. Track angle separately (for velocity control)
3. Use angle to determine Bresenham step rate
```

This would be more complex but potentially faster.

## Conclusion

**Current Implementation**: Parametric/Trigonometric
- ✅ Best fit for constant tangential velocity requirement
- ✅ Flexible and maintainable
- ✅ Fast enough for real-time (trig lookup ~5-10 µs)
- ✅ Natural fit for velocity control

**Bresenham**: Would be faster but:
- ❌ Harder to maintain constant tangential speed
- ❌ Less flexible for arbitrary arcs
- ❌ More complex code
- ❌ Not necessary (current approach is fast enough)

The current parametric approach is the right choice for this application because it naturally supports constant tangential velocity while maintaining good performance.
