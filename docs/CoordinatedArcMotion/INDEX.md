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

# Coordinated Motion System Documentation Index

## Main Documentation

- **[README.md](README.md)** - Complete theory of operation and usage guide
  - Comprehensive overview
  - Detailed theory of operation
  - Unit conversion system
  - Linear coordinated motion
  - Mixed motion chaining
  - Complete API reference
  - Usage examples
  - Troubleshooting guide

## Additional Documentation Files

The following detailed documentation files provide in-depth information on specific topics:

### Design and Architecture
- **DESIGN_COORDINATED_ARCS.md** - Complete design guide
  - Architecture overview
  - Component design
  - Integration details
  - Performance considerations

- **IMPLEMENTATION_SUMMARY.md** - Implementation overview
  - Files created/modified
  - Key features
  - API usage
  - Testing recommendations

### Technical Details
- **MOTION_SMOOTHNESS.md** - Motion smoothness analysis
  - Smoothness characteristics
  - Performance factors
  - Quality examples
  - Recommendations

- **CONSTANT_PATH_VELOCITY.md** - Velocity control details
  - Tangential velocity implementation
  - Motor speed adaptation
  - Mathematical foundation
  - Verification methods

- **ARC_PATH_DEVELOPMENT.md** - Path generation explanation
  - How arc is developed
  - Point-by-point calculation
  - Comparison with chord-based methods

- **ARC_POINT_COUNT.md** - Point count calculations
  - Dynamic point generation
  - Formula and examples
  - Resolution analysis

- **BRESENHAM_COMPARISON.md** - Algorithm comparison
  - Parametric vs. Bresenham
  - Trade-offs
  - Why parametric was chosen

- **UNIT_CONVERSION_DESIGN.md** - Unit conversion system design
  - Architecture overview
  - Supported units
  - Conversion formulas
  - Implementation details
  - Usage examples

## Quick Start

1. Read **[README.md](README.md)** for complete overview
2. Review **DESIGN_COORDINATED_ARCS.md** for architecture details
3. Check **CONSTANT_PATH_VELOCITY.md** for velocity control understanding
4. See examples in README.md Usage Guide section

## Code Examples

Example applications:
- **CoordinatedArcMoves** - Arc motion example
  - `Microchip_Examples/ClearPathModeExamples/ClearPath-SD_Series/CoordinatedArcMoves/CoordinatedArcMoves.cpp`
  
- **CoordinatedMovesWithUnits** - Comprehensive example with units and linear moves
  - `Microchip_Examples/ClearPathModeExamples/ClearPath-SD_Series/CoordinatedMovesWithUnits/CoordinatedMovesWithUnits.cpp`
  - Demonstrates single motor moves with units
  - Shows coordinated linear and arc moves
  - Includes feed rate programming in physical units

## Support

For questions or issues, contact:
**Adam G. Sweeney** <agsweeney@gmail.com>
