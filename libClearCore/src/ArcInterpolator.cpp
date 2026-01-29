/*
 * Copyright (c) 2026 Adam G. Sweeney <agsweeney@gmail.com>
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#include "ArcInterpolator.h"
#include "TrigLUT.h"
#include "SysTiming.h"
#include <math.h>
#include <stdlib.h>  // for abs()

namespace ClearCore {

ArcInterpolator::ArcInterpolator()
    : m_currentAngleQx(0),
      m_currentXQx(0),
      m_currentYQx(0),
      m_lastXSteps(0),
      m_lastYSteps(0),
      m_angleIncrementQx(0),
      m_velocityMax(0),
      m_accelMax(0),
      m_sampleRateHz(0),
      m_entrySpeed(0),
      m_exitSpeed(0),
      m_angleFracRad(0.0),
      m_currentAngleRad(0.0),
      m_lastXExact(0.0),
      m_lastYExact(0.0),
      m_remainX(0.0),
      m_remainY(0.0) {
    Reset();
}

bool ArcInterpolator::InitializeArc(int32_t centerX, int32_t centerY,
                                     int32_t radius,
                                     double startAngle, double endAngle,
                                     bool clockwise,
                                     uint32_t velocityMax, uint32_t accelMax,
                                     uint16_t sampleRateHz,
                                     uint32_t entrySpeed,
                                     uint32_t exitSpeed) {
    // Validate parameters
    if (radius <= 0) {
        return false;
    }
    
    // Convert angles to Q15 format
    int32_t startAngleQx = AngleToQx(startAngle);
    int32_t endAngleQx = AngleToQx(endAngle);
    
    // Normalize angles
    startAngleQx = NormalizeAngleQx(startAngleQx);
    endAngleQx = NormalizeAngleQx(endAngleQx);
    
    // Calculate arc angle span
    // For clockwise: angle decreases (270° → 0° wraps around, so 270° → 360° → 0° = 90°)
    // For counterclockwise: angle increases (0° → 270° = 270°)
    int32_t angleSpanQx;
    if (clockwise) {
        // Clockwise: decreasing angle
        if (endAngleQx < startAngleQx) {
            // Wraps around: going from larger angle to smaller (e.g., 270° → 0°)
            // Span = (2π - start) + end = 2π - (start - end)
            angleSpanQx = TWO_PI_QX - (startAngleQx - endAngleQx);
        } else {
            // Clockwise with end > start means we wrap through 2π
            angleSpanQx = TWO_PI_QX - (endAngleQx - startAngleQx);
        }
    } else {
        // Counterclockwise: increasing angle
        if (endAngleQx > startAngleQx) {
            // Normal case: end > start, span = end - start
            angleSpanQx = endAngleQx - startAngleQx;
        } else {
            // Wraps around: end < start (e.g., 0° → 270°)
            // Span = (2π - start) + end = 2π - (start - end)
            angleSpanQx = TWO_PI_QX - (startAngleQx - endAngleQx);
        }
    }
    
    // Calculate arc length and total steps
    // Arc length = radius * angle (in radians)
    // angleSpanQx is in Q15, convert to radians: angleSpanQx / 32768 * 2π
    // But we want steps, so: steps = radius * angleSpanQx / 32768
    int64_t arcLengthQx = ((int64_t)radius * angleSpanQx) >> 15;
    uint32_t totalSteps = (uint32_t)arcLengthQx;
    
    if (totalSteps == 0) {
        return false;
    }
    
    // Initialize arc segment
    m_currentArc.centerXQx = ((int64_t)centerX) << FRACT_BITS;
    m_currentArc.centerYQx = ((int64_t)centerY) << FRACT_BITS;
    m_currentArc.radiusQx = ((int64_t)radius) << FRACT_BITS;
    m_currentArc.startAngleQx = startAngleQx;
    m_currentArc.endAngleQx = endAngleQx;
    m_currentArc.clockwise = clockwise;
    m_currentArc.totalSteps = totalSteps;
    m_currentArc.stepsRemaining = totalSteps;
    
    // Calculate angle increment per step
    // angleIncrement = angleSpan / totalSteps
    m_angleIncrementQx = angleSpanQx / totalSteps;
    if (m_angleIncrementQx == 0) {
        m_angleIncrementQx = 1; // Minimum increment
    }
    
    // Store velocity and sample rate for path speed control
    // velocityMax is the TANGENTIAL velocity along the arc path (steps/sec)
    // This is the speed the tool will travel along the arc circumference
    // Individual motor speeds will adapt automatically based on their position
    m_velocityMax = velocityMax;
    m_accelMax = accelMax;
    m_sampleRateHz = sampleRateHz;
    m_entrySpeed = entrySpeed;
    m_exitSpeed = exitSpeed;
    m_angleFracRad = 0.0;
    m_remainX = 0.0;
    m_remainY = 0.0;

    // Initialize current state
    m_currentAngleQx = startAngleQx;
    m_currentAngleRad = startAngle;
    
    // Calculate initial position (steps)
    int32_t cosVal = CosQx(m_currentAngleQx);
    int32_t sinVal = SinQx(m_currentAngleQx);
    m_currentXQx = m_currentArc.centerXQx + 
                   (((int64_t)m_currentArc.radiusQx * cosVal) >> 15);
    m_currentYQx = m_currentArc.centerYQx + 
                   (((int64_t)m_currentArc.radiusQx * sinVal) >> 15);
    
    m_lastXSteps = (int32_t)(m_currentXQx >> FRACT_BITS);
    m_lastYSteps = (int32_t)(m_currentYQx >> FRACT_BITS);
    double cx = (double)centerX, cy = (double)centerY, r = (double)radius;
    m_lastXExact = cx + r * cos(startAngle);
    m_lastYExact = cy + r * sin(startAngle);
    
    return true;
}

bool ArcInterpolator::GenerateNextSteps(int32_t &stepsX, int32_t &stepsY) {
    // Check if arc is complete by angle (more reliable than step count)
    // Calculate remaining angle in the direction of travel
    // Use the same logic as angle span calculation to ensure consistency
    int32_t angleRemainingQx;
    if (m_currentArc.clockwise) {
        // Clockwise: angle decreases
        if (m_currentArc.endAngleQx < m_currentAngleQx) {
            // Normal clockwise case
            angleRemainingQx = m_currentAngleQx - m_currentArc.endAngleQx;
        } else {
            // Wrap around through 2π
            angleRemainingQx = TWO_PI_QX - (m_currentArc.endAngleQx - m_currentAngleQx);
        }
    } else {
        // Counterclockwise: angle increases
        if (m_currentArc.endAngleQx > m_currentAngleQx) {
            // Normal case: end > current
            angleRemainingQx = m_currentArc.endAngleQx - m_currentAngleQx;
        } else {
            // Wraps around: end < current
            angleRemainingQx = TWO_PI_QX - (m_currentAngleQx - m_currentArc.endAngleQx);
        }
    }
    
    // Update remaining steps estimate from angle remaining
    int64_t radiusSteps = m_currentArc.radiusQx >> FRACT_BITS;
    if (radiusSteps == 0) {
        radiusSteps = 1;
    }
    uint32_t stepsRemainingEstimate = (uint32_t)(((int64_t)radiusSteps * angleRemainingQx) >> 15);
    m_currentArc.stepsRemaining = stepsRemainingEstimate;

    // Already at end (should not generate more steps)
    if (angleRemainingQx <= 0) {
        stepsX = 0;
        stepsY = 0;
        return false;
    }

    // CRITICAL: Maintain TANGENTIAL velocity along the arc path
    // The speed along the arc circumference must be constant (velocityMax steps/sec)
    // Individual motor speeds will adapt automatically based on their position on the arc
    
    // Calculate angular velocity from tangential velocity:
    // Tangential velocity: v = r * ω  (steps/sec)
    // Angular velocity: ω = v / r    (rad/sec)
    // Angle increment per sample: dθ = ω * dt = (v / r) * (1 / sampleRateHz)
    
    // Calculate accel/decel-limited tangential speed
    uint32_t distFromStart = (m_currentArc.totalSteps > m_currentArc.stepsRemaining) ?
                              (m_currentArc.totalSteps - m_currentArc.stepsRemaining) : 0;
    uint32_t remainingDist = m_currentArc.stepsRemaining;
    double targetSpeed = (double)m_velocityMax;
    if (m_accelMax > 0) {
        double maxSpeedFromStart = sqrt((double)m_entrySpeed * (double)m_entrySpeed +
                                        2.0 * (double)m_accelMax * (double)distFromStart);
        double maxSpeedFromEnd = sqrt((double)m_exitSpeed * (double)m_exitSpeed +
                                      2.0 * (double)m_accelMax * (double)remainingDist);
        if (maxSpeedFromStart < targetSpeed) {
            targetSpeed = maxSpeedFromStart;
        }
        if (maxSpeedFromEnd < targetSpeed) {
            targetSpeed = maxSpeedFromEnd;
        }
    }
    // Avoid long start delay: use a fraction of feed when accel-limited to very low speed.
    double minTargetSpeed = (m_velocityMax > 0) ? (double)(m_velocityMax / 20) : 1.0;
    if (minTargetSpeed < 1.0) {
        minTargetSpeed = 1.0;
    }
    if (targetSpeed < minTargetSpeed) {
        targetSpeed = minTargetSpeed;
    }

    // Angle increment per sample: dθ (rad) = targetSpeed / (radius * sampleRate).
    double angleIncRad = (double)targetSpeed / ((double)radiusSteps * (double)m_sampleRateHz);
    double angleRemainingRad = (double)angleRemainingQx * (2.0 * M_PI / 32768.0);
    if (angleIncRad > angleRemainingRad) {
        angleIncRad = angleRemainingRad;
    }
    if (m_currentArc.clockwise) {
        angleIncRad = -angleIncRad;
    }

    // Advance angle every sample (no "wait for 1 Q15" — eliminates start delay).
    m_currentAngleRad += angleIncRad;
    const double twoPi = 2.0 * M_PI;
    while (m_currentAngleRad < 0.0) {
        m_currentAngleRad += twoPi;
    }
    while (m_currentAngleRad >= twoPi) {
        m_currentAngleRad -= twoPi;
    }

    m_currentAngleQx = (int32_t)(m_currentAngleRad * (32768.0 / twoPi));
    m_currentAngleQx = NormalizeAngleQx(m_currentAngleQx);

    // Exact position (steps) from continuous angle for fractional step accumulation.
    double cx = (double)(m_currentArc.centerXQx >> FRACT_BITS);
    double cy = (double)(m_currentArc.centerYQx >> FRACT_BITS);
    double r = (double)(m_currentArc.radiusQx >> FRACT_BITS);
    double xExact = cx + r * cos(m_currentAngleRad);
    double yExact = cy + r * sin(m_currentAngleRad);
    double deltaX = xExact - m_lastXExact;
    double deltaY = yExact - m_lastYExact;
    m_lastXExact = xExact;
    m_lastYExact = yExact;
    m_remainX += deltaX;
    m_remainY += deltaY;
    int32_t outX = (int32_t)(m_remainX >= 0.0 ? m_remainX + 0.5 : m_remainX - 0.5);
    int32_t outY = (int32_t)(m_remainY >= 0.0 ? m_remainY + 0.5 : m_remainY - 0.5);
    m_remainX -= (double)outX;
    m_remainY -= (double)outY;

    // Update Q15 position for remaining/clamp logic
    int64_t newXQx = m_currentArc.centerXQx +
                     (((int64_t)m_currentArc.radiusQx * CosQx(m_currentAngleQx)) >> 15);
    int64_t newYQx = m_currentArc.centerYQx +
                     (((int64_t)m_currentArc.radiusQx * SinQx(m_currentAngleQx)) >> 15);
    m_currentXQx = newXQx;
    m_currentYQx = newYQx;

    int32_t angleRemainingQxPost;
    if (m_currentArc.clockwise) {
        if (m_currentArc.endAngleQx < m_currentAngleQx) {
            angleRemainingQxPost = m_currentAngleQx - m_currentArc.endAngleQx;
        } else {
            angleRemainingQxPost = TWO_PI_QX - (m_currentArc.endAngleQx - m_currentAngleQx);
        }
    } else {
        if (m_currentArc.endAngleQx > m_currentAngleQx) {
            angleRemainingQxPost = m_currentArc.endAngleQx - m_currentAngleQx;
        } else {
            angleRemainingQxPost = TWO_PI_QX - (m_currentAngleQx - m_currentArc.endAngleQx);
        }
    }
    m_currentArc.stepsRemaining = (uint32_t)(((int64_t)radiusSteps * angleRemainingQxPost) >> 15);

    const int32_t ANGLE_DONE_QX = 2;
    if (angleRemainingQxPost <= ANGLE_DONE_QX) {
        int32_t cosEnd = CosQx(m_currentArc.endAngleQx);
        int32_t sinEnd = SinQx(m_currentArc.endAngleQx);
        int64_t endXQx = m_currentArc.centerXQx + (((int64_t)m_currentArc.radiusQx * cosEnd) >> 15);
        int64_t endYQx = m_currentArc.centerYQx + (((int64_t)m_currentArc.radiusQx * sinEnd) >> 15);
        int32_t endXSteps = (int32_t)(endXQx >> FRACT_BITS);
        int32_t endYSteps = (int32_t)(endYQx >> FRACT_BITS);
        stepsX = endXSteps - m_lastXSteps;
        stepsY = endYSteps - m_lastYSteps;
        m_lastXSteps = endXSteps;
        m_lastYSteps = endYSteps;
        m_currentXQx = endXQx;
        m_currentYQx = endYQx;
        m_currentAngleQx = m_currentArc.endAngleQx;
        m_currentAngleRad = (double)m_currentArc.endAngleQx * (twoPi / 32768.0);
        m_lastXExact = (double)endXSteps;
        m_lastYExact = (double)endYSteps;
        m_remainX = 0.0;
        m_remainY = 0.0;
        m_currentArc.stepsRemaining = 0;
        if (stepsX == 0 && stepsY == 0) {
            return false;
        }
        return true;
    }

    stepsX = outX;
    stepsY = outY;
    m_lastXSteps += outX;
    m_lastYSteps += outY;
    return true;
}

void ArcInterpolator::GetCurrentPosition(int32_t &posX, int32_t &posY) const {
    posX = m_lastXSteps;
    posY = m_lastYSteps;
}

void ArcInterpolator::Reset() {
    m_currentArc = ArcSegment();
    m_currentAngleQx = 0;
    m_currentXQx = 0;
    m_currentYQx = 0;
    m_lastXSteps = 0;
    m_lastYSteps = 0;
    m_angleIncrementQx = 0;
    m_velocityMax = 0;
    m_accelMax = 0;
    m_sampleRateHz = 0;
    m_entrySpeed = 0;
    m_exitSpeed = 0;
    m_angleFracRad = 0.0;
    m_currentAngleRad = 0.0;
    m_lastXExact = 0.0;
    m_lastYExact = 0.0;
    m_remainX = 0.0;
    m_remainY = 0.0;
}

int32_t ArcInterpolator::AngleToQx(double angleRad) {
    // Convert radians to Q15 format
    // Q15: 32768 = 2π radians
    // angleQx = angleRad * 32768 / (2π)
    const double scale = 32768.0 / (2.0 * M_PI);
    return (int32_t)(angleRad * scale);
}

int32_t ArcInterpolator::NormalizeAngleQx(int32_t angleQx) {
    // Normalize to 0-32767 range (0 to 2π)
    while (angleQx < 0) {
        angleQx += TWO_PI_QX;
    }
    while (angleQx >= TWO_PI_QX) {
        angleQx -= TWO_PI_QX;
    }
    return angleQx;
}

int32_t ArcInterpolator::CalculateAngleIncrement(int64_t radiusQx,
                                                 uint32_t velocityMax,
                                                 uint16_t sampleRateHz) {
    // Calculate angle increment per sample period
    // For constant velocity: angleIncrement = velocity / radius
    // In Q15: angleIncrementQx = (velocityMax * 32768) / (radiusQx * sampleRateHz)
    
    if (radiusQx == 0 || sampleRateHz == 0) {
        return 1; // Minimum increment
    }
    
    int64_t velocityQx = ((int64_t)velocityMax) << FRACT_BITS;
    int64_t angleIncrementQx = (velocityQx * TWO_PI_QX) / 
                                ((radiusQx >> FRACT_BITS) * sampleRateHz);
    
    if (angleIncrementQx < 1) {
        return 1;
    }
    if (angleIncrementQx > TWO_PI_QX) {
        return TWO_PI_QX;
    }
    
    return (int32_t)angleIncrementQx;
}

} // ClearCore namespace
