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
      m_sampleRateHz(0) {
    Reset();
}

bool ArcInterpolator::InitializeArc(int32_t centerX, int32_t centerY,
                                     int32_t radius,
                                     double startAngle, double endAngle,
                                     bool clockwise,
                                     uint32_t velocityMax,
                                     uint16_t sampleRateHz) {
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
    int32_t angleSpanQx;
    if (clockwise) {
        if (endAngleQx > startAngleQx) {
            angleSpanQx = TWO_PI_QX - (endAngleQx - startAngleQx);
        } else {
            angleSpanQx = startAngleQx - endAngleQx;
        }
    } else {
        if (endAngleQx > startAngleQx) {
            angleSpanQx = endAngleQx - startAngleQx;
        } else {
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
    m_sampleRateHz = sampleRateHz;
    
    // Initialize current state
    m_currentAngleQx = startAngleQx;
    
    // Calculate initial position
    int32_t cosVal = CosQx(m_currentAngleQx);
    int32_t sinVal = SinQx(m_currentAngleQx);
    m_currentXQx = m_currentArc.centerXQx + 
                   (((int64_t)m_currentArc.radiusQx * cosVal) >> 15);
    m_currentYQx = m_currentArc.centerYQx + 
                   (((int64_t)m_currentArc.radiusQx * sinVal) >> 15);
    
    m_lastXSteps = centerX + radius; // Initial X position in steps
    m_lastYSteps = centerY;         // Initial Y position in steps
    
    return true;
}

bool ArcInterpolator::GenerateNextSteps(int32_t &stepsX, int32_t &stepsY) {
    if (m_currentArc.stepsRemaining == 0) {
        stepsX = 0;
        stepsY = 0;
        return false;
    }
    
    // CRITICAL: Maintain constant TANGENTIAL velocity along the arc path
    // The speed along the arc circumference must be constant (velocityMax steps/sec)
    // Individual motor speeds will adapt automatically based on their position on the arc
    
    // Calculate angular velocity from tangential velocity:
    // Tangential velocity: v = r * ω  (steps/sec)
    // Angular velocity: ω = v / r    (rad/sec)
    // Angle increment per sample: dθ = ω * dt = (v / r) * (1 / sampleRateHz)
    
    int64_t radiusSteps = m_currentArc.radiusQx >> FRACT_BITS;
    if (radiusSteps == 0) {
        radiusSteps = 1; // Prevent divide by zero
    }
    
    // Calculate angle increment per sample period to maintain constant path speed
    // Formula: dθ_Q15 = (velocityMax * TWO_PI_QX) / (radiusSteps * sampleRateHz)
    // This ensures: tangential speed = radius * angular_speed = constant
    int64_t angleIncrementPerSampleQx = ((int64_t)m_velocityMax * TWO_PI_QX) / 
                                       (radiusSteps * (int64_t)m_sampleRateHz);
    
    // Ensure minimum increment
    if (angleIncrementPerSampleQx == 0) {
        angleIncrementPerSampleQx = 1; // Minimum 1 Q15 unit
    }
    
    // Limit increment to not exceed remaining arc angle
    int32_t angleRemainingQx;
    if (m_currentArc.clockwise) {
        int32_t currentToEndQx = m_currentAngleQx - m_currentArc.endAngleQx;
        if (currentToEndQx < 0) {
            currentToEndQx += TWO_PI_QX;
        }
        angleRemainingQx = currentToEndQx;
    } else {
        int32_t currentToEndQx = m_currentArc.endAngleQx - m_currentAngleQx;
        if (currentToEndQx < 0) {
            currentToEndQx += TWO_PI_QX;
        }
        angleRemainingQx = currentToEndQx;
    }
    
    // Clamp increment to remaining angle
    if ((int32_t)angleIncrementPerSampleQx > angleRemainingQx) {
        angleIncrementPerSampleQx = angleRemainingQx;
    }
    
    // Update angle smoothly by one sample period's increment
    if (m_currentArc.clockwise) {
        m_currentAngleQx -= (int32_t)angleIncrementPerSampleQx;
        if (m_currentAngleQx < 0) {
            m_currentAngleQx += TWO_PI_QX;
        }
    } else {
        m_currentAngleQx += (int32_t)angleIncrementPerSampleQx;
        if (m_currentAngleQx >= TWO_PI_QX) {
            m_currentAngleQx -= TWO_PI_QX;
        }
    }
    
    // Normalize angle to 0-2π range
    m_currentAngleQx = NormalizeAngleQx(m_currentAngleQx);
    
    // Calculate new position on arc using trigonometric functions
    int32_t cosVal = CosQx(m_currentAngleQx);
    int32_t sinVal = SinQx(m_currentAngleQx);
    
    int64_t newXQx = m_currentArc.centerXQx + 
                     (((int64_t)m_currentArc.radiusQx * cosVal) >> 15);
    int64_t newYQx = m_currentArc.centerYQx + 
                     (((int64_t)m_currentArc.radiusQx * sinVal) >> 15);
    
    // Convert fixed-point positions to integer step counts
    int32_t newXSteps = (int32_t)(newXQx >> FRACT_BITS);
    int32_t newYSteps = (int32_t)(newYQx >> FRACT_BITS);
    
    // Calculate step deltas (difference from last position)
    // These are the actual steps sent to the motors
    stepsX = newXSteps - m_lastXSteps;
    stepsY = newYSteps - m_lastYSteps;
    
    // Update state for next iteration
    m_lastXSteps = newXSteps;
    m_lastYSteps = newYSteps;
    m_currentXQx = newXQx;
    m_currentYQx = newYQx;
    
    // Update remaining steps estimate
    // Use the larger of X or Y step magnitude to estimate arc progress
    uint32_t stepsTaken = (uint32_t)(abs(stepsX) > abs(stepsY) ? abs(stepsX) : abs(stepsY));
    if (stepsTaken > m_currentArc.stepsRemaining) {
        stepsTaken = m_currentArc.stepsRemaining;
    }
    m_currentArc.stepsRemaining -= stepsTaken;
    if (m_currentArc.stepsRemaining > m_currentArc.totalSteps) {
        m_currentArc.stepsRemaining = 0; // Prevent underflow
    }
    
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
    m_sampleRateHz = 0;
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
