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
      m_remainY(0.0),
      m_startAngleRad(0.0),
      m_spanRad(0.0),
      m_sweptRad(0.0),
      m_done(true),
      m_haveExactEnd(false),
      m_endXSteps(0),
      m_endYSteps(0) {
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
    
    // Arc length (steps) = radius * span in radians. Q15 angles are fractions
    // of a full turn (TWO_PI_QX = 32768 = 2*pi), so span_rad = spanQx * 2*pi / 32768.
    // (Fix 2026-10-31: the 2*pi factor was missing, so the accel/decel ramps
    // below saw arcs 2*pi times shorter than they are and capped the speed at
    // sqrt(a*r/2) on a half circle.)
    // Exact sweep in double radians; the Q15 span above is kept for the
    // fields that still report it. Equal start and end angles mean a full turn.
    const double twoPi = 2.0 * M_PI;
    double a0 = fmod(startAngle, twoPi);
    double a1 = fmod(endAngle, twoPi);
    if (a0 < 0.0) a0 += twoPi;
    if (a1 < 0.0) a1 += twoPi;
    double spanRad = clockwise ? (a0 - a1) : (a1 - a0);
    if (spanRad <= 1e-12) {
        spanRad += twoPi;
    }
    uint32_t totalSteps = (uint32_t)((double)radius * spanRad + 0.5);
    
    if (totalSteps == 0) {
        return false;
    }
    
    // Initialize arc segment
    m_currentArc.centerXQx = ((int64_t)centerX) << FRACT_BITS;
    m_currentArc.centerYQx = ((int64_t)centerY) << FRACT_BITS;
    m_currentArc.radiusQx = ((int64_t)radius) << FRACT_BITS;
    m_currentArc.startAngleQx = startAngleQx;
    m_currentArc.endAngleQx = endAngleQx;
    m_currentArc.spanQx = angleSpanQx;
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
    
    double cx = (double)centerX, cy = (double)centerY, r = (double)radius;
    m_lastXExact = cx + r * cos(startAngle);
    m_lastYExact = cy + r * sin(startAngle);
    m_lastXSteps = (int32_t)llround(m_lastXExact);
    m_lastYSteps = (int32_t)llround(m_lastYExact);
    m_startAngleRad = a0;
    m_spanRad = spanRad;
    m_sweptRad = 0.0;
    m_done = false;
    m_haveExactEnd = false;
    m_endXSteps = (int32_t)llround(cx + r * cos(endAngle));
    m_endYSteps = (int32_t)llround(cy + r * sin(endAngle));
    
    return true;
}

bool ArcInterpolator::GenerateNextSteps(int32_t &stepsX, int32_t &stepsY) {
    // 2026-10-31 rewrite: the sweep is tracked in double radians from the
    // exact start angle, so the arc length, the accel/decel ramps and the end
    // point no longer depend on the Q15 angle resolution (one Q15 unit is
    // ~4.6 steps of arc at r = 24000 steps). The last sample lands on the end
    // point exactly (m_endXSteps/m_endYSteps).
    stepsX = 0;
    stepsY = 0;
    if (m_done) {
        return false;
    }

    double r = (double)(m_currentArc.radiusQx >> FRACT_BITS);
    if (r < 1.0) {
        r = 1.0;
    }
    const double remainingRad = m_spanRad - m_sweptRad;
    if (remainingRad <= 0.0) {
        m_done = true;
        m_currentArc.stepsRemaining = 0;
        return false;
    }

    // Accel/decel-limited tangential speed (steps/s).
    const double distFromStart = r * m_sweptRad;
    const double remainingDist = r * remainingRad;
    double targetSpeed = (double)m_velocityMax;
    if (m_accelMax > 0) {
        const double fromStart = sqrt((double)m_entrySpeed * (double)m_entrySpeed +
                                      2.0 * (double)m_accelMax * distFromStart);
        const double fromEnd = sqrt((double)m_exitSpeed * (double)m_exitSpeed +
                                    2.0 * (double)m_accelMax * remainingDist);
        if (fromStart < targetSpeed) {
            targetSpeed = fromStart;
        }
        if (fromEnd < targetSpeed) {
            targetSpeed = fromEnd;
        }
    }
    // Avoid a long crawl at the very start/end: never below 1/20 of feed.
    double minTargetSpeed = (m_velocityMax > 0) ? (double)(m_velocityMax / 20) : 1.0;
    if (minTargetSpeed < 1.0) {
        minTargetSpeed = 1.0;
    }
    if (targetSpeed < minTargetSpeed) {
        targetSpeed = minTargetSpeed;
    }

    double angleInc = targetSpeed / (r * (double)m_sampleRateHz);
    const bool last = (angleInc >= remainingRad);
    if (last) {
        m_sweptRad = m_spanRad;
    } else {
        m_sweptRad += angleInc;
    }
    const double angle = m_startAngleRad + (m_currentArc.clockwise ? -m_sweptRad : m_sweptRad);

    const double cx = (double)(m_currentArc.centerXQx >> FRACT_BITS);
    const double cy = (double)(m_currentArc.centerYQx >> FRACT_BITS);
    const double xExact = cx + r * cos(angle);
    const double yExact = cy + r * sin(angle);
    m_remainX += xExact - m_lastXExact;
    m_remainY += yExact - m_lastYExact;
    m_lastXExact = xExact;
    m_lastYExact = yExact;
    int32_t outX = (int32_t)(m_remainX >= 0.0 ? m_remainX + 0.5 : m_remainX - 0.5);
    int32_t outY = (int32_t)(m_remainY >= 0.0 ? m_remainY + 0.5 : m_remainY - 0.5);
    m_remainX -= (double)outX;
    m_remainY -= (double)outY;

    if (last) {
        // Land on the end point. Normally this changes the last sample by a
        // step or two; the cap keeps a wrong end point from becoming a burst.
        const int32_t kMaxEndFix = 4;
        const int32_t fixX = m_endXSteps - m_lastXSteps;
        const int32_t fixY = m_endYSteps - m_lastYSteps;
        if (fixX - outX <= kMaxEndFix && outX - fixX <= kMaxEndFix) {
            outX = fixX;
        }
        if (fixY - outY <= kMaxEndFix && outY - fixY <= kMaxEndFix) {
            outY = fixY;
        }
        m_remainX = 0.0;
        m_remainY = 0.0;
        m_done = true;
    }

    m_lastXSteps += outX;
    m_lastYSteps += outY;
    stepsX = outX;
    stepsY = outY;

    // Keep the Q15 mirrors current for code that still reads them.
    const double twoPi = 2.0 * M_PI;
    double norm = fmod(angle, twoPi);
    if (norm < 0.0) {
        norm += twoPi;
    }
    m_currentAngleRad = norm;
    m_currentAngleQx = NormalizeAngleQx((int32_t)(norm * (32768.0 / twoPi)));
    m_currentXQx = ((int64_t)m_lastXSteps) << FRACT_BITS;
    m_currentYQx = ((int64_t)m_lastYSteps) << FRACT_BITS;
    m_currentArc.stepsRemaining = last ? 0u : (uint32_t)(r * (m_spanRad - m_sweptRad));
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
    m_startAngleRad = 0.0;
    m_spanRad = 0.0;
    m_sweptRad = 0.0;
    m_done = true;
    m_haveExactEnd = false;
    m_endXSteps = 0;
    m_endYSteps = 0;
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
