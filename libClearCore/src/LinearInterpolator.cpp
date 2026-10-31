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

#include "LinearInterpolator.h"
#include <math.h>
#include <stdlib.h>

namespace ClearCore {

// Q15 fixed-point format (15 fractional bits)
#define Q15_SHIFT 15
#define Q15_ONE (1 << Q15_SHIFT)
#define Q15_FROM_DOUBLE(x) ((int32_t)((x) * Q15_ONE + 0.5))
#define Q15_TO_DOUBLE(x) ((double)(x) / Q15_ONE)

LinearInterpolator::LinearInterpolator()
    : m_startX(0),
      m_startY(0),
      m_endX(0),
      m_endY(0),
      m_currentXQx(0),
      m_currentYQx(0),
      m_currentX(0),
      m_currentY(0),
      m_stepIncrementXQx(0),
      m_stepIncrementYQx(0),
      m_totalSteps(0),
      m_stepsRemaining(0),
      m_velocityMax(5000),
      m_accelMax(0),
      m_sampleRateHz(5000),
      m_entrySpeed(0),
      m_exitSpeed(0),
      m_dirX(0.0),
      m_dirY(0.0),
      m_lastXSteps(0),
      m_lastYSteps(0) {
}

bool LinearInterpolator::InitializeLinear(int32_t startX, int32_t startY,
                                         int32_t endX, int32_t endY,
                                         uint32_t velocityMax, uint32_t accelMax,
                                         uint16_t sampleRateHz,
                                         uint32_t entrySpeed,
                                         uint32_t exitSpeed) {
    m_startX = startX;
    m_startY = startY;
    m_endX = endX;
    m_endY = endY;
    m_velocityMax = velocityMax;
    m_accelMax = accelMax;
    m_sampleRateHz = sampleRateHz;
    m_entrySpeed = entrySpeed;
    m_exitSpeed = exitSpeed;
    
    // Calculate total distance
    m_totalSteps = CalculateDistance(startX, startY, endX, endY);
    
    m_progress = 0.0;
    if (m_totalSteps == 0) {
        // No movement required
        m_stepsRemaining = 0;
        m_currentX = endX;
        m_currentY = endY;
        m_currentXQx = (int64_t)endX << Q15_SHIFT;
        m_currentYQx = (int64_t)endY << Q15_SHIFT;
        m_lastXSteps = endX;
        m_lastYSteps = endY;
        return false;
    }
    
    // Calculate step increments per sample period
    // For constant velocity along path:
    // stepsPerSample = velocityMax / sampleRateHz
    // But we need to distribute this along X and Y based on the path direction
    
    // Calculate direction vector (normalized)
    int32_t deltaX = endX - startX;
    int32_t deltaY = endY - startY;
    
    // Calculate unit direction vector components
    m_dirX = (double)deltaX / (double)m_totalSteps;
    m_dirY = (double)deltaY / (double)m_totalSteps;
    
    // Calculate steps per sample along the path (start at max speed)
    double stepsPerSample = (double)velocityMax / (double)sampleRateHz;
    
    // Calculate step increments per sample for each axis
    double incrementX = m_dirX * stepsPerSample;
    double incrementY = m_dirY * stepsPerSample;
    
    // Convert to Q15 fixed-point
    m_stepIncrementXQx = Q15_FROM_DOUBLE(incrementX);
    m_stepIncrementYQx = Q15_FROM_DOUBLE(incrementY);
    
    // Initialize current position
    m_currentX = startX;
    m_currentY = startY;
    m_currentXQx = (int64_t)startX << Q15_SHIFT;
    m_currentYQx = (int64_t)startY << Q15_SHIFT;
    m_lastXSteps = startX;
    m_lastYSteps = startY;
    
    // Calculate total steps remaining (based on path length)
    m_stepsRemaining = m_totalSteps;
    
    return true;
}

bool LinearInterpolator::GenerateNextSteps(int32_t& stepsX, int32_t& stepsY) {
    if (m_progress >= (double)m_totalSteps) {
        stepsX = 0;
        stepsY = 0;
        m_stepsRemaining = 0;
        return false;
    }

    /* Progress along the path (same distance metric as m_totalSteps and the planner) is
     * accumulated in double precision and both axes are computed from the start point each
     * sample. (ClearHPGL fix: per-axis Q15 increments rounded the minor axis of a nearly
     * axis-aligned move -- at 20 mm/s a 2-step X over 8800-step Y advanced 2 Q15 units per
     * sample instead of 2.4 -- and the segment only ended once *both* axes passed their end, so
     * the major axis overran by 1685 steps and the clamp snapped it back in one sample: a burst
     * the ClearPath faulted on. Slow segments were worst, since the increment is smallest.) */
    const double remainingDist = (double)m_totalSteps - m_progress;
    double targetSpeed = (double)m_velocityMax;
    if (m_accelMax > 0) {
        const double maxSpeedFromStart = sqrt((double)m_entrySpeed * (double)m_entrySpeed +
                                              2.0 * (double)m_accelMax * m_progress);
        const double maxSpeedFromEnd = sqrt((double)m_exitSpeed * (double)m_exitSpeed +
                                            2.0 * (double)m_accelMax * remainingDist);
        if (maxSpeedFromStart < targetSpeed) {
            targetSpeed = maxSpeedFromStart;
        }
        if (maxSpeedFromEnd < targetSpeed) {
            targetSpeed = maxSpeedFromEnd;
        }
    }

    // Minimum speed: avoid long delay before motion starts (same as arc interpolator).
    // Use a fraction of max when accel-limited at startup instead of 1 step/sec.
    double minSpeed = (m_velocityMax > 0) ? (double)(m_velocityMax / 20) : 1.0;
    if (minSpeed < 1.0) {
        minSpeed = 1.0;
    }
    if (targetSpeed < minSpeed) {
        targetSpeed = minSpeed;
    }

    const double stepsPerSample = targetSpeed / (double)m_sampleRateHz;
    m_stepIncrementXQx = Q15_FROM_DOUBLE(m_dirX * stepsPerSample);  /* diagnostics only */
    m_stepIncrementYQx = Q15_FROM_DOUBLE(m_dirY * stepsPerSample);
    m_progress += stepsPerSample;

    int32_t newXSteps, newYSteps;
    if (m_progress >= (double)m_totalSteps) {
        /* last sample of the segment: land exactly on the end */
        m_progress = (double)m_totalSteps;
        newXSteps = m_endX;
        newYSteps = m_endY;
    } else {
        newXSteps = m_startX + (int32_t)floor(m_dirX * m_progress + 0.5);
        newYSteps = m_startY + (int32_t)floor(m_dirY * m_progress + 0.5);
    }

    stepsX = newXSteps - m_lastXSteps;
    stepsY = newYSteps - m_lastYSteps;
    m_lastXSteps = newXSteps;
    m_lastYSteps = newYSteps;
    m_currentX = newXSteps;
    m_currentY = newYSteps;
    m_currentXQx = (int64_t)newXSteps << Q15_SHIFT;
    m_currentYQx = (int64_t)newYSteps << Q15_SHIFT;
    const double left = (double)m_totalSteps - m_progress;
    m_stepsRemaining = left > 0.0 ? (uint32_t)ceil(left) : 0;

    /* A sample may move 0 steps on both axes (slow); the segment is still running. */
    return true;
}

void LinearInterpolator::Reset() {
    m_progress = 0.0;
    m_startX = 0;
    m_startY = 0;
    m_endX = 0;
    m_endY = 0;
    m_currentXQx = 0;
    m_currentYQx = 0;
    m_currentX = 0;
    m_currentY = 0;
    m_stepIncrementXQx = 0;
    m_stepIncrementYQx = 0;
    m_totalSteps = 0;
    m_stepsRemaining = 0;
    m_velocityMax = 0;
    m_accelMax = 0;
    m_sampleRateHz = 0;
    m_entrySpeed = 0;
    m_exitSpeed = 0;
    m_dirX = 0.0;
    m_dirY = 0.0;
    m_lastXSteps = 0;
    m_lastYSteps = 0;
}

uint32_t LinearInterpolator::CalculateDistance(int32_t x1, int32_t y1,
                                               int32_t x2, int32_t y2) {
    int64_t dx = (int64_t)x2 - (int64_t)x1;
    int64_t dy = (int64_t)y2 - (int64_t)y1;
    
    // Use integer approximation: sqrt(dx² + dy²) ≈ max(|dx|, |dy|) + min(|dx|, |dy|)/2
    // This avoids floating-point math in ISR context
    int64_t absDx = (dx < 0) ? -dx : dx;
    int64_t absDy = (dy < 0) ? -dy : dy;
    
    if (absDx > absDy) {
        return (uint32_t)(absDx + (absDy >> 1));
    } else {
        return (uint32_t)(absDy + (absDx >> 1));
    }
    
    // For more accuracy, could use: sqrt(dx*dx + dy*dy)
    // But the approximation is sufficient for step counting
}

} // ClearCore namespace
