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
      m_sampleRateHz(5000),
      m_lastXSteps(0),
      m_lastYSteps(0) {
}

bool LinearInterpolator::InitializeLinear(int32_t startX, int32_t startY,
                                         int32_t endX, int32_t endY,
                                         uint32_t velocityMax, uint16_t sampleRateHz) {
    m_startX = startX;
    m_startY = startY;
    m_endX = endX;
    m_endY = endY;
    m_velocityMax = velocityMax;
    m_sampleRateHz = sampleRateHz;
    
    // Calculate total distance
    m_totalSteps = CalculateDistance(startX, startY, endX, endY);
    
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
    
    // Calculate steps per sample along the path
    double stepsPerSample = (double)velocityMax / (double)sampleRateHz;
    
    // Calculate unit direction vector components
    double dirX = (double)deltaX / (double)m_totalSteps;
    double dirY = (double)deltaY / (double)m_totalSteps;
    
    // Calculate step increments per sample for each axis
    double incrementX = dirX * stepsPerSample;
    double incrementY = dirY * stepsPerSample;
    
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
    if (m_stepsRemaining == 0) {
        stepsX = 0;
        stepsY = 0;
        return false;
    }
    
    // Update position by adding increments
    m_currentXQx += m_stepIncrementXQx;
    m_currentYQx += m_stepIncrementYQx;
    
    // Convert to integer steps
    int32_t newXSteps = (int32_t)(m_currentXQx >> Q15_SHIFT);
    int32_t newYSteps = (int32_t)(m_currentYQx >> Q15_SHIFT);
    
    // Calculate step deltas
    stepsX = newXSteps - m_lastXSteps;
    stepsY = newYSteps - m_lastYSteps;
    
    // Update last step counts
    m_lastXSteps = newXSteps;
    m_lastYSteps = newYSteps;
    
    // Update current position
    m_currentX = newXSteps;
    m_currentY = newYSteps;
    
    // Check if we've reached or passed the end point
    bool pastEndX = (m_stepIncrementXQx >= 0) ? (newXSteps >= m_endX) : (newXSteps <= m_endX);
    bool pastEndY = (m_stepIncrementYQx >= 0) ? (newYSteps >= m_endY) : (newYSteps <= m_endY);
    
    if (pastEndX && pastEndY) {
        // Snap to end position
        stepsX = m_endX - m_lastXSteps + stepsX;
        stepsY = m_endY - m_lastYSteps + stepsY;
        m_currentX = m_endX;
        m_currentY = m_endY;
        m_currentXQx = (int64_t)m_endX << Q15_SHIFT;
        m_currentYQx = (int64_t)m_endY << Q15_SHIFT;
        m_lastXSteps = m_endX;
        m_lastYSteps = m_endY;
        m_stepsRemaining = 0;
    } else {
        // Update remaining steps based on progress along path
        // Approximate: reduce by the larger of |stepsX| or |stepsY|
        uint32_t stepsThisSample = (abs(stepsX) > abs(stepsY)) ? abs(stepsX) : abs(stepsY);
        if (stepsThisSample > m_stepsRemaining) {
            m_stepsRemaining = 0;
        } else {
            m_stepsRemaining -= stepsThisSample;
        }
    }
    
    return true;
}

void LinearInterpolator::Reset() {
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
