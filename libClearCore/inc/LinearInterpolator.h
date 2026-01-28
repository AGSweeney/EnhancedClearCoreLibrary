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

/**
    \file LinearInterpolator.h
    ClearCore Linear Interpolator for Coordinated Motion

    This class generates coordinated step commands for linear moves
    between two motors, maintaining constant velocity along the path.
**/

#ifndef __LINEARINTERPOLATOR_H__
#define __LINEARINTERPOLATOR_H__

#include <stdint.h>

namespace ClearCore {

/**
    \class LinearInterpolator
    \brief Linear interpolation for coordinated motion
    
    Generates coordinated step commands for linear moves between two points,
    maintaining constant velocity along the path.
**/
class LinearInterpolator {
public:
    /**
        \brief Constructor
    **/
    LinearInterpolator();
    
    /**
        \brief Initialize a linear move
        
        \param[in] startX Starting X position in steps
        \param[in] startY Starting Y position in steps
        \param[in] endX Ending X position in steps
        \param[in] endY Ending Y position in steps
        \param[in] velocityMax Maximum velocity along path in steps/sec
        \param[in] sampleRateHz System sample rate (typically 5000 Hz)
        
        \return true if initialization successful
    **/
    bool InitializeLinear(int32_t startX, int32_t startY,
                         int32_t endX, int32_t endY,
                         uint32_t velocityMax, uint16_t sampleRateHz);
    
    /**
        \brief Generate next step pair for the linear move
        
        \param[out] stepsX X-axis steps to execute this sample
        \param[out] stepsY Y-axis steps to execute this sample
        
        \return true if steps were generated, false if move complete
    **/
    bool GenerateNextSteps(int32_t& stepsX, int32_t& stepsY);
    
    /**
        \brief Check if linear move is complete
        
        \return true if move is complete
    **/
    bool IsLinearComplete() const {
        return m_stepsRemaining == 0;
    }
    
    /**
        \brief Get current position
        
        \param[out] x Current X position in steps
        \param[out] y Current Y position in steps
    **/
    void GetCurrentPosition(int32_t& x, int32_t& y) const {
        x = m_currentX;
        y = m_currentY;
    }
    
    /**
        \brief Reset interpolator state
    **/
    void Reset();
    
    /**
        \brief Get total steps for current move
        
        \return Total steps along path
    **/
    uint32_t TotalSteps() const {
        return m_totalSteps;
    }
    
    /**
        \brief Get remaining steps
        
        \return Steps remaining
    **/
    uint32_t StepsRemaining() const {
        return m_stepsRemaining;
    }

private:
    // Current move parameters
    int32_t m_startX;
    int32_t m_startY;
    int32_t m_endX;
    int32_t m_endY;
    
    // Current position (in steps, Q15 fractional)
    int64_t m_currentXQx;
    int64_t m_currentYQx;
    int32_t m_currentX;
    int32_t m_currentY;
    
    // Step increments per sample (Q15 fixed-point)
    int32_t m_stepIncrementXQx;
    int32_t m_stepIncrementYQx;
    
    // Move state
    uint32_t m_totalSteps;
    uint32_t m_stepsRemaining;
    uint32_t m_velocityMax;
    uint16_t m_sampleRateHz;
    
    // Last step counts (for delta calculation)
    int32_t m_lastXSteps;
    int32_t m_lastYSteps;
    
    /**
        \brief Calculate distance between two points
        
        \param[in] x1 First point X
        \param[in] y1 First point Y
        \param[in] x2 Second point X
        \param[in] y2 Second point Y
        
        \return Distance in steps
    **/
    static uint32_t CalculateDistance(int32_t x1, int32_t y1,
                                      int32_t x2, int32_t y2);
};

} // ClearCore namespace

#endif // __LINEARINTERPOLATOR_H__
