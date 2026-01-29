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
    \file ArcInterpolator.h
    ClearCore Arc Interpolation Engine

    This class provides arc interpolation functionality for coordinated motion.
    It generates step sequences for circular arcs using fixed-point arithmetic.
**/

#ifndef __ARCINTERPOLATOR_H__
#define __ARCINTERPOLATOR_H__

#include <stdint.h>
#include "StepGenerator.h"

namespace ClearCore {

/**
    \struct ArcSegment
    \brief Represents a single arc segment with all parameters
**/
struct ArcSegment {
    int64_t centerXQx;      // Arc center X position (Q15 fixed-point)
    int64_t centerYQx;      // Arc center Y position (Q15 fixed-point)
    int64_t radiusQx;       // Arc radius (Q15 fixed-point)
    int32_t startAngleQx;  // Start angle (Q15 fixed-point, 0-2π = 0-32768)
    int32_t endAngleQx;    // End angle (Q15 fixed-point, 0-2π = 0-32768)
    bool clockwise;         // Direction of rotation
    uint32_t totalSteps;   // Total steps in this arc segment
    uint32_t stepsRemaining; // Steps remaining in this arc
    
    ArcSegment()
        : centerXQx(0),
          centerYQx(0),
          radiusQx(0),
          startAngleQx(0),
          endAngleQx(0),
          clockwise(true),
          totalSteps(0),
          stepsRemaining(0) {}
};

/**
    \class ArcInterpolator
    \brief Arc interpolation engine for generating coordinated step sequences

    This class handles the mathematical generation of arc paths, converting
    arc parameters into step sequences for two coordinated motors.
**/
class ArcInterpolator {
public:
    /**
        \brief Constructor
    **/
    ArcInterpolator();

    /**
        \brief Initialize an arc segment
        
        \param[in] centerX Center X position in steps
        \param[in] centerY Center Y position in steps
        \param[in] radius Arc radius in steps
        \param[in] startAngle Start angle in radians
        \param[in] endAngle End angle in radians
        \param[in] clockwise Direction (true = clockwise, false = CCW)
        \param[in] velocityMax Maximum velocity in steps/sec
        \param[in] sampleRateHz Sample rate in Hz
        
        \return true if arc initialized successfully
    **/
    bool InitializeArc(int32_t centerX, int32_t centerY,
                       int32_t radius,
                       double startAngle, double endAngle,
                       bool clockwise,
                       uint32_t velocityMax, uint32_t accelMax,
                       uint16_t sampleRateHz);

    /**
        \brief Generate next step pair for current arc
        
        \param[out] stepsX Steps for X motor
        \param[out] stepsY Steps for Y motor
        
        \return true if steps generated, false if arc complete
    **/
    bool GenerateNextSteps(int32_t &stepsX, int32_t &stepsY);

    /**
        \brief Check if current arc is complete
        
        \return true if arc is complete
    **/
    bool IsArcComplete() const {
        // Check if angle has reached end angle (with small threshold)
        // Calculate remaining angle in the direction of travel (same logic as GenerateNextSteps)
        int32_t angleRemainingQx;
        if (m_currentArc.clockwise) {
            if (m_currentArc.endAngleQx < m_currentAngleQx) {
                angleRemainingQx = m_currentAngleQx - m_currentArc.endAngleQx;
            } else {
                angleRemainingQx = TWO_PI_QX - (m_currentArc.endAngleQx - m_currentAngleQx);
            }
        } else {
            if (m_currentArc.endAngleQx > m_currentAngleQx) {
                angleRemainingQx = m_currentArc.endAngleQx - m_currentAngleQx;
            } else {
                angleRemainingQx = TWO_PI_QX - (m_currentAngleQx - m_currentArc.endAngleQx);
            }
        }
        
        const int32_t ANGLE_THRESHOLD_QX = (TWO_PI_QX / 3600); // ~0.1 degree
        return angleRemainingQx <= ANGLE_THRESHOLD_QX;
    }

    /**
        \brief Get current position
        
        \param[out] posX Current X position
        \param[out] posY Current Y position
    **/
    void GetCurrentPosition(int32_t &posX, int32_t &posY) const;

    /**
        \brief Reset arc state
    **/
    void Reset();

    /**
        \brief Get remaining steps in current arc
        
        \return Number of steps remaining
    **/
    uint32_t StepsRemaining() const {
        return m_currentArc.stepsRemaining;
    }

private:
    ArcSegment m_currentArc;
    
    // Current state
    int32_t m_currentAngleQx;  // Current angle along arc (Q15)
    int64_t m_currentXQx;      // Current X position (Q15)
    int64_t m_currentYQx;      // Current Y position (Q15)
    int32_t m_lastXSteps;       // Last X position in steps
    int32_t m_lastYSteps;       // Last Y position in steps
    
    // Velocity control
    int32_t m_angleIncrementQx; // Angle increment per step (Q15) - for reference
    uint32_t m_velocityMax;      // Tangential velocity along arc path (steps/sec)
    uint32_t m_accelMax;         // Tangential acceleration along arc path (steps/sec^2)
    uint16_t m_sampleRateHz;    // Sample rate (Hz)
    
    // Constants
    static const int32_t TWO_PI_QX = 32768; // 2π in Q15 format
    static const int32_t PI_QX = 16384;     // π in Q15 format
    
    /**
        \brief Convert angle from radians to Q15 fixed-point
        
        \param[in] angleRad Angle in radians
        
        \return Angle in Q15 format
    **/
    static int32_t AngleToQx(double angleRad);
    
    /**
        \brief Normalize angle to 0-2π range in Q15 format
        
        \param[in] angleQx Angle in Q15 format
        
        \return Normalized angle
    **/
    static int32_t NormalizeAngleQx(int32_t angleQx);
    
    /**
        \brief Calculate angle increment based on velocity
        
        \param[in] radiusQx Arc radius in Q15
        \param[in] velocityMax Maximum velocity in steps/sec
        \param[in] sampleRateHz Sample rate in Hz
        
        \return Angle increment per sample in Q15 format
    **/
    static int32_t CalculateAngleIncrement(int64_t radiusQx,
                                          uint32_t velocityMax,
                                          uint16_t sampleRateHz);
};

} // ClearCore namespace

#endif // __ARCINTERPOLATOR_H__
