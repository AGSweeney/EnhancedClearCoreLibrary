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
    \file CoordinatedMotionController.h
    ClearCore Coordinated Motion Controller

    This class manages coordinated motion between two motors, enabling
    continuous arc moves with smooth transitions.
**/

#ifndef __COORDINATEDMOTIONCONTROLLER_H__
#define __COORDINATEDMOTIONCONTROLLER_H__

#include <stdint.h>
#include "ArcInterpolator.h"
#include "LinearInterpolator.h"
#include "UnitConverter.h"

namespace ClearCore {

// Forward declaration
class MotorDriver;

/**
    \class CoordinatedMotionController
    \brief Controller for coordinated motion between two motors

    This class provides high-level control for coordinated arc motion,
    managing arc queues and integrating with the motor control system.
**/
class CoordinatedMotionController {
public:
    /**
        \brief Constructor
    **/
    CoordinatedMotionController();

    /**
        \brief Initialize with two motor references
        
        \param[in] motorX Pointer to X-axis motor
        \param[in] motorY Pointer to Y-axis motor
        
        \return true if initialization successful
    **/
    bool Initialize(MotorDriver* motorX, MotorDriver* motorY);

    /**
        \brief Issue a single arc move
        
        \param[in] centerX Arc center X position in steps
        \param[in] centerY Arc center Y position in steps
        \param[in] radius Arc radius in steps
        \param[in] startAngle Start angle in radians
        \param[in] endAngle End angle in radians
        \param[in] clockwise Direction (true = clockwise)
        
        \return true if arc command accepted
    **/
    bool MoveArc(int32_t centerX, int32_t centerY,
                 int32_t radius,
                 double startAngle, double endAngle,
                 bool clockwise);

    /**
        \brief Issue a continuous arc (chains from current position)
        
        \param[in] centerX Arc center X position in steps
        \param[in] centerY Arc center Y position in steps
        \param[in] radius Arc radius in steps
        \param[in] endAngle End angle in radians
        \param[in] clockwise Direction (true = clockwise)
        
        \return true if arc command accepted
    **/
    bool MoveArcContinuous(int32_t centerX, int32_t centerY,
                          int32_t radius,
                          double endAngle,
                          bool clockwise);
    
    /**
        \brief Queue an arc move (can chain from any motion type)
        
        This method allows chaining an arc move after a linear move or another arc.
        The arc will start smoothly from the current position when the previous
        motion completes.
        
        \param[in] centerX Arc center X position in steps
        \param[in] centerY Arc center Y position in steps
        \param[in] radius Arc radius in steps
        \param[in] endAngle End angle in radians
        \param[in] clockwise Direction (true = clockwise)
        
        \return true if arc command accepted
    **/
    bool QueueArc(int32_t centerX, int32_t centerY,
                 int32_t radius,
                 double endAngle,
                 bool clockwise);
    
    /**
        \brief Queue a linear move (can chain from any motion type)
        
        This method allows chaining a linear move after an arc move or another linear.
        The linear move will start smoothly from the current position when the previous
        motion completes.
        
        \param[in] endX Ending X position in steps
        \param[in] endY Ending Y position in steps
        
        \return true if move command accepted
    **/
    bool QueueLinear(int32_t endX, int32_t endY);

    /**
        \brief Issue a coordinated linear move
        
        \param[in] endX Ending X position in steps
        \param[in] endY Ending Y position in steps
        
        \return true if move command accepted
    **/
    bool MoveLinear(int32_t endX, int32_t endY);
    
    /**
        \brief Issue a coordinated linear move (absolute)
        
        \param[in] startX Starting X position in steps
        \param[in] startY Starting Y position in steps
        \param[in] endX Ending X position in steps
        \param[in] endY Ending Y position in steps
        
        \return true if move command accepted
    **/
    bool MoveLinearAbsolute(int32_t startX, int32_t startY,
                           int32_t endX, int32_t endY);
    
    /**
        \brief Issue a continuous linear move (chains from current position)
        
        \param[in] endX Ending X position in steps
        \param[in] endY Ending Y position in steps
        
        \return true if move command accepted
    **/
    bool MoveLinearContinuous(int32_t endX, int32_t endY);

    /**
        \brief Set maximum velocity for arc moves
        
        \param[in] velMax Maximum velocity in steps/sec
    **/
    void ArcVelMax(uint32_t velMax) {
        m_velocityMax = velMax;
    }

    /**
        \brief Set maximum acceleration for arc moves
        
        \param[in] accelMax Maximum acceleration in steps/sec²
    **/
    void ArcAccelMax(uint32_t accelMax) {
        m_accelMax = accelMax;
    }

    /**
        \brief Stop motion immediately
    **/
    void Stop();

    /**
        \brief Stop motion with deceleration
    **/
    void StopDecel();

    /**
        \brief Check if coordinated motion is active
        
        \return true if motion is active
    **/
    bool IsActive() const {
        return m_active;
    }

    /**
        \brief Check if all arcs are complete
        
        \return true if no arcs remaining
    **/
    bool ArcComplete() const {
        return !m_active && m_arcQueueCount == 0;
    }

    /**
        \brief Get current X position
        
        \return Current X position in steps
    **/
    int32_t CurrentX() const {
        return m_currentX;
    }

    /**
        \brief Get current Y position
        
        \return Current Y position in steps
    **/
    int32_t CurrentY() const {
        return m_currentY;
    }

    /**
        \brief Set current position (for homing/initialization)
        
        \param[in] x X position in steps
        \param[in] y Y position in steps
    **/
    void SetPosition(int32_t x, int32_t y) {
        m_currentX = x;
        m_currentY = y;
    }

    /**
        \brief ISR callback - called from MotorDriver::Refresh()
        
        This function generates and applies coordinated steps.
        Must be called at the sample rate (5 kHz).
    **/
    void UpdateFast();

    /**
        \brief Get number of arcs in queue
        
        \return Number of arcs queued
    **/
    uint8_t QueueCount() const {
        return m_arcQueueCount;
    }
    
    /**
        \brief Get total number of motions in unified queue
        
        \return Total number of motions queued (arcs + linear)
    **/
    uint8_t MotionQueueCount() const {
        return m_motionQueueCount;
    }

    // ========== Unit Conversion Support ==========
    
    /**
        \brief Set mechanical parameters for X-axis motor
        
        \param[in] stepsPerRev Motor steps per revolution
        \param[in] pitch Lead screw pitch
        \param[in] pitchUnit Units for pitch
        \param[in] gearRatio Gear ratio (default 1.0)
        
        \return true if configuration successful
    **/
    bool SetMechanicalParamsX(uint32_t stepsPerRev, double pitch,
                             UnitType pitchUnit, double gearRatio = 1.0);
    
    /**
        \brief Set mechanical parameters for Y-axis motor
        
        \param[in] stepsPerRev Motor steps per revolution
        \param[in] pitch Lead screw pitch
        \param[in] pitchUnit Units for pitch
        \param[in] gearRatio Gear ratio (default 1.0)
        
        \return true if configuration successful
    **/
    bool SetMechanicalParamsY(uint32_t stepsPerRev, double pitch,
                              UnitType pitchUnit, double gearRatio = 1.0);
    
    /**
        \brief Move arc in inches
        
        \param[in] centerX Arc center X position in inches
        \param[in] centerY Arc center Y position in inches
        \param[in] radius Arc radius in inches
        \param[in] startAngle Start angle in radians
        \param[in] endAngle End angle in radians
        \param[in] clockwise Direction (true = clockwise)
        
        \return true if arc command accepted
    **/
    bool MoveArcInches(double centerX, double centerY, double radius,
                      double startAngle, double endAngle, bool clockwise);
    
    /**
        \brief Move arc in millimeters
        
        \param[in] centerX Arc center X position in mm
        \param[in] centerY Arc center Y position in mm
        \param[in] radius Arc radius in mm
        \param[in] startAngle Start angle in radians
        \param[in] endAngle End angle in radians
        \param[in] clockwise Direction (true = clockwise)
        
        \return true if arc command accepted
    **/
    bool MoveArcMM(double centerX, double centerY, double radius,
                   double startAngle, double endAngle, bool clockwise);
    
    /**
        \brief Move linear in inches
        
        \param[in] endX Ending X position in inches
        \param[in] endY Ending Y position in inches
        
        \return true if move command accepted
    **/
    bool MoveLinearInches(double endX, double endY);
    
    /**
        \brief Move linear in millimeters
        
        \param[in] endX Ending X position in mm
        \param[in] endY Ending Y position in mm
        
        \return true if move command accepted
    **/
    bool MoveLinearMM(double endX, double endY);
    
    /**
        \brief Set arc feed rate in inches per minute
        
        \param[in] feedRate Feed rate in inches per minute
    **/
    void ArcFeedRateInchesPerMin(double feedRate);
    
    /**
        \brief Set arc feed rate in millimeters per minute
        
        \param[in] feedRate Feed rate in millimeters per minute
    **/
    void ArcFeedRateMMPerMin(double feedRate);
    
    /**
        \brief Set arc feed rate in millimeters per second
        
        \param[in] feedRate Feed rate in millimeters per second
    **/
    void ArcFeedRateMMPerSec(double feedRate);
    
    /**
        \brief Get current X position in inches
        
        \return Current X position in inches
    **/
    double CurrentXInches() const;
    
    /**
        \brief Get current Y position in inches
        
        \return Current Y position in inches
    **/
    double CurrentYInches() const;
    
    /**
        \brief Get current X position in millimeters
        
        \return Current X position in millimeters
    **/
    double CurrentXMM() const;
    
    /**
        \brief Get current Y position in millimeters
        
        \return Current Y position in millimeters
    **/
    double CurrentYMM() const;

private:
    static const uint8_t ARC_QUEUE_SIZE = 8;
    
    enum MotionType {
        MOTION_TYPE_NONE,
        MOTION_TYPE_ARC,
        MOTION_TYPE_LINEAR
    };
    
    // Unified motion queue entry
    enum QueuedMotionType {
        QUEUED_MOTION_ARC,
        QUEUED_MOTION_LINEAR
    };
    
    struct QueuedMotion {
        QueuedMotionType type;
        union {
            struct {
                int32_t centerX;
                int32_t centerY;
                int32_t radius;
                double endAngle;
                bool clockwise;
            } arc;
            struct {
                int32_t endX;
                int32_t endY;
            } linear;
        };
        bool valid;
    };
    
    MotorDriver* m_motorX;
    MotorDriver* m_motorY;
    
    ArcInterpolator m_arcInterpolator;
    LinearInterpolator m_linearInterpolator;
    
    // Unified motion queue (can hold both arcs and linear moves)
    QueuedMotion m_motionQueue[ARC_QUEUE_SIZE];
    uint8_t m_motionQueueHead;
    uint8_t m_motionQueueTail;
    uint8_t m_motionQueueCount;
    
    // Legacy separate queues (for backward compatibility)
    struct QueuedArc {
        int32_t centerX;
        int32_t centerY;
        int32_t radius;
        double endAngle;
        bool clockwise;
        bool valid;
    };
    
    struct QueuedLinear {
        int32_t endX;
        int32_t endY;
        bool valid;
    };
    
    QueuedArc m_arcQueue[ARC_QUEUE_SIZE];
    QueuedLinear m_linearQueue[ARC_QUEUE_SIZE];
    uint8_t m_arcQueueHead;
    uint8_t m_arcQueueTail;
    uint8_t m_arcQueueCount;
    uint8_t m_linearQueueHead;
    uint8_t m_linearQueueTail;
    uint8_t m_linearQueueCount;
    
    // State
    bool m_active;
    bool m_initialized;
    MotionType m_motionType;
    int32_t m_currentX;
    int32_t m_currentY;
    double m_currentAngle; // Current angle in radians
    
    // Motion parameters
    uint32_t m_velocityMax;
    uint32_t m_accelMax;
    
    // Unit conversion support
    MotorMechanicalConfig m_mechanicalConfigX;
    MotorMechanicalConfig m_mechanicalConfigY;
    bool m_unitsConfiguredX;
    bool m_unitsConfiguredY;
    
    /**
        \brief Process next arc from queue
    **/
    bool ProcessNextArc();
    
    /**
        \brief Calculate start angle for continuous arc
        
        \param[in] centerX Arc center X
        \param[in] centerY Arc center Y
        
        \return Start angle in radians
    **/
    double CalculateStartAngle(int32_t centerX, int32_t centerY) const;
    
    /**
        \brief Validate arc parameters
        
        \param[in] centerX Arc center X
        \param[in] centerY Arc center Y
        \param[in] radius Arc radius
        
        \return true if parameters are valid
    **/
    bool ValidateArc(int32_t centerX, int32_t centerY, int32_t radius) const;
    
    /**
        \brief Process next linear move from queue
    **/
    bool ProcessNextLinear();
    
    /**
        \brief Validate linear move parameters
        
        \param[in] endX Ending X position
        \param[in] endY Ending Y position
        
        \return true if parameters are valid
    **/
    bool ValidateLinear(int32_t endX, int32_t endY) const;
    
    /**
        \brief Process next motion from unified queue (arc or linear)
    **/
    bool ProcessNextMotion();
};

} // ClearCore namespace

#endif // __COORDINATEDMOTIONCONTROLLER_H__
