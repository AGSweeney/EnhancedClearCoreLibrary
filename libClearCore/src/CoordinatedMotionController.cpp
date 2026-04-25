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

#include "CoordinatedMotionController.h"
#include "MotorDriver.h"
#include "SysTiming.h"
#include "UnitConverter.h"
#include <math.h>

namespace ClearCore {

static const bool kUseGrblPlanner = true;  // Enable GRBL-style path planner for testing
static inline void NormalizeVec(double &x, double &y) {
    double mag = sqrt(x * x + y * y);
    if (mag > 1e-9) {
        x /= mag;
        y /= mag;
    }
    else {
        x = 0.0;
        y = 0.0;
    }
}

static inline double SegmentLengthSteps(int32_t dx, int32_t dy) {
    return sqrt((double)dx * (double)dx + (double)dy * (double)dy);
}

static inline void TangentDir(double angle, bool clockwise, double &x, double &y) {
    // CCW tangent: (-sin, cos) at angle
    double tx = -sin(angle);
    double ty = cos(angle);
    if (clockwise) {
        tx = -tx;
        ty = -ty;
    }
    x = tx;
    y = ty;
    NormalizeVec(x, y);
}

CoordinatedMotionController::CoordinatedMotionController()
    : m_motorX(nullptr),
      m_motorY(nullptr),
      m_motionQueueHead(0),
      m_motionQueueTail(0),
      m_motionQueueCount(0),
      m_motionQueuePlanned(0),
      m_arcQueueHead(0),
      m_arcQueueTail(0),
      m_arcQueueCount(0),
      m_linearQueueHead(0),
      m_linearQueueTail(0),
      m_linearQueueCount(0),
      m_active(false),
      m_initialized(false),
      m_motionType(MOTION_TYPE_NONE),
      m_junctionDeviationSteps(1.0),
      m_activeTargetX(0),
      m_activeTargetY(0),
      m_stopCounter(0),
      m_currentX(0),
      m_currentY(0),
      m_currentAngle(0.0),
      m_velocityMax(5000),
      m_accelMax(50000),
      m_unitsConfiguredX(false),
      m_unitsConfiguredY(false) {
    // Initialize queues
    for (uint8_t i = 0; i < ARC_QUEUE_SIZE; i++) {
        m_motionQueue[i].valid = false;
        m_motionQueue[i].nominalSpeed = 0;
        m_motionQueue[i].entrySpeed = 0;
        m_motionQueue[i].exitSpeed = 0;
        m_motionQueue[i].lengthSteps = 0;
        m_motionQueue[i].entryDirX = 0.0f;
        m_motionQueue[i].entryDirY = 0.0f;
        m_motionQueue[i].exitDirX = 0.0f;
        m_motionQueue[i].exitDirY = 0.0f;
        m_motionQueue[i].startX = 0;
        m_motionQueue[i].startY = 0;
        m_motionQueue[i].endX = 0;
        m_motionQueue[i].endY = 0;
        m_arcQueue[i].valid = false;
        m_linearQueue[i].valid = false;
    }
}

bool CoordinatedMotionController::Initialize(MotorDriver* motorX, MotorDriver* motorY) {
    if (motorX == nullptr || motorY == nullptr) {
        return false;
    }
    
    m_motorX = motorX;
    m_motorY = motorY;
    m_initialized = true;
    
    // Enable coordinated mode on both motors
    if (!m_motorX->CoordinatedMotionMode(true, this) ||
        !m_motorY->CoordinatedMotionMode(true, this)) {
        m_initialized = false;
        return false;
    }
    
    // Set motor references in MotorDriver (friend access)
    m_motorX->m_coordinatedMotorX = motorX;
    m_motorX->m_coordinatedMotorY = motorY;
    m_motorY->m_coordinatedMotorX = motorX;
    m_motorY->m_coordinatedMotorY = motorY;
    
    return true;
}

bool CoordinatedMotionController::MoveArc(int32_t centerX, int32_t centerY,
                                          int32_t radius,
                                          double startAngle, double endAngle,
                                          bool clockwise) {
    if (!m_initialized) {
        return false;
    }
    
    if (!ValidateArc(centerX, centerY, radius)) {
        return false;
    }
    
    // Stop any current motion
    Stop();
    
    // Initialize arc interpolator
    if (!m_arcInterpolator.InitializeArc(centerX, centerY, radius,
                                         startAngle, endAngle, clockwise,
                                         m_velocityMax, m_accelMax, SampleRateHz,
                                         0, 0)) {
        return false;
    }
    
    // Update current position
    int32_t tempX, tempY;
    m_arcInterpolator.GetCurrentPosition(tempX, tempY);
    m_currentX = tempX;
    m_currentY = tempY;
    m_motionType = MOTION_TYPE_ARC;
    
    // Calculate current angle
    double dx = (double)(m_currentX - centerX);
    double dy = (double)(m_currentY - centerY);
    m_currentAngle = atan2(dy, dx);
    if (m_currentAngle < 0) {
        m_currentAngle += 2.0 * M_PI;
    }
    
    m_stopCounter = 0;  // Reset stop counter when starting new motion
    m_activeTargetX = (int32_t)llround((double)centerX + (double)radius * cos(endAngle));
    m_activeTargetY = (int32_t)llround((double)centerY + (double)radius * sin(endAngle));
    m_active = true;
    return true;
}

bool CoordinatedMotionController::MoveArcContinuous(int32_t centerX, int32_t centerY,
                                                     int32_t radius,
                                                     double endAngle,
                                                     bool clockwise) {
    if (!m_initialized) {
        return false;
    }
    
    if (!ValidateArc(centerX, centerY, radius)) {
        return false;
    }
    
    // Critical section: protect queue operations from ISR
    __disable_irq();
    
    // Check if queue is full
    if (m_arcQueueCount >= ARC_QUEUE_SIZE) {
        __enable_irq();
        return false;
    }
    
    // If no active arc, start immediately
    if (!m_active) {
        __enable_irq();
        double startAngle = CalculateStartAngle(centerX, centerY);
        return MoveArc(centerX, centerY, radius, startAngle, endAngle, clockwise);
    }
    
    // Queue the arc
    uint8_t tail = m_arcQueueTail;
    QueuedArc& arc = m_arcQueue[tail];
    arc.centerX = centerX;
    arc.centerY = centerY;
    arc.radius = radius;
    arc.endAngle = endAngle;
    arc.clockwise = clockwise;
    arc.valid = true;
    
    m_arcQueueTail = (tail + 1) % ARC_QUEUE_SIZE;
    m_arcQueueCount++;
    
    __enable_irq();
    
    return true;
}

void CoordinatedMotionController::Stop() {
    m_active = false;
    m_motionType = MOTION_TYPE_NONE;
    m_arcInterpolator.Reset();
    m_linearInterpolator.Reset();
    
    // Clear all queues
    for (uint8_t i = 0; i < ARC_QUEUE_SIZE; i++) {
        m_motionQueue[i].valid = false;
        m_arcQueue[i].valid = false;
        m_linearQueue[i].valid = false;
    }
    m_motionQueueHead = 0;
    m_motionQueueTail = 0;
    m_motionQueueCount = 0;
    m_motionQueuePlanned = 0;
    m_arcQueueHead = 0;
    m_arcQueueTail = 0;
    m_arcQueueCount = 0;
    m_linearQueueHead = 0;
    m_linearQueueTail = 0;
    m_linearQueueCount = 0;
    
    // Stop motors
    if (m_motorX) {
        m_motorX->MoveStopAbrupt();
    }
    if (m_motorY) {
        m_motorY->MoveStopAbrupt();
    }
}

void CoordinatedMotionController::StopDecel() {
    m_active = false;
    m_motionType = MOTION_TYPE_NONE;
    m_arcInterpolator.Reset();
    m_linearInterpolator.Reset();
    
    // Clear all queues
    for (uint8_t i = 0; i < ARC_QUEUE_SIZE; i++) {
        m_motionQueue[i].valid = false;
        m_arcQueue[i].valid = false;
        m_linearQueue[i].valid = false;
    }
    m_motionQueueHead = 0;
    m_motionQueueTail = 0;
    m_motionQueueCount = 0;
    m_motionQueuePlanned = 0;
    m_arcQueueHead = 0;
    m_arcQueueTail = 0;
    m_arcQueueCount = 0;
    m_linearQueueHead = 0;
    m_linearQueueTail = 0;
    m_linearQueueCount = 0;
    
    // Stop motors with deceleration
    if (m_motorX) {
        m_motorX->MoveStopDecel();
    }
    if (m_motorY) {
        m_motorY->MoveStopDecel();
    }
}

void CoordinatedMotionController::UpdateFast() {
    if (!m_initialized) {
        return;
    }
    
    // If stop counter is active, continue sending zero steps to clear hardware buffer
    if (m_stopCounter > 0) {
        if (m_motorX) {
            m_motorX->SetCoordinatedSteps(0);
        }
        if (m_motorY) {
            m_motorY->SetCoordinatedSteps(0);
        }
        m_stopCounter--;
        return;
    }
    
    if (!m_active) {
        return;
    }
    
    bool motionComplete = false;
    
    // Check motion type and completion
    if (m_motionType == MOTION_TYPE_ARC) {
        if (m_arcInterpolator.IsArcComplete()) {
            // Update position from interpolator
            int32_t tempX, tempY;
            m_arcInterpolator.GetCurrentPosition(tempX, tempY);
            m_currentX = tempX;
            m_currentY = tempY;
            
            // Try unified queue first, then legacy queues
            if (!ProcessNextMotion()) {
                if (!ProcessNextArc()) {
                    motionComplete = true;
                }
            }
        }
    } else if (m_motionType == MOTION_TYPE_LINEAR) {
        if (m_linearInterpolator.IsLinearComplete()) {
            // Update position from interpolator
            int32_t tempX, tempY;
            m_linearInterpolator.GetCurrentPosition(tempX, tempY);
            m_currentX = tempX;
            m_currentY = tempY;
            
            // Try unified queue first, then legacy queues
            if (!ProcessNextMotion()) {
                if (!ProcessNextLinear()) {
                    motionComplete = true;
                }
            }
        }
    }
    
    if (motionComplete) {
        // No more moves, stop motion
        m_active = false;
        m_motionType = MOTION_TYPE_NONE;
        m_stopCounter = 10;  // Send zero steps for 10 more cycles (2ms at 5kHz)
        return;
    }
    
    // Generate next step pair
    int32_t stepsX = 0, stepsY = 0;
    bool stepsGenerated = false;
    
    if (m_motionType == MOTION_TYPE_ARC) {
        stepsGenerated = m_arcInterpolator.GenerateNextSteps(stepsX, stepsY);
    } else if (m_motionType == MOTION_TYPE_LINEAR) {
        stepsGenerated = m_linearInterpolator.GenerateNextSteps(stepsX, stepsY);
    }
    
    if (stepsGenerated) {
        // Safety check: if both steps are zero, motion is complete
        if (stepsX == 0 && stepsY == 0) {
            // Explicitly send zero steps to stop motors
            if (m_motorX) {
                m_motorX->SetCoordinatedSteps(0);
            }
            if (m_motorY) {
                m_motorY->SetCoordinatedSteps(0);
            }
            
            // No steps to apply - motion is complete
            if (m_motionType == MOTION_TYPE_LINEAR && m_linearInterpolator.IsLinearComplete()) {
                m_active = false;
                m_motionType = MOTION_TYPE_NONE;
                m_stopCounter = 10;  // Send zero steps for 10 more cycles (2ms at 5kHz)
                return;
            } else if (m_motionType == MOTION_TYPE_ARC && m_arcInterpolator.IsArcComplete()) {
                m_active = false;
                m_motionType = MOTION_TYPE_NONE;
                m_stopCounter = 10;  // Send zero steps for 10 more cycles (2ms at 5kHz)
                return;
            }
        }
        
        // Apply steps to motors
        m_motorX->SetCoordinatedSteps(stepsX);
        m_motorY->SetCoordinatedSteps(stepsY);
        
        // Update current position from interpolator (not by adding steps)
        // This ensures position matches interpolator's internal state
        if (m_motionType == MOTION_TYPE_ARC) {
            int32_t tempX, tempY;
            m_arcInterpolator.GetCurrentPosition(tempX, tempY);
            m_currentX = tempX;
            m_currentY = tempY;
        } else if (m_motionType == MOTION_TYPE_LINEAR) {
            int32_t tempX, tempY;
            m_linearInterpolator.GetCurrentPosition(tempX, tempY);
            m_currentX = tempX;
            m_currentY = tempY;
        }
    } else {
        // GenerateNextSteps returned false - motion should be complete
        // Explicitly send zero steps to stop motors
        if (m_motorX) {
            m_motorX->SetCoordinatedSteps(0);
        }
        if (m_motorY) {
            m_motorY->SetCoordinatedSteps(0);
        }
        
        // Double-check completion status and stop if needed
        if (m_motionType == MOTION_TYPE_ARC) {
            if (m_arcInterpolator.IsArcComplete()) {
                // Update position from interpolator one final time
                int32_t tempX, tempY;
                m_arcInterpolator.GetCurrentPosition(tempX, tempY);
                m_currentX = tempX;
                m_currentY = tempY;
                m_active = false;
                m_motionType = MOTION_TYPE_NONE;
                m_stopCounter = 10;  // Send zero steps for 10 more cycles (2ms at 5kHz)
            }
        } else if (m_motionType == MOTION_TYPE_LINEAR) {
            if (m_linearInterpolator.IsLinearComplete()) {
                // Update position from interpolator one final time
                int32_t tempX, tempY;
                m_linearInterpolator.GetCurrentPosition(tempX, tempY);
                m_currentX = tempX;
                m_currentY = tempY;
                m_active = false;
                m_motionType = MOTION_TYPE_NONE;
                m_stopCounter = 10;  // Send zero steps for 10 more cycles (2ms at 5kHz)
            }
        }
    }
}

bool CoordinatedMotionController::ProcessNextArc() {
    // This function is called from ISR context (UpdateFast), so interrupts are already disabled
    // No need for additional critical section protection
    
    if (m_arcQueueCount == 0) {
        return false;
    }
    
    // Get next arc from queue
    uint8_t head = m_arcQueueHead;
    QueuedArc& arc = m_arcQueue[head];
    if (!arc.valid) {
        return false;
    }
    
    // Calculate start angle (tangent to current position)
    double startAngle = CalculateStartAngle(arc.centerX, arc.centerY);
    
    // Initialize interpolator with new arc
    if (!m_arcInterpolator.InitializeArc(arc.centerX, arc.centerY, arc.radius,
                                         startAngle, arc.endAngle, arc.clockwise,
                                         m_velocityMax, m_accelMax, SampleRateHz,
                                         0, 0)) {
        // Failed to initialize, remove from queue
        arc.valid = false;
        m_arcQueueHead = (head + 1) % ARC_QUEUE_SIZE;
        m_arcQueueCount--;
        return false;
    }
    
    m_motionType = MOTION_TYPE_ARC;
    
    // Update current angle
    m_currentAngle = arc.endAngle;
    
    // Remove from queue
    arc.valid = false;
    m_arcQueueHead = (head + 1) % ARC_QUEUE_SIZE;
    m_arcQueueCount--;
    
    return true;
}

double CoordinatedMotionController::CalculateStartAngle(int32_t centerX, int32_t centerY) const {
    // Calculate angle from center to current position
    double dx = (double)(m_currentX - centerX);
    double dy = (double)(m_currentY - centerY);
    double angle = atan2(dy, dx);
    
    // Normalize to 0-2π
    if (angle < 0) {
        angle += 2.0 * M_PI;
    }
    
    return angle;
}

bool CoordinatedMotionController::ValidateArc(int32_t /*centerX*/, int32_t /*centerY*/, int32_t radius) const {
    // Check radius is positive
    if (radius <= 0) {
        return false;
    }
    
    // Check motors are enabled
    if (m_motorX && !m_motorX->EnableRequest()) {
        return false;
    }
    if (m_motorY && !m_motorY->EnableRequest()) {
        return false;
    }
    
    // Check for alerts
    if (m_motorX && m_motorX->StatusReg().bit.AlertsPresent) {
        return false;
    }
    if (m_motorY && m_motorY->StatusReg().bit.AlertsPresent) {
        return false;
    }
    
    return true;
}

// ========== Linear Move Methods ==========

bool CoordinatedMotionController::MoveLinear(int32_t endX, int32_t endY) {
    if (!m_initialized) {
        return false;
    }
    
    if (!ValidateLinear(endX, endY)) {
        return false;
    }
    
    // Stop any current motion
    Stop();
    
    // Initialize linear interpolator
    if (!m_linearInterpolator.InitializeLinear(m_currentX, m_currentY,
                                              endX, endY,
                                              m_velocityMax, m_accelMax, SampleRateHz,
                                              0, 0)) {
        return false;
    }
    
    m_motionType = MOTION_TYPE_LINEAR;
    m_activeTargetX = endX;
    m_activeTargetY = endY;
    m_stopCounter = 0;  // Reset stop counter when starting new motion
    m_active = true;
    return true;
}

bool CoordinatedMotionController::MoveLinearAbsolute(int32_t startX, int32_t startY,
                                                    int32_t endX, int32_t endY) {
    if (!m_initialized) {
        return false;
    }
    
    if (!ValidateLinear(endX, endY)) {
        return false;
    }
    
    // Stop any current motion
    Stop();
    
    // Set current position (protected - called from main thread, but motion stopped)
    __disable_irq();
    m_currentX = startX;
    m_currentY = startY;
    __enable_irq();
    
    // Initialize linear interpolator
    if (!m_linearInterpolator.InitializeLinear(startX, startY,
                                              endX, endY,
                                              m_velocityMax, m_accelMax, SampleRateHz,
                                              0, 0)) {
        return false;
    }
    
    m_motionType = MOTION_TYPE_LINEAR;
    m_activeTargetX = endX;
    m_activeTargetY = endY;
    m_stopCounter = 0;  // Reset stop counter when starting new motion
    m_active = true;
    return true;
}

bool CoordinatedMotionController::MoveLinearContinuous(int32_t endX, int32_t endY) {
    if (!m_initialized) {
        return false;
    }
    
    if (!ValidateLinear(endX, endY)) {
        return false;
    }
    
    // Critical section: protect queue operations from ISR
    __disable_irq();
    
    // Check if queue is full
    if (m_linearQueueCount >= ARC_QUEUE_SIZE) {
        __enable_irq();
        return false;
    }
    
    // If no active move, start immediately
    if (!m_active) {
        __enable_irq();
        return MoveLinear(endX, endY);
    }
    
    // Queue the linear move
    uint8_t tail = m_linearQueueTail;
    QueuedLinear& linear = m_linearQueue[tail];
    linear.endX = endX;
    linear.endY = endY;
    linear.valid = true;
    
    m_linearQueueTail = (tail + 1) % ARC_QUEUE_SIZE;
    m_linearQueueCount++;
    
    __enable_irq();
    
    return true;
}

bool CoordinatedMotionController::ProcessNextLinear() {
    // This function is called from ISR context (UpdateFast), so interrupts are already disabled
    // No need for additional critical section protection
    
    if (m_linearQueueCount == 0) {
        return false;
    }
    
    // Get next linear move from queue
    uint8_t head = m_linearQueueHead;
    QueuedLinear& linear = m_linearQueue[head];
    if (!linear.valid) {
        return false;
    }
    
    // Initialize interpolator with new linear move
    // Read current position (volatile, but safe in ISR context)
    int32_t currentX = m_currentX;
    int32_t currentY = m_currentY;
    if (!m_linearInterpolator.InitializeLinear(currentX, currentY,
                                               linear.endX, linear.endY,
                                               m_velocityMax, m_accelMax, SampleRateHz,
                                               0, 0)) {
        // Failed to initialize, remove from queue
        linear.valid = false;
        m_linearQueueHead = (head + 1) % ARC_QUEUE_SIZE;
        m_linearQueueCount--;
        return false;
    }
    
    m_motionType = MOTION_TYPE_LINEAR;
    
    // Remove from queue
    linear.valid = false;
    m_linearQueueHead = (head + 1) % ARC_QUEUE_SIZE;
    m_linearQueueCount--;
    
    return true;
}

bool CoordinatedMotionController::ValidateLinear(int32_t /*endX*/, int32_t /*endY*/) const {
    // Check motors are enabled
    if (m_motorX && !m_motorX->EnableRequest()) {
        return false;
    }
    if (m_motorY && !m_motorY->EnableRequest()) {
        return false;
    }
    
    // Check for alerts
    if (m_motorX && m_motorX->StatusReg().bit.AlertsPresent) {
        return false;
    }
    if (m_motorY && m_motorY->StatusReg().bit.AlertsPresent) {
        return false;
    }
    
    return true;
}

bool CoordinatedMotionController::QueueArc(int32_t centerX, int32_t centerY,
                                          int32_t radius,
                                          double endAngle,
                                          bool clockwise) {
    if (!m_initialized) {
        return false;
    }
    
    if (!ValidateArc(centerX, centerY, radius)) {
        return false;
    }
    
    if (!kUseGrblPlanner) {
        // Critical section: protect queue operations from ISR
        __disable_irq();
        
        // Check if queue is full
        if (m_motionQueueCount >= ARC_QUEUE_SIZE) {
            __enable_irq();
            return false;
        }
        
        // If no active motion, start immediately
        if (!m_active) {
            __enable_irq();
            double startAngle = CalculateStartAngle(centerX, centerY);
            return MoveArc(centerX, centerY, radius, startAngle, endAngle, clockwise);
        }
        
        // Queue the arc motion
        uint8_t tail = m_motionQueueTail;
        QueuedMotion& motion = m_motionQueue[tail];
        motion.type = QUEUED_MOTION_ARC;
        motion.arc.centerX = centerX;
        motion.arc.centerY = centerY;
        motion.arc.radius = radius;
        motion.arc.endAngle = endAngle;
        motion.arc.clockwise = clockwise;
        motion.valid = true;
        
        m_motionQueueTail = (tail + 1) % ARC_QUEUE_SIZE;
        m_motionQueueCount++;
        
        __enable_irq();
        
        return true;
    }

    // Planner-enabled path
    __disable_irq();
    if (m_motionQueueCount >= ARC_QUEUE_SIZE) {
        __enable_irq();
        return false;
    }
    int32_t startX = m_currentX;
    int32_t startY = m_currentY;
    if (m_motionQueueCount > 0) {
        uint8_t prevIdx = (m_motionQueueTail + ARC_QUEUE_SIZE - 1) % ARC_QUEUE_SIZE;
        startX = m_motionQueue[prevIdx].endX;
        startY = m_motionQueue[prevIdx].endY;
    } else if (m_active) {
        startX = m_activeTargetX;
        startY = m_activeTargetY;
    }
    double startAngle = atan2((double)startY - (double)centerY,
                              (double)startX - (double)centerX);
    if (startAngle < 0.0) {
        startAngle += 2.0 * M_PI;
    }
    double endXf = (double)centerX + (double)radius * cos(endAngle);
    double endYf = (double)centerY + (double)radius * sin(endAngle);
    int32_t endX = (int32_t)llround(endXf);
    int32_t endY = (int32_t)llround(endYf);
    double entryDirX = 0.0, entryDirY = 0.0;
    double exitDirX = 0.0, exitDirY = 0.0;
    TangentDir(startAngle, clockwise, entryDirX, entryDirY);
    TangentDir(endAngle, clockwise, exitDirX, exitDirY);
    double angleSpan;
    if (clockwise) {
        angleSpan = (startAngle >= endAngle) ? (startAngle - endAngle) :
                    (startAngle + 2.0 * M_PI - endAngle);
    } else {
        angleSpan = (endAngle >= startAngle) ? (endAngle - startAngle) :
                    (endAngle + 2.0 * M_PI - startAngle);
    }
    uint32_t lengthSteps = (uint32_t)(fabs((double)radius) * angleSpan + 0.5);
    uint8_t tail = m_motionQueueTail;
    QueuedMotion& motion = m_motionQueue[tail];
    motion.type = QUEUED_MOTION_ARC;
    motion.nominalSpeed = m_velocityMax;
    motion.entrySpeed = 0;
    motion.exitSpeed = 0;
    motion.lengthSteps = lengthSteps;
    motion.entryDirX = (float)entryDirX;
    motion.entryDirY = (float)entryDirY;
    motion.exitDirX = (float)exitDirX;
    motion.exitDirY = (float)exitDirY;
    motion.startX = startX;
    motion.startY = startY;
    motion.endX = endX;
    motion.endY = endY;
    motion.arc.centerX = centerX;
    motion.arc.centerY = centerY;
    motion.arc.radius = radius;
    motion.arc.startAngle = startAngle;
    motion.arc.endAngle = endAngle;
    motion.arc.clockwise = clockwise;
    motion.valid = true;
    m_motionQueueTail = (tail + 1) % ARC_QUEUE_SIZE;
    m_motionQueueCount++;
    if (kUseGrblPlanner) {
        RecalculatePlanner();
    }
    if (!m_active) {
        bool started = ProcessNextMotion();
        m_active = started;
    }
    __enable_irq();
    return true;
}

bool CoordinatedMotionController::QueueLinear(int32_t endX, int32_t endY) {
    if (!m_initialized) {
        return false;
    }
    
    if (!ValidateLinear(endX, endY)) {
        return false;
    }
    
    if (!kUseGrblPlanner) {
        // Critical section: protect queue operations from ISR
        __disable_irq();
        
        // Check if queue is full
        if (m_motionQueueCount >= ARC_QUEUE_SIZE) {
            __enable_irq();
            return false;
        }
        
        // If no active motion, start immediately
        if (!m_active) {
            __enable_irq();
            return MoveLinear(endX, endY);
        }
        
        // Queue the linear motion
        uint8_t tail = m_motionQueueTail;
        QueuedMotion& motion = m_motionQueue[tail];
        motion.type = QUEUED_MOTION_LINEAR;
        motion.linear.endX = endX;
        motion.linear.endY = endY;
        motion.valid = true;
        
        m_motionQueueTail = (tail + 1) % ARC_QUEUE_SIZE;
        m_motionQueueCount++;
        
        __enable_irq();
        
        return true;
    }

    // Planner-enabled path
    __disable_irq();
    if (m_motionQueueCount >= ARC_QUEUE_SIZE) {
        __enable_irq();
        return false;
    }
    int32_t startX = m_currentX;
    int32_t startY = m_currentY;
    if (m_motionQueueCount > 0) {
        uint8_t prevIdx = (m_motionQueueTail + ARC_QUEUE_SIZE - 1) % ARC_QUEUE_SIZE;
        startX = m_motionQueue[prevIdx].endX;
        startY = m_motionQueue[prevIdx].endY;
    } else if (m_active) {
        startX = m_activeTargetX;
        startY = m_activeTargetY;
    }
    int32_t dx = endX - startX;
    int32_t dy = endY - startY;
    double dirX = (double)dx;
    double dirY = (double)dy;
    NormalizeVec(dirX, dirY);
    uint32_t lengthSteps = (uint32_t)(SegmentLengthSteps(dx, dy) + 0.5);
    uint8_t tail = m_motionQueueTail;
    QueuedMotion& motion = m_motionQueue[tail];
    motion.type = QUEUED_MOTION_LINEAR;
    motion.nominalSpeed = m_velocityMax;
    motion.entrySpeed = 0;
    motion.exitSpeed = 0;
    motion.lengthSteps = lengthSteps;
    motion.entryDirX = (float)dirX;
    motion.entryDirY = (float)dirY;
    motion.exitDirX = (float)dirX;
    motion.exitDirY = (float)dirY;
    motion.startX = startX;
    motion.startY = startY;
    motion.endX = endX;
    motion.endY = endY;
    motion.linear.endX = endX;
    motion.linear.endY = endY;
    motion.valid = true;
    m_motionQueueTail = (tail + 1) % ARC_QUEUE_SIZE;
    m_motionQueueCount++;
    if (kUseGrblPlanner) {
        RecalculatePlanner();
    }
    if (!m_active) {
        bool started = ProcessNextMotion();
        m_active = started;
    }
    __enable_irq();
    return true;
}

void CoordinatedMotionController::RecalculatePlanner() {
    if (!kUseGrblPlanner) {
        return;  // Planner disabled, exit early
    }
    
    if (m_motionQueueCount == 0) {
        m_motionQueuePlanned = m_motionQueueHead;  // Reset planned pointer
        return;
    }
    
    uint8_t order[ARC_QUEUE_SIZE];
    uint8_t count = m_motionQueueCount;
    uint8_t idx = m_motionQueueHead;
    for (uint8_t i = 0; i < count; i++) {
        order[i] = idx;
        idx = (idx + 1) % ARC_QUEUE_SIZE;
    }
    
    // Optimization: Start planning from planned pointer, not from head
    // Only recalculate blocks that could be affected by new additions
    uint8_t plannedIdx = 0;
    if (kUseGrblPlanner && m_motionQueuePlanned != m_motionQueueHead) {
        // Find where planned pointer is in the order array
        for (uint8_t i = 0; i < count; i++) {
            if (order[i] == m_motionQueuePlanned) {
                plannedIdx = i;
                break;
            }
        }
        // If planned pointer not found (shouldn't happen), start from beginning
        if (plannedIdx >= count) {
            plannedIdx = 0;
        }
    }
    
    // Estimate current speed of active motion (if any) for first queued segment
    uint32_t activeExitSpeed = 0;
    double activeExitSpeedSqr = 0.0;
    if (m_active && count > 0) {
        // When active motion exists, estimate its exit speed
        // Use nominal speed as approximation (could be improved with actual speed tracking)
        activeExitSpeed = m_velocityMax;
        activeExitSpeedSqr = (double)activeExitSpeed * (double)activeExitSpeed;
    }
    
    // Helper function to calculate axis-limited acceleration for junction
    // For now, uses single m_accelMax, but structured for future per-axis limits
    auto GetJunctionAcceleration = [this](double dirX, double dirY) -> double {
        // Calculate junction unit vector magnitude
        double mag = sqrt(dirX * dirX + dirY * dirY);
        if (mag < 1e-9) {
            return (double)m_accelMax;  // Fallback
        }
        
        // Normalize direction (commented out for future per-axis limits)
        // double unitX = dirX / mag;
        // double unitY = dirY / mag;
        
        // For now, use single acceleration limit
        // Future: Could check per-axis limits here:
        // double accelX = m_motorX->GetAccelMax();
        // double accelY = m_motorY->GetAccelMax();
        // return min(accelX / abs(unitX), accelY / abs(unitY));
        
        return (double)m_accelMax;
    };
    
    // Step 1: Calculate initial junction-limited entry speeds (in squared space for efficiency)
    // Work from planned pointer forward to optimize performance
    for (uint8_t i = plannedIdx; i < count; i++) {
        QueuedMotion &cur = m_motionQueue[order[i]];
        double nominalSqr = (double)cur.nominalSpeed * (double)cur.nominalSpeed;
        double maxEntrySqr = nominalSqr;
        
        if (i == 0 && !m_active) {
            maxEntrySqr = 0.0;  // start from rest
        } else if (i == 0 && m_active) {
            // First queued segment: use active motion exit speed
            maxEntrySqr = activeExitSpeedSqr;
        } else if (i > 0) {
            // Calculate junction speed based on angle between segments
            QueuedMotion &prev = m_motionQueue[order[i - 1]];
            double cosTheta = (double)prev.exitDirX * (double)cur.entryDirX +
                              (double)prev.exitDirY * (double)cur.entryDirY;
            
            // Clamp cosTheta to valid range [-1, 1]
            if (cosTheta > 1.0) cosTheta = 1.0;
            if (cosTheta < -1.0) cosTheta = -1.0;
            
            // Calculate half-angle sine: sin(θ/2) = sqrt((1 - cos(θ)) / 2)
            double sinHalf = sqrt(0.5 * (1.0 - cosTheta));
            
            // Calculate junction unit vector for axis-limited acceleration
            double junctionDirX = (double)cur.entryDirX - (double)prev.exitDirX;
            double junctionDirY = (double)cur.entryDirY - (double)prev.exitDirY;
            double junctionAccel = GetJunctionAcceleration(junctionDirX, junctionDirY);
            
            double vmaxSqr;
            // Handle edge cases: straight line (sinHalf ≈ 0) and 180° turn (sinHalf ≈ 1)
            if (sinHalf < 1e-6) {
                // Straight line or very small angle - use minimum of both speeds (squared)
                double prevNominalSqr = (double)prev.nominalSpeed * (double)prev.nominalSpeed;
                vmaxSqr = fmin(prevNominalSqr, nominalSqr);
            } else if (sinHalf > 0.999) {
                // Near 180° turn - very sharp corner, limit speed significantly
                vmaxSqr = junctionAccel * m_junctionDeviationSteps * 0.5;
            } else {
                // Standard junction speed formula (GRBL-style) - compute squared speed directly
                // vmax² = (accel * junctionDeviation * sinHalf) / (1 - sinHalf)
                double denominator = 1.0 - sinHalf;
                if (denominator < 1e-6) {
                    denominator = 1e-6;  // Prevent division by zero
                }
                vmaxSqr = (junctionAccel * m_junctionDeviationSteps * sinHalf) / denominator;
            }
            
            // Junction speed is limited by both segments' nominal speeds (squared)
            double prevNominalSqr = (double)prev.nominalSpeed * (double)prev.nominalSpeed;
            double maxJunctionSqr = fmin(fmin(prevNominalSqr, nominalSqr), vmaxSqr);
            maxEntrySqr = maxJunctionSqr;
        }
        
        // Handle zero-length segments
        if (cur.lengthSteps == 0) {
            cur.entrySpeed = 0;
        } else {
            // Clamp entry speed squared to valid range [0, nominal²]
            if (maxEntrySqr > nominalSqr) {
                maxEntrySqr = nominalSqr;
            }
            if (maxEntrySqr < 0.0) {
                maxEntrySqr = 0.0;
            }
            // Convert squared speed to regular speed (only conversion needed)
            cur.entrySpeed = (uint32_t)sqrt(maxEntrySqr);
        }
    }
    
    // Step 2: Set exit speeds based on look-ahead logic
    // Look ahead: if there's a next move, exit speed = next move's entry speed (junction speed)
    // If no next move, exit speed = 0 (full stop)
    for (uint8_t i = 0; i < count; i++) {
        if (i + 1 < count) {
            // Look ahead: there IS a next move
            // Exit speed = next segment's entry speed (junction speed)
            m_motionQueue[order[i]].exitSpeed = m_motionQueue[order[i + 1]].entrySpeed;
        } else {
            // Look ahead: there is NO next move
            // Exit speed = 0 (full stop)
            m_motionQueue[order[i]].exitSpeed = 0;
        }
    }
    
    // Step 3: Backward pass - verify deceleration is possible (work in squared space)
    // Work backwards from last block to planned pointer
    // Start from last block (exit speed = 0)
    if (count > 0) {
        QueuedMotion &last = m_motionQueue[order[count - 1]];
        // Last block: maximum entry speed that allows deceleration to exit speed (0)
        if (last.lengthSteps > 0 && m_accelMax > 0) {
            double maxEntrySqr = 2.0 * (double)m_accelMax * (double)last.lengthSteps;
            double currentEntrySqr = (double)last.entrySpeed * (double)last.entrySpeed;
            if (currentEntrySqr > maxEntrySqr) {
                last.entrySpeed = (uint32_t)sqrt(maxEntrySqr);
            }
        }
        // Clamp to nominal
        if (last.entrySpeed > last.nominalSpeed) {
            last.entrySpeed = last.nominalSpeed;
        }
    }
    
    // Continue backward pass for remaining blocks
    for (int i = (int)count - 2; i >= (int)plannedIdx; i--) {
        QueuedMotion &cur = m_motionQueue[order[i]];
        QueuedMotion &next = m_motionQueue[order[i + 1]];
        
        // Work in squared space: entry_speed_sqr = next.entry_speed_sqr + 2 * accel * distance
        double nextEntrySqr = (double)next.entrySpeed * (double)next.entrySpeed;
        double maxEntrySqr;
        
        if (cur.lengthSteps > 0 && m_accelMax > 0) {
            maxEntrySqr = nextEntrySqr + 2.0 * (double)m_accelMax * (double)cur.lengthSteps;
        } else if (cur.lengthSteps == 0) {
            // Zero-length segment: entry speed must equal exit speed
            maxEntrySqr = nextEntrySqr;
        } else {
            maxEntrySqr = nextEntrySqr;  // No acceleration limit
        }
        
        double currentEntrySqr = (double)cur.entrySpeed * (double)cur.entrySpeed;
        double nominalSqr = (double)cur.nominalSpeed * (double)cur.nominalSpeed;
        
        // Limit to minimum of: calculated max, current entry, and nominal
        maxEntrySqr = fmin(fmin(maxEntrySqr, currentEntrySqr), nominalSqr);
        
        // Convert back to regular speed
        cur.entrySpeed = (uint32_t)sqrt(maxEntrySqr);
        
        // Update exit speed to match next entry speed (maintain consistency)
        cur.exitSpeed = next.entrySpeed;
    }
    
    // Step 4: Forward pass - optimize speeds (allow increases when acceleration is possible)
    // Work forward from planned pointer, can increase speeds when possible
    uint8_t optimalPlanned = plannedIdx;  // Track optimal planned pointer
    for (uint8_t i = plannedIdx + 1; i < count; i++) {
        QueuedMotion &prev = m_motionQueue[order[i - 1]];
        QueuedMotion &cur = m_motionQueue[order[i]];
        
        // Work in squared space
        double prevEntrySqr = (double)prev.entrySpeed * (double)prev.entrySpeed;
        double curEntrySqr = (double)cur.entrySpeed * (double)cur.entrySpeed;
        double nominalSqr = (double)cur.nominalSpeed * (double)cur.nominalSpeed;
        
        // Check if we can accelerate from prev to cur
        if (prev.lengthSteps > 0 && m_accelMax > 0) {
            // Maximum achievable entry speed from previous segment
            double maxEntryFromPrevSqr = prevEntrySqr + 2.0 * (double)m_accelMax * (double)prev.lengthSteps;
            
            if (curEntrySqr < maxEntryFromPrevSqr) {
                // Can accelerate! Increase current entry speed
                curEntrySqr = fmin(maxEntryFromPrevSqr, nominalSqr);
                cur.entrySpeed = (uint32_t)sqrt(curEntrySqr);
                // Update prev's exit speed to match (maintain consistency)
                prev.exitSpeed = cur.entrySpeed;
                // This block is now optimally planned
                optimalPlanned = i;
            } else if (curEntrySqr > maxEntryFromPrevSqr) {
                // Cannot reach current entry speed, reduce it
                curEntrySqr = maxEntryFromPrevSqr;
                cur.entrySpeed = (uint32_t)sqrt(curEntrySqr);
                prev.exitSpeed = cur.entrySpeed;
            }
        }
        
        // If entry speed equals nominal (maximum), this and all previous blocks are optimal
        if (cur.entrySpeed == cur.nominalSpeed) {
            optimalPlanned = i;
        }
    }
    
    // Update planned pointer to skip optimally planned blocks next time
    if (optimalPlanned < count) {
        m_motionQueuePlanned = order[optimalPlanned];
    } else {
        m_motionQueuePlanned = m_motionQueueHead;
    }
    
    // Step 5: Handle active motion transition (first segment)
    // If active motion exists, ensure first queued segment can transition smoothly
    if (m_active && count > 0) {
        QueuedMotion &first = m_motionQueue[order[0]];
        
        // Work in squared space
        double firstEntrySqr = (double)first.entrySpeed * (double)first.entrySpeed;
        double maxFromActiveSqr = activeExitSpeedSqr + 2.0 * (double)m_accelMax * 1000.0;  // Assume some distance
        
        if (firstEntrySqr > maxFromActiveSqr) {
            double nominalSqr = (double)first.nominalSpeed * (double)first.nominalSpeed;
            firstEntrySqr = fmin(maxFromActiveSqr, nominalSqr);
            first.entrySpeed = (uint32_t)sqrt(firstEntrySqr);
        }
    }
    
    // Step 6: Final validation - ensure all speeds are within bounds
    for (uint8_t i = 0; i < count; i++) {
        QueuedMotion &cur = m_motionQueue[order[i]];
        
        // Ensure entry speed doesn't exceed nominal
        if (cur.entrySpeed > cur.nominalSpeed) {
            cur.entrySpeed = cur.nominalSpeed;
        }
        
        // Ensure exit speed is achievable from entry speed
        if (cur.exitSpeed > cur.entrySpeed && cur.lengthSteps > 0 && m_accelMax > 0) {
            // Can we accelerate from entrySpeed to exitSpeed?
            double entrySqr = (double)cur.entrySpeed * (double)cur.entrySpeed;
            double exitSqr = (double)cur.exitSpeed * (double)cur.exitSpeed;
            double maxExitSqr = entrySqr + 2.0 * (double)m_accelMax * (double)cur.lengthSteps;
            
            if (exitSqr > maxExitSqr) {
                cur.exitSpeed = (uint32_t)sqrt(maxExitSqr);
            }
        }
        
        // Note: entrySpeed and exitSpeed are uint32_t (unsigned), so they cannot be negative
        // No need to check for negative values
    }
}

bool CoordinatedMotionController::ProcessNextMotion() {
    // This function is called from ISR context (UpdateFast), so interrupts are already disabled
    // No need for additional critical section protection
    
    if (m_motionQueueCount == 0) {
        return false;
    }
    
    // Get next motion from unified queue
    uint8_t head = m_motionQueueHead;
    QueuedMotion& motion = m_motionQueue[head];
    if (!motion.valid) {
        return false;
    }
    
    bool success = false;
    
    if (motion.type == QUEUED_MOTION_ARC) {
        double startAngle = kUseGrblPlanner
            ? motion.arc.startAngle
            : CalculateStartAngle(motion.arc.centerX, motion.arc.centerY);
        uint32_t nominal = kUseGrblPlanner ? motion.nominalSpeed : m_velocityMax;
        uint32_t entry = kUseGrblPlanner ? motion.entrySpeed : 0;
        uint32_t exit = kUseGrblPlanner ? motion.exitSpeed : 0;
        if (m_arcInterpolator.InitializeArc(motion.arc.centerX, motion.arc.centerY,
                                            motion.arc.radius,
                                            startAngle, motion.arc.endAngle,
                                            motion.arc.clockwise,
                                            nominal, m_accelMax, SampleRateHz,
                                            entry, exit)) {
            m_motionType = MOTION_TYPE_ARC;
            m_currentAngle = motion.arc.endAngle;
            m_activeTargetX = motion.endX;
            m_activeTargetY = motion.endY;
            success = true;
        }
    } else if (motion.type == QUEUED_MOTION_LINEAR) {
        int32_t currentX = m_currentX;
        int32_t currentY = m_currentY;
        uint32_t nominal = kUseGrblPlanner ? motion.nominalSpeed : m_velocityMax;
        uint32_t entry = kUseGrblPlanner ? motion.entrySpeed : 0;
        uint32_t exit = kUseGrblPlanner ? motion.exitSpeed : 0;
        if (m_linearInterpolator.InitializeLinear(currentX, currentY,
                                                 motion.linear.endX, motion.linear.endY,
                                                 nominal, m_accelMax, SampleRateHz,
                                                 entry, exit)) {
            m_motionType = MOTION_TYPE_LINEAR;
            m_activeTargetX = motion.endX;
            m_activeTargetY = motion.endY;
            success = true;
        }
    }
    
    // Remove from queue
    motion.valid = false;
    m_motionQueueHead = (head + 1) % ARC_QUEUE_SIZE;
    m_motionQueueCount--;
    
    // Update planned pointer if it was pointing to the removed block
    if (kUseGrblPlanner) {
        if (m_motionQueuePlanned == head) {
            m_motionQueuePlanned = m_motionQueueHead;  // Move planned pointer forward
        }
    }
    
    return success;
}

// ========== Unit Conversion Support ==========

bool CoordinatedMotionController::SetMechanicalParamsX(uint32_t stepsPerRev, double pitch,
                                                      UnitType pitchUnit, double gearRatio) {
    if (stepsPerRev == 0 || pitch <= 0.0 || gearRatio <= 0.0) {
        return false;
    }
    
    m_mechanicalConfigX.stepsPerRevolution = stepsPerRev;
    m_mechanicalConfigX.pitch = pitch;
    m_mechanicalConfigX.pitchUnit = pitchUnit;
    m_mechanicalConfigX.gearRatio = gearRatio;
    
    UnitConverter::CalculateConversionFactors(m_mechanicalConfigX);
    m_unitsConfiguredX = true;
    
    return true;
}

bool CoordinatedMotionController::SetMechanicalParamsY(uint32_t stepsPerRev, double pitch,
                                                       UnitType pitchUnit, double gearRatio) {
    if (stepsPerRev == 0 || pitch <= 0.0 || gearRatio <= 0.0) {
        return false;
    }
    
    m_mechanicalConfigY.stepsPerRevolution = stepsPerRev;
    m_mechanicalConfigY.pitch = pitch;
    m_mechanicalConfigY.pitchUnit = pitchUnit;
    m_mechanicalConfigY.gearRatio = gearRatio;
    
    UnitConverter::CalculateConversionFactors(m_mechanicalConfigY);
    m_unitsConfiguredY = true;
    
    return true;
}

bool CoordinatedMotionController::MoveArcInches(double centerX, double centerY, double radius,
                                                double startAngle, double endAngle, bool clockwise) {
    if (!m_unitsConfiguredX || !m_unitsConfiguredY) {
        return false;
    }
    
    int32_t centerXSteps = UnitConverter::DistanceToSteps(centerX, UNIT_INCHES, m_mechanicalConfigX);
    int32_t centerYSteps = UnitConverter::DistanceToSteps(centerY, UNIT_INCHES, m_mechanicalConfigY);
    int32_t radiusSteps = UnitConverter::DistanceToSteps(radius, UNIT_INCHES, m_mechanicalConfigX);
    
    return MoveArc(centerXSteps, centerYSteps, radiusSteps, startAngle, endAngle, clockwise);
}

bool CoordinatedMotionController::MoveArcMM(double centerX, double centerY, double radius,
                                            double startAngle, double endAngle, bool clockwise) {
    if (!m_unitsConfiguredX || !m_unitsConfiguredY) {
        return false;
    }
    
    int32_t centerXSteps = UnitConverter::DistanceToSteps(centerX, UNIT_MM, m_mechanicalConfigX);
    int32_t centerYSteps = UnitConverter::DistanceToSteps(centerY, UNIT_MM, m_mechanicalConfigY);
    int32_t radiusSteps = UnitConverter::DistanceToSteps(radius, UNIT_MM, m_mechanicalConfigX);
    
    return MoveArc(centerXSteps, centerYSteps, radiusSteps, startAngle, endAngle, clockwise);
}

bool CoordinatedMotionController::MoveLinearInches(double endX, double endY) {
    if (!m_unitsConfiguredX || !m_unitsConfiguredY) {
        return false;
    }
    
    int32_t endXSteps = UnitConverter::DistanceToSteps(endX, UNIT_INCHES, m_mechanicalConfigX);
    int32_t endYSteps = UnitConverter::DistanceToSteps(endY, UNIT_INCHES, m_mechanicalConfigY);
    
    return MoveLinear(endXSteps, endYSteps);
}

bool CoordinatedMotionController::MoveLinearMM(double endX, double endY) {
    if (!m_unitsConfiguredX || !m_unitsConfiguredY) {
        return false;
    }
    
    int32_t endXSteps = UnitConverter::DistanceToSteps(endX, UNIT_MM, m_mechanicalConfigX);
    int32_t endYSteps = UnitConverter::DistanceToSteps(endY, UNIT_MM, m_mechanicalConfigY);
    
    return MoveLinear(endXSteps, endYSteps);
}

void CoordinatedMotionController::SetPosition(int32_t x, int32_t y) {
    // Critical section: protect position writes from ISR
    __disable_irq();
    m_currentX = x;
    m_currentY = y;
    __enable_irq();
}

void CoordinatedMotionController::FeedRateInchesPerMin(double feedRate) {
    if (!m_unitsConfiguredX || !m_unitsConfiguredY) {
        return;
    }
    // Use X-axis config for conversion (assuming same units)
    uint32_t stepsPerSec = UnitConverter::FeedRateToStepsPerSec(feedRate, FR_UNIT_INCHES_PER_MIN, m_mechanicalConfigX);
    ArcVelMax(stepsPerSec);
}

void CoordinatedMotionController::FeedRateMMPerMin(double feedRate) {
    if (!m_unitsConfiguredX || !m_unitsConfiguredY) {
        return;
    }
    uint32_t stepsPerSec = UnitConverter::FeedRateToStepsPerSec(feedRate, FR_UNIT_MM_PER_MIN, m_mechanicalConfigX);
    ArcVelMax(stepsPerSec);
}

void CoordinatedMotionController::FeedRateMMPerSec(double feedRate) {
    if (!m_unitsConfiguredX || !m_unitsConfiguredY) {
        return;
    }
    uint32_t stepsPerSec = UnitConverter::FeedRateToStepsPerSec(feedRate, FR_UNIT_MM_PER_SEC, m_mechanicalConfigX);
    ArcVelMax(stepsPerSec);
}

double CoordinatedMotionController::CurrentXInches() const {
    if (!m_unitsConfiguredX) {
        return 0.0;
    }
    return UnitConverter::StepsToDistance(m_currentX, UNIT_INCHES, m_mechanicalConfigX);
}

double CoordinatedMotionController::CurrentYInches() const {
    if (!m_unitsConfiguredY) {
        return 0.0;
    }
    return UnitConverter::StepsToDistance(m_currentY, UNIT_INCHES, m_mechanicalConfigY);
}

double CoordinatedMotionController::CurrentXMM() const {
    if (!m_unitsConfiguredX) {
        return 0.0;
    }
    return UnitConverter::StepsToDistance(m_currentX, UNIT_MM, m_mechanicalConfigX);
}

double CoordinatedMotionController::CurrentYMM() const {
    if (!m_unitsConfiguredY) {
        return 0.0;
    }
    return UnitConverter::StepsToDistance(m_currentY, UNIT_MM, m_mechanicalConfigY);
}

bool CoordinatedMotionController::GetDebugInfo(uint32_t& stepsRemaining, int32_t& currentX, int32_t& currentY) const {
    currentX = m_currentX;
    currentY = m_currentY;
    
    if (m_motionType == MOTION_TYPE_LINEAR) {
        stepsRemaining = m_linearInterpolator.StepsRemaining();
        return m_active;
    } else if (m_motionType == MOTION_TYPE_ARC) {
        // Arc interpolator doesn't have StepsRemaining, use 0
        stepsRemaining = 0;
        return m_active;
    } else {
        stepsRemaining = 0;
        return false;
    }
}

} // ClearCore namespace
