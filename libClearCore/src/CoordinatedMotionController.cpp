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

CoordinatedMotionController::CoordinatedMotionController()
    : m_motorX(nullptr),
      m_motorY(nullptr),
      m_motionQueueHead(0),
      m_motionQueueTail(0),
      m_motionQueueCount(0),
      m_arcQueueHead(0),
      m_arcQueueTail(0),
      m_arcQueueCount(0),
      m_linearQueueHead(0),
      m_linearQueueTail(0),
      m_linearQueueCount(0),
      m_active(false),
      m_initialized(false),
      m_motionType(MOTION_TYPE_NONE),
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
                                         m_velocityMax, SampleRateHz)) {
        return false;
    }
    
    // Update current position
    m_arcInterpolator.GetCurrentPosition(m_currentX, m_currentY);
    m_motionType = MOTION_TYPE_ARC;
    
    // Calculate current angle
    double dx = (double)(m_currentX - centerX);
    double dy = (double)(m_currentY - centerY);
    m_currentAngle = atan2(dy, dx);
    if (m_currentAngle < 0) {
        m_currentAngle += 2.0 * M_PI;
    }
    
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
    
    // Check if queue is full
    if (m_arcQueueCount >= ARC_QUEUE_SIZE) {
        return false;
    }
    
    // If no active arc, start immediately
    if (!m_active) {
        double startAngle = CalculateStartAngle(centerX, centerY);
        return MoveArc(centerX, centerY, radius, startAngle, endAngle, clockwise);
    }
    
    // Queue the arc
    QueuedArc& arc = m_arcQueue[m_arcQueueTail];
    arc.centerX = centerX;
    arc.centerY = centerY;
    arc.radius = radius;
    arc.endAngle = endAngle;
    arc.clockwise = clockwise;
    arc.valid = true;
    
    m_arcQueueTail = (m_arcQueueTail + 1) % ARC_QUEUE_SIZE;
    m_arcQueueCount++;
    
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
    if (!m_active || !m_initialized) {
        return;
    }
    
    bool motionComplete = false;
    
    // Check motion type and completion
    if (m_motionType == MOTION_TYPE_ARC) {
        if (m_arcInterpolator.IsArcComplete()) {
            // Update position from interpolator
            m_arcInterpolator.GetCurrentPosition(m_currentX, m_currentY);
            
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
            m_linearInterpolator.GetCurrentPosition(m_currentX, m_currentY);
            
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
        // Apply steps to motors
        m_motorX->SetCoordinatedSteps(stepsX);
        m_motorY->SetCoordinatedSteps(stepsY);
        
        // Update current position
        m_currentX += stepsX;
        m_currentY += stepsY;
    }
}

bool CoordinatedMotionController::ProcessNextArc() {
    if (m_arcQueueCount == 0) {
        return false;
    }
    
    // Get next arc from queue
    QueuedArc& arc = m_arcQueue[m_arcQueueHead];
    if (!arc.valid) {
        return false;
    }
    
    // Calculate start angle (tangent to current position)
    double startAngle = CalculateStartAngle(arc.centerX, arc.centerY);
    
    // Initialize interpolator with new arc
    if (!m_arcInterpolator.InitializeArc(arc.centerX, arc.centerY, arc.radius,
                                         startAngle, arc.endAngle, arc.clockwise,
                                         m_velocityMax, SampleRateHz)) {
        // Failed to initialize, remove from queue
        arc.valid = false;
        m_arcQueueHead = (m_arcQueueHead + 1) % ARC_QUEUE_SIZE;
        m_arcQueueCount--;
        return false;
    }
    
    m_motionType = MOTION_TYPE_ARC;
    
    // Update current angle
    m_currentAngle = arc.endAngle;
    
    // Remove from queue
    arc.valid = false;
    m_arcQueueHead = (m_arcQueueHead + 1) % ARC_QUEUE_SIZE;
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

bool CoordinatedMotionController::ValidateArc(int32_t centerX, int32_t centerY, int32_t radius) const {
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
                                              m_velocityMax, SampleRateHz)) {
        return false;
    }
    
    m_motionType = MOTION_TYPE_LINEAR;
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
    
    // Set current position
    m_currentX = startX;
    m_currentY = startY;
    
    // Initialize linear interpolator
    if (!m_linearInterpolator.InitializeLinear(startX, startY,
                                              endX, endY,
                                              m_velocityMax, SampleRateHz)) {
        return false;
    }
    
    m_motionType = MOTION_TYPE_LINEAR;
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
    
    // Check if queue is full
    if (m_linearQueueCount >= ARC_QUEUE_SIZE) {
        return false;
    }
    
    // If no active move, start immediately
    if (!m_active) {
        return MoveLinear(endX, endY);
    }
    
    // Queue the linear move
    QueuedLinear& linear = m_linearQueue[m_linearQueueTail];
    linear.endX = endX;
    linear.endY = endY;
    linear.valid = true;
    
    m_linearQueueTail = (m_linearQueueTail + 1) % ARC_QUEUE_SIZE;
    m_linearQueueCount++;
    
    return true;
}

bool CoordinatedMotionController::ProcessNextLinear() {
    if (m_linearQueueCount == 0) {
        return false;
    }
    
    // Get next linear move from queue
    QueuedLinear& linear = m_linearQueue[m_linearQueueHead];
    if (!linear.valid) {
        return false;
    }
    
    // Initialize interpolator with new linear move
    if (!m_linearInterpolator.InitializeLinear(m_currentX, m_currentY,
                                               linear.endX, linear.endY,
                                               m_velocityMax, SampleRateHz)) {
        // Failed to initialize, remove from queue
        linear.valid = false;
        m_linearQueueHead = (m_linearQueueHead + 1) % ARC_QUEUE_SIZE;
        m_linearQueueCount--;
        return false;
    }
    
    m_motionType = MOTION_TYPE_LINEAR;
    
    // Remove from queue
    linear.valid = false;
    m_linearQueueHead = (m_linearQueueHead + 1) % ARC_QUEUE_SIZE;
    m_linearQueueCount--;
    
    return true;
}

bool CoordinatedMotionController::ValidateLinear(int32_t endX, int32_t endY) const {
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
    
    // Check if queue is full
    if (m_motionQueueCount >= ARC_QUEUE_SIZE) {
        return false;
    }
    
    // If no active motion, start immediately
    if (!m_active) {
        double startAngle = CalculateStartAngle(centerX, centerY);
        return MoveArc(centerX, centerY, radius, startAngle, endAngle, clockwise);
    }
    
    // Queue the arc motion
    QueuedMotion& motion = m_motionQueue[m_motionQueueTail];
    motion.type = QUEUED_MOTION_ARC;
    motion.arc.centerX = centerX;
    motion.arc.centerY = centerY;
    motion.arc.radius = radius;
    motion.arc.endAngle = endAngle;
    motion.arc.clockwise = clockwise;
    motion.valid = true;
    
    m_motionQueueTail = (m_motionQueueTail + 1) % ARC_QUEUE_SIZE;
    m_motionQueueCount++;
    
    return true;
}

bool CoordinatedMotionController::QueueLinear(int32_t endX, int32_t endY) {
    if (!m_initialized) {
        return false;
    }
    
    if (!ValidateLinear(endX, endY)) {
        return false;
    }
    
    // Check if queue is full
    if (m_motionQueueCount >= ARC_QUEUE_SIZE) {
        return false;
    }
    
    // If no active motion, start immediately
    if (!m_active) {
        return MoveLinear(endX, endY);
    }
    
    // Queue the linear motion
    QueuedMotion& motion = m_motionQueue[m_motionQueueTail];
    motion.type = QUEUED_MOTION_LINEAR;
    motion.linear.endX = endX;
    motion.linear.endY = endY;
    motion.valid = true;
    
    m_motionQueueTail = (m_motionQueueTail + 1) % ARC_QUEUE_SIZE;
    m_motionQueueCount++;
    
    return true;
}

bool CoordinatedMotionController::ProcessNextMotion() {
    if (m_motionQueueCount == 0) {
        return false;
    }
    
    // Get next motion from unified queue
    QueuedMotion& motion = m_motionQueue[m_motionQueueHead];
    if (!motion.valid) {
        return false;
    }
    
    bool success = false;
    
    if (motion.type == QUEUED_MOTION_ARC) {
        // Calculate start angle (tangent to current position)
        double startAngle = CalculateStartAngle(motion.arc.centerX, motion.arc.centerY);
        
        // Initialize arc interpolator
        if (m_arcInterpolator.InitializeArc(motion.arc.centerX, motion.arc.centerY,
                                            motion.arc.radius,
                                            startAngle, motion.arc.endAngle,
                                            motion.arc.clockwise,
                                            m_velocityMax, SampleRateHz)) {
            m_motionType = MOTION_TYPE_ARC;
            m_currentAngle = motion.arc.endAngle;
            success = true;
        }
    } else if (motion.type == QUEUED_MOTION_LINEAR) {
        // Initialize linear interpolator
        if (m_linearInterpolator.InitializeLinear(m_currentX, m_currentY,
                                                 motion.linear.endX, motion.linear.endY,
                                                 m_velocityMax, SampleRateHz)) {
            m_motionType = MOTION_TYPE_LINEAR;
            success = true;
        }
    }
    
    // Remove from queue
    motion.valid = false;
    m_motionQueueHead = (m_motionQueueHead + 1) % ARC_QUEUE_SIZE;
    m_motionQueueCount--;
    
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

void CoordinatedMotionController::ArcFeedRateInchesPerMin(double feedRate) {
    if (!m_unitsConfiguredX || !m_unitsConfiguredY) {
        return;
    }
    // Use X-axis config for conversion (assuming same units)
    uint32_t stepsPerSec = UnitConverter::FeedRateToStepsPerSec(feedRate, FR_UNIT_INCHES_PER_MIN, m_mechanicalConfigX);
    ArcVelMax(stepsPerSec);
}

void CoordinatedMotionController::ArcFeedRateMMPerMin(double feedRate) {
    if (!m_unitsConfiguredX || !m_unitsConfiguredY) {
        return;
    }
    uint32_t stepsPerSec = UnitConverter::FeedRateToStepsPerSec(feedRate, FR_UNIT_MM_PER_MIN, m_mechanicalConfigX);
    ArcVelMax(stepsPerSec);
}

void CoordinatedMotionController::ArcFeedRateMMPerSec(double feedRate) {
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

} // ClearCore namespace
