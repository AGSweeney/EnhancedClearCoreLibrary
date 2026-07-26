// EXCERPT — source: libClearCore/src/CoordinatedMotionController.cpp
// EVIDENCE: E1 | symbol: CoordinatedMotionController::Initialize | lines: 140-163

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
