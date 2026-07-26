// EXCERPT — source: libClearCore/inc/LinearInterpolator.h
// EVIDENCE: E1 | symbol: LinearInterpolator | lines: 45-94

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
                         uint32_t velocityMax, uint32_t accelMax,
                         uint16_t sampleRateHz,
                         uint32_t entrySpeed = 0,
                         uint32_t exitSpeed = 0);
    
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
        // Position check is primary - only complete when actually at target
        // Step count is just an estimate and can be inaccurate
        if (m_currentX == m_endX && m_currentY == m_endY) {
            return true;
        }
        // If not at target but stepsRemaining is 0, step count was wrong - not complete yet
        return false;
    }
