// EXCERPT — source: libClearCore/inc/ArcInterpolator.h
// EVIDENCE: E1 | symbol: ArcInterpolator | lines: 43-120

struct ArcSegment {
    int64_t centerXQx;       // Arc center X position (Q15 fixed-point)
    int64_t centerYQx;       // Arc center Y position (Q15 fixed-point)
    int64_t radiusQx;        // Arc radius (Q15 fixed-point)
    int32_t startAngleQx;    // Start angle (Q15 fixed-point, 0-2π = 0-32768)
    int32_t endAngleQx;      // End angle (Q15 fixed-point, 0-2π = 0-32768)
    int32_t spanQx;          // Total arc span (>0, ≤ TWO_PI_QX); used to clamp remaining so
                             // an overshoot beyond endAngle (current > end for CCW, or current
                             // < end for CW) doesn't wrap remaining to ~2π and produce a circle.
    bool clockwise;          // Direction of rotation
    uint32_t totalSteps;     // Total steps in this arc segment
    uint32_t stepsRemaining; // Steps remaining in this arc
    
    ArcSegment()
        : centerXQx(0),
          centerYQx(0),
          radiusQx(0),
          startAngleQx(0),
          endAngleQx(0),
          spanQx(0),
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
                       uint16_t sampleRateHz,
                       uint32_t entrySpeed = 0,
                       uint32_t exitSpeed = 0);

    /**
        \brief Update planned exit tangential speed while the arc is running (planner lookahead).
    **/
    void SetExitSpeed(uint32_t exitSpeed) {
        m_exitSpeed = exitSpeed;
    }

    /**
        \brief Generate next step pair for current arc
        
        \param[out] stepsX Steps for X motor
        \param[out] stepsY Steps for Y motor
        
        \return true if steps generated, false if arc complete
    **/
    bool GenerateNextSteps(int32_t &stepsX, int32_t &stepsY);
