// EXCERPT — source: libClearCore/inc/CoordinatedMotionController.h
// EVIDENCE: E1 | symbol: CoordinatedMotionController | lines: 52-161

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
        \param[in] startAngle Start angle in radians (from arc center to start point; must match
                             the segment that was parsed into the motion block — recomputing from
                             queued step endpoints can drift vs. I/J geometry and yield wrong span)
        \param[in] endAngle End angle in radians (from arc center to end point)
        \param[in] clockwise Direction (true = clockwise)
        
        \return true if arc command accepted
    **/
    bool QueueArc(int32_t centerX, int32_t centerY,
                 int32_t radius,
                 double startAngle,
                 double endAngle,
                 bool clockwise);

    /**
        \brief Maximum number of segments the planner queue can hold.
        Use this to cap the batch size in the host firmware so it never tries to submit
        more segments than fit, which would cause QueueArc/QueueLinear to fail.
    **/
    uint8_t PlannerQueueCapacity() const {
        return ARC_QUEUE_SIZE;
    }

    /**
        \brief When true, QueueArc/QueueLinear (planner path) only enqueue; call
        StartDeferredMotionIfPending() after batching so RecalculatePlanner sees
        the full chain (smoother arc quadrants, etc.). Default false.
    **/
    void SetDeferMotionQueueStart(bool defer) {
        m_deferQueueStart = defer;
    }

    /**
        \brief Start first queued motion if idle and queue non-empty (after batch enqueue).
        \return true if motion started
    **/
    bool StartDeferredMotionIfPending();
    
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
