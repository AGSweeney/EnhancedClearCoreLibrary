// EXCERPT — source: libClearCore/inc/MotorDriver.h
// EVIDENCE: E1 | symbol: CoordinatedMotionMode | lines: 457-522

    /**
        \brief Enable or disable coordinated motion mode
        
        When enabled, this motor will be controlled by a CoordinatedMotionController
        instead of its internal StepGenerator. The controller will generate
        coordinated step commands for multiple motors.
        
        \code{.cpp}
        // Enable coordinated motion mode for M-0 and M-1
        CoordinatedMotionController controller;
        controller.Initialize(&ConnectorM0, &ConnectorM1);
        ConnectorM0.CoordinatedMotionMode(true, &controller);
        ConnectorM1.CoordinatedMotionMode(true, &controller);
        \endcode
        
        \param[in] enable True to enable coordinated mode, false to disable
        \param[in] controller Pointer to the CoordinatedMotionController instance
        
        \return True if mode change successful
    **/
    bool CoordinatedMotionMode(bool enable, CoordinatedMotionController* controller);

    /**
        \brief Check if motor is in coordinated motion mode
        
        \return True if in coordinated mode
    **/
    bool IsCoordinatedMode() const {
        return m_coordinatedMode;
    }

    /**
        \brief Set coordinated steps directly (internal use by CoordinatedMotionController)
        
        This method bypasses the StepGenerator and directly sets the step count
        for the next sample period. Only valid when in coordinated mode.
        
        \param[in] steps Number of steps to output (signed)
    **/
    void SetCoordinatedSteps(int32_t steps);

    // ========== Unit Conversion Support ==========
    
    /**
        \brief Configure mechanical parameters for unit conversion
        
        Sets up the motor's mechanical configuration (steps per revolution,
        lead screw pitch, etc.) to enable unit-based move commands.
        
        \code{.cpp}
        // Configure motor: 800 steps/rev, 5mm pitch ball screw
        ConnectorM0.SetMechanicalParams(800, 5.0, UNIT_MM);
        
        // Now can use unit-based moves
        ConnectorM0.MoveMM(285.75);  // Move 285.75 mm
        \endcode
        
        \param[in] stepsPerRev Motor steps per revolution
        \param[in] pitch Lead screw pitch (or gear ratio for rotary)
        \param[in] pitchUnit Units for pitch (UNIT_MM, UNIT_INCHES, etc.)
        \param[in] gearRatio Gear ratio (1.0 = direct drive, default)
        
        \return true if configuration successful
    **/
    bool SetMechanicalParams(uint32_t stepsPerRev, double pitch,
                            UnitType pitchUnit, double gearRatio = 1.0);
