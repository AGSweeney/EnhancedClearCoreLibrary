// EXCERPT — source: libClearCore/inc/UnitConverter.h
// EVIDENCE: E1 | symbol: UnitConverter | lines: 42-131

enum UnitType {
    UNIT_STEPS,         ///< Raw steps (default, no conversion)
    UNIT_INCHES,        ///< Inches
    UNIT_MM,            ///< Millimeters
    UNIT_CM,            ///< Centimeters
    UNIT_METERS,        ///< Meters
    UNIT_REVOLUTIONS,   ///< Motor revolutions
    UNIT_DEGREES        ///< Degrees (for rotary axes)
};

/**
    \enum FeedRateUnit
    \brief Physical unit types for feed rate/velocity measurements
**/
enum FeedRateUnit {
    FR_UNIT_STEPS_PER_SEC,    ///< Steps per second (default)
    FR_UNIT_INCHES_PER_MIN,   ///< Inches per minute
    FR_UNIT_INCHES_PER_SEC,   ///< Inches per second
    FR_UNIT_MM_PER_MIN,       ///< Millimeters per minute
    FR_UNIT_MM_PER_SEC,       ///< Millimeters per second
    FR_UNIT_RPM,              ///< Revolutions per minute
    FR_UNIT_RPS               ///< Revolutions per second
};

/**
    \struct MotorMechanicalConfig
    \brief Configuration structure for motor mechanical parameters
**/
struct MotorMechanicalConfig {
    uint32_t stepsPerRevolution;  ///< Motor steps per revolution
    double pitch;                 ///< Lead screw pitch or gear ratio
    UnitType pitchUnit;           ///< Units for pitch (UNIT_MM, UNIT_INCHES, etc.)
    double gearRatio;             ///< Gear ratio (1.0 = direct drive)
    bool inverted;                ///< Direction inversion flag
    
    // Calculated conversion factors (set by SetMechanicalParams)
    double stepsPerUnit;          ///< Steps per unit (calculated)
    double unitsPerStep;          ///< Units per step (calculated)
    
    /**
        \brief Default constructor
    **/
    MotorMechanicalConfig()
        : stepsPerRevolution(1),
          pitch(1.0),
          pitchUnit(UNIT_STEPS),
          gearRatio(1.0),
          inverted(false),
          stepsPerUnit(1.0),
          unitsPerStep(1.0) {
    }
    
    /**
        \brief Check if configuration is valid
        \return true if configuration is valid
    **/
    bool IsValid() const {
        return stepsPerRevolution > 0 && pitch > 0.0 && gearRatio > 0.0;
    }
};

/**
    \class UnitConverter
    \brief Unit conversion utility class
    
    Provides static methods for converting between physical units
    and step counts based on motor mechanical configuration.
**/
class UnitConverter {
public:
    // Unit conversion constants
    static const double MM_PER_INCH;
    static const double CM_PER_INCH;
    static const double METERS_PER_INCH;
    static const double DEGREES_PER_REV;
    static const double SEC_PER_MIN;
    
    /**
        \brief Convert distance to steps
        
        \param[in] distance Distance in specified unit
        \param[in] unit Unit type of distance
        \param[in] config Motor mechanical configuration
        
        \return Number of steps (signed)
    **/
    static int32_t DistanceToSteps(double distance, UnitType unit,
                                   const MotorMechanicalConfig& config);
    
    /**
