// EXCERPT — source: libClearCore/src/UnitConverter.cpp
// EVIDENCE: E1 | symbol: UnitConverter::DistanceToSteps | lines: 35-60

int32_t UnitConverter::DistanceToSteps(double distance, UnitType unit,
                                        const MotorMechanicalConfig& config) {
    if (!config.IsValid()) {
        return 0;
    }
    
    // Convert distance to base units
    bool isRotary = (unit == UNIT_REVOLUTIONS || unit == UNIT_DEGREES);
    double baseUnits = ConvertToBaseUnit(distance, unit, isRotary);
    
    // For linear motion: baseUnits is in inches
    // For rotary motion: baseUnits is in revolutions
    
    if (isRotary) {
        // Rotary: revolutions * stepsPerRev * gearRatio
        double steps = baseUnits * config.stepsPerRevolution * config.gearRatio;
        return (int32_t)round(steps);
    } else {
        // Linear: (distance / pitch) * stepsPerRev * gearRatio
        // First convert pitch to inches if needed
        double pitchInches = ConvertToBaseUnit(config.pitch, config.pitchUnit, false);
        double revolutions = baseUnits / pitchInches;
        double steps = revolutions * config.stepsPerRevolution * config.gearRatio;
        return (int32_t)round(steps);
    }
}
