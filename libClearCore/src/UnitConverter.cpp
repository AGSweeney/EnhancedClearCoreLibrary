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

#include "UnitConverter.h"
#include <math.h>

namespace ClearCore {

// Unit conversion constants
const double UnitConverter::MM_PER_INCH = 25.4;
const double UnitConverter::CM_PER_INCH = 2.54;
const double UnitConverter::METERS_PER_INCH = 0.0254;
const double UnitConverter::DEGREES_PER_REV = 360.0;
const double UnitConverter::SEC_PER_MIN = 60.0;

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

double UnitConverter::StepsToDistance(int32_t steps, UnitType unit,
                                      const MotorMechanicalConfig& config) {
    if (!config.IsValid()) {
        return 0.0;
    }
    
    // Convert steps to base units
    bool isRotary = (unit == UNIT_REVOLUTIONS || unit == UNIT_DEGREES);
    
    double baseUnits;
    if (isRotary) {
        // Rotary: steps / (stepsPerRev * gearRatio) = revolutions
        baseUnits = (double)steps / (config.stepsPerRevolution * config.gearRatio);
    } else {
        // Linear: (steps / (stepsPerRev * gearRatio)) * pitch = distance
        double revolutions = (double)steps / (config.stepsPerRevolution * config.gearRatio);
        double pitchInches = ConvertToBaseUnit(config.pitch, config.pitchUnit, false);
        baseUnits = revolutions * pitchInches;
    }
    
    // Convert from base units to requested unit
    return ConvertFromBaseUnit(baseUnits, unit, isRotary);
}

uint32_t UnitConverter::FeedRateToStepsPerSec(double feedRate, FeedRateUnit unit,
                                              const MotorMechanicalConfig& config) {
    if (!config.IsValid() || feedRate < 0.0) {
        return 0;
    }
    
    // Convert feed rate to base units per second
    double baseUnitsPerSec;
    
    switch (unit) {
        case FR_UNIT_STEPS_PER_SEC:
            return (uint32_t)round(feedRate);
            
        case FR_UNIT_INCHES_PER_MIN:
            baseUnitsPerSec = feedRate / SEC_PER_MIN;  // inches/sec
            break;
            
        case FR_UNIT_INCHES_PER_SEC:
            baseUnitsPerSec = feedRate;  // inches/sec
            break;
            
        case FR_UNIT_MM_PER_MIN:
            baseUnitsPerSec = (feedRate / MM_PER_INCH) / SEC_PER_MIN;  // inches/sec
            break;
            
        case FR_UNIT_MM_PER_SEC:
            baseUnitsPerSec = feedRate / MM_PER_INCH;  // inches/sec
            break;
            
        case FR_UNIT_RPM:
            baseUnitsPerSec = feedRate / SEC_PER_MIN;  // rev/sec
            break;
            
        case FR_UNIT_RPS:
            baseUnitsPerSec = feedRate;  // rev/sec
            break;
            
        default:
            return 0;
    }
    
    // Determine if rotary or linear
    bool isRotary = (unit == FR_UNIT_RPM || unit == FR_UNIT_RPS);
    
    if (isRotary) {
        // Rotary: rev/sec * stepsPerRev * gearRatio = steps/sec
        double stepsPerSec = baseUnitsPerSec * config.stepsPerRevolution * config.gearRatio;
        return (uint32_t)round(stepsPerSec);
    } else {
        // Linear: (inches/sec / pitch) * stepsPerRev * gearRatio = steps/sec
        double pitchInches = ConvertToBaseUnit(config.pitch, config.pitchUnit, false);
        double revPerSec = baseUnitsPerSec / pitchInches;
        double stepsPerSec = revPerSec * config.stepsPerRevolution * config.gearRatio;
        return (uint32_t)round(stepsPerSec);
    }
}

double UnitConverter::StepsPerSecToFeedRate(uint32_t stepsPerSec, FeedRateUnit unit,
                                           const MotorMechanicalConfig& config) {
    if (!config.IsValid()) {
        return 0.0;
    }
    
    // Convert steps/sec to base units per second
    bool isRotary = (unit == FR_UNIT_RPM || unit == FR_UNIT_RPS);
    
    double baseUnitsPerSec;
    if (isRotary) {
        // Rotary: steps/sec / (stepsPerRev * gearRatio) = rev/sec
        baseUnitsPerSec = (double)stepsPerSec / (config.stepsPerRevolution * config.gearRatio);
    } else {
        // Linear: (steps/sec / (stepsPerRev * gearRatio)) * pitch = inches/sec
        double revPerSec = (double)stepsPerSec / (config.stepsPerRevolution * config.gearRatio);
        double pitchInches = ConvertToBaseUnit(config.pitch, config.pitchUnit, false);
        baseUnitsPerSec = revPerSec * pitchInches;
    }
    
    // Convert to requested feed rate unit
    switch (unit) {
        case FR_UNIT_STEPS_PER_SEC:
            return (double)stepsPerSec;
            
        case FR_UNIT_INCHES_PER_MIN:
            return baseUnitsPerSec * SEC_PER_MIN;
            
        case FR_UNIT_INCHES_PER_SEC:
            return baseUnitsPerSec;
            
        case FR_UNIT_MM_PER_MIN:
            return (baseUnitsPerSec * MM_PER_INCH) * SEC_PER_MIN;
            
        case FR_UNIT_MM_PER_SEC:
            return baseUnitsPerSec * MM_PER_INCH;
            
        case FR_UNIT_RPM:
            return baseUnitsPerSec * SEC_PER_MIN;
            
        case FR_UNIT_RPS:
            return baseUnitsPerSec;
            
        default:
            return 0.0;
    }
}

void UnitConverter::CalculateConversionFactors(MotorMechanicalConfig& config) {
    if (!config.IsValid()) {
        config.stepsPerUnit = 1.0;
        config.unitsPerStep = 1.0;
        return;
    }
    
    // Calculate steps per unit based on pitch unit
    double pitchInches = ConvertToBaseUnit(config.pitch, config.pitchUnit, false);
    
    if (pitchInches > 0.0) {
        // Linear: steps per inch = (stepsPerRev * gearRatio) / pitch
        config.stepsPerUnit = (config.stepsPerRevolution * config.gearRatio) / pitchInches;
        config.unitsPerStep = 1.0 / config.stepsPerUnit;
    } else {
        // Rotary or invalid
        config.stepsPerUnit = config.stepsPerRevolution * config.gearRatio;
        config.unitsPerStep = 1.0 / config.stepsPerUnit;
    }
}

double UnitConverter::ConvertToBaseUnit(double value, UnitType unit, bool isRotary) {
    if (isRotary) {
        // Rotary: base unit is revolutions
        switch (unit) {
            case UNIT_REVOLUTIONS:
                return value;
            case UNIT_DEGREES:
                return value / DEGREES_PER_REV;
            case UNIT_STEPS:
                // This shouldn't be called for steps in rotary mode
                return value;
            default:
                return value;
        }
    } else {
        // Linear: base unit is inches
        switch (unit) {
            case UNIT_INCHES:
                return value;
            case UNIT_MM:
                return value / MM_PER_INCH;
            case UNIT_CM:
                return value / CM_PER_INCH;
            case UNIT_METERS:
                return value / METERS_PER_INCH;
            case UNIT_STEPS:
                // This shouldn't be called for steps
                return value;
            default:
                return value;
        }
    }
}

double UnitConverter::ConvertFromBaseUnit(double value, UnitType unit, bool isRotary) {
    if (isRotary) {
        // Rotary: base unit is revolutions
        switch (unit) {
            case UNIT_REVOLUTIONS:
                return value;
            case UNIT_DEGREES:
                return value * DEGREES_PER_REV;
            case UNIT_STEPS:
                // This shouldn't be called for steps
                return value;
            default:
                return value;
        }
    } else {
        // Linear: base unit is inches
        switch (unit) {
            case UNIT_INCHES:
                return value;
            case UNIT_MM:
                return value * MM_PER_INCH;
            case UNIT_CM:
                return value * CM_PER_INCH;
            case UNIT_METERS:
                return value * METERS_PER_INCH;
            case UNIT_STEPS:
                // This shouldn't be called for steps
                return value;
            default:
                return value;
        }
    }
}

} // ClearCore namespace
