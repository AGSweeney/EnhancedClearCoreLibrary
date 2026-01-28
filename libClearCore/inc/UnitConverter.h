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

/**
    \file UnitConverter.h
    ClearCore Unit Conversion System

    This file provides unit conversion utilities for converting between
    physical units (inches, mm, degrees) and step counts.
**/

#ifndef __UNITCONVERTER_H__
#define __UNITCONVERTER_H__

#include <stdint.h>

namespace ClearCore {

/**
    \enum UnitType
    \brief Physical unit types for distance measurements
**/
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
        \brief Convert steps to distance
        
        \param[in] steps Number of steps
        \param[in] unit Desired output unit type
        \param[in] config Motor mechanical configuration
        
        \return Distance in specified unit
    **/
    static double StepsToDistance(int32_t steps, UnitType unit,
                                 const MotorMechanicalConfig& config);
    
    /**
        \brief Convert feed rate to steps per second
        
        \param[in] feedRate Feed rate in specified unit
        \param[in] unit Feed rate unit type
        \param[in] config Motor mechanical configuration
        
        \return Steps per second
    **/
    static uint32_t FeedRateToStepsPerSec(double feedRate, FeedRateUnit unit,
                                          const MotorMechanicalConfig& config);
    
    /**
        \brief Convert steps per second to feed rate
        
        \param[in] stepsPerSec Steps per second
        \param[in] unit Desired output feed rate unit type
        \param[in] config Motor mechanical configuration
        
        \return Feed rate in specified unit
    **/
    static double StepsPerSecToFeedRate(uint32_t stepsPerSec, FeedRateUnit unit,
                                       const MotorMechanicalConfig& config);
    
    /**
        \brief Calculate conversion factors from mechanical config
        
        \param[in,out] config Configuration structure (factors will be calculated)
    **/
    static void CalculateConversionFactors(MotorMechanicalConfig& config);
    
private:
    /**
        \brief Convert unit to base unit (inches for linear, revolutions for rotary)
        
        \param[in] value Value to convert
        \param[in] unit Unit type
        \param[in] isRotary true if rotary motion (degrees/revolutions)
        
        \return Value in base units
    **/
    static double ConvertToBaseUnit(double value, UnitType unit, bool isRotary);
    
    /**
        \brief Convert from base unit to specified unit
        
        \param[in] value Value in base units
        \param[in] unit Desired output unit type
        \param[in] isRotary true if rotary motion (degrees/revolutions)
        
        \return Value in specified units
    **/
    static double ConvertFromBaseUnit(double value, UnitType unit, bool isRotary);
};

} // ClearCore namespace

#endif // __UNITCONVERTER_H__
