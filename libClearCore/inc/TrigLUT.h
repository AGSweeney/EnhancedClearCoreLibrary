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
    \file TrigLUT.h
    Trigonometric Lookup Tables for Fixed-Point Arc Interpolation

    Provides fast sin/cos lookup tables for Q15 fixed-point angle calculations.
**/

#ifndef __TRIGLUT_H__
#define __TRIGLUT_H__

#include <stdint.h>

namespace ClearCore {

/**
    \brief Trigonometric lookup table size
    Table contains values for angles 0 to 2π (0 to 32768 in Q15)
**/
#define TRIG_LUT_SIZE 1024

/**
    \brief Get sine value from lookup table
    
    \param[in] angleQx Angle in Q15 format (0-32768 represents 0-2π)
    
    \return Sine value in Q15 format (-32768 to +32768 represents -1.0 to +1.0)
**/
int32_t SinQx(int32_t angleQx);

/**
    \brief Get cosine value from lookup table
    
    \param[in] angleQx Angle in Q15 format (0-32768 represents 0-2π)
    
    \return Cosine value in Q15 format (-32768 to +32768 represents -1.0 to +1.0)
**/
int32_t CosQx(int32_t angleQx);

} // ClearCore namespace

#endif // __TRIGLUT_H__
