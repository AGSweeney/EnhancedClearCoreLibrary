/**
 * \file SysUtils.cpp
 *
 * \brief System utility functions
 *
 * Copyright (c) 2020 Teknic, Inc.
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

#include <same53.h>
#include "SysUtils.h"

// Oscillator output into XOSC1
#define __CLEARCORE_OSC_HZ      (25000000)              // 25 MHz

/**
 * Update GClk frequency
 *
 * @brief  Updates the divisor on the specified GClk to
 *         generate the requested frequency
 */
void GClkFreqUpdate(uint8_t gclkIndex, uint32_t freqReq) {
    // This adjustment is only supported for GClks that use XOSC1 as the src
    if (GCLK->GENCTRL[gclkIndex].bit.SRC != GCLK_GENCTRL_SRC_XOSC1_Val) {
        return;
    }

    // Configure the clock divisor for the requested frequency
    GCLK->GENCTRL[gclkIndex].bit.DIV = __CLEARCORE_OSC_HZ / freqReq;
    while (GCLK->SYNCBUSY.vec.GENCTRL & (1 << gclkIndex)) {
        continue;
    }
}
