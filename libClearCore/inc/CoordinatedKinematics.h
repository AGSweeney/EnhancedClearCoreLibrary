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
    \file CoordinatedKinematics.h
    Machine kinematics for CoordinatedMotionController.

    The controller plans and interpolates in Cartesian X/Y. This header maps
    that path onto the two motors. It has no hardware dependencies, so the
    host tests use it directly.

    CoreXY (and H-bot, which has the same equations):

        A = X + Y          X = (A + B) / 2
        B = X - Y          Y = (A - B) / 2

    All values are in steps. One Cartesian step is the head travel for one
    step of belt, so both motors need the same steps per unit. Moving one
    motor alone moves the head diagonally. If a motor turns the wrong way for
    your belt routing, invert it at the drive or with the connector polarity;
    to exchange A and B, pass the motors to Initialize() the other way round.
**/

#ifndef __COORDINATEDKINEMATICS_H__
#define __COORDINATEDKINEMATICS_H__

#include <math.h>
#include <stdint.h>

namespace ClearCore {

/**
    \enum CoordKinematics
    How the X/Y path maps onto the controller's two motors.
**/
enum CoordKinematics {
    KINEMATICS_CARTESIAN = 0,   ///< motor 1 = X, motor 2 = Y
    KINEMATICS_COREXY = 1,      ///< motor 1 = A = X+Y, motor 2 = B = X-Y (CoreXY, H-bot)
};

namespace Kinematics {

/** Cartesian position or per-sample steps to motor steps. **/
inline void ToMotor(CoordKinematics k, int32_t x, int32_t y, int32_t &a, int32_t &b) {
    if (k == KINEMATICS_COREXY) {
        a = x + y;
        b = x - y;
    }
    else {
        a = x;
        b = y;
    }
}

/**
    Motor positions to the Cartesian position.

    In CoreXY a single motor step moves the head half a step in X and in Y,
    so when A+B is odd the motors sit between Cartesian steps. The result is
    then rounded down in X (Y = A - X), and the function returns false.
**/
inline bool ToCartesian(CoordKinematics k, int32_t a, int32_t b, int32_t &x, int32_t &y) {
    if (k != KINEMATICS_COREXY) {
        x = a;
        y = b;
        return true;
    }
    const int64_t sum = (int64_t)a + (int64_t)b;
    const int64_t odd = sum & 1;    // 1 for odd sums of either sign (two's complement)
    x = (int32_t)((sum - odd) / 2);
    y = (int32_t)((int64_t)a - (int64_t)x);
    return odd == 0;
}

/**
    Fastest motor speed for a path speed of 1 in direction (ux, uy), a unit
    vector. Cartesian: max(|ux|, |uy|), at most 1. CoreXY: |ux| + |uy|
    (the larger of |ux+uy| and |ux-uy|), from 1 on an axis move up to sqrt(2)
    on a 45-degree move, where one motor does all the work.
**/
inline double MotorSpeedFactor(CoordKinematics k, double ux, double uy) {
    const double ax = fabs(ux);
    const double ay = fabs(uy);
    if (k == KINEMATICS_COREXY) {
        return ax + ay;
    }
    return ax > ay ? ax : ay;
}

/** The largest MotorSpeedFactor over all directions. **/
inline double MaxMotorSpeedFactor(CoordKinematics k) {
    return k == KINEMATICS_COREXY ? 1.41421356237309505 : 1.0;
}

/**
    The largest MotorSpeedFactor along an arc that starts at angle startAngle
    (radians, centre to start point) and sweeps span radians, clockwise or
    not.

    The tangent at angle t is perpendicular to the radius, and both factors
    are unchanged by a 90-degree turn, so the factor at a point equals the
    factor of its radius angle. Both factors peak only at multiples of 45
    degrees, so the maximum is at an end or at one of those angles inside the
    sweep.
**/
inline double ArcMotorSpeedFactor(CoordKinematics k, double startAngle, double span,
                                  bool clockwise) {
    const double kTwoPi = 6.28318530717958648;
    const double kEighth = 0.785398163397448310;   // 45 degrees
    if (span >= kTwoPi - 1e-9) {
        return MaxMotorSpeedFactor(k);
    }
    if (span < 0.0) {
        span = -span;
    }
    const double lo = clockwise ? startAngle - span : startAngle;
    const double hi = lo + span;
    double best = MotorSpeedFactor(k, cos(lo), sin(lo));
    const double atEnd = MotorSpeedFactor(k, cos(hi), sin(hi));
    if (atEnd > best) {
        best = atEnd;
    }
    for (double c = ceil(lo / kEighth) * kEighth; c <= hi; c += kEighth) {
        const double f = MotorSpeedFactor(k, cos(c), sin(c));
        if (f > best) {
            best = f;
        }
    }
    return best;
}

/**
    Largest per-motor change in a junction's unit direction vector, (dDirX,
    dDirY) = entry direction - exit direction. The dVmax corner limit is
    dVmax / JunctionMotorDelta: in CoreXY a corner that changes X and Y
    together changes one motor by the sum.
**/
inline double JunctionMotorDelta(CoordKinematics k, double dDirX, double dDirY) {
    return MotorSpeedFactor(k, dDirX, dDirY);
}

/**
    Path speed for a segment whose worst-case motor speed factor is factor:
    pathSpeed, lowered so that no motor exceeds motorSpeedMax (steps/s).
    motorSpeedMax 0 means no motor limit. Never returns 0 for a non-zero path
    speed (the interpolators reject a zero speed).
**/
inline uint32_t CapPathSpeed(uint32_t pathSpeed, uint32_t motorSpeedMax, double factor) {
    if (motorSpeedMax == 0 || factor <= 1e-9) {
        return pathSpeed;
    }
    const double cap = (double)motorSpeedMax / factor;
    if (cap >= (double)pathSpeed) {
        return pathSpeed;
    }
    const uint32_t capped = (uint32_t)cap;
    return capped > 0 ? capped : 1;
}

} // Kinematics namespace

} // ClearCore namespace

#endif // __COORDINATEDKINEMATICS_H__
