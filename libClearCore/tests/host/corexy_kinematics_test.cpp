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

/*
 * Host test for CoordinatedKinematics.h (CoreXY). Checks:
 *   - ToMotor / ToCartesian round trip, including negative and odd A+B
 *   - MotorSpeedFactor on axis and diagonal moves
 *   - ArcMotorSpeedFactor against brute-force sampling of random arcs
 *   - LinearInterpolator and ArcInterpolator driven at 5 kHz, their per-sample
 *     steps sent through ToMotor the way CoordinatedMotionController::OutputSteps
 *     does: the motor counts must stay exactly X+Y / X-Y every sample, land on
 *     the end point, and with the path speed capped by CapPathSpeed no motor
 *     may run faster than the motor limit
 *
 *     make && ./corexy_kinematics_test [count] [seed]
 */
#include "ArcInterpolator.h"
#include "CoordinatedKinematics.h"
#include "LinearInterpolator.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>

using namespace ClearCore;

namespace {

const uint16_t kSampleRateHz = 5000;
const int kWindow = 100;   // samples (20 ms) for the motor speed check
int g_fail = 0;

double Rnd(double a, double b) { return a + (b - a) * (rand() / (double)RAND_MAX); }

void Fail(const char *msg, int k, double a = 0, double b = 0) {
    printf("FAIL %s (case %d: %.4f %.4f)\n", msg, k, a, b);
    g_fail++;
}

/* Tracks motor A/B counts and their peak speed over a sliding window. */
struct MotorTrack {
    int32_t a, b;
    int32_t histA[kWindow], histB[kWindow];
    long n;
    double peak;   // steps/s, either motor
    void Start(int32_t x, int32_t y) {
        Kinematics::ToMotor(KINEMATICS_COREXY, x, y, a, b);
        n = 0;
        peak = 0.0;
    }
    void Sample(int32_t sx, int32_t sy) {
        int32_t da, db;
        Kinematics::ToMotor(KINEMATICS_COREXY, sx, sy, da, db);
        a += da;
        b += db;
        const int i = (int)(n % kWindow);
        if (n >= kWindow) {
            const double va = fabs((double)(a - histA[i])) * kSampleRateHz / kWindow;
            const double vb = fabs((double)(b - histB[i])) * kSampleRateHz / kWindow;
            if (va > peak) peak = va;
            if (vb > peak) peak = vb;
        }
        histA[i] = a;
        histB[i] = b;
        n++;
    }
    /* Motors must match the interpolator's Cartesian position exactly. */
    bool Matches(int32_t x, int32_t y) const {
        int32_t ea, eb;
        Kinematics::ToMotor(KINEMATICS_COREXY, x, y, ea, eb);
        return ea == a && eb == b;
    }
};

bool SpeedOk(double peak, uint32_t motorMax) {
    /* 2% plus a step per sample of quantisation in the window. */
    return peak <= motorMax * 1.02 + (double)kSampleRateHz / kWindow;
}

void TestTransforms() {
    for (int k = 0; k < 200000; k++) {
        const int32_t x = (int32_t)Rnd(-4e8, 4e8), y = (int32_t)Rnd(-4e8, 4e8);
        int32_t a, b, x2, y2;
        Kinematics::ToMotor(KINEMATICS_COREXY, x, y, a, b);
        if (a != x + y || b != x - y) Fail("ToMotor", k, x, y);
        if (!Kinematics::ToCartesian(KINEMATICS_COREXY, a, b, x2, y2) || x2 != x || y2 != y) {
            Fail("round trip", k, x, y);
        }
        /* A one-step nudge of B makes A+B odd: X rounds down, Y = A - X. */
        if (Kinematics::ToCartesian(KINEMATICS_COREXY, a, b + 1, x2, y2)) Fail("odd not flagged", k, x, y);
        if (x2 != x || y2 != y) Fail("odd rounding", k, x2 - x, y2 - y);
        if (Kinematics::ToCartesian(KINEMATICS_COREXY, a, b - 1, x2, y2)) Fail("odd not flagged", k, x, y);
        if (x2 != x - 1 || y2 != a - x2) Fail("odd rounding neg", k, x2 - x, y2 - y);
        Kinematics::ToMotor(KINEMATICS_CARTESIAN, x, y, a, b);
        if (a != x || b != y) Fail("cartesian", k, x, y);
    }
    const double r2 = sqrt(2.0);
    struct { double ux, uy, f; } c[] = {
        {1, 0, 1}, {0, 1, 1}, {-1, 0, 1}, {1 / r2, 1 / r2, r2}, {1 / r2, -1 / r2, r2}, {0.6, 0.8, 1.4},
    };
    for (int k = 0; k < 6; k++) {
        const double f = Kinematics::MotorSpeedFactor(KINEMATICS_COREXY, c[k].ux, c[k].uy);
        if (fabs(f - c[k].f) > 1e-12) Fail("MotorSpeedFactor", k, f, c[k].f);
    }
    if (Kinematics::JunctionMotorDelta(KINEMATICS_CARTESIAN, 0.5, -1.5) != 1.5) Fail("junction cart", 0);
    if (Kinematics::JunctionMotorDelta(KINEMATICS_COREXY, 0.5, -1.5) != 2.0) Fail("junction corexy", 0);
    if (Kinematics::CapPathSpeed(10000, 0, r2) != 10000) Fail("cap off", 0);
    if (Kinematics::CapPathSpeed(10000, 20000, r2) != 10000) Fail("cap not needed", 0);
    if (Kinematics::CapPathSpeed(10000, 10000, r2) != 7071) Fail("cap", 0);
}

void TestArcFactor(int count) {
    const CoordKinematics ks[2] = {KINEMATICS_CARTESIAN, KINEMATICS_COREXY};
    for (int k = 0; k < count; k++) {
        const CoordKinematics kin = ks[k & 1];
        const double start = Rnd(-7, 7);
        const double span = (k % 7 == 0) ? Rnd(0, 0.2) : Rnd(0, 6.5);
        const bool cw = (k & 2) != 0;
        const double got = Kinematics::ArcMotorSpeedFactor(kin, start, span, cw);
        const double s = span > 2 * M_PI ? 2 * M_PI : span;
        double brute = 0.0;
        for (int i = 0; i <= 20000; i++) {
            const double t = start + (cw ? -1 : 1) * s * i / 20000.0;
            const double f = Kinematics::MotorSpeedFactor(kin, cos(t), sin(t));
            if (f > brute) brute = f;
        }
        if (got < brute - 1e-9 || got > brute + 1e-6) Fail("ArcMotorSpeedFactor", k, got, brute);
    }
}

void TestLinear(int count) {
    for (int k = 0; k < count; k++) {
        const int32_t sx = (int32_t)Rnd(-50000, 50000), sy = (int32_t)Rnd(-50000, 50000);
        int32_t ex, ey;
        if (k < 4) {   // axis moves and both diagonals
            const int32_t d = 40000;
            const int32_t dx[4] = {d, 0, d, d}, dy[4] = {0, d, d, -d};
            ex = sx + dx[k];
            ey = sy + dy[k];
        }
        else {
            ex = (int32_t)Rnd(-50000, 50000);
            ey = (int32_t)Rnd(-50000, 50000);
        }
        const uint32_t motorMax = (uint32_t)Rnd(4000, 30000);
        const uint32_t pathVel = (uint32_t)(motorMax * Rnd(0.8, 1.6));
        const uint32_t acc = (uint32_t)Rnd(50000, 400000);
        double ux = ex - sx, uy = ey - sy;
        const double len = sqrt(ux * ux + uy * uy);
        if (len < 1) continue;
        ux /= len;
        uy /= len;
        const uint32_t vel = Kinematics::CapPathSpeed(
            pathVel, motorMax, Kinematics::MotorSpeedFactor(KINEMATICS_COREXY, ux, uy));
        LinearInterpolator li;
        if (!li.InitializeLinear(sx, sy, ex, ey, vel, acc, kSampleRateHz, 0, 0)) {
            Fail("linear init", k);
            continue;
        }
        MotorTrack m;
        m.Start(sx, sy);
        long guard = 0;
        while (!li.IsLinearComplete() && guard++ < 20000000L) {
            int32_t stx = 0, sty = 0;
            li.GenerateNextSteps(stx, sty);
            m.Sample(stx, sty);
            int32_t x, y;
            li.GetCurrentPosition(x, y);
            if (!m.Matches(x, y)) {
                Fail("linear motors off the path", k, x, y);
                break;
            }
        }
        int32_t x, y;
        if (!Kinematics::ToCartesian(KINEMATICS_COREXY, m.a, m.b, x, y) || x != ex || y != ey) {
            Fail("linear end", k, x - ex, y - ey);
        }
        if (!SpeedOk(m.peak, motorMax)) Fail("linear motor overspeed", k, m.peak, motorMax);
        /* An X-only move: both motors turn at the path speed, the same way. */
        if (k == 0 && (m.a - (sx + sy) != ex - sx || m.b - (sx - sy) != ex - sx)) Fail("x-only", k);
        /* A 45-degree move: only motor A turns. */
        if (k == 2 && m.b != sx - sy) Fail("diagonal moved B", k);
    }
}

void TestArcs(int count) {
    for (int k = 0; k < count; k++) {
        const int32_t cx = (int32_t)Rnd(-20000, 20000), cy = (int32_t)Rnd(-20000, 20000);
        const double R = Rnd(500, 20000);
        const double a0 = Rnd(0, 2 * M_PI);
        const double sweep = Rnd(0.3, 2 * M_PI - 0.3);
        const bool cw = (k & 1) != 0;
        const double a1 = cw ? a0 - sweep : a0 + sweep;
        const int32_t sx = cx + (int32_t)llround(R * cos(a0)), sy = cy + (int32_t)llround(R * sin(a0));
        const int32_t ex = cx + (int32_t)llround(R * cos(a1)), ey = cy + (int32_t)llround(R * sin(a1));
        const uint32_t motorMax = (uint32_t)Rnd(4000, 30000);
        const uint32_t pathVel = (uint32_t)(motorMax * Rnd(0.8, 1.6));
        const uint32_t acc = (uint32_t)Rnd(50000, 400000);
        const double ra0 = atan2((double)sy - cy, (double)sx - cx);
        const double ra1 = atan2((double)ey - cy, (double)ex - cx);
        const uint32_t vel = Kinematics::CapPathSpeed(
            pathVel, motorMax, Kinematics::ArcMotorSpeedFactor(KINEMATICS_COREXY, a0, sweep, cw));
        ArcInterpolator ai;
        if (!ai.InitializeArc(cx, cy, (int32_t)llround(R), ra0, ra1, cw, vel, acc, kSampleRateHz, 0, 0)) {
            Fail("arc init", k);
            continue;
        }
        ai.SetExactEndpoints(sx, sy, ex, ey);
        MotorTrack m;
        m.Start(sx, sy);
        int32_t x = sx, y = sy;
        long guard = 0;
        while (!ai.IsArcComplete() && guard++ < 20000000L) {
            int32_t stx = 0, sty = 0;
            ai.GenerateNextSteps(stx, sty);
            x += stx;
            y += sty;
            m.Sample(stx, sty);
            if (!m.Matches(x, y)) {
                Fail("arc motors off the path", k, x, y);
                break;
            }
        }
        int32_t fx, fy;
        if (!Kinematics::ToCartesian(KINEMATICS_COREXY, m.a, m.b, fx, fy) || fx != ex || fy != ey) {
            Fail("arc end", k, fx - ex, fy - ey);
        }
        if (!SpeedOk(m.peak, motorMax)) Fail("arc motor overspeed", k, m.peak, motorMax);
    }
}

} /* namespace */

int main(int argc, char **argv) {
    const int count = argc > 1 ? atoi(argv[1]) : 2000;
    const unsigned seed = argc > 2 ? (unsigned)atoi(argv[2]) : 12345u;
    srand(seed);
    TestTransforms();
    TestArcFactor(count * 5);
    TestLinear(count);
    TestArcs(count);
    printf("CoreXY kinematics: transforms, %d arc factors, %d lines, %d arcs (seed %u)\n", count * 5, count,
           count, seed);
    printf(g_fail ? "FAILED (%d)\n" : "PASS\n", g_fail);
    return g_fail ? 1 : 0;
}
