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
 * Host test for ArcInterpolator (no hardware). Drives the interpolator the way
 * CoordinatedMotionController does at 5 kHz and checks every arc for:
 *   - landing exactly on its end point (with SetExactEndpoints)
 *   - IsArcComplete only after the last sample, GenerateNextSteps agreeing
 *   - no step bursts (per-sample steps within what the feed allows)
 *   - staying on the circle
 *   - sweeping the correct half (apex on the expected side of the chord)
 *   - not running faster than the commanded tangential speed
 * Arcs are random 180-degree arcs like a glue path (80 steps/mm, radius
 * 20-300 mm, 60-1500 rpm at 800 steps/rev, accel 0.5-6 M mm/min^2, a third of
 * them diagonal), plus fixed quarter and three-quarter turns.
 *
 *     make && ./arc_interpolator_test [count] [seed]
 */
#include "ArcInterpolator.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>

using namespace ClearCore;

namespace {

const uint16_t kSampleRateHz = 5000;
int g_fail = 0;

double Rnd(double a, double b) { return a + (b - a) * (rand() / (double)RAND_MAX); }

struct Result {
    int32_t endErrX, endErrY;
    int maxStep;
    double maxRadialErr;
    double apexSide;    // signed distance of the farthest point from the chord (+ = left of start->end)
    long samples;
};

Result RunArc(int32_t sx, int32_t sy, int32_t cx, int32_t cy, int32_t ex, int32_t ey, bool cw,
              uint32_t vel, uint32_t acc, double sweepSign)
{
    ArcInterpolator a;
    const double dx = sx - cx, dy = sy - cy;
    const double r = sqrt(dx * dx + dy * dy);
    const double a0 = atan2(dy, dx);
    const double a1 = atan2((double)ey - cy, (double)ex - cx);
    Result res = {0, 0, 0, 0.0, 0.0, 0};
    if (!a.InitializeArc(cx, cy, (int32_t)llround(r), a0, a1, cw, vel, acc, kSampleRateHz, 0, 0)) {
        printf("FAIL init\n");
        g_fail++;
        return res;
    }
    a.SetExactEndpoints(sx, sy, ex, ey);
    int32_t x = sx, y = sy;
    const double ux = ex - sx, uy = ey - sy, ul = sqrt(ux * ux + uy * uy);
    while (!a.IsArcComplete()) {
        int32_t stx = 0, sty = 0;
        const bool more = a.GenerateNextSteps(stx, sty);
        x += stx;
        y += sty;
        res.samples++;
        if (abs(stx) > res.maxStep) res.maxStep = abs(stx);
        if (abs(sty) > res.maxStep) res.maxStep = abs(sty);
        const double rr = sqrt((double)(x - cx) * (x - cx) + (double)(y - cy) * (y - cy));
        if (fabs(rr - r) > res.maxRadialErr) res.maxRadialErr = fabs(rr - r);
        if (ul > 0.0) {
            const double side = (ux * (y - sy) - uy * (x - sx)) / ul;
            if (side * sweepSign > res.apexSide * sweepSign) res.apexSide = side;
        }
        if (!more && !a.IsArcComplete()) {
            printf("FAIL GenerateNextSteps returned false before the arc completed\n");
            g_fail++;
            break;
        }
        if (res.samples > 50000000L) {
            printf("FAIL runaway arc\n");
            g_fail++;
            break;
        }
    }
    res.endErrX = x - ex;
    res.endErrY = y - ey;
    return res;
}

/* Common checks. radius in steps; minSweep = expected |apexSide| at least. */
void Check(const char *what, int k, const Result &r, double radius, double minApex, double sweepSign,
           double pathSteps, uint32_t vel)
{
    if (r.endErrX != 0 || r.endErrY != 0) {
        printf("FAIL %s %d: end error (%d, %d) steps\n", what, k, (int)r.endErrX, (int)r.endErrY);
        g_fail++;
    }
    const int allowed = (int)ceil((double)vel / kSampleRateHz) + 2;
    if (r.maxStep > allowed) {
        printf("FAIL %s %d: burst of %d steps in one sample (feed allows %d)\n", what, k, r.maxStep, allowed);
        g_fail++;
    }
    if (r.maxRadialErr > 2.0) {
        printf("FAIL %s %d: left the circle by %.2f steps\n", what, k, r.maxRadialErr);
        g_fail++;
    }
    if (minApex > 0.0 && r.apexSide * sweepSign < minApex - 2.0) {
        printf("FAIL %s %d: wrong half or short sweep (apex %.1f, expected %.1f)\n", what, k, r.apexSide, minApex);
        g_fail++;
    }
    const double minSamples = pathSteps / vel * kSampleRateHz;
    if (r.samples < minSamples * 0.98) {
        printf("FAIL %s %d: faster than commanded (%ld samples < %.0f)\n", what, k, r.samples, minSamples);
        g_fail++;
    }
    (void)radius;
}

} /* namespace */

int main(int argc, char **argv)
{
    const int count = argc > 1 ? atoi(argv[1]) : 4000;
    const unsigned seed = argc > 2 ? (unsigned)atoi(argv[2]) : 12345u;
    srand(seed);
    const double spm = 80.0;   /* steps per mm */
    int worstEnd = 0, worstStep = 0;
    double worstRad = 0.0;

    /* Random 180-degree arcs: end = start + 2 * (I, J). */
    for (int k = 0; k < count; k++) {
        const double R = Rnd(20, 300);
        const double phi = (k % 3 == 0) ? Rnd(-M_PI / 3, M_PI / 3) : 0.0;
        int32_t I = (int32_t)llround(R * cos(phi)), J = (int32_t)llround(R * sin(phi));
        if (k % 2) {
            I = -I;
            J = -J;
        }
        const bool cw = (rand() & 1) != 0;
        const int32_t sx = (int32_t)llround(Rnd(0, 2000) * spm), sy = (int32_t)llround(Rnd(0, 1500) * spm);
        const int32_t cx = sx + I * 80, cy = sy + J * 80;
        const int32_t ex = sx + 2 * I * 80, ey = sy + 2 * J * 80;
        const uint32_t vel = (uint32_t)(Rnd(60, 1500) / 60.0 * 800.0);
        const uint32_t acc = (uint32_t)(Rnd(0.5e6, 6e6) / 3600.0 * spm);
        const double Rs = sqrt((double)I * I + (double)J * J) * spm;
        /* y up: a clockwise half circle passes left of start->end (+), CCW right (-). */
        const double sign = cw ? 1.0 : -1.0;
        const Result r = RunArc(sx, sy, cx, cy, ex, ey, cw, vel, acc, sign);
        Check("half", k, r, Rs, Rs, sign, M_PI * Rs, vel);
        const int e = abs(r.endErrX) > abs(r.endErrY) ? abs(r.endErrX) : abs(r.endErrY);
        if (e > worstEnd) worstEnd = e;
        if (r.maxStep > worstStep) worstStep = r.maxStep;
        if (r.maxRadialErr > worstRad) worstRad = r.maxRadialErr;
    }

    /* Fixed quarter and three-quarter turns around (0, 0), r = 10000 steps, both directions. */
    {
        const int32_t r = 10000;
        const uint32_t vel = 20000, acc = 22222;
        struct { int32_t sx, sy, ex, ey; bool cw; double turns; } c[] = {
            {r, 0, 0, r, false, 0.25},  {r, 0, 0, -r, true, 0.25},
            {r, 0, 0, -r, false, 0.75}, {r, 0, 0, r, true, 0.75},
        };
        for (int k = 0; k < 4; k++) {
            const Result res = RunArc(c[k].sx, c[k].sy, 0, 0, c[k].ex, c[k].ey, c[k].cw, vel, acc, 0.0);
            Check("fixed", k, res, r, 0.0, 0.0, 2.0 * M_PI * r * c[k].turns, vel);
        }
    }

    printf("%d random half circles + 4 fixed arcs (seed %u)\n", count, seed);
    printf("worst end error %d steps, worst per-sample step %d, worst radial error %.2f steps\n", worstEnd,
           worstStep, worstRad);
    printf(g_fail ? "FAILED (%d)\n" : "PASS\n", g_fail);
    return g_fail ? 1 : 0;
}
