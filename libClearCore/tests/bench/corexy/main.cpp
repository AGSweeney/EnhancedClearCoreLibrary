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
 * CoreXY bench test firmware: ClearPath SD motors on M-0 (A) and M-1 (B), free shafts,
 * 800 steps/rev, no mechanics. Nothing moves until "run" arrives on USB serial.
 *
 *   en    enable both motors and leave them enabled
 *   run   enable both motors, run every test, print PASS/FAIL lines and the total steps sent
 *         to each motor (TOTAL A/B); the motors stay enabled
 *   id    enable, move M-0 +800 and M-1 +1600 on their own, disable (to tell the drives apart)
 *   dis   disable both motors
 *
 * Checks the board's commanded counts (A = X+Y, B = X-Y), that a 45-degree move turns
 * only one motor, motor speed against MotorVelMax(), SyncPositionFromMotors(), the
 * SetKinematics() refusals, and the controller's burst / timing / carry counters.
 * Peak speed is 10000 steps/s (750 rpm).
 */
#include "ClearCore.h"

#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define SerialPort ConnectorUsb

namespace {

MotorDriver &motorA = ConnectorM0;
MotorDriver &motorB = ConnectorM1;
CoordinatedMotionController cmc;

const uint32_t kPathVel = 9000;      // steps/s along the path
const uint32_t kMotorVel = 10000;    // per motor, steps/s (750 rpm at 800 steps/rev)
const uint32_t kAccel = 80000;       // path, steps/s^2

int g_fail = 0;
int g_pass = 0;
char g_line[200];

void Say(const char *fmt, ...) __attribute__((format(printf, 1, 2)));
void Say(const char *fmt, ...) {
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(g_line, sizeof(g_line), fmt, ap);
    va_end(ap);
    SerialPort.SendLine(g_line);
}

void Check(bool ok, const char *what, long got, long want) {
    if (ok) {
        g_pass++;
        Say("  ok   %s (%ld)", what, got);
    }
    else {
        g_fail++;
        Say("  FAIL %s: got %ld, want %ld", what, got, want);
    }
}

void CheckEq(const char *what, long got, long want) {
    Check(got == want, what, got, want);
}

bool Alerts() {
    return motorA.StatusReg().bit.AlertsPresent || motorB.StatusReg().bit.AlertsPresent ||
           motorA.StatusReg().bit.MotorInFault || motorB.StatusReg().bit.MotorInFault;
}

/* Motion statistics gathered while waiting: peak motor speed over a 20 ms window, and the
 * range of each motor's count (to show a motor stood still). */
struct Watch {
    static const int kWin = 20;   // 1 ms samples
    int32_t histA[kWin], histB[kWin];
    int n;
    uint32_t peakA, peakB;
    int32_t minA, maxA, minB, maxB;
    void Reset() {
        n = 0;
        peakA = peakB = 0;
        minA = maxA = motorA.PositionRefCommanded();
        minB = maxB = motorB.PositionRefCommanded();
    }
    void Sample() {
        const int32_t a = motorA.PositionRefCommanded();
        const int32_t b = motorB.PositionRefCommanded();
        if (a < minA) minA = a;
        if (a > maxA) maxA = a;
        if (b < minB) minB = b;
        if (b > maxB) maxB = b;
        const int i = n % kWin;
        if (n >= kWin) {
            const uint32_t va = (uint32_t)abs(a - histA[i]) * (1000 / kWin);
            const uint32_t vb = (uint32_t)abs(b - histB[i]) * (1000 / kWin);
            if (va > peakA) peakA = va;
            if (vb > peakB) peakB = vb;
        }
        histA[i] = a;
        histB[i] = b;
        n++;
    }
};
Watch g_watch;

/* Wait for the controller to finish, sampling once a millisecond. */
bool WaitIdle(uint32_t timeoutMs) {
    const uint32_t start = Milliseconds();
    uint32_t last = start;
    while (cmc.IsActive() || cmc.MotionQueueCount() > 0) {
        const uint32_t now = Milliseconds();
        if (now != last) {
            last = now;
            g_watch.Sample();
        }
        if (Alerts()) {
            Say("  FAIL motor alert while moving");
            g_fail++;
            cmc.Stop();
            return false;
        }
        if (now - start > timeoutMs) {
            Say("  FAIL timeout waiting for the move");
            g_fail++;
            cmc.Stop();
            return false;
        }
    }
    Delay_ms(20);   // let the last sample's steps go out
    return true;
}

bool WaitMotorSteps(uint32_t timeoutMs) {
    const uint32_t start = Milliseconds();
    while (!motorA.StepsComplete() || !motorB.StepsComplete()) {
        if (Milliseconds() - start > timeoutMs) {
            Say("  FAIL timeout waiting for a single-motor move");
            g_fail++;
            return false;
        }
    }
    Delay_ms(20);
    return true;
}

// Steps sent to each motor across PositionRefSet() resets, to compare with the drive's own count.
int32_t g_totalA = 0;
int32_t g_totalB = 0;

void Zero() {
    g_totalA += motorA.PositionRefCommanded();
    g_totalB += motorB.PositionRefCommanded();
    motorA.PositionRefSet(0);
    motorB.PositionRefSet(0);
    cmc.SetPosition(0, 0);
}

/* The motors must sit at A = X+Y, B = X-Y (CoreXY) for the controller's X/Y. */
void CheckMotorsMatch(const char *what) {
    int32_t a, b;
    Kinematics::ToMotor(cmc.Kinematics(), cmc.CurrentX(), cmc.CurrentY(), a, b);
    Say(" %s: X %ld Y %ld  A %ld B %ld", what, (long)cmc.CurrentX(), (long)cmc.CurrentY(),
        (long)motorA.PositionRefCommanded(), (long)motorB.PositionRefCommanded());
    CheckEq("motor A count", motorA.PositionRefCommanded(), a);
    CheckEq("motor B count", motorB.PositionRefCommanded(), b);
}

bool Linear(int32_t x, int32_t y) {
    g_watch.Reset();
    if (!cmc.QueueLinear(x, y)) {
        Say("  FAIL QueueLinear refused");
        g_fail++;
        return false;
    }
    return WaitIdle(30000);
}

void TestCartesian() {
    Say("== Cartesian square (regression)");
    Check(cmc.SetKinematics(KINEMATICS_CARTESIAN), "SetKinematics(CARTESIAN)", 1, 1);
    Zero();
    cmc.SetDeferMotionQueueStart(true);
    cmc.QueueLinear(8000, 0);
    cmc.QueueLinear(8000, 8000);
    cmc.QueueLinear(0, 8000);
    cmc.QueueLinear(2000, 3000);
    g_watch.Reset();
    cmc.StartDeferredMotionIfPending();
    cmc.SetDeferMotionQueueStart(false);
    WaitIdle(30000);
    CheckMotorsMatch("after square");
    CheckEq("A = X", motorA.PositionRefCommanded(), 2000);
    CheckEq("B = Y", motorB.PositionRefCommanded(), 3000);
}

void TestAxisAndDiagonal() {
    Say("== CoreXY axis and diagonal moves");
    Check(cmc.SetKinematics(KINEMATICS_COREXY), "SetKinematics(COREXY)", 1, 1);
    Zero();

    Linear(8000, 0);
    CheckMotorsMatch("+X 8000");
    CheckEq("+X: A", motorA.PositionRefCommanded(), 8000);
    CheckEq("+X: B", motorB.PositionRefCommanded(), 8000);
    Say("  peak A %lu B %lu steps/s", (unsigned long)g_watch.peakA, (unsigned long)g_watch.peakB);

    Linear(8000, 8000);
    CheckMotorsMatch("+Y 8000");
    CheckEq("+Y: A", motorA.PositionRefCommanded(), 16000);
    CheckEq("+Y: B", motorB.PositionRefCommanded(), 0);

    Linear(0, 0);   // 45 degrees: only A turns, at up to sqrt(2) x path speed
    CheckMotorsMatch("diagonal back");
    CheckEq("diagonal: B never moved (max-min)", g_watch.maxB - g_watch.minB, 0);
    Say("  diagonal peak A %lu steps/s (limit %lu, uncapped would be %lu)", (unsigned long)g_watch.peakA,
        (unsigned long)kMotorVel, (unsigned long)(kPathVel * 1.41421356));
    Check(g_watch.peakA <= kMotorVel * 103 / 100, "diagonal A within MotorVelMax", g_watch.peakA, kMotorVel);
    Check(g_watch.peakA >= kMotorVel * 90 / 100, "diagonal A reached the cap", g_watch.peakA, kMotorVel);
}

void TestArcs() {
    Say("== CoreXY full circle (two QueueArcTo halves)");
    Zero();
    cmc.SetDeferMotionQueueStart(true);
    cmc.QueueArcTo(6000, 0, 12000, 0, true);
    cmc.QueueArcTo(6000, 0, 0, 0, true);
    g_watch.Reset();
    cmc.StartDeferredMotionIfPending();
    cmc.SetDeferMotionQueueStart(false);
    WaitIdle(30000);
    CheckMotorsMatch("after circle");
    CheckEq("circle: A home", motorA.PositionRefCommanded(), 0);
    CheckEq("circle: B home", motorB.PositionRefCommanded(), 0);
    Say("  circle peak A %lu B %lu steps/s", (unsigned long)g_watch.peakA, (unsigned long)g_watch.peakB);
    Check(g_watch.peakA <= kMotorVel * 103 / 100 && g_watch.peakB <= kMotorVel * 103 / 100,
          "circle motors within MotorVelMax", g_watch.peakA > g_watch.peakB ? g_watch.peakA : g_watch.peakB,
          kMotorVel);
}

void TestMixedPath() {
    Say("== CoreXY mixed queued path (lines + arcs, dVmax corners)");
    Zero();
    cmc.JunctionDVmax(2000);
    cmc.SetDeferMotionQueueStart(true);
    const int32_t pts[][2] = {{6000, 0}, {6000, 3000}, {9000, 6000}, {3000, 6000}, {-2000, 1000}};
    for (unsigned i = 0; i < sizeof(pts) / sizeof(pts[0]); i++) {
        cmc.QueueLinear(pts[i][0], pts[i][1]);
    }
    cmc.QueueArcTo(-2000, -2000, -5000, -2000, false);   // r 3000 around (-2000,-2000), CCW
    cmc.QueueArcTo(-2000, -2000, -2000, -5000, false);
    cmc.QueueLinear(1000, -5000);
    cmc.QueueLinear(4000, 1000);
    g_watch.Reset();
    cmc.StartDeferredMotionIfPending();
    cmc.SetDeferMotionQueueStart(false);
    WaitIdle(60000);
    cmc.JunctionDVmax(0);
    CheckMotorsMatch("after mixed path");
    CheckEq("mixed: A = 4000+1000", motorA.PositionRefCommanded(), 5000);
    CheckEq("mixed: B = 4000-1000", motorB.PositionRefCommanded(), 3000);
    Say("  mixed peak A %lu B %lu steps/s", (unsigned long)g_watch.peakA, (unsigned long)g_watch.peakB);
    Check(g_watch.peakA <= kMotorVel * 103 / 100 && g_watch.peakB <= kMotorVel * 103 / 100,
          "mixed motors within MotorVelMax", g_watch.peakA > g_watch.peakB ? g_watch.peakA : g_watch.peakB,
          kMotorVel);
}

void TestSync() {
    Say("== SyncPositionFromMotors after single-motor moves");
    Zero();
    motorA.Move(1000);   // A and B together: +X only
    motorB.Move(1000);
    WaitMotorSteps(10000);
    Check(cmc.SyncPositionFromMotors(), "sync on grid returns true", 1, 1);
    CheckEq("sync: X", cmc.CurrentX(), 1000);
    CheckEq("sync: Y", cmc.CurrentY(), 0);
    motorA.Move(1000);   // A alone: +X/2 and +Y/2
    WaitMotorSteps(10000);
    cmc.SyncPositionFromMotors();
    CheckEq("A alone: X", cmc.CurrentX(), 1500);
    CheckEq("A alone: Y", cmc.CurrentY(), 500);
    motorA.Move(1);      // odd A+B: half a step off the grid
    WaitMotorSteps(5000);
    Check(!cmc.SyncPositionFromMotors(), "sync off grid returns false", 0, 0);
    CheckEq("off grid: X rounded down", cmc.CurrentX(), 1500);
    CheckEq("off grid: Y = A - X", cmc.CurrentY(), 501);
    motorA.Move(-1);     // back on the grid
    WaitMotorSteps(5000);
    Check(cmc.SyncPositionFromMotors(), "sync back on grid", 1, 1);
    Linear(0, 0);
    CheckMotorsMatch("coordinated move home after sync");
}

void TestRefusals() {
    Say("== SetKinematics refusals");
    Zero();
    g_watch.Reset();
    cmc.QueueLinear(4000, 0);
    Check(!cmc.SetKinematics(KINEMATICS_CARTESIAN), "refused while moving", 0, 0);
    WaitIdle(30000);
    CheckEq("still CoreXY", cmc.Kinematics(), KINEMATICS_COREXY);
    cmc.SetMechanicalParamsX(800, 40.0, UNIT_MM);
    cmc.SetMechanicalParamsY(800, 20.0, UNIT_MM);
    Check(!cmc.SetKinematics(KINEMATICS_COREXY), "refused with different X/Y steps per mm", 0, 0);
    cmc.SetMechanicalParamsY(800, 40.0, UNIT_MM);
    Check(cmc.SetKinematics(KINEMATICS_COREXY), "accepted with matching X/Y", 1, 1);
    Linear(0, 0);
    CheckMotorsMatch("home");
}

bool EnableMotors() {
    motorA.EnableRequest(true);
    motorB.EnableRequest(true);
    const uint32_t start = Milliseconds();
    while (motorA.HlfbState() != MotorDriver::HLFB_ASSERTED ||
           motorB.HlfbState() != MotorDriver::HLFB_ASSERTED) {
        if (Alerts()) {
            Say("FAIL motor alert while enabling (A alerts 0x%lx, B 0x%lx)",
                (unsigned long)motorA.AlertReg().reg, (unsigned long)motorB.AlertReg().reg);
            return false;
        }
        if (Milliseconds() - start > 5000) {
            Say("FAIL HLFB did not assert within 5 s (A %d, B %d)", (int)motorA.HlfbState(),
                (int)motorB.HlfbState());
            return false;
        }
    }
    return true;
}

void RunAll() {
    g_fail = 0;
    g_pass = 0;
    Say("CoreXY bench test: path %lu, motor %lu steps/s, accel %lu", (unsigned long)kPathVel,
        (unsigned long)kMotorVel, (unsigned long)kAccel);
    if (!EnableMotors()) {
        motorA.EnableRequest(false);
        motorB.EnableRequest(false);
        Say("RESULT FAIL (enable)");
        return;
    }
    Say("motors enabled");
    g_totalA = -motorA.PositionRefCommanded();
    g_totalB = -motorB.PositionRefCommanded();
    const uint32_t excessA0 = motorA.CoordinatedExcessSteps();
    const uint32_t excessB0 = motorB.CoordinatedExcessSteps();
    const uint32_t burst0 = cmc.BurstCount();
    cmc.ResetTimingProbe();

    TestCartesian();
    TestAxisAndDiagonal();
    TestArcs();
    TestMixedPath();
    TestSync();
    TestRefusals();

    Say("== Counters");
    Say("  bursts %lu, carried excess A %lu B %lu, late updates %lu, slow updates %lu, max update %lu cycles",
        (unsigned long)(cmc.BurstCount() - burst0),
        (unsigned long)(motorA.CoordinatedExcessSteps() - excessA0),
        (unsigned long)(motorB.CoordinatedExcessSteps() - excessB0), (unsigned long)cmc.LateUpdates(),
        (unsigned long)cmc.SlowUpdates(), (unsigned long)cmc.MaxUpdateCycles());
    CheckEq("no step bursts", cmc.BurstCount() - burst0, 0);
    CheckEq("no slow updates (>150 us)", cmc.SlowUpdates(), 0);
    Check(!Alerts(), "no motor alerts", 0, 0);

    // Motors stay enabled (send "dis"), so the drives' counts can be read without the
    // re-sync a ClearPath does on enable.
    Say("TOTAL A %ld B %ld", (long)(g_totalA + motorA.PositionRefCommanded()),
        (long)(g_totalB + motorB.PositionRefCommanded()));
    Say("RESULT %s (%d passed, %d failed)", g_fail ? "FAIL" : "PASS", g_pass, g_fail);
}

} // namespace

int main() {
    MotorMgr.MotorInputClocking(MotorManager::CLOCK_RATE_NORMAL);
    MotorMgr.MotorModeSet(MotorManager::MOTOR_ALL, Connector::CPM_MODE_STEP_AND_DIR);
    motorA.HlfbMode(MotorDriver::HLFB_MODE_HAS_BIPOLAR_PWM);
    motorB.HlfbMode(MotorDriver::HLFB_MODE_HAS_BIPOLAR_PWM);
    motorA.HlfbCarrier(MotorDriver::HLFB_CARRIER_482_HZ);
    motorB.HlfbCarrier(MotorDriver::HLFB_CARRIER_482_HZ);
    motorA.VelMax(5000);   // single-motor moves (SyncPositionFromMotors test)
    motorB.VelMax(5000);
    motorA.AccelMax(50000);
    motorB.AccelMax(50000);

    SerialPort.Mode(Connector::USB_CDC);
    SerialPort.Speed(115200);
    SerialPort.PortOpen();

    cmc.Initialize(&motorA, &motorB);
    cmc.ArcVelMax(kPathVel);
    cmc.ArcAccelMax(kAccel);
    cmc.MotorVelMax(kMotorVel);

    char cmd[40];
    uint8_t len = 0;
    uint32_t lastHello = 0;
    while (true) {
        if (Milliseconds() - lastHello > 3000) {
            lastHello = Milliseconds();
            SerialPort.SendLine("corexy-bench ready: send 'run' or 'dis'");
        }
        while (SerialPort.AvailableForRead() > 0) {
            const int c = SerialPort.CharGet();
            if (c == '\r' || c == '\n') {
                cmd[len] = 0;
                if (strcmp(cmd, "run") == 0) {
                    RunAll();
                }
                else if (strncmp(cmd, "lin ", 4) == 0 || strncmp(cmd, "mv ", 3) == 0 ||
                         strncmp(cmd, "circ ", 5) == 0 ||
                         strcmp(cmd, "k c") == 0 || strcmp(cmd, "k x") == 0 || strcmp(cmd, "z") == 0) {
                    // Exploration: "k c|x" kinematics, "lin X Y" one coordinated line,
                    // "mv A B" single-motor moves, "z" zero; each prints step totals.
                    g_totalA = -motorA.PositionRefCommanded();
                    g_totalB = -motorB.PositionRefCommanded();
                    const uint32_t rev0 = motorA.ReversalWaits() + motorB.ReversalWaits();
                    long p = 0, q = 0;
                    if (cmd[0] == 'k') {
                        cmc.SetKinematics(cmd[2] == 'x' ? KINEMATICS_COREXY : KINEMATICS_CARTESIAN);
                    }
                    else if (cmd[0] == 'z') {
                        Zero();
                    }
                    else if (cmd[0] == 'l' && sscanf(cmd + 4, "%ld %ld", &p, &q) == 2) {
                        Linear((int32_t)p, (int32_t)q);
                    }
                    else if (cmd[0] == 'c' && sscanf(cmd + 5, "%ld %ld", &p, &q) >= 1) {
                        // "circ DEG [halves]": circle r 6000 from the current point, the start
                        // point at DEG degrees round the centre, clockwise, as two QueueArcTo
                        // halves (halves 1 = only the first half).
                        const double phi = (double)p * M_PI / 180.0;
                        const int32_t x0 = cmc.CurrentX(), y0 = cmc.CurrentY();
                        const int32_t cx = x0 - (int32_t)lround(6000.0 * cos(phi));
                        const int32_t cy = y0 - (int32_t)lround(6000.0 * sin(phi));
                        cmc.SetDeferMotionQueueStart(true);
                        cmc.QueueArcTo(cx, cy, 2 * cx - x0, 2 * cy - y0, true);
                        if (q != 1) {
                            cmc.QueueArcTo(cx, cy, x0, y0, true);
                        }
                        g_watch.Reset();
                        cmc.StartDeferredMotionIfPending();
                        cmc.SetDeferMotionQueueStart(false);
                        WaitIdle(60000);
                    }
                    else if (cmd[0] == 'm' && sscanf(cmd + 3, "%ld %ld", &p, &q) == 2) {
                        motorA.Move((int32_t)p);
                        motorB.Move((int32_t)q);
                        WaitMotorSteps(20000);
                    }
                    Say("TOTAL A %ld B %ld", (long)(g_totalA + motorA.PositionRefCommanded()),
                        (long)(g_totalB + motorB.PositionRefCommanded()));
                    Say("RESULT done (X %ld Y %ld, reversal waits %lu)", (long)cmc.CurrentX(),
                        (long)cmc.CurrentY(),
                        (unsigned long)(motorA.ReversalWaits() + motorB.ReversalWaits() - rev0));
                }
                else if (cmd[0] == 't' && cmd[1] >= '1' && cmd[1] <= '6' && cmd[2] == 0) {
                    // One test on its own (send "en" first), with its own step totals.
                    void (*const tests[])() = {TestCartesian, TestAxisAndDiagonal, TestArcs,
                                               TestMixedPath, TestSync, TestRefusals};
                    g_fail = 0;
                    g_pass = 0;
                    g_totalA = -motorA.PositionRefCommanded();
                    g_totalB = -motorB.PositionRefCommanded();
                    const uint32_t revA0 = motorA.ReversalWaits(), revB0 = motorB.ReversalWaits();
                    tests[cmd[1] - '1']();
                    Say("TOTAL A %ld B %ld", (long)(g_totalA + motorA.PositionRefCommanded()),
                        (long)(g_totalB + motorB.PositionRefCommanded()));
                    Say("RESULT %s (%d passed, %d failed; reversal waits A %lu B %lu)",
                        g_fail ? "FAIL" : "PASS", g_pass, g_fail, (unsigned long)(motorA.ReversalWaits() - revA0),
                        (unsigned long)(motorB.ReversalWaits() - revB0));
                }
                else if (strcmp(cmd, "id") == 0) {
                    // Identify the drives: M-0 one rev (+800), M-1 two revs (+1600).
                    if (EnableMotors()) {
                        motorA.Move(800);
                        motorB.Move(1600);
                        WaitMotorSteps(10000);
                        Say("id done: M-0 +800, M-1 +1600");
                    }
                    motorA.EnableRequest(false);
                    motorB.EnableRequest(false);
                }
                else if (strcmp(cmd, "en") == 0) {
                    if (EnableMotors()) {
                        Say("enabled");
                    }
                }
                else if (strcmp(cmd, "dis") == 0) {
                    cmc.Stop();
                    motorA.EnableRequest(false);
                    motorB.EnableRequest(false);
                    SerialPort.SendLine("motors disabled");
                }
                len = 0;
                lastHello = Milliseconds();
            }
            else if (c >= 0 && len < sizeof(cmd) - 1) {
                cmd[len++] = (char)c;
            }
        }
    }
}
