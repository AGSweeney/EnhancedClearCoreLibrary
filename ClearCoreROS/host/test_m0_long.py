"""Longer M0 bench: repeated travels, dwells, then back to zero."""

import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "ros2_ws" / "src" / "clearcore_bridge"))

from clearcore_bridge.wire import SessionClient

HOST = "172.16.82.113"
# About two minutes at 0.1 m/s with a short dwell on each stop.
PATTERN_M = (0.05, 0.12, 0.02, -0.04, 0.08, -0.06, 0.10, 0.0)
CYCLES = 6
TOL_M = 0.0005
DWELL_S = 0.4


def wait_x(session, target, timeout):
    deadline = time.time() + timeout
    last = None
    while time.time() < deadline:
        last = session.call("get_status")
        if last["fault"] or last["estop"] or last["watchdog"] or not last["enabled"]:
            return last
        if abs(last["position"][0] - target) <= TOL_M and not last["moving"]:
            return last
        time.sleep(0.1)
    return last


def main():
    session = SessionClient(HOST, 9200, 3.0)
    errors = []
    moves = 0
    t0 = time.time()
    try:
        session.call(
            "configure",
            {"axis_mask": 1, "vel_steps": 16000, "accel_steps": 120000, "decel_steps": 120000},
        )
        session.call("set_test_mode", {"on": True})
        session.call("enable")
        print("enabled mask=1 vel=16000 steps/s  pattern=%d cycles" % CYCLES)
        for cycle in range(CYCLES):
            for target in PATTERN_M:
                moves += 1
                session.call("set_joints", {"x": target})
                # 0.12 m at 0.1 m/s plus accel is well under 8 s.
                status = wait_x(session, target, 8.0)
                if status is None:
                    raise SystemExit("no status")
                err = status["position"][0] - target
                errors.append(abs(err))
                print(
                    "c%02d mv%02d target=%+.3f x=%+.5f err=%+.5f moving=%s fault=%s wd=%s"
                    % (
                        cycle + 1,
                        moves,
                        target,
                        status["position"][0],
                        err,
                        status["moving"],
                        status["fault"],
                        status["watchdog"],
                    )
                )
                if status["fault"] or status["estop"] or status["watchdog"] or not status["enabled"]:
                    raise SystemExit("stopped on fault")
                if abs(err) > TOL_M or status["moving"]:
                    raise SystemExit("missed target")
                dwell_until = time.time() + DWELL_S
                while time.time() < dwell_until:
                    session.call("get_status")
                    time.sleep(0.1)
        final = session.call("get_status")
        print(
            "done moves=%d elapsed=%.1fs max|err|=%.5f m final_x=%.5f alerts=%s"
            % (moves, time.time() - t0, max(errors), final["position"][0], final["alerts"])
        )
    finally:
        try:
            session.call("stop")
            session.call("set_joints", {"x": 0.0})
            home = wait_x(session, 0.0, 8.0)
            if home:
                print("parked x=%.5f moving=%s" % (home["position"][0], home["moving"]))
            session.call("disable")
        except Exception as exc:
            print("cleanup:", exc)
        session.close()


if __name__ == "__main__":
    main()
