"""Net-zero returns on M0, with an MSP screenshot at each enabled settle and after disable.

Firmware position is generated steps. MSP Position (cnts) is read from the
ClearPath-MSP window afterward.
"""

import subprocess
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "ros2_ws" / "src" / "clearcore_bridge"))

from clearcore_bridge.wire import SessionClient

HOST = "172.16.82.113"
HERE = Path(__file__).resolve().parent
SHOT = HERE / "shot_msp.ps1"
CYCLES = 6
TARGET = 0.06


def shot(name: str) -> None:
    path = HERE / name
    subprocess.check_call(
        ["powershell", "-NoProfile", "-File", str(SHOT), str(path)],
        stdout=subprocess.DEVNULL,
    )
    print("shot", name, flush=True)


def wait_settled(session, target, timeout=8.0):
    deadline = time.time() + timeout
    last = None
    while time.time() < deadline:
        last = session.call("get_status")
        if last["fault"] or last["estop"] or last["watchdog"] or not last["enabled"]:
            return last
        if abs(last["position"][0] - target) <= 0.0005 and not last["moving"]:
            return last
        time.sleep(0.05)
    return last


def main():
    shot("msp_cycle_base.png")
    session = SessionClient(HOST, 9200, 3.0)
    try:
        session.call(
            "configure",
            {"axis_mask": 1, "vel_steps": 16000, "accel_steps": 120000, "decel_steps": 120000},
        )
        session.call("set_test_mode", {"on": True})
        session.call("enable")
        for cycle in range(1, CYCLES + 1):
            session.call("set_joints", {"x": TARGET})
            out = wait_settled(session, TARGET)
            session.call("set_joints", {"x": 0.0})
            home = wait_settled(session, 0.0)
            time.sleep(0.35)
            shot("msp_cycle_%02d_enabled.png" % cycle)
            print(
                "cycle %d commanded_x=%.5f moving=%s fault=%s"
                % (cycle, home["position"][0], home["moving"], home["fault"]),
                flush=True,
            )
            if abs(home["position"][0]) > 0.0005 or home["fault"] or home["moving"]:
                raise SystemExit("firmware did not settle at 0")
        time.sleep(0.2)
        shot("msp_before_disable.png")
        session.call("disable")
        time.sleep(0.6)
        shot("msp_after_disable.png")
        time.sleep(1.0)
        shot("msp_after_disable_1s.png")
        print("disabled", flush=True)
    finally:
        try:
            session.call("disable")
        except Exception:
            pass
        session.close()


if __name__ == "__main__":
    main()
