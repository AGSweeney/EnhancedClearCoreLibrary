"""One-shot bench: M0 only, after flashing ClearCoreROS."""

import json
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "ros2_ws" / "src" / "clearcore_bridge"))

from clearcore_bridge.wire import SessionClient, StreamClient

HOST = "172.16.82.113"


def show(title, obj):
    print("==", title)
    if isinstance(obj, str):
        print(obj)
    else:
        print(json.dumps(obj, indent=2))


def wait_x(session, target, timeout=8.0):
    last = None
    deadline = time.time() + timeout
    while time.time() < deadline:
        last = session.call("get_status")
        pos = last["position"][0]
        print(
            "  x=%.5f m moving=%s enabled=%s fault=%s estop=%s wd=%s"
            % (pos, last["moving"], last["enabled"], last["fault"], last["estop"], last["watchdog"])
        )
        if abs(pos - target) <= 0.0005 and not last["moving"]:
            return last
        time.sleep(0.15)
    return last


def main():
    session = SessionClient(HOST, 9200, 3.0)
    try:
        show("capabilities", session.call("get_capabilities"))
        show("status before", session.call("get_status"))
        show(
            "configure M0",
            session.call(
                "configure",
                {"axis_mask": 1, "vel_steps": 8000, "accel_steps": 80000, "decel_steps": 80000},
            ),
        )
        try:
            show("enable without test mode", session.call("enable"))
            hlfb = "passed"
        except Exception as exc:
            hlfb = str(exc)
            print("== enable without test mode:", hlfb)
        show("test mode", session.call("set_test_mode", {"on": True}))
        show("enable", session.call("enable"))
        show("status enabled", session.call("get_status"))

        show("move to 10 mm", session.call("set_joints", {"x": 0.01}))
        at_10 = wait_x(session, 0.01)
        show("status at 10 mm", at_10)

        show("return to 0", session.call("set_joints", {"x": 0.0}))
        at_0 = wait_x(session, 0.0)
        show("status home", at_0)

        stream = StreamClient(HOST, 9201, 3.0)
        try:
            stream.send_position(0x01, (0.005, 0.0, 0.0, 0.0))
            seen = None
            deadline = time.time() + 8.0
            while time.time() < deadline:
                stream.send_heartbeat()
                for frame in stream.read(0.2):
                    if frame["type"] != "state":
                        continue
                    seen = frame
                    print(
                        "  stream x=%.5f moving=%s enabled=%s fault=%s wd=%s"
                        % (
                            frame["position"][0],
                            frame["moving"],
                            frame["enabled"],
                            frame["fault"],
                            frame["watchdog"],
                        )
                    )
                if seen and abs(seen["position"][0] - 0.005) <= 0.0005 and not seen["moving"]:
                    break
            if seen is None:
                show("stream frame", "no state")
            else:
                show(
                    "stream at 5 mm",
                    {k: seen[k] for k in ("position", "enabled", "moving", "fault", "estop", "watchdog")},
                )
            stream.send_position(0x01, (0.0, 0.0, 0.0, 0.0))
            deadline = time.time() + 8.0
            while time.time() < deadline:
                stream.send_heartbeat()
                for frame in stream.read(0.2):
                    if frame["type"] == "state":
                        seen = frame
                if seen and abs(seen["position"][0]) <= 0.0005 and not seen["moving"]:
                    break
            print("== stream returned x=%.5f" % (seen["position"][0] if seen else 999))
        finally:
            stream.close()

        show("disable", session.call("disable"))
        show("final", session.call("get_status"))
        print("HLFB_ENABLE:", hlfb)
        ok = (
            at_10 is not None
            and abs(at_10["position"][0] - 0.01) <= 0.0005
            and at_0 is not None
            and abs(at_0["position"][0]) <= 0.0005
            and seen is not None
            and abs(seen["position"][0]) <= 0.0005
            and not at_10["fault"]
            and not at_0["fault"]
        )
        if not ok:
            raise SystemExit(1)
        print("M0_OK")
    finally:
        try:
            session.call("disable")
        except Exception:
            pass
        session.close()


if __name__ == "__main__":
    main()
