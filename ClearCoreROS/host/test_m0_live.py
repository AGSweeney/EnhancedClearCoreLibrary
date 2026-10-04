"""Repeat the first-motor 10 mm move on M0, then restore the saved setup.

The address comes from the command line or from CCROS_HOST. Test mode is
forced off. axis_mask is set to 1 for the move and written back afterward.
"""

import json
import os
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "ros2_ws" / "src" / "clearcore_bridge"))

from clearcore_bridge.wire import SessionClient, StreamClient

AXES = ("x", "y", "z", "a")


def host_from_args() -> str:
    if len(sys.argv) > 1:
        return sys.argv[1]
    env = os.environ.get("CCROS_HOST", "").strip()
    if env:
        return env
    raise SystemExit("pass the board address or set CCROS_HOST")


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
        err = abs(pos - target)
        print(
            "pos=%s err=%.5f moving=%s watchdog=%s"
            % (["%.4f" % p for p in last["position"]], err, last["moving"], last["watchdog"])
        )
        if (
            err <= 0.0005
            and not last["moving"]
            and last["enabled"]
            and not last["fault"]
            and not last["estop"]
            and not last["watchdog"]
        ):
            return last
        time.sleep(0.1)
    return last


def restore(session, saved):
    session.call("disable")
    session.call("set_test_mode", {"on": False})
    session.call(
        "configure",
        {
            "axis_mask": saved["axis_mask"],
            "steps_per_rev": saved["steps_per_rev"],
            "pitch_mm": saved["pitch_mm"],
        },
    )
    limits = {}
    for index, axis in enumerate(AXES):
        limits["min_%s" % axis] = saved["limits_min"][index]
        limits["max_%s" % axis] = saved["limits_max"][index]
    session.call("configure", limits)
    flags = int(saved["limit_flags"])
    clear = {}
    for index, axis in enumerate(AXES):
        if (flags & (1 << (index * 2))) == 0:
            clear["clear_min_%s" % axis] = True
        if (flags & (1 << (index * 2 + 1))) == 0:
            clear["clear_max_%s" % axis] = True
    if clear:
        session.call("configure", clear)


def main():
    host = host_from_args()
    session = SessionClient(host, 9200, 3.0)
    saved = None
    try:
        saved = session.call("get_config")
        show("saved", {k: saved[k] for k in ("axis_mask", "test_mode", "names", "limit_flags", "ip_address")})
        session.call("disable")
        session.call("set_test_mode", {"on": False})
        session.call("clear_alerts")
        show(
            "configure M0",
            session.call(
                "configure",
                {
                    "axis_mask": 1,
                    "steps_per_rev": 800,
                    "pitch_mm": 5,
                    "min_x": 0,
                    "max_x": 0.02,
                },
            ),
        )
        show("enable", session.call("enable"))
        show("move to 10 mm", session.call("set_joints", {"x": 0.01}))
        at_10 = wait_x(session, 0.01)
        show("status at 10 mm", at_10)
        show("return to 0", session.call("set_joints", {"x": 0.0}))
        at_0 = wait_x(session, 0.0)
        show("status home", at_0)

        stream = StreamClient(host, 9201, 3.0)
        seen = None
        try:
            stream.send_position(0x01, (0.005, 0.0, 0.0, 0.0))
            deadline = time.time() + 8.0
            while time.time() < deadline:
                stream.send_heartbeat()
                for frame in stream.read(0.2):
                    if frame["type"] != "state":
                        continue
                    seen = frame
                    print(
                        "  stream x=%.4f moving=%s enabled=%s"
                        % (frame["position"][0], frame["moving"], frame["enabled"])
                    )
                if seen and abs(seen["position"][0] - 0.005) <= 0.0005 and not seen["moving"]:
                    break
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

        session.call("disable")
        final = session.call("get_status")
        show("final", final)
        ok = (
            at_10 is not None
            and abs(at_10["position"][0] - 0.01) <= 0.0005
            and at_0 is not None
            and abs(at_0["position"][0]) <= 0.0005
            and seen is not None
            and abs(seen["position"][0]) <= 0.0005
            and not final["enabled"]
            and not final["fault"]
            and not final["watchdog"]
        )
        if not ok:
            raise SystemExit(1)
        print("M0_OK")
    finally:
        if saved is not None:
            try:
                restore(session, saved)
                restored = session.call("get_config")
                status = session.call("get_status")
                print(
                    "== restored mask=%s test=%s enabled=%s flags=%s pos=%s"
                    % (
                        restored["axis_mask"],
                        restored["test_mode"],
                        status["enabled"],
                        restored["limit_flags"],
                        ["%.4f" % p for p in status["position"]],
                    )
                )
            except Exception as exc:
                print("== restore failed:", exc)
        session.close()


if __name__ == "__main__":
    main()
