"""Repeat the first-motor 10 mm move on M0, then restore the saved setup.

The address comes from the command line or from CCROS_HOST. The move is 10 mm
only when X is linear, direction +1, gear 1, and offset 0. Test mode is turned
off for the move and written back afterward. axis_mask is set to 1 for the
move and written back afterward.

Pass --self-check to test the restore payload and the map gate with no board.
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
    args = [arg for arg in sys.argv[1:] if arg != "--self-check"]
    if args:
        return args[0]
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


def same_number(left, right, tol=1e-5) -> bool:
    if isinstance(left, (list, tuple)):
        return (
            isinstance(right, (list, tuple))
            and len(left) == len(right)
            and all(same_number(a, b, tol) for a, b in zip(left, right))
        )
    return abs(float(left) - float(right)) <= tol


def map_problems(cfg) -> list:
    """X must mean 10 mm. A rotary axis, or a non-default gear, makes 0.01 something else."""
    missing = [key for key in ("rotary", "direction", "gear", "offset") if key not in cfg]
    if missing:
        return ["get_config has no %s; flash ClearCoreROS 0.1.1" % ", ".join(missing)]
    problems = []
    if int(cfg["rotary"][0]) != 0:
        problems.append("rotary_x is %s, so 0.01 is radians" % cfg["rotary"][0])
    if int(cfg["direction"][0]) != 1:
        problems.append("direction_x is %s" % cfg["direction"][0])
    if not same_number(cfg["gear"][0], 1.0):
        problems.append("gear_x is %s" % cfg["gear"][0])
    if not same_number(cfg["offset"][0], 0.0):
        problems.append("offset_x is %s" % cfg["offset"][0])
    return problems


def print_map_help(host, problems) -> None:
    print("M0 is not the default linear map this 10 mm test requires:")
    for problem in problems:
        print(" ", problem)
    print("Disable the motors, then prepare X. Do not enable until configure succeeds:")
    print(
        "python host/ccros_cli.py --host %s configure "
        "--rotary-x 0 --direction-x 1 --gear-x 1 --offset-x 0" % host
    )
    print("Then rerun this script.")


def restore_params(saved) -> dict:
    """One configure object. Sending a limit value enables that side.

    Clear runs before the new value, so a disabled max of 0 must not be sent
    beside an enabled min of 0.01. That pair is valid while the max is off and
    rejected while both sides are on.
    """
    params = {
        "axis_mask": saved["axis_mask"],
        "steps_per_rev": saved["steps_per_rev"],
        "pitch_mm": saved["pitch_mm"],
    }
    flags = int(saved["limit_flags"])
    for index, axis in enumerate(AXES):
        if flags & (1 << (index * 2)):
            params["min_%s" % axis] = saved["limits_min"][index]
        else:
            params["clear_min_%s" % axis] = True
        if flags & (1 << (index * 2 + 1)):
            params["max_%s" % axis] = saved["limits_max"][index]
        else:
            params["clear_max_%s" % axis] = True
    return params


def restore_mismatches(saved, restored, status) -> list:
    bad = []
    if int(restored["axis_mask"]) != int(saved["axis_mask"]):
        bad.append("axis_mask")
    if not same_number(restored["steps_per_rev"], saved["steps_per_rev"], 0.0):
        bad.append("steps_per_rev")
    if not same_number(restored["pitch_mm"], saved["pitch_mm"]):
        bad.append("pitch_mm")
    if int(restored["limit_flags"]) != int(saved["limit_flags"]):
        bad.append("limit_flags")
    flags = int(saved["limit_flags"])
    for index, axis in enumerate(AXES):
        if flags & (1 << (index * 2)) and not same_number(
            restored["limits_min"][index], saved["limits_min"][index]
        ):
            bad.append("min_%s" % axis)
        if flags & (1 << (index * 2 + 1)) and not same_number(
            restored["limits_max"][index], saved["limits_max"][index]
        ):
            bad.append("max_%s" % axis)
    for key in ("names", "rotary", "direction"):
        if key in saved and [str(v) for v in restored[key]] != [str(v) for v in saved[key]]:
            bad.append(key)
    for key in ("gear", "offset"):
        if key in saved and not same_number(restored[key], saved[key]):
            bad.append(key)
    if bool(restored["test_mode"]) != bool(saved["test_mode"]):
        bad.append("test_mode")
    if status.get("enabled"):
        bad.append("enabled")
    return bad


def restore(session, saved):
    session.call("disable")
    session.call("set_test_mode", {"on": bool(saved["test_mode"])})
    session.call("configure", restore_params(saved))
    restored = session.call("get_config")
    status = session.call("get_status")
    bad = restore_mismatches(saved, restored, status)
    if bad:
        raise RuntimeError("restore mismatch: " + ", ".join(bad))
    return restored, status


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


def motion_ok(at_10, at_0, seen, final) -> bool:
    return (
        at_10 is not None
        and abs(at_10["position"][0] - 0.01) <= 0.0005
        and at_0 is not None
        and abs(at_0["position"][0]) <= 0.0005
        and seen is not None
        and abs(seen["position"][0]) <= 0.0005
        and final is not None
        and not final["enabled"]
        and not final["fault"]
        and not final["watchdog"]
    )


def main():
    host = host_from_args()
    session = SessionClient(host, 9200, 3.0)
    saved = None
    mutated = False
    test_error = None
    restore_error = None
    try:
        try:
            saved = session.call("get_config")
            show(
                "saved",
                {key: saved[key] for key in ("axis_mask", "test_mode", "names", "limit_flags", "ip_address")},
            )
            problems = map_problems(saved)
            if problems:
                print_map_help(host, problems)
                test_error = "joint map is not the default linear map"
            else:
                session.call("disable")
                mutated = True
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
                if not motion_ok(at_10, at_0, seen, final):
                    test_error = "motion did not match 10 mm"
                    print("==", test_error)
        except Exception as exc:
            test_error = str(exc)
            print("== test failed:", exc)
    finally:
        try:
            if mutated and saved is not None:
                try:
                    restored, status = restore(session, saved)
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
                    restore_error = str(exc)
                    print("== restore failed:", exc)
        finally:
            session.close()
    if test_error or restore_error:
        raise SystemExit(1)
    print("M0_OK")


def self_check() -> None:
    saved = {
        "axis_mask": 3,
        "steps_per_rev": [800, 800, 800, 800],
        "pitch_mm": [5.0, 5.0, 5.0, 5.0],
        "limit_flags": 1,
        "limits_min": [0.01, 0.0, 0.0, 0.0],
        "limits_max": [0.0, 0.0, 0.0, 0.0],
        "rotary": [0, 0, 0, 1],
        "direction": [1, 1, 1, 1],
        "gear": [1.0, 1.0, 1.0, 1.0],
        "offset": [0.0, 0.0, 0.0, 0.0],
        "names": ["joint_x", "joint_y", "joint_z", "joint_a"],
        "test_mode": False,
    }
    params = restore_params(saved)
    assert params["min_x"] == 0.01
    assert "max_x" not in params
    assert params["clear_max_x"] is True
    assert "clear_min_x" not in params
    assert params["clear_min_y"] is True and params["clear_max_y"] is True
    both = dict(saved)
    both["limit_flags"] = 3
    both["limits_max"] = [0.02, 0.0, 0.0, 0.0]
    both_params = restore_params(both)
    assert both_params["min_x"] == 0.01 and both_params["max_x"] == 0.02
    assert "clear_min_x" not in both_params and "clear_max_x" not in both_params
    none = dict(saved)
    none["limit_flags"] = 0
    none_params = restore_params(none)
    assert "min_x" not in none_params and "max_x" not in none_params
    assert none_params["clear_min_x"] is True and none_params["clear_max_x"] is True
    assert map_problems(saved) == []
    rotary = dict(saved)
    rotary["rotary"] = [1, 0, 0, 1]
    assert map_problems(rotary)
    geared = dict(saved)
    geared["gear"] = [2.0, 1.0, 1.0, 1.0]
    assert any("gear_x" in item for item in map_problems(geared))
    shifted = dict(saved)
    shifted["offset"] = [0.01, 0.0, 0.0, 0.0]
    assert any("offset_x" in item for item in map_problems(shifted))
    reversed_x = dict(saved)
    reversed_x["direction"] = [-1, 1, 1, 1]
    assert any("direction_x" in item for item in map_problems(reversed_x))
    status = {"enabled": False}
    assert restore_mismatches(saved, saved, status) == []
    changed = dict(saved)
    changed["axis_mask"] = 1
    assert restore_mismatches(saved, changed, status) == ["axis_mask"]
    assert motion_ok(
        {"position": [0.01, 0, 0, 0]},
        {"position": [0, 0, 0, 0]},
        {"position": [0, 0, 0, 0]},
        {"enabled": False, "fault": False, "watchdog": False},
    )
    print("self-check ok")


if __name__ == "__main__":
    if "--self-check" in sys.argv:
        self_check()
    else:
        main()
