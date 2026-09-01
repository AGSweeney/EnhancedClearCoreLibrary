"""Test motion/diagnostics enhancements: jog_velocity/jog_stop, alert decoding,
motion log, uptime/counters/distance, batched moves, and move timing."""

from __future__ import annotations

import json
import sys
import time

from clearai.client import ClearAiClient, discover


def main() -> int:
    host = discover() or "172.16.82.113"
    print(f"Host: {host}")
    client = ClearAiClient()
    client.connect_tcp(host)

    errors: list[str] = []

    caps = client.call("get_capabilities")
    methods = caps.get("methods", [])
    print("methods:", methods)
    for need in ("jog_velocity", "jog_stop", "move_batch", "get_log", "clear_log"):
        if need not in methods:
            errors.append(f"missing method {need}")

    # ---- safe session setup ----
    client.call("disable")
    client.call("clear_alerts")
    client.call("configure", {"watchdog_ms": 0})
    client.call("set_mode", {"mode": "rel"})
    client.call("enable")
    client.call("keepalive")

    # ---- get_status new fields ----
    print("\n-- get_status (new fields) --")
    st = client.call("get_status")
    print(json.dumps({k: st.get(k) for k in
                      ("uptime_ms", "moves", "moves_rejected", "distance",
                       "arc_path", "alerts_decoded", "alert_reg_axis", "alert_reg")},
                     indent=2))
    for k in ("uptime_ms", "moves", "moves_rejected", "distance", "arc_path",
              "alerts_decoded", "alert_reg_axis"):
        if k not in st:
            errors.append(f"get_status missing {k}")
    if not isinstance(st.get("alerts_decoded"), list):
        errors.append(f"alerts_decoded not a list: {st.get('alerts_decoded')}")
    if not isinstance(st.get("alert_reg_axis"), list) or len(st.get("alert_reg_axis", [])) != 4:
        errors.append(f"alert_reg_axis not a 4-list: {st.get('alert_reg_axis')}")
    if not isinstance(st.get("distance"), dict):
        errors.append(f"distance not an object: {st.get('distance')}")

    moves_before = st.get("moves", 0)

    # ---- move timing: est_ms in move_linear, elapsed_ms in wait_idle ----
    print("\n-- move timing --")
    r = client.call("move_linear", {"x": 1, "feed": 500})
    print("move_linear:", json.dumps(r))
    if "est_ms" not in r:
        errors.append("move_linear reply missing est_ms")
    elif not isinstance(r.get("est_ms"), int) or r.get("est_ms") < 0:
        errors.append(f"est_ms invalid: {r.get('est_ms')}")
    w = client.call("wait_idle", {"timeout_ms": 15000})
    print("wait_idle:", json.dumps(w))
    if "elapsed_ms" not in w:
        errors.append("wait_idle reply missing elapsed_ms")

    st = client.call("get_status")
    if st.get("moves", 0) <= moves_before:
        errors.append(f"moves not incremented: {moves_before} -> {st.get('moves')}")
    if st.get("uptime_ms", 0) <= 0:
        errors.append("uptime_ms not positive")

    # ---- jog_velocity / jog_stop ----
    print("\n-- jog_velocity / jog_stop --")
    client.call("jog_velocity", {"x": 2})
    time.sleep(0.3)
    client.call("jog_stop")
    client.call("wait_idle", {"timeout_ms": 10000})
    st = client.call("get_status")
    print("moving after jog_stop:", st.get("moving"))
    if st.get("moving"):
        errors.append("still moving after jog_stop")

    # ---- move_batch (good) ----
    print("\n-- move_batch (good) --")
    r = client.call("move_batch", {"moves": [
        {"x": 1, "feed": 500},
        {"seconds": 0.2},
        {"x": -1, "feed": 500},
    ]})
    print("move_batch:", json.dumps(r))
    if not r.get("ok"):
        errors.append(f"move_batch ok=false: {r}")
    if r.get("accepted") != 3:
        errors.append(f"move_batch accepted != 3: {r.get('accepted')}")
    client.call("wait_idle", {"timeout_ms": 15000})

    # ---- move_batch (bad element) ----
    print("\n-- move_batch (bad element) --")
    try:
        r = client.call("move_batch", {"moves": [
            {"x": 1, "feed": 500},
            {},
        ]})
        print("move_batch bad:", json.dumps(r))
        if r.get("ok"):
            errors.append("bad move_batch accepted")
        if r.get("failed_at") != 1:
            errors.append(f"failed_at != 1: {r.get('failed_at')}")
        if r.get("accepted") != 1:
            errors.append(f"accepted != 1: {r.get('accepted')}")
    except Exception as exc:
        errors.append(f"bad move_batch raised instead of result: {exc}")
    client.call("wait_idle", {"timeout_ms": 10000})

    # ---- motion log ----
    print("\n-- motion log --")
    client.call("clear_log")
    log = client.call("get_log")
    print("log after clear:", json.dumps(log))
    if log != []:
        errors.append(f"get_log not empty after clear_log: {log}")

    # trigger a deterministic rejection: arc without i/j (or XY not configured)
    try:
        client.call("move_arc", {"x": 0, "y": 0})
        errors.append("move_arc without i/j accepted")
    except Exception as exc:
        print("move_arc rejected as expected:", exc)
    # also the bad move_batch above should have logged
    log = client.call("get_log")
    print("log after rejections:", json.dumps(log))
    found = any(isinstance(e, dict) and e.get("kind") == 0 for e in log)
    if not found:
        errors.append("no rejected entry in log after rejections")

    client.call("clear_log")
    log = client.call("get_log")
    if log != []:
        errors.append(f"get_log not empty after second clear_log: {log}")

    client.close()

    if errors:
        print("\nFAILED:")
        for err in errors:
            print(f"  - {err}")
        return 1
    print("\nPASSED")
    return 0


if __name__ == "__main__":
    sys.exit(main())
