"""Live bench test on M0 (X axis, bare motor, no mechanics). Small relative moves,
jog-velocity, batched moves; observe est_ms/elapsed_ms, counters, distance, log, alerts."""

from __future__ import annotations

import json
import sys
import time

from clearai.client import ClearAiClient, discover


def pose(c):
    return c.call("get_pose")


def main() -> int:
    host = discover() or "172.16.82.113"
    print(f"Host: {host}")
    c = ClearAiClient()
    c.connect_tcp(host)

    print("\n== setup ==")
    c.call("disable")
    c.call("clear_alerts")
    c.call("configure", {"watchdog_ms": 0})
    c.call("set_mode", {"mode": "rel"})
    c.call("set_units", {"units": "mm"})
    c.call("enable")
    c.call("keepalive")
    st = c.call("get_status")
    print("enabled:", st.get("enabled"), "hlfb:", st.get("hlfb"), "test_mode:", st.get("test_mode"))

    moves0 = st.get("moves", 0)
    p0 = pose(c)
    print("start pose:", json.dumps(p0), " moves:", moves0)

    print("\n== move_linear x=+1mm feed=2000 (expect est_ms) ==")
    r = c.call("move_linear", {"x": 1, "feed": 2000})
    print("reply:", json.dumps(r))
    w = c.call("wait_idle", {"timeout_ms": 10000})
    print("wait_idle:", json.dumps(w))
    p1 = pose(c)
    print("pose after +1mm:", json.dumps(p1))

    print("\n== move_linear x=-1mm feed=2000 (return) ==")
    r = c.call("move_linear", {"x": -1, "feed": 2000})
    print("reply:", json.dumps(r))
    c.call("wait_idle", {"timeout_ms": 10000})
    p2 = pose(c)
    print("pose after -1mm:", json.dumps(p2))

    print("\n== jog_velocity x=20 mm/s for 0.4s then jog_stop ==")
    c.call("jog_velocity", {"x": 20})
    time.sleep(0.4)
    c.call("jog_stop")
    w = c.call("wait_idle", {"timeout_ms": 10000})
    print("wait_idle:", json.dumps(w))
    p3 = pose(c)
    print("pose after jog:", json.dumps(p3))
    st = c.call("get_status")
    print("moving after stop:", st.get("moving"))

    print("\n== move_batch: +0.5mm, dwell 0.2s, -0.5mm ==")
    r = c.call("move_batch", {"moves": [
        {"x": 0.5, "feed": 2000},
        {"seconds": 0.2},
        {"x": -0.5, "feed": 2000},
    ]})
    print("move_batch:", json.dumps(r))
    c.call("wait_idle", {"timeout_ms": 10000})
    p4 = pose(c)
    print("pose after batch:", json.dumps(p4))

    print("\n== counters / distance / log / alerts ==")
    st = c.call("get_status")
    print(json.dumps({k: st.get(k) for k in
                      ("uptime_ms", "moves", "moves_rejected", "distance", "arc_path",
                       "alert_reg", "alerts_decoded", "alert_reg_axis", "hlfb", "moving")}, indent=2))
    print("moves delta:", st.get("moves", 0) - moves0)
    log = c.call("get_log")
    print("log:", json.dumps(log))

    c.call("stop")
    c.close()
    print("\nDONE")
    return 0


if __name__ == "__main__":
    sys.exit(main())
