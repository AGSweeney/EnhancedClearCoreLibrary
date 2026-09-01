"""Calibration move series for M0. Each move is a whole number of revolutions
(5mm/rev, 800 counts/rev). User zeros the MSP position count beforehand, then
reports the final count to compare against the commanded pose."""

from __future__ import annotations

import json
import sys

from clearai.client import ClearAiClient, discover

# 5mm per rev, 800 counts/rev -> 1 rev = 5mm = 800 counts
MOVES = [
    (+10.0, "2 rev = 1600 counts"),
    (+20.0, "4 rev = 3200 counts  (running +6 rev)"),
    (-15.0, "-3 rev = -2400 counts (running +3 rev)"),
    (+5.0,  "1 rev = +800 counts  (running +4 rev)"),
]


def main() -> int:
    host = discover() or "172.16.82.113"
    print(f"Host: {host}")
    c = ClearAiClient()
    c.connect_tcp(host)

    c.call("disable")
    c.call("clear_alerts")
    c.call("configure", {"watchdog_ms": 0})
    c.call("set_mode", {"mode": "rel"})
    c.call("set_units", {"units": "mm"})
    c.call("enable")
    c.call("keepalive")
    st = c.call("get_status")
    print("enabled:", st.get("enabled"), "hlfb:", st.get("hlfb"))

    p0 = c.call("get_pose")
    print("\nstart pose (should be your zero):", json.dumps(p0))
    print(">>> Zero the MSP count now, then press Enter here is not needed - moves run immediately.\n")

    feed = 6000  # mm/min (100 mm/s) - moderate, visible
    for i, (dx, note) in enumerate(MOVES, 1):
        r = c.call("move_linear", {"x": dx, "feed": feed})
        w = c.call("wait_idle", {"timeout_ms": 15000})
        p = c.call("get_pose")
        print(f"move {i}: x={dx:+.1f}mm  | {note}")
        print(f"         reply est_ms={r.get('est_ms')}  elapsed_ms={w.get('elapsed_ms')}  pose.x={p.get('x'):+.4f}mm")

    pf = c.call("get_pose")
    print("\n== final commanded pose ==")
    print(json.dumps(pf))
    expected_mm = pf.get("x", 0.0)
    expected_rev = expected_mm / 5.0
    expected_counts = expected_rev * 800.0
    print(f"commanded: {expected_mm:+.3f} mm = {expected_rev:+.3f} rev = {expected_counts:+.0f} counts (at 800/rev)")
    print("Tell me the MSP 'Position (cnts)' reading and we'll compare.")

    c.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
