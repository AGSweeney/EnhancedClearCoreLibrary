"""25-move calibration series for M0. All moves are whole-revolution multiples (5mm/rev).
Net commanded displacement = +50mm = +10 rev = +8000 counts (at 800 counts/rev).
User re-zeros the MSP count beforehand, then reports the final count."""

from __future__ import annotations

import json
import sys

from clearai.client import ClearAiClient, discover

MOVES = [
    +10, -5, +15, -10, +20,
    -15, +5, -20, +25, -10,
    +15, -5, +10, -20, +15,
    +5, -15, +20, -10, +5,
    +15, -10, +20, -15, +5,
]


def main() -> int:
    assert len(MOVES) == 25, len(MOVES)
    net = sum(MOVES)
    print(f"25 moves, net commanded = {net:+.1f} mm = {net/5.0:+.2f} rev = {net/5.0*800:+.0f} counts")

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
    print("start pose (your zero point):", json.dumps({k: p0.get(k) for k in ("x",)}))
    print(">>> Re-zero the MSP count now. Moves run immediately.\n")

    feed = 6000  # mm/min (100 mm/s)
    running = 0.0
    for i, dx in enumerate(MOVES, 1):
        r = c.call("move_linear", {"x": dx, "feed": feed})
        c.call("wait_idle", {"timeout_ms": 15000})
        running += dx
        print(f"move {i:2d}: x={dx:+4.1f}mm  running={running:+7.2f}mm = {running/5.0:+6.2f}rev  est_ms={r.get('est_ms')}")

    pf = c.call("get_pose")
    print("\n== final commanded pose ==")
    print(json.dumps({k: pf.get(k) for k in ("x",)}))
    expected_mm = pf.get("x", 0.0) - p0.get("x", 0.0)
    print(f"net from your zero: {expected_mm:+.3f} mm = {expected_mm/5.0:+.3f} rev = {expected_mm/5.0*800:+.0f} counts")
    print("Tell me the MSP 'Position (cnts)' reading.")

    c.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
