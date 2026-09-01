"""Quick verification of sub-step residual tracking: many relative moves whose
mm->step conversion does NOT divide evenly, then return to 0. Without the
residual fix the commanded pose drifts; with the fix it should return to ~0."""

from __future__ import annotations

import json
import sys

from clearai.client import ClearAiClient, discover

# 0.01 mm = 1.6 steps (160 steps/mm). Each move rounds; residual should cancel it.
N = 200
DX = 0.01


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

    p0 = c.call("get_pose")
    start_x = p0.get("x", 0.0)
    print(f"start pose.x = {start_x:+.6f} mm")

    net = 0.0
    for i in range(N):
        c.call("move_linear", {"x": DX, "feed": 2000})
        c.call("wait_idle", {"timeout_ms": 5000})
        net += DX
    print(f"after {N} x {DX}mm moves: net={net:+.4f}mm  (exact steps = {net*160:.1f})")
    p1 = c.call("get_pose")
    print(f"pose.x = {p1.get('x'):+.6f} mm  (delta = {p1.get('x')-start_x:+.6f} mm = {(p1.get('x')-start_x)*160:+.1f} counts)")

    # return to 0
    c.call("move_linear", {"x": -net, "feed": 2000})
    c.call("wait_idle", {"timeout_ms": 10000})
    p2 = c.call("get_pose")
    delta = p2.get("x") - start_x
    print(f"\nafter return to 0: pose.x = {p2.get('x'):+.6f} mm")
    print(f"final pose delta = {delta:+.6f} mm = {delta*160:+.1f} counts")
    print("expected ~0 counts with residual fix; large drift without.")

    c.call("stop")
    c.close()
    ok = abs(delta * 160) < 2.0
    print("\nPASS" if ok else "\nFAIL")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
