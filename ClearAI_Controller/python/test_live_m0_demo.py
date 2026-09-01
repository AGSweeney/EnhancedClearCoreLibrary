"""Live M0 demo: longer, faster moves with dwells, a sustained jog, and a batch.
Bare motor on M0 (X axis, 160 steps/mm, 5mm/rev). 100mm == 20 revs."""

from __future__ import annotations

import json
import sys
import time

from clearai.client import ClearAiClient, discover


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

    def go(label, method, params, wait=True):
        print(f"\n[{label}] {method} {json.dumps(params)}")
        r = c.call(method, params)
        print("  reply:", json.dumps(r))
        if wait and method != "dwell":
            w = c.call("wait_idle", {"timeout_ms": 30000})
            print("  wait_idle:", json.dumps(w))

    # --- fast long back-and-forth with dwells ---
    go("fast +50mm", "move_linear", {"x": 50, "feed": 20000})
    go("dwell 0.5s", "dwell", {"seconds": 0.5}, wait=False)
    c.call("wait_idle", {"timeout_ms": 5000})
    go("fast -50mm", "move_linear", {"x": -50, "feed": 20000})
    go("dwell 0.5s", "dwell", {"seconds": 0.5}, wait=False)
    c.call("wait_idle", {"timeout_ms": 5000})

    # --- longer 100mm (20 revs) each way ---
    go("fast +100mm (20 rev)", "move_linear", {"x": 100, "feed": 20000})
    go("dwell 1.0s", "dwell", {"seconds": 1.0}, wait=False)
    c.call("wait_idle", {"timeout_ms": 5000})
    go("fast -100mm (20 rev)", "move_linear", {"x": -100, "feed": 20000})
    go("dwell 1.0s", "dwell", {"seconds": 1.0}, wait=False)
    c.call("wait_idle", {"timeout_ms": 5000})

    # --- sustained jog near max, then stop ---
    print("\n[jog_velocity x=150 mm/s for 1.5s]")
    c.call("jog_velocity", {"x": 150})
    time.sleep(1.5)
    c.call("jog_stop")
    w = c.call("wait_idle", {"timeout_ms": 15000})
    print("  jog wait_idle:", json.dumps(w))
    p = c.call("get_pose")
    print("  pose after jog:", json.dumps(p))

    go("dwell 0.5s", "dwell", {"seconds": 0.5}, wait=False)
    c.call("wait_idle", {"timeout_ms": 5000})

    # --- batched sequence: three moves + dwells, return to start ---
    print("\n[move_batch: +40, dwell 0.3, +40, dwell 0.3, -80]")
    r = c.call("move_batch", {"moves": [
        {"x": 40, "feed": 20000},
        {"seconds": 0.3},
        {"x": 40, "feed": 20000},
        {"seconds": 0.3},
        {"x": -80, "feed": 20000},
    ]})
    print("  move_batch:", json.dumps(r))
    c.call("wait_idle", {"timeout_ms": 30000})
    p = c.call("get_pose")
    print("  pose after batch:", json.dumps(p))

    # --- final summary ---
    st = c.call("get_status")
    print("\n== summary ==")
    print(json.dumps({k: st.get(k) for k in
                      ("moves", "moves_rejected", "distance", "arc_path",
                       "alert_reg", "alerts_decoded", "hlfb", "moving")}, indent=2))
    c.call("get_log")
    print("log:", json.dumps(c.call("get_log")))

    c.call("stop")
    c.close()
    print("\nDONE")
    return 0


if __name__ == "__main__":
    sys.exit(main())
