"""Test A-axis units, HLFB torque readback, and queue introspection/flush."""

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
    for need in ("set_units_a", "queue_status", "queue_clear"):
        if need not in methods:
            errors.append(f"missing method {need}")

    print("\n-- get_config (initial) --")
    cfg = client.call("get_config")
    print(json.dumps({k: cfg[k] for k in ("units", "units_a", "mode") if k in cfg}, indent=2))
    if cfg.get("units_a") not in ("deg", "rev"):
        errors.append(f"units_a missing/invalid: {cfg.get('units_a')}")

    print("\n-- set_units_a rev --")
    client.call("set_units_a", {"units": "rev"})
    cfg = client.call("get_config")
    print("units_a =", cfg.get("units_a"))
    if cfg.get("units_a") != "rev":
        errors.append(f"set_units_a rev failed: {cfg.get('units_a')}")
    pose = client.call("get_pose")
    print("pose:", json.dumps(pose, indent=2))
    if "units_a" not in pose:
        errors.append("get_pose missing units_a")

    print("\n-- set_units_a deg --")
    client.call("set_units_a", {"units": "deg"})
    cfg = client.call("get_config")
    print("units_a =", cfg.get("units_a"))
    if cfg.get("units_a") != "deg":
        errors.append(f"set_units_a deg failed: {cfg.get('units_a')}")

    print("\n-- bad units_a --")
    try:
        client.call("set_units_a", {"units": "radians"})
        errors.append("set_units_a radians should have been rejected")
    except Exception as exc:
        print("rejected as expected:", exc)

    print("\n-- get_status (hlfb_percent / queue_active / units_a) --")
    st = client.call("get_status")
    print(json.dumps({k: st.get(k) for k in
                     ("units", "units_a", "queue", "queue_active", "hlfb_percent", "a")}, indent=2))
    if "hlfb_percent" not in st:
        errors.append("get_status missing hlfb_percent")
    else:
        hp = st["hlfb_percent"]
        if not isinstance(hp, list) or len(hp) != 4:
            errors.append(f"hlfb_percent not 4-element array: {hp}")

    print("\n-- queue_status --")
    qs = client.call("queue_status")
    print(json.dumps(qs, indent=2))
    if "queue" not in qs or "active" not in qs:
        errors.append(f"queue_status missing fields: {qs}")

    print("\n-- queue_clear --")
    qc = client.call("queue_clear")
    print(json.dumps(qc, indent=2))
    qs = client.call("queue_status")
    print("after clear:", json.dumps(qs, indent=2))
    if qs.get("queue") != 0:
        errors.append(f"queue not empty after clear: {qs}")
    if qs.get("active"):
        errors.append("queue still active after clear")

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
