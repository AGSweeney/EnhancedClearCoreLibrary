"""Test Safety / limits features: per-axis dynamics, machine-coord soft limits,
limit-triggered fault latch, and host watchdog / keepalive."""

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
    for need in ("keepalive",):
        if need not in methods:
            errors.append(f"missing method {need}")

    # ---- get_config should report the new fields ----
    print("\n-- get_config (new fields) --")
    cfg = client.call("get_config")
    print(json.dumps({k: cfg.get(k) for k in
                     ("vel_axis", "accel_axis", "decel_axis", "watchdog_ms", "nvm_version")}, indent=2))
    for k in ("vel_axis", "accel_axis", "decel_axis", "watchdog_ms"):
        if k not in cfg:
            errors.append(f"get_config missing {k}")
    if cfg.get("nvm_version") not in (4, 5, 6, 7):
        errors.append(f"nvm_version unexpected: {cfg.get('nvm_version')}")

    # ---- get_status should report watchdog + limit_status ----
    print("\n-- get_status (watchdog / limit_status) --")
    st = client.call("get_status")
    print(json.dumps({k: st.get(k) for k in
                     ("watchdog_ms", "watchdog_tripped", "limit_status")}, indent=2))
    for k in ("watchdog_ms", "watchdog_tripped", "limit_status"):
        if k not in st:
            errors.append(f"get_status missing {k}")
    if not isinstance(st.get("limit_status"), dict):
        errors.append(f"limit_status not an object: {st.get('limit_status')}")

    # ---- per-axis dynamics: require disable (same as global vel/accel/decel) ----
    print("\n-- configure per-axis vel_z=2000 (requires disable) --")
    client.call("disable")
    r = client.call("configure", {"vel_z": 2000})
    print(json.dumps(r, indent=2))
    cfg = client.call("get_config")
    print("vel_axis =", cfg.get("vel_axis"), "nvm_version =", cfg.get("nvm_version"))
    if cfg.get("vel_axis", [None])[2] != 2000:
        errors.append(f"vel_z not applied: {cfg.get('vel_axis')}")
    if cfg.get("nvm_version") != 7:
        errors.append(f"nvm_version expected 7 after save, got {cfg.get('nvm_version')}")

    # clear it back to inherit
    client.call("configure", {"vel_z": 0})
    cfg = client.call("get_config")
    if cfg.get("vel_axis", [None])[2] != 0:
        errors.append(f"vel_z not cleared: {cfg.get('vel_axis')}")

    # ---- watchdog: set short timeout, fail to keepalive, expect trip ----
    print("\n-- watchdog: set 800ms, stop keepalive, expect trip --")
    client.call("enable")
    client.call("configure", {"watchdog_ms": 800})
    client.call("keepalive")
    st = client.call("get_status")
    print("right after arm:", st.get("watchdog_tripped"), "watchdog_ms=", st.get("watchdog_ms"))
    if st.get("watchdog_tripped"):
        errors.append("watchdog tripped immediately after arm")
    if st.get("watchdog_ms") != 800:
        errors.append(f"watchdog_ms not applied: {st.get('watchdog_ms')}")

    print("sleeping 1.5s without keepalive...")
    time.sleep(1.5)
    st = client.call("get_status")
    print("after sleep:", st.get("watchdog_tripped"))
    if not st.get("watchdog_tripped"):
        errors.append("watchdog did not trip after timeout")

    # tripped -> motion must be blocked
    print("\n-- motion blocked while tripped --")
    try:
        client.call("move_linear", {"x": 0, "y": 0, "feed": 500})
        errors.append("move accepted while watchdog tripped")
    except Exception as exc:
        print("rejected as expected:", exc)

    # keepalive clears the latch
    print("\n-- keepalive clears trip --")
    client.call("keepalive")
    st = client.call("get_status")
    print("after keepalive:", st.get("watchdog_tripped"))
    if st.get("watchdog_tripped"):
        errors.append("watchdog_tripped not cleared by keepalive")

    # disable watchdog for the rest of the session
    client.call("configure", {"watchdog_ms": 0})
    st = client.call("get_status")
    if st.get("watchdog_ms") != 0:
        errors.append(f"watchdog_ms not disabled: {st.get('watchdog_ms')}")

    # ---- limit_status latch clear via clear_alerts ----
    print("\n-- clear_alerts clears limit_status --")
    client.call("clear_alerts")
    st = client.call("get_status")
    print("limit_status after clear_alerts:", st.get("limit_status"))
    if st.get("limit_status", {}).get("tripped"):
        errors.append("limit_status.tripped still true after clear_alerts")

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
