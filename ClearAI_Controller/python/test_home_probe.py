"""Test homing/probing validation paths (no physical switches needed).

The actual seek->trigger->zero success path requires physical limit /
probe switches, so this test only covers the param-validation and guard
paths that are verifiable on a bare board.
"""

from __future__ import annotations

import sys

from clearai.client import ClearAiClient, discover


def expect_reject(client, label, params, method="home"):
    try:
        client.call(method, params)
        return f"{label}: expected rejection but call succeeded"
    except Exception as e:
        print(f"  {label}: rejected -> {e}")
        return None


def main() -> int:
    host = discover() or "172.16.82.113"
    print(f"Host: {host}")
    client = ClearAiClient()
    client.connect_tcp(host)

    errors: list[str] = []

    caps = client.call("get_capabilities")
    methods = caps.get("methods", [])
    if "home" not in methods or "probe" not in methods:
        errors.append("capabilities missing home/probe")
    print("methods include home/probe:", all(m in methods for m in ("home", "probe")))

    # clean outputs + clear limits
    for p in range(6):
        client.call("write_output", {"pin": p, "state": False})
    client.call("configure", {"clear_limits": True})

    print("\n-- home validation --")
    e = expect_reject(client, "home missing axis", {"dir": "pos"})
    if e: errors.append(e)
    e = expect_reject(client, "home bad dir", {"axis": "x", "dir": "up"})
    if e: errors.append(e)
    e = expect_reject(client, "home bad axis", {"axis": "w", "dir": "pos"})
    if e: errors.append(e)
    e = expect_reject(client, "home limit not configured", {"axis": "x", "dir": "pos"})
    if e: errors.append(e)
    e = expect_reject(client, "home axis not in mask (z)", {"axis": "z", "dir": "pos"})
    if e: errors.append(e)

    print("\n-- probe validation --")
    e = expect_reject(client, "probe missing axis", {"dir": "neg", "pin": 6}, method="probe")
    if e: errors.append(e)
    e = expect_reject(client, "probe bad dir", {"axis": "x", "dir": "up", "pin": 6}, method="probe")
    if e: errors.append(e)
    e = expect_reject(client, "probe missing pin", {"axis": "x", "dir": "neg"}, method="probe")
    if e: errors.append(e)
    e = expect_reject(client, "probe pin 0 disabled", {"axis": "x", "dir": "neg", "pin": 0}, method="probe")
    if e: errors.append(e)
    e = expect_reject(client, "probe pin 13 invalid", {"axis": "x", "dir": "neg", "pin": 13}, method="probe")
    if e: errors.append(e)

    print("\n-- probe pin reserved for limit --")
    client.call("configure", {"pos_lim_x": 7})
    e = expect_reject(client, "probe on limit pin", {"axis": "x", "dir": "pos", "pin": 7}, method="probe")
    if e: errors.append(e)
    client.call("configure", {"clear_limits": True})

    print("\n-- probe already-active guard --")
    # IO-1 (pin 1) reads low when undriven; active=low -> already triggered.
    e = expect_reject(client, "probe already active",
                      {"axis": "x", "dir": "neg", "pin": 1, "active": "low"}, method="probe")
    if e: errors.append(e)

    print("\n-- home limit-already-active guard --")
    # Assign X+ limit to IO-1 (pin 1) which reads low; force pos polarity by
    # using active-high: pin reads low -> not active, so this should NOT trip
    # the already-active guard. Instead verify the guard trips for active-low
    # interpretation by configuring neg_lim_x=1 and homing neg (limit reads low
    # = active under active-low? firmware limit polarity is raw state!=0).
    # Firmware HwLimitSwitchActive uses State()!=0 (active high). Pin 1 low -> not active.
    # So home neg toward pin1 should start seeking (no immediate guard). Skip success.

    client.close()
    if errors:
        print("\nFAILED:")
        for e in errors:
            print(f"  - {e}")
        return 1
    print("\nPASSED: all homing/probing validation paths behave correctly")
    return 0


if __name__ == "__main__":
    sys.exit(main())
