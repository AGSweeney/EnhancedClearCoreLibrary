"""Verify A-axis deg<->rev conversion via work origin round-trip."""

from __future__ import annotations

import json
import sys

from clearai.client import ClearAiClient, discover


def main() -> int:
    host = discover() or "172.16.82.113"
    client = ClearAiClient()
    client.connect_tcp(host)

    errors: list[str] = []

    # Zero origin first
    client.call("set_work_origin", {})

    # In deg mode, set A origin to 360 deg -> pose a should be 360
    client.call("set_units_a", {"units": "deg"})
    client.call("set_work_origin", {"a": 360.0})
    pose = client.call("get_pose")
    a_deg = pose.get("a")
    print(f"deg mode: set origin a=360 -> pose a = {a_deg} ({pose.get('units_a')})")

    # Switch to rev: same physical origin (360 deg = 1 rev) -> pose a should be 1.0
    client.call("set_units_a", {"units": "rev"})
    pose = client.call("get_pose")
    a_rev = pose.get("a")
    print(f"rev mode: pose a = {a_rev} ({pose.get('units_a')})")

    # Now set origin a=0.5 rev in rev mode -> 180 deg -> switch back to deg -> 180
    client.call("set_work_origin", {"a": 0.5})
    client.call("set_units_a", {"units": "deg"})
    pose = client.call("get_pose")
    a_back = pose.get("a")
    print(f"back to deg: set origin a=0.5rev -> pose a = {a_back} ({pose.get('units_a')})")

    if abs(a_deg - 360.0) > 1e-3:
        errors.append(f"deg pose {a_deg} != 360")
    if abs(a_rev - 1.0) > 1e-4:
        errors.append(f"rev pose {a_rev} != 1.0")
    if abs(a_back - 180.0) > 1e-3:
        errors.append(f"round-trip pose {a_back} != 180")

    # restore
    client.call("set_work_origin", {})
    client.call("set_units_a", {"units": "deg"})
    client.close()

    if errors:
        print("FAILED:")
        for e in errors:
            print(f"  - {e}")
        return 1
    print("PASSED")
    return 0


if __name__ == "__main__":
    sys.exit(main())
