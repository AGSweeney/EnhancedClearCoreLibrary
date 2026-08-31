"""Test I/O enhancements: read_analog, write_analog, write_pwm, and conflict guards."""

from __future__ import annotations

import json
import sys

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
    for need in ("read_analog", "write_analog", "write_pwm"):
        if need not in methods:
            errors.append(f"missing method {need}")

    # ---- read_analog on A-9 (volts + raw sane) ----
    print("\n-- read_analog pin 9 --")
    r = client.call("read_analog", {"pin": 9})
    print(json.dumps(r, indent=2))
    pins = r.get("pins", [])
    if not pins or pins[0].get("pin") != 9:
        errors.append(f"read_analog pin 9 bad response: {r}")
    else:
        v = pins[0].get("volts")
        raw = pins[0].get("raw")
        if not isinstance(v, (int, float)) or not (0 <= v <= 5):
            errors.append(f"read_analog volts out of range: {v}")
        if not isinstance(raw, int) or not (0 <= raw <= 4095):
            errors.append(f"read_analog raw out of range: {raw}")

    # ---- read_analog all (9-12) ----
    print("\n-- read_analog all --")
    r = client.call("read_analog", {})
    print(json.dumps(r, indent=2))
    if len(r.get("pins", [])) < 1:
        errors.append("read_analog all returned no pins")

    # ---- write_analog IO-0 (microamps) ----
    print("\n-- write_analog IO-0 microamps=5000 --")
    r = client.call("write_analog", {"pin": 0, "microamps": 5000})
    print(json.dumps(r, indent=2))
    ri = client.call("read_inputs", {"pin": 0})
    print("read_inputs pin 0:", ri)
    p0 = ri.get("pins", [{}])[0]
    if p0.get("mode") != "analog_out":
        errors.append(f"IO-0 mode not analog_out after write_analog: {p0.get('mode')}")

    # back to digital so we don't leave a current sourcing
    client.call("write_output", {"pin": 0, "state": False})

    # ---- write_pwm IO-1 ----
    print("\n-- write_pwm IO-1 duty=128 --")
    r = client.call("write_pwm", {"pin": 1, "duty": 128})
    print(json.dumps(r, indent=2))
    ri = client.call("read_inputs", {"pin": 1})
    print("read_inputs pin 1:", ri)
    p1 = ri.get("pins", [{}])[0]
    if p1.get("mode") != "pwm":
        errors.append(f"IO-1 mode not pwm after write_pwm: {p1.get('mode')}")
    st = client.call("get_status")
    if st.get("pwm_duty", [0])[1] != 128:
        errors.append(f"pwm_duty[1] not 128: {st.get('pwm_duty')}")

    # restore IO-1 to digital off
    client.call("write_output", {"pin": 1, "state": False})

    # ---- conflict guards ----
    print("\n-- conflict guards --")
    # write_pwm on pin > 5 must be rejected
    try:
        client.call("write_pwm", {"pin": 6, "duty": 10})
        errors.append("write_pwm pin 6 accepted (should be rejected)")
    except Exception as exc:
        print("write_pwm pin 6 rejected as expected:", exc)

    # write_analog on pin != 0 must be rejected
    try:
        client.call("write_analog", {"pin": 1, "value": 100})
        errors.append("write_analog pin 1 accepted (should be rejected)")
    except Exception as exc:
        print("write_analog pin 1 rejected as expected:", exc)

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
