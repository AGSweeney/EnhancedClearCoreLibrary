"""Test input-change notifications: subscribe_inputs -> input_changed on telemetry 9101."""

from __future__ import annotations

import sys
import threading
import time

from clearai.client import ClearAiClient, TelemetryReader, discover


def main() -> int:
    host = discover() or "172.16.82.113"
    print(f"Host: {host}")
    client = ClearAiClient()
    client.connect_tcp(host)

    errors: list[str] = []

    caps = client.call("get_capabilities")
    methods = caps.get("methods", [])
    for need in ("subscribe_inputs", "unsubscribe_inputs"):
        if need not in methods:
            errors.append(f"missing method {need}")

    # Use DI-6 (input-only, never a limit by default) as the watched pin.
    watch_pin = 6

    events: list[tuple] = []
    ready = threading.Event()

    def on_event(method, params):
        if method == "input_changed":
            events.append((params.get("pin"), params.get("state"), params.get("edge")))

    reader = TelemetryReader(host)
    reader.start(on_event)
    time.sleep(0.5)

    # ---- subscribe ----
    print(f"\n-- subscribe_inputs pin {watch_pin} --")
    r = client.call("subscribe_inputs", {"pins": [watch_pin], "debounce_ms": 5})
    print(r)

    # Toggle the watched pin via... we cannot drive DI-6 (input only). Instead,
    # subscribe to an output-capable pin (IO-7? no). Use IO-2: drive it as output
    # and subscribe to read its own state? Outputs are not readable as inputs the
    # same way. Instead, toggle IO-2 and watch pin 2 -- read_inputs reports the
    # driven output state, and MotionPollInputs reads connector->State() which
    # reflects the driven level for output pins too.
    watch_pin = 2
    r = client.call("subscribe_inputs", {"pins": [watch_pin], "debounce_ms": 5})
    print("re-subscribe on pin 2:", r)
    time.sleep(0.1)

    # drive a rising edge then a falling edge
    print("driving pin 2 rising...")
    client.call("write_output", {"pin": 2, "state": True})
    time.sleep(0.3)
    print("driving pin 2 falling...")
    client.call("write_output", {"pin": 2, "state": False})
    time.sleep(0.3)

    print("events seen:", events)
    rising = any(e[0] == 2 and e[2] == "rising" for e in events)
    falling = any(e[0] == 2 and e[2] == "falling" for e in events)
    if not rising:
        errors.append("no rising edge event for pin 2")
    if not falling:
        errors.append("no falling edge event for pin 2")

    # ---- unsubscribe stops events ----
    print("\n-- unsubscribe_inputs --")
    client.call("unsubscribe_inputs", {})
    events.clear()
    time.sleep(0.2)
    client.call("write_output", {"pin": 2, "state": True})
    time.sleep(0.4)
    client.call("write_output", {"pin": 2, "state": False})
    time.sleep(0.2)
    if events:
        errors.append(f"events received after unsubscribe: {events}")

    reader.stop()
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
