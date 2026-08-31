"""Walk IO-0..IO-5 outputs and verify read_inputs readback."""

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

    caps = client.call("get_capabilities")
    methods = caps.get("methods", [])
    for need in ("read_inputs", "write_output"):
        if need not in methods:
            print(f"ERROR: firmware missing {need}")
            return 1

    def all_off() -> None:
        for pin in range(6):
            client.call("write_output", {"pin": pin, "state": False})

    def pin_map(resp: dict) -> dict[int, dict]:
        return {int(entry["pin"]): entry for entry in resp.get("pins", [])}

    errors: list[str] = []
    try:
        all_off()
        time.sleep(0.2)
        print("Walking outputs 0-5 with readback...")
        for active in range(6):
            all_off()
            client.call("write_output", {"pin": active, "state": True})
            time.sleep(0.15)
            full = pin_map(client.call("read_inputs"))
            single = pin_map(client.call("read_inputs", {"pin": active}))
            act = full.get(active)
            one = single.get(active)
            print(
                f"  pin {active}: write=true  "
                f"all_read={json.dumps(act)}  single_read={json.dumps(one)}"
            )
            if not act or act.get("state") != 1:
                errors.append(f"pin {active}: expected state=1 in full read, got {act}")
            if not act or act.get("mode") != "out":
                errors.append(f"pin {active}: expected mode=out, got {act and act.get('mode')}")
            if not one or one.get("state") != 1:
                errors.append(f"pin {active}: expected state=1 in single read, got {one}")
            for pin in range(6):
                if pin == active:
                    continue
                other = full.get(pin)
                if other and other.get("state") != 0:
                    errors.append(
                        f"pin {active} active but pin {pin} reads state={other.get('state')}"
                    )

        all_off()
        final = pin_map(client.call("read_inputs"))
        print("All off:", {p: final.get(p, {}).get("state") for p in range(6)})
    finally:
        all_off()

    if errors:
        print("FAILED:")
        for err in errors:
            print(f"  - {err}")
        return 1

    print("PASSED: walking pattern with readback OK")
    return 0


if __name__ == "__main__":
    sys.exit(main())
