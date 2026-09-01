"""Test static persisted IP: configure_network, get_config reporting, validation,
and restart (with reconnect). DHCP is restored at the end so the board returns to
DHCP on its next boot."""

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
    for need in ("configure_network", "restart"):
        if need not in methods:
            errors.append(f"missing method {need}")

    # ---- get_config should report network fields ----
    print("\n-- get_config (network fields) --")
    cfg = client.call("get_config")
    print(json.dumps({k: cfg.get(k) for k in
                      ("network_mode", "ip_address", "netmask", "gateway", "nvm_version")},
                     indent=2))
    for k in ("network_mode", "ip_address", "netmask", "gateway"):
        if k not in cfg:
            errors.append(f"get_config missing {k}")
    if cfg.get("nvm_version") != 7:
        errors.append(f"nvm_version expected 7, got {cfg.get('nvm_version')}")

    # ---- configure static IP ----
    print("\n-- configure_network static 192.168.0.201 --")
    r = client.call("configure_network", {
        "mode": "static",
        "ip_address": "192.168.0.201",
        "netmask": "255.255.255.0",
        "gateway": "192.168.0.1",
    })
    print(json.dumps(r, indent=2))
    if r.get("network_mode") != "static":
        errors.append(f"result network_mode not static: {r.get('network_mode')}")
    if r.get("applies_on") != "restart":
        errors.append(f"result applies_on not restart: {r.get('applies_on')}")

    cfg = client.call("get_config")
    print("get_config network:", cfg.get("network_mode"), cfg.get("ip_address"),
          cfg.get("netmask"), cfg.get("gateway"))
    if cfg.get("network_mode") != "static":
        errors.append(f"get_config network_mode not static: {cfg.get('network_mode')}")
    if cfg.get("ip_address") != "192.168.0.201":
        errors.append(f"ip_address not persisted: {cfg.get('ip_address')}")
    if cfg.get("netmask") != "255.255.255.0":
        errors.append(f"netmask not persisted: {cfg.get('netmask')}")
    if cfg.get("gateway") != "192.168.0.1":
        errors.append(f"gateway not persisted: {cfg.get('gateway')}")
    if cfg.get("nvm_version") != 7:
        errors.append(f"nvm_version not 7 after save: {cfg.get('nvm_version')}")

    # ---- validation guards ----
    print("\n-- validation guards --")
    try:
        client.call("configure_network", {"mode": "static", "ip_address": "999.1.1.1"})
        errors.append("invalid ip_address accepted")
    except Exception as exc:
        print("invalid ip rejected as expected:", exc)
    try:
        client.call("configure_network", {"mode": "foo"})
        errors.append("bad mode accepted")
    except Exception as exc:
        print("bad mode rejected as expected:", exc)
    # static without ip_address (and octets currently zero from prior? they are set now) -> ok since octets exist
    # but clearing octets then static-without-ip should fail; skip to avoid state coupling.

    # ---- restore DHCP (persisted) so a later reboot returns to DHCP ----
    print("\n-- restore DHCP --")
    client.call("configure_network", {"mode": "dhcp"})
    cfg = client.call("get_config")
    if cfg.get("network_mode") != "dhcp":
        errors.append(f"network_mode not restored to dhcp: {cfg.get('network_mode')}")

    # ---- restart (optional, last) ----
    print("\n-- restart (expect disconnect + reconnect) --")
    try:
        client.call("restart", {}, timeout=3.0)
        # If the call returns at all, the board didn't reset promptly.
        print("restart call returned without disconnect (board may reset shortly)")
    except Exception as exc:
        print("restart dropped connection as expected:", exc)

    client.close()

    # Re-discover and reconnect; confirm DHCP mode after reboot.
    print("waiting for board to come back up...")
    time.sleep(6)
    new_host = discover() or host
    print(f"rediscovered host: {new_host}")
    client2 = ClearAiClient()
    try:
        client2.connect_tcp(new_host, timeout=5.0)
        cfg2 = client2.call("get_config", timeout=5.0)
        print("post-restart network:", cfg2.get("network_mode"),
              cfg2.get("ip_address"))
        if cfg2.get("network_mode") != "dhcp":
            errors.append(f"post-restart network_mode not dhcp: {cfg2.get('network_mode')}")
    except Exception as exc:
        errors.append(f"failed to reconnect after restart: {exc}")
    finally:
        client2.close()

    if errors:
        print("\nFAILED:")
        for err in errors:
            print(f"  - {err}")
        return 1
    print("\nPASSED")
    return 0


if __name__ == "__main__":
    sys.exit(main())
