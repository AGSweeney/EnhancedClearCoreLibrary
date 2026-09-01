"""Extended drift test for M0 (~30 minutes). Runs a net-neutral cycle of varied
move sizes repeatedly. Commanded final position should equal the start, so the
expected MSP count (zeroed at start) is ~0. Any non-zero MSP reading at the end
is accumulated drift/miscount.

Logs progress every 60s. Prints final commanded net (mm/rev/counts) at the end."""

from __future__ import annotations

import json
import sys
import time

from clearai.client import ClearAiClient, discover

DURATION_S = 30 * 60
FEED = 5000  # mm/min (~83 mm/s)
# 10 varied moves summing to 0 (net-neutral cycle): 25-10-20+5+15-15-5+20+10-25 = 0
CYCLE = [+25, -10, -20, +5, +15, -15, -5, +20, +10, -25]
DWELL_EVERY_CYCLES = 5
DWELL_S = 0.05


def main() -> int:
    host = discover() or "172.16.82.113"
    print(f"Host: {host}", flush=True)
    print(f"Duration: {DURATION_S}s ({DURATION_S/60:.0f} min)  feed={FEED} mm/min", flush=True)
    print(f"Cycle (net 0): {CYCLE}", flush=True)

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
    print("enabled:", st.get("enabled"), "hlfb:", st.get("hlfb"), "alert_reg:", st.get("alert_reg"), flush=True)

    p_start = c.call("get_pose")
    start_x = p_start.get("x", 0.0)
    print(f"start pose.x = {start_x:+.4f} mm (zero MSP at this point)", flush=True)
    print(">>> Go. Re-zero the MSP count now if you haven't.\n", flush=True)

    net = 0.0
    moves_done = 0
    cycles = 0
    t0 = time.time()
    last_report = t0
    alerts_seen = 0
    try:
        while True:
            elapsed = time.time() - t0
            if elapsed >= DURATION_S:
                break
            for dx in CYCLE:
                c.call("move_linear", {"x": dx, "feed": FEED})
                c.call("wait_idle", {"timeout_ms": 15000})
                net += dx
                moves_done += 1
            cycles += 1
            if cycles % DWELL_EVERY_CYCLES == 0:
                c.call("dwell", {"seconds": DWELL_S})
                c.call("wait_idle", {"timeout_ms": 5000})

            now = time.time()
            if now - last_report >= 60.0:
                p = c.call("get_pose")
                st = c.call("get_status")
                if st.get("alert_reg", 0):
                    alerts_seen = st.get("alert_reg")
                print(f"[{elapsed:6.0f}s] cycles={cycles} moves={moves_done} "
                      f"net={net:+8.2f}mm pose.x={p.get('x'):+8.3f} "
                      f"hlfb={st.get('hlfb')} alert_reg={st.get('alert_reg')}", flush=True)
                last_report = now
    except KeyboardInterrupt:
        print("\n*** interrupted", flush=True)
    except Exception as exc:
        print(f"\n*** exception: {exc}", flush=True)

    p_end = c.call("get_pose")
    end_x = p_end.get("x", 0.0)
    net_pose = end_x - start_x
    elapsed = time.time() - t0
    print("\n== done ==", flush=True)
    print(f"elapsed: {elapsed:.1f}s ({elapsed/60:.2f} min)", flush=True)
    print(f"cycles: {cycles}  moves: {moves_done}", flush=True)
    print(f"start pose.x = {start_x:+.4f}", flush=True)
    print(f"end   pose.x = {end_x:+.4f}", flush=True)
    print(f"net (sum of dx)   = {net:+.4f} mm", flush=True)
    print(f"net (pose delta)  = {net_pose:+.4f} mm", flush=True)
    print(f"commanded net     = {net:+.4f} mm = {net/5.0:+.4f} rev = {net*160:+.0f} counts (160 counts/mm)", flush=True)
    if alerts_seen:
        print(f"alerts observed during run: 0x{alerts_seen:x}", flush=True)
    print("Tell me the MSP 'Position (cnts)' reading.", flush=True)

    c.call("stop")
    c.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
