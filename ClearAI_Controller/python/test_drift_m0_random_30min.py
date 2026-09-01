"""Extended random drift test for M0 (~30 minutes). Completely random large
moves (random size 20-60mm, random direction, random feed, occasional dwells),
then a single return move to bring the commanded net back to 0 at the end.
Expected final MSP count (zeroed at start) = 0. Any non-zero reading is drift."""

from __future__ import annotations

import json
import random
import sys
import time

from clearai.client import ClearAiClient, discover

DURATION_S = 30 * 60
SEED = 12345
DX_MIN = 20.0
DX_MAX = 60.0
FEED_MIN = 3000
FEED_MAX = 10000
DWELL_PROB = 0.05  # 5% of moves followed by a short dwell


def main() -> int:
    host = discover() or "172.16.82.113"
    print(f"Host: {host}", flush=True)
    print(f"Duration: {DURATION_S}s ({DURATION_S/60:.0f} min)  seed={SEED}", flush=True)
    print(f"Random |dx| in [{DX_MIN},{DX_MAX}] mm, feed in [{FEED_MIN},{FEED_MAX}] mm/min, dwell prob {DWELL_PROB}", flush=True)
    rng = random.Random(SEED)

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
    dwells = 0
    t0 = time.time()
    last_report = t0
    alerts_seen = 0
    max_abs_net = 0.0
    try:
        while True:
            elapsed = time.time() - t0
            if elapsed >= DURATION_S:
                break
            mag = rng.uniform(DX_MIN, DX_MAX)
            dx = mag if rng.random() < 0.5 else -mag
            feed = rng.uniform(FEED_MIN, FEED_MAX)
            c.call("move_linear", {"x": dx, "feed": feed})
            c.call("wait_idle", {"timeout_ms": 30000})
            net += dx
            moves_done += 1
            if abs(net) > max_abs_net:
                max_abs_net = abs(net)
            if rng.random() < DWELL_PROB:
                d = rng.uniform(0.1, 0.5)
                c.call("dwell", {"seconds": d})
                c.call("wait_idle", {"timeout_ms": 5000})
                dwells += 1

            now = time.time()
            if now - last_report >= 60.0:
                p = c.call("get_pose")
                st = c.call("get_status")
                if st.get("alert_reg", 0):
                    alerts_seen = st.get("alert_reg")
                print(f"[{elapsed:6.0f}s] moves={moves_done} dwells={dwells} "
                      f"net={net:+9.2f}mm (max|net|={max_abs_net:7.1f}) "
                      f"pose.x={p.get('x'):+9.3f} hlfb={st.get('hlfb')} alert_reg={st.get('alert_reg')}", flush=True)
                last_report = now
    except KeyboardInterrupt:
        print("\n*** interrupted", flush=True)
    except Exception as exc:
        print(f"\n*** exception: {exc}", flush=True)

    # Return to 0: single move that cancels the accumulated net.
    print("\n== returning to 0 ==", flush=True)
    print(f"accumulated net before return: {net:+.4f} mm", flush=True)
    ret = -net
    c.call("move_linear", {"x": ret, "feed": 6000})
    c.call("wait_idle", {"timeout_ms": 60000})
    net += ret

    p_end = c.call("get_pose")
    end_x = p_end.get("x", 0.0)
    net_pose = end_x - start_x
    elapsed = time.time() - t0
    print("\n== done ==", flush=True)
    print(f"elapsed: {elapsed:.1f}s ({elapsed/60:.2f} min)", flush=True)
    print(f"moves: {moves_done}  dwells: {dwells}  max|net| during run: {max_abs_net:.1f} mm", flush=True)
    print(f"start pose.x = {start_x:+.4f}", flush=True)
    print(f"end   pose.x = {end_x:+.4f}", flush=True)
    print(f"net (sum of dx)   = {net:+.4f} mm", flush=True)
    print(f"net (pose delta)  = {net_pose:+.4f} mm", flush=True)
    print(f"commanded net     = {net:+.4f} mm = {net/5.0:+.4f} rev = {net*160:+.0f} counts", flush=True)
    if alerts_seen:
        print(f"alerts observed during run: 0x{alerts_seen:x}", flush=True)
    print("Tell me the MSP 'Position (cnts)' reading.", flush=True)

    c.call("stop")
    c.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
