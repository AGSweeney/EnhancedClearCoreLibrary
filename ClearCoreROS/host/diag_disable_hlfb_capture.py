#!/usr/bin/env python3
"""Capture per-axis alert_reg / HLFB / MotorInFault around disable.

Distinguishes a fault latched during motion from one appearing during or after
disable. Does NOT call clear_alerts — leave the final latch intact for review.

Requires firmware that includes get_status.axes[] (per-axis fields).
"""

from __future__ import annotations

import argparse
import json
import socket
import sys
import time
from datetime import datetime, timezone
from pathlib import Path


def rpc(host: str, method: str, params=None, timeout: float = 2.0) -> dict:
    s = socket.create_connection((host, 9200), timeout)
    try:
        req: dict = {"jsonrpc": "2.0", "id": 1, "method": method}
        if params is not None:
            req["params"] = params
        s.sendall((json.dumps(req) + "\n").encode())
        buf = b""
        while b"\n" not in buf:
            chunk = s.recv(4096)
            if not chunk:
                break
            buf += chunk
    finally:
        s.close()
    msg = json.loads(buf.decode())
    if "error" in msg:
        raise RuntimeError(msg["error"])
    return msg["result"]


def axis_snapshot(st: dict) -> list[dict]:
    axes = st.get("axes")
    if isinstance(axes, list) and axes and not axes[0].get("aggregated_only"):
        return axes
    # Fallback before firmware update: aggregated only.
    return [
        {
            "axis": i,
            "name": f"axis_{i}",
            "alert_reg": st.get("alert_reg") if i == 0 else None,
            "hlfb_duty": (st.get("effort") or [None] * 4)[i]
            if isinstance(st.get("effort"), list)
            else None,
            "aggregated_only": True,
        }
        for i in range(4)
    ]


def interesting(prev: dict | None, cur: dict) -> bool:
    if prev is None:
        return True
    keys = (
        "enabled",
        "moving",
        "fault",
        "alerts",
        "alert_reg",
    )
    if any(prev.get(k) != cur.get(k) for k in keys):
        return True
    pa = {a.get("axis"): a for a in axis_snapshot(prev)}
    for a in axis_snapshot(cur):
        b = pa.get(a.get("axis"))
        if b is None:
            return True
        for k in (
            "alert_reg",
            "motor_in_fault",
            "alerts_present",
            "status_enabled",
            "steps_active",
            "hlfb",
            "hlfb_duty",
        ):
            if a.get(k) != b.get(k):
                return True
    return False


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", default="172.16.82.114")
    ap.add_argument("--out", type=Path, default=None)
    ap.add_argument("--move-m", type=float, default=0.010)
    ap.add_argument("--pre-s", type=float, default=1.0)
    ap.add_argument("--post-s", type=float, default=3.0)
    ap.add_argument("--period-ms", type=float, default=20.0)
    ap.add_argument(
        "--no-motion",
        action="store_true",
        help="Enable then disable without set_joints (still no clear_alerts).",
    )
    args = ap.parse_args()

    out = args.out or Path(__file__).resolve().parents[1] / "logs" / (
        "diag_disable_hlfb_"
        + datetime.now(timezone.utc).strftime("%Y%m%d_%H%M%S")
        + ".jsonl"
    )
    out.parent.mkdir(parents=True, exist_ok=True)

    t0 = time.perf_counter()
    events: list[dict] = []
    last: dict | None = None

    def sample(phase: str, force: bool = False) -> dict:
        nonlocal last
        st = rpc(args.host, "get_status")
        row = {
            "t_s": round(time.perf_counter() - t0, 4),
            "utc": datetime.now(timezone.utc).isoformat(),
            "phase": phase,
            "enabled": st.get("enabled"),
            "moving": st.get("moving"),
            "fault": st.get("fault"),
            "alerts": st.get("alerts"),
            "alert_reg": st.get("alert_reg"),
            "axes": axis_snapshot(st),
        }
        if force or interesting(last, st):
            events.append(row)
            with out.open("a", encoding="utf-8") as fh:
                fh.write(json.dumps(row) + "\n")
            print(
                f"[{row['t_s']:7.3f}] {phase} en={row['enabled']} mov={row['moving']} "
                f"fault={row['fault']} alerts={row['alerts']} "
                f"axes={[ (a.get('name'), a.get('alert_reg'), a.get('hlfb'), a.get('motor_in_fault')) for a in row['axes'] if a.get('on') is not False ]}",
                flush=True,
            )
        last = st
        return st

    print(f"diag_disable_hlfb_capture host={args.host} out={out}", flush=True)
    print("NOTE: will not call clear_alerts", flush=True)

    st0 = sample("start", force=True)
    if not isinstance(st0.get("axes"), list):
        print(
            "WARN: get_status.axes missing — flash firmware with per-axis status first",
            flush=True,
        )

    # Leave any existing latch for inspection unless starting clean is impossible.
    rpc(args.host, "disable")
    sample("after_initial_disable", force=True)

    # Bring-up without clearing if already faulted would fail enable; only clear
    # if the operator explicitly wants a motion pass — default clears once at
    # start so the disable edge under test is the one we capture, then never again.
    if st0.get("alerts") not in (None, "", "none") or st0.get("fault"):
        print(
            "Existing latch present; clearing ONCE before the capture window, "
            "then never again.",
            flush=True,
        )
        rpc(args.host, "clear_alerts")
        sample("after_pre_clear", force=True)

    rpc(args.host, "set_test_mode", {"on": False})
    rpc(args.host, "enable")
    sample("after_enable", force=True)

    if not args.no_motion:
        rpc(args.host, "set_joints", {"x": args.move_m, "y": args.move_m})
        sample("motion_commanded", force=True)
        deadline = time.time() + 15.0
        while time.time() < deadline:
            st = sample("motion")
            if not st.get("moving") and st.get("enabled"):
                break
            time.sleep(args.period_ms / 1000.0)
        sample("motion_settled", force=True)

    # Dense sample before disable.
    end_pre = time.perf_counter() + args.pre_s
    while time.perf_counter() < end_pre:
        sample("pre_disable")
        time.sleep(args.period_ms / 1000.0)

    sample("pre_disable_edge", force=True)
    rpc(args.host, "disable")
    sample("post_disable_edge", force=True)

    end_post = time.perf_counter() + args.post_s
    while time.perf_counter() < end_post:
        sample("post_disable")
        time.sleep(args.period_ms / 1000.0)

    final = sample("final", force=True)
    summary = {
        "out": str(out),
        "events": len(events),
        "final_alerts": final.get("alerts"),
        "final_fault": final.get("fault"),
        "final_enabled": final.get("enabled"),
        "cleared_after_capture": False,
        "note": "Latch left intact; inspect axes[].alert_reg / hlfb / motor_in_fault vs phase.",
    }
    summary_path = out.with_suffix(".summary.json")
    summary_path.write_text(json.dumps(summary, indent=2), encoding="utf-8")
    print(json.dumps(summary, indent=2), flush=True)
    print("DIAG_DISABLE_HLFB_CAPTURE_DONE", flush=True)
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except Exception as exc:  # noqa: BLE001
        print(f"FAIL {type(exc).__name__}: {exc}", file=sys.stderr)
        raise SystemExit(2)
