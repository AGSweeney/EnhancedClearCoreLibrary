"""No-motion TCP reconnect stress for the Transport TcpData Close fix.

Repeatedly opens session (and optionally stream), calls get_status/disable,
disconnects, and checks that the board keeps answering. Reports
tcp_*_accepts/closes from get_status when present.
Session accepts may read one ahead of closes while the status socket is still
open; after disconnect, closes catch up. Does not enable motors or command motion.
"""

from __future__ import annotations

import argparse
import json
import socket
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "ros2_ws" / "src" / "clearcore_bridge"))

from clearcore_bridge.wire import SessionClient, StreamClient  # noqa: E402


def one_cycle(host: str, with_stream: bool, timeout: float) -> dict:
    session = SessionClient(host, 9200, timeout)
    try:
        status = session.call("get_status")
        session.call("disable")
        cfg_ok = True
        try:
            session.call("get_config")
        except Exception:
            cfg_ok = False
        stream_ok = True
        if with_stream:
            stream = StreamClient(host, 9201, timeout)
            try:
                frames = stream.read(0.3)
                stream_ok = isinstance(frames, list)
            finally:
                stream.close()
        return {
            "ok": True,
            "enabled": status.get("enabled"),
            "alerts": status.get("alerts"),
            "tcp_session_accepts": status.get("tcp_session_accepts"),
            "tcp_session_closes": status.get("tcp_session_closes"),
            "tcp_stream_accepts": status.get("tcp_stream_accepts"),
            "tcp_stream_closes": status.get("tcp_stream_closes"),
            "cfg_ok": cfg_ok,
            "stream_ok": stream_ok,
        }
    finally:
        session.close()


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", default="172.16.82.114")
    ap.add_argument("--cycles", type=int, default=200)
    ap.add_argument("--timeout", type=float, default=2.0)
    ap.add_argument("--with-stream", action="store_true", default=True)
    ap.add_argument("--no-stream", action="store_true")
    ap.add_argument("--pause", type=float, default=0.05)
    args = ap.parse_args()
    with_stream = False if args.no_stream else args.with_stream

    print(f"TCP reconnect stress host={args.host} cycles={args.cycles} stream={with_stream}")
    first = None
    last = None
    failures = 0
    for i in range(1, args.cycles + 1):
        try:
            result = one_cycle(args.host, with_stream, args.timeout)
        except Exception as exc:  # noqa: BLE001
            failures += 1
            print(f"FAIL cycle={i} {type(exc).__name__}: {exc}", flush=True)
            if failures >= 5:
                print("ABORT too many consecutive failures")
                return 2
            time.sleep(0.5)
            continue
        failures = 0
        if first is None:
            first = result
        last = result
        if i == 1 or i % 25 == 0 or i == args.cycles:
            print(
                f"ok cycle={i}/{args.cycles} enabled={result['enabled']} "
                f"alerts={result['alerts']} "
                f"sess={result['tcp_session_accepts']}/{result['tcp_session_closes']} "
                f"strm={result['tcp_stream_accepts']}/{result['tcp_stream_closes']}",
                flush=True,
            )
        if result.get("enabled") is True:
            print("FAIL motors became enabled during no-motion stress")
            return 3
        time.sleep(args.pause)

    print("first", json.dumps(first, sort_keys=True))
    print("last", json.dumps(last, sort_keys=True))
    if last is None:
        return 1
    # Counters should exist on the fixed image and move with reconnects.
    for key in (
        "tcp_session_accepts",
        "tcp_session_closes",
        "tcp_stream_accepts",
        "tcp_stream_closes",
    ):
        if last.get(key) is None:
            print(f"WARN missing {key} (older firmware?)")
        elif first is not None and last[key] < first.get(key, 0):
            print(f"FAIL {key} decreased")
            return 4
    if last.get("tcp_session_accepts") is not None:
        if last["tcp_session_accepts"] < args.cycles:
            print(
                "WARN session accepts "
                f"{last['tcp_session_accepts']} < cycles {args.cycles} "
                "(board may have started mid-count)"
            )
    print("TCP_RECONNECT_STRESS_OK")
    return 0


if __name__ == "__main__":
    sys.exit(main())
