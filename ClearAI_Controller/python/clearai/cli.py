"""CLI: python -m clearai --port COM5 enable"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Any, Optional

from .client import ClearAiClient, ClearAiError, discover
from .mcp_server import run_mcp


METHODS = (
    "get_capabilities",
    "get_status",
    "get_pose",
    "enable",
    "disable",
    "clear_alerts",
    "stop",
    "estop",
    "wait_idle",
    "configure",
    "set_test_mode",
    "set_units",
    "set_mode",
    "set_work_origin",
    "move_linear",
    "move_arc",
    "jog",
    "dwell",
)


def _add_common(p: argparse.ArgumentParser) -> None:
    p.add_argument("--port", help="USB serial port (COMx)")
    p.add_argument("--host", help="ClearAI TCP host")
    p.add_argument("--tcp-port", type=int, default=9100)
    p.add_argument("--baud", type=int, default=115200)


def _connect(args: argparse.Namespace) -> ClearAiClient:
    client = ClearAiClient()
    if args.host:
        client.connect_tcp(args.host, port=args.tcp_port)
    elif args.port:
        client.connect_serial(args.port, baud=args.baud)
    else:
        ip = discover()
        if not ip:
            raise SystemExit("no ClearAI on UDP 9102; pass --host or --port")
        client.connect_tcp(ip, port=args.tcp_port)
    return client


def _params_from_args(args: argparse.Namespace, keys: tuple[str, ...]) -> dict[str, Any]:
    out: dict[str, Any] = {}
    for key in keys:
        attr = key.replace("-", "_")
        val = getattr(args, attr, None)
        if val is not None:
            out[key] = val
    return out


def main(argv: Optional[list[str]] = None) -> int:
    parser = argparse.ArgumentParser(prog="clearai", description="ClearAI gantry JSON-RPC client")
    _add_common(parser)
    sub = parser.add_subparsers(dest="cmd", required=True)

    mcp_p = sub.add_parser("mcp", help="Run MCP stdio server over Ethernet (UDP discover + TCP 9100)")
    _add_common(mcp_p)
    disc_p = sub.add_parser("discover", help="UDP-broadcast CLEARAI_DISCOVER? and print the board IP")
    _ = disc_p

    for name in METHODS:
        sp = sub.add_parser(name)
        if name == "wait_idle":
            sp.add_argument("--timeout-ms", type=int, default=60000)
        if name == "set_units":
            sp.add_argument("--units", required=True, choices=("mm", "inch"))
        if name == "set_mode":
            sp.add_argument("--mode", required=True, choices=("abs", "rel"))
        if name in ("move_linear", "jog", "set_work_origin"):
            sp.add_argument("--x", type=float)
            sp.add_argument("--y", type=float)
            sp.add_argument("--z", type=float)
            sp.add_argument("--a", type=float)
        if name == "move_linear":
            sp.add_argument("--feed", type=float)
            sp.add_argument("--rapid", action="store_true")
        if name == "jog":
            sp.add_argument("--feed", type=float)
        if name == "move_arc":
            sp.add_argument("--x", type=float)
            sp.add_argument("--y", type=float)
            sp.add_argument("--i", type=float, required=True)
            sp.add_argument("--j", type=float, required=True)
            sp.add_argument("--clockwise", action="store_true")
            sp.add_argument("--feed", type=float)
        if name == "dwell":
            sp.add_argument("--seconds", type=float, required=True)
        if name == "set_test_mode":
            sp.add_argument("--on", dest="enabled", action="store_true", default=True)
            sp.add_argument("--off", dest="enabled", action="store_false")
        if name == "configure":
            sp.add_argument("--axis-mask", type=int)
            sp.add_argument("--vel", type=int)
            sp.add_argument("--accel", type=int)
            sp.add_argument("--decel", type=int)
            sp.add_argument("--estop-di6", type=int)
            sp.add_argument("--steps-per-rev-x", type=int)
            sp.add_argument("--steps-per-rev-y", type=int)
            sp.add_argument("--pitch-x", type=float)
            sp.add_argument("--pitch-y", type=float)
            sp.add_argument("--test-mode", type=int, choices=(0, 1))

    tools_p = sub.add_parser("tools", help="Print OpenAI/Anthropic tool schema JSON")
    _ = tools_p

    args = parser.parse_args(argv)
    if args.cmd == "tools":
        path = Path(__file__).with_name("tools.json")
        sys.stdout.write(path.read_text(encoding="utf-8"))
        return 0
    if args.cmd == "mcp":
        return run_mcp(args)
    if args.cmd == "discover":
        ip = discover()
        if not ip:
            sys.stderr.write("no reply on UDP 9102\n")
            return 2
        json.dump({"host": ip, "tcp": 9100, "tel": 9101, "discover": 9102}, sys.stdout)
        sys.stdout.write("\n")
        return 0

    client = _connect(args)
    try:
        params: dict[str, Any] = {}
        if args.cmd == "wait_idle":
            params = {"timeout_ms": args.timeout_ms}
            timeout = max(1.0, args.timeout_ms / 1000.0 + 2.0)
        elif args.cmd == "set_units":
            params = {"units": args.units}
            timeout = 5.0
        elif args.cmd == "set_mode":
            params = {"mode": args.mode}
            timeout = 5.0
        elif args.cmd == "set_test_mode":
            params = {"enabled": bool(args.enabled)}
            timeout = 5.0
        elif args.cmd == "dwell":
            params = {"seconds": args.seconds}
            timeout = max(1.0, args.seconds + 2.0)
        elif args.cmd == "move_arc":
            params = _params_from_args(args, ("x", "y", "i", "j", "feed"))
            params["clockwise"] = bool(args.clockwise)
            timeout = 5.0
        elif args.cmd == "move_linear":
            params = _params_from_args(args, ("x", "y", "z", "a", "feed"))
            if args.rapid:
                params["rapid"] = True
            timeout = 5.0
        elif args.cmd in ("jog", "set_work_origin"):
            params = _params_from_args(args, ("x", "y", "z", "a") + (("feed",) if args.cmd == "jog" else ()))
            timeout = 5.0
        elif args.cmd == "configure":
            params = {}
            mapping = {
                "axis_mask": "axis_mask",
                "vel": "vel",
                "accel": "accel",
                "decel": "decel",
                "estop_di6": "estop_di6",
                "steps_per_rev_x": "steps_per_rev_x",
                "steps_per_rev_y": "steps_per_rev_y",
                "pitch_x": "pitch_x",
                "pitch_y": "pitch_y",
                "test_mode": "test_mode",
            }
            for attr, key in mapping.items():
                val = getattr(args, attr, None)
                if val is not None:
                    params[key] = bool(val) if key == "test_mode" else val
            timeout = 5.0
        else:
            timeout = 5.0
        result = client.call(args.cmd, params, timeout=timeout)
        json.dump(result, sys.stdout, indent=2)
        sys.stdout.write("\n")
        return 0
    except ClearAiError as exc:
        sys.stderr.write(f"error {exc.code}: {exc.message}\n")
        return 2
    finally:
        client.close()


if __name__ == "__main__":
    raise SystemExit(main())
