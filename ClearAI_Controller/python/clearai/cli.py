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
    "get_config",
    "enable",
    "disable",
    "clear_alerts",
    "stop",
    "estop",
    "wait_idle",
    "configure",
    "reset_config",
    "set_test_mode",
    "set_units",
    "set_units_a",
    "set_mode",
    "set_work_origin",
    "move_linear",
    "move_arc",
    "jog",
    "dwell",
    "read_inputs",
    "write_output",
    "queue_status",
    "queue_clear",
    "home",
    "probe",
    "keepalive",
    "read_analog",
    "write_analog",
    "write_pwm",
    "subscribe_inputs",
    "unsubscribe_inputs",
    "configure_network",
    "restart",
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
        if name == "set_units_a":
            sp.add_argument("--units", required=True, choices=("deg", "rev"))
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
        if name == "read_inputs":
            sp.add_argument("--pin", type=int)
        if name == "read_analog":
            sp.add_argument("--pin", type=int)
        if name == "write_output":
            sp.add_argument("--pin", type=int, required=True)
            sp.add_argument("--state", type=int, choices=(0, 1), required=True)
        if name == "write_analog":
            sp.add_argument("--pin", type=int, default=0)
            sp.add_argument("--value", type=int)
            sp.add_argument("--microamps", type=int)
        if name == "write_pwm":
            sp.add_argument("--pin", type=int, required=True)
            sp.add_argument("--duty", type=int, required=True)
        if name == "subscribe_inputs":
            sp.add_argument("--pin", type=int, nargs="+", required=True)
            sp.add_argument("--debounce-ms", type=int)
        if name == "unsubscribe_inputs":
            pass
        if name == "configure_network":
            sp.add_argument("--mode", choices=("dhcp", "static"))
            sp.add_argument("--ip", dest="ip_address")
            sp.add_argument("--netmask")
            sp.add_argument("--gateway")
        if name == "restart":
            pass
        if name in ("home", "probe"):
            sp.add_argument("--axis", required=True, choices=("x", "y", "z", "a"))
            sp.add_argument("--dir", required=True, choices=("pos", "neg"))
            sp.add_argument("--feed", type=float)
            sp.add_argument("--seek", type=float)
            sp.add_argument("--backoff", type=float)
            sp.add_argument("--zero", action="store_true", default=True)
            sp.add_argument("--no-zero", dest="zero", action="store_false")
            sp.add_argument("--timeout-ms", type=int)
        if name == "probe":
            sp.add_argument("--pin", type=int, required=True)
            sp.add_argument("--active", choices=("high", "low"))
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
            sp.add_argument("--min-x", type=float)
            sp.add_argument("--max-x", type=float)
            sp.add_argument("--min-y", type=float)
            sp.add_argument("--max-y", type=float)
            sp.add_argument("--min-z", type=float)
            sp.add_argument("--max-z", type=float)
            sp.add_argument("--min-a", type=float)
            sp.add_argument("--max-a", type=float)
            sp.add_argument("--clear-limits", action="store_true")
            sp.add_argument("--clear-min-x", action="store_true")
            sp.add_argument("--clear-max-x", action="store_true")
            sp.add_argument("--clear-min-y", action="store_true")
            sp.add_argument("--clear-max-y", action="store_true")
            sp.add_argument("--clear-min-z", action="store_true")
            sp.add_argument("--clear-max-z", action="store_true")
            sp.add_argument("--clear-min-a", action="store_true")
            sp.add_argument("--clear-max-a", action="store_true")
            sp.add_argument("--pos-lim-x", type=int)
            sp.add_argument("--neg-lim-x", type=int)
            sp.add_argument("--pos-lim-y", type=int)
            sp.add_argument("--neg-lim-y", type=int)
            sp.add_argument("--pos-lim-z", type=int)
            sp.add_argument("--neg-lim-z", type=int)
            sp.add_argument("--pos-lim-a", type=int)
            sp.add_argument("--neg-lim-a", type=int)
            sp.add_argument("--watchdog-ms", type=int)
            sp.add_argument("--vel-x", type=int)
            sp.add_argument("--vel-y", type=int)
            sp.add_argument("--vel-z", type=int)
            sp.add_argument("--vel-a", type=int)
            sp.add_argument("--accel-x", type=int)
            sp.add_argument("--accel-y", type=int)
            sp.add_argument("--accel-z", type=int)
            sp.add_argument("--accel-a", type=int)
            sp.add_argument("--decel-x", type=int)
            sp.add_argument("--decel-y", type=int)
            sp.add_argument("--decel-z", type=int)
            sp.add_argument("--decel-a", type=int)
            sp.add_argument("--out-power-on-0", type=int)
            sp.add_argument("--out-power-on-1", type=int)
            sp.add_argument("--out-power-on-2", type=int)
            sp.add_argument("--out-power-on-3", type=int)
            sp.add_argument("--out-power-on-4", type=int)
            sp.add_argument("--out-power-on-5", type=int)

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
        elif args.cmd == "set_units_a":
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
        elif args.cmd in ("home", "probe"):
            params = {"axis": args.axis, "dir": args.dir}
            if args.feed is not None:
                params["feed"] = args.feed
            if args.seek is not None:
                params["seek"] = args.seek
            if args.backoff is not None:
                params["backoff"] = args.backoff
            if args.zero is not None:
                params["zero"] = args.zero
            if args.timeout_ms is not None:
                params["timeout_ms"] = args.timeout_ms
            if args.cmd == "probe":
                params["pin"] = args.pin
                if args.active is not None:
                    params["active"] = args.active
            timeout = max(1.0, (args.timeout_ms or 30000) / 1000.0 + 2.0)
        elif args.cmd == "read_inputs":
            if args.pin is not None:
                params = {"pin": args.pin}
            timeout = 5.0
        elif args.cmd == "read_analog":
            if args.pin is not None:
                params = {"pin": args.pin}
            timeout = 5.0
        elif args.cmd == "write_output":
            params = {"pin": args.pin, "state": bool(args.state)}
            timeout = 5.0
        elif args.cmd == "write_analog":
            params = {"pin": args.pin}
            if args.microamps is not None:
                params["microamps"] = args.microamps
            elif args.value is not None:
                params["value"] = args.value
            timeout = 5.0
        elif args.cmd == "write_pwm":
            params = {"pin": args.pin, "duty": args.duty}
            timeout = 5.0
        elif args.cmd == "subscribe_inputs":
            params = {"pins": list(args.pin)}
            if args.debounce_ms is not None:
                params["debounce_ms"] = args.debounce_ms
            timeout = 5.0
        elif args.cmd == "unsubscribe_inputs":
            params = {}
            timeout = 5.0
        elif args.cmd == "configure_network":
            params = {}
            if args.mode is not None:
                params["mode"] = args.mode
            if args.ip_address is not None:
                params["ip_address"] = args.ip_address
            if args.netmask is not None:
                params["netmask"] = args.netmask
            if args.gateway is not None:
                params["gateway"] = args.gateway
            timeout = 5.0
        elif args.cmd == "restart":
            params = {}
            timeout = 5.0
        elif args.cmd == "keepalive":
            params = {}
            timeout = 5.0
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
                "min_x": "min_x",
                "max_x": "max_x",
                "min_y": "min_y",
                "max_y": "max_y",
                "min_z": "min_z",
                "max_z": "max_z",
                "min_a": "min_a",
                "max_a": "max_a",
                "pos_lim_x": "pos_lim_x",
                "neg_lim_x": "neg_lim_x",
                "pos_lim_y": "pos_lim_y",
                "neg_lim_y": "neg_lim_y",
                "pos_lim_z": "pos_lim_z",
                "neg_lim_z": "neg_lim_z",
                "pos_lim_a": "pos_lim_a",
                "neg_lim_a": "neg_lim_a",
                "watchdog_ms": "watchdog_ms",
                "vel_x": "vel_x",
                "vel_y": "vel_y",
                "vel_z": "vel_z",
                "vel_a": "vel_a",
                "accel_x": "accel_x",
                "accel_y": "accel_y",
                "accel_z": "accel_z",
                "accel_a": "accel_a",
                "decel_x": "decel_x",
                "decel_y": "decel_y",
                "decel_z": "decel_z",
                "decel_a": "decel_a",
                "out_power_on_0": "out_power_on_0",
                "out_power_on_1": "out_power_on_1",
                "out_power_on_2": "out_power_on_2",
                "out_power_on_3": "out_power_on_3",
                "out_power_on_4": "out_power_on_4",
                "out_power_on_5": "out_power_on_5",
            }
            bool_keys = (
                "clear_limits",
                "clear_min_x",
                "clear_max_x",
                "clear_min_y",
                "clear_max_y",
                "clear_min_z",
                "clear_max_z",
                "clear_min_a",
                "clear_max_a",
            )
            for attr, key in mapping.items():
                val = getattr(args, attr, None)
                if val is not None:
                    params[key] = bool(val) if key == "test_mode" else val
            for key in bool_keys:
                if getattr(args, key, False):
                    params[key] = True
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
