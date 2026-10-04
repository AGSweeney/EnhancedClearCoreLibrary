"""Bench client for ClearCoreROS. Does not import ROS."""

from __future__ import annotations

import argparse
import json
import os
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "ros2_ws" / "src" / "clearcore_bridge"))

from clearcore_bridge.wire import SessionClient, StreamClient, discover  # noqa: E402

AXIS = ("x", "y", "z", "a")


def _session(args) -> SessionClient:
    return SessionClient(args.host, args.session_port, args.timeout)


def _print(result) -> None:
    print(json.dumps(result, indent=2))


def cmd_discover(args) -> None:
    print(discover(args.host, args.discover_port, args.timeout))


def cmd_call(args) -> None:
    client = _session(args)
    try:
        params = json.loads(args.params) if args.params else None
        _print(client.call(args.method, params))
    finally:
        client.close()


def cmd_configure(args) -> None:
    params = {}
    if args.axis_mask is not None:
        params["axis_mask"] = args.axis_mask
    if args.steps_per_rev is not None:
        params["steps_per_rev"] = args.steps_per_rev
    if args.pitch_mm is not None:
        params["pitch_mm"] = args.pitch_mm
    if args.vel is not None:
        params["vel_steps"] = args.vel
    if args.accel is not None:
        params["accel_steps"] = args.accel
    if args.decel is not None:
        params["decel_steps"] = args.decel
    if args.watchdog_ms is not None:
        params["watchdog_ms"] = args.watchdog_ms
    if args.estop_di6 is not None:
        params["estop_di6"] = args.estop_di6
    for axis in AXIS:
        for side in ("min", "max"):
            value = getattr(args, f"{side}_{axis}")
            if value is not None:
                params[f"{side}_{axis}"] = value
        for side in ("pos", "neg"):
            value = getattr(args, f"{side}_lim_{axis}")
            if value is not None:
                params[f"{side}_lim_{axis}"] = value
    if args.clear_limits:
        params["clear_limits"] = True
    client = _session(args)
    try:
        _print(client.call("configure", params))
    finally:
        client.close()


def cmd_xrce_connect(args) -> None:
    params = {"ip_address": args.ip}
    if args.port is not None:
        params["port"] = args.port
    client = _session(args)
    try:
        _print(client.call("xrce_connect", params))
    finally:
        client.close()


def cmd_configure_network(args) -> None:
    params = {}
    if args.mode is not None:
        params["mode"] = args.mode
    if args.ip is not None:
        params["ip_address"] = args.ip
    if args.netmask is not None:
        params["netmask"] = args.netmask
    if args.gateway is not None:
        params["gateway"] = args.gateway
    client = _session(args)
    try:
        _print(client.call("configure_network", params))
    finally:
        client.close()


def cmd_test_mode(args) -> None:
    client = _session(args)
    try:
        _print(client.call("set_test_mode", {"on": args.on}))
    finally:
        client.close()


def _simple(args, method: str) -> None:
    client = _session(args)
    try:
        _print(client.call(method))
    finally:
        client.close()


def cmd_move(args) -> None:
    named = {"x": args.x, "y": args.y, "z": args.z, "a": args.a}
    present = {k: v for k, v in named.items() if v is not None}
    if not present:
        raise SystemExit("pass at least one of --x --y --z --a (meters, A in radians)")
    mask = 0
    q = [0.0, 0.0, 0.0, 0.0]
    for name, value in present.items():
        bit = AXIS.index(name)
        mask |= 1 << bit
        q[bit] = value

    session = _session(args)
    stream = None
    try:
        if args.stream:
            stream = StreamClient(args.host, args.stream_port, args.timeout)
            stream.send_position(mask, q)
        else:
            _print(session.call("set_joints", present))
        deadline = time.time() + args.move_timeout
        tol = args.tol
        while time.time() < deadline:
            if stream is not None:
                stream.send_heartbeat()
            status = session.call("get_status")
            pos = status["position"]
            err = max(abs(pos[AXIS.index(name)] - value) for name, value in present.items())
            print(
                f"pos={['%.4f' % p for p in pos]} err={err:.5f} "
                f"moving={status['moving']} watchdog={status['watchdog']}"
            )
            if status.get("watchdog"):
                raise SystemExit("watchdog tripped; call clear-alerts")
            if status.get("estop"):
                raise SystemExit("estop active")
            if status.get("fault"):
                raise SystemExit("fault")
            if not status.get("enabled"):
                raise SystemExit("motor not enabled")
            if err <= tol and not status["moving"]:
                _print(status)
                return
            time.sleep(0.1)
        raise SystemExit("timed out waiting for the joint target")
    finally:
        if stream is not None:
            stream.close()
        session.close()


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="ClearCoreROS bench client")
    parser.add_argument("--host", default=os.environ.get("CCROS_HOST", "192.168.0.109"))
    parser.add_argument("--session-port", type=int, default=9200)
    parser.add_argument("--stream-port", type=int, default=9201)
    parser.add_argument("--discover-port", type=int, default=9202)
    parser.add_argument("--timeout", type=float, default=2.0)
    sub = parser.add_subparsers(dest="cmd", required=True)

    sub.add_parser("discover").set_defaults(func=cmd_discover)

    call = sub.add_parser("call")
    call.add_argument("method")
    call.add_argument("--params", help="JSON object")
    call.set_defaults(func=cmd_call)

    for name in ("capabilities", "config", "status", "enable", "disable", "stop", "estop", "clear-alerts", "keepalive"):
        method = name.replace("-", "_")
        if name == "capabilities":
            method = "get_capabilities"
        elif name == "config":
            method = "get_config"
        elif name == "status":
            method = "get_status"
        item = sub.add_parser(name)
        item.set_defaults(func=lambda a, m=method: _simple(a, m))

    cfg = sub.add_parser("configure")
    cfg.add_argument("--axis-mask", type=int)
    cfg.add_argument("--steps-per-rev", type=int)
    cfg.add_argument("--pitch-mm", type=float)
    cfg.add_argument("--vel", type=int)
    cfg.add_argument("--accel", type=int)
    cfg.add_argument("--decel", type=int)
    cfg.add_argument("--watchdog-ms", type=int)
    cfg.add_argument("--estop-di6", type=int)
    cfg.add_argument("--clear-limits", action="store_true")
    for axis in AXIS:
        cfg.add_argument(f"--min-{axis}", type=float)
        cfg.add_argument(f"--max-{axis}", type=float)
        cfg.add_argument(f"--pos-lim-{axis}", type=int)
        cfg.add_argument(f"--neg-lim-{axis}", type=int)
    cfg.set_defaults(func=cmd_configure)

    net = sub.add_parser("configure-network", help="Persist DHCP or a static address. Applies on restart.")
    net.add_argument("--mode", choices=("dhcp", "static"))
    net.add_argument("--ip")
    net.add_argument("--netmask")
    net.add_argument("--gateway")
    net.set_defaults(func=cmd_configure_network)

    xrce = sub.add_parser("xrce-connect", help="Publish JointState to a micro-ROS agent.")
    xrce.add_argument("--ip", required=True)
    xrce.add_argument("--port", type=int)
    xrce.set_defaults(func=cmd_xrce_connect)
    sub.add_parser("xrce-disconnect").set_defaults(func=lambda a: _simple(a, "xrce_disconnect"))

    sub.add_parser("restart").set_defaults(func=lambda a: _simple(a, "restart"))
    sub.add_parser("reset-config").set_defaults(func=lambda a: _simple(a, "reset_config"))

    test = sub.add_parser("test-mode")
    test.add_argument("--on", action="store_true")
    test.set_defaults(func=cmd_test_mode)

    move = sub.add_parser("move", help="Absolute joint target. X/Y/Z meters, A radians.")
    move.add_argument("--x", type=float)
    move.add_argument("--y", type=float)
    move.add_argument("--z", type=float)
    move.add_argument("--a", type=float)
    move.add_argument("--stream", action="store_true", help="Send a binary position frame instead of set_joints")
    move.add_argument("--tol", type=float, default=0.0005)
    move.add_argument("--move-timeout", type=float, default=30.0)
    move.set_defaults(func=cmd_move)
    return parser


def main() -> None:
    parser = build_parser()
    args = parser.parse_args()
    args.func(args)


if __name__ == "__main__":
    main()
