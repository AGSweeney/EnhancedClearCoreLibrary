"""Code as Policies primitive surface matching firmware method names."""

from __future__ import annotations

from typing import Any, Optional

from .client import ClearAiClient

_client: Optional[ClearAiClient] = None


def _c() -> ClearAiClient:
    if _client is None:
        raise RuntimeError("call connect_serial() or connect_tcp() first")
    return _client


def connect_serial(port: str, baud: int = 115200) -> ClearAiClient:
    global _client
    _client = ClearAiClient()
    _client.connect_serial(port, baud=baud)
    return _client


def connect_tcp(host: str, port: int = 9100) -> ClearAiClient:
    global _client
    _client = ClearAiClient()
    _client.connect_tcp(host, port=port)
    return _client


def set_client(client: ClearAiClient) -> None:
    global _client
    _client = client


def get_capabilities() -> Any:
    return _c().call("get_capabilities")


def get_status() -> Any:
    return _c().call("get_status")


def get_pose() -> Any:
    return _c().call("get_pose")


def get_config() -> Any:
    return _c().call("get_config")


def enable() -> Any:
    return _c().call("enable")


def disable() -> Any:
    return _c().call("disable")


def clear_alerts() -> Any:
    return _c().call("clear_alerts")


def stop() -> Any:
    return _c().call("stop")


def estop() -> Any:
    return _c().call("estop")


def wait_idle(timeout_ms: int = 60000) -> Any:
    timeout_s = max(1.0, timeout_ms / 1000.0 + 2.0)
    return _c().call("wait_idle", {"timeout_ms": timeout_ms}, timeout=timeout_s)


def configure(**kwargs: Any) -> Any:
    return _c().call("configure", kwargs)


def reset_config() -> Any:
    return _c().call("reset_config")


def set_units(units: str) -> Any:
    return _c().call("set_units", {"units": units})


def set_units_a(units: str) -> Any:
    return _c().call("set_units_a", {"units": units})


def set_mode(mode: str) -> Any:
    return _c().call("set_mode", {"mode": mode})


def set_test_mode(enabled: bool = True) -> Any:
    return _c().call("set_test_mode", {"enabled": enabled})


def set_work_origin(**kwargs: Any) -> Any:
    return _c().call("set_work_origin", kwargs)


def move_linear(
    x: Optional[float] = None,
    y: Optional[float] = None,
    z: Optional[float] = None,
    a: Optional[float] = None,
    feed: Optional[float] = None,
    rapid: Optional[bool] = None,
) -> Any:
    params: dict[str, Any] = {}
    if x is not None:
        params["x"] = x
    if y is not None:
        params["y"] = y
    if z is not None:
        params["z"] = z
    if a is not None:
        params["a"] = a
    if feed is not None:
        params["feed"] = feed
    if rapid is not None:
        params["rapid"] = rapid
    return _c().call("move_linear", params)


def move_arc(
    x: Optional[float] = None,
    y: Optional[float] = None,
    i: float = 0.0,
    j: float = 0.0,
    clockwise: bool = False,
    feed: Optional[float] = None,
) -> Any:
    params: dict[str, Any] = {"i": i, "j": j, "clockwise": clockwise}
    if x is not None:
        params["x"] = x
    if y is not None:
        params["y"] = y
    if feed is not None:
        params["feed"] = feed
    return _c().call("move_arc", params)


def jog(
    x: Optional[float] = None,
    y: Optional[float] = None,
    z: Optional[float] = None,
    a: Optional[float] = None,
    feed: Optional[float] = None,
) -> Any:
    params: dict[str, Any] = {}
    if x is not None:
        params["x"] = x
    if y is not None:
        params["y"] = y
    if z is not None:
        params["z"] = z
    if a is not None:
        params["a"] = a
    if feed is not None:
        params["feed"] = feed
    return _c().call("jog", params)


def dwell(seconds: float) -> Any:
    timeout_s = max(1.0, seconds + 2.0)
    return _c().call("dwell", {"seconds": seconds}, timeout=timeout_s)


def read_inputs(pin: Optional[int] = None) -> Any:
    params: dict[str, Any] = {}
    if pin is not None:
        params["pin"] = pin
    return _c().call("read_inputs", params)


def write_output(pin: int, state: bool) -> Any:
    return _c().call("write_output", {"pin": pin, "state": state})


def read_analog(pin: Optional[int] = None) -> Any:
    """Read analog voltage (volts) and raw ADC for A-9..A-12."""
    params: dict[str, Any] = {}
    if pin is not None:
        params["pin"] = pin
    return _c().call("read_analog", params)


def write_analog(pin: int = 0, value: Optional[int] = None, microamps: Optional[int] = None) -> Any:
    """Drive IO-0 analog current output. Pass raw value (0-2047) or microamps."""
    params: dict[str, Any] = {"pin": pin}
    if microamps is not None:
        params["microamps"] = microamps
    elif value is not None:
        params["value"] = value
    return _c().call("write_analog", params)


def write_pwm(pin: int, duty: int) -> Any:
    """Drive IO-0..IO-5 as PWM output (duty 0-255). Frequency is fixed."""
    return _c().call("write_pwm", {"pin": pin, "duty": duty})


def subscribe_inputs(pins: list[int], debounce_ms: Optional[int] = None) -> Any:
    """Opt-in to edge notifications for the given pins (0-12). Events arrive on
    the telemetry stream (port 9101) as input_changed notifications."""
    params: dict[str, Any] = {"pins": list(pins)}
    if debounce_ms is not None:
        params["debounce_ms"] = debounce_ms
    return _c().call("subscribe_inputs", params)


def unsubscribe_inputs() -> Any:
    """Stop input edge notifications."""
    return _c().call("unsubscribe_inputs", {})


def configure_network(
    mode: Optional[str] = None,
    ip_address: Optional[str] = None,
    netmask: Optional[str] = None,
    gateway: Optional[str] = None,
) -> Any:
    """Configure network IP mode (dhcp/static) and static IP/netmask/gateway.
    Persisted to NVM; applies on next restart()."""
    params: dict[str, Any] = {}
    if mode is not None:
        params["mode"] = mode
    if ip_address is not None:
        params["ip_address"] = ip_address
    if netmask is not None:
        params["netmask"] = netmask
    if gateway is not None:
        params["gateway"] = gateway
    return _c().call("configure_network", params)


def restart() -> Any:
    """Reset the ClearCore board to apply pending network config."""
    return _c().call("restart", {}, timeout=5.0)


def jog_velocity(
    x: Optional[float] = None,
    y: Optional[float] = None,
    z: Optional[float] = None,
    a: Optional[float] = None,
) -> Any:
    """Start a continuous per-axis velocity jog (user units/sec, signed). Stop with jog_stop or stop."""
    params: dict[str, Any] = {}
    if x is not None:
        params["x"] = x
    if y is not None:
        params["y"] = y
    if z is not None:
        params["z"] = z
    if a is not None:
        params["a"] = a
    return _c().call("jog_velocity", params)


def jog_stop() -> Any:
    """Decelerate-stop any active jog_velocity motion."""
    return _c().call("jog_stop", {})


def move_batch(moves: list[dict[str, Any]]) -> Any:
    """Queue a batch of moves (list of linear/arc/dwell param dicts) in one round-trip."""
    return _c().call("move_batch", {"moves": moves}, timeout=10.0)


def get_log() -> Any:
    """Return the recent motion log (most-recent first)."""
    return _c().call("get_log")


def clear_log() -> Any:
    """Clear the motion log ring buffer."""
    return _c().call("clear_log", {})


def queue_status() -> Any:
    return _c().call("queue_status")


def queue_clear() -> Any:
    return _c().call("queue_clear")


def home(
    axis: str,
    dir: str,
    feed: Optional[float] = None,
    seek: Optional[float] = None,
    backoff: Optional[float] = None,
    zero: Optional[bool] = None,
    timeout_ms: Optional[int] = None,
) -> Any:
    params: dict[str, Any] = {"axis": axis, "dir": dir}
    if feed is not None:
        params["feed"] = feed
    if seek is not None:
        params["seek"] = seek
    if backoff is not None:
        params["backoff"] = backoff
    if zero is not None:
        params["zero"] = zero
    if timeout_ms is not None:
        params["timeout_ms"] = timeout_ms
    timeout = max(1.0, (timeout_ms or 30000) / 1000.0 + 2.0)
    return _c().call("home", params, timeout=timeout)


def probe(
    axis: str,
    dir: str,
    pin: int,
    feed: Optional[float] = None,
    seek: Optional[float] = None,
    backoff: Optional[float] = None,
    zero: Optional[bool] = None,
    active: Optional[str] = None,
    timeout_ms: Optional[int] = None,
) -> Any:
    params: dict[str, Any] = {"axis": axis, "dir": dir, "pin": pin}
    if feed is not None:
        params["feed"] = feed
    if seek is not None:
        params["seek"] = seek
    if backoff is not None:
        params["backoff"] = backoff
    if zero is not None:
        params["zero"] = zero
    if active is not None:
        params["active"] = active
    if timeout_ms is not None:
        params["timeout_ms"] = timeout_ms
    timeout = max(1.0, (timeout_ms or 30000) / 1000.0 + 2.0)
    return _c().call("probe", params, timeout=timeout)


def keepalive() -> Any:
    """Reset the host watchdog timer and clear any watchdog trip latch."""
    return _c().call("keepalive", {}, timeout=5.0)


def configure_safety(
    watchdog_ms: Optional[int] = None,
    vel_x: Optional[int] = None,
    vel_y: Optional[int] = None,
    vel_z: Optional[int] = None,
    vel_a: Optional[int] = None,
    accel_x: Optional[int] = None,
    accel_y: Optional[int] = None,
    accel_z: Optional[int] = None,
    accel_a: Optional[int] = None,
    decel_x: Optional[int] = None,
    decel_y: Optional[int] = None,
    decel_z: Optional[int] = None,
    decel_a: Optional[int] = None,
    out_power_on_0: Optional[int] = None,
    out_power_on_1: Optional[int] = None,
    out_power_on_2: Optional[int] = None,
    out_power_on_3: Optional[int] = None,
    out_power_on_4: Optional[int] = None,
    out_power_on_5: Optional[int] = None,
) -> Any:
    """Configure safety/dynamics overrides. Per-axis values of 0 inherit the
    global vel/accel/decel. watchdog_ms of 0 disables the watchdog.
    out_power_on_<n> sets the boot state for IO-n (0/1 set, 255 don't care)."""
    params: dict[str, Any] = {}
    if watchdog_ms is not None:
        params["watchdog_ms"] = watchdog_ms
    for name, val in (
        ("vel_x", vel_x), ("vel_y", vel_y), ("vel_z", vel_z), ("vel_a", vel_a),
        ("accel_x", accel_x), ("accel_y", accel_y), ("accel_z", accel_z), ("accel_a", accel_a),
        ("decel_x", decel_x), ("decel_y", decel_y), ("decel_z", decel_z), ("decel_a", decel_a),
        ("out_power_on_0", out_power_on_0), ("out_power_on_1", out_power_on_1),
        ("out_power_on_2", out_power_on_2), ("out_power_on_3", out_power_on_3),
        ("out_power_on_4", out_power_on_4), ("out_power_on_5", out_power_on_5),
    ):
        if val is not None:
            params[name] = val
    return _c().call("configure", params, timeout=10.0)
