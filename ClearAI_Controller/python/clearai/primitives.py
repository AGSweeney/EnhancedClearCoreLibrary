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


def set_units(units: str) -> Any:
    return _c().call("set_units", {"units": units})


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
