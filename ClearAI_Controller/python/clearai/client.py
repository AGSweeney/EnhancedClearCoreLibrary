"""JSON-RPC Lines client for ClearAI firmware (USB serial or TCP)."""

from __future__ import annotations

import json
import re
import socket
import time
from typing import Any, Optional, Union

try:
    import serial
except ImportError:  # pragma: no cover
    serial = None

_DISCOVER_RE = re.compile(r"IP=(\d{1,3}(?:\.\d{1,3}){3})")


class ClearAiError(RuntimeError):
    def __init__(self, code: int, message: str) -> None:
        super().__init__(f"{code}: {message}")
        self.code = code
        self.message = message


def discover(timeout: float = 2.0, udp_port: int = 9102) -> Optional[str]:
    """UDP broadcast CLEARAI_DISCOVER?; return the board's IPv4 or None."""
    req = b"CLEARAI_DISCOVER?"
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    sock.settimeout(0.4)
    try:
        sock.bind(("", 0))
        for dest in (
            "255.255.255.255",
            "<broadcast>",
            "192.168.0.255",
            "192.168.1.255",
        ):
            try:
                sock.sendto(req, (dest, udp_port))
            except OSError:
                continue
        deadline = time.time() + timeout
        while time.time() < deadline:
            try:
                data, addr = sock.recvfrom(256)
            except socket.timeout:
                continue
            text = data.decode("utf-8", errors="replace")
            match = _DISCOVER_RE.search(text)
            if match:
                return match.group(1)
            if "CLEARAI" in text:
                return addr[0]
    finally:
        sock.close()
    return None


class ClearAiClient:
    def __init__(self) -> None:
        self._serial = None
        self._sock: Optional[socket.socket] = None
        self._rx = b""
        self._next_id = 1

    def connect_serial(self, port: str, baud: int = 115200, timeout: float = 2.0) -> None:
        if serial is None:
            raise RuntimeError("pyserial is required for serial connections")
        self.close()
        self._serial = serial.Serial(
            port, baudrate=baud, timeout=timeout, dsrdtr=True, rtscts=False
        )
        time.sleep(0.15)
        try:
            self._serial.reset_input_buffer()
        except Exception:
            pass

    def connected(self) -> bool:
        if self._serial is not None:
            return bool(getattr(self._serial, "is_open", True))
        if self._sock is None:
            return False
        try:
            self._sock.getpeername()
            return True
        except OSError:
            return False

    def connect_tcp(self, host: str, port: int = 9100, timeout: float = 3.0) -> None:
        self.close()
        sock = socket.create_connection((host, port), timeout=timeout)
        sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_KEEPALIVE, 1)
        sock.settimeout(timeout)
        self._sock = sock
        self._rx = b""

    def close(self) -> None:
        if self._serial is not None:
            try:
                self._serial.close()
            except Exception:
                pass
            self._serial = None
        if self._sock is not None:
            try:
                self._sock.close()
            except Exception:
                pass
            self._sock = None
        self._rx = b""

    def _write_line(self, text: str) -> None:
        payload = (text + "\n").encode("utf-8")
        if self._serial is not None:
            self._serial.write(payload)
            self._serial.flush()
            return
        if self._sock is not None:
            self._sock.sendall(payload)
            return
        raise RuntimeError("not connected")

    def _read_line(self, timeout: Optional[float]) -> str:
        if self._serial is not None:
            old = self._serial.timeout
            if timeout is not None:
                self._serial.timeout = timeout
            try:
                raw = self._serial.readline()
            finally:
                self._serial.timeout = old
            if not raw:
                raise TimeoutError("ClearAI serial timeout")
            return raw.decode("utf-8", errors="replace").strip()
        if self._sock is None:
            raise RuntimeError("not connected")
        old = self._sock.gettimeout()
        if timeout is not None:
            self._sock.settimeout(timeout)
        try:
            while b"\n" not in self._rx:
                chunk = self._sock.recv(4096)
                if not chunk:
                    raise TimeoutError("ClearAI TCP closed")
                self._rx += chunk
            line, self._rx = self._rx.split(b"\n", 1)
            return line.decode("utf-8", errors="replace").strip()
        finally:
            self._sock.settimeout(old)

    def call(
        self,
        method: str,
        params: Optional[dict[str, Any]] = None,
        timeout: Optional[float] = 5.0,
        req_id: Optional[Union[int, str]] = None,
    ) -> Any:
        if req_id is None:
            req_id = self._next_id
            self._next_id += 1
        msg = {
            "jsonrpc": "2.0",
            "id": req_id,
            "method": method,
            "params": params or {},
        }
        self._write_line(json.dumps(msg, separators=(",", ":")))
        deadline = None if timeout is None else time.time() + timeout
        while True:
            remaining = None
            if deadline is not None:
                remaining = max(0.05, deadline - time.time())
                if time.time() > deadline:
                    raise TimeoutError(f"timed out waiting for id={req_id}")
            line = self._read_line(remaining)
            if not line:
                continue
            try:
                obj = json.loads(line)
            except json.JSONDecodeError:
                continue
            if not isinstance(obj, dict):
                continue
            if "id" not in obj:
                continue
            if obj.get("id") != req_id:
                continue
            if "error" in obj:
                err = obj["error"] or {}
                raise ClearAiError(int(err.get("code", -32000)), str(err.get("message", "error")))
            return obj.get("result")
