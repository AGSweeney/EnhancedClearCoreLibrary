"""Localhost JSON-RPC + binary stream stand-in for ClearCoreROS firmware."""

from __future__ import annotations

import json
import socket
import threading
import time

from clearcore_bridge.wire import FLAG_ENABLED, pack_state


class JsonRpcBoard:
    """Accepts newline JSON-RPC and replies with result objects. Records methods."""

    def __init__(self, replies: dict | None = None):
        self.methods: list[str] = []
        self.requests: list[dict] = []
        self.replies = dict(replies or {})
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._sock.bind(("127.0.0.1", 0))
        self._sock.listen(4)
        self._sock.settimeout(0.2)
        self.port = self._sock.getsockname()[1]
        self._thread = threading.Thread(target=self._accept, daemon=True)
        self._thread.start()

    def close(self) -> None:
        self._stop.set()
        try:
            self._sock.close()
        except OSError:
            pass
        self._thread.join(timeout=2.0)

    def _accept(self) -> None:
        while not self._stop.is_set():
            try:
                conn, _ = self._sock.accept()
            except (TimeoutError, OSError):
                continue
            threading.Thread(target=self._serve, args=(conn,), daemon=True).start()

    def _serve(self, conn: socket.socket) -> None:
        conn.settimeout(0.5)
        buf = b""
        try:
            while not self._stop.is_set():
                try:
                    chunk = conn.recv(4096)
                except TimeoutError:
                    continue
                except OSError:
                    break
                if not chunk:
                    break
                buf += chunk
                while b"\n" in buf:
                    line, buf = buf.split(b"\n", 1)
                    if not line.strip():
                        continue
                    req = json.loads(line.decode("utf-8"))
                    method = str(req.get("method", ""))
                    with self._lock:
                        self.methods.append(method)
                        self.requests.append(req)
                    result = self.replies.get(method, {"ok": True})
                    if callable(result):
                        result = result(req)
                    reply = {
                        "jsonrpc": "2.0",
                        "id": req.get("id", 1),
                        "result": result,
                    }
                    conn.sendall((json.dumps(reply) + "\n").encode("utf-8"))
        finally:
            try:
                conn.close()
            except OSError:
                pass


class StreamBoard:
    """Accepts TCP clients and sends packed state frames until closed."""

    def __init__(self, flags: int = FLAG_ENABLED):
        self.flags = flags
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._drop = threading.Event()
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._sock.bind(("127.0.0.1", 0))
        self._sock.listen(4)
        self._sock.settimeout(0.2)
        self.port = self._sock.getsockname()[1]
        self._conns: list[socket.socket] = []
        self._thread = threading.Thread(target=self._accept, daemon=True)
        self._thread.start()

    def set_flags(self, flags: int) -> None:
        with self._lock:
            self.flags = flags

    def drop_clients(self) -> None:
        self._drop.set()
        with self._lock:
            conns = list(self._conns)
        for conn in conns:
            try:
                conn.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
            try:
                conn.close()
            except OSError:
                pass

    def close(self) -> None:
        self._stop.set()
        self.drop_clients()
        try:
            self._sock.close()
        except OSError:
            pass
        self._thread.join(timeout=2.0)

    def _accept(self) -> None:
        while not self._stop.is_set():
            try:
                conn, _ = self._sock.accept()
            except (TimeoutError, OSError):
                continue
            self._drop.clear()
            with self._lock:
                self._conns.append(conn)
            threading.Thread(target=self._serve, args=(conn,), daemon=True).start()

    def _serve(self, conn: socket.socket) -> None:
        conn.settimeout(0.2)
        try:
            while not self._stop.is_set() and not self._drop.is_set():
                with self._lock:
                    flags = self.flags
                try:
                    conn.sendall(pack_state(flags=flags, position=(0.01, 0.0, 0.0, 0.0)))
                except OSError:
                    break
                try:
                    conn.recv(256)
                except TimeoutError:
                    pass
                except OSError:
                    break
                time.sleep(0.02)
        finally:
            with self._lock:
                if conn in self._conns:
                    self._conns.remove(conn)
            try:
                conn.close()
            except OSError:
                pass
