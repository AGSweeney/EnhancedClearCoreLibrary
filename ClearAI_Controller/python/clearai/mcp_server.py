"""MCP stdio server over Ethernet: UDP discover, then TCP JSON-RPC on 9100."""

from __future__ import annotations

import json
import os
import sys
from pathlib import Path
from typing import Any, Optional

from .client import ClearAiClient, ClearAiError, discover
from .primitives import set_client

_client: Optional[ClearAiClient] = None
_connect_kwargs: dict[str, Any] = {}


def _read_exact(n: int) -> Optional[bytes]:
    chunks = bytearray()
    while len(chunks) < n:
        piece = sys.stdin.buffer.read(n - len(chunks))
        if not piece:
            return None
        chunks.extend(piece)
    return bytes(chunks)


def _read_message() -> Optional[dict[str, Any]]:
    headers: dict[str, str] = {}
    while True:
        line = sys.stdin.buffer.readline()
        if not line:
            return None
        if line in (b"\r\n", b"\n"):
            break
        text = line.decode("utf-8", errors="replace").strip()
        if ":" in text:
            k, v = text.split(":", 1)
            headers[k.strip().lower()] = v.strip()
    try:
        length = int(headers.get("content-length", "0"))
    except ValueError:
        return {"jsonrpc": "2.0", "method": "_skip"}
    if length <= 0:
        return {"jsonrpc": "2.0", "method": "_skip"}
    body = _read_exact(length)
    if body is None:
        return None
    try:
        return json.loads(body.decode("utf-8"))
    except json.JSONDecodeError:
        return {"jsonrpc": "2.0", "method": "_skip"}


def _write_message(obj: dict[str, Any]) -> None:
    raw = json.dumps(obj, separators=(",", ":")).encode("utf-8")
    sys.stdout.buffer.write(f"Content-Length: {len(raw)}\r\n\r\n".encode("ascii"))
    sys.stdout.buffer.write(raw)
    sys.stdout.buffer.flush()


def _tools() -> list[dict[str, Any]]:
    path = Path(__file__).with_name("tools.json")
    openai = json.loads(path.read_text(encoding="utf-8"))
    out = []
    for item in openai:
        fn = item.get("function", {})
        name = fn.get("name")
        if not name:
            continue
        schema = fn.get("parameters") or {"type": "object", "properties": {}}
        if "type" not in schema:
            schema = {"type": "object", "properties": schema.get("properties", {})}
        out.append(
            {
                "name": name,
                "description": fn.get("description", ""),
                "inputSchema": schema,
            }
        )
    return out


def _ensure_client() -> ClearAiClient:
    global _client
    if _client is not None and _client.connected():
        return _client
    host = (_connect_kwargs.get("host") or os.environ.get("CLEARAI_HOST") or "").strip()
    if not host:
        host = discover(timeout=2.5) or os.environ.get("CLEARAI_FALLBACK_IP", "192.168.0.109")
    tcp_port = int(_connect_kwargs.get("tcp_port") or os.environ.get("CLEARAI_TCP_PORT", "9100"))
    client = ClearAiClient()
    client.connect_tcp(host, port=tcp_port, timeout=3.0)
    _client = client
    set_client(client)
    return client


def _call_tool(name: str, arguments: dict[str, Any]) -> Any:
    timeout = 5.0
    if name == "wait_idle":
        timeout_ms = int(arguments.get("timeout_ms", 60000))
        timeout = max(1.0, timeout_ms / 1000.0 + 2.0)
    elif name == "dwell":
        timeout = max(1.0, float(arguments.get("seconds", 0)) + 2.0)
    return _ensure_client().call(name, arguments or {}, timeout=timeout)


def _ok(req_id: Any, result: Any) -> None:
    _write_message({"jsonrpc": "2.0", "id": req_id, "result": result})


def run_mcp(args: Any) -> int:
    global _connect_kwargs
    _connect_kwargs = {
        "host": getattr(args, "host", None) or os.environ.get("CLEARAI_HOST"),
        "tcp_port": getattr(args, "tcp_port", None)
        or int(os.environ.get("CLEARAI_TCP_PORT", "9100")),
    }

    try:
        while True:
            msg = _read_message()
            if msg is None:
                return 0
            method = msg.get("method")
            req_id = msg.get("id")
            params = msg.get("params") or {}
            if method in ("_skip", None):
                continue
            if method == "initialize":
                proto = params.get("protocolVersion") or "2024-11-05"
                _ok(
                    req_id,
                    {
                        "protocolVersion": proto,
                        "capabilities": {
                            "tools": {"listChanged": False},
                            "resources": {"listChanged": False},
                            "prompts": {"listChanged": False},
                        },
                        "serverInfo": {"name": "clearai", "version": "1.0.0"},
                    },
                )
            elif method in ("notifications/initialized", "notifications/cancelled"):
                continue
            elif method == "tools/list":
                _ok(req_id, {"tools": _tools()})
            elif method == "resources/list":
                _ok(req_id, {"resources": []})
            elif method == "resources/templates/list":
                _ok(req_id, {"resourceTemplates": []})
            elif method == "prompts/list":
                _ok(req_id, {"prompts": []})
            elif method == "tools/call":
                name = params.get("name", "")
                arguments = params.get("arguments") or {}
                try:
                    result = _call_tool(name, arguments)
                    _ok(
                        req_id,
                        {
                            "content": [{"type": "text", "text": json.dumps(result)}],
                            "isError": False,
                        },
                    )
                except (ClearAiError, TimeoutError, RuntimeError, OSError) as exc:
                    _ok(
                        req_id,
                        {
                            "content": [{"type": "text", "text": str(exc)}],
                            "isError": True,
                        },
                    )
            elif method == "ping":
                _ok(req_id, {})
            elif req_id is not None:
                _write_message(
                    {
                        "jsonrpc": "2.0",
                        "id": req_id,
                        "error": {"code": -32601, "message": f"unknown method {method}"},
                    }
                )
    finally:
        if _client is not None:
            _client.close()
    return 0
