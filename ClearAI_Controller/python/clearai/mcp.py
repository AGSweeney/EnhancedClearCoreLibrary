"""python -m clearai.mcp — Cursor stdio MCP over Ethernet (UDP discover + TCP 9100)."""

from __future__ import annotations

import os
from types import SimpleNamespace

from .mcp_server import run_mcp


def main() -> int:
    host = os.environ.get("CLEARAI_HOST", "").strip() or None
    return run_mcp(
        SimpleNamespace(
            host=host,
            tcp_port=int(os.environ.get("CLEARAI_TCP_PORT", "9100")),
        )
    )


if __name__ == "__main__":
    raise SystemExit(main())
