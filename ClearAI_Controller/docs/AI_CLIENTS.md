# AI clients: Cursor and LM Studio

Both hosts use Cursor-compatible `mcp.json` and stdio MCP servers.

## Recommended models

| Use | Model guidance |
|-----|----------------|
| Motor + Detect tool loops | **Qwen2.5-7B-Instruct** or **14B-Instruct** (GGUF in LM Studio) |
| Cloud / Cursor | Haiku, GPT-4.1-mini, or similar tool-capable small models |
| Avoid for live motion | Nano/tiny 4B chat models that print fake tool XML |

System prompt sketch:

```text
You control ClearAI motors and LocoRix Detect via MCP tools only.
After every move_linear or jog, call wait_idle.
For hole position, call locorix get_cx or plan_move_x — never invent CX.
Do not emit G-code. Do not print raw tool XML; use real tool calls.
Units are mm unless told otherwise.
```

## Cursor

User MCP config is typically `%USERPROFILE%\.cursor\mcp.json`.

Project `.cursor/mcp.json` in this repo may be empty (`{"mcpServers":{}}`) so only the User config owns TCP 9100.

Example User servers:

```json
{
  "mcpServers": {
    "clearai": {
      "command": "D:/CCDev/EnhancedClearCoreLibrary/ClearAI_Controller/mcp/clearai-mcp.exe",
      "args": [],
      "env": {
        "CLEARAI_HOST": "172.16.82.113",
        "CLEARAI_TCP_PORT": "9100",
        "CLEARAI_FALLBACK_IP": "192.168.0.109"
      }
    },
    "locorix": {
      "command": "D:/CCDev/EnhancedClearCoreLibrary/ClearAI_Controller/locorix-mcp/locorix-mcp.exe",
      "args": [],
      "env": {
        "LOCORIX_BASE_URL": "http://172.16.82.121:8080",
        "LOCORIX_CAMERA_ID": "223400130",
        "LOCORIX_PRESET": "hole-locate"
      }
    }
  }
}
```

Reload / toggle the MCP servers after rebuilding `.exe` files or changing env IPs.

## LM Studio

Requires LM Studio **0.3.17+** (MCP host). Docs: [lmstudio.ai/docs/app/mcp](https://lmstudio.ai/docs/app/mcp).

1. Program tab (right sidebar) → **Install → Edit mcp.json**.  
2. Paste the same `mcpServers` entries as above (clearai and/or locorix).  
3. Save; confirm tools appear.  
4. Load a **tool-calling Instruct** model.  
5. Enable MCP/tools for the chat; approve tool calls when prompted.

### Contention with Cursor

Only **one** process may hold ClearAI TCP **9100**. If LM Studio times out on `disable` / `get_status` while Cursor still has clearai connected, disable clearai in Cursor (or quit that MCP) and retry.

LocoRix HTTP Detect does not have that single-client limit; clearai does.

## After rebuilding MCP binaries

Restart or toggle the MCP server in the host so it respawns `clearai-mcp.exe` / `locorix-mcp.exe`. Stale processes keep old behavior (e.g. forced calibration).

## Related

- [MCP_CLEARAI.md](MCP_CLEARAI.md)  
- [MCP_LOCORIX.md](MCP_LOCORIX.md)  
- [VISION_MOTION.md](VISION_MOTION.md)  
- [TROUBLESHOOTING.md](TROUBLESHOOTING.md)
