# ClearAI MCP server (`clearai-mcp`)

Official **Go** MCP SDK server that exposes ClearAI firmware primitives as tools for Cursor, LM Studio, and other MCP hosts.

| Item | Value |
|------|--------|
| Path | `ClearAI_Controller/mcp/` |
| Binary | `clearai-mcp.exe` |
| SDK | `github.com/modelcontextprotocol/go-sdk` v1.6.1 |
| Transport to host | stdio (newline-delimited JSON) |
| Transport to board | UDP discover **9102**, TCP control **9100** |

The leftover Python MCP under `python/clearai/` is **not** the recommended path for Cursor/LM Studio.

## Build

```powershell
cd D:\CCDev\EnhancedClearCoreLibrary\ClearAI_Controller\mcp
go build -o clearai-mcp.exe .
```

## Environment

| Variable | Default / notes |
|----------|-----------------|
| `CLEARAI_HOST` | If set, skip discover and dial this IP |
| `CLEARAI_TCP_PORT` | `9100` |
| `CLEARAI_FALLBACK_IP` | Used when discover fails (e.g. `192.168.0.109`) |

Discover also probes broadcast destinations including `172.16.82.255`. Prefer pinning `CLEARAI_HOST` on multi-subnet LANs.

## Important: one control client

Firmware accepts **one** TCP control connection on 9100. If Cursor and LM Studio both run `clearai-mcp`, the second client will see I/O timeouts / connection resets. Enable clearai in **one** host at a time.

## Tools

All tools mirror firmware methods unless noted. Motion tools are non-blocking; always follow with `wait_idle`.

| Tool | Description |
|------|-------------|
| `get_capabilities` | Protocol version, axes, units, mode, method list, ports |
| `get_status` | Enable/moving/HLFB/estop/test_mode/alerts/queue/pose |
| `get_pose` | Work coordinates x,y,z,a |
| `enable` / `disable` | Enable or abrupt disable |
| `clear_alerts` | Clear motor alert flags |
| `stop` | Decelerate; motors stay enabled |
| `estop` | Immediate stop + disable |
| `wait_idle` | Block until idle or `timeout_ms` |
| `configure` | Mechanics, `axis_mask`, vel/accel/decel, optional `test_mode` |
| `set_test_mode` | Bench gate bypass |
| `set_units` | `mm` or `inch` |
| `set_mode` | `abs` or `rel` |
| `set_work_origin` | G92-style work offset |
| `move_linear` | Queue linear move |
| `move_arc` | Queue XY arc (i,j from start) |
| `jog` | Relative move (ignores abs/rel) |
| `dwell` | Wait seconds (max 600) |
| `discover` | UDP discover; returns host/ports (does not move) |

### Feed units

`feed` is **mm/min** (or inch/min) in the active linear unit. Example: 7500 mm/min ≈ 1500 rpm at 5 mm pitch / 800 ppr.

## Example Cursor / LM Studio entry

```json
"clearai": {
  "command": "D:/CCDev/EnhancedClearCoreLibrary/ClearAI_Controller/mcp/clearai-mcp.exe",
  "args": [],
  "env": {
    "CLEARAI_HOST": "172.16.82.113",
    "CLEARAI_TCP_PORT": "9100",
    "CLEARAI_FALLBACK_IP": "192.168.0.109"
  }
}
```

## Suggested agent sequence

```
get_status → (configure / set_test_mode if needed) → enable →
move_linear | jog → wait_idle → get_pose
```

Never invent G-code. Never open COM serial from MCP while NetBurner or another tool holds the port.

## Related

- [MCP_LOCORIX.md](MCP_LOCORIX.md)  
- [AI_CLIENTS.md](AI_CLIENTS.md)  
- [PROTOCOL.md](../PROTOCOL.md)
