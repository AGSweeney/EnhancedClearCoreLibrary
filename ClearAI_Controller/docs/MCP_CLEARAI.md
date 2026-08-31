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
| `get_status` | Enable/moving/HLFB/estop/test_mode/alerts/queue/`queue_active`/`units_a`/`hlfb_percent` (per-axis torque %)/`watchdog_ms`/`watchdog_tripped`/`limit_status`/`pwm_duty`/pose |
| `get_pose` | Work coordinates x,y,z,a (A in configured A units) |
| `get_config` | Live config + NVM validity (`axis_mask`, test_mode, mechanics, vel, `units_a`, soft limits, hw limit DIs, `vel_axis`/`accel_axis`/`decel_axis`, `watchdog_ms`, `out_power_on_state`/`out_power_on_mask`, …) |
| `enable` / `disable` | Enable or abrupt disable |
| `clear_alerts` | Clear motor alert flags |
| `stop` | Decelerate; motors stay enabled |
| `estop` | Immediate stop + disable |
| `wait_idle` | Block until idle or `timeout_ms` |
| `configure` | Mechanics, `axis_mask`, vel/accel/decel, per-axis `vel_x`/…/`decel_a` (0=inherit), soft limits (machine-coord), hardware limit DIs (`pos_lim_x`/…), `watchdog_ms`, `out_power_on_0`/…/`out_power_on_5` (0/1 set, 255 don't care), optional `test_mode` (**NVM v6**) |
| `reset_config` | Compile defaults + clear NVM (motors disabled) |
| `set_test_mode` | Bench gate bypass (**NVM**) |
| `set_units` | `mm` or `inch` |
| `set_units_a` | A-axis unit `deg` or `rev` (**NVM**) |
| `set_mode` | `abs` or `rel` |
| `set_work_origin` | G92-style work offset |
| `move_linear` | Queue linear move |
| `move_arc` | Queue XY arc (i,j from start) |
| `jog` | Relative move (ignores abs/rel) |
| `dwell` | Wait seconds (max 600) |
| `read_inputs` | Optional pin 0–12; raw digital state for onboard I/O |
| `write_output` | Drive IO-0…IO-5 high/low |
| `read_analog` | Read analog voltage (V) + raw ADC for A-9…A-12 |
| `write_analog` | Drive IO-0 analog current output (raw 0–2047 or microamps) |
| `write_pwm` | Drive IO-0…IO-5 as PWM (duty 0–255; fixed frequency) |
| `subscribe_inputs` | Opt-in edge notifications for pins 0–12 (events on telemetry 9101) |
| `unsubscribe_inputs` | Stop input edge notifications |
| `queue_status` | Pending coordinated queue depth + active flag |
| `queue_clear` | Decelerate active move + drop pending queued segments |
| `home` | Seek configured hw limit for `axis`/`dir`, stop, optional backoff + zero |
| `probe` | Move until probe DI triggers; report touch position, optional zero |
| `keepalive` | Reset host watchdog timer + clear watchdog trip latch |
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
