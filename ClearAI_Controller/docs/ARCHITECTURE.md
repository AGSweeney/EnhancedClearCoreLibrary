# ClearAI architecture

**Experimental.** Not certified for production or safety-critical use.

ClearAI lets a host (CLI, MCP, or LLM) command ClearPath servos on a Teknic ClearCore using **named motion primitives** over JSON-RPC Lines — not G-code.

## System diagram

```
┌─────────────────────┐     stdio MCP      ┌──────────────────┐
│ Cursor / LM Studio  │◄──────────────────►│ clearai-mcp.exe  │
│ (tool-calling LLM)  │                    │ locorix-mcp.exe  │
└─────────────────────┘                    └────────┬─────────┘
                                                    │
                          TCP 9100 JSONL            │ HTTP POST
                          ┌─────────────────────────┘
                          ▼
                 ┌─────────────────┐         ┌──────────────────────┐
                 │ ClearCore board │         │ Jetson UI / LocoRix  │
                 │ ClearAI_Firmware│         │ :8080 /api/lrx/run   │
                 │ M0..M3 ClearPath│         │ Helios ToF Detect    │
                 └─────────────────┘         └──────────────────────┘
```

## Layers

| Layer | Path | Role |
|-------|------|------|
| Firmware | `ClearCoreFirmware/` | Parse JSON-RPC, gate safety, queue motion, report pose |
| Protocol | `PROTOCOL.md` | Wire format and primitive definitions (source of truth) |
| Python host | `python/clearai/` | CLI + TCP/serial client; optional legacy Python MCP |
| ClearAI MCP | `mcp/` | Official Go MCP SDK → UDP discover + TCP 9100 |
| LocoRix MCP | `locorix-mcp/` | Go MCP → Helios Detect structured symbols (CX, etc.) |

## Design principles

1. **Code as Policies** — the model calls primitives (`move_linear`, `wait_idle`); firmware executes them.
2. **One control session** — only one TCP client may hold port **9100** at a time.
3. **Pose from commanded steps** — `get_pose` / status work coordinates come from each motor’s `PositionRefCommanded()`, not the unused XY planner alone.
4. **Vision is numbers only** — LocoRix MCP never estimates pose from images; it reads `result.symbols["holes.0.cx"]` and related fields.
5. **ClearAI ≠ ClearCNC** — G-code stays in `ClearCNC_Controller`. Ports must not collide (ClearCNC 8888/8889/10040).

## Ports

| Port | Protocol | Purpose |
|------|----------|---------|
| USB CDC 115200 | JSONL | Same RPC as Ethernet |
| **9100** | TCP JSONL | Control (single client) |
| **9101** | TCP JSONL | Telemetry notifications (~50 ms) |
| **9102** | UDP | Discovery request `CLEARAI_DISCOVER?` |

## Axis map

| Axis | Connector | Notes |
|------|-----------|--------|
| X | M0 | Linear (mm/inch) |
| Y | M1 | Linear |
| Z | M2 | Linear |
| A | M3 | Rotary **degrees** always |

`axis_mask` bits: bit0=X, bit1=Y, bit2=Z, bit3=A. Default `0x3` (XY). One-motor bench uses `1` (X only).

## Documentation map

| Doc | Contents |
|-----|----------|
| [FIRMWARE.md](FIRMWARE.md) | Build, flash, modules, config, ClearPath MSP |
| [../PROTOCOL.md](../PROTOCOL.md) | Full RPC primitive reference |
| [HOST_PYTHON.md](HOST_PYTHON.md) | CLI and Python client |
| [MCP_CLEARAI.md](MCP_CLEARAI.md) | clearai-mcp tools and env |
| [MCP_LOCORIX.md](MCP_LOCORIX.md) | locorix-mcp Detect tools |
| [AI_CLIENTS.md](AI_CLIENTS.md) | Cursor and LM Studio setup |
| [VISION_MOTION.md](VISION_MOTION.md) | Detect → plan → move loop |
| [TROUBLESHOOTING.md](TROUBLESHOOTING.md) | Common failures |
