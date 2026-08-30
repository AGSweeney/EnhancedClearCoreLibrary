# LocoRix MCP server (`locorix-mcp`)

Go MCP server that gives an LLM access to **Helios ToF Detect** on the Jetson UI host via structured LocoRix API numbers only.

| Item | Value |
|------|--------|
| Path | `ClearAI_Controller/locorix-mcp/` |
| Binary | `locorix-mcp.exe` |
| Detect | `POST {base}/api/lrx/run` |
| Default host | `http://172.16.82.121:8080` |
| Default camera | Helios `223400130` |
| Default preset | `hole-locate` |

**Does not** estimate pose from images, point clouds, masks, or screenshots.  
**Does not** command ClearAI motors — pair with [`MCP_CLEARAI.md`](MCP_CLEARAI.md).

## Build

```powershell
cd D:\CCDev\EnhancedClearCoreLibrary\ClearAI_Controller\locorix-mcp
go mod tidy
go build -o locorix-mcp.exe .
```

Live smoke test (optional):

```powershell
$env:LOCORIX_LIVE="1"
go test -v -run TestRunDetectLive
```

## Prerequisites

1. Jetson UI host reachable on the LAN.  
2. Helios camera `223400130` **streaming** on the UI.  
3. LocoRix hole-locate assets / job available on the device.

## Environment

| Variable | Default |
|----------|---------|
| `LOCORIX_BASE_URL` | `http://172.16.82.121:8080` |
| `LOCORIX_CAMERA_ID` | `223400130` |
| `LOCORIX_PRESET` | `hole-locate` |
| `LOCORIX_TIMEOUT_MS` | `20000` |

## Detect request

```http
POST /api/lrx/run
Content-Type: application/json

{
  "cameraId": "223400130",
  "preset": "hole-locate",
  "persist": false
}
```

### Symbols used (camera-frame mm)

| Symbol | Meaning |
|--------|---------|
| `holes.count` | Number of holes |
| `holes.0.cx` | **CX** — X position (mm) |
| `holes.0.cy` | Y (mm) |
| `holes.0.cz` | Z (mm) |
| `holes.0.diameter_mm` | Diameter |
| `holes.0.score` | Score |
| `gauges.holes.pass` / `gauges_all_pass` | Gauge status |

Also reads `result.ok`, `result.message`, and `result.gauges[]`.

**CX is camera frame**, not robot/motor frame, until you apply an optional calibration.

## Tools

| Tool | Behavior |
|------|----------|
| `detect_hole` | Run Detect; return structured hole snapshot |
| `get_cx` | Run Detect; focus on `cx_mm` (+ related fields) |
| `plan_move_x` | Detect → decision JSON (below) |
| `set_calibration` | Optional store: `move_x_mm = cx_mm * scale + offset_mm` |
| `get_config` | Base URL, camera, preset, stored calibration |

### `plan_move_x` result shape

```json
{"go": true, "cx_mm": 46.54, "move_x_mm": 46.54, "reason": "ok; move_x_mm = cx_mm (camera frame)"}
```

| Field | Rule |
|-------|------|
| `go` | `false` if Detect failed, no hole, or gauges fail (when required) |
| `cx_mm` | From Detect only — never invented |
| `move_x_mm` | Default `= cx_mm`; or `cx_mm * scale + offset_mm` if calibration/args set |
| `reason` | Short status |

Calibration is **optional**. Without it, `move_x_mm` equals camera-frame CX.

## MCP host entry

```json
"locorix": {
  "command": "D:/CCDev/EnhancedClearCoreLibrary/ClearAI_Controller/locorix-mcp/locorix-mcp.exe",
  "args": [],
  "env": {
    "LOCORIX_BASE_URL": "http://172.16.82.121:8080",
    "LOCORIX_CAMERA_ID": "223400130",
    "LOCORIX_PRESET": "hole-locate"
  }
}
```

## Agent rules (for system prompts)

- Never invent CX. If Detect fails or `holes.count` is 0, say so and do not command motion.  
- Prefer gauges pass when present.  
- For motor motion, call **clearai** tools with `move_x_mm` / absolute targets — locorix only plans.  
- Do not request point clouds, masks, or screenshots for position.

## Curl reference

```bash
curl -s -X POST http://172.16.82.121:8080/api/lrx/run \
  -H "Content-Type: application/json" \
  -d '{"cameraId":"223400130","preset":"hole-locate","persist":false}'
```

## Related

- [VISION_MOTION.md](VISION_MOTION.md)  
- [MCP_CLEARAI.md](MCP_CLEARAI.md)  
- [AI_CLIENTS.md](AI_CLIENTS.md)
