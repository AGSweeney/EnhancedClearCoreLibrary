# Vision → motion loop

End-to-end pattern: Helios Detect finds a hole; a local LLM (or script) uses CX to command ClearAI X.

```
Helios streaming → locorix plan_move_x / get_cx
                 → clearai move_linear / jog → wait_idle → get_pose
```

## Preconditions

1. ClearAI firmware flashed; board on LAN (example `172.16.82.113`).  
2. One-motor or gantry configured (`axis_mask`, test mode if needed), motors enabled.  
3. Jetson UI at `172.16.82.121:8080` with Helios `223400130` streaming.  
4. Both MCP servers configured in the AI host ([AI_CLIENTS.md](AI_CLIENTS.md)).  
5. Only one clearai MCP connected to TCP 9100.

## Recommended tool sequence

1. `clearai.get_status` — confirm enabled / idle.  
2. `locorix.plan_move_x` (or `get_cx`) — read Detect; check `go` and `cx_mm`.  
3. If `go` is false — stop; report `reason`. Do **not** invent CX.  
4. `clearai.move_linear` with `x` = desired target (e.g. `move_x_mm`, or an absolute home).  
5. `clearai.wait_idle`.  
6. `clearai.get_pose` — verify.

### Example intent: “move X to the hole CX”

```text
plan_move_x  →  {"go":true,"cx_mm":46.54,"move_x_mm":46.54,"reason":"..."}
move_linear x=46.54 feed=...
wait_idle
get_pose
```

### Example intent: “move X to 0”

Use **clearai only** (`move_linear` x=0). Do not invent a locorix JSON blob.

## Calibration (optional)

By default `move_x_mm = cx_mm` (camera frame). When you have a known offset:

```text
set_calibration offset_mm=<...> scale=<...>
# or pass offset_mm / scale on plan_move_x
```

Formula: `move_x_mm = cx_mm * scale + offset_mm`.

## Safety notes

- CX without calibration is **not** guaranteed to be motor millimetres.  
- Keep a physical estop; test mode bypasses board gates.  
- Prefer models that actually invoke tools (not 4B nano chat models that emit fake tool XML).

## Related

- [MCP_LOCORIX.md](MCP_LOCORIX.md)  
- [MCP_CLEARAI.md](MCP_CLEARAI.md)  
- [FIRMWARE.md](FIRMWARE.md)
