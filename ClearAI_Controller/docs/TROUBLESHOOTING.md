# Troubleshooting

## ClearAI TCP timeouts / “I/O timeout” on enable/disable

**Cause:** Another MCP or CLI already holds control port **9100**, or a stale TCP session after flash/reboot.

**Fix:**

1. Disable clearai MCP in the other host (Cursor vs LM Studio).  
2. Wait a few seconds after flashing for Ethernet to come up.  
3. Confirm `CLEARAI_HOST` matches the board’s current DHCP address.  
4. Retry `get_status`.

## `xy queue rejected` / empty M1 alerts

**Cause:** Default `axis_mask` 3 expects Y; missing M1 faults coordinated XY.

**Fix:** `configure` with `axis_mask: 1` (X only). Current firmware also falls back to independent `Move` when the partner is unhealthy — still set mask 1 for clean status.

## Pose always `x: 0` after moves

**Cause:** Old firmware tracked coordinated planner XY (stays 0 on independent moves).

**Fix:** Flash firmware that uses `PositionRefCommanded()` for `MachineSteps` (see [FIRMWARE.md](FIRMWARE.md)).

## HLFB false while moving / after ASG torque PWM

Expected with ASG-Position w/Measured Torque: HLFB is not continuously “asserted” the same way as a simple digital HLFB during torque PWM. Test mode skips HLFB gates. Idle + asserted is the usual quiet state.

## LocoRix Detect fails / no CX

1. Confirm Helios `223400130` is streaming on the Jetson UI.  
2. Curl `/api/lrx/run` and check `result.ok` and `holes.count`.  
3. Check `LOCORIX_BASE_URL` / camera id env on the MCP.  
4. Never invent CX in the LLM reply.

## LM Studio invents JSON / prints `</tool_call>`

**Cause:** Model too small or not tool-tuned (e.g. Nemotron Nano 4B).

**Fix:** Switch to Qwen2.5-7B/14B Instruct; enable real MCP tools; strengthen system prompt ([AI_CLIENTS.md](AI_CLIENTS.md)).

## MCP tools missing after rebuild

Restart/toggle the MCP server so the host respawns the new `.exe`.

## Discover returns nothing

UDP broadcast may not cross subnets. Set `CLEARAI_HOST` explicitly.

## Serial COM open conflicts

MCP should use Ethernet. Close NetBurner/other serial sessions on the ClearCore CDC port before flashing or exclusive USB use.

## `alert_reg` non-zero but `alerts: false`

With `axis_mask` 1, empty axes may still contribute raw register bits historically; motion gating uses configured axes / test mode. Use `clear_alerts` after intentional stops if needed.
