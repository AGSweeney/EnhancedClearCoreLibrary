# ClearAI protocol

JSON-RPC 2.0 **subset**, one object per line (JSONL). This is the command language an LLM or host uses to drive a ClearPath XY gantry (optional Z/A) on ClearCore. It is **not** G-code and **not** the Teknic letter protocol.

Companion docs: [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md) · [docs/FIRMWARE.md](docs/FIRMWARE.md) · [docs/MCP_CLEARAI.md](docs/MCP_CLEARAI.md) · [README.md](README.md)

Inspired by [Code as Policies](https://code-as-policies.github.io/) (Liang et al., 2022) and ProgPrompt (Singh et al., 2023): the model calls named primitives; the firmware executes them.

## Transport

| Channel | Detail |
|---------|--------|
| USB CDC | 115200 baud, newline-terminated JSON |
| Control TCP | **9100** (same JSONL as USB) |
| Telemetry TCP | **9101**, ~50 ms `status` notifications |
| UDP discovery | **9102**, request `CLEARAI_DISCOVER?` |

Do not use ClearCNC ports (8888/8889/10040) on the same board/network session.

## Request / response

```json
{"jsonrpc":"2.0","id":1,"method":"move_linear","params":{"x":10,"y":20,"feed":500}}
{"jsonrpc":"2.0","id":1,"result":{"ok":true,"queued":true}}
{"jsonrpc":"2.0","id":1,"error":{"code":-32000,"message":"motor not enabled"}}
```

- `id` is **required** (number or string) so replies correlate.
- `params` is an object (not an array). Empty/`{}` is allowed.
- Max line 512 characters.
- Motion methods return immediately with `"queued":true`. Use `wait_idle` to block until complete.

Unsolicited notifications (no `id`):

```json
{"jsonrpc":"2.0","method":"status","params":{"ready":true,"hint":"call get_capabilities"}}
{"jsonrpc":"2.0","method":"status","params":{"enabled":true,"moving":false,"x":0,"y":0,"z":0,"a":0}}
{"jsonrpc":"2.0","method":"fault","params":{"message":"motion init failed"}}
```

## Error codes

| Code | Meaning |
|------|---------|
| -32700 | Parse error |
| -32600 | Invalid request (missing method/id) |
| -32601 | Method not found |
| -32602 | Invalid params |
| -32000 | Application error (not enabled, alert, estop, timeout, queue reject) |

## Safety

- `enable` is required before motion (unless `test_mode` is on; then motion auto-enables).
- Alerts and hardware estop (DI-6, same polarity as ClearCNC: mode 1 = OK when pin ON) block motion.
- `estop`, `stop`, and `disable` are accepted during `wait_idle` / `dwell`.
- `estop` stops immediately and disables motors. `stop` decelerates.
- **`set_test_mode`** / `configure` `test_mode`: bench only. Bypasses hardware DI-6, HLFB wait, alert gates, and the enable requirement. Status `estop` is forced false; `hw_estop` still reports the raw pin. Software `stop` / `estop` / `disable` still halt. Default off (`CLEARAI_TEST_MODE_DEFAULT` 0). Not for production.

## Primitives

### Session

| Method | Params | Result |
|--------|--------|--------|
| `get_capabilities` | — | protocol version, axis mask, units, mode, `test_mode`, method list, ports |
| `get_status` | — | enabled, moving, hlfb, estop, `hw_estop`, `test_mode`, alerts, queue, pose |
| `get_pose` | — | work coordinates in active units |
| `enable` | — | `{ok:true}` after enable request (short HLFB wait) |
| `disable` | — | abrupt stop + disable |
| `clear_alerts` | — | clear motor alerts |
| `stop` | — | decelerate |
| `estop` | — | abrupt stop + disable |
| `wait_idle` | `timeout_ms` (default 60000) | `{ok:true}` or timeout/estop error |

### Configuration

| Method | Params |
|--------|--------|
| `configure` | `steps_per_rev_x/y/z/a`, `pitch_x/y/z` (mm), `gear_x/y/z/a`, `vel`/`accel`/`decel` (steps/s, steps/s²), `axis_mask` (bit0=X … bit3=A), `estop_di6` (0=off, 1=default, 2=inverted), `test_mode` (bool). Mechanical fields require motors **disabled**; `test_mode` may change while enabled. |
| `set_test_mode` | `enabled` or `test_mode` (bool). Omitted params turn **on**. Allowed during `wait_idle`. |
| `set_units` | `units`: `"mm"` or `"inch"` |
| `set_mode` | `mode`: `"abs"` or `"rel"` |
| `set_work_origin` | `x`,`y`,`z`,`a` — new work coordinates at the current machine pose (G92-style offset). Omitted axes unchanged. Empty object zeros all axes. |

Default mechanics: 800 steps/rev, 5 mm pitch, XY enabled (`axis_mask` 3), mm, absolute.

One-motor bench: `configure` `axis_mask` **1** (X/M0 only) so enable/HLFB/alerts ignore the empty connector. If the partner XY motor is missing or in alert, `move_linear` / `jog` fall back to independent `Move` instead of coordinated `QueueLinear` (that path was rejecting with `xy queue rejected`).

Axis A is rotary: values are **degrees** regardless of `set_units`.

### Motion

Feed is **units per minute** in the active linear unit (mm/min or inch/min).

| Method | Params |
|--------|--------|
| `move_linear` | `x`,`y` optional `z`,`a`, `feed`, `rapid` (bool). XY uses coordinated `QueueLinear`. |
| `move_arc` | `x`,`y`,`i`,`j`,`clockwise`,`feed`. I/J relative to start (G-code convention). XY only; `z`/`a` rejected. |
| `jog` | relative `x`,`y`,`z`,`a`,`feed` (ignores abs/rel mode) |
| `dwell` | `seconds` (max 600), interruptible by estop |

## Examples

Enable and make an absolute 10 mm X move:

```json
{"jsonrpc":"2.0","id":1,"method":"enable","params":{}}
{"jsonrpc":"2.0","id":2,"method":"move_linear","params":{"x":10,"y":0,"feed":500}}
{"jsonrpc":"2.0","id":3,"method":"wait_idle","params":{"timeout_ms":15000}}
{"jsonrpc":"2.0","id":4,"method":"get_pose","params":{}}
```

Relative jog then stop:

```json
{"jsonrpc":"2.0","id":5,"method":"jog","params":{"x":1,"y":0,"feed":200}}
{"jsonrpc":"2.0","id":6,"method":"stop","params":{}}
```

Clockwise XY arc (center offset I=5, J=0 mm):

```json
{"jsonrpc":"2.0","id":7,"method":"move_arc","params":{"x":10,"y":0,"i":5,"j":0,"clockwise":true,"feed":400}}
```

Bench: bypass hardware interlocks (after flashing firmware that includes `set_test_mode`):

```json
{"jsonrpc":"2.0","id":8,"method":"set_test_mode","params":{"enabled":true}}
{"jsonrpc":"2.0","id":9,"method":"get_status","params":{}}
```

## Code as Policies (host Python)

The host exposes the same names as functions. An LLM can be few-shot prompted as:

```python
# Move the gantry 10 mm in X at 500 mm/min, then report pose.
enable()
move_linear(x=10, y=0, feed=500)
wait_idle(timeout_ms=15000)
print(get_pose())
```

See `python/clearai/primitives.py` and `python/clearai/tools.json`.
