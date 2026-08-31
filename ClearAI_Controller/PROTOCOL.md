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
| -32000 | Application error (not enabled, alert, estop, timeout, queue reject, out of limits) |

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
| `get_capabilities` | — | protocol version, axis mask, units, mode, `test_mode`, `nvm`, method list, ports |
| `get_status` | — | enabled, moving, hlfb, estop, `hw_estop`, `test_mode`, alerts, queue, `queue_active`, `units_a`, `hlfb_percent` (per-axis % of peak torque, or null if unknown), `watchdog_ms`, `watchdog_tripped`, `limit_status` (`{tripped,axis,dir}` or `{tripped:false}`), pose |
| `get_pose` | — | work coordinates in active units (A in configured A units) |
| `get_config` | — | live config + `nvm` / `nvm_valid` (axis_mask, test_mode, mechanics, vel/accel/decel, `units_a`, `limit_flags`, `limits_min`/`limits_max`, `pos_lim_di`/`neg_lim_di`, `vel_axis`/`accel_axis`/`decel_axis`, `watchdog_ms`, …) |
| `enable` | — | `{ok:true}` after enable request (short HLFB wait) |
| `disable` | — | abrupt stop + disable |
| `clear_alerts` | — | clear motor alerts |
| `stop` | — | decelerate |
| `estop` | — | abrupt stop + disable |
| `wait_idle` | `timeout_ms` (default 60000) | `{ok:true}` or timeout/estop error |
| `read_inputs` | optional `pin` (0–12); omit for all onboard pins | `{pins:[{pin,state,mode},…]}` — raw digital state; `mode` is `in`, `out`, or `other` |
| `write_output` | `pin` (0–5 only), `state` (bool or 0/1) | `{ok:true}` — sets **OUTPUT_DIGITAL** then drives the pin |
| `queue_status` | — | `{queue:N, active:bool}` — pending coordinated segments and active flag. Allowed during `wait_idle`. |
| `queue_clear` | — | `{ok:true}` — decelerate the active move to a stop and drop pending queued segments. Motors stay enabled. Accepted during `wait_idle`/`dwell` (interrupts the wait). |
| `home` | `axis` (`x`/`y`/`z`/`a`), `dir` (`pos`/`neg`), optional `feed`, `seek` (max travel, default 1000), `backoff`, `zero` (default true), `timeout_ms` (default 30000) | `{homed:true,axis,dir,pos,limit_pin}` — seek the configured hardware limit for `axis`/`dir`, decel-stop on contact, optional backoff, optional zero. Requires `pos_lim_<axis>`/`neg_lim_<axis>` DI configured. Blocking (interruptible by `estop`/`stop`/`queue_clear`). |
| `probe` | `axis`, `dir`, `pin` (DI 1–12), optional `feed`, `seek`, `backoff`, `zero` (default false), `active` (`high`/`low`, default `high`), `timeout_ms` | `{probed:true,axis,dir,pos,pin}` — move at probe feed until the probe DI triggers, stop, report touch position. Blocking (interruptible). |
| `keepalive` | — | `{ok:true}` — reset the host watchdog timer and clear the `watchdog_tripped` latch. Allowed during `wait_idle`/`dwell`/`home`/`probe`. |

### Configuration

| Method | Params |
|--------|--------|
| `configure` | `steps_per_rev_x/y/z/a`, `pitch_x/y/z` (mm), `gear_x/y/z/a`, `vel`/`accel`/`decel` (steps/s, steps/s²), `axis_mask` (bit0=X … bit3=A), `estop_di6` (0=off, 1=default, 2=inverted), `test_mode` (bool), soft limits `min_x`/`max_x`/… (work units; A in configured A units), hardware limits `pos_lim_x`/`neg_lim_x`/… (ClearCore **pin index** 0–12: **0–5** = IO-0…IO-5 configurable in/out — firmware forces input when used as a limit; **6–12** = DI-6…A-12 input-only; **0** or **255** = disabled), `clear_limits`, `clear_min_x`/`clear_max_x`/… (bool), per-axis dynamics `vel_x`/`vel_y`/`vel_z`/`vel_a`/`accel_x`/…/`decel_a` (steps/s, steps/s²; **0** = inherit the global `vel`/`accel`/`decel`), `watchdog_ms` (host keepalive timeout in ms; **0** = disabled, default). Mechanical fields require motors **disabled**; `test_mode`, soft limits, hardware limit pins, per-axis dynamics, and `watchdog_ms` may change while enabled. Soft limits are enforced in **machine (absolute) coordinates** — they bound the physical travel envelope and do **not** move with `set_work_origin`; they reject out-of-range targets. Hardware limits reject moves into an active switch and decelerate-stop during motion (latching `limit_status` in `get_status`, cleared by `clear_alerts`). If `watchdog_ms` is set, the board decelerates to a stop and latches `watchdog_tripped` if no `keepalive`/`enable`/`configure` arrives within the window; further motion is blocked until `keepalive` (or `clear_alerts`). **Persisted to ClearCore NVM** (blob version 5). |
| `reset_config` | — | Restore compile-time defaults and clear NVM blob. Motors must be **disabled**. |
| `set_test_mode` | `enabled` or `test_mode` (bool). Omitted params turn **on**. Allowed during `wait_idle`. **Persisted to NVM.** |
| `set_units` | `units`: `"mm"` or `"inch"` (**NVM**) |
| `set_units_a` | `units`: `"deg"` or `"rev"` — rotary A-axis unit (**NVM**, blob v4). Internal storage stays degrees; pose and limits convert accordingly. |
| `set_mode` | `mode`: `"abs"` or `"rel"` (**NVM**) |
| `set_work_origin` | `x`,`y`,`z`,`a` — new work coordinates at the current machine pose (G92-style offset). Omitted axes unchanged. Empty object zeros all axes. **Not** NVM-backed (session only). |

Default mechanics: 800 steps/rev, 5 mm pitch, XY enabled (`axis_mask` 3), mm, absolute. On boot, a valid NVM blob overrides these defaults.

One-motor bench: `configure` `axis_mask` **1** (X/M0 only) so enable/HLFB/alerts ignore the empty connector. If the partner XY motor is missing or in alert, `move_linear` / `jog` fall back to independent `Move` instead of coordinated `QueueLinear` (that path was rejecting with `xy queue rejected`).

Axis A is rotary: values are in the configured A-axis unit (`set_units_a`, default **degrees**) regardless of `set_units`.

### Motion

Feed is **units per minute** in the active linear unit (mm/min or inch/min).

| Method | Params |
|--------|--------|
| `move_linear` | `x`,`y` optional `z`,`a`, `feed`, `rapid` (bool). XY uses coordinated `QueueLinear`. |
| `move_arc` | `x`,`y`,`i`,`j`,`clockwise`,`feed`. I/J relative to start (G-code convention). XY only; `z`/`a` rejected. |
| `jog` | relative `x`,`y`,`z`,`a`,`feed` (ignores abs/rel mode) |
| `dwell` | `seconds` (max 600), interruptible by estop |

### Digital I/O

ClearCore onboard pin indices **0–12** (IO-0 … A-12). IO-0 … IO-5 are configurable input/output; DI-6 … A-12 are input-only.

- **`read_inputs`**: read raw `state` (0/1) for one pin or all pins. Allowed during `wait_idle`.
- **`write_output`**: drive IO-0 … IO-5 high/low. Rejects pins **6–12** and pins reserved for hardware limit switches.

```json
{"jsonrpc":"2.0","id":20,"method":"read_inputs","params":{"pin":6}}
{"jsonrpc":"2.0","id":21,"method":"write_output","params":{"pin":0,"state":true}}
```

## Homing / probing

- **`home`**: seek the hardware limit switch configured for `axis`/`dir` (`pos_lim_<axis>` or `neg_lim_<axis>`) at a slow `feed`, decelerate to a stop on contact, optionally `backoff` away from the switch, then optionally set the work origin to 0 (`zero`, default true). The seek move bypasses soft limits; the hardware limit is expected to stop it. Fails with `"limit not configured"` if no DI is assigned, `"limit already active"` if the switch is already tripped, or `"limit not reached"` if the seek distance is exhausted without contact. Requires `enable` (or `test_mode`).
- **`probe`**: move `axis` toward `dir` at probe `feed` until the probe input `pin` (DI 1–12) triggers, then stop and report the touch `pos` in work units. `active` selects trigger polarity (default `high` = touched when the pin reads high). Optional `backoff` and `zero` (default false). Fails with `"probe already active"`, `"probe not reached"`, or `"pin reserved for limit"`.

Both are **blocking** and interruptible by `estop` / `stop` / `queue_clear` (which abort the seek and return an error).

```json
{"jsonrpc":"2.0","id":30,"method":"configure","params":{"pos_lim_x":7}}
{"jsonrpc":"2.0","id":31,"method":"home","params":{"axis":"x","dir":"pos","feed":200,"backoff":1.0}}
{"jsonrpc":"2.0","id":32,"method":"probe","params":{"axis":"z","dir":"neg","pin":8,"feed":100,"zero":true}}
```

## Safety / limits

- **Soft limits** are enforced in **machine (absolute) coordinates**. They bound the physical travel envelope and do **not** move with `set_work_origin`, so a work-origin shift cannot be used to escape them. Configure with `min_x`/`max_x`/… (work units; A in configured A units) and clear with `clear_min_x`/`clear_max_x`/… or `clear_limits`.
- **Hardware limits** (`pos_lim_<axis>`/`neg_lim_<axis>`, ClearCore pin index 0–12) reject any move that would start into an already-active switch, and decelerate the active move to a stop if a switch trips mid-move. A tripped hardware limit latches `limit_status` in `get_status` (`{tripped:true,axis,dir}`); clear it with `clear_alerts`.
- **Per-axis dynamics** (`vel_x`/…/`vel_a`, `accel_x`/…/`accel_a`, `decel_x`/…/`decel_a`) cap each axis independently. A value of **0** means "inherit the global `vel`/`accel`/`decel`". Useful for slowing a Z axis while leaving X/Y fast.
- **Host watchdog** (`watchdog_ms`): when non-zero, the board expects a `keepalive` (or `enable`/`configure`) within every `watchdog_ms` window. If the host goes silent, the board decelerates to a stop, latches `watchdog_tripped` in `get_status`, and blocks further motion until `keepalive` (or `clear_alerts`) is received. Default **0** (disabled). `keepalive` is allowed during blocking waits.

```json
{"jsonrpc":"2.0","id":40,"method":"configure","params":{"vel_z":2000,"accel_z":5000,"watchdog_ms":1000}}
{"jsonrpc":"2.0","id":41,"method":"keepalive","params":{}}
{"jsonrpc":"2.0","id":42,"method":"get_status","params":{}}
```

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

Set X soft limits 0–300 mm (persisted; limits may be set while enabled):

```json
{"jsonrpc":"2.0","id":10,"method":"configure","params":{"min_x":0,"max_x":300}}
{"jsonrpc":"2.0","id":11,"method":"move_linear","params":{"x":350,"feed":5000}}
```

Map X positive hardware limit to DI-7 (active when DI reads ON):

```json
{"jsonrpc":"2.0","id":12,"method":"configure","params":{"pos_lim_x":7}}
```

Set the A axis to revolutions and read torque/queue:

```json
{"jsonrpc":"2.0","id":13,"method":"set_units_a","params":{"units":"rev"}}
{"jsonrpc":"2.0","id":14,"method":"get_status","params":{}}
{"jsonrpc":"2.0","id":15,"method":"queue_status","params":{}}
{"jsonrpc":"2.0","id":16,"method":"queue_clear","params":{}}
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
