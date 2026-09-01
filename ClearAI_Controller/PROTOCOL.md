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
| `get_status` | — | enabled, moving, hlfb, estop, `hw_estop`, `test_mode`, alerts, queue, `queue_active`, `alert_reg` (OR'd), `alert_reg_axis` (per-axis), `alerts_decoded` (array of names: `motion_canceled_*`, `motor_faulted`), `units_a`, `hlfb_percent` (per-axis % of peak torque, or null if unknown), `watchdog_ms`, `watchdog_tripped`, `limit_status` (`{tripped,axis,dir}` or `{tripped:false}`), `pwm_duty` (last commanded duty per IO-0…IO-5), `uptime_ms`, `moves`, `moves_rejected`, `distance` (`{x,y,z,a}` in user units), `arc_path`, pose |
| `get_pose` | — | work coordinates in active units (A in configured A units) |
| `get_config` | — | live config + `nvm` / `nvm_valid` (axis_mask, test_mode, mechanics, vel/accel/decel, `units_a`, `limit_flags`, `limits_min`/`limits_max`, `pos_lim_di`/`neg_lim_di`, `vel_axis`/`accel_axis`/`decel_axis`, `watchdog_ms`, `out_power_on_state` (boot state per IO-0…IO-5), `out_power_on_mask`, `network_mode` (`dhcp`/`static`), `ip_address`, `netmask`, `gateway`, …) |
| `enable` | — | `{ok:true}` after enable request (short HLFB wait) |
| `disable` | — | abrupt stop + disable |
| `clear_alerts` | — | clear motor alerts |
| `stop` | — | decelerate |
| `estop` | — | abrupt stop + disable |
| `wait_idle` | `timeout_ms` (default 60000) | `{ok:true,elapsed_ms:N}` or timeout/estop error |
| `read_inputs` | optional `pin` (0–12); omit for all onboard pins | `{pins:[{pin,state,mode},…]}` — raw digital state; `mode` is `in`, `out`, `pwm`, `analog_in`, `analog_out`, or `other` |
| `write_output` | `pin` (0–5 only), `state` (bool or 0/1) | `{ok:true}` — sets **OUTPUT_DIGITAL** then drives the pin |
| `read_analog` | optional `pin` (9–12); omit for all analog inputs | `{pins:[{pin,volts,raw},…]}` — analog voltage (V) and raw ADC for A-9…A-12. Sets **INPUT_ANALOG** on read. Rejects pins reserved for hardware limits. Allowed during `wait_idle`. |
| `write_analog` | `pin` (must be 0), `value` (raw 11-bit 0–2047, 0–20 mA) **or** `microamps` (0–20000) | `{ok:true}` — sets **OUTPUT_ANALOG** on IO-0 and commands the DAC. |
| `write_pwm` | `pin` (0–5 only), `duty` (0–255) | `{ok:true}` — sets **OUTPUT_PWM** and commands the duty. Frequency is fixed by the timer. |
| `subscribe_inputs` | `pins` (array of 0–12), optional `debounce_ms` (default 5) | `{subscribed:[…]}` — opt-in to edge notifications for the given pins. `input_changed` events are pushed on the telemetry stream (port 9101). Allowed during `wait_idle`/`dwell`/`home`/`probe`. |
| `unsubscribe_inputs` | — | `{ok:true}` — stop input edge notifications. |
| `configure_network` | optional `mode` (`dhcp`/`static`), `ip_address`/`netmask`/`gateway` (dotted-quad strings; gateway `0.0.0.0` = none) | `{network_mode,ip_address,netmask,gateway,applies_on:"restart"}` — persisted to NVM; applies on next boot. Static mode requires `ip_address`. Allowed while enabled. |
| `restart` | — | `{ok:true}` then the board resets (TCP connection drops). Use to apply pending `configure_network` changes. |
| `queue_status` | — | `{queue:N, active:bool}` — pending coordinated segments and active flag. Allowed during `wait_idle`. |
| `get_log` | — | JSON array of recent log entries `{ms,method,reason,kind}` (most-recent first; `kind` 0=rejected move, 1=info). Allowed during `wait_idle`. |
| `clear_log` | — | `{ok:true}` — clear the motion log ring buffer. Allowed during `wait_idle`. |
| `queue_clear` | — | `{ok:true}` — decelerate the active move to a stop and drop pending queued segments. Motors stay enabled. Accepted during `wait_idle`/`dwell` (interrupts the wait). |
| `home` | `axis` (`x`/`y`/`z`/`a`), `dir` (`pos`/`neg`), optional `feed`, `seek` (max travel, default 1000), `backoff`, `zero` (default true), `timeout_ms` (default 30000) | `{homed:true,axis,dir,pos,limit_pin}` — seek the configured hardware limit for `axis`/`dir`, decel-stop on contact, optional backoff, optional zero. Requires `pos_lim_<axis>`/`neg_lim_<axis>` DI configured. Blocking (interruptible by `estop`/`stop`/`queue_clear`). |
| `probe` | `axis`, `dir`, `pin` (DI 1–12), optional `feed`, `seek`, `backoff`, `zero` (default false), `active` (`high`/`low`, default `high`), `timeout_ms` | `{probed:true,axis,dir,pos,pin}` — move at probe feed until the probe DI triggers, stop, report touch position. Blocking (interruptible). |
| `keepalive` | — | `{ok:true}` — reset the host watchdog timer and clear the `watchdog_tripped` latch. Allowed during `wait_idle`/`dwell`/`home`/`probe`. |

### Configuration

| Method | Params |
|--------|--------|
| `configure` | `steps_per_rev_x/y/z/a`, `pitch_x/y/z` (mm), `gear_x/y/z/a`, `vel`/`accel`/`decel` (steps/s, steps/s²), `axis_mask` (bit0=X … bit3=A), `estop_di6` (0=off, 1=default, 2=inverted), `test_mode` (bool), soft limits `min_x`/`max_x`/… (work units; A in configured A units), hardware limits `pos_lim_x`/`neg_lim_x`/… (ClearCore **pin index** 0–12: **0–5** = IO-0…IO-5 configurable in/out — firmware forces input when used as a limit; **6–12** = DI-6…A-12 input-only; **0** or **255** = disabled), `clear_limits`, `clear_min_x`/`clear_max_x`/… (bool), per-axis dynamics `vel_x`/`vel_y`/`vel_z`/`vel_a`/`accel_x`/…/`decel_a` (steps/s, steps/s²; **0** = inherit the global `vel`/`accel`/`decel`), `watchdog_ms` (host keepalive timeout in ms; **0** = disabled, default), `out_power_on_0`/…/`out_power_on_5` (boot state for IO-n: **0**/**1** set, **255** = don't care / clear). Mechanical fields require motors **disabled**; `test_mode`, soft limits, hardware limit pins, per-axis dynamics, `watchdog_ms`, and `out_power_on_*` may change while enabled. Soft limits are enforced in **machine (absolute) coordinates** — they bound the physical travel envelope and do **not** move with `set_work_origin`; they reject out-of-range targets. Hardware limits reject moves into an active switch and decelerate-stop during motion (latching `limit_status` in `get_status`, cleared by `clear_alerts`). If `watchdog_ms` is set, the board decelerates to a stop and latches `watchdog_tripped` if no `keepalive`/`enable`/`configure` arrives within the window; further motion is blocked until `keepalive` (or `clear_alerts`). `out_power_on_*` drives the pin to the stored state at boot (after limit DI modes are applied). **Persisted to ClearCore NVM** (blob version 7). |
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
| `move_linear` | `x`,`y` optional `z`,`a`, `feed`, `rapid` (bool). XY uses coordinated `QueueLinear`. Reply: `{ok:true,queued:true,est_ms:N}` (rough cruise estimate, ignores ramps). |
| `move_arc` | `x`,`y`,`i`,`j`,`clockwise`,`feed`. I/J relative to start (G-code convention). XY only; `z`/`a` rejected. Reply: `{ok:true,queued:true,est_ms:N}`. |
| `jog` | relative `x`,`y`,`z`,`a`,`feed` (ignores abs/rel mode) |
| `jog_velocity` | per-axis velocity `x`,`y`,`z`,`a` in user units/sec (signed) | `{ok:true}` — starts continuous velocity motion. Hardware limits auto-stop; soft limits are NOT enforced. Stop with `jog_stop` or `stop`. |
| `jog_stop` | — | `{ok:true}` — decelerate-stop any active `jog_velocity`. Allowed during blocking waits. |
| `move_batch` | `moves` (array of linear/arc/dwell param objects) | `{ok:true,queued:true,accepted:N}` or `{ok:false,accepted:K,failed_at:i,error}` on the first rejected element. Dwell-in-batch blocks. |
| `dwell` | `seconds` (max 600), interruptible by estop |

### Digital I/O

ClearCore onboard pin indices **0–12** (IO-0 … A-12). IO-0 … IO-5 are configurable input/output (and PWM-capable; IO-0 also provides a 0–20 mA analog current output via the DAC); DI-6 … A-12 are input-only (A-9 … A-12 additionally support analog input). The live per-pin function is reported by `read_inputs`/`get_status` as `mode` (`in`, `out`, `pwm`, `analog_in`, `analog_out`, `other`).

- **`read_inputs`**: read raw `state` (0/1) for one pin or all pins. Allowed during `wait_idle`.
- **`write_output`**: drive IO-0 … IO-5 high/low. Rejects pins **6–12** and pins reserved for hardware limit switches.
- **`read_analog`**: read analog voltage (V) and raw ADC for A-9 … A-12. Sets **INPUT_ANALOG** on read; rejects pins reserved for limits. Allowed during `wait_idle`.
- **`write_analog`**: drive IO-0 analog current output (0–20 mA). Pass raw `value` (0–2047) or `microamps` (0–20000). Pin 0 only; rejects limit pins.
- **`write_pwm`**: drive IO-0 … IO-5 as PWM (`duty` 0–255). Frequency is fixed by the timer (no per-pin frequency API). Rejects pins **>5** and limit pins. The last commanded duty per pin is reported in `get_status` as `pwm_duty`.
- **`subscribe_inputs` / `unsubscribe_inputs`**: opt-in edge notifications for pins 0–12. `input_changed` events are pushed on the telemetry stream (port 9101) as JSON-RPC notifications with `{pin,state,edge}` (`edge` = `rising`/`falling`), debounced by `debounce_ms` (default 5). Allowed during blocking waits.

```json
{"jsonrpc":"2.0","id":20,"method":"read_inputs","params":{"pin":6}}
{"jsonrpc":"2.0","id":21,"method":"write_output","params":{"pin":0,"state":true}}
{"jsonrpc":"2.0","id":22,"method":"read_analog","params":{"pin":9}}
{"jsonrpc":"2.0","id":23,"method":"write_analog","params":{"pin":0,"microamps":5000}}
{"jsonrpc":"2.0","id":24,"method":"write_pwm","params":{"pin":1,"duty":128}}
{"jsonrpc":"2.0","id":25,"method":"subscribe_inputs","params":{"pins":[6,9],"debounce_ms":5}}
{"jsonrpc":"2.0","id":26,"method":"unsubscribe_inputs","params":{}}
```

Telemetry notification (port 9101) for a subscribed edge:

```json
{"jsonrpc":"2.0","method":"input_changed","params":{"pin":6,"state":1,"edge":"rising"}}
```

### Output power-on default

`configure` accepts `out_power_on_0` … `out_power_on_5` (**0**/**1** set the boot state, **255** = don't care / clear). At boot — after hardware-limit DI modes are applied — each IO-0 … IO-5 with a set bit is driven to its stored state. Default mask 0 leaves boot behavior unchanged. Reported by `get_config` as `out_power_on_state` / `out_power_on_mask`. May change while motors are enabled.

### Network configuration (NVM v7)

The board boots with **DHCP** by default; if DHCP fails it falls back to `192.168.0.109`. `configure_network` persists a static IP configuration to NVM (mode `dhcp`/`static`, `ip_address`, `netmask`, `gateway` as dotted-quad strings; `gateway` `0.0.0.0` = none). Static mode requires `ip_address`. Changes are applied at the next boot in `TransportInitEthernet` (static sets `LocalIp`/`NetmaskIp`/`GatewayIp` and skips DHCP; DHCP calls `DhcpBegin` with the fallback on failure). Because `EthernetMgr.Setup()` runs once, changes only take effect after a reboot — call `restart` to reset the board and apply them. `configure_network` is allowed while motors are enabled and during blocking waits. Reported by `get_config` as `network_mode` / `ip_address` / `netmask` / `gateway`.

```json
{"jsonrpc":"2.0","id":30,"method":"configure_network","params":{"mode":"static","ip_address":"192.168.0.201","netmask":"255.255.255.0","gateway":"192.168.0.1"}}
{"jsonrpc":"2.0","id":31,"method":"configure_network","params":{"mode":"dhcp"}}
{"jsonrpc":"2.0","id":32,"method":"restart","params":{}}
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

## Jog velocity

`jog_velocity` starts a continuous per-axis velocity move (`MotorDriver::MoveVelocity` per axis; XY is not coordinated in this mode). Provide signed velocities in user units/sec. Hardware limit switches auto-stop the move (alert bit `motion_canceled_positive_limit` / `motion_canceled_negative_limit`); **soft limits are not enforced** in velocity mode. Stop with `jog_stop` or `stop`.

```json
{"jsonrpc":"2.0","id":50,"method":"jog_velocity","params":{"x":5}}
{"jsonrpc":"2.0","id":51,"method":"jog_stop","params":{}}
```

## Batched moves

`move_batch` queues an array of move objects (each a `move_linear`/`move_arc`/`dwell` params set) in one round-trip. It stops at the first rejected element and reports `failed_at` (0-based index) plus `accepted` count. A `dwell` element blocks the RPC thread until it completes (matches single-`dwell` semantics). Max 8 elements.

```json
{"jsonrpc":"2.0","id":60,"method":"move_batch","params":{"moves":[{"x":10,"y":0,"feed":500},{"seconds":0.5},{"x":0,"y":0,"feed":500}]}}
```

## Motion log

`get_log` returns a JSON array of recent log entries (RAM ring, most-recent first, 16 deep, cleared on reboot): `{ms,method,reason,kind}` where `kind` is `0` for a rejected move and `1` for an info event (estop/stop/disable). `clear_log` empties the buffer.

```json
{"jsonrpc":"2.0","id":70,"method":"get_log","params":{}}
{"jsonrpc":"2.0","id":71,"method":"clear_log","params":{}}
```

## Move timing

`move_linear`/`move_arc` replies include `est_ms` — a rough cruise-time estimate (path length / cruise velocity) that **ignores accel/decel ramps**; treat as a lower bound. `wait_idle` replies include `elapsed_ms` (actual wall-clock time spent waiting).

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
