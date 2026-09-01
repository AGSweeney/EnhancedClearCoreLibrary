# ClearAI firmware

Microchip Studio project that runs on ClearCore (ATSAME53) and executes JSON-RPC Lines motion primitives.

**Solution:** `ClearCoreFirmware/ClearAI_Firmware.atsln`  
**Config header:** `ClearCoreFirmware/ClearAI_Config.h`  
**Protocol:** [`../PROTOCOL.md`](../PROTOCOL.md)

## ClearPath MSP setup

Configure each ClearPath for:

| Setting | Value |
|---------|--------|
| Input | Step and Direction (or library QUAD_AB) |
| HLFB | ASG-Position w/Measured Torque, **482 Hz** |

Firmware applies `HLFB_MODE_HAS_BIPOLAR_PWM` and `HLFB_CARRIER_482_HZ` on enabled axes.

## Build and flash

### Microchip Studio 7

1. Open `ClearCoreFirmware/ClearAI_Firmware.atsln`.
2. Set **ClearAI_Firmware** as the startup project.
3. Build (Debug). Output: `ClearCoreFirmware/Debug/ClearAI_Firmware.bin` (+ `.elf`, `.uf2`).
4. Flash via the project custom programming tool (`Tools/flash_clearcore.cmd`) or copy the UF2 if using that workflow.

### Command-line rebuild (Windows)

Requires Atmel Studio 7 ARM GCC on the machine:

```powershell
cd ClearAI_Controller\ClearCoreFirmware\Debug
& "C:\Program Files (x86)\Atmel\Studio\7.0\shellutils\make.exe" all
```

Flash:

```powershell
..\..\..\Tools\flash_clearcore.cmd ClearAI_Firmware.bin
```

(`flash_clearcore.cmd` finds VID `2890` / PID `8022`, enters bootloader, programs with bossac at offset `0x4000`.)

Linked libraries: `libClearCore`, `LwIP` (Ethernet).

## Source modules

| File | Responsibility |
|------|----------------|
| `ClearAI_Firmware.cpp` | `main`: USB → motion init → Ethernet → poll loop |
| `ClearAI_Config.h` | Ports, defaults, protocol version |
| `JsonRpcLine.*` | Line parse (jsmn), `RpcParams` |
| `Primitives.*` | Method dispatch → MotionBridge / replies |
| `MotionBridge.*` | Enable, gates, XY coordinated + independent moves, pose, wait/dwell |
| `Transport.*` | USB CDC, TCP 9100/9101, UDP 9102 |
| `third_party/jsmn/` | Minimal JSON tokenizer |

### Main loop (conceptual)

1. `TransportPollEthernet` / discovery / telemetry  
2. `MotionPollEstop`  
3. Read one JSONL line → `PrimitivesDispatchLine`  
4. `Delay_ms(1)`

## Defaults (`ClearAI_Config.h`)

| Symbol | Default | Meaning |
|--------|---------|---------|
| `CLEARAI_PROTOCOL_VERSION` | `"1.0"` | Reported by `get_capabilities` |
| `CLEARAI_TCP_CONTROL_PORT` | 9100 | Control |
| `CLEARAI_TCP_TELEMETRY_PORT` | 9101 | Status stream |
| `CLEARAI_UDP_DISCOVERY_PORT` | 9102 | Discover |
| `CLEARAI_DEFAULT_STEPS_PER_REV` | 800 | Encoder/step count per rev |
| `CLEARAI_DEFAULT_PITCH_MM` | 5.0 | Linear pitch |
| `CLEARAI_DEFAULT_VEL_STEPS` | 27000 | ~2025 rpm at 800 ppr |
| `CLEARAI_DEFAULT_ACCEL_STEPS` | 250000 | Accel/decel steps/s² |
| `CLEARAI_DEFAULT_AXIS_MASK` | `0x3` | XY |
| `CLEARAI_TEST_MODE_DEFAULT` | 0 | Boot with gates on (overridden if NVM valid) |

## NVM-backed configuration

ClearAI stores a versioned blob in ClearCore **user NVM** (`NvmManager`, offset `NVM_LOC_USER_START`) so you only `configure` when values change.

| Behavior | Detail |
|----------|--------|
| Load | On `MotionInit`, after compile defaults; valid magic `'CAIC'` + version 1, 2, 3, 4, or 5 applies |
| Save | After successful `configure`, `set_test_mode`, `set_units`, `set_mode` |
| Read | `get_config` — live values + `nvm` / `nvm_valid` |
| Clear | `reset_config` (motors disabled) — compile defaults + wipe blob |
| Not stored | Work origin / enable / pose (session only) |

Requires board supply suitable for NVM writes (~20 V+ per libClearCore). User page survives chip erase; invalid/missing blob → compile defaults.

### Soft travel limits (NVM v2)

Per-axis min/max (`min_x`/`max_x`, …). Values use active linear units (mm/inch); axis A uses the configured A-axis unit (see NVM v4). Negative ranges are supported (e.g. X −50…50 mm). As of NVM v5 these are enforced in **machine (absolute) coordinates** — they bound the physical travel envelope and do not move with `set_work_origin` (see Safety / limits).

| `limit_flags` bit | Meaning |
|-------------------|---------|
| 0,1 | X min / max enabled |
| 2,3 | Y min / max |
| 4,5 | Z min / max |
| 6,7 | A min / max |

`move_linear`, `jog`, and `move_arc` (endpoint XY) reject targets outside enabled limits with `"x below min limit"`, etc. Limits can be set via `configure` while motors are enabled; mechanical fields still require disable. Use `clear_limits` or `clear_min_x`/`clear_max_x`/… to remove limits.

### Hardware limit switches (NVM v3)

Optional per-axis POS/NEG limit inputs on ClearCore onboard I/O **pin indices 0–12** (libClearCore `ClearCorePins`):

| Index | Connector | Capability |
|-------|-----------|------------|
| 0–5 | IO-0 … IO-5 | Digital **input or output** on the connector; when assigned as a limit, firmware forces **INPUT_DIGITAL** |
| 6–8 | DI-6 … DI-8 | **Input only** |
| 9–12 | A-9 … A-12 | **Input only** (also analog-capable, not used as analog here) |

Configure with `pos_lim_x`, `neg_lim_x`, … (pin index). **0** or **255** disables that input. CCIO expansion pins (64+) are not supported yet.

- Rejects queued moves that would travel further into an active limit (`"x pos limit active"`, `"y neg limit active"`, …).
- During motion, triggers deceleration stop if a limit activates while moving in that direction.
- Bypassed in `test_mode` (same as hardware estop).
- `clear_limits` clears soft limits and hardware limit DI assignments.

### Digital I/O (`read_inputs` / `write_output` / `read_analog` / `write_analog` / `write_pwm`)

| Method | Detail |
|--------|--------|
| `read_inputs` | Optional `pin` 0–12; returns `{pins:[{pin,state,mode}]}` with raw digital level. `mode` is `in`, `out`, `pwm`, `analog_in`, `analog_out`, or `other` |
| `write_output` | `pin` 0–5, `state` bool — drives IO-0…IO-5 as **OUTPUT_DIGITAL** |
| `read_analog` | Optional `pin` 9–12; returns `{pins:[{pin,volts,raw}]}` — sets **INPUT_ANALOG** and reads `AnalogVoltage()` + `State()` for A-9…A-12 |
| `write_analog` | `pin` 0 only, `value` (raw 0–2047) or `microamps` (0–20000) — sets **OUTPUT_ANALOG** on IO-0 and commands the DAC (0–20 mA) |
| `write_pwm` | `pin` 0–5, `duty` 0–255 — sets **OUTPUT_PWM** and commands the duty. Frequency is fixed by the timer. Last duty per pin reported in `get_status` as `pwm_duty` |

DI-6…A-12 are read-only via `read_inputs`. Pins assigned as hardware limits cannot be written or re-purposed by `read_analog`/`write_analog`/`write_pwm`.

### Input-change notifications (NVM v6)

`subscribe_inputs` opts in to edge notifications for a set of pins (0–12); `input_changed` events are pushed on the telemetry stream (port 9101) as JSON-RPC notifications `{pin,state,edge}` (`edge` = `rising`/`falling`), debounced per pin by `debounce_ms` (default 5, `CLEARAI_INPUT_DEBOUNCE_DEFAULT_MS`). `unsubscribe_inputs` stops them. Both are allowed during blocking waits (`wait_idle`/`dwell`/`home`/`probe`). Subscription state is RAM-only (not persisted).

### Output power-on default (NVM v6)

`configure` accepts `out_power_on_0` … `out_power_on_5` (0/1 set, 255 = don't care / clear). At boot — after hardware-limit DI modes are applied — `ApplyOutputDefaults()` drives each IO-0…IO-5 with a set mask bit to its stored state. Default mask 0 leaves boot behavior unchanged. Reported by `get_config` as `out_power_on_state` / `out_power_on_mask`. May change while motors are enabled.

### Network configuration (NVM v7)

The board boots with DHCP by default (fallback `192.168.0.109` if DHCP fails). `configure_network` persists a static IP config to NVM: `netMode` (0=DHCP, 1=static) plus `ipOctets`/`netmaskOctets`/`gatewayOctets` (4 octets each). `TransportInitEthernet` reads `MotionGetNetworkConfig()` and, for static mode, sets `EthernetMgr.LocalIp`/`NetmaskIp`/`GatewayIp` (which only apply when `m_dhcp` is false, i.e. DHCP is not started) and skips `DhcpBegin()`; DHCP mode calls `DhcpBegin()` with the fallback on failure. Because `EthernetMgr.Setup()` runs once, network changes only apply after a reboot — the `restart` RPC calls `SysMgr.ResetBoard()` to apply them. `configure_network` validates dotted-quad strings (`ParseIpOctets`) and is allowed while enabled and during blocking waits. Reported by `get_config` as `network_mode` / `ip_address` / `netmask` / `gateway`. Older blobs (v1–v6) load with DHCP defaults (`ResetNetworkDefaults`).

### A-axis units (NVM v4)

Axis A is rotary. The user-facing unit is configurable with `set_units_a` (`"deg"` or `"rev"`), persisted to NVM (blob v4). Internal storage stays **degrees** (motor A mechanical params use `UNIT_DEGREES`), so `ToInternal`/`FromInternal` convert revolutions ↔ 360°. `get_pose`, `get_status`, and soft limits all report/accept A values in the active A unit. Older NVM blobs (v1–v3) load with the default `deg`.

### Motor torque / HLFB readback

HLFB is configured for **measured torque** (`HLFB_MODE_HAS_BIPOLAR_PWM`). `get_status` returns `hlfb_percent`, a per-axis array of the latest HLFB duty cycle as a percentage of peak torque (−100…100). `null` means no measurement yet (`HLFB_DUTY_UNKNOWN`). The aggregate `hlfb` boolean remains true only when every enabled axis is asserted.

### Move queue introspection & flush

Coordinated XY moves queue in the planner (`MotionQueueCount`). `queue_status` returns `{queue, active}` (pending segment count and whether a move is executing); allowed during `wait_idle`. `queue_clear` decelerates the active coordinated segment to a stop and drops all pending segments (it zeroes the planner queue counts), then stops independent Z/A with `MoveStopDecel`. Motors stay enabled. Accepted during `wait_idle`/`dwell` (treated as a safety method, it interrupts the wait).

### Jog-continuous velocity (`jog_velocity` / `jog_stop`)

`jog_velocity` issues `MotorDriver::MoveVelocity` per axis (X and Y are **not** coordinated in this mode — the `CoordinatedMotionController` has no velocity API). Velocities are signed user-units/sec converted to steps/sec via `ToInternal(axis, v) * g_stepsPerMm[axis]` (A uses deg→steps). Hardware limit switches auto-stop a velocity move (alert bits `motion_canceled_positive_limit` / `motion_canceled_negative_limit`); **soft limits are not enforced** in velocity mode. `jog_stop` decelerates all axes to a stop and is allowed during blocking waits.

### Motion log / last-error (`get_log` / `clear_log`)

A RAM ring buffer (`g_log[16]`) records `ClearAiLogEntry { ms, method, reason, kind }` — `kind` 0 for rejected moves (logged at every motion-method error return) and 1 for info events (estop/stop/disable). `get_log` returns the ring as a JSON array most-recent-first; `clear_log` empties it. The buffer is cleared on reboot. Both are allowed during `wait_idle`.

### Uptime / move counters / distance

`get_status` reports `uptime_ms` (since boot), `moves` (accepted move count), `moves_rejected`, per-axis `distance` (cumulative `|Δ|` in user units for linear/jog moves), and `arc_path` (cumulative XY arc length in mm). Counters accumulate synchronously when a move is issued; distance is derived from `(target - start)` steps converted via `FromInternal`.

### Batched moves (`move_batch`)

`move_batch` parses a `moves` array (max 8 elements) of `move_linear`/`move_arc`/`dwell` param objects using a shared `ParseObjectParams` helper, then dispatches each to the corresponding motion function. It stops at the first rejected element and reports `accepted` and `failed_at`. A `dwell` element blocks the RPC thread (same as a standalone `dwell`).

### Move timing

`move_linear`/`move_arc` replies include `est_ms` — a rough cruise estimate (path length / cruise velocity, ignoring accel/decel ramps); treat as a lower bound. `wait_idle` replies include `elapsed_ms` (actual wall-clock wait).

### Alert decoding

`get_status` reports `alert_reg_axis` (per-axis raw `AlertReg`) and `alerts_decoded` (array of names decoded from the OR'd alert register: `motion_canceled_in_alert`, `motion_canceled_positive_limit`, `motion_canceled_negative_limit`, `motion_canceled_sensor_estop`, `motion_canceled_motor_disabled`, `motor_faulted`).

### Homing / zeroing (`home`)

`home` seeks the hardware limit switch assigned for `axis`/`dir` (`pos_lim_<axis>`/`neg_lim_<axis>`) at a slow `feed`. The seek move bypasses soft limits (the physical limit is expected to stop it) and is commanded as a far relative target — coordinated `QueueLinear` for X/Y (other axis held), independent `Move` for Z/A. The poll loop watches the limit DI; on contact it decelerates everything to a stop, optionally retracts by `backoff`, and optionally sets the work origin to 0 (`zero`, default true) at the final position. Requires the limit DI to be configured; rejects if the switch is already active or never reached (`seek` max travel, default 1000 work units; `timeout_ms` default 30000). Blocking and interruptible by `estop`/`stop`/`queue_clear`.

### Probing (`probe`)

`probe` moves `axis` toward `dir` at probe `feed` until a probe input DI (`pin`, 1–12, not a pin reserved for a limit) triggers. `active` selects polarity (default `high` = touched when the pin reads high). On contact it decelerates to a stop and reports the touch `pos` in work units; optional `backoff` and `zero` (default false). Same seek/timeout defaults as `home`. Blocking and interruptible.

### Safety / limits (NVM v5)

- **Soft limits are machine-coordinate.** `ValidateTargetSteps` compares the target against `MachineInternal(axis)` directly (not the work-shifted value), so soft limits bound the absolute physical envelope and cannot be escaped by `set_work_origin`.
- **Per-axis vel/accel/decel overrides.** `vel_x`/…/`vel_a`, `accel_x`/…/`accel_a`, `decel_x`/…/`decel_a` (steps/s, steps/s²). A value of **0** inherits the global `vel`/`accel`/`decel`. `AxisVelCap`/`AxisAccelCap`/`AxisDecelCap` resolve the effective cap; `ApplyLimits` and `ApplyFeed` apply them per motor. Persisted in NVM v5; older blobs load with all per-axis fields = 0 (inherit).
- **Limit-triggered fault state.** When `MotionPollEstop` decelerates a move because a hardware limit tripped mid-move, it latches `g_limitTrippedAxis`/`g_limitTrippedPos`. Exposed as `limit_status` in `get_status` and cleared by `clear_alerts`.
- **Host watchdog / keepalive.** `watchdog_ms` (0 = disabled, default). `MotionPollWatchdog` (called from the main loop) decelerates to a stop and latches `g_watchdogTripped` if `(now - g_lastKeepaliveMs) >= watchdogMs`. `GateMotion` blocks further motion while tripped. `keepalive` (and `enable`, and setting `watchdog_ms` via `configure`) re-arm the timer and clear the latch. `keepalive` is in the allowed-during-wait list. Persisted in NVM v5.

## Motion behavior

### Pose tracking

Work pose for `get_pose` / `get_status` is derived from each axis motor’s **`PositionRefCommanded()`** (machine steps → mm/deg via steps-per-unit), minus `set_work_origin` offsets. Independent `Move` on a single-motor bench therefore still updates `x`.

### XY vs independent moves

- If both X and Y are healthy and enabled in the mask, linear XY uses coordinated `QueueLinear`.
- If the partner axis is missing/unhealthy (typical one-motor bench), firmware falls back to independent `MotorDriver::Move` so you do not get `xy queue rejected`.

### Feed

Feed is **units per minute** (mm/min or inch/min). For independent moves, firmware also applies velocity limits on the motor (`VelMax`) from feed so jogs honor speed.

Rough RPM (800 ppr, 5 mm pitch):  
`feed_mm_per_min ≈ rpm × pitch_mm` → 2000 rpm ≈ 10000 mm/min.

### Test mode

`set_test_mode` / `configure` `test_mode`:

- Bypasses hardware DI-6 estop gate, HLFB wait, alert gate, and enable requirement (auto-enable for motion).
- Status: `test_mode: true`, `estop` forced false; `hw_estop` still reports raw DI-6.
- Software `stop` / `estop` / `disable` still halt.

**Bench only.** Do not use with people or payload at risk.

### Safety still enforced in normal mode

- Hardware estop DI-6 (polarity: `estop_di6` 0=off, 1=OK when pin ON, 2=inverted).
- Motor alerts block motion unless test mode.
- `enable` required unless test mode.
- `estop` / `stop` / `disable` accepted during `wait_idle` and `dwell`.

## One-motor bench checklist

1. Flash current firmware.  
2. Disable motors.  
3. `configure` with `axis_mask: 1`, optionally `vel` / `accel` / `decel`.  
4. `set_test_mode` on (if DI-6 / empty M1 would otherwise block).  
5. `enable`.  
6. `jog` / `move_linear` + `wait_idle`; confirm `get_pose` tracks X.

CLI example (serial):

```powershell
cd ClearAI_Controller\python
$env:PYTHONPATH="."
python -m clearai --port COM4 disable
python -m clearai --port COM4 configure --axis-mask 1 --vel 27000 --accel 250000 --decel 250000
python -m clearai --port COM4 set_test_mode --on
python -m clearai --port COM4 enable
```

Ethernet (replace host):

```powershell
python -m clearai --host 172.16.82.113 get_status
```

## Ethernet notes

- Prefer DHCP; firmware may fall back to a static address if DHCP fails (historically `192.168.0.109` — confirm on your network).
- Pin MCP/CLI with `CLEARAI_HOST` / `--host` when UDP discover is flaky across subnets.
- Do not leave USB CDC open while another tool holds it (can reset the CDC link). MCP is intended over **TCP**, not serial.

## Related docs

- [ARCHITECTURE.md](ARCHITECTURE.md)  
- [PROTOCOL.md](../PROTOCOL.md)  
- [TROUBLESHOOTING.md](TROUBLESHOOTING.md)
