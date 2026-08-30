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
| `CLEARAI_TEST_MODE_DEFAULT` | 0 | Boot with gates on |

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
