# ClearCoreROS wire protocol

**Experimental.** Not certified for production or safety-critical use.

ClearCore (ATSAME53) does not run a ROS 2 graph. DDS and `rclcpp` do not fit the board. This firmware is the motor side of a ROS 2 system: a host bridge or `ros2_control` plugin is the ROS node. The joint model matches `sensor_msgs/JointState` and a position command interface.

| Joint | Motor | Type | Command / state unit |
|-------|-------|------|----------------------|
| `joint_x` | M0 | prismatic | meters |
| `joint_y` | M1 | prismatic | meters |
| `joint_z` | M2 | prismatic | meters |
| `joint_a` | M3 | revolute | radians |

Zero is the pose at firmware boot (`PositionRefSet(0)`). Commands are absolute. The host converts a trajectory into these units; the firmware converts to steps with nearest-step rounding on the absolute target.

Default mechanics match ClearAI: 800 steps/rev, 5 mm pitch, 27000 steps/s, 250000 steps/s². That is 160000 steps per meter on X/Y/Z, so 0.01 m is exactly 1600 steps.

## Ports

Do not collide with ClearAI (9100–9102) or ClearCNC (8888, 8889, 10040).

| Port | Protocol | Role |
|------|----------|------|
| USB CDC 115200 | JSONL | Same session methods as TCP |
| **9200** | TCP JSONL | Session, one client |
| **9201** | TCP binary | Joint state out, position/velocity/heartbeat in |
| **9202** | UDP | Discovery |

The board boots from the saved network mode. DHCP is the default. If DHCP fails, the address falls back to `192.168.0.109`. A saved static address is applied instead of DHCP and takes effect on the next boot.

Discovery request (ASCII, no newline required):

```text
CLEARCORE_ROS_DISCOVER?
```

Reply:

```text
CLEARCORE_ROS ClearCoreROS IP=<addr> TCP=9200 STREAM=9201 FW=1.0
```

## Session (JSON-RPC Lines)

One JSON object per line, UTF-8, `\n` terminated. `\r` is ignored.

```json
{"jsonrpc":"2.0","id":1,"method":"enable"}
```

Success:

```json
{"jsonrpc":"2.0","id":1,"result":{"enabled":true}}
```

Failure uses `"error":{"code":-32000,"message":"..."}`. Unknown methods use `-32601`. Parse errors use `-32700`.

| Method | Params | Result |
|--------|--------|--------|
| `get_capabilities` | — | protocol, ports, joint names, units, `axis_mask`, `nvm` |
| `get_config` | — | mechanics, watchdog, estop mode, test mode, `nvm` / `nvm_valid`, network |
| `get_status` | — | flags, `alert_reg`, `alerts`, position/velocity/effort |
| `configure` | see below | `{"ok":true}` and the live configuration is written to NVM |
| `reset_config` | — | compile defaults, and the NVM blob is cleared. Motors must be disabled. |
| `configure_network` | `mode` `dhcp` or `static`, plus `ip_address`, `netmask`, `gateway` | saved network settings. `applies_on` is `restart` |
| `restart` | — | resets the board so a saved address takes effect |
| `set_test_mode` | `{"on":true}` | test mode skips DI-6 and the HLFB check |
| `enable` / `disable` | — | enable waits up to 500 ms for HLFB on every masked axis. If HLFB is not asserted, enable fails and the motors are left disabled. Test mode skips that check. Enable also fails while a watchdog latch is set. |
| `stop` | — | decelerate, drop the goal, stay enabled |
| `estop` | — | abrupt stop and disable |
| `clear_alerts` | — | `ClearAlerts()` and the only call that clears a watchdog latch. Estop stays if DI-6 is still faulted. |
| `keepalive` | — | refreshes the host timer. A tripped watchdog stays tripped. |
| `set_joints` | `x`,`y`,`z`,`a` in meters / radians | absolute goal for the named joints |
| `move_linear` | absolute `x`,`y`,`z`,`a`, optional `feed_mps` | straight line; XY are coordinated when both are enabled |
| `move_arc` | end `x`,`y`, center offset `i`,`j`, optional `cw`, `feed_mps` | XY arc. Requires both axes |
| `wait_idle` | optional `timeout_ms` | blocks until motion is still |
| `home` | `axis`, `dir`, optional `seek`, `backoff`, `timeout_ms`, `zero` | seek that axis's limit switch |
| `probe` | `axis`, `dir`, `pin`, optional `active`, `seek`, `backoff`, `zero` | seek until the probe input trips |
| `xrce_connect` | `ip_address`, optional `port` | start the XRCE-DDS client toward a micro-ROS agent |
| `xrce_disconnect` | — | stop the XRCE-DDS client |

`configure` fields:

| Field | Meaning |
|-------|---------|
| `axis_mask` | bits 0..3 = X Y Z A. Range 1..15. Requires motors disabled. |
| `steps_per_rev` | one number (all axes) or four numbers. Requires disabled. |
| `pitch_mm` | linear pitch. Axis A is always revolute and ignores pitch. Requires disabled. |
| `vel_steps`, `accel_steps`, `decel_steps` | step generator limits. Applied immediately. |
| `watchdog_ms` | `0` disables. Default 500. |
| `estop_di6` | `0` off, `1` fault when DI-6 is low (default), `2` fault when DI-6 is high. |
| `min_x` … `min_a`, `max_x` … `max_a` | Soft travel limit in joint units (meters, A in radians). Setting one enables that side. Allowed while motors are enabled. |
| `clear_min_x` … `clear_max_a` | `true` disables that one side. |
| `clear_limits` | `true` clears every soft limit and every limit-switch assignment. |
| `pos_lim_x` … `pos_lim_a`, `neg_lim_x` … `neg_lim_a` | Digital input for that direction. `0` or `255` disables it. `1`..`12` are IO-0…IO-5, DI-6…DI-8, and A-9…A-12. The pin is forced to a digital input. |

`configure`, `set_test_mode`, and `configure_network` write one blob (`magic` `CROS`) at `NvmManager` user offset 0. Version 1 is mechanics and network. Version 2 adds the soft limits and limit-switch pins. Boot still loads a version 1 blob and treats limits as unset. An unrecognized blob, including a ClearAI `CAIC` blob, is left untouched and the compile defaults stay in effect. A successful save writes version 2 and replaces those bytes. `get_capabilities` reports `nvm` true after a blob has been loaded or saved. `nvm_valid` is true when the stored blob passes the version and range checks. NVM writes require a supply above the ClearCore undervoltage lockout; a low supply returns `nvm write failed`.

`configure_network` with `mode:"static"` requires `ip_address` and `netmask` when none are already stored. `gateway` may be omitted. The running Ethernet stack is not changed in place. Call `restart` after saving. `mode:"dhcp"` is the boot default.

Soft limits are checked against the commanded joint position. A target past an enabled side is rejected (`"x above max limit"`, `"x below min limit"`). A velocity or track frame that would travel farther past that side is stopped. `limit_flags` bits are min X, max X, min Y, max Y, min Z, max Z, min A, max A.

A limit switch is active when the input reads high. Motion toward an active switch is rejected, and motion already underway decelerates to a stop on that axis. `get_status` reports the last stop in `travel_limit` until `clear_alerts`. Hardware switches are ignored in test mode. Soft limits are not. DI-6 remains the estop input when `estop_di6` is non-zero; it can also be assigned as a limit.

Only axes in `axis_mask` are enabled and included in `alert_reg`. A disabled motor's `motor_disabled` alert is not reported, same as ClearAI.

`effort` in status and in the state frame is HLFB duty divided by 100, in -1..1. It is a torque proxy, not a calibrated Newton-meter reading. Unknown HLFB duty is reported as 0.

## Stream frames

Little-endian. Every frame:

| Offset | Type | Field |
|--------|------|-------|
| 0 | u8 | magic `0xC5` |
| 1 | u8 | version `1` |
| 2 | u8 | type |
| 3 | u8 | reserved `0` |
| 4 | u16 | payload length |

| Type | Value | Payload |
|------|-------|---------|
| state | 1 | 60 bytes, firmware → host, every 20 ms |
| position | 2 | 20 bytes, absolute joint command |
| velocity | 3 | 20 bytes, joint velocity command |
| heartbeat | 4 | 2 bytes, refreshes the watchdog only |
| track | 5 | 36 bytes, scheduled position and velocity together |

State payload:

| Offset | Type | Field |
|--------|------|-------|
| 0 | u32 | `time_ms` since boot |
| 4 | u16 | sequence |
| 6 | u8 | flags |
| 7 | u8 | `axis_mask` |
| 8 | u32 | `alert_reg` |
| 12 | f32[4] | position (m, m, m, rad) |
| 28 | f32[4] | velocity (m/s, m/s, m/s, rad/s) |
| 44 | f32[4] | effort (HLFB / 100) |
| 60 | u8 | `track_mask` (axes currently in timed tracking) |
| 64 | f32[4] | target position latched by firmware |
| 80 | f32[4] | target velocity latched by firmware |
| 96 | f32[4] | velocity actually given to the step generator |
| 112 | u32[4] | board time when that axis last accepted a track frame |

Target position, target velocity, latch time, generated position, and `time_ms` are taken in the same firmware sample. The position correction is computed only when the track frame is accepted, against the latched position. It is not recomputed against the time-advanced reference while that target is held. Gain stays at `Kp = 8`.

The local streaming diagnostic is tracking relative to the received reference, not relative to the host clock:

`q_ref(time_ms) = q_latched + v_latched * (time_ms - latch_time)`

Compare generated position with `q_ref` at that same `time_ms`. Host-schedule synchronization and physical shaft position during the move are separate measurements.

On the 80 mm, 4 s ramp the peak of that comparison was 0.05 mm in both directions, about eight generated steps. Latch time is a whole millisecond. At 30 mm/s, 1 ms is 0.03 mm, so 0.05 mm is about 1.67 ms — larger than one timestamp tick. Whole-millisecond timestamps make the timing uncertainty significant, but they do not show that the whole residual is a timestamp artifact. The residual is too close to that resolution to justify changing `Kp` from this measurement alone. Finer timestamps would separate timing quantization from tracking error.

Flag bits: `0x01` enabled, `0x02` moving, `0x04` estop, `0x08` fault, `0x10` watchdog.

Position and velocity payloads:

| Offset | Type | Field |
|--------|------|-------|
| 0 | u16 | sequence |
| 2 | u8 | axis mask of commanded joints |
| 3 | u8 | reserved |
| 4 | f32[4] | values (unused axes are ignored) |

Heartbeat payload is a `u16` sequence.

Track payload:

| Offset | Type | Field |
|--------|------|-------|
| 0 | u16 | sequence |
| 2 | u8 | axis mask |
| 3 | u8 | reserved |
| 4 | f32[4] | scheduled position (m, m, m, rad) |
| 20 | f32[4] | feedforward velocity |

The firmware commands `feedforward + clamp(Kp * position_error, ±vel_steps/4)` with `Kp = 8` steps/s per step of error. Absolute `position` frames are unchanged.

A bad magic byte is skipped. A known type with the wrong length is skipped. The host and firmware codecs live in `firmware/RosProtocol.h` and `ros2_ws/src/clearcore_bridge/clearcore_bridge/wire.py`. `host/test_wire.py` checks they match.

## How a goal becomes steps

`StepGenerator::Move()` keeps the current speed and plans a stop at the new endpoint. Calling it on every 20 ms sample would make the motor brake toward each intermediate point.

- A goal that stays unchanged for 40 ms becomes **one** absolute `Move()` at `vel_steps` / `accel_steps`. `set_joints` and `forward_command_controller` use this path.
- A goal that is still changing is followed with `MoveVelocity()`. Each new integer steps/s value is applied. There is no percentage deadband.
- A `track` frame is the timed-execution path: velocity is feedforward and the position error adds a bounded correction. A later absolute `position` frame leaves tracking and uses the settled-move path.
- `clearcore_bridge` samples `FollowJointTrajectory` against `time_from_start` and sends `track` frames. Specified point velocities are the spline boundary conditions. Omitted velocities use the segment slope. A non-zero acceleration limits the change in the streamed velocity. Path tolerances are checked against the sample. The bridge accepts one goal at a time. In `stream_mode:=velocity`, the hardware plugin sends the same `track` frame while the command is changing, then a position hold.

The watchdog trips only while a goal is unfinished or a velocity command is nonzero and the host has been silent for `watchdog_ms`. Reaching the target and then going quiet does not trip. A stream disconnect mid-move trips immediately. `keepalive` does not clear the latch, and hosts must not do it automatically. `clear_alerts` is the recovery; until then position and velocity commands are ignored. The hardware plugin also latches the trip and refuses further writes until the controller activates again, which calls `clear_alerts` before `enable`.

## Coordinated XY, homing, and probing

`move_linear` takes absolute `x`,`y`,`z`,`a` in meters and radians. When both X and Y are in `axis_mask` and enabled, those two axes run on the coordinated planner so the path is one straight line. Otherwise each named axis moves independently. Z and A are always independent. Optional `feed_mps` is the path speed; omitted, the move uses `vel_steps`. `move_arc` is XY only: `x` and `y` are the end point, `i` and `j` are the center offset from the start in meters, and `cw` selects direction. It requires both X and Y. Both calls return `est_ms` and do not wait. `wait_idle` blocks until motion has been still for 20 ms, or until `timeout_ms` (default 60000).

`home` seeks the limit switch configured for `axis` (`x`,`y`,`z`,`a`) and `dir` (`pos` or `neg`). `seek` and `backoff` are in that joint's units (meters or radians). Defaults are a 1 m / 1 rad seek, no backoff, and a 30 s timeout. `zero` defaults to true and sets that joint's generated position to 0 after the seek. The seek ignores soft limits. It still reads the switch when test mode is on.

`probe` seeks until digital input `pin` (1..12) reads `active` (`high` by default, or `low`). The pin cannot be one already assigned as a limit. `zero` defaults to false. A hit stops that axis. Hardware estop aborts the seek. The call blocks, so a following `stop` is not read until it returns.

## XRCE-DDS

The board can publish the four joints to a micro-ROS agent. This does not run a DDS participant on the ClearCore. The agent is the DDS participant. The board is an XRCE-DDS 1.0 client.

`xrce_connect` takes `ip_address` and an optional `port` (default 8888, the micro-ROS agent UDP port). The board binds local UDP **9203**. That stays clear of ClearAI, ClearCNC, and the session ports. `xrce_disconnect` stops it. The agent address is not stored in NVM.

Once the agent accepts the session, the firmware publishes `sensor_msgs/JointState` on `rt/joint_states` at 20 Hz. The names are `joint_x`, `joint_y`, `joint_z`, and `joint_a`, in meters and radians, from the same generated-step sample as the binary state frame. The stamp is time since boot, not a synchronized host clock. `get_status` reports `xrce` as `off`, `connecting`, `creating`, or `streaming`.

Commands still use the session and the binary stream. The XRCE client only publishes.
