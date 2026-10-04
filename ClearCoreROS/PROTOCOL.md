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

DHCP is used when the link is up. If DHCP fails, the address falls back to `192.168.0.109`. Static IP is not stored yet.

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
| `get_capabilities` | — | protocol, ports, joint names, units, `axis_mask` |
| `get_config` | — | mechanics, watchdog, estop mode, test mode |
| `get_status` | — | flags, `alert_reg`, `alerts`, position/velocity/effort |
| `configure` | see below | `{"ok":true}` |
| `set_test_mode` | `{"on":true}` | test mode skips DI-6 and the HLFB wait |
| `enable` / `disable` | — | enable waits up to 500 ms for HLFB, then continues |
| `stop` | — | decelerate, drop the goal, stay enabled |
| `estop` | — | abrupt stop and disable |
| `clear_alerts` | — | `ClearAlerts()`, clear watchdog; estop stays if DI-6 is still faulted |
| `keepalive` | — | clears a watchdog latch |
| `set_joints` | `x`,`y`,`z`,`a` in meters / radians | absolute goal for the named joints |

`configure` fields:

| Field | Meaning |
|-------|---------|
| `axis_mask` | bits 0..3 = X Y Z A. Range 1..15. Requires motors disabled. |
| `steps_per_rev` | one number (all axes) or four numbers. Requires disabled. |
| `pitch_mm` | linear pitch. Axis A is always revolute and ignores pitch. Requires disabled. |
| `vel_steps`, `accel_steps`, `decel_steps` | step generator limits. Applied immediately. |
| `watchdog_ms` | `0` disables. Default 500. |
| `estop_di6` | `0` off, `1` fault when DI-6 is low (default), `2` fault when DI-6 is high. |

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

Flag bits: `0x01` enabled, `0x02` moving, `0x04` estop, `0x08` fault, `0x10` watchdog.

Position and velocity payloads:

| Offset | Type | Field |
|--------|------|-------|
| 0 | u16 | sequence |
| 2 | u8 | axis mask of commanded joints |
| 3 | u8 | reserved |
| 4 | f32[4] | values (unused axes are ignored) |

Heartbeat payload is a `u16` sequence.

A bad magic byte is skipped. A known type with the wrong length is skipped. The host and firmware codecs live in `firmware/RosProtocol.h` and `ros2_ws/src/clearcore_bridge/clearcore_bridge/wire.py`. `host/test_wire.py` checks they match.

## How a goal becomes steps

`StepGenerator::Move()` keeps the current speed and plans a stop at the new endpoint. Calling it on every 20 ms sample would make the motor brake toward each intermediate point.

- A goal that stays unchanged for 40 ms becomes **one** absolute `Move()` at `vel_steps` / `accel_steps`. This is the path used by `set_joints`, `forward_command_controller`, and the FollowJointTrajectory bridge (one frame per trajectory point).
- A goal that is still changing is followed with `MoveVelocity()`, and the velocity command is updated only when it changes by about 8% (or 200 steps/s). The cruise speed is the configured `vel_steps`, not the ROS trajectory's time parameterization.
- A velocity frame runs that joint in velocity mode until a later position frame replaces it. `stream_mode:=velocity` on the hardware plugin sends `dq/dt` while a `joint_trajectory_controller` command is moving, then one position frame to land.

The watchdog trips only while a goal is unfinished or a velocity command is nonzero and the host has been silent for `watchdog_ms`. Reaching the target and then going quiet does not trip. A stream disconnect mid-move trips immediately. Clearing the latch requires `keepalive` or `clear_alerts`.

## Not in this scaffold

- NVM (static IP, saved mechanics). `configure` is RAM-only.
- Soft travel limits and DI limit switches.
- Homing, probing, arcs, and the XY coordinated planner. Joints move independently.
- micro-ROS / XRCE-DDS. The joint mapping above is what a future XRCE transport would publish.
