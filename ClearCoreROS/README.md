# ClearCoreROS

**Experimental.** Not certified for production or safety-critical use. Keep a physical estop in the circuit.

ClearCore firmware and a ROS 2 host that expose ClearPath motors M0–M3 as joints. The board runs the step/direction generator. ROS 2 runs on the host (a Jetson or other Linux machine). The wire format is [PROTOCOL.md](PROTOCOL.md).

Start with [docs/FIRST_MOTOR.md](docs/FIRST_MOTOR.md) and the firmware file in [releases/](releases/README.md). That path is one ClearPath on M0, the released `.bin`, and `host/ccros_cli.py`. It does not need Microchip Studio or ROS.

The motor setup matches ClearAI: Step and Direction, HLFB ASG-Position with measured torque at 482 Hz, pose from `PositionRefCommanded()`, and alert bits reported only for axes in `axis_mask`. Reported position is generated steps, not a shaft encoder.

## Choose a setup

Three host programs talk to the same firmware. Only one of them should own motion, and only one of them should be the `/joint_states` source you are reading.

| Setup | What you run | Who commands motion | Who publishes `/joint_states` |
|-------|----------------|---------------------|-------------------------------|
| First motor, no ROS | `host/ccros_cli.py` | The session on TCP 9200 | Nobody, unless you start an agent |
| Python bridge | `clearcore_bridge` | `follow_joint_trajectory` on that node, over the session and the binary stream | The bridge, for joints in `axis_mask`. Stamps are host reception time. |
| `ros2_control` | `clearcore_hardware` plus a controller | The controller's command interface, over the same session and stream | `joint_state_broadcaster`, if the launch starts it |
| XRCE telemetry | `xrce_connect` and a micro-ROS agent | Nobody. This client only publishes. | The agent node `clearcore_ros`, all four joints, in metres and radians. The stamp is time since boot. |

The session port accepts one TCP client. The bridge and the `ros2_control` plugin cannot both hold it. Stop the bridge and `joint_state_broadcaster` before treating `/joint_states` as the agent topic.

Defaults. `name_x` through `name_a`, `rotary_*`, `direction_*`, `gear_*`, and `offset_*` change the map. The bridge reads `names` from `get_config` after it connects.

| Joint | Motor | Unit |
|-------|-------|------|
| `joint_x` | M0 | meters |
| `joint_y` | M1 | meters |
| `joint_z` | M2 | meters |
| `joint_a` | M3 | radians |

Zero is the pose at boot, or the pose after `home` with `zero` true. `0.01` m is 10 mm. At the default 800 steps/rev and 5 mm pitch, that is 1600 steps.

## Layout

| Path | Role |
|------|------|
| `docs/FIRST_MOTOR.md` | One motor, from the released firmware image, with no ROS install |
| `releases/` | Versioned firmware binary, checksum, and what that revision was run on |
| `firmware/` | Microchip Studio project `ClearCoreROS.atsln` |
| `PROTOCOL.md` | Session methods, binary frames, NVM, limits, XRCE |
| `host/ccros_cli.py` | Bench client. No ROS install. |
| `host/test_wire.py`, `host/test_safety.py` | Codec and bridge checks. No board. |
| `host/xrce_check.py` | Accepts the XRCE session and prints `JointState` |
| `host/test_m0_*.py` | Live M0 benches. Set `HOST` to the board address. |
| `ros2_ws/src/clearcore_bridge` | `/joint_states`, Trigger services, `FollowJointTrajectory` |
| `ros2_ws/src/clearcore_hardware` | `ros2_control` `SystemInterface` |

## Firmware

The first-motor image is [releases/ClearCoreROS-0.1.0.bin](releases/ClearCoreROS-0.1.0.bin). Flash instructions and the tested-revision list are in [releases/README.md](releases/README.md) and [docs/FIRST_MOTOR.md](docs/FIRST_MOTOR.md).

To build instead of using that file, open `firmware/ClearCoreROS.atsln` in Microchip Studio 7 and flash with `Tools/flash_clearcore.cmd`. Command-line build from the Debug directory, same toolchain as ClearAI:

```powershell
cd ClearCoreROS\firmware\Debug
& "C:\Program Files (x86)\Atmel\Studio\7.0\shellutils\make.exe" all
```

Output: `firmware/Debug/ClearCoreROS.bin`.

```powershell
..\..\..\Tools\flash_clearcore.cmd ClearCoreROS.bin
```

Flashing this image replaces whatever application is on the board. ClearAI uses the same bootloader. User NVM survives the flash. A ClearAI `CAIC` blob is left in place and is not applied. The first successful `configure`, `set_test_mode`, or `configure_network` writes a `CROS` blob over those bytes.

## Ports and network

| Port | Role |
|------|------|
| **9200** TCP | JSON-RPC session, one client |
| **9201** TCP | Binary joint state and commands, one client |
| **9202** UDP | `CLEARCORE_ROS_DISCOVER?` |
| **9203** UDP | XRCE-DDS client. The agent listens on the host. |

These stay clear of ClearAI (9100–9102) and ClearCNC (8888, 8889, 10040).

DHCP is the compile-time default. If DHCP fails, the address falls back to `192.168.0.109`. `configure-network` saves `dhcp` or `static` in NVM. The new address is used after `restart`. `get_config` reports the saved address. While the mode is `dhcp`, that field stays `0.0.0.0` and the live address is the lease.

`configure` and `test-mode` are saved in the same blob and restored on boot. `reset-config` restores the compile defaults and clears the blob. The motors must be disabled for `reset-config` and for changes to `axis_mask`, `steps_per_rev`, `pitch_mm`, `rotary_*`, `direction_*`, `gear_*`, or `offset_*`. A joint name can change while enabled. NVM writes need a supply above the ClearCore undervoltage lockout. Version 3 of the blob stores the joint map. A version 1 or version 2 blob still loads, and the map stays at the defaults until the next save.

The bench board is static at **172.16.82.114**, netmask `255.255.255.0`, gateway `172.16.82.1`.

## Bench

M0 and M1 are installed. `axis_mask` 3 enables X and Y. HLFB asserts on both, so test mode stays off. A single-motor bench uses `--axis-mask 1`. `test-mode --on` skips DI-6 and the HLFB check.

```powershell
cd ClearCoreROS
python host\ccros_cli.py --host 172.16.82.114 discover
python host\ccros_cli.py --host 172.16.82.114 config
python host\ccros_cli.py --host 172.16.82.114 enable
python host\ccros_cli.py --host 172.16.82.114 move --x 0.01
python host\ccros_cli.py --host 172.16.82.114 disable
```

`move --stream` sends the same target as a binary position frame on port 9201. Only one session client and one stream client can be connected.

A coordinated line and arc use `call`. `wait_idle` blocks until the path has been still. Give the session timeout longer than the move.

```powershell
python host\ccros_cli.py --host 172.16.82.114 --timeout 15 call move_linear --params "{\"x\":0.04,\"y\":0.0,\"feed_mps\":0.03}"
python host\ccros_cli.py --host 172.16.82.114 --timeout 15 call wait_idle --params "{\"timeout_ms\":15000}"
python host\ccros_cli.py --host 172.16.82.114 --timeout 20 call move_arc --params "{\"x\":-0.04,\"y\":0.0,\"i\":-0.04,\"j\":0.0,\"cw\":false,\"feed_mps\":0.03}"
python host\ccros_cli.py --host 172.16.82.114 --timeout 20 call wait_idle --params "{\"timeout_ms\":20000}"
```

`move_arc` requires both X and Y, both linear, with the same gear. `i` and `j` are the center offset from the start, in joint units. Two independent joint moves are not that arc. `feed_mps` is joint units per second along the path. `est_ms` is the longer axis component at that feed, not the path duration.

`configure` also takes `--name-x`, `--rotary-x`, `--direction-x`, `--gear-x`, and `--offset-x`, and the same suffixes for `y`, `z`, and `a`. The bridge reads `names` from `get_config`. The released `0.1.0` image does not have these fields.

On the bench, flashing the joint-map image kept the saved static address `172.16.82.114` and the stored limits from the version 2 blob. With the motors disabled, M0 was set to the name `shoulder`, direction `-1`, gear `2`, and offset `0.01` m. Status reported `0.010` m at zero steps. After enable, a move to `0.012` m and back to `0.010` m matched those positions. The map was then restored to `joint_x` through `joint_a`, direction `+1`, gear `1`, and offset `0`, and status reported `0`.

A later run at those defaults moved a 30 mm square at 30 mm/s, one axis at a time, then a coordinated diagonal to (30 mm, 30 mm) and back. Every endpoint matched at 0.001 mm. During the diagonal the largest |Y−X| was 0.007 mm. Each diagonal took about 1.6 s while `est_ms` stayed 1000. The motors were left disabled, test mode off, with no alerts.

Soft limits and limit switches are `configure` fields, in joint units. `pos-lim-x 8` assigns a digital input. `0` clears it. `clear-limits` clears every soft limit and every switch assignment. Limits are stored from NVM version 2 onward.

```powershell
python host\ccros_cli.py --host 172.16.82.114 configure --min-x -0.05 --max-x 0.05
python host\ccros_cli.py --host 172.16.82.114 configure --clear-limits
```

`home` seeks the switch configured for that axis and direction. `probe` seeks until another digital input trips. Both block. `zero` on `home` defaults to true and sets that joint's generated position to 0.

```powershell
python host\ccros_cli.py --host 172.16.82.114 call home --params "{\"axis\":\"x\",\"dir\":\"neg\",\"seek\":0.05,\"backoff\":0.001}"
python host\ccros_cli.py --host 172.16.82.114 call probe --params "{\"axis\":\"x\",\"dir\":\"pos\",\"pin\":8,\"seek\":0.02}"
```

Timed tracking compares generated position with `q_latched + v_latched * (time_ms - latch_time)` at the same board timestamp. On the 80 mm, 4 s ramp that peak was 0.05 mm in both directions. `Kp` stays 8. Host-clock alignment and shaft-encoder position during the move are separate measurements. See PROTOCOL.md.

## XRCE-DDS

`xrce-connect` publishes `sensor_msgs/JointState` on `rt/joint_states` at 20 Hz. The board binds UDP 9203 and sends to the agent. X, Y, and Z are meters. A is radians. The stamp is time since boot. The body has no CDR encapsulation; Fast DDS adds it. Commands stay on the session and the binary stream.

```powershell
python host\xrce_check.py 9204
python host\ccros_cli.py --host 172.16.82.114 xrce-connect --ip 172.16.82.199 --port 9204
python host\ccros_cli.py --host 172.16.82.114 xrce-disconnect
```

`xrce_check.py` answers the XRCE session and prints the joint sample. It is not a DDS bridge. A micro-ROS agent places `/joint_states` on the ROS graph from the node `clearcore_ros`. The login header uses session id `0x80`, and the client queues the agent's reply datagrams. `get_status` reports `xrce` as `off`, `connecting`, `creating`, or `streaming`. The agent address is not stored in NVM.

With the bridge and `joint_state_broadcaster` stopped, that agent topic followed both axes from 0 to 0.030 m and back to 0. The board endpoints matched and there were no alerts. Positions on this topic are meters, not millimeters.

## ROS 2

Target is ROS 2 Jazzy on Linux. The hardware plugin uses POSIX sockets. Build on the ROS machine:

```bash
cd ClearCoreROS/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

### Trajectory action

`clearcore_bridge` connects, enables, publishes `/joint_states`, and serves `follow_joint_trajectory`. The action follows `time_from_start`. Point velocities are spline boundaries. When accelerations are set they are quintic boundaries, not a cap on the feedforward. Path tolerance is generated position versus the time-advanced received reference in that state frame, not versus the host schedule. Action feedback pairs the current host schedule sample with the latest received position, so that gap is not the 0.05 mm local tracking result. A second goal is rejected until the first finishes. A result `error_code` of 0 is `SUCCESSFUL`. Joint state stamps are the time the sample was received. A stale, disabled, faulted, or tripped sample is not a finished move. `enable` fails unless HLFB is asserted. Pass `test_mode:=true` on a bare motor. Launch always writes that parameter, including false, because test mode is stored in NVM.

A two-axis goal on the bench ran out and back to the origin. X and Y stayed within about 0.01 mm of each other in the feedback printout. The action returned success. That run does not exercise the XRCE publisher.

```bash
ros2 launch clearcore_bridge bridge.launch.py host:=172.16.82.114 axis_mask:=3
ros2 service call /clearcore_bridge/enable std_srvs/srv/Trigger
```

Services on the node: `enable`, `disable`, `stop`, `estop`, `clear_alerts`.

### ros2_control

`clearcore_hardware` exports a position command and position, velocity, and effort state. Effort is HLFB duty scaled to -1..1. Joint names come from the hardware parameters `name_x`, `name_y`, `name_z`, and `name_a` (defaults `joint_x` through `joint_a`). `rotary_*`, `direction_*`, `gear_*`, and `offset_*` are sent with `configure` on activate. The launch file starts `joint_state_broadcaster` and `forward_position_controller` for `joint_x` and `joint_y`.

```bash
ros2 launch clearcore_hardware hardware.launch.py host:=172.16.82.114
```

`stream_mode` is `position` by default: a settled command becomes one trapezoidal move. Set `stream_mode` to `velocity` in the xacro when a `joint_trajectory_controller` is streaming interpolated samples. The plugin sends `dq/dt` with the position while the command is changing, then a position hold. A watchdog flag latches the plugin. Writes stop until the hardware is activated again, which calls `clear_alerts` before `enable`.

URDF `<limit>` tags are not copied onto the board. Set soft limits with `configure`.

## Checks without a board

```powershell
python host\test_wire.py
python host\test_safety.py
```

`test_wire.py` checks the binary frame codec against `firmware/RosProtocol.h`. `test_safety.py` checks the bridge goal gate, trajectory sampling, and stale-state rules.

## Safety

- DI-6 defaults to an active-low estop (`estop_di6` 1). `test-mode` is bench-only. It skips that input and the HLFB wait. It also ignores limit switches. Soft limits still apply.
- `stop` decelerates and drops the goal. `estop` and `disable` drop the enable line.
- A host that disappears mid-move trips the watchdog (default 500 ms). A dropped stream socket stops immediately. Clearing that latch is `clear_alerts`, not a keepalive.
- `travel_limit` holds the last soft-limit or switch stop until `clear_alerts`.
- Do not connect the CLI and a ROS node at the same time. Each of ports 9200 and 9201 accepts one client.
