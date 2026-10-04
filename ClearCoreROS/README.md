# ClearCoreROS

**Experimental.** Not certified for production or safety-critical use. Keep a physical estop in the circuit.

ClearCore firmware and a ROS 2 host that expose ClearPath motors M0–M3 as joints. The board runs the step/direction generator. ROS 2 runs on the host (a Jetson or other Linux machine). The wire format is [PROTOCOL.md](PROTOCOL.md).

The motor setup matches ClearAI: Step and Direction, HLFB ASG-Position with measured torque at 482 Hz, pose from `PositionRefCommanded()`, and alert bits reported only for axes in `axis_mask`. Reported position is generated steps, not a shaft encoder.

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
| `firmware/` | Microchip Studio project `ClearCoreROS.atsln` |
| `PROTOCOL.md` | Session methods, binary frames, NVM, limits, XRCE |
| `host/ccros_cli.py` | Bench client. No ROS install. |
| `host/test_wire.py`, `host/test_safety.py` | Codec and bridge checks. No board. |
| `host/xrce_check.py` | Accepts the XRCE session and prints `JointState` |
| `host/test_m0_*.py` | Live M0 benches. Set `HOST` to the board address. |
| `ros2_ws/src/clearcore_bridge` | `/joint_states`, Trigger services, `FollowJointTrajectory` |
| `ros2_ws/src/clearcore_hardware` | `ros2_control` `SystemInterface` |

## Firmware

Open `firmware/ClearCoreROS.atsln` in Microchip Studio 7, build, and flash with `Tools/flash_clearcore.cmd`. Command-line build from the Debug directory, same toolchain as ClearAI:

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

`configure` and `test-mode` are saved in the same blob and restored on boot. `reset-config` restores the compile defaults and clears the blob. The motors must be disabled for `reset-config` and for changes to `axis_mask`, `steps_per_rev`, or `pitch_mm`. NVM writes need a supply above the ClearCore undervoltage lockout.

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

`move_arc` requires both X and Y. `i` and `j` are the center offset from the start, in meters. Two independent joint moves are not that arc. `feed_mps` is meters per second along the path.

Soft limits and limit switches are `configure` fields, in joint units. `pos-lim-x 8` assigns a digital input. `0` clears it. `clear-limits` clears every soft limit and every switch assignment. Limits are stored in NVM version 2.

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

`xrce-connect` publishes `sensor_msgs/JointState` on `rt/joint_states` at 20 Hz. The board binds UDP 9203 and sends to the agent. The stamp is time since boot. Commands stay on the session and the binary stream.

```powershell
python host\xrce_check.py 9204
python host\ccros_cli.py --host 172.16.82.114 xrce-connect --ip 172.16.82.199 --port 9204
python host\ccros_cli.py --host 172.16.82.114 xrce-disconnect
```

`xrce_check.py` answers the XRCE session and prints the joint sample. A micro-ROS agent on port 8888 is what places that topic on a ROS graph. `get_status` reports `xrce` as `off`, `connecting`, `creating`, or `streaming`. The agent address is not stored in NVM.

## ROS 2

Target is ROS 2 Jazzy on Linux. The hardware plugin uses POSIX sockets. Build on the ROS machine:

```bash
cd ClearCoreROS/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

### Trajectory action

`clearcore_bridge` connects, enables, publishes `/joint_states`, and serves `follow_joint_trajectory`. The action follows `time_from_start`, including point velocities and accelerations when they are set, and checks path tolerances while the arm is moving. Streaming error is generated position versus the received reference advanced to the board sample time, not versus the host clock. A second goal is rejected until the first finishes. Joint state stamps are the time the sample was received. A stale, disabled, faulted, or tripped sample is not a finished move. `enable` fails unless HLFB is asserted. Pass `test_mode:=true` on a bare motor.

```bash
ros2 launch clearcore_bridge bridge.launch.py host:=172.16.82.114 axis_mask:=3
ros2 service call /clearcore_bridge/enable std_srvs/srv/Trigger
```

Services on the node: `enable`, `disable`, `stop`, `estop`, `clear_alerts`.

### ros2_control

`clearcore_hardware` exports a position command and position, velocity, and effort state. Effort is HLFB duty scaled to -1..1. The launch file starts `joint_state_broadcaster` and `forward_position_controller` for `joint_x` and `joint_y`.

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
