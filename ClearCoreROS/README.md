# ClearCoreROS

**Experimental.** Not certified for production or safety-critical use. Keep a physical estop in the circuit.

ClearCore firmware and a ROS 2 host that expose ClearPath motors M0–M3 as joints. The board runs the step/direction generator. ROS 2 runs on the host (a Jetson or other Linux machine). The wire format is [PROTOCOL.md](PROTOCOL.md).

This follows the ClearAI motor setup: Step and Direction, HLFB ASG-Position with measured torque at 482 Hz, pose taken from `PositionRefCommanded()`, and alert bits reported only for axes in `axis_mask`.

## Layout

| Path | Role |
|------|------|
| `firmware/` | Microchip Studio project `ClearCoreROS.atsln` |
| `PROTOCOL.md` | Session methods and binary frames |
| `host/ccros_cli.py` | Bench client (no ROS install) |
| `ros2_ws/src/clearcore_bridge` | `JointState`, Trigger services, `FollowJointTrajectory` |
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

Flashing this image replaces whatever firmware is on the board (ClearAI uses the same bootloader and a different application).

Ports: session **9200**, joint stream **9201**, discovery **9202**.

## Bench (one motor on M0)

X is meters. `0.01` is 10 mm. Default `axis_mask` is `3` (XY); a single motor on M0 needs mask `1`. `test-mode` skips the DI-6 estop input.

```powershell
cd ClearCoreROS
python host\ccros_cli.py --host 172.16.82.113 discover
python host\ccros_cli.py --host 172.16.82.113 configure --axis-mask 1 --vel 27000 --accel 250000 --decel 250000
python host\ccros_cli.py --host 172.16.82.113 test-mode --on
python host\ccros_cli.py --host 172.16.82.113 enable
python host\ccros_cli.py --host 172.16.82.113 move --x 0.01
python host\ccros_cli.py --host 172.16.82.113 disable
```

`move --stream` sends the same target as a binary position frame on port 9201. Only one session client and one stream client can be connected.

Check the frame codec without hardware:

```powershell
python host\test_wire.py
```

## ROS 2

Target is ROS 2 Jazzy on Linux. The hardware plugin uses POSIX sockets. Build on the ROS machine:

```bash
cd ClearCoreROS/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

### Trajectory action

`clearcore_bridge` connects, enables, publishes `/joint_states`, and serves `follow_joint_trajectory`. The action follows `time_from_start`, including point velocities and accelerations when they are set, and checks path tolerances while the arm is moving. Streaming error is generated position versus the received reference advanced to the board sample time (`q_latched + v_latched * (time_ms - latch_time)`), not versus the host clock. A second goal is rejected until the first finishes. Joint state stamps are the time the sample was received, and a stale, disabled, faulted, or tripped sample is not treated as a finished move. For the M0 bench, pass `axis_mask:=1`. `enable` fails unless HLFB is asserted; pass `test_mode:=true` on a bare motor.

```bash
ros2 launch clearcore_bridge bridge.launch.py host:=172.16.82.113 axis_mask:=1
ros2 service call /clearcore_bridge/enable std_srvs/srv/Trigger
```

Services on the node: `enable`, `disable`, `stop`, `estop`, `clear_alerts`.

### ros2_control

`clearcore_hardware` exports a position command and position / velocity / effort state. Effort is HLFB duty scaled to -1..1. The launch file starts `joint_state_broadcaster` and `forward_position_controller` for `joint_x` and `joint_y` (`axis_mask` 3).

```bash
ros2 launch clearcore_hardware hardware.launch.py host:=172.16.82.113
```

`stream_mode` is `position` by default: a settled command becomes one trapezoidal move. Set `stream_mode` to `velocity` in the xacro when a `joint_trajectory_controller` is streaming interpolated samples. The plugin sends `dq/dt` while the command is changing, then a position hold on the next cycle. A watchdog flag latches the plugin; writes stop until the hardware is activated again.

URDF `<limit>` tags are not enforced on the board. Soft limits are not implemented yet.

## Safety

- DI-6 defaults to an active-low estop (`estop_di6` 1), same as ClearAI. `set_test_mode` is bench-only.
- `stop` decelerates. `estop` and `disable` drop the enable line.
- A host that disappears mid-move trips the watchdog (default 500 ms) or, if the stream socket drops, stops immediately. Clearing that latch is `clear_alerts`, not a keepalive.
- Do not connect the CLI and a ROS node at the same time. Each port accepts one client.
