# ClearCoreROS 0.1.0

Prebuilt firmware for the first-motor guide. You do not need Microchip Studio to flash this file.

| | |
|--|--|
| File | `ClearCoreROS-0.1.0.bin` |
| SHA-256 | `781b4705e0bb766c923245069b2c5fb71d6d4c491800844dde94eef15d6be611` |
| Protocol | `1.0` (`CCROS_PROTOCOL_VERSION`) |
| Git | `fb52394` |
| Host package | `clearcore_bridge` `0.1.0` |

The image is the firmware tree at that git revision. `host/ccros_cli.py` from the same revision is the matching bench client. It needs Python 3.10 or newer and does not need ROS.

Flash instructions are in [../docs/FIRST_MOTOR.md](../docs/FIRST_MOTOR.md).

## What this revision was run on

These are bench checks, not a continuous-integration suite. The Python bridge and `ros2_control` are different programs.

| Check | Path | Result on the bench |
|-------|------|---------------------|
| Discover, configure one axis, enable, 10 mm move, return | `ccros_cli.py` session | Used for bring-up. Positions are generated steps. |
| Two-axis `follow_joint_trajectory`, out and back, result `error_code` 0 | Python bridge | Passed. Feedback is not the board-local tracking measurement. |
| `/joint_states` while both axes moved 0 → 0.030 m → 0 | XRCE through a micro-ROS agent, publisher node `clearcore_ros` only | Passed. Board endpoints matched. No alerts. |
| `ros2_control` `forward_position_controller` or `joint_trajectory_controller` | hardware plugin | Not run as a hardware goal. |
| Cancellation, connection loss, restart, homing, recovery | either ROS path | Not an automated matrix. Homing and probing exist on the session API. |
