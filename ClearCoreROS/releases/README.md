# ClearCoreROS releases

Prebuilt firmware for the first-motor guide. You do not need Microchip Studio to flash the current file.

## 0.1.2

| | |
|--|--|
| File | `ClearCoreROS-0.1.2.bin` |
| SHA-256 | `85ed5e5bba52e6bc40b0188f8cb2890b7c4e10394ecc93be87cff74948c37f34` |
| Length | 285052 |
| Protocol | `1.0` (`CCROS_PROTOCOL_VERSION`) |
| Firmware sources | `5ecef62` |
| Host package | `clearcore_bridge` `0.1.0` |

This image keeps the 0.1.1 joint map and NVM version 3. It withholds XRCE `rt/joint_states` until `TIMESTAMP_REPLY`. Stamps are the agent's system clock, not ROS `/clock`. Until then `get_status` reports `xrce_time` as `unsync`. `host/ccros_cli.py` from this tree is the matching bench client.

Flash instructions are in [../docs/FIRST_MOTOR.md](../docs/FIRST_MOTOR.md).

### What this revision was run on

These are bench checks, not a continuous-integration suite. The Python bridge and `ros2_control` are different programs.

| Check | Path | Result on the bench |
|-------|------|---------------------|
| Flash over a version 3 blob from 0.1.1 | session `get_status` | Static `172.16.82.114`, `axis_mask` 3, motors disabled, `test_mode` false. |
| XRCE publish withheld until `TIMESTAMP_REPLY`, then three samples | `host/xrce_check.py` (0.4 s delayed reply) | No `JointState` before the reply. After it, epoch stamps ~50 ms apart (20 Hz period, not a clock-offset measurement). Status went `streaming` / `unsync`, then `synced`. Motors left disabled. |
| Mapping, first-motor 10 mm, two-axis `joint_trajectory_controller` | session / `ros2_control` | Not re-run on this `.bin`. Motion path is unchanged from 0.1.1. |
| `ros2_control` cancellation, connection loss, restart, homing, recovery | hardware plugin | Not an automated matrix. Homing and probing exist on the session API. |

## 0.1.1

| | |
|--|--|
| File | `ClearCoreROS-0.1.1.bin` |
| SHA-256 | `9d9d3f2fbe95eb838649e65f0a2a6d52cef326d6d99f7b9da56592a22681bc2b` |
| Length | 283420 |
| Protocol | `1.0` (`CCROS_PROTOCOL_VERSION`) |
| Firmware sources | `0bf5dbf` |
| Host package | `clearcore_bridge` `0.1.0` |

This image adds the joint map: `name_*`, `rotary_*`, `direction_*`, `gear_*`, and `offset_*`. NVM version 3 stores that map. Boot still loads a version 1 or version 2 blob and keeps its network settings and limits. The map stays at the defaults until the next save. The first-motor guide uses 0.1.2.

Flash instructions are in [../docs/FIRST_MOTOR.md](../docs/FIRST_MOTOR.md).

### What this revision was run on

These are bench checks, not a continuous-integration suite. The Python bridge and `ros2_control` are different programs.

| Check | Path | Result on the bench |
|-------|------|---------------------|
| Flash over a version 2 blob | session `get_config` | Static `172.16.82.114` and the stored limits remained. `nvm_version` was 2 until the next `configure`. |
| M0 renamed to `shoulder`, direction `-1`, gear `2`, offset `0.01` m, then a move to `0.012` m and back | session | Reported `0.010` m at zero steps, then `0.012` m and `0.010` m. Map restored to the defaults. |
| 30 mm square, one axis at a time, then a coordinated diagonal out and back, 30 mm/s | `move_linear` | Endpoints matched at 0.001 mm. Peak \|Y−X\| on the diagonal was 0.007 mm. `est_ms` stayed 1000; the diagonal took about 1.6 s. |
| First-motor 10 mm move on M0 with `test_mode` false, then return | `host/test_m0_live.py` against this `.bin` | Position lines matched the guide (`0.0100` then `0.0000`). `axis_mask` 3, limit flags, and test mode were restored. Motors left disabled. |
| Two-axis `joint_trajectory_controller` goal, (0.03, 0.03) m then origin | `trajectory.launch.py` and `send_trajectory_goal.py` | Result `error_code` 0. Peak |Y−X| 0.013 mm. End −0.09 mm. Motors left disabled. |
| `ros2_control` cancellation, connection loss, restart, homing, recovery | hardware plugin | Not an automated matrix. Homing and probing exist on the session API. |

## 0.1.0

| | |
|--|--|
| File | `ClearCoreROS-0.1.0.bin` |
| SHA-256 | `781b4705e0bb766c923245069b2c5fb71d6d4c491800844dde94eef15d6be611` |
| Length | 274052 |
| Protocol | `1.0` |
| Firmware sources | `fb52394` |
| Host package | `clearcore_bridge` `0.1.0` |

This file has no per-axis joint name, rotary flag, direction, gear, or offset. The first-motor guide uses 0.1.2.

| Check | Path | Result on the bench |
|-------|------|---------------------|
| Discover, configure one axis, enable, 10 mm move, return | `ccros_cli.py` session | Used for bring-up. Positions are generated steps. |
| Two-axis `follow_joint_trajectory`, out and back, result `error_code` 0 | Python bridge | Passed. Feedback is not the board-local tracking measurement. |
| `/joint_states` while both axes moved 0 → 0.030 m → 0 | XRCE through a micro-ROS agent, publisher node `clearcore_ros` only | Passed. Board endpoints matched. No alerts. |
| `ros2_control` `forward_position_controller` or `joint_trajectory_controller` | hardware plugin | Not run as a hardware goal. |
| Cancellation, connection loss, restart, homing, recovery | either ROS path | Not an automated matrix. Homing and probing exist on the session API. |
