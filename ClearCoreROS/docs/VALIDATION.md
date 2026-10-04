# ClearCoreROS validation matrix

Versioned results for the ClearCoreROS ROS 2 host and firmware. A passing Jazzy CI job is **automated with simulated hardware**. It must not be read as proof that physical homing or fault recovery ran.

## Revisions in this file

| Field | Value |
|-------|--------|
| Firmware image | `ClearCoreROS-0.1.3.bin` |
| Firmware sources | `4eeab56` (empty `JointState.effort`, `rt/hlfb_duty`) |
| Host packages | `clearcore_bridge` `0.1.0`, `clearcore_hardware` `0.1.0` |
| Protocol | `1.0` |
| CI | `.github/workflows/clearcore-ros-jazzy.yml` |

Earlier firmware rows keep the image that was on the bench.

## Result labels

| Label | Meaning |
|-------|---------|
| `automated-sim` | Ran in CI or `host/run_ci_tests.py` against simulated sockets, frames, or JSON. No ClearCore. |
| `hardware-bench` | Ran on the lab board (static `172.16.82.114`, M0/M1 as installed). |
| `not-tested` | Required by the adoption list and not yet executed on that path. |

## Python bridge

| Scenario | Expected behavior | Path | Firmware | Host | Result | Evidence |
|----------|-------------------|------|----------|------|--------|----------|
| Cancellation | Second goal rejected; only the owner may cancel; cancel is not `motion_succeeded` | `GoalGate` / `host/test_safety.py` | n/a (host) | 0.1.0 | `automated-sim` | `test_second_goal_is_rejected_and_cancel_is_not_shared`, `test_cancel_releases_the_gate_and_does_not_count_as_success` |
| Cancellation on hardware | Cancel in-flight `follow_joint_trajectory` stops motion and does not report success | `clearcore_bridge` action | 0.1.0+ | 0.1.0 | `not-tested` | No recorded cancel-during-move on the bench |
| Connection loss | Stream EOF is `ConnectionError`, not an empty update | `StreamClient.read` / `test_safety.py` | n/a | 0.1.0 | `automated-sim` | `test_stream_eof_is_disconnect` |
| Connection loss on hardware | Dropped TCP 9201 mid-move stops immediately | binary stream | 0.1.3 | 0.1.0 | `not-tested` | Documented in PROTOCOL.md; no captured disconnect log |
| Board/host restart | Missing or stale state is not a finished move; reconnect `disable` / `clear_alerts` / `configure` / `enable` | `test_safety.py` + `bridge_node._ensure_connected` | n/a | 0.1.0 | `automated-sim` | `test_host_restart_needs_a_fresh_state_sample` |
| Board restart on hardware | Flash keeps NVM; session answers after reboot | `get_status` after flash | 0.1.3 | 0.1.0 | `hardware-bench` | After 0.1.3 flash: `axis_mask` 3, static `172.16.82.114`, motors disabled, `test_mode` false |
| Homing | `home` is a session method, not the trajectory action | JSON-RPC encode / `test_safety.py` | n/a | 0.1.0 | `automated-sim` | `test_home_is_a_session_method_not_a_trajectory_success` |
| Homing on hardware | Seek a limit, optional zero | session `home` | 0.1.3 | 0.1.0 | `not-tested` | API exists; no bench seek recorded in this matrix |
| Recovery | Watchdog blocks success until `clear_alerts`; keepalive is not recovery | `state_block_reason` / `test_safety.py` | n/a | 0.1.0 | `automated-sim` | `test_watchdog_blocks_until_clear_alerts_not_keepalive` |
| Recovery on hardware | After a tripped watchdog, `clear_alerts` then `enable` | session | 0.1.3 | 0.1.0 | `not-tested` | No captured watchdog trip |
| Two-axis `follow_joint_trajectory` out and back | Result `error_code` 0 | Python bridge | 0.1.0 | 0.1.0 | `hardware-bench` | releases/README 0.1.0 row; feedback is not the 0.05 mm local tracking residual |
| `/joint_states` effort empty, `/hlfb_duty` published | Names and stamps match; effort array empty on `/joint_states` | mocked `ClearCoreBridge._publish_state` | n/a | 0.1.0 | `automated-sim` | Review of `4eeab56`; code in `bridge_node.py` |

## ros2_control

| Scenario | Expected behavior | Path | Firmware | Host | Result | Evidence |
|----------|-------------------|------|----------|------|--------|----------|
| Cancellation | `GoalStatus.STATUS_CANCELED` is failure even when `error_code` is 0 | `send_trajectory_goal.py` / `test_hardware_sim.py` | n/a | 0.1.0 | `automated-sim` | `test_canceled_jtc_result_is_failure_even_if_error_code_is_zero`; commit `22b30ba` |
| Cancellation on hardware | Cancel an in-flight JTC goal | `joint_trajectory_controller` | 0.1.1+ | 0.1.0 | `not-tested` | Bench goal was run to completion, not canceled |
| Connection loss | Closed stream refuses `write`; `recv` 0 is `ERROR` | plugin `drain_stream` / `test_hardware_sim.py` | n/a | 0.1.0 | `automated-sim` | `test_stream_eof_is_a_read_error` |
| Connection loss on hardware | Unplug host Ethernet mid-move | plugin | 0.1.3 | 0.1.0 | `not-tested` | |
| Board/host restart | Deactivate `disable`; activate `disable`, `clear_alerts`, `configure`, `set_test_mode`, `enable` | `test_hardware_sim.py` | n/a | 0.1.0 | `automated-sim` | `test_host_restart_is_deactivate_then_activate` |
| Homing | Plugin activate does not call `home` | `PLUGIN_ACTIVATE_METHODS` | n/a | 0.1.0 | `automated-sim` | `test_home_is_not_a_plugin_activate_step` |
| Homing on hardware | Session `home` while the plugin is down | session | 0.1.3 | 0.1.0 | `not-tested` | |
| Recovery | Watchdog flag latches; writes blocked until next activate `clear_alerts` | packed state frame / `test_hardware_sim.py` | n/a | 0.1.0 | `automated-sim` | `test_watchdog_frame_latches_and_blocks_write` |
| Recovery on hardware | Trip watchdog, reactivate | plugin | 0.1.3 | 0.1.0 | `not-tested` | |
| Two-axis JTC goal (0.03, 0.03) m then origin | `error_code` 0, peak \|Y−X\| 0.013 mm, end −0.09 mm | `trajectory.launch.py`, `send_trajectory_goal.py` | 0.1.1 | 0.1.0 | `hardware-bench` | releases/README 0.1.1; motors left disabled |
| colcon build on Jazzy | `clearcore_hardware` and `clearcore_bridge` compile | GitHub Actions | n/a | 0.1.0 | `automated-sim` | `.github/workflows/clearcore-ros-jazzy.yml` |

## Session / firmware (not a ROS action)

| Scenario | Expected behavior | Path | Firmware | Host | Result | Evidence |
|----------|-------------------|------|----------|------|--------|----------|
| First-motor 10 mm on M0 | `0.0100` then `0.0000`; limits and test mode restored | `host/test_m0_live.py` | 0.1.1 | 0.1.0 | `hardware-bench` | releases/README 0.1.1 |
| Joint map name/direction/gear/offset | Report 0.010 m at zero steps, move 0.012 m and back | session `configure` | 0.1.1 | 0.1.0 | `hardware-bench` | releases/README 0.1.1 |
| Square then coordinated diagonal | Endpoints 0.001 mm; peak \|Y−X\| 0.007 mm | `move_linear` | 0.1.1 | 0.1.0 | `hardware-bench` | releases/README 0.1.1 |
| NVM survive flash | Static IP and limits remain | `get_config` after flash | 0.1.1 / 0.1.3 | 0.1.0 | `hardware-bench` | 0.1.1 v2 blob; 0.1.3 v3 blob |

## XRCE

| Scenario | Expected behavior | Path | Firmware | Host | Result | Evidence |
|----------|-------------------|------|----------|------|--------|----------|
| Withhold until TIMESTAMP_REPLY | No `JointState` before reply; then agent-clock stamps | `host/xrce_check.py` | 0.1.2, 0.1.3 | 0.1.0 | `hardware-bench` | 0.1.3: three samples, `effort_len=0` |
| Empty effort + `hlfb_duty` | Writer 1 rejects filled effort; writer 2 is HLFB | `host/test_xrce_payloads.py` | n/a | 0.1.0 | `automated-sim` | `test_filled_effort_joint_states_rejected` |
| Move while agent publishes | 0 → 0.030 m → 0 on `/joint_states` | micro-ROS agent | 0.1.0 | 0.1.0 | `hardware-bench` | releases/README 0.1.0; effort then still HLFB/100 |

## How CI reports

`host/run_ci_tests.py` prints `CI label: automated with simulated hardware` and `HOST_CI_OK`. The workflow copies that distinction into the GitHub job summary. Mapping, 10 mm, and JTC hardware rows were not re-run on 0.1.3; the motion path did not change after 0.1.1.
