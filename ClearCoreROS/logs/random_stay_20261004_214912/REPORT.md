# Stay-enabled ROS random-move endurance

**Log dir:** `ClearCoreROS/logs/random_stay_20261004_214912`  
**Host:** `172.16.82.114` · **axis_mask:** 3 (M0/M1) · **test_mode:** false  
**Stack:** `clearcore_hardware` `trajectory.launch.py` → `joint_trajectory_controller`  
**Harness:** `host/run_ros_random_stay_enabled_30m.py`  
**Seed:** `1791168552`  
**Clock:** start ~2026-10-04 21:49 local · ended `2026-10-05T03:22:36Z` · **1798 s of motion** (target 1800 s)

Drive mode on the bench: **ASGH Position w/ Measured Torque** on both axes.

Positions in this report are **generated-step / PositionRefCommanded**, not shaft-encoder readings. JTC success is `FollowJointTrajectory` `STATUS_SUCCEEDED` (4) and `error_code` `SUCCESSFUL` (0).

---

## Verdict

| Track | Result |
|---|---|
| Motion | **PASS** — 1151 / 1151 goals succeeded, 0 rejects/aborts |
| Stay connected | **PASS** — one `trajectory.launch` for the whole hour; ROS was not killed between moves |
| Stay enabled | **PASS** for the test definition — motors were not session-disabled or plugin-deactivated during the 30 minutes |
| Final disabled | **NOT VERIFIED** — post-teardown `get_status` timed out |

This is a **ROS-control-in-operation** run, not the earlier bring-up/teardown battery. It does not exercise disable/HLFB edges, so it does not close the `motor_faulted`-while-disabled investigation.

---

## What ran

Bring-up reclaimed TCP **9200** first (kill leftover ROS/plugin holders, wait until `get_status` answered). Preflight:

- `enabled`: false  
- `alerts`: none  
- session counters: accepts 1856 / closes 1855  

Then **one** `ros2 launch clearcore_hardware trajectory.launch.py` (`host:=172.16.82.114`, `axis_mask:=3`, `test_mode:=false`). JTC action was ready at t=4.0 s; `/joint_states` was live the same second.

From then until t=1798.4 s the harness sent **random XY** `FollowJointTrajectory` goals:

- Envelope: **1.0–28.0 mm** on each axis (inside the 30 mm soft max)  
- Independent X and Y  
- Segment time: distance / 15 mm/s, clamped **1.5–5.0 s** (observed 1.50–2.33 s, mean 1.53 s)  
- **No** `disable`, **no** `clear_alerts`, **no** ROS teardown, **no** parallel session `get_status` (plugin already owns 9200)

Disable was attempted only after the clock stopped.

---

## Motion results

| Metric | Value |
|---|---|
| Goals | 1151 |
| Failed / rejected | 0 |
| Action status | all `4` (SUCCEEDED) |
| error_code | all `0` |
| Goal rate | ~38.5 / min (mean spacing 1.56 s) |
| Commanded X | 1.00–27.98 mm, mean 14.20 mm |
| Commanded Y | 1.01–27.97 mm, mean 14.74 mm |
| `/joint_states` samples | 89,740 (~50 Hz, matches controller manager 50 Hz) |
| Peak \|Y−X\| on sampled states | 26.45 mm |
| Last reported XY | 2.66 mm, 13.19 mm |

`traj_launch.log` shows a continuous accept → goal reached → next goal loop through the last sample (`Goal reached, success!`). The only controller-manager warning at start is FIFO RT scheduling not permitted (WSL); it did not affect goal success.

Controller goal tolerance is 2 mm (`controllers.yaml`). End-of-goal `last_xy` in the harness log tracked commanded targets inside that band on the sampled checkpoints.

---

## What this does *not* prove

1. **`motor_faulted` is gone.** The bring-up battery (`battery_20261004_202712`) still saw that latch **between cycles while disabled**. This run never disabled, so it never hit that path. Session `alerts` were also not sampled during the hour (exclusive 9200).

2. **Final disabled state.** After JTC teardown, `get_status` timed out (same single-session rule, plus a half-open leftover). `final_board.json` has `status_known: false`. Motors may still have been enabled until the TCP hold dropped.

3. **Encoder tracking / HLFB torque.** Effort on `/joint_states` from the broadcaster is empty; HLFB is on `/dynamic_joint_states`. This harness did not log HLFB.

---

## Relation to the bring-up battery

`battery_20261004_202712` (same evening): 35/35 motion cycles OK, but **1 reconnect**, **77 `motor_faulted` observation rows**, `connection_ok: false` in separated verdicts because of that reconnect, `alerts_clear: false`. That script killed ROS and `disable`/`enable` around every move.

This stay-enabled run shows: **with JTC left up, random FJT for 30 minutes does not fail.** The TCP leak / readiness-race fixes are consistent with that. The disable-edge fault remains a separate defect/investigation.

---

## Artifacts

| File | Content |
|---|---|
| `random.log` / `events.jsonl` | Start, every 10th goal, finish |
| `moves.csv` | All 1151 goals (t, x, y, dur, status, error_code) |
| `traj_launch.log` | `ros2_control` / JTC accept and “Goal reached” |
| `summary.json` | Machine verdicts |
| `final_board.json` | Teardown status (unknown) |

---

## Follow-up (teardown only)

Post-run session reclaim must **wait until 9200 answers** after killing `ros2_control_node`, then `disable` and record `get_status.axes[]`. That is required for a verified disabled ending; it is not a motion failure of this hour.
