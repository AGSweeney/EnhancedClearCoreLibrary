# ClearCoreROS 30-minute ROS hardware battery — separated verdicts

- Log dir: `/mnt/d/CCDev/EnhancedClearCoreLibrary/ClearCoreROS/logs/battery_20261004_185552`
- Duration: 1801.544 s (target 1800)
- Motion/connection/final-disabled: **PASS**
- Alerts clear at finalize: **no** (separate from motion PASS)
- Hardware note: M0 and M1 are both set to **ASGH Position w/ Measured Torque** (corrected after the run; earlier asymmetry note withdrawn).

## Verdicts

- motion_ok: `True` — 35/35 cycles; FJT/cancel/JTC/origin all zero fails
- connection_ok: `True` — reconnects=0; failures=0
- final_disabled_verified: `True` — enabled=False, test_mode=False, status_known=True
- alerts_clear: `False` — final alerts=`motor_faulted` (latch left intact for investigation)
- motor_faulted_observations: `81` (cycle*_pre=34, origin_pre_pre_clear=34, tail=12)

Result framing: motion PASS, connection PASS, final disabled verified, **alert investigation open** — not an entirely clean run.

## Alert observation counting

81 alert log rows do **not** mean 81 distinct fault events.

- `cycleNNN_pre` and `origin_pre_pre_clear` in the same cycle often capture the **same latched** `motor_faulted` before `clear_alerts`.
- Inter-cycle `tail` samples can repeatedly report one final latch while disabled.
- Treat counts as observation frequency; dedupe by latch lifetime when investigating root cause.

Seeing `motor_faulted` while disabled establishes observation state, not necessarily when or why it first latched. Next investigation: latch timing relative to `disable` and HLFB dropping.

## TCP counters (final)

- session accepts/closes: 1877/1876
- stream accepts/closes: 484/484

Session accepts exceeding closes by one is **expected** when `get_status` is sampled on a still-open session connection (that Accept is counted; Close happens after the RPC returns and the client disconnects). Stream counts matched in this run.

## Note

The original `summary.md` Result FAIL conflated final `motor_faulted` with motion endurance.
This addendum separates those tracks. Gaps>150ms count: 3; watchdog_events: 0.
