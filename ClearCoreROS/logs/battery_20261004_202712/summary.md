# ClearCoreROS 30-minute ROS hardware battery

- Host: `172.16.82.114`
- Log dir: `/mnt/d/CCDev/EnhancedClearCoreLibrary/ClearCoreROS/logs/battery_20261004_202712`
- Duration: 1801.545 s (target 1800)
- Motion/connection/final-disabled: **FAIL**
- Alerts clear at finalize: **no** (reported separately; not required for motion PASS)

## Verdicts

- motion_ok: `True`
- connection_ok: `False`
- final_disabled_verified: `True`
- alerts_clear: `False`
- motor_faulted_observations: `77` (rows, not necessarily distinct faults)

## Revisions

- git HEAD: `09cdadf2b63945acea8a981227261a6473c1e81b`
- discover: `CLEARCORE_ROS ClearCoreROS IP=172.16.82.114 TCP=9200 STREAM=9201 FW=1.0`
- host packages: clearcore_bridge/hardware `{'clearcore_bridge': '0.1.0', 'clearcore_hardware': '0.1.0'}`
- NVM: `{'nvm': True, 'nvm_valid': True, 'nvm_version': 3, 'axis_mask': 3, 'test_mode': False, 'names': ['joint_x', 'joint_y', 'joint_z', 'joint_a'], 'ip_address': '172.16.82.114'}`

## Counts

- cycles_started: 35
- cycles_completed: 35
- cycles_ok: 35
- cycles_failed: 0
- reconnects: 1
- bridge_fjt_ok: 35
- bridge_fjt_fail: 0
- bridge_cancel_ok: 35
- bridge_cancel_fail: 0
- jtc_ok: 35
- jtc_fail: 0
- origin_ok: 105
- origin_fail: 0
- watchdog_events: 0
- alert_events: 77
- state_gaps_over_150ms: 5

## Per-cycle drift / tracking (generated-step units, not shaft encoder)

| cycle | ok | bridge peak\|Y-X\| mm | JTC reported end mm | origin return err mm | gaps>150ms | reconnects |
|------:|:--:|----------------------:|--------------------:|---------------------:|-----------:|-----------:|
| 1 | True | 0.012500000593718141 | (-0.144, -0.144) | 0.0 | 0 | 0 |
| 2 | True | 0.006251037120819092 | (-0.106, -0.106) | 0.0 | 0 | 0 |
| 3 | True | 0.006251037120819092 | (-0.144, -0.144) | 0.0 | 1 | 1 |
| 4 | True | 0.006251037120819092 | (-0.056, -0.056) | 0.0 | 0 | 0 |
| 5 | True | 0.006250105798244476 | (-0.144, -0.144) | 0.0 | 0 | 0 |
| 6 | True | 0.006251037120819092 | (-0.069, -0.069) | 0.0 | 0 | 0 |
| 7 | True | 0.006251037120819092 | (-0.144, -0.144) | 0.0 | 0 | 0 |
| 8 | True | 0.006250105798244476 | (-0.144, -0.144) | 0.0 | 0 | 0 |
| 9 | True | 0.006251037120819092 | (-0.144, -0.144) | 0.0 | 0 | 0 |
| 10 | True | 0.006250105798244476 | (-0.063, -0.063) | 0.0 | 0 | 0 |
| 11 | True | 0.006251037120819092 | (-0.144, -0.144) | 0.0 | 0 | 0 |
| 12 | True | 0.006250105798244476 | (-0.144, -0.144) | 0.0 | 0 | 0 |
| 13 | True | 0.006250105798244476 | (-0.112, -0.112) | 0.0 | 0 | 0 |
| 14 | True | 0.006251037120819092 | (-0.144, -0.144) | 0.0 | 0 | 0 |
| 15 | True | 0.006250105798244476 | (-0.144, -0.144) | 0.0 | 0 | 0 |
| 16 | True | 0.006250105798244476 | (-0.144, -0.144) | 0.0 | 0 | 0 |
| 17 | True | 0.006251037120819092 | (-0.144, -0.144) | 0.0 | 0 | 0 |
| 18 | True | 0.006250105798244476 | (-0.144, -0.144) | 0.0 | 0 | 0 |
| 19 | True | 0.006250105798244476 | (-0.125, -0.125) | 0.0 | 1 | 0 |
| 20 | True | 0.006251037120819092 | (-0.144, -0.144) | 0.0 | 0 | 0 |
| 21 | True | 0.006250105798244476 | (-0.087, -0.087) | 0.0 | 0 | 0 |
| 22 | True | 0.006251037120819092 | (-0.144, -0.144) | 0.0 | 0 | 0 |
| 23 | True | 0.006251037120819092 | (-0.144, -0.144) | 0.0 | 0 | 0 |
| 24 | True | 0.006251037120819092 | (-0.144, -0.144) | 0.0 | 0 | 0 |
| 25 | True | 0.006250105798244476 | (-0.063, -0.063) | 0.0 | 0 | 0 |
| 26 | True | 0.006250105798244476 | (-0.106, -0.106) | 0.0 | 0 | 0 |
| 27 | True | 0.006251037120819092 | (-0.144, -0.144) | 0.0 | 1 | 0 |
| 28 | True | 0.006250105798244476 | (-0.144, -0.144) | 0.0 | 0 | 0 |
| 29 | True | 0.006250105798244476 | (-0.094, -0.094) | 0.0 | 0 | 0 |
| 30 | True | 0.006251037120819092 | (-0.144, -0.144) | 0.0 | 2 | 0 |
| 31 | True | 0.006250105798244476 | (-0.063, -0.063) | 0.0 | 0 | 0 |
| 32 | True | 0.006251037120819092 | (-0.075, -0.075) | 0.0 | 0 | 0 |
| 33 | True | 0.006251037120819092 | (-0.144, -0.144) | 0.0 | 0 | 0 |
| 34 | True | 0.006251037120819092 | (-0.081, -0.081) | 0.0 | 0 | 0 |
| 35 | True | 0.006251037120819092 | (-0.144, -0.144) | 0.0 | 0 | 0 |

JTC reported endpoint is listed separately from origin return error; do not treat the JTC end_mm (e.g. −0.14 mm) as the origin check.

## Final disabled state

- enabled: `False`
- test_mode_status: `False`
- test_mode_config: `False`
- moving: `False`
- watchdog: `False`
- alerts: `motor_faulted`
- position_m: `[0.0, 0.0, 0.0, 0.0]`
- names: `['joint_x', 'joint_y', 'joint_z', 'joint_a']`
- axis_mask: `3`

## Final checks

- status_known: True
- enabled_false: True
- test_mode_false: True
- names_default: True
- axis_mask_3: True
- no_alerts: False
- no_watchdog: True

## Alert events (77)

- t=48.8s where=cycle002_pre alerts=motor_faulted
- t=48.8s where=origin_pre_pre_clear alerts=motor_faulted
- t=98.7s where=cycle003_pre alerts=motor_faulted
- t=98.7s where=origin_pre_pre_clear alerts=motor_faulted
- t=166.4s where=cycle004_pre alerts=motor_faulted
- t=166.5s where=origin_pre_pre_clear alerts=motor_faulted
- t=215.6s where=cycle005_pre alerts=motor_faulted
- t=215.7s where=origin_pre_pre_clear alerts=motor_faulted
- t=267.2s where=cycle006_pre alerts=motor_faulted
- t=267.2s where=origin_pre_pre_clear alerts=motor_faulted
- t=316.1s where=cycle007_pre alerts=motor_faulted
- t=316.1s where=origin_pre_pre_clear alerts=motor_faulted
- t=366.7s where=cycle008_pre alerts=motor_faulted
- t=366.7s where=origin_pre_pre_clear alerts=motor_faulted
- t=418.5s where=cycle009_pre alerts=motor_faulted
- t=418.5s where=origin_pre_pre_clear alerts=motor_faulted
- t=467.6s where=cycle010_pre alerts=motor_faulted
- t=467.7s where=origin_pre_pre_clear alerts=motor_faulted
- t=515.1s where=cycle011_pre alerts=motor_faulted
- t=515.1s where=origin_pre_pre_clear alerts=motor_faulted
