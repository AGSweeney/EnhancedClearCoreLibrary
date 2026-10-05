# ClearCoreROS 30-minute ROS hardware battery

- Host: `172.16.82.114`
- Log dir: `/mnt/d/CCDev/EnhancedClearCoreLibrary/ClearCoreROS/logs/battery_20261004_184947`
- Duration: 254.241 s (target 1800)
- Result: **FAIL**

## Revisions

- git HEAD: `09cdadf2b63945acea8a981227261a6473c1e81b`
- discover: `CLEARCORE_ROS ClearCoreROS IP=172.16.82.114 TCP=9200 STREAM=9201 FW=1.0`
- host packages: clearcore_bridge/hardware `{'clearcore_bridge': '0.1.0', 'clearcore_hardware': '0.1.0'}`
- NVM: `{'nvm': True, 'nvm_valid': True, 'nvm_version': 3, 'axis_mask': 3, 'test_mode': False, 'names': ['joint_x', 'joint_y', 'joint_z', 'joint_a'], 'ip_address': '172.16.82.114'}`

## Counts

- cycles_started: 7
- cycles_completed: 7
- cycles_ok: 0
- cycles_failed: 7
- reconnects: 0
- bridge_fjt_ok: 0
- bridge_fjt_fail: 7
- bridge_cancel_ok: 7
- bridge_cancel_fail: 0
- jtc_ok: 7
- jtc_fail: 0
- origin_ok: 21
- origin_fail: 0
- watchdog_events: 0
- alert_events: 7
- state_gaps_over_150ms: 0

## Per-cycle drift / tracking (generated-step units, not shaft encoder)

| cycle | ok | bridge peak\|Y-X\| mm | JTC reported end mm | origin return err mm | gaps>150ms | reconnects |
|------:|:--:|----------------------:|--------------------:|---------------------:|-----------:|-----------:|
| 1 | False | 0.006250105798244476 | (-0.144, -0.144) | 0.0 | 0 | 0 |
| 2 | False | 0.006249989382922649 | (-0.125, -0.125) | 0.0 | 0 | 0 |
| 3 | False | 0.006250047590583563 | (-0.144, -0.144) | 0.0 | 0 | 0 |
| 4 | False | 0.006250105798244476 | (-0.144, -0.144) | 0.0 | 0 | 0 |
| 5 | False | 0.006250105798244476 | (-0.144, -0.144) | 0.0 | 0 | 0 |
| 6 | False | 0.006250105798244476 | (-0.112, -0.112) | 0.0 | 0 | 0 |
| 7 | False | 0.0 | (-0.144, -0.144) | 0.0 | 0 | 0 |

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

- enabled_false: True
- test_mode_false: True
- names_default: True
- axis_mask_3: True
- no_alerts: False
- no_watchdog: True

## Failures

- `{"t_s": 5.436215427000008, "step": "bridge_fjt", "rc": 3}`
- `{"t_s": 43.595332912, "step": "bridge_fjt", "rc": 3}`
- `{"t_s": 80.54108345100002, "step": "bridge_fjt", "rc": 3}`
- `{"t_s": 118.811600059, "step": "bridge_fjt", "rc": 3}`
- `{"t_s": 156.647244118, "step": "bridge_fjt", "rc": 3}`
- `{"t_s": 193.42132349400003, "step": "bridge_fjt", "rc": 3}`
- `{"t_s": 231.651984352, "step": "bridge_fjt", "rc": 3}`

## Alert events (7)

- t=40.7s where=cycle002_pre alerts=motor_faulted
- t=77.9s where=cycle003_pre alerts=motor_faulted
- t=153.9s where=cycle005_pre alerts=motor_faulted
- t=190.6s where=cycle006_pre alerts=motor_faulted
- t=212.0s where=cycle006_post alerts=motor_faulted
- t=228.6s where=cycle007_pre alerts=motor_faulted
- t=254.2s where=finalize alerts=motor_faulted
