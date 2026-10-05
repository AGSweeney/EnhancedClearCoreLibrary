# ClearCoreROS 30-minute ROS hardware battery

- Host: `172.16.82.114`
- Log dir: `/mnt/d/CCDev/EnhancedClearCoreLibrary/ClearCoreROS/logs/battery_20261004_180442`
- Duration: 214.969 s (target 1800)
- Result: **FAIL**

## Revisions

- git HEAD: `a31e0d75afe11d0e600aa9b780c677a47656e567`
- discover: `CLEARCORE_ROS ClearCoreROS IP=172.16.82.114 TCP=9200 STREAM=9201 FW=1.0`
- host packages: clearcore_bridge/hardware `{'clearcore_bridge': '0.1.0', 'clearcore_hardware': '0.1.0'}`
- NVM: `{'nvm': True, 'nvm_valid': True, 'nvm_version': 3, 'axis_mask': 3, 'test_mode': False, 'names': ['joint_x', 'joint_y', 'joint_z', 'joint_a'], 'ip_address': '172.16.82.114'}`

## Counts

- cycles_started: 5
- cycles_completed: 4
- cycles_ok: 1
- cycles_failed: 3
- reconnects: 1
- bridge_fjt_ok: 1
- bridge_fjt_fail: 4
- bridge_cancel_ok: 5
- bridge_cancel_fail: 0
- jtc_ok: 4
- jtc_fail: 0
- origin_ok: 13
- origin_fail: 2
- watchdog_events: 0
- alert_events: 4
- state_gaps_over_150ms: 1

## Per-cycle drift / tracking (generated-step units, not shaft encoder)

| cycle | ok | bridge peak\|Y-X\| mm | JTC reported end mm | origin return err mm | gaps>150ms | reconnects |
|------:|:--:|----------------------:|--------------------:|---------------------:|-----------:|-----------:|
| 1 | True | 0.012500211596488953 | (-0.075, -0.075) | 0.0 | 0 | 0 |
| 2 | False | 0.006250105798244476 | (-0.094, -0.094) | 0.0 | 0 | 0 |
| 3 | False | 0.006249989382922649 | (-0.144, -0.144) | 0.0 | 0 | 0 |
| 4 | False | 0.006250105798244476 | (-0.144, -0.144) | 0.0 | 1 | 0 |

JTC reported endpoint is listed separately from origin return error; do not treat the JTC end_mm (e.g. −0.14 mm) as the origin check.

## Final disabled state

- enabled: `None`
- test_mode_status: `None`
- test_mode_config: `None`
- moving: `None`
- watchdog: `None`
- alerts: `None`
- position_m: `None`
- names: `None`
- axis_mask: `None`

## Final checks

- enabled_false: False
- test_mode_false: False
- names_default: False
- axis_mask_3: False
- no_alerts: True
- no_watchdog: False

## Failures

- `{"t_s": 37.440608164, "step": "bridge_fjt", "rc": 3}`
- `{"t_s": 64.82974162000001, "step": "bridge_fjt", "rc": 3}`
- `{"t_s": 93.960057307, "step": "bridge_fjt", "rc": 3}`
- `{"t_s": 120.79960321499999, "step": "bridge_fjt", "rc": 3}`
- `{"t_s": 125.34364313500001, "step": "origin_mid", "error": "[Errno 104] Connection reset by peer"}`
- `{"t_s": 213.415848878, "step": "origin_post", "error": "[Errno 104] Connection reset by peer"}`

## Alert events (4)

- t=34.1s where=cycle002_pre alerts=motor_faulted
- t=61.9s where=cycle003_pre alerts=motor_faulted
- t=91.2s where=cycle004_pre alerts=motor_faulted
- t=118.0s where=cycle005_pre alerts=motor_faulted
