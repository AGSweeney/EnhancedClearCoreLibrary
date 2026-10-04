# First motor

Get one ClearPath moving on connector M0. This uses the released firmware and `host/ccros_cli.py`. It does not use ROS, `ros2_control`, or the XRCE publisher.

This exercise uses the released image and the defaults. M0 is `joint_x`, a linear axis in meters, direction `+1`, gear `1`, and offset `0`. The image can rename an axis and set `rotary_*`, `direction_*`, `gear_*`, and `offset_*`. Change direction, gear, rotary, or offset only while the motors are disabled. A reversed shaft can be fixed in MSP, in the cable, or with `configure --direction-x -1` while disabled. Leave the defaults in place for this exercise.

Reported position is generated steps (`PositionRefCommanded`), not a shaft encoder. `0.01` in a status position is 0.01 m, which is 10 mm.

## What you need

- A ClearCore and one ClearPath motor.
- Python 3.10 or newer on the computer that will talk to the board.
- This repository at a revision that lists firmware `0.1.3` in [../releases/README.md](../releases/README.md). The firmware sources for that image are git `4eeab56`.
- The motor's power supply, and a way to stop the shaft if it runs the wrong direction. Keep a physical estop in the circuit.

The computer and the ClearCore must be on the same Ethernet network. USB is only for flashing and for the serial log.

## 1. MSP

In ClearPath MSP, set this motor to:

- **Step and Direction**
- **HLFB: ASG-Position with Measured Torque**, 482 Hz
- Steps per revolution equal to the value you will send as `steps_per_rev` (the firmware default is 800)

The screw or belt pitch is not an MSP setting. You send it to the ClearCore as `pitch_mm`.

## 2. Wiring

Plug that motor's control cable into the ClearCore **M0** connector. M0 is `joint_x`. Leave M1–M3 empty for this exercise.

DI-6 is the estop input. The default (`estop_di6` 1) treats DI-6 **low** as estop, and `enable` then fails. Either wire the input so it is high when motion is allowed, or turn the check off in step 6. Do not use test mode to hide a missing HLFB signal. Test mode also skips the estop check.

## 3. Flash

The file is [../releases/ClearCoreROS-0.1.3.bin](../releases/ClearCoreROS-0.1.3.bin). Confirm the SHA-256 in [../releases/README.md](../releases/README.md) before flashing. Flashing replaces the application on the board. User NVM is kept, including a version 1 or version 2 blob. A ClearAI configuration blob is not applied. The first `configure` after an older blob saves version 3 and keeps the network settings and limits you do not change.

The running board is USB VID `2890`, PID `8022`. The bootloader is PID `0022`. The application is written at offset `0x4000`.

On Windows, from the repository root:

```powershell
.\Tools\flash_clearcore.cmd .\ClearCoreROS\releases\ClearCoreROS-0.1.3.bin
```

On Linux, install `bossac` (the BOSSA command-line tool), then enter the bootloader and write the image. The USB port name changes when the bootloader enumerates. If the first command is aimed at the running application, it drops the port; wait, then run the write against the bootloader port.

```bash
bossac --info --debug --port=/dev/ttyACM0 --arduino-erase
bossac --info --debug --port=/dev/ttyACM0 --usb-port --write --erase --verify --offset=0x4000 --reset ClearCoreROS/releases/ClearCoreROS-0.1.3.bin
```

The Windows script was the one used on the bench. The Linux lines are the same `bossac` invocation.

## 4. Find the board

DHCP is the default. If DHCP fails, the address falls back to `192.168.0.109`. From `ClearCoreROS`:

```bash
python3 host/ccros_cli.py --host 255.255.255.255 discover
```

A reply looks like:

```text
CLEARCORE_ROS ClearCoreROS IP=192.168.1.50 TCP=9200 STREAM=9201 FW=1.0
```

Use that IP as `--host` below. If the global broadcast is dropped, send the same command to the subnet broadcast, or to `192.168.0.109`.

```bash
python3 host/ccros_cli.py --host 192.168.1.255 discover
```

The USB serial port is 115200 8N1. At boot it prints a one-line `ready` status. That line does not contain the IP. Discovery does.

## 5. Mechanics

Disable is the power-up state. Set the mask to M0 only, then set steps per revolution and pitch in millimetres. One number is copied to all four axes. The mask is what matters for this motor.

```bash
python3 host/ccros_cli.py --host 192.168.1.50 configure --axis-mask 1 --steps-per-rev 800 --pitch-mm 5
python3 host/ccros_cli.py --host 192.168.1.50 config
```

`pitch_mm` must match the mechanics. A 5 mm lead and 800 steps/rev is 160 steps per millimetre. `config` shows `axis_mask` 1, those mechanics, and the default map: names `joint_x` through `joint_a`, `rotary` only on A, `direction` 1, `gear` 1, and `offset` 0. Change `axis_mask`, `steps_per_rev`, `pitch_mm`, `rotary_*`, `direction_*`, `gear_*`, or `offset_*` only while the motors are disabled. A joint name can change while enabled.

This is saved in NVM and restored on the next boot.

## 6. Limits and estop

Put a soft limit a short distance past the move you are about to command, in metres. This example allows 0 to 20 mm on X and ignores the other axes because they are not in the mask.

```bash
python3 host/ccros_cli.py --host 192.168.1.50 configure --min-x 0 --max-x 0.02
```

Flashing keeps NVM, so a previous session can still have `test_mode` true. Turn it off before checking estop or enabling. Without `--on`, this saves false and restores the HLFB, estop, and limit-switch checks.

```bash
python3 host/ccros_cli.py --host 192.168.1.50 test-mode
```

Check estop before enabling:

```bash
python3 host/ccros_cli.py --host 192.168.1.50 status
```

If `estop` is true and you have no switch, turn the DI-6 check off. DI-6 going low latches estop. Setting `estop_di6` to 0 removes the input and leaves that latch set, so clear alerts before reading status again:

```bash
python3 host/ccros_cli.py --host 192.168.1.50 configure --estop-di6 0
python3 host/ccros_cli.py --host 192.168.1.50 clear-alerts
python3 host/ccros_cli.py --host 192.168.1.50 status
```

`estop` should be false. `test_mode` should be false.

## 7. Enable and move 10 mm

```bash
python3 host/ccros_cli.py --host 192.168.1.50 enable
python3 host/ccros_cli.py --host 192.168.1.50 move --x 0.01
```

`enable` waits for HLFB. If it fails, MSP HLFB is not asserting. Do not turn test mode on to skip that.

`move` prints a position line about ten times a second until the axis is in position. A completed 10 mm move looks like:

```text
pos=['0.0100', '0.0000', '0.0000', '0.0000'] err=0.00000 moving=False watchdog=False
```

Then the status JSON. `position` values are metres. `enabled` is true, `moving` is false, `fault` is false, `estop` is false, and `watchdog` is false.

Return to zero and disable:

```bash
python3 host/ccros_cli.py --host 192.168.1.50 move --x 0
python3 host/ccros_cli.py --host 192.168.1.50 disable
python3 host/ccros_cli.py --host 192.168.1.50 status
```

The first position is about `0.0000`. `enabled` is false.

## If it does not move

| What you see | What it means |
|--------------|----------------|
| `estop` true | DI-6 is low and the default check is on, or the latch from an earlier low is still set. `clear-alerts` clears that latch once DI-6 is high or `estop_di6` is 0. |
| `enable` fails mentioning HLFB | MSP HLFB is not the ASG-Position setting above, or the HLFB wire is open. |
| `fault` true | Read `alerts` in the status JSON. `clear-alerts` is the recovery after the cause is gone. |
| Position counts the opposite direction | Reverse it in MSP or the cable, or, with the motor disabled, `configure --direction-x -1`. This exercise leaves direction at `+1`. |
| `axis_mask` is 3 and enable fails | M1 is in the mask and has no motor. Use `--axis-mask 1`. |

ROS, `ros2_control`, and XRCE are separate setups. See [../README.md](../README.md).
