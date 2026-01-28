# Motion Streaming Example

This example demonstrates streaming motion commands to ClearCore over serial (USB CDC) or Ethernet (TCP server). Commands can be sent in real-time to execute coordinated motion moves, configure parameters, and query status.

## Features

- **Dual Communication Modes**: Supports both serial (USB CDC) and Ethernet (TCP server)
- **G-Code Compatible**: Accepts standard G-code commands for motion control
- **Unit Support**: Works with both inches and millimeters
- **Coordinated Motion**: Supports linear moves (G01) and arc moves (G02/G03)
- **Automatic Command Queuing**: Commands are automatically queued and executed in order
  - If motion is active, new commands are queued (up to 8 commands)
  - If no motion is active, commands execute immediately
  - Commands execute sequentially without interrupting each other
- **Status Queries**: Query current position and motion status

## Requirements

1. Two ClearPath motors connected to Connector M-0 and M-1
2. Motors configured for Step and Direction mode via MSP software
3. HLFB mode set to "ASG-Position w/Measured Torque" at 482 Hz
4. For Ethernet mode: Ethernet cable connected and network configured

## Configuration

### Communication Mode

Edit `MotionStreaming.cpp` to select communication mode:

```cpp
#define COMM_MODE SERIAL_MODE  // For serial (USB CDC)
// or
#define COMM_MODE ETHERNET_MODE  // For Ethernet TCP
```

### Serial Mode Configuration

- **Baud Rate**: Default 115200 (configurable via `SERIAL_BAUD_RATE`)
- **Port**: Default USB CDC (configurable via `SerialPort`)

### Ethernet Mode Configuration

- **Port**: Default 8888 (configurable via `TCP_PORT`)
- **IP Configuration**: 
  - DHCP: Set `usingDhcp = true` (default)
  - Manual: Set `usingDhcp = false` and configure `manualIp`

### Mechanical Configuration

Edit these values to match your system:

```cpp
#define MOTOR_X_STEPS_PER_REV 800
#define MOTOR_X_PITCH_MM 5.0
#define MOTOR_Y_STEPS_PER_REV 800
#define MOTOR_Y_PITCH_MM 5.0
```

## Command Protocol

Commands are sent as text lines, terminated with newline (`\n`). The system responds with `ok` for successful commands or error messages.

### Motion Commands

#### G01 - Linear Move
```
G01 X10.5 Y20.3 F100
```
- `X`, `Y`: Target position (in current units)
- `F`: Feed rate (optional, uses last set rate if omitted)
- **Note**: Motors must be enabled (M202) before motion commands will execute. If motors are disabled, returns error: "Motors not enabled (use M202 to enable)"

#### G02 - Arc Move Clockwise
```
G02 X10 Y10 I5 J0 F100
```
- `X`, `Y`: End position
- `I`, `J`: Center offset from start position
- `F`: Feed rate (optional)
- **Note**: Motors must be enabled (M202) before motion commands will execute. If motors are disabled, returns error: "Motors not enabled (use M202 to enable)"

#### G03 - Arc Move Counterclockwise
```
G03 X10 Y10 I5 J0 F100
```
Same parameters as G02, but counterclockwise direction.
- **Note**: Motors must be enabled (M202) before motion commands will execute. If motors are disabled, returns error: "Motors not enabled (use M202 to enable)"

### Coordinate System Commands

#### G90 - Absolute Coordinates
```
G90
```
All positions are absolute from origin.

#### G91 - Incremental Coordinates
```
G91
```
All positions are relative to current position.

#### G92 - Set Coordinate System Offset (Set Origin)
```
G92 X0 Y0        ; Set current position as origin (0,0)
G92 X10 Y5       ; Set current position to (10, 5) - shifts coordinate system
G92              ; Reset offset - set current position to (0, 0)
```
Sets the coordinate system origin. Without parameters, resets to (0,0). With X/Y parameters, sets the current physical position to those coordinate values, effectively shifting the coordinate system.

### Unit Commands

#### G20 - Units in Inches
```
G20
```

#### G21 - Units in Millimeters
```
G21
```

### Control Commands

#### M200 - Emergency Stop
```
M200
```
Immediately stops all motion (abrupt stop) and clears the command queue. Motors remain enabled. Use for emergency situations.

#### M201 - Stop with Deceleration
```
M201
```
Stops motion smoothly with deceleration and clears the command queue. Motors remain enabled. Use when you want a controlled stop.

#### M202 - Enable Motors
```
M202
```
Enables both motors. Motors must be enabled before motion commands will execute.

#### M203 - Disable Motors
```
M203
```
Stops all motion, clears queue, and disables motors. Use for shutdown or when motors should not move.

#### M114 - Get Current Position
```
M114
```
Response: `X:10.500 Y:20.300`

#### M115 - Get Status
```
M115
```
Response: `Status: Active=1 Queue=2 Units=mm Coords=abs`

## Usage Examples

### Serial Mode

1. Connect ClearCore via USB
2. Open serial terminal (115200 baud)
3. Send commands:
   ```
   G21        ; Set units to millimeters
   G90        ; Set absolute coordinates
   G01 X10 Y10 F100  ; Move to (10mm, 10mm) at 100mm/min
   M114       ; Query position
   ```

### Ethernet Mode

1. Connect ClearCore via Ethernet
2. Connect TCP client to ClearCore IP on port 8888
3. Send commands same as serial mode

### Example Sequence

```
G21          ; Millimeters
G90          ; Absolute
M202         ; Enable motors
G01 X0 Y0 F100  ; Home position
G01 X50 Y0 F100  ; Move right 50mm
G02 X50 Y50 I0 J25 F100  ; Arc up (25mm radius)
G01 X0 Y50 F100  ; Move left
G01 X0 Y0 F100   ; Return home
M114         ; Check position
```

## Building and Running

1. Open `MotionStreaming.atsln` in Microchip Studio
2. Set MotionStreaming as startup project
3. Build and flash to ClearCore
4. Connect via serial or Ethernet
5. Start sending commands

## Notes

- Commands are executed immediately when received
- Motion commands queue automatically if previous motion is active
- Feed rate persists until changed
- Coordinate and unit modes persist until changed
- For Ethernet mode, status messages also appear on USB serial port for debugging

## Troubleshooting

- **No response**: Check communication mode matches connection type
- **Commands ignored**: Ensure motors are enabled (M202)
- **Motion not smooth**: Check feed rate is appropriate for your system
- **Ethernet not connecting**: Verify network configuration and cable connection

## License

Copyright (c) 2026 Adam G. Sweeney <agsweeney@gmail.com>

MIT License - see LICENSE file for details.
