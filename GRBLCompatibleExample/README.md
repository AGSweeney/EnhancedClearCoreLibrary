# GRBL-Compatible Example

This example provides a GRBL-compatible interface for ClearCore, allowing use with GRBL-compatible software like Universal G-code Sender, bCNC, CNCjs, and other GRBL streaming tools.

## Features

- **GRBL Protocol Compatibility**: Implements GRBL v1.1 protocol for 3-axis motion (X/Y coordinated, Z independent)
- **Real-time Commands**: Supports `?` (status), `!` (feed hold), `~` (resume), Ctrl-X (soft reset)
- **GRBL Status Format**: `<Idle|Run|Hold|Alarm>,MPos:X.xxx,Y.yyy,Z.zzz,FS:F,SS:0>`
- **GRBL Error Codes**: Standard numeric error codes (error:1, error:2, etc.)
- **Standard G-codes**: G00 (rapid), G01 (linear), G02/G03 (arcs), G20/G21 (units), G28 (home), G38 (probe), G90/G91 (coords), G92 (set origin)
- **3-Axis Support**: X and Y axes coordinated for arcs; Z axis moves independently
- **Home Switches**: Optional X, Y, and Z home switch inputs for homing sequence
- **Probe Input**: Optional touch probe input for workpiece measurement
- **Limit Switches**: Optional limit switches for safety (X/Y/Z min/max)
- **Automatic Status Reporting**: Periodic status reports every 250ms

## Requirements

1. Three ClearPath motors connected to Connector M-0 (X), M-1 (Y), and M-2 (Z)
2. Motors configured for Step and Direction mode
3. HLFB mode: "ASG-Position w/Measured Torque" with 482 Hz PWM carrier frequency

## GRBL Protocol

### Status Reporting

Status reports are sent automatically every 250ms and on request (`?` command):

```
<Idle,MPos:10.500,20.300,5.000,FS:100,SS:0>
```

Format:
- **State**: `Idle`, `Run`, `Hold`, `Alarm`, `Door`, `Check`, `Home`, `Sleep`
- **MPos**: Machine position (X, Y, Z) in current units
- **FS**: Feed rate and spindle speed
- **SS**: Spindle speed (always 0 for this implementation)

### Real-time Commands

- **`?`**: Request status report immediately
- **`!`**: Feed hold (pause motion with deceleration)
- **`~`**: Resume from feed hold
- **Ctrl-X** (0x18): Soft reset (stop motion, clear queue)

### G-codes

#### Motion Commands

- **G00 X Y Z**: Rapid move (uses maximum velocity)
  - X, Y move coordinated; Z moves independently
- **G01 X Y Z F**: Linear interpolation with feed rate
  - X, Y move coordinated; Z moves independently at same feed rate
- **G02 X Y Z I J F**: Circular interpolation clockwise
  - X, Y arc coordinated; Z moves linearly during arc
- **G03 X Y Z I J F**: Circular interpolation counterclockwise
  - X, Y arc coordinated; Z moves linearly during arc

#### Coordinate System

- **G90**: Absolute coordinates
- **G91**: Incremental coordinates
- **G92 X Y Z**: Set coordinate system offset (set origin)

#### Units

- **G20**: Units in inches
- **G21**: Units in millimeters (default)

#### Homing

- **G28**: Home (move to home switches, then set origin to 0,0,0)
  - If home switches are configured, moves each axis toward switches until triggered
  - If no home switches, simply moves to origin (0,0,0)
  - Sets coordinate system origin after homing
  - Homes X, Y, then Z sequentially

#### Probing

- **G38.2 X Y Z F**: Probe toward position (stops when probe triggers)
  - Reports probe position: `[PRB:X.xxx,Y.yyy,Z.zzz:1]`
  - Requires probe input to be configured
  - Moves all three axes toward target, stops when probe triggers

### M-codes

- **M0**: Program stop
- **M1**: Optional stop
- **M3/M4/M5**: Spindle control (acknowledged but not implemented)
- **M30**: Program end

### Configuration Commands ($)

- **`$`**: Show help
- **`$G`**: Show G-code parser state
- **`$I`**: Show build info
- **`$N`**: Show startup blocks

### Error Codes

- **error:1**: G-code words consist of a letter and a value
- **error:2**: Numeric value format is not valid or missing an expected value
- **error:9**: Homing cycle not started (motors not enabled)
- **error:20**: Unsupported command
- **error:23**: Buffer overflow
- **error:24**: Line exceeds maximum line length

## Usage

### Connecting with GRBL Software

1. **Build and flash** the example to ClearCore
2. **Connect** via USB serial (115200 baud)
3. **Open GRBL-compatible software**:
   - Universal G-code Sender
   - bCNC
   - CNCjs
   - Candle
   - Other GRBL streaming tools
4. **Configure connection**:
   - Port: USB serial port (COM port on Windows, /dev/ttyACM0 on Linux)
   - Baud rate: 115200
   - Protocol: GRBL

### Example Session

```
Grbl 1.1f ['$' for help]
> G21
ok
> G90
ok
> G01 X10 Y10 Z5 F100
ok
<Run,MPos:10.000,10.000,5.000,FS:100,SS:0>
<Idle,MPos:10.000,10.000,5.000,FS:100,SS:0>
> G02 X20 Y20 Z5 I5 J5 F100
ok
<Run,MPos:20.000,20.000,5.000,FS:100,SS:0>
```

### Feed Hold Example

```
> G01 X50 Y50 Z10 F100
ok
<Run,MPos:25.000,25.000,7.500,FS:100,SS:0>
> !                    (feed hold)
<Hold,MPos:30.000,30.000,9.000,FS:100,SS:0>
> ~                    (resume)
<Run,MPos:30.000,30.000,9.000,FS:100,SS:0>
```

## Differences from Standard GRBL

This implementation supports **3-axis motion**:
- X and Y axes are coordinated (for arcs and linear moves)
- Z-axis moves independently (not coordinated with X/Y)
- Arcs (G02/G03) are 2D in X/Y plane; Z moves linearly during arcs
- Spindle control (M3/M4/M5) is acknowledged but not implemented

## Building

1. Open `GRBLCompatible.atsln` in Microchip Studio
2. Set GRBLCompatible as startup project
3. Build and flash to ClearCore

## Configuration

Edit `GRBLCompatible.cpp` to configure:

- **Motor steps per revolution**: `MOTOR_X_STEPS_PER_REV`, `MOTOR_Y_STEPS_PER_REV`, `MOTOR_Z_STEPS_PER_REV`
- **Pitch**: `MOTOR_X_PITCH_MM`, `MOTOR_Y_PITCH_MM`, `MOTOR_Z_PITCH_MM`
- **Default feed rate**: `DEFAULT_FEED_RATE_MM_PER_MIN`
- **Status report interval**: `STATUS_REPORT_INTERVAL` (default 250ms)

### Input Configuration

Configure optional inputs by editing the pin definitions:

```cpp
// Home switches
#define HOME_X_PIN ConnectorIO0  // X-axis home switch
#define HOME_Y_PIN ConnectorIO1  // Y-axis home switch
#define HOME_Z_PIN ConnectorDI7  // Z-axis home switch
#define HOME_SWITCH_ACTIVE_LOW true  // Switch polarity

// Probe
#define PROBE_PIN ConnectorIO2    // Touch probe input
#define PROBE_ACTIVE_LOW true     // Probe polarity

// Limit switches
#define LIMIT_X_MIN_PIN ConnectorIO3  // X-axis minimum limit
#define LIMIT_X_MAX_PIN ConnectorIO4  // X-axis maximum limit
#define LIMIT_Y_MIN_PIN ConnectorIO5  // Y-axis minimum limit
#define LIMIT_Y_MAX_PIN ConnectorDI6  // Y-axis maximum limit
#define LIMIT_Z_MIN_PIN ConnectorDI8  // Z-axis minimum limit
#define LIMIT_Z_MAX_PIN ConnectorA9   // Z-axis maximum limit
#define LIMIT_SWITCH_ACTIVE_LOW true  // Switch polarity
```

To disable any input, comment out its `#define` line.

**Switch Polarity**:
- `ACTIVE_LOW true`: Switch is closed (triggered) when input reads 0V (low)
- `ACTIVE_LOW false`: Switch is closed (triggered) when input reads 5V (high)

## Notes

- Motors are automatically enabled on startup
- Commands queue automatically if motion is active
- Feed rate persists until changed
- Coordinate and unit modes persist until changed
- Status reports are sent automatically every 250ms
- X/Y axes move coordinated (for arcs); Z axis moves independently
- For 3-axis moves, X/Y execute coordinated while Z executes in parallel
- X/Y axes move coordinated (for arcs); Z axis moves independently
- For 3-axis moves, X/Y execute coordinated while Z executes in parallel

## Compatibility

This example is compatible with:
- **Universal G-code Sender** (UGS)
- **bCNC**
- **CNCjs**
- **Candle**
- **Other GRBL-compatible streaming software**

## Links

- **ClearCore Documentation**: https://teknic-inc.github.io/ClearCore-library/
- **GRBL Documentation**: https://github.com/gnea/grbl/wiki
- **Universal G-code Sender**: https://winder.github.io/ugs_website/
