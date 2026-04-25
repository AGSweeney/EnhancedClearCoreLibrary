# G-Code Streaming Guide

This guide explains how to connect to the Motion Streaming example and send G-code commands.

## Prerequisites

1. Build and flash the `MotionStreaming` example to your ClearCore
2. Ensure motors are connected and configured (M-0 and M-1)
3. Choose your communication method: Serial (USB) or Ethernet

---

## Method 1: Serial (USB CDC) Streaming

### Step 1: Configure for Serial Mode

In `MotionStreaming.cpp`, ensure:
```cpp
#define COMM_MODE SERIAL_MODE
```

### Step 2: Connect via USB

1. Connect ClearCore to your computer via USB cable
2. The ClearCore will appear as a COM port (e.g., COM3, COM4, etc.)
3. Note the COM port number from Device Manager (Windows) or `ls /dev/tty*` (Linux/Mac)

### Step 3: Open Serial Terminal

**Windows Options:**
- **PuTTY**: Download from https://www.putty.org/
  - Connection type: Serial
  - Serial line: COM3 (use your COM port)
  - Speed: 115200
  - Click "Open"
  
- **Tera Term**: Download from https://ttssh2.osdn.jp/
  - Setup → Serial port
  - Port: COM3 (use your COM port)
  - Speed: 115200
  - Click "OK"

- **Arduino Serial Monitor**: If you have Arduino IDE installed
  - Tools → Port → Select COM port
  - Set baud rate to 115200
  - Open Serial Monitor

- **Windows PowerShell/Command Prompt**:
  ```powershell
  # List COM ports
  Get-WmiObject Win32_SerialPort | Select-Object DeviceID, Description
  
  # Connect (replace COM3 with your port)
  # Use a terminal emulator - PowerShell doesn't have built-in serial support
  ```

**Linux/Mac Options:**
- **minicom**: 
  ```bash
  sudo minicom -D /dev/ttyACM0 -b 115200
  ```
  (Replace `/dev/ttyACM0` with your device)

- **screen**:
  ```bash
  screen /dev/ttyACM0 115200
  ```

- **Arduino Serial Monitor**: Works on Linux/Mac too

### Step 4: Send Commands

Once connected, you should see:
```
Motion Streaming Example Ready
Send G-code commands (G01, G02, G03, etc.)
```

Type commands and press Enter:
```
G21
G90
M202
G01 X10 Y10 F100
M114
```

Each command should receive an `ok` response.

---

## Method 2: Ethernet (TCP) Streaming

### Step 1: Configure for Ethernet Mode

In `MotionStreaming.cpp`, change:
```cpp
#define COMM_MODE ETHERNET_MODE
```

Also configure network settings:
```cpp
bool usingDhcp = true;  // Use DHCP (recommended)
// OR
bool usingDhcp = false;
IpAddress manualIp = IpAddress(192, 168, 0, 109);  // Manual IP
```

### Step 2: Connect Ethernet Cable

1. Connect Ethernet cable from ClearCore to your network
2. Build and flash the updated code
3. Check USB serial output (if connected) to see the assigned IP address:
   ```
   DHCP IP: 192.168.1.100
   TCP server listening on port 8888
   ```

### Step 3: Connect TCP Client

**Windows Options:**

- **PuTTY**:
  1. Connection type: Raw
  2. Host Name: `192.168.1.100` (use your ClearCore IP)
  3. Port: `8888`
  4. Click "Open"

- **Tera Term**:
  1. Setup → TCP/IP
  2. Host: `192.168.1.100` (use your ClearCore IP)
  3. Port: `8888`
  4. TCP
  5. Click "OK"

- **PowerShell** (using `Test-NetConnection` to test, but use PuTTY/Tera Term for actual streaming):
  ```powershell
  Test-NetConnection -ComputerName 192.168.1.100 -Port 8888
  ```

- **Python Script** (example):
  ```python
  import socket
  
  # Connect to ClearCore
  sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
  sock.connect(('192.168.1.100', 8888))
  
  # Send commands
  commands = [
      'G21\n',
      'G90\n',
      'M202\n',
      'G01 X10 Y10 F100\n',
      'M114\n'
  ]
  
  for cmd in commands:
      sock.send(cmd.encode())
      response = sock.recv(1024).decode()
      print(f"Sent: {cmd.strip()}, Received: {response.strip()}")
  
  sock.close()
  ```

**Linux/Mac Options:**

- **netcat (nc)**:
  ```bash
  nc 192.168.1.100 8888
  ```
  Then type commands and press Enter

- **telnet**:
  ```bash
  telnet 192.168.1.100 8888
  ```

- **Python** (same as Windows example above)

### Step 4: Send Commands

Once connected, you should see the ready message. Send commands same as serial mode:
```
G21
G90
M202
G01 X10 Y10 F100
M114
```

---

## Command Examples

### Basic Setup Sequence
```
G21          ; Set units to millimeters
G90          ; Set absolute coordinates
M202         ; Enable motors
G92 X0 Y0    ; Set current position as origin (after homing)
```

### Simple Square Pattern
```
G92 X0 Y0           ; Set current position as origin
G01 X0 Y0 F100      ; Move to origin (if not already there)
G01 X50 Y0 F100     ; Move right 50mm
G01 X50 Y50 F100    ; Move up 50mm
G01 X0 Y50 F100     ; Move left 50mm
G01 X0 Y0 F100      ; Return to origin
```

### Arc Move Example
```
G01 X0 Y0 F100           ; Start at origin
G02 X50 Y50 I25 J0 F100 ; Arc to (50,50) with center offset (25,0)
```

### Continuous Motion (Automatic Queuing)
```
G01 X10 Y0 F100
G01 X10 Y10 F100    ; Queued automatically, executes after first completes
G01 X0 Y10 F100     ; Also queued
G01 X0 Y0 F100      ; Also queued
```
**Important**: Commands are automatically queued! If motion is active when you send a command, it will be queued (up to 8 commands). If no motion is active, the command executes immediately. This allows you to stream commands continuously without waiting for each to complete.

### Emergency Stop Commands
```
M200         ; Emergency stop - immediate abrupt stop, clear queue
M201         ; Stop with deceleration - smooth stop, clear queue
M203         ; Disable motors - stop motion, clear queue, disable motors
```
**Important**: 
- **M200**: Immediately stops all motion (abrupt) and clears the command queue. Use for emergency situations.
- **M201**: Stops motion smoothly with deceleration and clears queue. Use when you want controlled stopping.
- **M203**: Stops motion, clears queue, and disables motors. Use for shutdown.

### Status Queries
```
M114         ; Get current position
M115         ; Get status (active, queue count, units, coords)
```

---

## Troubleshooting

### Serial Mode Issues

**Problem**: Can't find COM port
- **Solution**: Check Device Manager (Windows) or `dmesg` (Linux) for USB device
- Ensure USB cable is data-capable (not charge-only)

**Problem**: No response to commands
- **Solution**: 
  - Verify baud rate is 115200
  - Check that you're sending newline (`\n`) after each command
  - Ensure motors are enabled (send `M202`)

**Problem**: Motion commands return "error: Motors not enabled"
- **Solution**: 
  - Motors must be enabled before sending motion commands (G01, G02, G03)
  - Send `M202` to enable motors
  - Wait for `ok` response before sending motion commands
  - If motors were disabled (M203), you must re-enable them (M202) before motion will work

**Problem**: Commands received but motion doesn't execute
- **Solution**:
  - Check motor connections (M-0 and M-1)
  - Verify motors are enabled: `M202`
  - Check for motor alerts/faults
  - Verify mechanical parameters match your system

### Ethernet Mode Issues

**Problem**: Can't connect to TCP server
- **Solution**:
  - Verify IP address from USB serial output
  - Check Ethernet cable is connected
  - Verify network connectivity: `ping 192.168.1.100`
  - Check firewall isn't blocking port 8888

**Problem**: Connection drops
- **Solution**:
  - Check Ethernet cable quality
  - Verify network stability
  - Reconnect TCP client

**Problem**: DHCP fails
- **Solution**:
  - Set `usingDhcp = false` and use manual IP
  - Ensure manual IP is on same subnet as your network
  - Check network configuration

---

## Advanced Usage

### Scripting with Python

Create a file `send_gcode.py`:
```python
#!/usr/bin/env python3
import socket
import time
import sys

def send_gcode(host, port, commands):
    """Send G-code commands to ClearCore"""
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.connect((host, port))
        sock.settimeout(5.0)
        
        print(f"Connected to {host}:{port}")
        
        # Read initial message
        response = sock.recv(1024).decode()
        print(f"Server: {response.strip()}")
        
        for cmd in commands:
            if not cmd.strip():
                continue
                
            # Send command
            full_cmd = cmd.strip() + '\n'
            sock.send(full_cmd.encode())
            print(f"Sent: {cmd.strip()}")
            
            # Wait for response
            response = sock.recv(1024).decode()
            print(f"Response: {response.strip()}")
            
            time.sleep(0.1)  # Small delay between commands
        
        sock.close()
        print("Disconnected")
        
    except Exception as e:
        print(f"Error: {e}")
        sys.exit(1)

if __name__ == "__main__":
    # Configuration
    HOST = "192.168.1.100"  # Change to your ClearCore IP
    PORT = 8888
    
    # G-code commands
    commands = [
        "G21",           # Millimeters
        "G90",           # Absolute
        "M202",          # Enable motors
        "G01 X0 Y0 F100", # Home
        "G01 X50 Y0 F100", # Move right
        "G01 X50 Y50 F100", # Move up
        "G01 X0 Y50 F100",  # Move left
        "G01 X0 Y0 F100",   # Return home
        "M114",          # Query position
    ]
    
    send_gcode(HOST, PORT, commands)
```

Run with:
```bash
python send_gcode.py
```

### Using G-Code Sender Software

Many CNC control software packages can be configured to send G-code over serial or TCP:

- **Universal G-Code Sender (UGS)**: https://winder.github.io/ugs_website/
  - Configure connection as serial (COM port) or TCP (IP:port)
  - Load G-code file or type commands manually

- **CNCjs**: https://cnc.js.org/
  - Supports serial and TCP connections
  - Web-based interface

- **Grbl Controller**: Various implementations
  - Configure for serial connection

---

## Tips

1. **Always enable motors first**: Send `M202` before motion commands
2. **Set units and coordinates**: `G21` (mm) or `G20` (inches), `G90` (absolute) or `G91` (incremental)
3. **Use feed rates**: Include `F` parameter in motion commands for controlled speed
4. **Check status**: Use `M114` and `M115` to verify system state
5. **Queue management**: Commands automatically queue if motion is active
6. **Error handling**: If you get an error, check motor status and connections

---

## Example Session

```
> G21
ok
> G90
ok
> M202
ok
> G01 X0 Y0 F100
ok
> G01 X10 Y10 F100
ok
> M114
X:10.000 Y:10.000
ok
> G02 X20 Y20 I5 J5 F100
ok
> M115
Status: Active=1 Queue=0 Units=mm Coords=abs
ok
> M203
ok
```

---

For more information, see the main README.md file.
