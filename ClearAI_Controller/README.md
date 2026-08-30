# ClearAI Controller

**Experimental.** Not certified for production or safety-critical use. Use with machine safeguards.

JSON-RPC Lines firmware and hosts so an LLM can command ClearPath servos on ClearCore (**Code as Policies** primitives — not G-code). Optional **LocoRix** MCP reads Helios ToF Detect numbers for hole CX.

## Documentation

| Doc | Description |
|-----|-------------|
| **[docs/ARCHITECTURE.md](docs/ARCHITECTURE.md)** | System overview, ports, layers |
| **[docs/FIRMWARE.md](docs/FIRMWARE.md)** | Build, flash, modules, ClearPath setup, test mode |
| **[PROTOCOL.md](PROTOCOL.md)** | Wire protocol and primitive reference (source of truth) |
| **[docs/HOST_PYTHON.md](docs/HOST_PYTHON.md)** | Python CLI and library |
| **[docs/MCP_CLEARAI.md](docs/MCP_CLEARAI.md)** | `clearai-mcp.exe` tools and env |
| **[docs/MCP_LOCORIX.md](docs/MCP_LOCORIX.md)** | `locorix-mcp.exe` Detect → CX |
| **[docs/AI_CLIENTS.md](docs/AI_CLIENTS.md)** | Cursor and LM Studio `mcp.json` |
| **[docs/VISION_MOTION.md](docs/VISION_MOTION.md)** | Detect → plan → move loop |
| **[docs/TROUBLESHOOTING.md](docs/TROUBLESHOOTING.md)** | Common failures |

## Repository layout

| Path | Role |
|------|------|
| `ClearCoreFirmware/` | Microchip Studio firmware (`ClearAI_Firmware.atsln`) |
| `PROTOCOL.md` | JSON-RPC Lines command language |
| `python/` | Serial/TCP CLI and CaP helpers |
| `mcp/` | Official Go MCP → ClearAI TCP 9100 |
| `locorix-mcp/` | Go MCP → LocoRix Helios Detect |
| `docs/` | Full documentation |

## Quick start

### 1. Firmware

Open `ClearCoreFirmware/ClearAI_Firmware.atsln` in Microchip Studio 7, build, flash (`Tools/flash_clearcore.cmd`). Details: [docs/FIRMWARE.md](docs/FIRMWARE.md).

**ClearPath:** Step + Direction; HLFB ASG-Position w/Measured Torque @ 482 Hz.

### 2. One-motor bench (Ethernet)

```powershell
cd ClearAI_Controller\python
$env:PYTHONPATH="."
python -m clearai --host 172.16.82.113 disable
python -m clearai --host 172.16.82.113 configure --axis-mask 1 --vel 27000 --accel 250000 --decel 250000
python -m clearai --host 172.16.82.113 set_test_mode --on
python -m clearai --host 172.16.82.113 enable
python -m clearai --host 172.16.82.113 move_linear --x 10 --feed 500
python -m clearai --host 172.16.82.113 wait_idle --timeout-ms 15000
python -m clearai --host 172.16.82.113 get_pose
```

Replace the host IP with your board. Ports: control **9100**, telemetry **9101**, discover **9102**. Do not collide with ClearCNC (8888/8889/10040).

### 3. MCP (Cursor or LM Studio)

Build:

```powershell
cd ClearAI_Controller\mcp
go build -o clearai-mcp.exe .

cd ..\locorix-mcp
go build -o locorix-mcp.exe .
```

`mcp.json` snippet (full guide: [docs/AI_CLIENTS.md](docs/AI_CLIENTS.md)):

```json
{
  "mcpServers": {
    "clearai": {
      "command": "D:/CCDev/EnhancedClearCoreLibrary/ClearAI_Controller/mcp/clearai-mcp.exe",
      "args": [],
      "env": {
        "CLEARAI_HOST": "172.16.82.113",
        "CLEARAI_TCP_PORT": "9100",
        "CLEARAI_FALLBACK_IP": "192.168.0.109"
      }
    },
    "locorix": {
      "command": "D:/CCDev/EnhancedClearCoreLibrary/ClearAI_Controller/locorix-mcp/locorix-mcp.exe",
      "args": [],
      "env": {
        "LOCORIX_BASE_URL": "http://172.16.82.121:8080",
        "LOCORIX_CAMERA_ID": "223400130",
        "LOCORIX_PRESET": "hole-locate"
      }
    }
  }
}
```

**Only one** clearai MCP may hold TCP 9100 at a time.

### 4. Agent habits

- After `move_linear` / `jog` → always `wait_idle`.  
- Hole CX → `locorix` tools only; never invent numbers.  
- Prefer a tool-calling Instruct model (e.g. Qwen2.5-7B+), not tiny nano chat models.

## Safety

- Hardware estop (DI-6) and alerts apply unless **test mode** (bench only).  
- Software `stop` / `estop` / `disable` always halt.  
- Physical estop remains mandatory around live axes.
