# LocoRix MCP (Helios Detect → CX)

Go MCP server for LM Studio / Cursor. Structured Detect API only — no image pose estimation.

**Full documentation:** [../docs/MCP_LOCORIX.md](../docs/MCP_LOCORIX.md)  
**Vision → motion:** [../docs/VISION_MOTION.md](../docs/VISION_MOTION.md)  
**AI clients:** [../docs/AI_CLIENTS.md](../docs/AI_CLIENTS.md)

## Build

```powershell
cd D:\CCDev\EnhancedClearCoreLibrary\ClearAI_Controller\locorix-mcp
go mod tidy
go build -o locorix-mcp.exe .
```

## Quick mcp.json

```json
"locorix": {
  "command": "D:/CCDev/EnhancedClearCoreLibrary/ClearAI_Controller/locorix-mcp/locorix-mcp.exe",
  "args": [],
  "env": {
    "LOCORIX_BASE_URL": "http://172.16.82.121:8080",
    "LOCORIX_CAMERA_ID": "223400130",
    "LOCORIX_PRESET": "hole-locate"
  }
}
```
