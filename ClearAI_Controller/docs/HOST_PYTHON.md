# ClearAI Python host

CLI and library under `ClearAI_Controller/python/`. Useful for bench scripts and debugging. For Cursor / LM Studio, prefer the **Go** MCP binaries (`mcp/`, `locorix-mcp/`).

## Setup

```powershell
cd ClearAI_Controller\python
pip install -r requirements.txt
$env:PYTHONPATH = (Get-Location).Path
```

## CLI

```text
python -m clearai [--port COMx | --host IP] [--tcp-port 9100] <command> ...
```

If neither `--port` nor `--host` is given, the CLI UDP-discovers on port 9102.

### Commands

Same names as protocol methods: `get_capabilities`, `get_status`, `get_pose`, `enable`, `disable`, `clear_alerts`, `stop`, `estop`, `wait_idle`, `configure`, `set_test_mode`, `set_units`, `set_mode`, `set_work_origin`, `move_linear`, `move_arc`, `jog`, `dwell`.

Also:

| Command | Purpose |
|---------|---------|
| `discover` | Print board IP from UDP |
| `mcp` | Legacy Python MCP stdio (prefer `clearai-mcp.exe`) |

### Examples

```powershell
python -m clearai --host 172.16.82.113 get_status
python -m clearai --host 172.16.82.113 move_linear --x 50 --feed 2000
python -m clearai --host 172.16.82.113 wait_idle --timeout-ms 15000
python -m clearai --host 172.16.82.113 get_pose
```

```powershell
python -m clearai --port COM4 set_test_mode --on
python -m clearai --port COM4 configure --axis-mask 1
```

## Library layout

| Module | Role |
|--------|------|
| `clearai/client.py` | Serial + TCP JSONL client, `discover()` |
| `clearai/primitives.py` | CaP-style functions for few-shot Python |
| `clearai/cli.py` | Argument parsers |
| `clearai/tools.json` | OpenAI/Anthropic-style tool schemas |
| `clearai/mcp_server.py` | Legacy MCP (not recommended for Cursor) |

### Few-shot style

```python
from clearai.primitives import connect_tcp, enable, move_linear, wait_idle, get_pose

connect_tcp("172.16.82.113")
enable()
move_linear(x=10, feed=500)
wait_idle(timeout_ms=20000)
print(get_pose())
```

## Serial caveats

- Opening USB CDC can reset the board depending on DTR; the client uses `dsrdtr=True` to reduce that.
- Do not leave COM held open while flashing or while another MCP expects Ethernet-only access.

## Related

- [PROTOCOL.md](../PROTOCOL.md)  
- [MCP_CLEARAI.md](MCP_CLEARAI.md)
