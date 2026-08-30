@echo off
set PYTHONUNBUFFERED=1
if not defined CLEARAI_TCP_PORT set CLEARAI_TCP_PORT=9100
cd /d "%~dp0"
"C:\Python313\python.exe" -u -m clearai.mcp
