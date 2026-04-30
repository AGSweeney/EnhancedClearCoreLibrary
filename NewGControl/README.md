# NewGControl

`NewGControl` is a standalone GRBL-family desktop controller focused on gSender-style workflows.

## Scope Delivered

- Standalone Qt app target (`NewGControl`) with separate app identity and settings namespace.
- GRBL-family protocol adapter with profile detection for:
  - GRBL 1.1
  - grblHAL
  - FluidNC
- Program streaming baseline with run/pause/resume/stop and queue-aware behavior.
- GRBL configuration surfaces:
  - `$$` settings read and edit
  - `$#`, `$G`, `$I`, `$N` retrieval
  - Snapshot import/export (JSON)

## Build

Use your existing Qt6 setup and configure from this folder:

```powershell
cmake -S .\NewGControl -B .\build\NewGControl -DCMAKE_PREFIX_PATH="C:\Qt\6.x.x\msvc2022_64"
cmake --build .\build\NewGControl --config Release
```

## Notes

- In this environment, build verification can fail if Qt6 is not discoverable by CMake.
- Snapshot import/export stores only controller state data (not machine motion files).
