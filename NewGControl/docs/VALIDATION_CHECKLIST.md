# Validation Checklist

Use this checklist when validating `NewGControl` against a target controller profile.

## Test Setup

- Firmware and profile under test documented (`GRBL 1.1`, `grblHAL`, or `FluidNC`).
- Transport documented (`Serial` or `Ethernet`).
- Safe machine state confirmed before motion tests.

## Connection and State

- Connect succeeds and profile banner appears in log.
- Status updates populate DRO values.
- Disconnect/ reconnect works without restart.

## Motion and Realtime

- MDI move executes and reports completion.
- Jog buttons execute expected incremental motion.
- `Pause` issues feed-hold and machine decelerates.
- `Resume` continues motion.
- `Stop` halts active job safely.
- Emergency stop / soft reset path behaves as expected.

## Program Streaming

- Load `.nc/.tap/.gcode` file.
- Run from start to end with status updates.
- Pause/resume at least once mid-job.
- Verify stop path and ability to re-run.
- Confirm alarm/error is surfaced in UI when firmware reports one.

## GRBL Configuration

- `Read $$` populates settings table.
- Edit one writable setting and apply.
- `Read $#`/`$G`/`$I`/`$N` retrieval works.
- Export snapshot JSON and re-import successfully.

## Reporting

- Record pass/fail for each checklist row.
- Capture controller firmware version/build string.
- Document profile-specific gaps in compatibility matrix notes.
