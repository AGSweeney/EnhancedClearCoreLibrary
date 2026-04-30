# GRBL Compatibility Matrix

This matrix defines the expected support level for `NewGControl` v1.

## Profiles

- `GRBL 1.1`: canonical gnea/grbl response and command model.
- `grblHAL`: GRBL superset with expanded settings and reports.
- `FluidNC`: GRBL-like protocol with board/network extensions.

## Feature Matrix

| Feature | GRBL 1.1 | grblHAL | FluidNC | Notes |
|---|---|---|---|---|
| Connect / disconnect | Yes | Yes | Yes | Serial baseline; TCP depends on firmware config |
| Status polling (`?`) | Yes | Yes | Yes | Parsed from `<...>` status frames |
| MDI (`G0/G1/...`) | Yes | Yes | Yes | Modal handling is firmware-specific |
| Jog (`$J=`) | Yes | Yes | Yes | Uses incremental jog format |
| Feed hold / resume (`!` / `~`) | Yes | Yes | Yes | Realtime channel |
| Soft reset (Ctrl-X) | Yes | Yes | Yes | Routed as realtime byte |
| Program run/pause/stop | Yes | Yes | Yes | Queue and ack behavior vary by firmware |
| Settings read (`$$`) | Yes | Yes | Yes | Parsed to settings manager table |
| Settings write (`$x=v`) | Yes | Yes | Yes* | `*` some values may be read-only |
| Parameters (`$#`) | Yes | Yes | Yes | Captured in manager metadata |
| Parser state (`$G`) | Yes | Yes | Yes | Captured in manager metadata |
| Build info (`$I`) | Yes | Yes | Yes | Variant detection input |
| Startup blocks (`$N`) | Yes | Yes | Partial | Firmware dependent |
| Snapshot export/import | Yes | Yes | Yes | JSON-based local file workflow |

## Validation Expectations

- Perform hardware validation on one board/controller per profile.
- Record firmware version string from startup/build info for each run.
- Keep profile-specific known limitations in release notes.
