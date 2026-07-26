# Project docs

ClearCNC / ClearHPGL / NewGControl application knowledge.

| Area | Path |
|------|------|
| Inventory | [COMPONENT_INVENTORY.md](COMPONENT_INVENTORY.md) |
| Architecture | [architecture/system-overview.md](architecture/system-overview.md) |
| Subsystems | [subsystems/README.md](subsystems/README.md) |
| Recipes | [recipes/README.md](recipes/README.md) |
| Open questions | [OPEN_QUESTIONS.md](OPEN_QUESTIONS.md) |

## Alias table

| Term | Meaning |
|------|---------|
| Planner queue | L01 `ARC_QUEUE_SIZE` / `MotionQueueCount` |
| Firmware queue | P01 `MOTION_QUEUE_SIZE` (16) |
| Coordinated mode | L06 `CoordinatedMotionMode` + L01 active/queued |
| ClearCNC protocol | P01 line commands (not GRBL) |
