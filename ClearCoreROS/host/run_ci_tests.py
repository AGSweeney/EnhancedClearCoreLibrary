"""Run ClearCoreROS host tests and print a CI result table.

This is simulated-host coverage. It does not home motors, trip a physical
watchdog, or recover a board fault.
"""

from __future__ import annotations

import runpy
import sys
from pathlib import Path

HOST = Path(__file__).resolve().parent
SCRIPTS = (
    "test_wire.py",
    "test_safety.py",
    "test_session_bringup.py",
    "test_hardware_sim.py",
    "test_xrce_payloads.py",
)


def main() -> int:
    print("CI label: automated with simulated hardware")
    print("A passing run does not mean physical homing or fault recovery was exercised.")
    failed = []
    for name in SCRIPTS:
        path = HOST / name
        print(f"RUN {name}", flush=True)
        try:
            runpy.run_path(str(path), run_name="__main__")
        except SystemExit as exc:
            if exc.code not in (0, None):
                failed.append(name)
                print(f"FAIL {name} exit={exc.code}", flush=True)
        except Exception as exc:  # noqa: BLE001
            failed.append(name)
            print(f"FAIL {name}: {exc}", flush=True)
        else:
            print(f"PASS {name}", flush=True)
    if failed:
        print("FAILED: " + ", ".join(failed))
        return 1
    print("HOST_CI_OK")
    return 0


if __name__ == "__main__":
    sys.exit(main())
