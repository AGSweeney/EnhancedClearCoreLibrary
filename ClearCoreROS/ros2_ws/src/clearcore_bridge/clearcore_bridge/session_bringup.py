"""Session bring-up shared by the bridge node and simulated-host tests.

No ROS imports. Keepalive is not recovery; clear_alerts is.
"""

from __future__ import annotations

from clearcore_bridge.wire import apply_joint_map


def bring_up_session(
    session,
    *,
    axis_mask: int,
    steps_per_rev: int = 800,
    pitch_mm: float = 5.0,
    vel_steps: int = 27000,
    accel_steps: int = 250000,
    watchdog_ms: int = 500,
    test_mode: bool = False,
):
    """Disable, clear_alerts, configure, set_test_mode, enable. Returns get_config."""
    session.call("disable")
    session.call("clear_alerts")
    session.call(
        "configure",
        {
            "axis_mask": axis_mask,
            "steps_per_rev": int(steps_per_rev),
            "pitch_mm": float(pitch_mm),
            "vel_steps": int(vel_steps),
            "accel_steps": int(accel_steps),
            "decel_steps": int(accel_steps),
            "watchdog_ms": int(watchdog_ms),
        },
    )
    session.call("set_test_mode", {"on": bool(test_mode)})
    cfg = session.call("get_config")
    if isinstance(cfg, dict):
        apply_joint_map(cfg.get("names"), cfg.get("rotary"))
    session.call("enable")
    return cfg


def recover_watchdog(session) -> None:
    """Clear the firmware watchdog latch. keepalive is not a substitute."""
    session.call("clear_alerts")
