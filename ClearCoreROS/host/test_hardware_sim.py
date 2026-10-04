"""Simulated ros2_control contracts. These do not open a board or controller_manager.

They encode the plugin rules in clearcore_system.cpp: a watchdog flag latches
until the next on_activate (which calls clear_alerts), a stream EOF is an
error, and home is not part of activate.
"""

from __future__ import annotations

import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "ros2_ws" / "src" / "clearcore_bridge"))

from clearcore_bridge.wire import FLAG_WATCHDOG, feed, pack_state  # noqa: E402

# Matches on_activate in clearcore_system.cpp. Home is not in this list.
PLUGIN_ACTIVATE_METHODS = (
    "disable",
    "clear_alerts",
    "configure",
    "set_test_mode",
    "enable",
)
PLUGIN_DEACTIVATE_METHODS = ("disable",)

# action_msgs/GoalStatus and control_msgs FollowJointTrajectory.Result
STATUS_SUCCEEDED = 4
STATUS_CANCELED = 5
STATUS_ABORTED = 6
RESULT_SUCCESSFUL = 0


def plugin_watchdog_latches(flags: int) -> bool:
    """Same test as drain_stream: flags & 0x10 sets fault_latched_."""
    return bool(flags & FLAG_WATCHDOG)


def plugin_write_allowed(stream_open: bool, fault_latched: bool) -> bool:
    """Same gate as ClearCoreSystemHardware::write."""
    return stream_open and not fault_latched


def jtc_goal_succeeded(status: int, error_code: int) -> bool:
    """Same rule as send_trajectory_goal.py: canceled is not success."""
    return status == STATUS_SUCCEEDED and error_code == RESULT_SUCCESSFUL


def test_watchdog_frame_latches_and_blocks_write():
    raw = pack_state(flags=FLAG_WATCHDOG, position=(0.01, 0.0, 0.0, 0.0))
    parsed = feed(bytearray(raw))
    assert parsed[0]["watchdog"] is True
    assert plugin_watchdog_latches(parsed[0]["flags"]) is True
    assert plugin_write_allowed(stream_open=True, fault_latched=True) is False
    # Recovery is the next activate (clear_alerts), not a keepalive write.
    assert plugin_write_allowed(stream_open=True, fault_latched=False) is True


def test_stream_eof_is_a_read_error():
    """recv() returning 0 is return_type::ERROR in drain_stream."""
    assert plugin_write_allowed(stream_open=False, fault_latched=False) is False


def test_host_restart_is_deactivate_then_activate():
    assert PLUGIN_DEACTIVATE_METHODS == ("disable",)
    assert PLUGIN_ACTIVATE_METHODS[0] == "disable"
    assert PLUGIN_ACTIVATE_METHODS[1] == "clear_alerts"
    assert "home" not in PLUGIN_ACTIVATE_METHODS
    assert "home" not in PLUGIN_DEACTIVATE_METHODS


def test_home_is_not_a_plugin_activate_step():
    """Physical homing is session `home`. CI and activate do not call it."""
    assert "home" not in PLUGIN_ACTIVATE_METHODS


def test_canceled_jtc_result_is_failure_even_if_error_code_is_zero():
    assert jtc_goal_succeeded(STATUS_SUCCEEDED, RESULT_SUCCESSFUL) is True
    assert jtc_goal_succeeded(STATUS_CANCELED, RESULT_SUCCESSFUL) is False
    assert jtc_goal_succeeded(STATUS_ABORTED, RESULT_SUCCESSFUL) is False


if __name__ == "__main__":
    test_watchdog_frame_latches_and_blocks_write()
    test_stream_eof_is_a_read_error()
    test_host_restart_is_deactivate_then_activate()
    test_home_is_not_a_plugin_activate_step()
    test_canceled_jtc_result_is_failure_even_if_error_code_is_zero()
    print("hardware_sim ok")
