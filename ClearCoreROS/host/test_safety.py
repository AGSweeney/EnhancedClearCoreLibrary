"""Host-side checks for the motion bugs found in review."""

from __future__ import annotations

import sys
import threading
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "ros2_ws" / "src" / "clearcore_bridge"))

from clearcore_bridge.follow import (  # noqa: E402
    GoalGate,
    build_knots,
    limit_step,
    motion_succeeded,
    path_violation,
    sample_trajectory,
    state_block_reason,
)
from clearcore_bridge.wire import StreamClient, feed, pack_track  # noqa: E402


class _ClosedSock:
    def setsockopt(self, *args):
        pass

    def settimeout(self, timeout):
        pass

    def recv(self, _n):
        return b""

    def close(self):
        pass


def test_track_frame_roundtrip():
    raw = pack_track(4, 0x01, (0.01, 0.0, 0.0, 0.0), (0.02, 0.0, 0.0, 0.0))
    parsed = feed(bytearray(raw))
    assert parsed[0]["type"] == "track"
    assert parsed[0]["seq"] == 4 and parsed[0]["mask"] == 0x01
    assert abs(parsed[0]["position"][0] - 0.01) < 1e-7
    assert abs(parsed[0]["velocity"][0] - 0.02) < 1e-7


def test_stream_eof_is_disconnect():
    client = StreamClient.__new__(StreamClient)
    client.sock = _ClosedSock()
    client._buf = bytearray()
    client._seq = 1
    client._send_lock = threading.Lock()
    try:
        client.read(0.1)
    except ConnectionError as exc:
        assert "closed" in str(exc)
    else:
        raise AssertionError("EOF was treated as an empty update")


def _healthy(**overrides):
    state = {
        "enabled": True,
        "moving": False,
        "estop": False,
        "fault": False,
        "watchdog": False,
        "position": (0.01, 0.0, 0.0, 0.0),
        "velocity": (0.0, 0.0, 0.0, 0.0),
    }
    state.update(overrides)
    return state


def test_cached_faulted_state_is_not_success():
    """The reviewed failure: disconnected, faulted, disabled, already at the target."""
    cached = _healthy(enabled=False, fault=True)
    assert motion_succeeded(cached, 0.0, ["joint_x"], [0.01], {}, 0.001) is False
    assert state_block_reason(cached, 0.0) == "fault"
    assert motion_succeeded(_healthy(), 1.0, ["joint_x"], [0.01], {}, 0.001) is False
    assert state_block_reason(_healthy(), 1.0) == "stale state"
    assert state_block_reason(None, 0.0) == "no state"
    assert motion_succeeded(_healthy(), 0.0, ["joint_x"], [0.01], {}, 0.001) is True
    assert motion_succeeded(_healthy(watchdog=True), 0.0, ["joint_x"], [0.01], {}, 0.001) is False
    assert motion_succeeded(_healthy(enabled=False), 0.0, ["joint_x"], [0.01], {}, 0.001) is False


def test_second_goal_is_rejected_and_cancel_is_not_shared():
    gate = GoalGate()
    assert gate.try_reserve()
    assert not gate.try_reserve()
    assert gate.claim("goal-a")
    assert gate.accepts_cancel("goal-a")
    assert not gate.accepts_cancel("goal-b")
    gate.release("goal-a")
    assert gate.try_reserve()
    assert gate.accepts_cancel("goal-b")
    gate.release_pending()
    assert not gate.busy()


def test_trajectory_uses_time_and_requested_velocity():
    names = ["joint_x"]
    points = [
        {"t": 0.0, "positions": {"joint_x": 0.0}, "velocities": None, "accelerations": None},
        {"t": 1.0, "positions": {"joint_x": 0.1}, "velocities": None, "accelerations": None},
    ]
    knots = build_knots(names, points, {"joint_x": 0.0})
    pos, vel, done = sample_trajectory(knots, names, 0.5, {"joint_x": 0.0}, 0.02)
    assert not done
    assert abs(pos["joint_x"] - 0.05) < 1e-6
    assert abs(vel["joint_x"] - 0.1) < 1e-6

    pointed = [
        {"t": 0.0, "positions": {"joint_x": 0.0}, "velocities": {"joint_x": 0.0}, "accelerations": None},
        {"t": 1.0, "positions": {"joint_x": 0.1}, "velocities": {"joint_x": 0.0}, "accelerations": None},
    ]
    shaped = build_knots(names, pointed, {"joint_x": 0.0})
    _pos, shaped_vel, _done = sample_trajectory(shaped, names, 0.5, {"joint_x": 0.0}, 0.02)
    assert abs(shaped_vel["joint_x"] - 0.15) < 1e-6

    limited = limit_step(0.0, 1.0, 1.0, 0.02)
    assert abs(limited - 0.02) < 1e-9
    state = _healthy(position=(0.0, 0.0, 0.0, 0.0))
    assert path_violation(state, {"joint_x": 0.05}, {"joint_x": 0.01}) == "joint_x"
    assert path_violation(state, {"joint_x": 0.05}, {}) is None


if __name__ == "__main__":
    test_track_frame_roundtrip()
    test_stream_eof_is_disconnect()
    test_cached_faulted_state_is_not_success()
    test_second_goal_is_rejected_and_cancel_is_not_shared()
    test_trajectory_uses_time_and_requested_velocity()
    print("safety ok")
