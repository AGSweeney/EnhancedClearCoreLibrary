"""colcon smoke: the package imports without a board."""

from clearcore_bridge.follow import GoalGate
from clearcore_bridge.wire import FLAG_WATCHDOG, pack_state


def test_goal_gate():
    gate = GoalGate()
    assert gate.try_reserve()
    gate.release_pending()


def test_watchdog_flag_in_packed_state():
    frame = pack_state(flags=FLAG_WATCHDOG)
    assert frame[12] & FLAG_WATCHDOG
