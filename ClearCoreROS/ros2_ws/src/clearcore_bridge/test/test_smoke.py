"""colcon smoke: the package imports without a board."""

import unittest

from clearcore_bridge.follow import GoalGate
from clearcore_bridge.wire import FLAG_WATCHDOG, pack_state


class TestSmoke(unittest.TestCase):
    def test_goal_gate(self):
        gate = GoalGate()
        self.assertTrue(gate.try_reserve())
        gate.release_pending()

    def test_watchdog_flag_in_packed_state(self):
        frame = pack_state(flags=FLAG_WATCHDOG)
        self.assertTrue(frame[12] & FLAG_WATCHDOG)


if __name__ == "__main__":
    unittest.main()
