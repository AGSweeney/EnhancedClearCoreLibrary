"""Drive ClearCoreBridge against a simulated session/stream board."""

from __future__ import annotations

import time
import unittest
from types import SimpleNamespace

import rclpy
from control_msgs.action import FollowJointTrajectory
from std_srvs.srv import Trigger

from clearcore_bridge.bridge_node import ClearCoreBridge
from clearcore_bridge.sim_board import JsonRpcBoard, StreamBoard


def _names():
    return {
        "get_config": {
            "names": ["joint_x", "joint_y", "joint_z", "joint_a"],
            "rotary": [False, False, False, True],
        }
    }


class _Handle:
    def __init__(self):
        self.goal_id = SimpleNamespace(uuid=bytes(range(16)))
        point = SimpleNamespace(
            positions=[0.01],
            velocities=[],
            accelerations=[],
            time_from_start=SimpleNamespace(sec=0, nanosec=0),
        )
        self.request = SimpleNamespace(
            trajectory=SimpleNamespace(joint_names=["joint_x"], points=[point]),
            goal_tolerance=[],
            path_tolerance=[],
        )
        self.is_cancel_requested = True
        self.outcome = None

    def canceled(self):
        self.outcome = "canceled"

    def abort(self):
        self.outcome = "aborted"

    def succeed(self):
        self.outcome = "succeeded"

    def publish_feedback(self, _feedback):
        pass


def _wait_state(node, timeout=2.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        with node._lock:
            if node._state is not None:
                return
        time.sleep(0.02)
    raise AssertionError("bridge never received a stream state frame")


class TestBridgeSim(unittest.TestCase):
    def test_bridge_reconnect_home_recovery_and_cancel(self):
        session = JsonRpcBoard(_names())
        stream = StreamBoard()
        node = None
        try:
            rclpy.init(
                args=[
                    "--ros-args",
                    "-p",
                    "host:=127.0.0.1",
                    "-p",
                    f"session_port:={session.port}",
                    "-p",
                    f"stream_port:={stream.port}",
                    "-p",
                    "axis_mask:=3",
                ]
            )
            node = ClearCoreBridge()
            node._ensure_connected()
            self.assertIsNotNone(node._session)
            _wait_state(node)
            bring_up = [
                "disable",
                "clear_alerts",
                "configure",
                "set_test_mode",
                "get_config",
                "enable",
            ]
            self.assertEqual(session.methods, bring_up)

            node._close()
            node._ensure_connected()
            _wait_state(node)
            self.assertEqual(session.methods, bring_up + bring_up)

            node._call("home", {"axis": "x", "dir": "neg"})
            self.assertEqual(session.methods[-1], "home")

            node._call("keepalive")
            resp = Trigger.Response()
            node._trigger("clear_alerts", resp)
            self.assertTrue(resp.success)
            self.assertEqual(session.methods[-2], "keepalive")
            self.assertEqual(session.methods[-1], "clear_alerts")

            handle = _Handle()
            self.assertTrue(node._gate.try_reserve())
            result = node._execute(handle)
            self.assertEqual(handle.outcome, "canceled")
            self.assertEqual(result.error_string, "canceled")
            self.assertEqual(result.error_code, FollowJointTrajectory.Result.SUCCESSFUL)
            self.assertEqual(session.methods[-1], "stop")
        finally:
            if node is not None:
                node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
            stream.close()
            session.close()


if __name__ == "__main__":
    unittest.main()
