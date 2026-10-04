#!/usr/bin/env python3
"""Send the bench XY goal to joint_trajectory_controller and print the result.

The trajectory is joint_x and joint_y: (0.03, 0.03) m at 2 s, then (0, 0) at 4 s.
Raw joint positions are metres. error_code 0 is SUCCESSFUL.
"""

from __future__ import annotations

import sys

import rclpy
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint


class GoalSender(Node):
    def __init__(self) -> None:
        super().__init__("send_trajectory_goal")
        self.declare_parameter(
            "action", "/joint_trajectory_controller/follow_joint_trajectory"
        )
        action = self.get_parameter("action").get_parameter_value().string_value
        self._client = ActionClient(self, FollowJointTrajectory, action)
        self._samples: list[tuple[float, float]] = []
        self.create_subscription(JointState, "/joint_states", self._on_state, 10)

    def _on_state(self, msg) -> None:
        names = list(msg.name)
        if "joint_x" not in names or "joint_y" not in names:
            return
        x = float(msg.position[names.index("joint_x")])
        y = float(msg.position[names.index("joint_y")])
        self._samples.append((x, y))

    def send(self) -> int:
        if not self._client.wait_for_server(timeout_sec=15.0):
            self.get_logger().error("trajectory action is not available")
            return 1
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = ["joint_x", "joint_y"]
        first = JointTrajectoryPoint()
        first.positions = [0.03, 0.03]
        first.velocities = [0.0, 0.0]
        first.time_from_start.sec = 2
        second = JointTrajectoryPoint()
        second.positions = [0.0, 0.0]
        second.velocities = [0.0, 0.0]
        second.time_from_start.sec = 4
        goal.trajectory.points = [first, second]
        send = self._client.send_goal_async(goal, feedback_callback=self._feedback)
        rclpy.spin_until_future_complete(self, send)
        handle = send.result()
        if handle is None or not handle.accepted:
            self.get_logger().error("goal rejected")
            return 1
        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result().result
        code = int(result.error_code)
        self.get_logger().info(
            "result error_code=%d (%s) %s"
            % (code, _code_name(code), result.error_string)
        )
        if self._samples:
            xs = [sample[0] for sample in self._samples]
            ys = [sample[1] for sample in self._samples]
            peak = max(abs(x - y) for x, y in self._samples)
            end_x, end_y = self._samples[-1]
            self.get_logger().info(
                "samples=%d x_mm=[%.2f, %.2f] y_mm=[%.2f, %.2f] peak_|Y-X|_mm=%.3f end_mm=(%.3f, %.3f)"
                % (
                    len(self._samples),
                    min(xs) * 1000.0,
                    max(xs) * 1000.0,
                    min(ys) * 1000.0,
                    max(ys) * 1000.0,
                    peak * 1000.0,
                    end_x * 1000.0,
                    end_y * 1000.0,
                )
            )
        return 0 if code == FollowJointTrajectory.Result.SUCCESSFUL else 1

    def _feedback(self, feedback) -> None:
        point = feedback.feedback.actual
        if len(point.positions) < 2:
            return
        desired = feedback.feedback.desired.positions
        self.get_logger().info(
            "actual_mm=(%.2f, %.2f) desired_mm=(%.2f, %.2f)"
            % (
                point.positions[0] * 1000.0,
                point.positions[1] * 1000.0,
                desired[0] * 1000.0 if desired else float("nan"),
                desired[1] * 1000.0 if len(desired) > 1 else float("nan"),
            )
        )


def _code_name(code: int) -> str:
    names = {
        FollowJointTrajectory.Result.SUCCESSFUL: "SUCCESSFUL",
        FollowJointTrajectory.Result.INVALID_GOAL: "INVALID_GOAL",
        FollowJointTrajectory.Result.INVALID_JOINTS: "INVALID_JOINTS",
        FollowJointTrajectory.Result.OLD_HEADER_TIMESTAMP: "OLD_HEADER_TIMESTAMP",
        FollowJointTrajectory.Result.PATH_TOLERANCE_VIOLATED: "PATH_TOLERANCE_VIOLATED",
        FollowJointTrajectory.Result.GOAL_TOLERANCE_VIOLATED: "GOAL_TOLERANCE_VIOLATED",
    }
    return names.get(code, "UNKNOWN")


def main() -> None:
    rclpy.init()
    node = GoalSender()
    try:
        code = node.send()
    finally:
        node.destroy_node()
        rclpy.shutdown()
    sys.exit(code)


if __name__ == "__main__":
    main()
