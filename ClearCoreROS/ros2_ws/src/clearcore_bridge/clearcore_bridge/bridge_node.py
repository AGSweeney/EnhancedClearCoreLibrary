"""ROS 2 node: JointState, Trigger services, and FollowJointTrajectory.

The action sends each trajectory point as one absolute joint target. The
firmware runs its own trapezoid (vel_steps / accel_steps). time_from_start is
the deadline for that point, not a spline the motor interpolates.
"""

from __future__ import annotations

import threading
import time

import rclpy
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger

from clearcore_bridge.wire import AXIS, JOINTS, SessionClient, StreamClient


class ClearCoreBridge(Node):
    def __init__(self) -> None:
        super().__init__("clearcore_bridge")
        self.declare_parameter("host", "192.168.0.109")
        self.declare_parameter("session_port", 9200)
        self.declare_parameter("stream_port", 9201)
        self.declare_parameter("axis_mask", 3)
        self.declare_parameter("steps_per_rev", 800)
        self.declare_parameter("pitch_mm", 5.0)
        self.declare_parameter("vel_steps", 27000)
        self.declare_parameter("accel_steps", 250000)
        self.declare_parameter("watchdog_ms", 500)
        self.declare_parameter("goal_tolerance_m", 0.001)
        self.declare_parameter("goal_tolerance_rad", 0.01)

        self._host = self.get_parameter("host").value
        self._session_port = int(self.get_parameter("session_port").value)
        self._stream_port = int(self.get_parameter("stream_port").value)
        self._mask = int(self.get_parameter("axis_mask").value) & 0x0F
        self._tol_m = float(self.get_parameter("goal_tolerance_m").value)
        self._tol_rad = float(self.get_parameter("goal_tolerance_rad").value)

        self._lock = threading.Lock()
        self._session_lock = threading.Lock()
        self._session: SessionClient | None = None
        self._stream: StreamClient | None = None
        self._state = None
        self._stop_reader = threading.Event()
        self._reader: threading.Thread | None = None

        group = ReentrantCallbackGroup()
        self._pub = self.create_publisher(JointState, "joint_states", 10)
        self.create_timer(0.05, self._publish_state, callback_group=group)
        self.create_timer(2.0, self._ensure_connected, callback_group=group)
        for name, method in (
            ("enable", "enable"),
            ("disable", "disable"),
            ("stop", "stop"),
            ("estop", "estop"),
            ("clear_alerts", "clear_alerts"),
        ):
            self.create_service(
                Trigger, name, lambda req, resp, m=method: self._trigger(m, resp), callback_group=group
            )
        self._action = ActionServer(
            self,
            FollowJointTrajectory,
            "follow_joint_trajectory",
            execute_callback=self._execute,
            goal_callback=self._goal,
            cancel_callback=self._cancel,
            callback_group=group,
        )
        self.get_logger().info(
            f"ClearCoreROS bridge -> {self._host}:{self._session_port} mask={self._mask}"
        )

    def _goal(self, _goal):
        return GoalResponse.ACCEPT

    def _cancel(self, _goal):
        return CancelResponse.ACCEPT

    def _trigger(self, method: str, response):
        try:
            self._call(method)
            response.success = True
            response.message = method
        except Exception as exc:  # noqa: BLE001 — report the firmware/socket error to the caller
            response.success = False
            response.message = str(exc)
        return response

    def _call(self, method: str, params: dict | None = None):
        with self._session_lock:
            if self._session is None:
                raise RuntimeError("not connected")
            return self._session.call(method, params)

    def _ensure_connected(self) -> None:
        if self._session is not None and self._stream is not None:
            return
        self._close()
        session = None
        stream = None
        try:
            session = SessionClient(self._host, self._session_port, 2.0)
            session.call("disable")
            session.call(
                "configure",
                {
                    "axis_mask": self._mask,
                    "steps_per_rev": int(self.get_parameter("steps_per_rev").value),
                    "pitch_mm": float(self.get_parameter("pitch_mm").value),
                    "vel_steps": int(self.get_parameter("vel_steps").value),
                    "accel_steps": int(self.get_parameter("accel_steps").value),
                    "decel_steps": int(self.get_parameter("accel_steps").value),
                    "watchdog_ms": int(self.get_parameter("watchdog_ms").value),
                },
            )
            session.call("enable")
            stream = StreamClient(self._host, self._stream_port, 2.0)
        except Exception as exc:  # noqa: BLE001
            if stream is not None:
                stream.close()
            if session is not None:
                session.close()
            self.get_logger().warning(f"connect failed: {exc}")
            return
        self._session = session
        self._stream = stream
        self._stop_reader.clear()
        self._reader = threading.Thread(target=self._read_loop, daemon=True)
        self._reader.start()
        self.get_logger().info("connected and enabled")

    def _read_loop(self) -> None:
        while not self._stop_reader.is_set():
            stream = self._stream
            if stream is None:
                break
            try:
                frames = stream.read(0.2)
            except Exception as exc:  # noqa: BLE001
                self.get_logger().warning(f"stream closed: {exc}")
                self._close()
                break
            for frame in frames:
                if frame["type"] == "state":
                    with self._lock:
                        self._state = frame
                    if frame["watchdog"]:
                        try:
                            self._call("keepalive")
                        except Exception:
                            pass

    def _publish_state(self) -> None:
        with self._lock:
            state = self._state
        if state is None:
            return
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        for axis, name in enumerate(JOINTS):
            if (self._mask & (1 << axis)) == 0:
                continue
            msg.name.append(name)
            msg.position.append(float(state["position"][axis]))
            msg.velocity.append(float(state["velocity"][axis]))
            msg.effort.append(float(state["effort"][axis]))
        self._pub.publish(msg)

    def _execute(self, goal_handle):
        result = FollowJointTrajectory.Result()
        goal = goal_handle.request
        names = list(goal.trajectory.joint_names)
        if not names or not goal.trajectory.points:
            result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
            result.error_string = "trajectory is empty"
            goal_handle.abort()
            return result
        unknown = [name for name in names if name not in AXIS]
        if unknown:
            result.error_code = FollowJointTrajectory.Result.INVALID_JOINTS
            result.error_string = "unknown joints: " + ",".join(unknown)
            goal_handle.abort()
            return result

        start = time.time()
        for index, point in enumerate(goal.trajectory.points):
            if goal_handle.is_cancel_requested:
                try:
                    self._call("stop")
                except Exception:
                    pass
                result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
                result.error_string = "canceled"
                goal_handle.canceled()
                return result
            if len(point.positions) != len(names):
                result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
                result.error_string = f"point {index} position count"
                goal_handle.abort()
                return result
            mask = 0
            q = [0.0, 0.0, 0.0, 0.0]
            for name, value in zip(names, point.positions):
                axis = AXIS[name]
                mask |= 1 << axis
                q[axis] = float(value)
            try:
                stream = self._stream
                if stream is None:
                    raise RuntimeError("not connected")
                stream.send_position(mask, q)
            except Exception as exc:  # noqa: BLE001
                result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
                result.error_string = str(exc)
                goal_handle.abort()
                return result

            deadline = start + _seconds(point.time_from_start) + 2.0
            if _seconds(point.time_from_start) <= 0.0:
                deadline = time.time() + 30.0
            if not self._wait_point(goal_handle, names, point, deadline):
                try:
                    self._call("stop")
                except Exception:
                    pass
                if goal_handle.is_cancel_requested:
                    result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
                    result.error_string = "canceled"
                    goal_handle.canceled()
                    return result
                result.error_code = FollowJointTrajectory.Result.GOAL_TOLERANCE_VIOLATED
                result.error_string = f"point {index} missed tolerance"
                goal_handle.abort()
                return result

        result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
        result.error_string = ""
        goal_handle.succeed()
        return result

    def _wait_point(self, goal_handle, names, point, deadline) -> bool:
        tols = {}
        for tol in goal_handle.request.goal_tolerance:
            tols[tol.name] = abs(float(tol.position))
        next_beat = 0.0
        while time.time() < deadline:
            if goal_handle.is_cancel_requested:
                return False
            now = time.time()
            if now >= next_beat:
                try:
                    if self._stream is not None:
                        self._stream.send_heartbeat()
                except Exception:
                    return False
                next_beat = now + 0.1
            with self._lock:
                state = self._state
            if state is None:
                time.sleep(0.02)
                continue
            ok = True
            actual = []
            for name, value in zip(names, point.positions):
                axis = AXIS[name]
                default = self._tol_rad if axis == 3 else self._tol_m
                if abs(float(state["position"][axis]) - float(value)) > tols.get(name, default):
                    ok = False
                actual.append(float(state["position"][axis]))
            if ok and not state["moving"]:
                return True
            feedback = FollowJointTrajectory.Feedback()
            feedback.joint_names = list(names)
            feedback.actual.positions = actual
            feedback.desired.positions = list(point.positions)
            goal_handle.publish_feedback(feedback)
            time.sleep(0.05)
        return False

    def _close(self) -> None:
        self._stop_reader.set()
        with self._session_lock:
            stream, session = self._stream, self._session
            self._stream = None
            self._session = None
        if stream is not None:
            stream.close()
        if session is not None:
            session.close()

    def destroy_node(self):
        self._close()
        return super().destroy_node()


def _seconds(duration) -> float:
    return float(duration.sec) + float(duration.nanosec) * 1e-9


def main() -> None:
    rclpy.init()
    node = ClearCoreBridge()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
