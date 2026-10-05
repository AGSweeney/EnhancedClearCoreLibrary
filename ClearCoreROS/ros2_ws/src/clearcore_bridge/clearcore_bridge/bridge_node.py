"""ROS 2 node: JointState, Trigger services, and FollowJointTrajectory.

The action samples the trajectory on time_from_start. Point velocities are
boundary conditions when present; otherwise the segment slope is used.
When a waypoint also supplies acceleration, that segment is a quintic spline.
Path tolerance uses generated position versus the time-advanced reference in
the same state frame. One goal owns the motors until it finishes. A watchdog
latch is not cleared here; call clear_alerts.
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

from clearcore_bridge.follow import (
    FRESH_S,
    GOAL_SETTLE_S,
    GoalGate,
    build_knots,
    is_immediate,
    local_tracking_violation,
    motion_succeeded,
    sample_trajectory,
    state_block_reason,
)
from clearcore_bridge.session_bringup import bring_up_session
from clearcore_bridge.wire import AXIS, JOINTS, ROTARY, SessionClient, StreamClient


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
        self.declare_parameter("test_mode", False)
        self.declare_parameter("goal_tolerance_m", 0.001)
        self.declare_parameter("goal_tolerance_rad", 0.01)
        self.declare_parameter("goal_settle_s", GOAL_SETTLE_S)
        self.declare_parameter("state_fresh_s", FRESH_S)
        self.declare_parameter("default_speed_m_s", 0.15)
        self.declare_parameter("stream_hz", 50.0)

        self._host = self.get_parameter("host").value
        self._session_port = int(self.get_parameter("session_port").value)
        self._stream_port = int(self.get_parameter("stream_port").value)
        self._mask = int(self.get_parameter("axis_mask").value) & 0x0F
        self._tol_m = float(self.get_parameter("goal_tolerance_m").value)
        self._tol_rad = float(self.get_parameter("goal_tolerance_rad").value)
        self._settle_s = float(self.get_parameter("goal_settle_s").value)
        self._fresh_s = float(self.get_parameter("state_fresh_s").value)
        self._default_speed = float(self.get_parameter("default_speed_m_s").value)
        hz = float(self.get_parameter("stream_hz").value)
        self._period = 1.0 / hz if hz > 1.0 else 0.02

        self._lock = threading.Lock()
        self._session_lock = threading.Lock()
        self._gate = GoalGate()
        self._session: SessionClient | None = None
        self._stream: StreamClient | None = None
        self._state = None
        self._state_mono = None
        self._state_stamp = None
        self._stop_reader = threading.Event()
        self._reader: threading.Thread | None = None
        self._logged_watchdog = False

        group = ReentrantCallbackGroup()
        self._pub = self.create_publisher(JointState, "joint_states", 10)
        self._hlfb_pub = self.create_publisher(JointState, "hlfb_duty", 10)
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

    def _goal(self, _goal_request):
        if not self._gate.try_reserve():
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _cancel(self, goal_handle):
        if self._gate.accepts_cancel(_goal_key(goal_handle)):
            return CancelResponse.ACCEPT
        return CancelResponse.REJECT

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
            bring_up_session(
                session,
                axis_mask=self._mask,
                steps_per_rev=int(self.get_parameter("steps_per_rev").value),
                pitch_mm=float(self.get_parameter("pitch_mm").value),
                vel_steps=int(self.get_parameter("vel_steps").value),
                accel_steps=int(self.get_parameter("accel_steps").value),
                watchdog_ms=int(self.get_parameter("watchdog_ms").value),
                test_mode=bool(self.get_parameter("test_mode").value),
            )
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
                if frame["type"] != "state":
                    continue
                with self._lock:
                    if self._stream is not stream:
                        return
                    self._state = frame
                    self._state_mono = time.monotonic()
                    self._state_stamp = self.get_clock().now().to_msg()
                if frame["watchdog"] and not self._logged_watchdog:
                    self._logged_watchdog = True
                    self.get_logger().error(
                        "watchdog tripped; motion stays rejected until clear_alerts"
                    )
                elif not frame["watchdog"]:
                    self._logged_watchdog = False

    def _publish_state(self) -> None:
        with self._lock:
            state = self._state
            stamp = self._state_stamp
            mono = self._state_mono
            connected = self._stream is not None
        if not connected or state is None or stamp is None or mono is None:
            return
        if time.monotonic() - mono > self._fresh_s:
            return
        msg = JointState()
        msg.header.stamp = stamp
        for axis, name in enumerate(JOINTS):
            if (self._mask & (1 << axis)) == 0:
                continue
            msg.name.append(name)
            msg.position.append(float(state["position"][axis]))
            msg.velocity.append(float(state["velocity"][axis]))
        duty = JointState()
        duty.header.stamp = stamp
        for axis, name in enumerate(JOINTS):
            if (self._mask & (1 << axis)) == 0:
                continue
            duty.name.append(name)
            duty.effort.append(float(state["effort"][axis]))
        self._pub.publish(msg)
        self._hlfb_pub.publish(duty)

    def _execute(self, goal_handle):
        gid = _goal_key(goal_handle)
        result = FollowJointTrajectory.Result()
        if not self._gate.claim(gid):
            self._gate.release_pending()
            return self._abort(goal_handle, result, "another trajectory is active")
        try:
            return self._execute_owned(goal_handle, result)
        finally:
            self._gate.release(gid)

    def _execute_owned(self, goal_handle, result):
        goal = goal_handle.request
        names = list(goal.trajectory.joint_names)
        if not names or not goal.trajectory.points:
            return self._abort(goal_handle, result, "trajectory is empty")
        unknown = [name for name in names if name not in AXIS]
        if unknown:
            result.error_code = FollowJointTrajectory.Result.INVALID_JOINTS
            result.error_string = "unknown joints: " + ",".join(unknown)
            goal_handle.abort()
            return result
        outside = [name for name in names if (self._mask & (1 << AXIS[name])) == 0]
        if outside:
            return self._abort(goal_handle, result, "joint is outside axis_mask: " + ",".join(outside))

        snap = self._live_state()
        if isinstance(snap, str):
            return self._abort(goal_handle, result, snap)
        start = {name: float(snap["position"][AXIS[name]]) for name in names}
        try:
            knots = build_knots(names, _plain_points(names, goal.trajectory.points), start)
        except ValueError as exc:
            return self._abort(goal_handle, result, str(exc))

        goal_tol = _tolerance_map(goal.goal_tolerance)
        path_tol = _tolerance_map(goal.path_tolerance)
        final = [knots[-1]["positions"][name] for name in names]
        timeout = self._settle_s
        if is_immediate(knots):
            distance = max(abs(final[i] - start[name]) for i, name in enumerate(names))
            speed = self._default_speed if self._default_speed > 1e-3 else 0.15
            timeout = distance / speed + self._settle_s
        try:
            self._raise_if_stopped(goal_handle)
            self._require_live()
            if not is_immediate(knots):
                self._stream_schedule(goal_handle, names, knots, path_tol)
                self._send_velocity(names, {name: 0.0 for name in names})
            self._send_position(names, knots[-1]["positions"])
            held = self._wait_for_hold(goal_handle, names, final, goal_tol, timeout)
        except ConnectionError as exc:
            return self._abort(goal_handle, result, str(exc))
        except _Canceled:
            self._halt()
            result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
            result.error_string = "canceled"
            goal_handle.canceled()
            return result
        except _PathError as exc:
            self._halt()
            result.error_code = FollowJointTrajectory.Result.PATH_TOLERANCE_VIOLATED
            result.error_string = f"path tolerance exceeded on {exc.joint}"
            goal_handle.abort()
            return result
        except _Blocked as exc:
            self._halt()
            return self._abort(goal_handle, result, str(exc))

        if not held:
            self._halt()
            if goal_handle.is_cancel_requested:
                result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
                result.error_string = "canceled"
                goal_handle.canceled()
                return result
            result.error_code = FollowJointTrajectory.Result.GOAL_TOLERANCE_VIOLATED
            result.error_string = "goal tolerance missed"
            goal_handle.abort()
            return result
        result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
        result.error_string = ""
        goal_handle.succeed()
        return result

    def _stream_schedule(self, goal_handle, names, knots, path_tol) -> None:
        t0 = time.monotonic()
        while True:
            self._raise_if_stopped(goal_handle)
            elapsed = time.monotonic() - t0
            pos, vel, done = sample_trajectory(knots, names, elapsed)
            if done:
                return
            state = self._require_live()
            # Path tolerance is the board-local diagnostic: generated position
            # versus the time-advanced reference in this state frame. It does
            # not measure error against the host trajectory setpoint. Feedback
            # below pairs that setpoint with the latest received position, and
            # those two numbers are not from the same instant.
            violated = local_tracking_violation(state, path_tol)
            if violated:
                raise _PathError(violated)
            self._send_track(names, pos, vel)
            self._publish_feedback(goal_handle, names, pos, state)
            time.sleep(self._period)

    def _wait_for_hold(self, goal_handle, names, targets, goal_tol, timeout) -> bool:
        deadline = time.monotonic() + timeout
        next_beat = 0.0
        while time.monotonic() < deadline:
            if goal_handle.is_cancel_requested:
                return False
            now = time.monotonic()
            if now >= next_beat:
                self._send_heartbeat()
                next_beat = now + 0.1
            state = self._require_live()
            if motion_succeeded(
                state, 0.0, names, targets, goal_tol, self._default_tol, self._fresh_s
            ):
                return True
            time.sleep(self._period)
        return False

    def _require_live(self):
        snap = self._live_state()
        if isinstance(snap, str):
            raise _Blocked(snap)
        return snap

    def _live_state(self):
        with self._lock:
            if self._stream is None:
                return "not connected"
            state = self._state
            mono = self._state_mono
        age = 1e9 if mono is None else time.monotonic() - mono
        reason = state_block_reason(state, age, self._fresh_s)
        if reason:
            return reason
        return state

    def _raise_if_stopped(self, goal_handle) -> None:
        if goal_handle.is_cancel_requested:
            raise _Canceled()

    def _send_velocity(self, names, values) -> None:
        mask, vec = _mask_vector(names, values)
        stream = self._stream
        if stream is None:
            raise ConnectionError("not connected")
        stream.send_velocity(mask, vec)

    def _send_track(self, names, positions, velocities) -> None:
        mask, pos = _mask_vector(names, positions)
        _mask, vel = _mask_vector(names, velocities)
        stream = self._stream
        if stream is None:
            raise ConnectionError("not connected")
        stream.send_track(mask, pos, vel)

    def _send_position(self, names, values) -> None:
        mask, vec = _mask_vector(names, values)
        stream = self._stream
        if stream is None:
            raise ConnectionError("not connected")
        stream.send_position(mask, vec)

    def _send_heartbeat(self) -> None:
        stream = self._stream
        if stream is None:
            raise ConnectionError("not connected")
        stream.send_heartbeat()

    def _halt(self) -> None:
        try:
            self._call("stop")
        except Exception:
            pass

    def _default_tol(self, name: str) -> float:
        return self._tol_rad if ROTARY[AXIS[name]] else self._tol_m

    def _publish_feedback(self, goal_handle, names, commanded, state) -> None:
        feedback = FollowJointTrajectory.Feedback()
        feedback.joint_names = list(names)
        feedback.desired.positions = [float(commanded[name]) for name in names]
        feedback.actual.positions = [float(state["position"][AXIS[name]]) for name in names]
        feedback.desired.velocities = []
        goal_handle.publish_feedback(feedback)

    def _abort(self, goal_handle, result, text):
        result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
        result.error_string = text
        goal_handle.abort()
        return result

    def _close(self) -> None:
        self._stop_reader.set()
        with self._session_lock:
            stream, session = self._stream, self._session
            self._stream = None
            self._session = None
        with self._lock:
            self._state = None
            self._state_mono = None
            self._state_stamp = None
        if stream is not None:
            stream.close()
        if session is not None:
            session.close()

    def destroy_node(self):
        self._close()
        return super().destroy_node()


class _Canceled(Exception):
    pass


class _Blocked(Exception):
    pass


class _PathError(Exception):
    def __init__(self, joint: str):
        super().__init__(joint)
        self.joint = joint


def _goal_key(goal_handle):
    return tuple(goal_handle.goal_id.uuid)


def _seconds(duration) -> float:
    return float(duration.sec) + float(duration.nanosec) * 1e-9


def _optional_map(names, values):
    if not values:
        return None
    if len(values) != len(names):
        raise ValueError("trajectory field count does not match joint_names")
    return {name: float(values[index]) for index, name in enumerate(names)}


def _plain_points(names, points):
    plain = []
    for point in points:
        if len(point.positions) != len(names):
            raise ValueError("position count does not match joint_names")
        plain.append({
            "t": _seconds(point.time_from_start),
            "positions": {name: float(point.positions[index]) for index, name in enumerate(names)},
            "velocities": _optional_map(names, point.velocities),
            "accelerations": _optional_map(names, point.accelerations),
        })
    return plain


def _tolerance_map(items) -> dict:
    out = {}
    for item in items:
        out[item.name] = abs(float(item.position))
    return out


def _mask_vector(names, values):
    mask = 0
    vec = [0.0, 0.0, 0.0, 0.0]
    for name in names:
        axis = AXIS[name]
        mask |= 1 << axis
        vec[axis] = float(values[name])
    return mask, vec


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
        # destroy_node / SIGINT can already shut the context down.
        if hasattr(rclpy, "try_shutdown"):
            rclpy.try_shutdown()
        elif rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
