"""Trajectory sampling, goal ownership, and motion-success checks.

No ROS imports. The bridge uses these so a cached or faulted sample cannot be
reported as a finished move, and so a second goal cannot share the motors.
"""

from __future__ import annotations

import threading

from clearcore_bridge.wire import AXIS

FRESH_S = 0.25
GOAL_SETTLE_S = 0.5


class GoalGate:
    """One active trajectory. A new goal is rejected until the owner releases."""

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._owner = None

    def try_reserve(self) -> bool:
        with self._lock:
            if self._owner is not None:
                return False
            self._owner = "pending"
            return True

    def claim(self, goal_id) -> bool:
        with self._lock:
            if self._owner != "pending":
                return False
            self._owner = goal_id
            return True

    def release(self, goal_id) -> None:
        with self._lock:
            if self._owner == goal_id:
                self._owner = None

    def release_pending(self) -> None:
        with self._lock:
            if self._owner == "pending":
                self._owner = None

    def owns(self, goal_id) -> bool:
        with self._lock:
            return self._owner == goal_id

    def accepts_cancel(self, goal_id) -> bool:
        with self._lock:
            return self._owner == goal_id or self._owner == "pending"

    def busy(self) -> bool:
        with self._lock:
            return self._owner is not None


def state_block_reason(state, age_s: float, fresh_s: float = FRESH_S) -> str | None:
    """Why this sample must not count as a successful move. None means usable."""
    if state is None:
        return "no state"
    if age_s is None or age_s > fresh_s:
        return "stale state"
    if state.get("watchdog"):
        return "watchdog tripped; call clear_alerts"
    if state.get("estop"):
        return "estop"
    if state.get("fault"):
        return "fault"
    if not state.get("enabled"):
        return "motor not enabled"
    return None


def motion_succeeded(state, age_s, names, targets, tolerances, default_tol, fresh_s=FRESH_S) -> bool:
    if state_block_reason(state, age_s, fresh_s) is not None:
        return False
    if state.get("moving"):
        return False
    for name, target in zip(names, targets):
        axis = AXIS[name]
        tol = tolerances.get(name, default_tol(name) if callable(default_tol) else default_tol)
        if abs(float(state["position"][axis]) - float(target)) > tol:
            return False
    return True


def path_violation(state, commanded, tolerances) -> str | None:
    """Reported position versus a host schedule sample.

    The two arguments must already share one time base. The stream loop does
    not use this: a just-computed schedule is ahead of the latest state frame.
    """
    if not tolerances:
        return None
    for name, limit in tolerances.items():
        if name not in commanded or name not in AXIS:
            continue
        err = abs(float(state["position"][AXIS[name]]) - float(commanded[name]))
        if err > limit:
            return name
    return None


def _board_dt_s(now_ms, latch_ms) -> float:
    return ((int(now_ms) - int(latch_ms)) & 0xFFFFFFFF) / 1000.0


def local_tracking_violation(state, tolerances) -> str | None:
    """Generated position versus the time-advanced reference in this state frame.

    q_ref = q_latched + v_latched * (time_ms - latch_ms), using only that sample.
    Joints that are not in track_mask are skipped.
    """
    if not tolerances or state is None:
        return None
    mask = int(state.get("track_mask") or 0)
    targets = state.get("target_position")
    speeds = state.get("target_velocity")
    latches = state.get("target_latch_ms")
    if targets is None or speeds is None or latches is None:
        return None
    for name, limit in tolerances.items():
        if name not in AXIS:
            continue
        axis = AXIS[name]
        if (mask & (1 << axis)) == 0 or int(latches[axis]) == 0:
            continue
        q_ref = float(targets[axis]) + float(speeds[axis]) * _board_dt_s(state["time_ms"], latches[axis])
        if abs(float(state["position"][axis]) - q_ref) > float(limit):
            return name
    return None


def _quintic(p0, v0, a0, p1, v1, a1, dt, s):
    """Position and velocity of the quintic with those endpoint boundaries."""
    if dt <= 1e-9:
        return p1, 0.0
    t = s * dt
    t2 = t * t
    t3 = t2 * t
    t4 = t3 * t
    t5 = t4 * t
    dt2 = dt * dt
    dt3 = dt2 * dt
    dt4 = dt3 * dt
    dt5 = dt4 * dt
    c0 = p0
    c1 = v0
    c2 = 0.5 * a0
    c3 = (20.0 * p1 - 20.0 * p0 - (8.0 * v1 + 12.0 * v0) * dt - (3.0 * a0 - a1) * dt2) / (2.0 * dt3)
    c4 = (30.0 * p0 - 30.0 * p1 + (14.0 * v1 + 16.0 * v0) * dt + (3.0 * a0 - 2.0 * a1) * dt2) / (2.0 * dt4)
    c5 = (12.0 * p1 - 12.0 * p0 - 6.0 * (v0 + v1) * dt - (a0 - a1) * dt2) / (2.0 * dt5)
    pos = c0 + c1 * t + c2 * t2 + c3 * t3 + c4 * t4 + c5 * t5
    vel = c1 + 2.0 * c2 * t + 3.0 * c3 * t2 + 4.0 * c4 * t3 + 5.0 * c5 * t4
    return pos, vel


def _hermite(p0, v0, p1, v1, dt, s):
    if dt <= 1e-9:
        return p1, 0.0
    s2 = s * s
    s3 = s2 * s
    h00 = 2.0 * s3 - 3.0 * s2 + 1.0
    h10 = s3 - 2.0 * s2 + s
    h01 = -2.0 * s3 + 3.0 * s2
    h11 = s3 - s2
    pos = h00 * p0 + h10 * dt * v0 + h01 * p1 + h11 * dt * v1
    dh00 = 6.0 * s2 - 6.0 * s
    dh10 = 3.0 * s2 - 4.0 * s + 1.0
    dh01 = -6.0 * s2 + 6.0 * s
    dh11 = 3.0 * s2 - 2.0 * s
    vel = (dh00 * p0 + dh10 * dt * v0 + dh01 * p1 + dh11 * dt * v1) / dt
    return pos, vel


def _segment_slope(knots, index, name, forward):
    if forward and index + 1 < len(knots):
        dt = knots[index + 1]["t"] - knots[index]["t"]
        if dt > 1e-9:
            return (knots[index + 1]["positions"][name] - knots[index]["positions"][name]) / dt
    if index > 0:
        dt = knots[index]["t"] - knots[index - 1]["t"]
        if dt > 1e-9:
            return (knots[index]["positions"][name] - knots[index - 1]["positions"][name]) / dt
    return 0.0


def _assigned_velocity(knots, index, name):
    given = knots[index].get("velocities")
    if given is not None and name in given and given[name] is not None:
        return float(given[name])
    return _segment_slope(knots, index, name, forward=True)


def build_knots(names, points, start_positions):
    """points are dicts: t, positions, optional velocities, optional accelerations."""
    if not names:
        raise ValueError("trajectory has no joints")
    if not points:
        raise ValueError("trajectory is empty")
    knots = [{
        "t": 0.0,
        "positions": {n: float(start_positions[n]) for n in names},
        "velocities": None,
        "accelerations": None,
    }]
    previous_t = 0.0
    for point in points:
        t = float(point["t"])
        if t + 1e-9 < previous_t:
            raise ValueError("time_from_start is not monotonic")
        previous_t = t
        positions = point["positions"]
        if any(n not in positions for n in names):
            raise ValueError("point is missing a joint position")
        knots.append({
            "t": t,
            "positions": {n: float(positions[n]) for n in names},
            "velocities": point.get("velocities"),
            "accelerations": point.get("accelerations"),
        })
    if len(knots) > 1 and knots[1]["t"] <= 1e-9:
        knots.pop(0)
    for index, knot in enumerate(knots):
        knot["vel"] = {n: _assigned_velocity(knots, index, n) for n in names}
    return knots


def is_immediate(knots) -> bool:
    return knots[-1]["t"] - knots[0]["t"] <= 1e-9


def _boundary_accel(knot, name):
    """Signed waypoint acceleration, or None when the point does not supply one."""
    acc = knot.get("accelerations")
    if not acc or name not in acc or acc[name] is None:
        return None
    return float(acc[name])


def _segment_index(knots, elapsed):
    for index in range(len(knots) - 1):
        if elapsed <= knots[index + 1]["t"]:
            return index
    return len(knots) - 2


def sample_trajectory(knots, names, elapsed):
    """Return position dict, velocity dict, and whether the schedule is finished."""
    if elapsed >= knots[-1]["t"]:
        pos = dict(knots[-1]["positions"])
        vel = {n: 0.0 for n in names}
        return pos, vel, True
    if len(knots) == 1:
        return dict(knots[0]["positions"]), {n: 0.0 for n in names}, True
    index = _segment_index(knots, max(elapsed, knots[0]["t"]))
    left = knots[index]
    right = knots[index + 1]
    dt = right["t"] - left["t"]
    s = 0.0 if dt <= 1e-9 else (max(elapsed, left["t"]) - left["t"]) / dt
    s = min(1.0, max(0.0, s))
    pos = {}
    vel = {}
    for name in names:
        a0 = _boundary_accel(left, name)
        a1 = _boundary_accel(right, name)
        if a0 is not None or a1 is not None:
            p, v = _quintic(
                left["positions"][name], left["vel"][name], 0.0 if a0 is None else a0,
                right["positions"][name], right["vel"][name], 0.0 if a1 is None else a1,
                dt, s)
        else:
            p, v = _hermite(
                left["positions"][name], left["vel"][name],
                right["positions"][name], right["vel"][name],
                dt, s)
        pos[name] = p
        vel[name] = v
    return pos, vel, False
