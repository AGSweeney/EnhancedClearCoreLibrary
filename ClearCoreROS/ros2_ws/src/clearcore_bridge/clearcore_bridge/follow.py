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
    """Joint name whose tracking error exceeds its path tolerance, if any were set."""
    if not tolerances:
        return None
    for name, limit in tolerances.items():
        if name not in commanded or name not in AXIS:
            continue
        err = abs(float(state["position"][AXIS[name]]) - float(commanded[name]))
        if err > limit:
            return name
    return None


def limit_step(previous: float, desired: float, accel: float | None, dt: float) -> float:
    if accel is None or accel <= 0.0 or dt <= 0.0:
        return desired
    delta = desired - previous
    cap = accel * dt
    if delta > cap:
        return previous + cap
    if delta < -cap:
        return previous - cap
    return desired


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


def _accel_cap(knot, name):
    acc = knot.get("accelerations")
    if not acc or name not in acc or acc[name] is None:
        return None
    value = abs(float(acc[name]))
    if value <= 1e-12:
        return None
    return value


def _segment_index(knots, elapsed):
    for index in range(len(knots) - 1):
        if elapsed <= knots[index + 1]["t"]:
            return index
    return len(knots) - 2


def sample_trajectory(knots, names, elapsed, prev_velocity, sample_dt):
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
        p, v = _hermite(
            left["positions"][name], left["vel"][name],
            right["positions"][name], right["vel"][name],
            dt, s)
        pos[name] = p
        cap = _accel_cap(right, name) or _accel_cap(left, name)
        previous = 0.0 if prev_velocity is None else float(prev_velocity.get(name, 0.0))
        vel[name] = limit_step(previous, v, cap, sample_dt)
    return pos, vel, False
