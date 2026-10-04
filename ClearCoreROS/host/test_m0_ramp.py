"""Local streaming diagnostic for M0.

Compare generated position with the received reference advanced to the sample's
board time:

    q_ref = q_latched + v_latched * (time_ms - latch_time)

That is tracking against the stream the firmware has accepted. It is not
host-schedule synchronization, and it is not the MSP shaft position.
Latch time is an integer millisecond. At 30 mm/s, 1 ms is 0.03 mm, so a
0.05 mm residual is about 1.67 ms: larger than one tick, and too close to
the timestamp resolution to tune gain from this measurement alone.
"""

import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "ros2_ws" / "src" / "clearcore_bridge"))

from clearcore_bridge.follow import build_knots, sample_trajectory
from clearcore_bridge.wire import SessionClient, StreamClient

HOST = "172.16.82.113"
STEPS_PER_M = 160000.0  # 800 steps/rev, 5 mm pitch
TOL_M = 0.0005


def wait_settled(session, target, timeout=8.0):
    deadline = time.time() + timeout
    last = None
    while time.time() < deadline:
        last = session.call("get_status")
        if last["fault"] or last["estop"] or last["watchdog"] or not last["enabled"]:
            return last
        if abs(last["position"][0] - target) <= TOL_M and not last["moving"]:
            return last
        time.sleep(0.05)
    return last


def out_and_back(session):
    print("== settled out-and-back (one absolute move per target)", flush=True)
    arrivals = []
    for cycle in range(4):
        for target in (0.06, 0.0):
            session.call("set_joints", {"x": target})
            status = wait_settled(session, target)
            err = status["position"][0] - target
            arrivals.append(abs(err))
            print(
                "  cycle %d target=%+.3f reported=%+.5f err_mm=%+.3f steps=%+.1f fault=%s"
                % (
                    cycle + 1,
                    target,
                    status["position"][0],
                    err * 1000.0,
                    err * STEPS_PER_M,
                    status["fault"],
                ),
                flush=True,
            )
            if abs(err) > TOL_M or status["moving"] or status["fault"]:
                raise SystemExit("arrival missed")
    print(
        "  arrivals=%d max|err|=%.5f m (%.2f steps)"
        % (len(arrivals), max(arrivals), max(arrivals) * STEPS_PER_M),
        flush=True,
    )


def latest_state(stream):
    seen = None
    for frame in stream.read(0.0):
        if frame["type"] == "state":
            seen = frame
    return seen


def schedule_at(knots, t):
    pos, vel, _done = sample_trajectory(knots, ["joint_x"], max(0.0, t), None, 0.02)
    return pos["joint_x"], vel["joint_x"]


def median(values):
    if not values:
        return None
    ordered = sorted(values)
    return ordered[len(ordered) // 2]


def peak(rows, key):
    return max(abs(row[key]) for row in rows) if rows else 0.0


def summarize(label, rows, duration_s):
    def band(lo, hi):
        return [row for row in rows if lo <= row["board_t"] <= hi]

    ends = band(0.0, 0.8) + band(duration_s - 0.8, duration_s)
    middle = band(1.2, duration_s - 1.2)
    fast = [row for row in rows if abs(row["target_vel"]) > 0.01]
    print("  -- %s  local diagnostic: generated vs time-advanced received reference" % label, flush=True)
    v_peak = max(abs(row["target_vel"]) for row in rows)
    tick_mm = v_peak * 1000.0 * 0.001
    ref_peak_mm = peak(rows, "err_ref") * 1000.0
    equiv_ms = ref_peak_mm / tick_mm if tick_mm else float("nan")
    print(
        "  peak |generated - q_ref|=%.3f mm (%.2f ms at peak |v|=%.1f mm/s); 1 ms tick is %.3f mm"
        % (ref_peak_mm, equiv_ms, v_peak * 1000.0, tick_mm),
        flush=True,
    )
    print(
        "  held-target peak=%.3f mm is the frozen-reference comparison, not this diagnostic"
        % (peak(rows, "err_track") * 1000.0,),
        flush=True,
    )
    print(
        "  accel ends peak held=%.3f ref=%.3f    mid peak held=%.3f ref=%.3f    median hold=%.1f ms"
        % (
            peak(ends, "err_track") * 1000.0,
            peak(ends, "err_ref") * 1000.0,
            peak(middle, "err_track") * 1000.0,
            peak(middle, "err_ref") * 1000.0,
            1000.0 * (median([row["hold_s"] for row in rows]) or 0.0),
        ),
        flush=True,
    )
    corr = median([abs(row["command_vel"] - row["target_vel"]) for row in rows])
    print(
        "  median |command_vel - feedforward|=%.3f mm/s"
        % (1000.0 * (corr or 0.0),),
        flush=True,
    )
    tau_track = median([-row["err_track"] / row["target_vel"] for row in fast])
    tau_aligned = median([-row["err_aligned"] / row["sched_vel"] for row in fast if abs(row["sched_vel"]) > 0.01])
    tau_host = median([-row["err_host"] / row["sched_vel_host"] for row in fast if abs(row["sched_vel_host"]) > 0.01])
    age = median([row["age_s"] for row in rows])
    print(
        "  median delay implied by error/velocity: track=%.1f ms aligned=%.1f ms host-clock=%.1f ms  report age=%.1f ms"
        % (
            1000.0 * tau_track if tau_track is not None else float("nan"),
            1000.0 * tau_aligned if tau_aligned is not None else float("nan"),
            1000.0 * tau_host if tau_host is not None else float("nan"),
            1000.0 * age if age is not None else float("nan"),
        ),
        flush=True,
    )


def run_ramp(session, stream, start_m, end_m, duration_s, label):
    names = ["joint_x"]
    points = [
        {
            "t": 0.0,
            "positions": {"joint_x": start_m},
            "velocities": {"joint_x": 0.0},
            "accelerations": None,
        },
        {
            "t": duration_s,
            "positions": {"joint_x": end_m},
            "velocities": {"joint_x": 0.0},
            "accelerations": None,
        },
    ]
    knots = build_knots(names, points, {"joint_x": start_m})
    print(
        "== timed ramp %s  %.3f -> %.3f m in %.1f s (zero end velocity)"
        % (label, start_m, end_m, duration_s),
        flush=True,
    )
    print(
        "  hold_ms  fw_target  q_ref  generated  track_mm  ref_mm  corr_mm_s  tgt_vel  cmd_vel",
        flush=True,
    )
    sync = None
    sync_deadline = time.time() + 1.0
    while sync is None and time.time() < sync_deadline:
        sync = latest_state(stream)
        time.sleep(0.02)
    if sync is None or "target_position" not in sync:
        raise SystemExit("state frame has no firmware target fields")
    t_sync_host = time.monotonic()
    t_sync_board = sync["time_ms"]
    samples = []
    prev = {"joint_x": 0.0}
    t0 = time.monotonic()
    next_print = 0.0
    period = 0.02
    while True:
        host_elapsed = time.monotonic() - t0
        pos, vel, done = sample_trajectory(knots, names, host_elapsed, prev, period)
        if done:
            break
        stream.send_track(
            0x01,
            (pos["joint_x"], 0.0, 0.0, 0.0),
            (vel["joint_x"], 0.0, 0.0, 0.0),
        )
        prev = vel
        state = latest_state(stream)
        if state is None or "target_position" not in state:
            time.sleep(period)
            continue
        board_t = (state["time_ms"] - t_sync_board) / 1000.0 - (t0 - t_sync_host)
        age = host_elapsed - board_t
        sched_b, sched_vel_b = schedule_at(knots, board_t)
        sched_h, sched_vel_h = schedule_at(knots, host_elapsed)
        generated = state["position"][0]
        target = state["target_position"][0]
        target_vel = state["target_velocity"][0]
        hold_s = (state["time_ms"] - state["target_latch_ms"][0]) / 1000.0
        if hold_s < 0.0:
            hold_s += 4294967.296
        q_ref = target + target_vel * hold_s
        row = {
            "board_t": board_t,
            "host_t": host_elapsed,
            "age_s": age,
            "hold_s": hold_s,
            "generated": generated,
            "target": target,
            "q_ref": q_ref,
            "target_vel": target_vel,
            "command_vel": state["command_velocity"][0],
            "sched_vel": sched_vel_b,
            "sched_vel_host": sched_vel_h,
            "err_track": generated - target,
            "err_ref": generated - q_ref,
            "err_aligned": generated - sched_b,
            "err_host": generated - sched_h,
        }
        samples.append(row)
        if host_elapsed + 1e-9 >= next_print:
            print(
                "  %6.1f  %+.5f  %+.5f  %+.5f  %+8.3f  %+8.3f  %+8.3f  %+.4f  %+.4f"
                % (
                    hold_s * 1000.0,
                    target,
                    q_ref,
                    generated,
                    row["err_track"] * 1000.0,
                    row["err_ref"] * 1000.0,
                    (row["command_vel"] - target_vel) * 1000.0,
                    target_vel,
                    row["command_vel"],
                ),
                flush=True,
            )
            next_print += 0.5
        time.sleep(period)
    stream.send_velocity(0x01, (0.0, 0.0, 0.0, 0.0))
    stream.send_position(0x01, (end_m, 0.0, 0.0, 0.0))
    settled = wait_settled(session, end_m)
    if not samples:
        raise SystemExit("ramp produced no samples")
    print("  samples=%d settled_x=%.5f" % (len(samples), settled["position"][0]), flush=True)
    summarize(label, samples, duration_s)
    if settled["fault"] or settled["watchdog"] or not settled["enabled"]:
        raise SystemExit("ramp ended in fault")
    if abs(settled["position"][0] - end_m) > TOL_M:
        raise SystemExit("ramp did not settle")


def main():
    session = SessionClient(HOST, 9200, 3.0)
    stream = None
    try:
        session.call(
            "configure",
            {"axis_mask": 1, "vel_steps": 16000, "accel_steps": 120000, "decel_steps": 120000},
        )
        session.call("set_test_mode", {"on": True})
        session.call("enable")
        stream = StreamClient(HOST, 9201, 3.0)
        deadline = time.time() + 1.0
        while latest_state(stream) is None and time.time() < deadline:
            time.sleep(0.02)
        run_ramp(session, stream, 0.0, 0.08, 4.0, "out")
        run_ramp(session, stream, 0.08, 0.0, 4.0, "back")
        print("reported position is generated steps, not an MSP encoder count", flush=True)
    finally:
        if stream is not None:
            stream.close()
        try:
            session.call("stop")
            session.call("set_joints", {"x": 0.0})
            home = wait_settled(session, 0.0)
            if home:
                print("parked reported x=%.5f m" % home["position"][0], flush=True)
            session.call("disable")
        except Exception as exc:
            print("cleanup:", exc, flush=True)
        session.close()


if __name__ == "__main__":
    main()
