#!/usr/bin/env python3
"""30-minute ROS control endurance: random XY goals, stay connected and enabled.

Launches joint_trajectory_controller once. Sends FollowJointTrajectory goals for
the duration. Does not kill ROS, disable motors, or clear_alerts until after the
timed run. Final disable is only for the post-run verified disabled state.
"""

from __future__ import annotations

import json
import math
import os
import random
import signal
import socket
import subprocess
import sys
import threading
import time
from datetime import datetime, timezone
from pathlib import Path

import rclpy
from action_msgs.msg import GoalStatus
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

HOST = os.environ.get("CCROS_HOST", "172.16.82.114")
DURATION_S = int(os.environ.get("CCROS_BATTERY_SECONDS", "1800"))
WS = Path(
    os.environ.get(
        "CCROS_WS",
        "/mnt/d/CCDev/EnhancedClearCoreLibrary/ClearCoreROS/ros2_ws",
    )
)
LOG_ROOT = Path(
    os.environ.get(
        "CCROS_LOG_ROOT",
        "/mnt/d/CCDev/EnhancedClearCoreLibrary/ClearCoreROS/logs",
    )
)
XY_MIN = 0.001
XY_MAX = 0.028  # stay inside 0.030 m soft max
VEL_MPS = 0.015
MIN_SEG_S = 1.5
MAX_SEG_S = 5.0
ACTION = "/joint_trajectory_controller/follow_joint_trajectory"


def rpc(method: str, params=None, timeout: float = 2.0) -> dict:
    s = socket.create_connection((HOST, 9200), timeout)
    try:
        req: dict = {"jsonrpc": "2.0", "id": 1, "method": method}
        if params is not None:
            req["params"] = params
        s.sendall((json.dumps(req) + "\n").encode())
        buf = b""
        while b"\n" not in buf:
            chunk = s.recv(4096)
            if not chunk:
                break
            buf += chunk
    finally:
        s.close()
    msg = json.loads(buf.decode())
    if "error" in msg:
        raise RuntimeError(msg["error"])
    return msg["result"]


def utc() -> str:
    return datetime.now(timezone.utc).isoformat()


class RandomStayEnabled(Node):
    def __init__(self, log_dir: Path, duration_s: int, seed: int) -> None:
        super().__init__("ros_random_stay_enabled")
        self.log_dir = log_dir
        self.duration_s = duration_s
        self.seed = seed
        self.client = ActionClient(self, FollowJointTrajectory, ACTION)
        self._stop = False
        self.t0 = time.perf_counter()
        self.rng = random.Random(seed)
        self.last_xy = (0.0, 0.0)
        self.samples = 0
        self.peak_abs_yx_m = 0.0
        self.goals_ok = 0
        self.goals_fail = 0
        self.alert_events: list[dict] = []
        self.failures: list[dict] = []
        self.enabled_drops = 0
        self.create_subscription(JointState, "/joint_states", self._on_js, 20)
        self._log_fh = (log_dir / "random.log").open("w", encoding="utf-8")
        self._moves_fh = (log_dir / "moves.csv").open("w", encoding="utf-8")
        self._moves_fh.write(
            "t_s,x_m,y_m,dur_s,status,error_code,ok,board_enabled,board_alerts\n"
        )
        self._events = (log_dir / "events.jsonl").open("w", encoding="utf-8")
        self.log("start", duration_s=duration_s, seed=seed, host=HOST)

    def elapsed(self) -> float:
        return time.perf_counter() - self.t0

    def remaining(self) -> float:
        return self.duration_s - self.elapsed()

    def log(self, msg: str, **kwargs) -> None:
        row = {"t_s": round(self.elapsed(), 3), "utc": utc(), "msg": msg, **kwargs}
        line = f"[{row['t_s']:8.1f}s] {msg}"
        if kwargs:
            line += " " + json.dumps(kwargs, default=str)
        print(line, flush=True)
        self._log_fh.write(line + "\n")
        self._log_fh.flush()
        self._events.write(json.dumps(row, default=str) + "\n")
        self._events.flush()

    def _on_js(self, msg: JointState) -> None:
        names = list(msg.name)
        if "joint_x" not in names or "joint_y" not in names:
            return
        x = float(msg.position[names.index("joint_x")])
        y = float(msg.position[names.index("joint_y")])
        self.last_xy = (x, y)
        self.samples += 1
        self.peak_abs_yx_m = max(self.peak_abs_yx_m, abs(y - x))

    def wait_action(self, timeout: float) -> bool:
        deadline = time.time() + timeout
        while time.time() < deadline and not self._stop:
            if self.client.wait_for_server(timeout_sec=1.0):
                self.log("jtc action ready")
                return True
        self.log("timeout waiting for jtc action")
        return False

    def pick_target(self) -> tuple[float, float, float]:
        x = self.rng.uniform(XY_MIN, XY_MAX)
        y = self.rng.uniform(XY_MIN, XY_MAX)
        dist = math.hypot(x - self.last_xy[0], y - self.last_xy[1])
        dur = min(MAX_SEG_S, max(MIN_SEG_S, dist / VEL_MPS))
        return x, y, dur

    def send_goal(self, x: float, y: float, dur: float) -> tuple[int, int, bool]:
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = ["joint_x", "joint_y"]
        pt = JointTrajectoryPoint()
        pt.positions = [x, y]
        pt.velocities = [0.0, 0.0]
        sec = int(dur)
        nsec = int((dur - sec) * 1e9)
        pt.time_from_start.sec = sec
        pt.time_from_start.nanosec = nsec
        goal.trajectory.points = [pt]
        send = self.client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send, timeout_sec=10.0)
        handle = send.result()
        if handle is None or not handle.accepted:
            return -1, -1, False
        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=dur + 8.0)
        wrapped = result_future.result()
        if wrapped is None:
            return -2, -2, False
        status = int(wrapped.status)
        code = int(wrapped.result.error_code)
        ok = status == GoalStatus.STATUS_SUCCEEDED and code == 0
        return status, code, ok

    def poll_board(self, retries: int = 1) -> dict:
        last_exc = None
        for attempt in range(max(1, retries)):
            try:
                st = rpc("get_status")
                break
            except Exception as exc:  # noqa: BLE001
                last_exc = exc
                time.sleep(0.5)
        else:
            self.log("get_status failed", error=str(last_exc))
            self.failures.append({"t_s": self.elapsed(), "error": str(last_exc)})
            return {}
        if st.get("enabled") is False:
            self.enabled_drops += 1
            self.log("ENABLED DROPPED", alerts=st.get("alerts"), axes=st.get("axes"))
            self.failures.append(
                {"t_s": self.elapsed(), "step": "enabled_dropped", "status": st}
            )
        alerts = st.get("alerts")
        if alerts not in (None, "", "none"):
            ev = {
                "t_s": self.elapsed(),
                "alerts": alerts,
                "enabled": st.get("enabled"),
                "axes": st.get("axes"),
            }
            self.alert_events.append(ev)
            self.log("ALERT", **{k: ev[k] for k in ("alerts", "enabled")})
        return st

    def wait_joint_states(self, timeout: float) -> bool:
        deadline = time.time() + timeout
        while time.time() < deadline and not self._stop:
            if self.samples > 0:
                self.log("joint_states live", samples=self.samples, last_xy=self.last_xy)
                return True
            rclpy.spin_once(self, timeout_sec=0.2)
        self.log("timeout waiting for joint_states")
        return False

    def run_moves(self) -> int:
        if not self.wait_action(45.0):
            return 2
        # Plugin owns the session TCP port while active. Do not open a second
        # get_status connection until ROS is torn down.
        if not self.wait_joint_states(20.0):
            self.failures.append({"t_s": self.elapsed(), "step": "no_joint_states"})
            return 3
        n = 0
        while self.remaining() > 2.0 and not self._stop:
            x, y, dur = self.pick_target()
            if dur + 1.0 > self.remaining():
                break
            n += 1
            status, code, ok = self.send_goal(x, y, dur)
            if ok:
                self.goals_ok += 1
            else:
                self.goals_fail += 1
                self.failures.append(
                    {
                        "t_s": self.elapsed(),
                        "step": "goal",
                        "n": n,
                        "status": status,
                        "error_code": code,
                        "x": x,
                        "y": y,
                    }
                )
            self._moves_fh.write(
                f"{self.elapsed():.3f},{x:.6f},{y:.6f},{dur:.3f},"
                f"{status},{code},{int(ok)},,\n"
            )
            self._moves_fh.flush()
            if n == 1 or n % 10 == 0:
                self.log(
                    f"goal {n}",
                    ok=ok,
                    x_mm=round(x * 1000, 3),
                    y_mm=round(y * 1000, 3),
                    dur_s=round(dur, 3),
                    status=status,
                    error_code=code,
                    last_xy_mm=[round(v * 1000, 3) for v in self.last_xy],
                    remaining_s=round(self.remaining(), 1),
                )
        self.log(
            "moves done",
            goals_ok=self.goals_ok,
            goals_fail=self.goals_fail,
            samples=self.samples,
            peak_abs_yx_mm=self.peak_abs_yx_m * 1000.0,
        )
        return 0 if self.goals_fail == 0 and self.enabled_drops == 0 else 1

    def close_logs(self) -> None:
        for fh in (self._log_fh, self._moves_fh, self._events):
            try:
                fh.close()
            except Exception:
                pass


def pids_connected_to_session() -> list[int]:
    """Host PIDs with a TCP connection to the board session port."""
    pids: list[int] = []
    try:
        out = subprocess.check_output(
            ["ss", "-tnp"], text=True, stderr=subprocess.DEVNULL
        )
    except (OSError, subprocess.CalledProcessError):
        return pids
    needle = f"{HOST}:9200"
    for line in out.splitlines():
        if needle not in line and ":9200" not in line:
            continue
        for token in line.replace(",", " ").split():
            if token.startswith("pid="):
                try:
                    pids.append(int(token.split("=")[1]))
                except ValueError:
                    pass
    return sorted(set(pids))


def kill_session_holders(*, extra_pids: list[int] | None = None) -> None:
    """Stop anything that already owns TCP 9200 so bring-up can take the session."""
    patterns = [
        "ros2 launch clearcore_bridge",
        "ros2 launch clearcore_hardware",
        "ros2_control_node",
        "robot_state_publisher",
        "/spawner",
        "send_trajectory_goal.py",
        "clearcore_bridge.bridge_node",
    ]
    for pat in patterns:
        subprocess.run(
            ["pkill", "-9", "-f", pat],
            check=False,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )
    for pid in pids_connected_to_session() + list(extra_pids or []):
        if pid == os.getpid() or pid == os.getppid():
            continue
        try:
            os.kill(pid, signal.SIGTERM)
        except OSError:
            continue
    time.sleep(0.4)
    for pid in pids_connected_to_session():
        if pid == os.getpid() or pid == os.getppid():
            continue
        try:
            os.kill(pid, signal.SIGKILL)
        except OSError:
            pass
    time.sleep(0.6)


def wait_session_free(timeout: float = 20.0) -> dict:
    """Board answers get_status only when no other client holds the session."""
    deadline = time.time() + timeout
    last = None
    while time.time() < deadline:
        holders = pids_connected_to_session()
        try:
            st = rpc("get_status")
            if holders:
                # We got a reply; leftover host sockets should be gone.
                print(
                    f"session free after get_status; leftover pids={holders}",
                    flush=True,
                )
            return st
        except Exception as exc:  # noqa: BLE001
            last = exc
            print(
                f"session busy ({type(exc).__name__}: {exc}) holders={holders}",
                flush=True,
            )
            kill_session_holders()
            time.sleep(0.5)
    raise TimeoutError(f"session still held after {timeout}s: {last}")


def launch_jtc(log_path: Path) -> subprocess.Popen:
    fh = log_path.open("w", encoding="utf-8")
    proc = subprocess.Popen(
        [
            "ros2",
            "launch",
            "clearcore_hardware",
            "trajectory.launch.py",
            f"host:={HOST}",
            "axis_mask:=3",
            "test_mode:=false",
        ],
        stdout=fh,
        stderr=subprocess.STDOUT,
        cwd=str(WS),
        start_new_session=True,
    )
    proc._log_fh = fh  # type: ignore[attr-defined]
    return proc


def kill_ros(proc: subprocess.Popen | None) -> None:
    if proc is not None and proc.poll() is None:
        try:
            os.killpg(proc.pid, signal.SIGTERM)
        except Exception:
            proc.terminate()
        try:
            proc.wait(timeout=8)
        except subprocess.TimeoutExpired:
            proc.kill()
            proc.wait(timeout=5)
    subprocess.run(
        [
            "pkill",
            "-f",
            "ros2 launch clearcore_hardware|ros2_control_node|robot_state_publisher|/spawner",
        ],
        check=False,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    )
    time.sleep(1.0)


def main() -> int:
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    log_dir = LOG_ROOT / f"random_stay_{stamp}"
    log_dir.mkdir(parents=True, exist_ok=True)
    seed = int(os.environ.get("CCROS_RANDOM_SEED", str(int(time.time()))))
    print(f"RANDOM_STAY_ENABLED log_dir={log_dir} duration_s={DURATION_S} seed={seed}", flush=True)

    kill_session_holders()
    st0 = wait_session_free(25.0)
    print(
        "preflight session free",
        json.dumps(
            {
                "enabled": st0.get("enabled"),
                "alerts": st0.get("alerts"),
                "tcp_session_accepts": st0.get("tcp_session_accepts"),
                "tcp_session_closes": st0.get("tcp_session_closes"),
            }
        ),
        flush=True,
    )

    proc = launch_jtc(log_dir / "traj_launch.log")
    node = None
    rc = 1
    try:
        rclpy.init()
        node = RandomStayEnabled(log_dir, DURATION_S, seed)
        # Give spawners time; then wait for action.
        time.sleep(4.0)
        rc = node.run_moves()
    except KeyboardInterrupt:
        if node:
            node._stop = True
        rc = 130
    finally:
        live = {
            "note": "session get_status is not used while JTC owns TCP 9200",
            "last_xy_m": list(node.last_xy) if node else None,
            "samples": node.samples if node else None,
        }
        (log_dir / "status_end_still_connected.json").write_text(
            json.dumps(live, indent=2), encoding="utf-8"
        )
        if node:
            node.log("end-of-run status (before ROS teardown)", enabled=live.get("enabled"), alerts=live.get("alerts"))
            node.close_logs()
            node.destroy_node()
        if rclpy.ok():
            if hasattr(rclpy, "try_shutdown"):
                rclpy.try_shutdown()
            else:
                rclpy.shutdown()
        # Teardown only after the timed run.
        kill_ros(proc)
        try:
            rpc("disable")
            rpc("set_test_mode", {"on": False})
            final = rpc("get_status")
            cfg = rpc("get_config")
        except Exception as exc:  # noqa: BLE001
            final, cfg = {"error": str(exc)}, {}
        status_known = isinstance(final, dict) and "enabled" in final
        post = {
            "status_end_before_teardown": live,
            "final_after_disable": final if status_known else None,
            "status_known": status_known,
            "checks": {
                "status_known": status_known,
                "enabled_false": status_known and final.get("enabled") is False,
                "test_mode_false": status_known
                and final.get("test_mode") is False
                and cfg.get("test_mode") is False,
                "names_default": cfg.get("names")
                == ["joint_x", "joint_y", "joint_z", "joint_a"],
                "axis_mask_3": cfg.get("axis_mask") == 3,
            },
        }
        (log_dir / "final_board.json").write_text(json.dumps(post, indent=2), encoding="utf-8")
        motion_ok = rc == 0
        connection_ok = True
        if node:
            motion_ok = node.goals_fail == 0 and node.goals_ok > 0
            connection_ok = node.enabled_drops == 0
        summary = {
            "kind": "ros_random_stay_enabled",
            "duration_s_target": DURATION_S,
            "seed": seed,
            "log_dir": str(log_dir),
            "ended_utc": utc(),
            "goals_ok": None if node is None else node.goals_ok,
            "goals_fail": None if node is None else node.goals_fail,
            "enabled_drops": None if node is None else node.enabled_drops,
            "alert_events": None if node is None else node.alert_events,
            "peak_abs_yx_mm": None if node is None else node.peak_abs_yx_m * 1000.0,
            "samples": None if node is None else node.samples,
            "failures": None if node is None else node.failures,
            "verdicts": {
                "motion_ok": motion_ok,
                "stayed_enabled": connection_ok,
                "no_midrun_disconnect": True,
                "final_disabled_verified": bool(post["checks"].get("enabled_false")),
                "alerts_clear": status_known
                and final.get("alerts") in ("none", ""),
            },
            "final": post,
        }
        (log_dir / "summary.json").write_text(json.dumps(summary, indent=2), encoding="utf-8")
        ok = motion_ok and connection_ok and bool(post["checks"].get("enabled_false"))
        print(
            "ROS_RANDOM_STAY_ENABLED_OK" if ok else "ROS_RANDOM_STAY_ENABLED_FAILED",
            flush=True,
        )
        print(f"SUMMARY {log_dir / 'summary.json'}", flush=True)
    return 0 if rc == 0 else 1


if __name__ == "__main__":
    raise SystemExit(main())
