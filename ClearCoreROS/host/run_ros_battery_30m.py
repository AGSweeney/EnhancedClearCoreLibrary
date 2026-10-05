#!/usr/bin/env python3
"""30-minute ClearCoreROS hardware battery with detailed logging.

Captures completed cycles, failures, reconnects; peak |Y-X| and return-position
error per cycle; state-sample gaps, alerts, watchdog events; firmware/host
revisions and final disabled state.

JTC reported endpoint (e.g. -0.14 mm) is logged separately from the later
session origin check. Neither is a shaft-encoder measurement; both are
generated-step / PositionRefCommanded values.
"""

from __future__ import annotations

import json
import os
import re
import signal
import socket
import subprocess
import sys
import threading
import time
from datetime import datetime, timezone
from pathlib import Path

HOST = os.environ.get("CCROS_HOST", "172.16.82.114")
SESSION_PORT = 9200
DURATION_S = int(os.environ.get("CCROS_BATTERY_SECONDS", "1800"))
WS = Path(os.environ.get(
    "CCROS_WS",
    "/mnt/d/CCDev/EnhancedClearCoreLibrary/ClearCoreROS/ros2_ws",
))
LOG_ROOT = Path(os.environ.get(
    "CCROS_LOG_ROOT",
    "/mnt/d/CCDev/EnhancedClearCoreLibrary/ClearCoreROS/logs",
))
REPO = Path(os.environ.get(
    "CCROS_REPO",
    "/mnt/d/CCDev/EnhancedClearCoreLibrary",
))

# Stay inside known soft-limit envelope (X max 0.03 m).
BRIDGE_OUT = 0.020
CANCEL_OUT = 0.018


def _mm(meters: float) -> float:
    return meters * 1000.0


class JointStateSampler:
    """Background ros2 topic echo sampler for /joint_states gaps and peak |Y-X|."""

    def __init__(self, log_path: Path, topic: str = "/joint_states") -> None:
        self.log_path = log_path
        self.topic = topic
        self.proc: subprocess.Popen | None = None
        self._thread: threading.Thread | None = None
        self._stop = threading.Event()
        self.samples: list[dict] = []
        self.gaps_ms: list[float] = []
        self.peak_abs_yx_m = 0.0
        self.alerts_seen = 0  # filled externally from status polls
        self._last_mono: float | None = None
        self._fh = None

    def start(self) -> None:
        self._fh = self.log_path.open("w", encoding="utf-8")
        self.proc = subprocess.Popen(
            ["ros2", "topic", "echo", self.topic],
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            bufsize=1,
        )
        self._thread = threading.Thread(target=self._read_loop, daemon=True)
        self._thread.start()

    def _read_loop(self) -> None:
        assert self.proc is not None and self.proc.stdout is not None
        block: list[str] = []
        for line in self.proc.stdout:
            if self._stop.is_set():
                break
            if self._fh is not None:
                self._fh.write(line)
            if line.strip() == "---":
                self._consume_block(block)
                block = []
            else:
                block.append(line.rstrip("\n"))
        if block:
            self._consume_block(block)

    def _consume_block(self, block: list[str]) -> None:
        names: list[str] = []
        positions: list[float] = []
        section = None
        for line in block:
            s = line.strip()
            if s.startswith("name:"):
                section = "name"
                continue
            if s.startswith("position:"):
                section = "position"
                continue
            if s.startswith("velocity:") or s.startswith("effort:") or s.startswith("header:"):
                section = None
                continue
            if section == "name" and s.startswith("- "):
                names.append(s[2:].strip().strip("'\"") )
            elif section == "position" and s.startswith("- "):
                try:
                    positions.append(float(s[2:].strip()))
                except ValueError:
                    pass
        if "joint_x" not in names or "joint_y" not in names or len(positions) < 2:
            return
        x = positions[names.index("joint_x")]
        y = positions[names.index("joint_y")]
        now = time.monotonic()
        gap_ms = None
        if self._last_mono is not None:
            gap_ms = (now - self._last_mono) * 1000.0
            self.gaps_ms.append(gap_ms)
        self._last_mono = now
        abs_yx = abs(y - x)
        if abs_yx > self.peak_abs_yx_m:
            self.peak_abs_yx_m = abs_yx
        self.samples.append(
            {
                "mono_s": now,
                "x_m": x,
                "y_m": y,
                "abs_yx_m": abs_yx,
                "gap_ms": gap_ms,
            }
        )

    def stop(self) -> dict:
        self._stop.set()
        if self.proc is not None and self.proc.poll() is None:
            self.proc.send_signal(signal.SIGINT)
            try:
                self.proc.wait(timeout=3)
            except subprocess.TimeoutExpired:
                self.proc.kill()
                self.proc.wait(timeout=3)
        if self._thread is not None:
            self._thread.join(timeout=3)
        if self._fh is not None:
            self._fh.close()
        gaps = self.gaps_ms
        # Nominal bridge publish is 20 Hz (50 ms). Flag larger gaps.
        large = [g for g in gaps if g > 150.0]
        return {
            "sample_count": len(self.samples),
            "peak_abs_yx_m": self.peak_abs_yx_m,
            "peak_abs_yx_mm": _mm(self.peak_abs_yx_m),
            "gap_count": len(gaps),
            "gap_max_ms": max(gaps) if gaps else None,
            "gap_mean_ms": (sum(gaps) / len(gaps)) if gaps else None,
            "gaps_over_150ms": len(large),
            "gaps_over_150ms_values": large[:50],
            "note": (
                "Positions are generated-step / PositionRefCommanded joint units, "
                "not shaft-encoder measurements."
            ),
        }


class Battery:
    def __init__(self) -> None:
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.log_dir = LOG_ROOT / f"battery_{stamp}"
        self.log_dir.mkdir(parents=True, exist_ok=True)
        self.events_path = self.log_dir / "events.jsonl"
        self.main_log = self.log_dir / "battery.log"
        self.metrics_csv = self.log_dir / "cycle_metrics.csv"
        self.bridge_proc: subprocess.Popen | None = None
        self.traj_proc: subprocess.Popen | None = None
        self._t0 = time.monotonic()
        self._stop = False
        self._reconnects = 0
        self._watchdog_events: list[dict] = []
        self._alert_events: list[dict] = []
        self.summary = {
            "host": HOST,
            "started_utc": datetime.now(timezone.utc).isoformat(),
            "duration_s_target": DURATION_S,
            "revisions": {},
            "cycles": [],
            "counts": {
                "cycles_started": 0,
                "cycles_completed": 0,
                "cycles_ok": 0,
                "cycles_failed": 0,
                "reconnects": 0,
                "bridge_fjt_ok": 0,
                "bridge_fjt_fail": 0,
                "bridge_cancel_ok": 0,
                "bridge_cancel_fail": 0,
                "jtc_ok": 0,
                "jtc_fail": 0,
                "origin_ok": 0,
                "origin_fail": 0,
                "watchdog_events": 0,
                "alert_events": 0,
                "state_gaps_over_150ms": 0,
            },
            "failures": [],
            "measurement_note": (
                "Peak |Y-X|, JTC reported endpoint, and session origin checks are "
                "generated-step / PositionRefCommanded values, not shaft-encoder "
                "measurements. JTC reported endpoint is kept separate from the "
                "later origin check."
            ),
        }
        self.metrics_csv.write_text(
            ",".join(
                [
                    "cycle",
                    "ok",
                    "bridge_peak_abs_yx_mm",
                    "jtc_peak_abs_yx_mm",
                    "jtc_reported_end_x_mm",
                    "jtc_reported_end_y_mm",
                    "origin_return_x_mm",
                    "origin_return_y_mm",
                    "origin_return_err_mm",
                    "bridge_gap_max_ms",
                    "jtc_gap_max_ms",
                    "gaps_over_150ms",
                    "watchdog_events",
                    "alert_events",
                    "reconnects_this_cycle",
                ]
            )
            + "\n",
            encoding="utf-8",
        )
        signal.signal(signal.SIGINT, self._on_signal)
        signal.signal(signal.SIGTERM, self._on_signal)

    def _on_signal(self, signum, _frame) -> None:
        self.log(f"signal {signum}; stopping after current step")
        self._stop = True

    def elapsed(self) -> float:
        return time.monotonic() - self._t0

    def remaining(self) -> float:
        return DURATION_S - self.elapsed()

    def log(self, msg: str, **fields) -> None:
        line = f"[{self.elapsed():8.1f}s] {msg}"
        if fields:
            line += " " + json.dumps(fields, sort_keys=True, default=str)
        print(line, flush=True)
        with self.main_log.open("a", encoding="utf-8") as fh:
            fh.write(line + "\n")
        event = {
            "t_s": round(self.elapsed(), 3),
            "utc": datetime.now(timezone.utc).isoformat(),
            "msg": msg,
            **fields,
        }
        with self.events_path.open("a", encoding="utf-8") as fh:
            fh.write(json.dumps(event, default=str) + "\n")

    def session_call(self, method: str, params=None, timeout: float = 10.0, retries: int = 4):
        last_exc: Exception | None = None
        for attempt in range(retries):
            try:
                sock = socket.create_connection((HOST, SESSION_PORT), timeout)
                try:
                    msg = {"jsonrpc": "2.0", "id": 1, "method": method}
                    if params is not None:
                        msg["params"] = params
                    sock.sendall((json.dumps(msg) + "\n").encode("utf-8"))
                    sock.settimeout(timeout)
                    buf = b""
                    while b"\n" not in buf:
                        chunk = sock.recv(4096)
                        if not chunk:
                            raise ConnectionError("session closed")
                        buf += chunk
                    reply = json.loads(buf.decode("utf-8"))
                    if "error" in reply:
                        raise RuntimeError(reply["error"])
                    return reply.get("result", reply)
                finally:
                    sock.close()
            except Exception as exc:  # noqa: BLE001
                last_exc = exc
                self.log(
                    "session_call retry",
                    method=method,
                    attempt=attempt + 1,
                    error=str(exc),
                )
                time.sleep(1.0 + attempt)
        assert last_exc is not None
        raise last_exc

    def collect_revisions(self) -> dict:
        revisions: dict = {
            "host_packages": {
                "clearcore_bridge": "0.1.0",
                "clearcore_hardware": "0.1.0",
            },
            "protocol": "1.0",
            "git_head": None,
            "discover": None,
            "get_config_nvm": None,
        }
        try:
            out = subprocess.check_output(
                ["git", "-C", str(REPO), "rev-parse", "HEAD"],
                text=True,
                stderr=subprocess.DEVNULL,
            ).strip()
            revisions["git_head"] = out
        except Exception as exc:  # noqa: BLE001
            revisions["git_head_error"] = str(exc)
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            sock.settimeout(2.0)
            sock.sendto(b"CLEARCORE_ROS_DISCOVER?", (HOST, 9202))
            data, _ = sock.recvfrom(2048)
            sock.close()
            text = data.decode("utf-8", errors="replace").strip()
            revisions["discover"] = text
            # Example: CLEARCORE_ROS ClearCoreROS IP=... FW=1.0
            m = re.search(r"FW=([0-9.]+)", text)
            if m:
                revisions["firmware_protocol_fw"] = m.group(1)
        except Exception as exc:  # noqa: BLE001
            revisions["discover_error"] = str(exc)
        try:
            cfg = self.session_call("get_config")
            revisions["get_config_nvm"] = {
                "nvm": cfg.get("nvm"),
                "nvm_valid": cfg.get("nvm_valid"),
                "nvm_version": cfg.get("nvm_version"),
                "axis_mask": cfg.get("axis_mask"),
                "test_mode": cfg.get("test_mode"),
                "names": cfg.get("names"),
                "ip_address": cfg.get("ip_address"),
            }
        except Exception as exc:  # noqa: BLE001
            revisions["get_config_error"] = str(exc)
        self.summary["revisions"] = revisions
        (self.log_dir / "revisions.json").write_text(
            json.dumps(revisions, indent=2), encoding="utf-8"
        )
        self.log("revisions", **{k: revisions[k] for k in revisions if k != "get_config_nvm"})
        return revisions

    def note_status_flags(self, status: dict, where: str) -> None:
        if status.get("watchdog"):
            ev = {"t_s": self.elapsed(), "where": where, "kind": "watchdog", "status": status}
            self._watchdog_events.append(ev)
            self.summary["counts"]["watchdog_events"] += 1
            self.log("WATCHDOG", where=where, alerts=status.get("alerts"))
        alerts = status.get("alerts")
        if alerts not in (None, "", "none"):
            ev = {"t_s": self.elapsed(), "where": where, "kind": "alert", "alerts": alerts}
            self._alert_events.append(ev)
            self.summary["counts"]["alert_events"] += 1
            self.log("ALERT", where=where, alerts=alerts)

    def dump_board(self, label: str, cycle_dir: Path | None = None) -> dict:
        try:
            status = self.session_call("get_status")
            cfg = self.session_call("get_config")
        except Exception as exc:  # noqa: BLE001
            payload = {"label": label, "error": str(exc)}
            self.log(f"board {label} FAILED", error=str(exc))
            target = (cycle_dir or self.log_dir) / f"board_{label}.json"
            target.write_text(json.dumps(payload, indent=2), encoding="utf-8")
            return payload
        self.note_status_flags(status, label)
        payload = {"label": label, "status": status, "config": cfg}
        self.log(
            f"board {label}",
            enabled=status.get("enabled"),
            moving=status.get("moving"),
            watchdog=status.get("watchdog"),
            alerts=status.get("alerts"),
            test_mode=status.get("test_mode"),
            pos_m=status.get("position", [])[:2],
            pos_mm=[_mm(p) for p in status.get("position", [])[:2]],
        )
        target = (cycle_dir or self.log_dir) / f"board_{label}.json"
        target.write_text(json.dumps(payload, indent=2), encoding="utf-8")
        return payload

    def ensure_origin(self, cycle_dir: Path, label: str = "origin") -> dict:
        """Session set_joints to origin. Separate from JTC reported endpoint."""
        out = {
            "label": label,
            "ok": False,
            "return_x_m": None,
            "return_y_m": None,
            "return_err_m": None,
            "measurement": "generated-step / PositionRefCommanded (not shaft encoder)",
            "separate_from": "jtc_reported_endpoint",
        }
        try:
            # Preserve pre-clear alert transitions (e.g. motor_faulted while disabled)
            # before clear_alerts — do not auto-clear without a snapshot.
            try:
                pre_clear = self.session_call("get_status")
            except Exception as exc:  # noqa: BLE001
                pre_clear = {"error": str(exc)}
            self.note_status_flags(
                pre_clear if isinstance(pre_clear, dict) else {},
                f"{label}_pre_clear",
            )
            (cycle_dir / f"{label}_pre_clear_status.json").write_text(
                json.dumps(
                    {
                        "label": label,
                        "note": "Status immediately before clear_alerts",
                        "status": pre_clear,
                    },
                    indent=2,
                ),
                encoding="utf-8",
            )
            alerts = (
                pre_clear.get("alerts")
                if isinstance(pre_clear, dict)
                else None
            )
            if alerts not in (None, "", "none"):
                self.log(
                    "preserved alert transition before clear",
                    where=label,
                    alerts=alerts,
                    enabled=pre_clear.get("enabled")
                    if isinstance(pre_clear, dict)
                    else None,
                    fault=pre_clear.get("fault")
                    if isinstance(pre_clear, dict)
                    else None,
                )

            self.session_call("disable")
            self.session_call("clear_alerts")
            self.session_call("set_test_mode", {"on": False})
            self.session_call("enable")
            self.session_call("set_joints", {"x": 0.0, "y": 0.0})
            deadline = time.time() + 20.0
            while time.time() < deadline:
                st = self.session_call("get_status")
                self.note_status_flags(st, f"{label}_wait")
                pos = st["position"]
                if (
                    not st.get("moving")
                    and abs(pos[0]) < 0.001
                    and abs(pos[1]) < 0.001
                ):
                    break
                time.sleep(0.1)
            self.session_call("disable")
            st = self.session_call("get_status")
            self.note_status_flags(st, f"{label}_final")
            x, y = float(st["position"][0]), float(st["position"][1])
            err = max(abs(x), abs(y))
            out.update(
                {
                    "ok": err < 0.001,
                    "return_x_m": x,
                    "return_y_m": y,
                    "return_err_m": err,
                    "return_x_mm": _mm(x),
                    "return_y_mm": _mm(y),
                    "return_err_mm": _mm(err),
                    "status": st,
                }
            )
            key = "origin_ok" if out["ok"] else "origin_fail"
            self.summary["counts"][key] += 1
            if not out["ok"]:
                self.summary["failures"].append(
                    {"t_s": self.elapsed(), "step": label, "pos_mm": [_mm(x), _mm(y)]}
                )
            self.log(
                f"{label} {'ok' if out['ok'] else 'FAIL'}",
                return_x_mm=_mm(x),
                return_y_mm=_mm(y),
                return_err_mm=_mm(err),
                note=out["measurement"],
            )
        except Exception as exc:  # noqa: BLE001
            out["error"] = str(exc)
            self.summary["counts"]["origin_fail"] += 1
            self.summary["failures"].append(
                {"t_s": self.elapsed(), "step": label, "error": str(exc)}
            )
            self.log(f"{label} exception", error=str(exc))
            try:
                self.session_call("disable")
            except Exception:
                pass
        (cycle_dir / f"{label}.json").write_text(json.dumps(out, indent=2), encoding="utf-8")
        return out

    def kill_ros(self, reason: str = "cleanup") -> None:
        for proc in (self.bridge_proc, self.traj_proc):
            if proc is not None and proc.poll() is None:
                proc.send_signal(signal.SIGINT)
                try:
                    proc.wait(timeout=8)
                except subprocess.TimeoutExpired:
                    proc.kill()
                    proc.wait(timeout=5)
        self.bridge_proc = None
        self.traj_proc = None
        subprocess.run(
            [
                "pkill",
                "-f",
                "clearcore_bridge|ros2_control_node|robot_state_publisher|spawner|send_trajectory_goal",
            ],
            check=False,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )
        time.sleep(1.5)
        self.log("ros stopped", reason=reason)

    def launch(self, args: list[str], log_path: Path) -> subprocess.Popen:
        fh = log_path.open("w", encoding="utf-8")
        proc = subprocess.Popen(
            args,
            stdout=fh,
            stderr=subprocess.STDOUT,
            cwd=str(WS),
            start_new_session=True,
        )
        proc._log_fh = fh  # type: ignore[attr-defined]
        return proc

    def wait_for(self, checker, timeout: float, label: str) -> bool:
        deadline = time.time() + timeout
        while time.time() < deadline:
            if self._stop:
                return False
            try:
                if checker():
                    self.log(f"ready {label}")
                    return True
            except Exception:
                pass
            time.sleep(0.5)
        self.log(f"timeout waiting for {label}")
        return False

    def action_present(self, name: str) -> bool:
        out = subprocess.check_output(
            ["ros2", "action", "list"], text=True, stderr=subprocess.DEVNULL
        )
        return name in out

    def run_python(self, code: str, log_path: Path, timeout: float) -> int:
        with log_path.open("w", encoding="utf-8") as fh:
            proc = subprocess.run(
                [sys.executable, "-c", code],
                stdout=fh,
                stderr=subprocess.STDOUT,
                timeout=timeout,
                check=False,
            )
        return int(proc.returncode)

    def _log_contains(self, log_path: Path, needle: str) -> bool:
        try:
            text = log_path.read_text(encoding="utf-8", errors="replace")
        except OSError:
            return False
        return needle in text

    def _fresh_joint_states(self, min_samples: int = 2, max_age_s: float = 0.5) -> bool:
        """Require recent /joint_states samples (bridge publishes only when connected)."""
        try:
            proc = subprocess.run(
                [
                    "ros2",
                    "topic",
                    "echo",
                    "/joint_states",
                    "--once",
                ],
                capture_output=True,
                text=True,
                timeout=2.0,
                check=False,
            )
        except Exception:
            return False
        if proc.returncode != 0 or "position:" not in (proc.stdout or ""):
            return False
        # A second sample proves the publisher is still live, not a stale once.
        try:
            proc2 = subprocess.run(
                ["ros2", "topic", "echo", "/joint_states", "--once"],
                capture_output=True,
                text=True,
                timeout=2.0,
                check=False,
            )
        except Exception:
            return False
        return proc2.returncode == 0 and "position:" in (proc2.stdout or "")

    def wait_bridge_ready(self, log_path: Path, label: str) -> bool:
        """Action alone is not enough: connect runs on a 2 s timer after advertise."""
        if not self.wait_for(
            lambda: self.action_present("/follow_joint_trajectory"),
            30.0,
            f"bridge action ({label})",
        ):
            return False
        if not self.wait_for(
            lambda: self._log_contains(log_path, "connected and enabled"),
            15.0,
            f"bridge connected ({label})",
        ):
            return False
        if not self.wait_for(
            self._fresh_joint_states,
            10.0,
            f"bridge fresh joint_states ({label})",
        ):
            return False
        return True

    def wait_jtc_ready(self, log_path: Path, label: str) -> bool:
        if not self.wait_for(
            lambda: self.action_present(
                "/joint_trajectory_controller/follow_joint_trajectory"
            ),
            45.0,
            f"jtc action ({label})",
        ):
            return False
        # Plugin activate logs success; joint_states from the broadcaster prove live HW.
        if not self.wait_for(
            lambda: (
                self._log_contains(log_path, "Successful 'activate'")
                or self._log_contains(log_path, "activate")
                or self._fresh_joint_states()
            ),
            20.0,
            f"jtc hardware live ({label})",
        ):
            return False
        if not self.wait_for(
            self._fresh_joint_states,
            10.0,
            f"jtc fresh joint_states ({label})",
        ):
            return False
        return True

    def connect_bridge(self, cycle_dir: Path, attempt_label: str) -> bool:
        log_path = cycle_dir / f"bridge_launch_{attempt_label}.log"
        self.bridge_proc = self.launch(
            [
                "ros2",
                "launch",
                "clearcore_bridge",
                "bridge.launch.py",
                f"host:={HOST}",
                "axis_mask:=3",
                "test_mode:=false",
            ],
            log_path,
        )
        ok = self.wait_bridge_ready(log_path, attempt_label)
        if not ok:
            self._reconnects += 1
            self.summary["counts"]["reconnects"] += 1
            self.log("reconnect bridge", attempt=attempt_label)
            self.kill_ros(reason=f"bridge_reconnect_{attempt_label}")
            time.sleep(1.0)
            retry_log = cycle_dir / f"bridge_launch_{attempt_label}_retry.log"
            self.bridge_proc = self.launch(
                [
                    "ros2",
                    "launch",
                    "clearcore_bridge",
                    "bridge.launch.py",
                    f"host:={HOST}",
                    "axis_mask:=3",
                    "test_mode:=false",
                ],
                retry_log,
            )
            ok = self.wait_bridge_ready(retry_log, f"{attempt_label}_retry")
        return ok

    def connect_jtc(self, cycle_dir: Path) -> bool:
        log_path = cycle_dir / "traj_launch.log"
        self.traj_proc = self.launch(
            [
                "ros2",
                "launch",
                "clearcore_hardware",
                "trajectory.launch.py",
                f"host:={HOST}",
                "axis_mask:=3",
                "test_mode:=false",
            ],
            log_path,
        )
        ok = self.wait_jtc_ready(log_path, "main")
        if not ok:
            self._reconnects += 1
            self.summary["counts"]["reconnects"] += 1
            self.log("reconnect jtc")
            self.kill_ros(reason="jtc_reconnect")
            time.sleep(1.0)
            retry_log = cycle_dir / "traj_launch_retry.log"
            self.traj_proc = self.launch(
                [
                    "ros2",
                    "launch",
                    "clearcore_hardware",
                    "trajectory.launch.py",
                    f"host:={HOST}",
                    "axis_mask:=3",
                    "test_mode:=false",
                ],
                retry_log,
            )
            ok = self.wait_jtc_ready(retry_log, "retry")
        return ok

    def bridge_fjt(self, cycle_dir: Path, target: float) -> bool:
        code = f"""
import sys
import rclpy
from action_msgs.msg import GoalStatus
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectoryPoint

class Sender(Node):
    def __init__(self):
        super().__init__('battery_bridge_fjt')
        self.client = ActionClient(self, FollowJointTrajectory, '/follow_joint_trajectory')
    def run(self):
        if not self.client.wait_for_server(timeout_sec=20.0):
            self.get_logger().error('action unavailable')
            return 1
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = ['joint_x', 'joint_y']
        a = JointTrajectoryPoint(); a.positions=[{target}, {target}]; a.velocities=[0.0,0.0]; a.time_from_start.sec=2
        b = JointTrajectoryPoint(); b.positions=[0.0,0.0]; b.velocities=[0.0,0.0]; b.time_from_start.sec=4
        goal.trajectory.points=[a,b]
        fut = self.client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, fut)
        handle = fut.result()
        if handle is None or not handle.accepted:
            self.get_logger().error('rejected')
            return 2
        res = handle.get_result_async()
        rclpy.spin_until_future_complete(self, res)
        wrapped = res.result()
        status=int(wrapped.status); code=int(wrapped.result.error_code)
        self.get_logger().info(f'result status={{status}} error_code={{code}} {{wrapped.result.error_string}}')
        return 0 if status==GoalStatus.STATUS_SUCCEEDED and code==FollowJointTrajectory.Result.SUCCESSFUL else 3

rclpy.init()
node=Sender()
try:
    sys.exit(node.run())
finally:
    node.destroy_node(); rclpy.shutdown()
"""
        rc = self.run_python(code, cycle_dir / "bridge_fjt.log", timeout=90)
        ok = rc == 0
        self.summary["counts"]["bridge_fjt_ok" if ok else "bridge_fjt_fail"] += 1
        if not ok:
            self.summary["failures"].append(
                {"t_s": self.elapsed(), "step": "bridge_fjt", "rc": rc}
            )
        self.log("bridge_fjt done", rc=rc, ok=ok, target_m=target)
        return ok

    def bridge_cancel(self, cycle_dir: Path, target: float) -> bool:
        code = f"""
import sys, time
import rclpy
from action_msgs.msg import GoalStatus
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectoryPoint

class Canceller(Node):
    def __init__(self):
        super().__init__('battery_bridge_cancel')
        self.client = ActionClient(self, FollowJointTrajectory, '/follow_joint_trajectory')
    def run(self):
        if not self.client.wait_for_server(timeout_sec=15.0):
            self.get_logger().error('action unavailable')
            return 1
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = ['joint_x', 'joint_y']
        a = JointTrajectoryPoint(); a.positions=[{target}, {target}]; a.velocities=[0.0,0.0]; a.time_from_start.sec=3
        b = JointTrajectoryPoint(); b.positions=[0.0,0.0]; b.velocities=[0.0,0.0]; b.time_from_start.sec=6
        goal.trajectory.points=[a,b]
        fut = self.client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, fut)
        handle = fut.result()
        if handle is None or not handle.accepted:
            self.get_logger().error('rejected')
            return 2
        time.sleep(0.7)
        cancel_fut = handle.cancel_goal_async()
        rclpy.spin_until_future_complete(self, cancel_fut)
        res = handle.get_result_async()
        rclpy.spin_until_future_complete(self, res)
        wrapped = res.result()
        status=int(wrapped.status); code=int(wrapped.result.error_code)
        self.get_logger().info(f'cancel status={{status}} error_code={{code}} {{wrapped.result.error_string}}')
        if status == GoalStatus.STATUS_CANCELED:
            return 0
        if status == GoalStatus.STATUS_SUCCEEDED:
            return 3
        self.get_logger().warning(f'unexpected status {{status}}')
        return 0

rclpy.init()
node=Canceller()
try:
    sys.exit(node.run())
finally:
    node.destroy_node(); rclpy.shutdown()
"""
        rc = self.run_python(code, cycle_dir / "bridge_cancel.log", timeout=90)
        ok = rc == 0
        self.summary["counts"]["bridge_cancel_ok" if ok else "bridge_cancel_fail"] += 1
        if not ok:
            self.summary["failures"].append(
                {"t_s": self.elapsed(), "step": "bridge_cancel", "rc": rc}
            )
        self.log("bridge_cancel done", rc=rc, ok=ok, target_m=target)
        return ok

    def parse_jtc_log(self, text: str) -> dict:
        """Pull JTC reported endpoint/peak from send_trajectory_goal output.

        Kept separate from session origin check. Generated-step units only.
        """
        out = {
            "reported_end_x_mm": None,
            "reported_end_y_mm": None,
            "reported_peak_abs_yx_mm": None,
            "result_line": None,
            "samples_line": None,
            "measurement": "generated-step / PositionRefCommanded (not shaft encoder)",
            "separate_from": "session_origin_check",
        }
        for line in text.splitlines():
            if "result status=" in line:
                out["result_line"] = line.strip()
            if "samples=" in line and "end_mm=" in line:
                out["samples_line"] = line.strip()
                m_peak = re.search(r"peak_\|Y-X\|_mm=([0-9.+-eE]+)", line)
                m_end = re.search(r"end_mm=\(([0-9.+-eE]+),\s*([0-9.+-eE]+)\)", line)
                if m_peak:
                    out["reported_peak_abs_yx_mm"] = float(m_peak.group(1))
                if m_end:
                    out["reported_end_x_mm"] = float(m_end.group(1))
                    out["reported_end_y_mm"] = float(m_end.group(2))
        return out

    def jtc_goal(self, cycle_dir: Path) -> tuple[bool, dict]:
        log_path = cycle_dir / "jtc_goal.log"
        with log_path.open("w", encoding="utf-8") as fh:
            proc = subprocess.run(
                ["ros2", "run", "clearcore_hardware", "send_trajectory_goal.py"],
                stdout=fh,
                stderr=subprocess.STDOUT,
                timeout=120,
                check=False,
            )
        rc = int(proc.returncode)
        text = log_path.read_text(encoding="utf-8", errors="replace")
        parsed = self.parse_jtc_log(text)
        ok = rc == 0
        self.summary["counts"]["jtc_ok" if ok else "jtc_fail"] += 1
        if not ok:
            self.summary["failures"].append(
                {"t_s": self.elapsed(), "step": "jtc", "rc": rc}
            )
        self.log(
            "jtc done",
            rc=rc,
            ok=ok,
            jtc_reported_end_mm=[
                parsed.get("reported_end_x_mm"),
                parsed.get("reported_end_y_mm"),
            ],
            jtc_reported_peak_abs_yx_mm=parsed.get("reported_peak_abs_yx_mm"),
            note="JTC reported endpoint is separate from later origin check",
        )
        (cycle_dir / "jtc_reported_endpoint.json").write_text(
            json.dumps(parsed, indent=2), encoding="utf-8"
        )
        return ok, parsed

    def run_cycle(self, cycle_idx: int) -> dict:
        cycle_dir = self.log_dir / f"cycle_{cycle_idx:03d}"
        cycle_dir.mkdir(parents=True, exist_ok=True)
        reconnects_before = self._reconnects
        watchdog_before = len(self._watchdog_events)
        alerts_before = len(self._alert_events)
        result = {
            "cycle": cycle_idx,
            "t_start_s": round(self.elapsed(), 3),
            "steps": {},
            "metrics": {
                "bridge_peak_abs_yx_mm": None,
                "jtc_peak_abs_yx_mm_from_sampler": None,
                "jtc_reported_endpoint": None,
                "origin_return": None,
                "bridge_state_gaps": None,
                "jtc_state_gaps": None,
            },
            "measurement_note": self.summary["measurement_note"],
        }
        self.summary["counts"]["cycles_started"] += 1
        self.log(
            f"===== cycle {cycle_idx} start =====",
            remaining_s=round(self.remaining(), 1),
        )
        self.kill_ros(reason=f"cycle_{cycle_idx}_start")
        self.dump_board(f"cycle{cycle_idx:03d}_pre", cycle_dir)
        origin_pre = self.ensure_origin(cycle_dir, "origin_pre")
        result["steps"]["origin_pre"] = origin_pre["ok"]
        if not origin_pre["ok"]:
            result["ok"] = False
            result["t_end_s"] = round(self.elapsed(), 3)
            self._finish_cycle_record(result, cycle_dir, reconnects_before, watchdog_before, alerts_before)
            return result

        # Bridge phase with live joint_states sampling.
        ready = self.connect_bridge(cycle_dir, "main")
        result["steps"]["bridge_ready"] = ready
        bridge_sampler = None
        if ready:
            bridge_sampler = JointStateSampler(cycle_dir / "bridge_joint_states_stream.log")
            bridge_sampler.start()
            time.sleep(0.5)
            result["steps"]["bridge_fjt"] = self.bridge_fjt(cycle_dir, BRIDGE_OUT)
            if self.remaining() > 90 and not self._stop:
                result["steps"]["bridge_cancel"] = self.bridge_cancel(cycle_dir, CANCEL_OUT)
            bridge_stats = bridge_sampler.stop()
            result["metrics"]["bridge_peak_abs_yx_mm"] = bridge_stats["peak_abs_yx_mm"]
            result["metrics"]["bridge_state_gaps"] = {
                k: bridge_stats[k]
                for k in (
                    "sample_count",
                    "gap_max_ms",
                    "gap_mean_ms",
                    "gaps_over_150ms",
                    "gaps_over_150ms_values",
                )
            }
            self.summary["counts"]["state_gaps_over_150ms"] += bridge_stats["gaps_over_150ms"]
            (cycle_dir / "bridge_sampler.json").write_text(
                json.dumps(bridge_stats, indent=2), encoding="utf-8"
            )
            self.log(
                "bridge sampler",
                peak_abs_yx_mm=bridge_stats["peak_abs_yx_mm"],
                gap_max_ms=bridge_stats["gap_max_ms"],
                gaps_over_150ms=bridge_stats["gaps_over_150ms"],
            )
        self.kill_ros(reason=f"cycle_{cycle_idx}_after_bridge")
        time.sleep(1.0)

        if self.remaining() < 100 or self._stop:
            result["ok"] = all(bool(v) for v in result["steps"].values()) if result["steps"] else False
            result["t_end_s"] = round(self.elapsed(), 3)
            self._finish_cycle_record(result, cycle_dir, reconnects_before, watchdog_before, alerts_before)
            return result

        origin_mid = self.ensure_origin(cycle_dir, "origin_mid")
        result["steps"]["origin_mid"] = origin_mid["ok"]

        ready = self.connect_jtc(cycle_dir)
        result["steps"]["jtc_ready"] = ready
        jtc_reported = None
        if ready:
            jtc_sampler = JointStateSampler(cycle_dir / "jtc_joint_states_stream.log")
            jtc_sampler.start()
            time.sleep(2.0)
            ok, jtc_reported = self.jtc_goal(cycle_dir)
            result["steps"]["jtc"] = ok
            jtc_stats = jtc_sampler.stop()
            result["metrics"]["jtc_peak_abs_yx_mm_from_sampler"] = jtc_stats["peak_abs_yx_mm"]
            result["metrics"]["jtc_reported_endpoint"] = jtc_reported
            result["metrics"]["jtc_state_gaps"] = {
                k: jtc_stats[k]
                for k in (
                    "sample_count",
                    "gap_max_ms",
                    "gap_mean_ms",
                    "gaps_over_150ms",
                    "gaps_over_150ms_values",
                )
            }
            self.summary["counts"]["state_gaps_over_150ms"] += jtc_stats["gaps_over_150ms"]
            (cycle_dir / "jtc_sampler.json").write_text(
                json.dumps(jtc_stats, indent=2), encoding="utf-8"
            )
            self.log(
                "jtc sampler",
                peak_abs_yx_mm=jtc_stats["peak_abs_yx_mm"],
                jtc_reported_end_mm=[
                    jtc_reported.get("reported_end_x_mm"),
                    jtc_reported.get("reported_end_y_mm"),
                ],
                note="reported endpoint != later origin check",
            )
        self.kill_ros(reason=f"cycle_{cycle_idx}_after_jtc")
        time.sleep(1.0)

        # Later origin check — separate from JTC reported endpoint.
        origin_post = self.ensure_origin(cycle_dir, "origin_post")
        result["steps"]["origin_post"] = origin_post["ok"]
        result["metrics"]["origin_return"] = {
            "x_mm": origin_post.get("return_x_mm"),
            "y_mm": origin_post.get("return_y_mm"),
            "err_mm": origin_post.get("return_err_mm"),
            "ok": origin_post.get("ok"),
            "measurement": origin_post.get("measurement"),
            "separate_from": "jtc_reported_endpoint",
        }
        self.dump_board(f"cycle{cycle_idx:03d}_post", cycle_dir)

        result["ok"] = all(bool(v) for v in result["steps"].values()) if result["steps"] else False
        result["t_end_s"] = round(self.elapsed(), 3)
        self._finish_cycle_record(result, cycle_dir, reconnects_before, watchdog_before, alerts_before)
        return result

    def _finish_cycle_record(
        self,
        result: dict,
        cycle_dir: Path,
        reconnects_before: int,
        watchdog_before: int,
        alerts_before: int,
    ) -> None:
        reconnects_this = self._reconnects - reconnects_before
        watchdog_this = len(self._watchdog_events) - watchdog_before
        alerts_this = len(self._alert_events) - alerts_before
        result["reconnects_this_cycle"] = reconnects_this
        result["watchdog_events_this_cycle"] = watchdog_this
        result["alert_events_this_cycle"] = alerts_this
        self.summary["counts"]["cycles_completed"] += 1
        if result.get("ok"):
            self.summary["counts"]["cycles_ok"] += 1
        else:
            self.summary["counts"]["cycles_failed"] += 1

        metrics = result.get("metrics", {})
        jtc_rep = metrics.get("jtc_reported_endpoint") or {}
        origin = metrics.get("origin_return") or {}
        bridge_gaps = metrics.get("bridge_state_gaps") or {}
        jtc_gaps = metrics.get("jtc_state_gaps") or {}
        gaps_over = int(bridge_gaps.get("gaps_over_150ms") or 0) + int(
            jtc_gaps.get("gaps_over_150ms") or 0
        )
        gap_max_bridge = bridge_gaps.get("gap_max_ms")
        gap_max_jtc = jtc_gaps.get("gap_max_ms")

        with self.metrics_csv.open("a", encoding="utf-8") as fh:
            fh.write(
                ",".join(
                    str(x) if x is not None else ""
                    for x in [
                        result["cycle"],
                        result.get("ok"),
                        metrics.get("bridge_peak_abs_yx_mm"),
                        jtc_rep.get("reported_peak_abs_yx_mm"),
                        jtc_rep.get("reported_end_x_mm"),
                        jtc_rep.get("reported_end_y_mm"),
                        origin.get("x_mm"),
                        origin.get("y_mm"),
                        origin.get("err_mm"),
                        gap_max_bridge,
                        gap_max_jtc,
                        gaps_over,
                        watchdog_this,
                        alerts_this,
                        reconnects_this,
                    ]
                )
                + "\n"
            )

        (cycle_dir / "result.json").write_text(
            json.dumps(result, indent=2), encoding="utf-8"
        )
        self.log(
            f"===== cycle {result['cycle']} end =====",
            ok=result.get("ok"),
            bridge_peak_abs_yx_mm=metrics.get("bridge_peak_abs_yx_mm"),
            jtc_reported_end_mm=[
                jtc_rep.get("reported_end_x_mm"),
                jtc_rep.get("reported_end_y_mm"),
            ],
            origin_return_err_mm=origin.get("err_mm"),
            reconnects=reconnects_this,
            watchdog_events=watchdog_this,
            alert_events=alerts_this,
            gaps_over_150ms=gaps_over,
        )

    def finalize(self) -> int:
        self.log("finalizing")
        self.kill_ros(reason="finalize")
        try:
            self.session_call("disable")
            self.session_call("set_test_mode", {"on": False})
            cfg = self.session_call("get_config")
            st = self.session_call("get_status")
            self.note_status_flags(st, "finalize")
        except Exception as exc:  # noqa: BLE001
            self.log("finalize session failed", error=str(exc))
            cfg = {}
            st = {}
        status_known = isinstance(st, dict) and "enabled" in st
        if not status_known:
            alerts = "unknown"
            no_alerts_check = False
            no_alerts_value = "unknown"
            watchdog = "unknown"
        else:
            alerts = st.get("alerts")
            watchdog = st.get("watchdog")
            if alerts in ("none", ""):
                no_alerts_check = True
                no_alerts_value = True
            elif alerts is None:
                # Missing alerts field is unknown, never a pass.
                no_alerts_check = False
                no_alerts_value = "unknown"
            else:
                no_alerts_check = False
                no_alerts_value = False
        post = {
            "status": st if status_known else None,
            "status_known": status_known,
            "config": cfg if cfg else None,
            "disabled_state": {
                "enabled": st.get("enabled") if status_known else "unknown",
                "test_mode_status": st.get("test_mode") if status_known else "unknown",
                "test_mode_config": cfg.get("test_mode") if cfg else "unknown",
                "moving": st.get("moving") if status_known else "unknown",
                "watchdog": watchdog if status_known else "unknown",
                "alerts": alerts if status_known else "unknown",
                "position_m": st.get("position") if status_known else "unknown",
                "names": cfg.get("names") if cfg else "unknown",
                "axis_mask": cfg.get("axis_mask") if cfg else "unknown",
            },
            "checks": {
                "status_known": status_known,
                "enabled_false": status_known and st.get("enabled") is False,
                "test_mode_false": status_known
                and st.get("test_mode") is False
                and bool(cfg)
                and cfg.get("test_mode") is False,
                "names_default": bool(cfg)
                and cfg.get("names")
                == ["joint_x", "joint_y", "joint_z", "joint_a"],
                "axis_mask_3": bool(cfg) and cfg.get("axis_mask") == 3,
                "no_alerts": no_alerts_check,
                "no_watchdog": status_known and watchdog is False,
            },
            "check_details": {
                "no_alerts": no_alerts_value,
                "enabled": st.get("enabled") if status_known else "unknown",
                "watchdog": watchdog if status_known else "unknown",
            },
        }
        (self.log_dir / "final_board.json").write_text(
            json.dumps(post, indent=2), encoding="utf-8"
        )
        (self.log_dir / "watchdog_events.json").write_text(
            json.dumps(self._watchdog_events, indent=2), encoding="utf-8"
        )
        (self.log_dir / "alert_events.json").write_text(
            json.dumps(self._alert_events, indent=2), encoding="utf-8"
        )

        # Drift view across cycles from origin return error.
        origin_errs = []
        for cyc in self.summary["cycles"]:
            err = ((cyc.get("metrics") or {}).get("origin_return") or {}).get("err_mm")
            if err is not None:
                origin_errs.append({"cycle": cyc["cycle"], "origin_return_err_mm": err})

        self.summary["ended_utc"] = datetime.now(timezone.utc).isoformat()
        self.summary["duration_s_actual"] = round(self.elapsed(), 3)
        self.summary["final"] = post
        self.summary["log_dir"] = str(self.log_dir)
        self.summary["origin_return_errors_mm"] = origin_errs
        self.summary["counts"]["reconnects"] = self._reconnects
        motion_ok = (
            self.summary["counts"]["cycles_failed"] == 0
            and self.summary["counts"]["bridge_fjt_fail"] == 0
            and self.summary["counts"]["bridge_cancel_fail"] == 0
            and self.summary["counts"]["jtc_fail"] == 0
            and self.summary["counts"]["origin_fail"] == 0
        )
        connection_ok = self._reconnects == 0 and not any(
            "Connection reset" in str(f) or "reset by peer" in str(f)
            for f in self.summary["failures"]
        )
        final_disabled_ok = bool(
            post["checks"].get("status_known")
            and post["checks"].get("enabled_false")
            and post["checks"].get("test_mode_false")
            and post["checks"].get("names_default")
            and post["checks"].get("axis_mask_3")
            and post["checks"].get("no_watchdog")
        )
        alerts_clear = bool(post["checks"].get("no_alerts"))
        # motor_faulted / other alerts are a separate investigation track from
        # motion endurance and connection stability.
        motor_faulted_events = [
            e for e in self._alert_events if e.get("alerts") == "motor_faulted"
        ]
        # Observation rows ≠ distinct faults: cycle*_pre and origin_*_pre_clear
        # often share one latched event; tail polls can repeat the final latch.
        self.summary["verdicts"] = {
            "motion_ok": motion_ok,
            "connection_ok": connection_ok,
            "final_disabled_verified": final_disabled_ok,
            "alerts_clear": alerts_clear,
            "motor_faulted_observations": len(motor_faulted_events),
            "alert_observations": len(self._alert_events),
            "alert_observations_note": (
                "Counts are log rows, not unique fault events; "
                "pre/pre_clear may duplicate one latch; tail may resample it."
            ),
        }
        # Overall pass requires motion + connection + verified final disabled.
        # Alerts (e.g. inter-cycle motor_faulted) are reported separately and
        # do not by themselves fail motion endurance.
        ok = motion_ok and connection_ok and final_disabled_ok
        self.summary["ok"] = ok
        self.summary["ok_includes_alerts_clear"] = False
        (self.log_dir / "summary.json").write_text(
            json.dumps(self.summary, indent=2), encoding="utf-8"
        )

        rev = self.summary.get("revisions") or {}
        md = [
            "# ClearCoreROS 30-minute ROS hardware battery",
            "",
            f"- Host: `{HOST}`",
            f"- Log dir: `{self.log_dir}`",
            f"- Duration: {self.summary['duration_s_actual']} s (target {DURATION_S})",
            f"- Motion/connection/final-disabled: **{'PASS' if ok else 'FAIL'}**",
            f"- Alerts clear at finalize: **{'yes' if alerts_clear else 'no'}** "
            f"(reported separately; not required for motion PASS)",
            "",
            "## Verdicts",
            "",
            f"- motion_ok: `{motion_ok}`",
            f"- connection_ok: `{connection_ok}`",
            f"- final_disabled_verified: `{final_disabled_ok}`",
            f"- alerts_clear: `{alerts_clear}`",
            f"- motor_faulted_observations: `{len(motor_faulted_events)}` "
            f"(rows, not necessarily distinct faults)",
            "",
            "## Revisions",
            "",
            f"- git HEAD: `{rev.get('git_head')}`",
            f"- discover: `{rev.get('discover')}`",
            f"- host packages: clearcore_bridge/hardware `{rev.get('host_packages')}`",
            f"- NVM: `{rev.get('get_config_nvm')}`",
            "",
            "## Counts",
            "",
        ]
        for key, value in self.summary["counts"].items():
            md.append(f"- {key}: {value}")
        md.extend(
            [
                "",
                "## Per-cycle drift / tracking (generated-step units, not shaft encoder)",
                "",
                "| cycle | ok | bridge peak\\|Y-X\\| mm | JTC reported end mm | origin return err mm | gaps>150ms | reconnects |",
                "|------:|:--:|----------------------:|--------------------:|---------------------:|-----------:|-----------:|",
            ]
        )
        for cyc in self.summary["cycles"]:
            m = cyc.get("metrics") or {}
            jtc = m.get("jtc_reported_endpoint") or {}
            origin = m.get("origin_return") or {}
            gaps = int((m.get("bridge_state_gaps") or {}).get("gaps_over_150ms") or 0) + int(
                (m.get("jtc_state_gaps") or {}).get("gaps_over_150ms") or 0
            )
            end = ""
            if jtc.get("reported_end_x_mm") is not None:
                end = f"({jtc.get('reported_end_x_mm')}, {jtc.get('reported_end_y_mm')})"
            md.append(
                f"| {cyc['cycle']} | {cyc.get('ok')} | {m.get('bridge_peak_abs_yx_mm')} | "
                f"{end} | {origin.get('err_mm')} | {gaps} | {cyc.get('reconnects_this_cycle')} |"
            )
        md.extend(
            [
                "",
                "JTC reported endpoint is listed separately from origin return error; "
                "do not treat the JTC end_mm (e.g. −0.14 mm) as the origin check.",
                "",
                "## Final disabled state",
                "",
            ]
        )
        for key, value in post["disabled_state"].items():
            md.append(f"- {key}: `{value}`")
        md.extend(["", "## Final checks", ""])
        for key, value in post["checks"].items():
            md.append(f"- {key}: {value}")
        if self.summary["failures"]:
            md.extend(["", "## Failures", ""])
            for item in self.summary["failures"]:
                md.append(f"- `{json.dumps(item)}`")
        if self._watchdog_events:
            md.extend(["", f"## Watchdog events ({len(self._watchdog_events)})", ""])
            for item in self._watchdog_events[:20]:
                md.append(f"- t={item['t_s']:.1f}s where={item['where']}")
        if self._alert_events:
            md.extend(["", f"## Alert events ({len(self._alert_events)})", ""])
            for item in self._alert_events[:20]:
                md.append(f"- t={item['t_s']:.1f}s where={item['where']} alerts={item['alerts']}")
        (self.log_dir / "summary.md").write_text("\n".join(md) + "\n", encoding="utf-8")
        self.log("BATTERY_DONE", ok=ok, log_dir=str(self.log_dir))
        print(f"SUMMARY {self.log_dir / 'summary.md'}", flush=True)
        print(
            "ROS_HARDWARE_BATTERY_30M_OK" if ok else "ROS_HARDWARE_BATTERY_30M_FAILED",
            flush=True,
        )
        return 0 if ok else 1

    def run(self) -> int:
        self.log(
            "battery start",
            host=HOST,
            duration_s=DURATION_S,
            ws=str(WS),
            log_dir=str(self.log_dir),
        )
        self.collect_revisions()
        self.dump_board("start")
        cycle = 0
        try:
            while self.remaining() > 120 and not self._stop:
                cycle += 1
                try:
                    result = self.run_cycle(cycle)
                except Exception as exc:  # noqa: BLE001
                    self.log("cycle crashed", cycle=cycle, error=str(exc))
                    result = {
                        "cycle": cycle,
                        "ok": False,
                        "error": str(exc),
                        "t_start_s": round(self.elapsed(), 3),
                        "t_end_s": round(self.elapsed(), 3),
                        "steps": {},
                        "metrics": {},
                    }
                    self.summary["counts"]["cycles_started"] += 1
                    self.summary["counts"]["cycles_completed"] += 1
                    self.summary["counts"]["cycles_failed"] += 1
                    self.summary["failures"].append(
                        {"t_s": self.elapsed(), "step": f"cycle_{cycle}", "error": str(exc)}
                    )
                    # Board may need time after a TCP wedge.
                    time.sleep(5.0)
                self.summary["cycles"].append(result)
                # Cool-down between cycles so HLFB/alerts can settle.
                for _ in range(15):
                    if self.remaining() <= 0 or self._stop:
                        break
                    time.sleep(1.0)
            while self.remaining() > 5 and not self._stop:
                self.dump_board("tail")
                time.sleep(min(10.0, max(1.0, self.remaining())))
        finally:
            rc = self.finalize()
        return rc


def main() -> int:
    return Battery().run()


if __name__ == "__main__":
    sys.exit(main())
