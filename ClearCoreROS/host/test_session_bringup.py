"""SessionClient + bring_up_session against a simulated JSON-RPC board."""

from __future__ import annotations

import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "ros2_ws" / "src" / "clearcore_bridge"))

from clearcore_bridge.session_bringup import bring_up_session, recover_watchdog  # noqa: E402
from clearcore_bridge.sim_board import JsonRpcBoard  # noqa: E402
from clearcore_bridge.wire import SessionClient  # noqa: E402

BRING_UP = (
    "disable",
    "clear_alerts",
    "configure",
    "set_test_mode",
    "get_config",
    "enable",
)


def _names():
    return {
        "get_config": {
            "names": ["joint_x", "joint_y", "joint_z", "joint_a"],
            "rotary": [False, False, False, True],
        }
    }


def test_home_is_session_call():
    board = JsonRpcBoard(_names())
    try:
        session = SessionClient("127.0.0.1", board.port, 2.0)
        try:
            bring_up_session(session, axis_mask=3)
            session.call("home", {"axis": "x", "dir": "neg"})
        finally:
            session.close()
        assert board.methods[-1] == "home"
        home_req = next(r for r in board.requests if r["method"] == "home")
        assert home_req["params"]["axis"] == "x"
        assert home_req["params"]["dir"] == "neg"
        assert "follow_joint_trajectory" not in board.methods
    finally:
        board.close()


def test_reconnect_repeats_bring_up():
    board = JsonRpcBoard(_names())
    try:
        first = SessionClient("127.0.0.1", board.port, 2.0)
        bring_up_session(first, axis_mask=3)
        first.close()
        second = SessionClient("127.0.0.1", board.port, 2.0)
        bring_up_session(second, axis_mask=3, test_mode=False)
        second.close()
        assert board.methods == list(BRING_UP) + list(BRING_UP)
        assert "keepalive" not in board.methods
        assert "home" not in board.methods
    finally:
        board.close()


def test_recovery_sends_clear_alerts_not_keepalive():
    board = JsonRpcBoard(_names())
    try:
        session = SessionClient("127.0.0.1", board.port, 2.0)
        try:
            bring_up_session(session, axis_mask=3)
            session.call("keepalive")
            recover_watchdog(session)
        finally:
            session.close()
        assert board.methods[:6] == list(BRING_UP)
        assert "keepalive" in board.methods
        assert board.methods[-1] == "clear_alerts"
        assert board.methods.count("clear_alerts") == 2
        assert board.methods.index("keepalive") > board.methods.index("enable")
    finally:
        board.close()


if __name__ == "__main__":
    test_home_is_session_call()
    test_reconnect_repeats_bring_up()
    test_recovery_sends_clear_alerts_not_keepalive()
    print("session_bringup ok")
