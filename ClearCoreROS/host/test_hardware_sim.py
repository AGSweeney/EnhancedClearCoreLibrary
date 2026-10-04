"""Import the production JTC success predicate. Plugin behavior is the C++ gtest."""

from __future__ import annotations

import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
SCRIPTS = ROOT / "ros2_ws" / "src" / "clearcore_hardware" / "scripts"
sys.path.insert(0, str(SCRIPTS))

from jtc_result import (  # noqa: E402
    RESULT_SUCCESSFUL,
    STATUS_ABORTED,
    STATUS_CANCELED,
    STATUS_SUCCEEDED,
    goal_succeeded,
)


def test_canceled_jtc_result_is_failure_even_if_error_code_is_zero():
    assert goal_succeeded(STATUS_SUCCEEDED, RESULT_SUCCESSFUL) is True
    assert goal_succeeded(STATUS_CANCELED, RESULT_SUCCESSFUL) is False
    assert goal_succeeded(STATUS_ABORTED, RESULT_SUCCESSFUL) is False


if __name__ == "__main__":
    test_canceled_jtc_result_is_failure_even_if_error_code_is_zero()
    print("hardware_sim ok")
