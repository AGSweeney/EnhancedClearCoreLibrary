"""JTC result predicate used by send_trajectory_goal.py.

STATUS_SUCCEEDED is action_msgs/GoalStatus. RESULT_SUCCESSFUL is
control_msgs/FollowJointTrajectory.Result. A canceled goal can still carry
error_code 0; status must be SUCCEEDED.
"""

STATUS_SUCCEEDED = 4
STATUS_CANCELED = 5
STATUS_ABORTED = 6
RESULT_SUCCESSFUL = 0


def goal_succeeded(status: int, error_code: int) -> bool:
    return status == STATUS_SUCCEEDED and error_code == RESULT_SUCCESSFUL
