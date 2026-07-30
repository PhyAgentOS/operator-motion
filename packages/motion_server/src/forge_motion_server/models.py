from __future__ import annotations

from dataclasses import dataclass
from typing import Literal

from forge_msgs import (
    FollowJointTrajectoryGoal,
    MoveJointsFeedback,
    MoveJointsResult,
    MovePoseFeedback,
    MovePoseResult,
)

GoalKind = Literal["move_joints", "move_pose"]
GoalStatus = Literal["succeeded", "aborted", "canceled"]


@dataclass(frozen=True, slots=True)
class MotionServerOutput:
    parent_kind: GoalKind | None = None
    parent_goal_id: str | None = None
    parent_goal_status: GoalStatus | None = None
    move_joints_feedback: MoveJointsFeedback | None = None
    move_joints_result: MoveJointsResult | None = None
    move_pose_feedback: MovePoseFeedback | None = None
    move_pose_result: MovePoseResult | None = None
    child_goal_id: str | None = None
    child_goal: FollowJointTrajectoryGoal | None = None
    child_cancel_goal_id: str | None = None
