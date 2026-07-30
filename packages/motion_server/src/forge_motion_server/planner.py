from __future__ import annotations

from dataclasses import dataclass
from typing import Protocol

from forge_msgs import MoveJointsGoal, MovePoseGoal, Pose

from .config import MotionGroupConfig, MotionServerConfig
from .kinematics import MotionKinematics
from .state_cache import JointStateSnapshot
from .trajectory import PlannedTrajectory, TrajectoryGenerationError, plan_quintic_trajectory


class PlanningFailure(RuntimeError):
    def __init__(self, error_code: str, message: str) -> None:
        super().__init__(message)
        self.error_code = error_code


@dataclass(frozen=True, slots=True)
class PlannedMotion:
    target_positions: tuple[float, ...]
    target_pose: Pose | None
    trajectory: PlannedTrajectory


class MotionPlanner(Protocol):
    def plan_joints(
        self,
        group: MotionGroupConfig,
        state: JointStateSnapshot,
        goal: MoveJointsGoal,
    ) -> PlannedMotion: ...

    def plan_pose(
        self,
        group: MotionGroupConfig,
        state: JointStateSnapshot,
        goal: MovePoseGoal,
    ) -> PlannedMotion: ...


class KinematicPlanner:
    """Deterministic IK plus synchronized quintic joint-space planning."""

    def __init__(self, config: MotionServerConfig, kinematics: MotionKinematics) -> None:
        self.config = config
        self.kinematics = kinematics

    def plan_joints(
        self,
        group: MotionGroupConfig,
        state: JointStateSnapshot,
        goal: MoveJointsGoal,
    ) -> PlannedMotion:
        target = tuple(goal.positions)
        try:
            self.kinematics.validate_positions(group.name, target)
        except (TypeError, ValueError) as error:
            raise PlanningFailure("JOINT_LIMIT_VIOLATION", str(error)) from error
        return PlannedMotion(
            target_positions=target,
            target_pose=None,
            trajectory=self._trajectory(group, state, target, goal),
        )

    def plan_pose(
        self,
        group: MotionGroupConfig,
        state: JointStateSnapshot,
        goal: MovePoseGoal,
    ) -> PlannedMotion:
        position_tolerance = (
            group.final_position_tolerance_m
            if goal.position_tolerance_m is None
            else goal.position_tolerance_m
        )
        orientation_tolerance = (
            group.final_orientation_tolerance_rad
            if goal.orientation_tolerance_rad is None
            else goal.orientation_tolerance_rad
        )
        try:
            outcome = self.kinematics.inverse(
                group.name,
                goal.target_pose,
                state.positions,
                self.config.planning_timeout_ns,
                position_tolerance,
                orientation_tolerance,
            )
        except (FloatingPointError, TypeError, ValueError) as error:
            raise PlanningFailure("IK_FAILED", str(error)) from error
        if outcome.positions is None:
            code = "IK_TIMED_OUT" if outcome.status == "TIMED_OUT" else "IK_FAILED"
            raise PlanningFailure(code, f"{outcome.status}: {outcome.message}")
        try:
            self.kinematics.validate_positions(group.name, outcome.positions)
        except (TypeError, ValueError) as error:
            raise PlanningFailure("JOINT_LIMIT_VIOLATION", str(error)) from error
        return PlannedMotion(
            target_positions=outcome.positions,
            target_pose=goal.target_pose,
            trajectory=self._trajectory(group, state, outcome.positions, goal),
        )

    @staticmethod
    def _trajectory(
        group: MotionGroupConfig,
        state: JointStateSnapshot,
        target: tuple[float, ...],
        goal: MoveJointsGoal | MovePoseGoal,
    ) -> PlannedTrajectory:
        try:
            return plan_quintic_trajectory(
                joint_names=group.joint_names,
                start_positions=state.positions,
                target_positions=target,
                max_velocity=group.max_velocity,
                max_acceleration=group.max_acceleration,
                velocity_scale=goal.velocity_scale,
                acceleration_scale=goal.acceleration_scale,
                sample_period_ns=group.trajectory_sample_period_ns,
                requested_duration_ns=goal.requested_duration_ns,
                max_points=group.max_trajectory_points,
            )
        except TrajectoryGenerationError as error:
            raise PlanningFailure("TRAJECTORY_GENERATION_FAILED", str(error)) from error
