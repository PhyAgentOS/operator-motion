from __future__ import annotations

from collections import OrderedDict
from dataclasses import dataclass
from typing import Literal

from forge_msgs import (
    FollowJointTrajectoryFeedback,
    FollowJointTrajectoryGoal,
    FollowJointTrajectoryResult,
    JointState,
    MoveJointsFeedback,
    MoveJointsGoal,
    MoveJointsResult,
    MovePoseFeedback,
    MovePoseGoal,
    MovePoseResult,
    Pose,
)

from .config import MotionGroupConfig, MotionServerConfig
from .kinematics import MotionKinematics, pose_error, pose_to_matrix
from .models import GoalKind, GoalStatus, MotionServerOutput
from .planner import KinematicPlanner, MotionPlanner, PlannedMotion, PlanningFailure
from .state_cache import JointStateCache, JointStateSnapshot

ExecutionState = Literal["EXECUTING", "CANCELING", "SETTLING"]


@dataclass(slots=True)
class _ActiveGoal:
    kind: GoalKind
    goal_id: str
    goal: MoveJointsGoal | MovePoseGoal
    group: MotionGroupConfig
    target_positions: tuple[float, ...]
    target_pose: Pose | None
    child_goal_id: str
    started_ns: int
    trajectory_duration_ns: int
    state: ExecutionState = "EXECUTING"
    cancel_requested_ns: int | None = None
    last_cancel_sent_ns: int | None = None
    cancel_error_code: str | None = None
    cancel_message: str | None = None
    cancel_goal_status: GoalStatus | None = None
    settling_started_ns: int | None = None


class MotionServer:
    """Deterministic MoveJoints/MovePose orchestration core without Dora dependencies."""

    def __init__(
        self,
        config: MotionServerConfig,
        kinematics: MotionKinematics,
        planner: MotionPlanner | None = None,
    ) -> None:
        self.config = config
        self.kinematics = kinematics
        self.planner = planner or KinematicPlanner(config, kinematics)
        self.state_cache = JointStateCache()
        self._active: _ActiveGoal | None = None
        self._result_cache: OrderedDict[tuple[GoalKind, str], MotionServerOutput] = OrderedDict()
        self._last_now_ns: int | None = None
        self._child_sequence = 0

    @property
    def active_goal_id(self) -> str | None:
        return None if self._active is None else self._active.goal_id

    @property
    def active_child_goal_id(self) -> str | None:
        return None if self._active is None else self._active.child_goal_id

    def update_joint_state(self, state: JointState, now_ns: int) -> None:
        self._validate_now(now_ns)
        self.state_cache.update(state, now_ns)

    def accept_move_joints(
        self, goal_id: str, goal: MoveJointsGoal, now_ns: int
    ) -> MotionServerOutput | None:
        self._validate_goal_id(goal_id)
        self._validate_now(now_ns)
        handled, response = self._existing_goal_response("move_joints", goal_id, goal)
        if handled:
            return response
        group = self.config.groups.get(goal.group_name)
        if group is None:
            return self._reject_joints(
                goal_id, "INVALID_GROUP", f"unknown group {goal.group_name!r}"
            )
        if tuple(goal.joint_names) != group.joint_names:
            return self._reject_joints(
                goal_id,
                "INVALID_JOINTS",
                "joint_names must exactly match the configured complete group order",
            )
        snapshot = self._fresh_snapshot(group, now_ns)
        if snapshot is None:
            return self._reject_joints(
                goal_id,
                "NO_FRESH_ROBOT_STATE",
                "fresh complete JointState feedback is required",
            )
        moving = self._moving_joints(group, snapshot)
        if moving:
            return self._reject_joints(
                goal_id,
                "PLANNING_FAILED",
                f"current state exceeds start velocity tolerance for joints: {moving}",
            )
        try:
            planned = self.planner.plan_joints(group, snapshot, goal)
        except PlanningFailure as error:
            return self._reject_joints(goal_id, error.error_code, str(error))
        return self._start_execution(
            kind="move_joints",
            goal_id=goal_id,
            goal=goal,
            group=group,
            snapshot=snapshot,
            planned=planned,
            now_ns=now_ns,
        )

    def accept_move_pose(
        self, goal_id: str, goal: MovePoseGoal, now_ns: int
    ) -> MotionServerOutput | None:
        self._validate_goal_id(goal_id)
        self._validate_now(now_ns)
        handled, response = self._existing_goal_response("move_pose", goal_id, goal)
        if handled:
            return response
        group = self.config.groups.get(goal.group_name)
        if group is None:
            return self._reject_pose(goal_id, "INVALID_GROUP", f"unknown group {goal.group_name!r}")
        if goal.reference_frame != group.base_frame or goal.target_frame != group.tip_frame:
            return self._reject_pose(
                goal_id,
                "INVALID_FRAME",
                f"expected {group.base_frame!r} -> {group.tip_frame!r}",
            )
        if goal.position_tolerance_m == 0.0 or goal.orientation_tolerance_rad == 0.0:
            return self._reject_pose(
                goal_id,
                "INVALID_GOAL",
                "Cartesian position and orientation tolerances must be positive",
            )
        try:
            pose_to_matrix(goal.target_pose)
        except (TypeError, ValueError) as error:
            return self._reject_pose(goal_id, "INVALID_GOAL", str(error))
        snapshot = self._fresh_snapshot(group, now_ns)
        if snapshot is None:
            return self._reject_pose(
                goal_id,
                "NO_FRESH_ROBOT_STATE",
                "fresh complete JointState feedback is required",
            )
        moving = self._moving_joints(group, snapshot)
        if moving:
            return self._reject_pose(
                goal_id,
                "PLANNING_FAILED",
                f"current state exceeds start velocity tolerance for joints: {moving}",
            )
        try:
            planned = self.planner.plan_pose(group, snapshot, goal)
        except PlanningFailure as error:
            return self._reject_pose(goal_id, error.error_code, str(error))
        return self._start_execution(
            kind="move_pose",
            goal_id=goal_id,
            goal=goal,
            group=group,
            snapshot=snapshot,
            planned=planned,
            now_ns=now_ns,
        )

    def reject_invalid_goal(
        self, kind: GoalKind, goal_id: str, message: str, now_ns: int
    ) -> MotionServerOutput | None:
        self._validate_goal_id(goal_id)
        self._validate_now(now_ns)
        cached = self._cached(kind, goal_id)
        if cached is not None:
            return cached
        if self._active is not None and self._active.goal_id == goal_id:
            return None
        if kind == "move_joints":
            return self._reject_joints(goal_id, "INVALID_GOAL", message)
        return self._reject_pose(goal_id, "INVALID_GOAL", message)

    def cancel_goal(self, kind: GoalKind, goal_id: str, now_ns: int) -> MotionServerOutput | None:
        self._validate_goal_id(goal_id)
        self._validate_now(now_ns)
        cached = self._cached(kind, goal_id)
        if cached is not None:
            return cached
        active = self._active
        if active is None or active.kind != kind or active.goal_id != goal_id:
            return None
        if active.state == "CANCELING":
            return None
        if active.state == "SETTLING":
            snapshot = self._fresh_snapshot(active.group, now_ns)
            return self._finish(
                active,
                "CANCELED",
                "goal canceled while validating final state",
                "canceled",
                now_ns,
                snapshot=snapshot,
            )
        return self._begin_child_cancel(
            active,
            now_ns,
            error_code="CANCELED",
            message="goal canceled by client",
            goal_status="canceled",
        )

    def handle_trajectory_feedback(
        self,
        child_goal_id: str,
        feedback: FollowJointTrajectoryFeedback,
        now_ns: int,
    ) -> MotionServerOutput | None:
        self._validate_now(now_ns)
        active = self._active
        if active is None or active.child_goal_id != child_goal_id:
            return None
        if active.state != "EXECUTING":
            return None
        snapshot = self._fresh_snapshot(active.group, now_ns)
        elapsed_ns = max(0, now_ns - active.started_ns)
        progress = min(1.0, feedback.elapsed_ns / max(1, feedback.duration_ns))
        if active.kind == "move_joints":
            actual = tuple(feedback.actual.positions)
            return MotionServerOutput(
                parent_kind=active.kind,
                parent_goal_id=active.goal_id,
                move_joints_feedback=self._joints_feedback(
                    active,
                    actual,
                    elapsed_ns,
                    phase="EXECUTING",
                    progress=progress,
                    message="controller is executing trajectory",
                ),
            )
        actual_pose = None
        position_residual = None
        orientation_residual = None
        if snapshot is not None and active.target_pose is not None:
            try:
                actual_pose = self.kinematics.forward(active.group.name, snapshot.positions)
                position_residual, orientation_residual = pose_error(
                    actual_pose, active.target_pose
                )
            except (FloatingPointError, TypeError, ValueError):
                actual_pose = None
        return MotionServerOutput(
            parent_kind=active.kind,
            parent_goal_id=active.goal_id,
            move_pose_feedback=MovePoseFeedback(
                phase="EXECUTING",
                progress=progress,
                elapsed_ns=elapsed_ns,
                estimated_duration_ns=active.trajectory_duration_ns,
                actual_pose=actual_pose,
                position_error_m=position_residual,
                orientation_error_rad=orientation_residual,
                message="controller is executing trajectory",
            ),
        )

    def handle_trajectory_result(
        self,
        child_goal_id: str,
        child_status: GoalStatus,
        result: FollowJointTrajectoryResult,
        now_ns: int,
    ) -> MotionServerOutput | None:
        self._validate_now(now_ns)
        active = self._active
        if active is None or active.child_goal_id != child_goal_id:
            return None
        if child_status == "canceled" and result.error_code == "CANCELED":
            if active.state == "CANCELING":
                assert active.cancel_error_code is not None
                assert active.cancel_message is not None
                assert active.cancel_goal_status is not None
                snapshot = self._fresh_snapshot(active.group, now_ns)
                return self._finish(
                    active,
                    active.cancel_error_code,
                    active.cancel_message,
                    active.cancel_goal_status,
                    now_ns,
                    snapshot=snapshot,
                )
            return self._finish(
                active,
                "TRAJECTORY_EXECUTION_FAILED",
                "child trajectory canceled without a parent cancel request",
                "aborted",
                now_ns,
            )
        if child_status != "succeeded" or result.error_code != "SUCCESS":
            rejected_codes = {"BUSY", "INVALID_GOAL", "INVALID_JOINTS", "START_STATE_MISMATCH"}
            code = (
                "TRAJECTORY_REJECTED"
                if result.error_code in rejected_codes
                else "TRAJECTORY_EXECUTION_FAILED"
            )
            return self._finish(
                active,
                code,
                f"child trajectory {result.error_code}: {result.message}",
                "aborted",
                now_ns,
            )
        if active.state == "CANCELING" and active.cancel_goal_status == "aborted":
            assert active.cancel_error_code is not None
            assert active.cancel_message is not None
            snapshot = self._fresh_snapshot(active.group, now_ns)
            return self._finish(
                active,
                active.cancel_error_code,
                active.cancel_message,
                "aborted",
                now_ns,
                snapshot=snapshot,
            )
        active.state = "SETTLING"
        active.settling_started_ns = now_ns
        completed = self._try_complete_success(active, now_ns, final_attempt=False)
        return completed if completed is not None else self._settling_feedback(active, now_ns)

    def tick(self, now_ns: int) -> MotionServerOutput | None:
        self._validate_now(now_ns)
        active = self._active
        if active is None:
            return None
        if active.state == "EXECUTING":
            deadline_ns = (
                active.started_ns
                + active.trajectory_duration_ns
                + self.config.execution_timeout_margin_ns
            )
            if now_ns > deadline_ns:
                return self._begin_child_cancel(
                    active,
                    now_ns,
                    error_code="TRAJECTORY_EXECUTION_FAILED",
                    message="child trajectory execution timed out",
                    goal_status="aborted",
                )
            return None
        if active.state == "CANCELING":
            assert active.cancel_requested_ns is not None
            if now_ns - active.cancel_requested_ns > self.config.child_cancel_timeout_ns:
                return self._finish(
                    active,
                    "TRAJECTORY_EXECUTION_FAILED",
                    "timed out waiting for child trajectory cancellation",
                    "aborted",
                    now_ns,
                )
            assert active.last_cancel_sent_ns is not None
            if now_ns - active.last_cancel_sent_ns >= self.config.child_cancel_retry_ns:
                active.last_cancel_sent_ns = now_ns
                return MotionServerOutput(child_cancel_goal_id=active.child_goal_id)
            return None
        assert active.settling_started_ns is not None
        final_attempt = now_ns - active.settling_started_ns >= self.config.final_state_timeout_ns
        return self._try_complete_success(active, now_ns, final_attempt=final_attempt)

    def _begin_child_cancel(
        self,
        active: _ActiveGoal,
        now_ns: int,
        *,
        error_code: str,
        message: str,
        goal_status: GoalStatus,
    ) -> MotionServerOutput:
        active.state = "CANCELING"
        active.cancel_requested_ns = now_ns
        active.last_cancel_sent_ns = now_ns
        active.cancel_error_code = error_code
        active.cancel_message = message
        active.cancel_goal_status = goal_status
        return MotionServerOutput(child_cancel_goal_id=active.child_goal_id)

    def _try_complete_success(
        self,
        active: _ActiveGoal,
        now_ns: int,
        *,
        final_attempt: bool,
    ) -> MotionServerOutput | None:
        snapshot = self._fresh_snapshot(active.group, now_ns)
        if snapshot is None:
            if not final_attempt:
                return None
            return self._finish(
                active,
                "TRAJECTORY_EXECUTION_FAILED",
                "child succeeded but no fresh final JointState arrived",
                "aborted",
                now_ns,
            )
        errors = tuple(
            target - actual
            for target, actual in zip(active.target_positions, snapshot.positions, strict=True)
        )
        if active.kind == "move_joints":
            if all(abs(error) <= active.group.final_joint_tolerance for error in errors):
                return self._finish(
                    active,
                    "SUCCESS",
                    "joint target reached",
                    "succeeded",
                    now_ns,
                    snapshot=snapshot,
                )
            if not final_attempt:
                return None
            return self._finish(
                active,
                "FINAL_JOINT_TOLERANCE_VIOLATED",
                "final joint position tolerance was violated",
                "aborted",
                now_ns,
                snapshot=snapshot,
            )
        assert active.target_pose is not None
        try:
            actual_pose = self.kinematics.forward(active.group.name, snapshot.positions)
            position_residual, orientation_residual = pose_error(actual_pose, active.target_pose)
        except (FloatingPointError, TypeError, ValueError) as error:
            return self._finish(
                active,
                "INTERNAL_ERROR",
                f"failed to evaluate final FK: {error}",
                "aborted",
                now_ns,
                snapshot=snapshot,
            )
        pose_goal = active.goal
        assert isinstance(pose_goal, MovePoseGoal)
        position_tolerance = (
            active.group.final_position_tolerance_m
            if pose_goal.position_tolerance_m is None
            else pose_goal.position_tolerance_m
        )
        orientation_tolerance = (
            active.group.final_orientation_tolerance_rad
            if pose_goal.orientation_tolerance_rad is None
            else pose_goal.orientation_tolerance_rad
        )
        residual = (position_residual, orientation_residual)
        if (
            position_residual <= position_tolerance
            and orientation_residual <= orientation_tolerance
        ):
            return self._finish(
                active,
                "SUCCESS",
                "pose target reached",
                "succeeded",
                now_ns,
                snapshot=snapshot,
                final_pose=actual_pose,
                pose_residual=residual,
            )
        if not final_attempt:
            return None
        return self._finish(
            active,
            "FINAL_POSE_TOLERANCE_VIOLATED",
            "final Cartesian pose tolerance was violated",
            "aborted",
            now_ns,
            snapshot=snapshot,
            final_pose=actual_pose,
            pose_residual=residual,
        )

    def _settling_feedback(self, active: _ActiveGoal, now_ns: int) -> MotionServerOutput:
        snapshot = self._fresh_snapshot(active.group, now_ns)
        elapsed_ns = max(0, now_ns - active.started_ns)
        if active.kind == "move_joints":
            actual = [] if snapshot is None else list(snapshot.positions)
            errors = (
                []
                if snapshot is None
                else [
                    target - value
                    for target, value in zip(
                        active.target_positions, snapshot.positions, strict=True
                    )
                ]
            )
            return MotionServerOutput(
                parent_kind=active.kind,
                parent_goal_id=active.goal_id,
                move_joints_feedback=MoveJointsFeedback(
                    phase="SETTLING",
                    progress=1.0,
                    elapsed_ns=elapsed_ns,
                    estimated_duration_ns=active.trajectory_duration_ns,
                    joint_names=list(active.group.joint_names),
                    actual_positions=actual,
                    target_positions=list(active.target_positions),
                    position_errors=errors,
                    message="waiting for final robot state",
                ),
            )
        actual_pose = None
        position_residual = None
        orientation_residual = None
        if snapshot is not None and active.target_pose is not None:
            try:
                actual_pose = self.kinematics.forward(active.group.name, snapshot.positions)
                position_residual, orientation_residual = pose_error(
                    actual_pose, active.target_pose
                )
            except (FloatingPointError, TypeError, ValueError):
                pass
        return MotionServerOutput(
            parent_kind=active.kind,
            parent_goal_id=active.goal_id,
            move_pose_feedback=MovePoseFeedback(
                phase="SETTLING",
                progress=1.0,
                elapsed_ns=elapsed_ns,
                estimated_duration_ns=active.trajectory_duration_ns,
                actual_pose=actual_pose,
                position_error_m=position_residual,
                orientation_error_rad=orientation_residual,
                message="waiting for final robot state",
            ),
        )

    def _start_execution(
        self,
        *,
        kind: GoalKind,
        goal_id: str,
        goal: MoveJointsGoal | MovePoseGoal,
        group: MotionGroupConfig,
        snapshot: JointStateSnapshot,
        planned: PlannedMotion,
        now_ns: int,
    ) -> MotionServerOutput:
        self._child_sequence += 1
        child_goal_id = f"{goal_id}/trajectory/{self._child_sequence}"
        active = _ActiveGoal(
            kind=kind,
            goal_id=goal_id,
            goal=goal,
            group=group,
            target_positions=planned.target_positions,
            target_pose=planned.target_pose,
            child_goal_id=child_goal_id,
            started_ns=now_ns,
            trajectory_duration_ns=planned.trajectory.duration_ns,
        )
        self._active = active
        child_goal = FollowJointTrajectoryGoal(
            trajectory=planned.trajectory.trajectory,
            path_tolerance=[],
            goal_tolerance=[],
            goal_time_tolerance_ns=None,
        )
        if kind == "move_joints":
            return MotionServerOutput(
                parent_kind=kind,
                parent_goal_id=goal_id,
                child_goal_id=child_goal_id,
                child_goal=child_goal,
                move_joints_feedback=self._joints_feedback(
                    active,
                    snapshot.positions,
                    0,
                    phase="WAITING_FOR_CONTROLLER",
                    progress=0.0,
                    message="trajectory generated and sent to controller",
                ),
            )
        return MotionServerOutput(
            parent_kind=kind,
            parent_goal_id=goal_id,
            child_goal_id=child_goal_id,
            child_goal=child_goal,
            move_pose_feedback=MovePoseFeedback(
                phase="WAITING_FOR_CONTROLLER",
                progress=0.0,
                elapsed_ns=0,
                estimated_duration_ns=planned.trajectory.duration_ns,
                actual_pose=None,
                position_error_m=None,
                orientation_error_rad=None,
                message="IK solved; trajectory sent to controller",
            ),
        )

    def _finish(
        self,
        active: _ActiveGoal,
        error_code: str,
        message: str,
        goal_status: GoalStatus,
        now_ns: int,
        *,
        snapshot: JointStateSnapshot | None = None,
        final_pose: Pose | None = None,
        pose_residual: tuple[float, float] | None = None,
    ) -> MotionServerOutput:
        elapsed_ns = max(0, now_ns - active.started_ns)
        if active.kind == "move_joints":
            positions = [] if snapshot is None else list(snapshot.positions)
            errors = (
                []
                if snapshot is None
                else [
                    target - actual
                    for target, actual in zip(
                        active.target_positions, snapshot.positions, strict=True
                    )
                ]
            )
            output = MotionServerOutput(
                parent_kind=active.kind,
                parent_goal_id=active.goal_id,
                parent_goal_status=goal_status,
                move_joints_result=MoveJointsResult(
                    error_code=error_code,  # type: ignore[arg-type]
                    message=message,
                    elapsed_ns=elapsed_ns,
                    joint_names=list(active.group.joint_names) if snapshot is not None else [],
                    final_positions=positions,
                    final_position_errors=errors,
                ),
            )
        else:
            position_error = None if pose_residual is None else pose_residual[0]
            orientation_error = None if pose_residual is None else pose_residual[1]
            output = MotionServerOutput(
                parent_kind=active.kind,
                parent_goal_id=active.goal_id,
                parent_goal_status=goal_status,
                move_pose_result=MovePoseResult(
                    error_code=error_code,  # type: ignore[arg-type]
                    message=message,
                    elapsed_ns=elapsed_ns,
                    final_pose=final_pose,
                    final_position_error_m=position_error,
                    final_orientation_error_rad=orientation_error,
                    joint_names=list(active.group.joint_names) if snapshot is not None else [],
                    final_joint_positions=([] if snapshot is None else list(snapshot.positions)),
                ),
            )
        self._active = None
        self._cache(output)
        return output

    def _joints_feedback(
        self,
        active: _ActiveGoal,
        actual_positions: tuple[float, ...],
        elapsed_ns: int,
        *,
        phase: str,
        progress: float,
        message: str,
    ) -> MoveJointsFeedback:
        return MoveJointsFeedback(
            phase=phase,  # type: ignore[arg-type]
            progress=progress,
            elapsed_ns=elapsed_ns,
            estimated_duration_ns=active.trajectory_duration_ns,
            joint_names=list(active.group.joint_names),
            actual_positions=list(actual_positions),
            target_positions=list(active.target_positions),
            position_errors=[
                target - actual
                for target, actual in zip(active.target_positions, actual_positions, strict=True)
            ],
            message=message,
        )

    def _fresh_snapshot(self, group: MotionGroupConfig, now_ns: int) -> JointStateSnapshot | None:
        try:
            return self.state_cache.snapshot(
                group.joint_names, now_ns, self.config.state_timeout_ns
            )
        except ValueError:
            return None

    @staticmethod
    def _moving_joints(group: MotionGroupConfig, snapshot: JointStateSnapshot) -> list[str]:
        return [
            name
            for name, velocity in zip(group.joint_names, snapshot.velocities, strict=True)
            if abs(velocity) > group.start_velocity_tolerance
        ]

    def _existing_goal_response(
        self,
        kind: GoalKind,
        goal_id: str,
        goal: MoveJointsGoal | MovePoseGoal,
    ) -> tuple[bool, MotionServerOutput | None]:
        cached = self._cached(kind, goal_id)
        if cached is not None:
            return True, cached
        if self._active is None:
            return False, None
        if self._active.goal_id == goal_id:
            # Active duplicates and same-ID protocol conflicts never restart or terminate the goal.
            return True, None
        if kind == "move_joints":
            response = self._reject_joints(
                goal_id, "BUSY", "motion server already has an active goal"
            )
        else:
            response = self._reject_pose(
                goal_id, "BUSY", "motion server already has an active goal"
            )
        return True, response

    def _reject_joints(self, goal_id: str, code: str, message: str) -> MotionServerOutput:
        output = MotionServerOutput(
            parent_kind="move_joints",
            parent_goal_id=goal_id,
            parent_goal_status="aborted",
            move_joints_result=MoveJointsResult(
                error_code=code,  # type: ignore[arg-type]
                message=message,
                elapsed_ns=0,
                joint_names=[],
                final_positions=[],
                final_position_errors=[],
            ),
        )
        self._cache(output)
        return output

    def _reject_pose(self, goal_id: str, code: str, message: str) -> MotionServerOutput:
        output = MotionServerOutput(
            parent_kind="move_pose",
            parent_goal_id=goal_id,
            parent_goal_status="aborted",
            move_pose_result=MovePoseResult(
                error_code=code,  # type: ignore[arg-type]
                message=message,
                elapsed_ns=0,
                final_pose=None,
                final_position_error_m=None,
                final_orientation_error_rad=None,
                joint_names=[],
                final_joint_positions=[],
            ),
        )
        self._cache(output)
        return output

    def _cached(self, kind: GoalKind, goal_id: str) -> MotionServerOutput | None:
        key = (kind, goal_id)
        output = self._result_cache.get(key)
        if output is not None:
            self._result_cache.move_to_end(key)
        return output

    def _cache(self, output: MotionServerOutput) -> None:
        assert output.parent_kind is not None and output.parent_goal_id is not None
        key = (output.parent_kind, output.parent_goal_id)
        self._result_cache[key] = output
        self._result_cache.move_to_end(key)
        while len(self._result_cache) > self.config.result_cache_size:
            self._result_cache.popitem(last=False)

    @staticmethod
    def _validate_goal_id(goal_id: str) -> None:
        if not isinstance(goal_id, str) or not goal_id:
            raise ValueError("goal_id must be a non-empty string")

    def _validate_now(self, now_ns: int) -> None:
        if now_ns < 0:
            raise ValueError("now_ns must be non-negative")
        if self._last_now_ns is not None and now_ns < self._last_now_ns:
            raise ValueError("now_ns must be monotonic")
        self._last_now_ns = now_ns
