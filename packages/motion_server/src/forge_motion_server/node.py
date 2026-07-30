from __future__ import annotations

import logging
import time
from collections.abc import Callable, Iterator
from typing import Any, Protocol

import pyarrow as pa
from forge_msgs import (
    FollowJointTrajectoryFeedback,
    FollowJointTrajectoryResult,
    JointState,
    MoveJointsGoal,
    MovePoseGoal,
)

from .config import MotionServerConfig
from .kinematics import ForgeKinematicsAdapter, MotionKinematics
from .models import GoalKind, GoalStatus, MotionServerOutput
from .server import MotionServer

logger = logging.getLogger(__name__)


class DoraNodeLike(Protocol):
    def __iter__(self) -> Iterator[dict[str, Any]]: ...

    def send_output(self, output_id: str, value: Any, **kwargs: Any) -> None: ...


class MotionServerNode:
    """Thin Dora transport adapter around the deterministic MotionServer core."""

    def __init__(self, server: MotionServer) -> None:
        self.server = server

    def handle_event(self, node: DoraNodeLike, event: dict[str, Any], now_ns: int) -> bool:
        event_type = event.get("type")
        if event_type == "STOP":
            return False
        if event_type == "ERROR":
            logger.error("Dora node error: %s", event.get("error", "unknown"))
            return False
        if event_type != "INPUT":
            return True

        input_id = event.get("id")
        try:
            output = self._handle_input(event, now_ns)
        except (KeyError, TypeError, ValueError) as error:
            logger.warning("Ignoring invalid %s input: %s", input_id, error)
            return True
        self._emit(node, output)
        return True

    def _handle_input(self, event: dict[str, Any], now_ns: int) -> MotionServerOutput | None:
        input_id = event.get("id")
        if input_id == "joint_state":
            self.server.update_joint_state(JointState.from_arrow(event["value"]), now_ns)
            return None
        if input_id == "move_joints_goal":
            return self._handle_parent_goal("move_joints", event, now_ns)
        if input_id == "move_pose_goal":
            return self._handle_parent_goal("move_pose", event, now_ns)
        if input_id == "move_joints_cancel":
            return self._handle_parent_cancel("move_joints", event, now_ns)
        if input_id == "move_pose_cancel":
            return self._handle_parent_cancel("move_pose", event, now_ns)
        if input_id == "trajectory_feedback":
            goal_id = _goal_id(event)
            if goal_id is None:
                logger.warning("Ignoring trajectory_feedback without goal_id metadata")
                return None
            feedback = FollowJointTrajectoryFeedback.from_arrow(event["value"])
            return self.server.handle_trajectory_feedback(goal_id, feedback, now_ns)
        if input_id == "trajectory_result":
            goal_id = _goal_id(event)
            goal_status = _goal_status(event)
            if goal_id is None or goal_status is None:
                logger.warning("Ignoring trajectory_result without valid action metadata")
                return None
            result = FollowJointTrajectoryResult.from_arrow(event["value"])
            return self.server.handle_trajectory_result(goal_id, goal_status, result, now_ns)
        if input_id == "tick":
            return self.server.tick(now_ns)
        return None

    def _handle_parent_goal(
        self, kind: GoalKind, event: dict[str, Any], now_ns: int
    ) -> MotionServerOutput | None:
        goal_id = _goal_id(event)
        if goal_id is None:
            logger.warning("Ignoring %s goal without goal_id metadata", kind)
            return None
        try:
            goal = (
                MoveJointsGoal.from_arrow(event["value"])
                if kind == "move_joints"
                else MovePoseGoal.from_arrow(event["value"])
            )
        except Exception as error:
            return self.server.reject_invalid_goal(
                kind, goal_id, f"failed to decode {kind} goal: {error}", now_ns
            )
        if kind == "move_joints":
            assert isinstance(goal, MoveJointsGoal)
            return self.server.accept_move_joints(goal_id, goal, now_ns)
        assert isinstance(goal, MovePoseGoal)
        return self.server.accept_move_pose(goal_id, goal, now_ns)

    def _handle_parent_cancel(
        self, kind: GoalKind, event: dict[str, Any], now_ns: int
    ) -> MotionServerOutput | None:
        goal_id = _goal_id(event)
        if goal_id is None:
            logger.warning("Ignoring %s cancel without goal_id metadata", kind)
            return None
        return self.server.cancel_goal(kind, goal_id, now_ns)

    @staticmethod
    def _emit(node: DoraNodeLike, output: MotionServerOutput | None) -> None:
        if output is None:
            return
        if output.move_joints_feedback is not None:
            _require_parent(output)
            node.send_output(
                "move_joints_feedback",
                output.move_joints_feedback.to_arrow(),
                metadata=_action_metadata(output.parent_goal_id),
            )
        if output.move_pose_feedback is not None:
            _require_parent(output)
            node.send_output(
                "move_pose_feedback",
                output.move_pose_feedback.to_arrow(),
                metadata=_action_metadata(output.parent_goal_id),
            )
        if output.child_goal is not None:
            if output.child_goal_id is None:
                raise ValueError("child trajectory goal requires child_goal_id")
            node.send_output(
                "trajectory_goal",
                output.child_goal.to_arrow(),
                metadata=_action_metadata(output.child_goal_id),
            )
        if output.child_cancel_goal_id is not None:
            node.send_output(
                "trajectory_cancel",
                pa.array([True], type=pa.bool_()),
                metadata=_action_metadata(output.child_cancel_goal_id),
            )
        if output.move_joints_result is not None:
            _require_terminal_parent(output)
            node.send_output(
                "move_joints_result",
                output.move_joints_result.to_arrow(),
                metadata=_action_metadata(output.parent_goal_id, output.parent_goal_status),
            )
        if output.move_pose_result is not None:
            _require_terminal_parent(output)
            node.send_output(
                "move_pose_result",
                output.move_pose_result.to_arrow(),
                metadata=_action_metadata(output.parent_goal_id, output.parent_goal_status),
            )


def run_dora_motion_server(
    config: MotionServerConfig,
    *,
    kinematics: MotionKinematics | None = None,
    node_factory: Callable[[], DoraNodeLike] | None = None,
    clock: Callable[[], int] = time.monotonic_ns,
) -> int:
    backend = kinematics or ForgeKinematicsAdapter(config)
    factory = node_factory
    if factory is None:
        from dora import Node

        factory = Node
    node = factory()
    adapter = MotionServerNode(MotionServer(config, backend))
    for event in node:
        if not adapter.handle_event(node, event, clock()):
            break
    return 0


def _metadata_parameters(event: dict[str, Any]) -> dict[str, Any] | None:
    metadata = event.get("metadata")
    if not isinstance(metadata, dict):
        return None
    parameters = metadata.get("parameters")
    return parameters if isinstance(parameters, dict) else metadata


def _goal_id(event: dict[str, Any]) -> str | None:
    parameters = _metadata_parameters(event)
    if parameters is None or parameters.get("goal_id") is None:
        return None
    goal_id = str(parameters["goal_id"])
    return goal_id if goal_id else None


def _goal_status(event: dict[str, Any]) -> GoalStatus | None:
    parameters = _metadata_parameters(event)
    if parameters is None:
        return None
    status = parameters.get("goal_status")
    return status if status in {"succeeded", "aborted", "canceled"} else None


def _action_metadata(goal_id: str | None, goal_status: GoalStatus | None = None) -> dict[str, Any]:
    if not goal_id:
        raise ValueError("action metadata requires a non-empty goal_id")
    metadata: dict[str, Any] = {"goal_id": goal_id}
    if goal_status is not None:
        metadata["goal_status"] = goal_status
    return metadata


def _require_parent(output: MotionServerOutput) -> None:
    if output.parent_kind is None or output.parent_goal_id is None:
        raise ValueError("parent feedback requires parent action identity")


def _require_terminal_parent(output: MotionServerOutput) -> None:
    _require_parent(output)
    if output.parent_goal_status is None:
        raise ValueError("parent result requires goal_status")
