from __future__ import annotations

from typing import Any

import pytest
from forge_motion_server.node import MotionServerNode
from forge_motion_server.server import MotionServer
from forge_msgs import (
    FollowJointTrajectoryGoal,
    FollowJointTrajectoryResult,
    JointState,
    MoveJointsFeedback,
    MoveJointsResult,
)


class FakeNode:
    def __init__(self) -> None:
        self.sent: list[tuple[str, Any, dict[str, Any] | None]] = []

    def __iter__(self):
        return iter(())

    def send_output(self, output_id: str, value: Any, **kwargs: Any) -> None:
        self.sent.append((output_id, value, kwargs.get("metadata")))


class FailingOutputNode(FakeNode):
    def send_output(self, output_id: str, value: Any, **kwargs: Any) -> None:
        raise ValueError("transport failed")


def _event_metadata(goal_id: str, status: str | None = None, *, envelope: bool = False):
    values = {"goal_id": goal_id}
    if status is not None:
        values["goal_status"] = status
    return {"parameters": values} if envelope else values


def test_node_routes_parent_child_action_and_flat_output_metadata(
    server_config, fake_kinematics, joints_goal
) -> None:
    adapter = MotionServerNode(MotionServer(server_config, fake_kinematics))
    node = FakeNode()
    adapter.handle_event(
        node,
        {
            "type": "INPUT",
            "id": "joint_state",
            "value": JointState(
                name=["j1", "j2", "j3"],
                position=[0.0, 0.0, 0.0],
                velocity=[0.0, 0.0, 0.0],
                effort=[],
            ).to_arrow(),
        },
        0,
    )
    adapter.handle_event(
        node,
        {
            "type": "INPUT",
            "id": "move_joints_goal",
            "value": joints_goal.to_arrow(),
            "metadata": _event_metadata("parent", envelope=True),
        },
        0,
    )

    assert [item[0] for item in node.sent] == ["move_joints_feedback", "trajectory_goal"]
    feedback = MoveJointsFeedback.from_arrow(node.sent[0][1])
    child = FollowJointTrajectoryGoal.from_arrow(node.sent[1][1])
    assert feedback.phase == "WAITING_FOR_CONTROLLER"
    assert child.trajectory.joint_names == ["j1", "j2", "j3"]
    assert node.sent[0][2] == {"goal_id": "parent"}
    child_id = node.sent[1][2]["goal_id"]

    adapter.handle_event(
        node,
        {
            "type": "INPUT",
            "id": "joint_state",
            "value": JointState(
                name=["j1", "j2", "j3"],
                position=list(joints_goal.positions),
                velocity=[0.0, 0.0, 0.0],
                effort=[],
            ).to_arrow(),
        },
        500_000_000,
    )
    result = FollowJointTrajectoryResult(
        error_code="SUCCESS",
        message="done",
        elapsed_ns=500_000_000,
        joint_names=["j1", "j2", "j3"],
        final_position_error=[0.0, 0.0, 0.0],
        final_velocity_error=[0.0, 0.0, 0.0],
    )
    adapter.handle_event(
        node,
        {
            "type": "INPUT",
            "id": "trajectory_result",
            "value": result.to_arrow(),
            "metadata": _event_metadata(child_id, "succeeded"),
        },
        500_000_000,
    )

    terminal = MoveJointsResult.from_arrow(node.sent[-1][1])
    assert terminal.error_code == "SUCCESS"
    assert node.sent[-1][2] == {"goal_id": "parent", "goal_status": "succeeded"}


def test_node_output_failure_is_not_swallowed(server_config, fake_kinematics, joints_goal) -> None:
    adapter = MotionServerNode(MotionServer(server_config, fake_kinematics))
    state_event = {
        "type": "INPUT",
        "id": "joint_state",
        "value": JointState(
            name=["j1", "j2", "j3"],
            position=[0.0, 0.0, 0.0],
            velocity=[0.0, 0.0, 0.0],
            effort=[],
        ).to_arrow(),
    }
    adapter.handle_event(FakeNode(), state_event, 0)

    with pytest.raises(ValueError, match="transport failed"):
        adapter.handle_event(
            FailingOutputNode(),
            {
                "type": "INPUT",
                "id": "move_joints_goal",
                "value": joints_goal.to_arrow(),
                "metadata": _event_metadata("transport"),
            },
            0,
        )


def test_node_malformed_goal_returns_terminal_invalid_goal(server_config, fake_kinematics) -> None:
    adapter = MotionServerNode(MotionServer(server_config, fake_kinematics))
    node = FakeNode()

    adapter.handle_event(
        node,
        {
            "type": "INPUT",
            "id": "move_joints_goal",
            "value": b"not-arrow",
            "metadata": _event_metadata("malformed"),
        },
        0,
    )

    result = MoveJointsResult.from_arrow(node.sent[0][1])
    assert result.error_code == "INVALID_GOAL"
    assert node.sent[0][2] == {"goal_id": "malformed", "goal_status": "aborted"}
