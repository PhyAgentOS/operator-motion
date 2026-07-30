from __future__ import annotations

import pytest
from forge_motion_server.kinematics import InverseKinematicsOutcome
from forge_msgs import (
    FollowJointTrajectoryFeedback,
    FollowJointTrajectoryResult,
    JointState,
    JointTrajectoryPoint,
    Pose,
)


def _state(positions, velocities=None):
    return JointState(
        name=["j1", "j2", "j3"],
        position=list(positions),
        velocity=[0.0, 0.0, 0.0] if velocities is None else list(velocities),
        effort=[],
    )


def _child_result(code="SUCCESS", message="done"):
    return FollowJointTrajectoryResult(
        error_code=code,
        message=message,
        elapsed_ns=500_000_000,
        joint_names=["j1", "j2", "j3"],
        final_position_error=[0.0, 0.0, 0.0],
        final_velocity_error=[0.0, 0.0, 0.0],
    )


def _point(positions, time_ns):
    return JointTrajectoryPoint(
        positions=list(positions),
        velocities=[0.0, 0.0, 0.0],
        accelerations=[],
        effort=[],
        time_from_start_ns=time_ns,
    )


def test_move_joints_requires_fresh_complete_state(server, joints_goal) -> None:
    output = server.accept_move_joints("no-state", joints_goal, 0)
    assert output is not None
    assert output.move_joints_result.error_code == "NO_FRESH_ROBOT_STATE"

    server.update_joint_state(_state((0.0, 0.0, 0.0)), 1_000_000_000)
    stale = server.accept_move_joints("stale", joints_goal, 1_200_000_001)
    assert stale is not None
    assert stale.move_joints_result.error_code == "NO_FRESH_ROBOT_STATE"


def test_move_joints_plans_child_and_maps_feedback_and_success(server, joints_goal) -> None:
    server.update_joint_state(_state((0.0, 0.0, 0.0)), 0)
    started = server.accept_move_joints("move-1", joints_goal, 0)

    assert started is not None
    assert started.child_goal is not None
    assert started.child_goal_id == server.active_child_goal_id
    assert started.move_joints_feedback.phase == "WAITING_FOR_CONTROLLER"
    assert started.child_goal.trajectory.joint_names == ["j1", "j2", "j3"]
    assert started.child_goal.trajectory.points[0].velocities == [0.0, 0.0, 0.0]

    child_id = started.child_goal_id
    feedback = FollowJointTrajectoryFeedback(
        sequence=1,
        point_index=1,
        elapsed_ns=100_000_000,
        duration_ns=500_000_000,
        desired=_point((0.1, -0.05, 0.02), 100_000_000),
        actual=_point((0.08, -0.04, 0.01), 100_000_000),
        error=_point((0.02, -0.01, 0.01), 100_000_000),
    )
    mapped = server.handle_trajectory_feedback(child_id, feedback, 100_000_000)
    assert mapped is not None
    assert mapped.move_joints_feedback.phase == "EXECUTING"
    assert mapped.move_joints_feedback.progress == pytest.approx(0.2)

    server.update_joint_state(_state(joints_goal.positions), 500_000_000)
    terminal = server.handle_trajectory_result(child_id, "succeeded", _child_result(), 500_000_000)

    assert terminal is not None
    assert terminal.parent_goal_status == "succeeded"
    assert terminal.move_joints_result.error_code == "SUCCESS"
    assert server.active_goal_id is None


def test_final_joint_residual_is_checked_after_child_success(server, joints_goal) -> None:
    server.update_joint_state(_state((0.0, 0.0, 0.0)), 0)
    started = server.accept_move_joints("residual", joints_goal, 0)
    server.update_joint_state(_state((0.0, 0.0, 0.0)), 500_000_000)

    settling = server.handle_trajectory_result(
        started.child_goal_id, "succeeded", _child_result(), 500_000_000
    )
    assert settling.move_joints_feedback.phase == "SETTLING"

    terminal = server.tick(700_000_000)
    assert terminal.move_joints_result.error_code == "FINAL_JOINT_TOLERANCE_VIOLATED"
    assert terminal.parent_goal_status == "aborted"


def test_busy_duplicate_cancel_and_terminal_retry(server, joints_goal) -> None:
    server.update_joint_state(_state((0.0, 0.0, 0.0)), 0)
    active = server.accept_move_joints("active", joints_goal, 0)

    assert server.accept_move_joints("active", joints_goal, 1) is None
    conflicting = joints_goal.model_copy(update={"positions": [0.1, 0.1, 0.1]})
    assert server.accept_move_joints("active", conflicting, 2) is None

    busy = server.accept_move_joints("busy", joints_goal, 3)
    assert busy.move_joints_result.error_code == "BUSY"
    assert server.active_goal_id == "active"

    cancel = server.cancel_goal("move_joints", "active", 4)
    assert cancel.child_cancel_goal_id == active.child_goal_id
    assert server.cancel_goal("move_joints", "active", 5) is None

    terminal = server.handle_trajectory_result(
        active.child_goal_id,
        "canceled",
        _child_result("CANCELED", "canceled"),
        6,
    )
    assert terminal.parent_goal_status == "canceled"
    assert terminal.move_joints_result.error_code == "CANCELED"

    replay = server.accept_move_joints("active", joints_goal, 7)
    assert replay == terminal
    assert replay.child_goal is None


def test_cancel_is_retried_until_child_acknowledges(server, joints_goal) -> None:
    server.update_joint_state(_state((0.0, 0.0, 0.0)), 0)
    started = server.accept_move_joints("cancel-retry", joints_goal, 0)
    server.cancel_goal("move_joints", "cancel-retry", 1)

    retry = server.tick(20_000_001)

    assert retry.child_cancel_goal_id == started.child_goal_id


def test_cancel_timeout_aborts_parent(server, joints_goal) -> None:
    server.update_joint_state(_state((0.0, 0.0, 0.0)), 0)
    server.accept_move_joints("cancel-timeout", joints_goal, 0)
    server.cancel_goal("move_joints", "cancel-timeout", 1)

    terminal = server.tick(100_000_002)

    assert terminal.move_joints_result.error_code == "TRAJECTORY_EXECUTION_FAILED"
    assert terminal.parent_goal_status == "aborted"


def test_goal_validation_rejects_joint_order_limits_velocity_and_duration(
    server, joints_goal
) -> None:
    server.update_joint_state(_state((0.0, 0.0, 0.0)), 0)
    wrong_order = joints_goal.model_copy(update={"joint_names": ["j2", "j1", "j3"]})
    invalid = server.accept_move_joints("order", wrong_order, 0)
    assert invalid.move_joints_result.error_code == "INVALID_JOINTS"

    outside = joints_goal.model_copy(update={"positions": [3.0, 0.0, 0.0]})
    invalid = server.accept_move_joints("limits", outside, 0)
    assert invalid.move_joints_result.error_code == "JOINT_LIMIT_VIOLATION"

    moving_server = type(server)(server.config, server.kinematics)
    moving_server.update_joint_state(_state((0.0, 0.0, 0.0), (0.2, 0.0, 0.0)), 0)
    invalid = moving_server.accept_move_joints("moving", joints_goal, 0)
    assert invalid.move_joints_result.error_code == "PLANNING_FAILED"

    too_fast = joints_goal.model_copy(update={"requested_duration_ns": 1})
    invalid = server.accept_move_joints("duration", too_fast, 0)
    assert invalid.move_joints_result.error_code == "TRAJECTORY_GENERATION_FAILED"


def test_execution_timeout_cancels_child_and_aborts_after_ack(server, joints_goal) -> None:
    server.update_joint_state(_state((0.0, 0.0, 0.0)), 0)
    started = server.accept_move_joints("execution-timeout", joints_goal, 0)
    duration_ns = started.child_goal.trajectory.points[-1].time_from_start_ns

    cancel = server.tick(duration_ns + server.config.execution_timeout_margin_ns + 1)
    assert cancel.child_cancel_goal_id == started.child_goal_id

    terminal = server.handle_trajectory_result(
        started.child_goal_id,
        "canceled",
        _child_result("CANCELED", "canceled after timeout"),
        duration_ns + server.config.execution_timeout_margin_ns + 2,
    )
    assert terminal.move_joints_result.error_code == "TRAJECTORY_EXECUTION_FAILED"
    assert terminal.parent_goal_status == "aborted"


def test_execution_timeout_is_not_overwritten_by_late_child_success(server, joints_goal) -> None:
    server.update_joint_state(_state((0.0, 0.0, 0.0)), 0)
    started = server.accept_move_joints("late-success", joints_goal, 0)
    duration_ns = started.child_goal.trajectory.points[-1].time_from_start_ns
    deadline_ns = duration_ns + server.config.execution_timeout_margin_ns
    server.tick(deadline_ns + 1)
    server.update_joint_state(_state(joints_goal.positions), deadline_ns + 2)

    terminal = server.handle_trajectory_result(
        started.child_goal_id,
        "succeeded",
        _child_result(),
        deadline_ns + 2,
    )

    assert terminal.move_joints_result.error_code == "TRAJECTORY_EXECUTION_FAILED"
    assert terminal.parent_goal_status == "aborted"


def test_child_cancel_without_parent_request_aborts(server, joints_goal) -> None:
    server.update_joint_state(_state((0.0, 0.0, 0.0)), 0)
    started = server.accept_move_joints("external-cancel", joints_goal, 0)

    terminal = server.handle_trajectory_result(
        started.child_goal_id,
        "canceled",
        _child_result("CANCELED", "external"),
        1,
    )

    assert terminal.move_joints_result.error_code == "TRAJECTORY_EXECUTION_FAILED"
    assert terminal.parent_goal_status == "aborted"


def test_final_state_can_arrive_after_child_success(server, joints_goal) -> None:
    server.update_joint_state(_state((0.0, 0.0, 0.0)), 0)
    started = server.accept_move_joints("state-race", joints_goal, 0)
    server.update_joint_state(_state((0.0, 0.0, 0.0)), 500_000_000)

    settling = server.handle_trajectory_result(
        started.child_goal_id, "succeeded", _child_result(), 500_000_000
    )
    assert settling.move_joints_feedback.phase == "SETTLING"

    server.update_joint_state(_state(joints_goal.positions), 520_000_000)
    terminal = server.tick(520_000_001)
    assert terminal.move_joints_result.error_code == "SUCCESS"
    assert terminal.parent_goal_status == "succeeded"


def test_child_rejection_and_execution_failure_are_mapped(server, joints_goal) -> None:
    server.update_joint_state(_state((0.0, 0.0, 0.0)), 0)
    started = server.accept_move_joints("rejected", joints_goal, 0)
    terminal = server.handle_trajectory_result(
        started.child_goal_id,
        "aborted",
        _child_result("START_STATE_MISMATCH", "bad start"),
        1,
    )
    assert terminal.move_joints_result.error_code == "TRAJECTORY_REJECTED"

    server.update_joint_state(_state((0.0, 0.0, 0.0)), 2)
    started = server.accept_move_joints("failed", joints_goal, 2)
    terminal = server.handle_trajectory_result(
        started.child_goal_id,
        "aborted",
        _child_result("PATH_TOLERANCE_VIOLATED", "path"),
        3,
    )
    assert terminal.move_joints_result.error_code == "TRAJECTORY_EXECUTION_FAILED"


def test_move_pose_uses_ik_and_checks_final_fk(server, fake_kinematics, pose_goal) -> None:
    server.update_joint_state(_state((0.0, 0.0, 0.0)), 0)
    started = server.accept_move_pose("pose", pose_goal, 0)

    assert started.child_goal is not None
    assert started.move_pose_feedback.phase == "WAITING_FOR_CONTROLLER"
    assert fake_kinematics.inverse_calls == 1

    server.update_joint_state(_state((0.2, -0.1, 0.3)), 500_000_000)
    terminal = server.handle_trajectory_result(
        started.child_goal_id, "succeeded", _child_result(), 500_000_000
    )

    assert terminal.move_pose_result.error_code == "SUCCESS"
    assert terminal.move_pose_result.final_position_error_m == pytest.approx(0.0)
    assert terminal.parent_goal_status == "succeeded"


def test_move_pose_rejects_frame_ik_failure_and_final_pose_error(
    server, fake_kinematics, pose_goal
) -> None:
    server.update_joint_state(_state((0.0, 0.0, 0.0)), 0)
    wrong_frame = pose_goal.model_copy(update={"reference_frame": "world"})
    invalid = server.accept_move_pose("frame", wrong_frame, 0)
    assert invalid.move_pose_result.error_code == "INVALID_FRAME"

    zero_tolerance = pose_goal.model_copy(update={"position_tolerance_m": 0.0})
    invalid = server.accept_move_pose("zero-tolerance", zero_tolerance, 0)
    assert invalid.move_pose_result.error_code == "INVALID_GOAL"

    nonfinite_pose = pose_goal.model_copy(update={"target_pose": Pose(x=float("inf"), y=0.0)})
    invalid = server.accept_move_pose("nonfinite-pose", nonfinite_pose, 0)
    assert invalid.move_pose_result.error_code == "INVALID_GOAL"

    fake_kinematics.inverse_outcome = InverseKinematicsOutcome(
        status="TIMED_OUT", message="deadline", positions=None
    )
    invalid = server.accept_move_pose("ik-timeout", pose_goal, 0)
    assert invalid.move_pose_result.error_code == "IK_TIMED_OUT"

    fake_kinematics.inverse_outcome = InverseKinematicsOutcome(
        status="SUCCESS", message="solved", positions=(0.2, -0.1, 0.3)
    )
    distant_goal = pose_goal.model_copy(update={"target_pose": Pose(x=0.8, y=-0.1, z=0.3)})
    started = server.accept_move_pose("pose-residual", distant_goal, 0)
    server.update_joint_state(_state((0.2, -0.1, 0.3)), 500_000_000)
    settling = server.handle_trajectory_result(
        started.child_goal_id, "succeeded", _child_result(), 500_000_000
    )
    assert settling.move_pose_feedback.phase == "SETTLING"

    terminal = server.tick(700_000_000)
    assert terminal.move_pose_result.error_code == "FINAL_POSE_TOLERANCE_VIOLATED"
