from __future__ import annotations

import asyncio

import pytest
from forge_motion_server.action_endpoint import (
    ACTION_DESCRIPTOR,
    MotionActionEndpoint,
    goal_id_for_key,
)
from forge_msgs import MoveJointsFeedback, MoveJointsResult

from forge_tool import (
    ToolContext,
    ToolEndpointError,
    ToolEvent,
    ToolExecutionKey,
    ToolRequest,
)


class Events:
    def __init__(self) -> None:
        self.items: list[ToolEvent] = []

    async def emit(self, event: ToolEvent) -> None:
        self.items.append(event)


def _context(
    key: ToolExecutionKey,
    operation: str = "move_joints",
    *,
    deadline_ms: int | None = None,
) -> ToolContext:
    return ToolContext(
        execution_key=key,
        tool_id=f"motion.{operation}",
        implementation_id="motion",
        endpoint_id="motion.server",
        operation=operation,
        deadline_ms=deadline_ms,
    )


def _request(position: float = 0.1) -> ToolRequest:
    return ToolRequest(
        {
            "group_name": "arm",
            "joint_names": ["j1"],
            "positions": [position],
            "velocity_scale": 0.5,
            "acceleration_scale": 0.5,
        }
    )


def _result(code: str = "SUCCESS") -> MoveJointsResult:
    return MoveJointsResult(
        error_code=code,
        message="done" if code == "SUCCESS" else "rejected",
        elapsed_ns=10,
        joint_names=["j1"],
        final_positions=[0.1],
        final_position_errors=[0.0],
    )


def test_action_descriptor_exposes_complete_action_operations() -> None:
    assert ACTION_DESCRIPTOR.endpoint_id == "motion.server"
    operations = [
        (item.name, item.semantics, item.cancellable) for item in ACTION_DESCRIPTOR.operations
    ]
    assert operations == [
        ("move_joints", "action", True),
        ("move_pose", "action", True),
    ]


def test_action_maps_execution_key_and_is_idempotent() -> None:
    sent = []
    endpoint = MotionActionEndpoint(
        lambda kind, goal_id, goal: sent.append((kind, goal_id, goal)),
        lambda kind, goal_id: None,
    )
    key = ToolExecutionKey("invocation", "attempt")
    events = Events()

    first = asyncio.run(endpoint.start(_request(), _context(key), events))
    duplicate = asyncio.run(endpoint.start(_request(), _context(key), events))

    assert first.details["goal_id"] == goal_id_for_key(key)
    assert first.details["idempotent"] is False
    assert duplicate.details["idempotent"] is True
    assert len(sent) == 1
    assert asyncio.run(endpoint.status(key)).phase == "running"
    assert asyncio.run(endpoint.result(key)).status == "pending"


def test_action_rejects_other_execution_as_busy_and_key_conflict() -> None:
    endpoint = MotionActionEndpoint(lambda *args: None, lambda *args: None)
    first_key = ToolExecutionKey("one", "attempt")
    asyncio.run(endpoint.start(_request(), _context(first_key), Events()))

    with pytest.raises(ToolEndpointError) as busy:
        asyncio.run(
            endpoint.start(
                _request(),
                _context(ToolExecutionKey("two", "attempt")),
                Events(),
            )
        )
    assert busy.value.error.code == "BUSY"
    assert busy.value.error.retryable is True

    with pytest.raises(ToolEndpointError) as conflict:
        asyncio.run(endpoint.start(_request(0.2), _context(first_key), Events()))
    assert conflict.value.error.code == "FORGE_EXECUTION_CONFLICT"


def test_action_rejects_expired_deadline_before_sending_goal() -> None:
    sent = []
    endpoint = MotionActionEndpoint(
        lambda *args: sent.append(args),
        lambda *args: None,
        epoch_ms=lambda: 1_000,
    )

    with pytest.raises(ToolEndpointError) as captured:
        asyncio.run(
            endpoint.start(
                _request(),
                _context(ToolExecutionKey("one", "attempt"), deadline_ms=1_000),
                Events(),
            )
        )

    assert captured.value.error.code == "FORGE_DEADLINE_EXCEEDED"
    assert sent == []


def test_shutdown_stops_admission_and_cancels_without_claiming_terminal() -> None:
    cancelled = []
    endpoint = MotionActionEndpoint(
        lambda *args: None,
        lambda kind, goal_id: cancelled.append((kind, goal_id)),
    )
    key = ToolExecutionKey("one", "attempt")
    accepted = asyncio.run(endpoint.start(_request(), _context(key), Events()))

    outcome = endpoint.begin_shutdown()

    assert outcome.active_goal_id == accepted.details["goal_id"]
    assert outcome.cancel_requested is True
    assert outcome.terminal_confirmed is False
    assert len(cancelled) == 1
    assert asyncio.run(endpoint.status(key)).phase == "stopping"
    with pytest.raises(ToolEndpointError) as captured:
        asyncio.run(
            endpoint.start(
                _request(),
                _context(ToolExecutionKey("two", "attempt")),
                Events(),
            )
        )
    assert captured.value.error.code == "FORGE_ENDPOINT_STOPPING"


def test_action_feedback_is_rate_limited_and_terminal_result_is_cached() -> None:
    now = [0]
    endpoint = MotionActionEndpoint(
        lambda *args: None,
        lambda *args: None,
        feedback_interval_ms=100,
        clock_ns=lambda: now[0],
    )
    key = ToolExecutionKey("one", "attempt")
    events = Events()
    accepted = asyncio.run(endpoint.start(_request(), _context(key), events))
    goal_id = accepted.details["goal_id"]
    feedback = MoveJointsFeedback(
        phase="EXECUTING",
        progress=0.5,
        elapsed_ns=5,
        estimated_duration_ns=10,
        joint_names=["j1"],
        actual_positions=[0.05],
        target_positions=[0.1],
        position_errors=[0.05],
        message="moving",
    )

    asyncio.run(endpoint.handle_feedback(goal_id, feedback))
    now[0] = 50_000_000
    asyncio.run(endpoint.handle_feedback(goal_id, feedback))
    asyncio.run(endpoint.handle_result(goal_id, _result(), "succeeded"))
    asyncio.run(endpoint.handle_result(goal_id, _result(), "succeeded"))

    assert [event.type for event in events.items] == ["progress", "executor_completed"]
    assert asyncio.run(endpoint.status(key)).phase == "completed"
    response = asyncio.run(endpoint.result(key))
    assert response.status == "available"
    assert response.result is not None
    assert response.result.status == "succeeded"
    assert response.result.outputs["motion_result"]["error_code"] == "SUCCESS"


def test_action_cancel_delegates_and_preserves_motion_error_code() -> None:
    cancelled = []
    endpoint = MotionActionEndpoint(
        lambda *args: None,
        lambda kind, goal_id: cancelled.append((kind, goal_id)),
    )
    key = ToolExecutionKey("one", "attempt")
    events = Events()
    accepted = asyncio.run(endpoint.start(_request(), _context(key), events))

    response = asyncio.run(endpoint.cancel(key, "operator"))
    asyncio.run(
        endpoint.handle_result(accepted.details["goal_id"], _result("CANCELED"), "canceled")
    )

    assert response.status == "accepted"
    assert len(cancelled) == 1
    assert asyncio.run(endpoint.status(key)).phase == "cancelled"
    result = asyncio.run(endpoint.result(key)).result
    assert result is not None
    assert result.status == "cancelled"
    assert result.outputs["motion_result"]["error_code"] == "CANCELED"


def test_action_failed_result_keeps_motion_error_code() -> None:
    endpoint = MotionActionEndpoint(lambda *args: None, lambda *args: None)
    key = ToolExecutionKey("one", "attempt")
    accepted = asyncio.run(endpoint.start(_request(), _context(key), Events()))

    asyncio.run(endpoint.handle_result(accepted.details["goal_id"], _result("BUSY"), "aborted"))

    status = asyncio.run(endpoint.status(key))
    result = asyncio.run(endpoint.result(key)).result
    assert status.phase == "failed"
    assert status.error is not None and status.error.code == "BUSY"
    assert result is not None and result.error is not None
    assert result.error.code == "BUSY"
