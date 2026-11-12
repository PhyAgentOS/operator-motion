"""Forge Action ToolEndpoint SPI adapter over injected Motion goal/cancel senders.

The surrounding policy runner currently supplies Dora senders, but this module owns no
Dora transport, IK, planning, controller orchestration, or residual validation.
"""

from __future__ import annotations

import hashlib
import json
import logging
import math
import time
from collections import OrderedDict
from collections.abc import Callable, Mapping
from dataclasses import dataclass
from typing import Any, Literal

from forge_msgs import (
    MoveJointsFeedback,
    MoveJointsGoal,
    MoveJointsResult,
    MovePoseFeedback,
    MovePoseGoal,
    MovePoseResult,
    Pose,
)
from forge_tool import (
    TOOL_ENDPOINT_PROTOCOL,
    ToolAccepted,
    ToolContext,
    ToolControlResponse,
    ToolEndpointDescriptor,
    ToolEndpointError,
    ToolError,
    ToolEvent,
    ToolEventEmitter,
    ToolExecutionKey,
    ToolExecutionStatus,
    ToolOperationDescriptor,
    ToolRequest,
    ToolResult,
    ToolResultResponse,
)

ACTION_ENDPOINT_ID = "motion.server"
logger = logging.getLogger(__name__)
ACTION_DESCRIPTOR = ToolEndpointDescriptor(
    protocol_version=TOOL_ENDPOINT_PROTOCOL,
    endpoint_id=ACTION_ENDPOINT_ID,
    operations=(
        ToolOperationDescriptor(
            name="move_joints",
            semantics="action",
            cancellable=True,
            status_supported=True,
            max_concurrency=1,
        ),
        ToolOperationDescriptor(
            name="move_pose",
            semantics="action",
            cancellable=True,
            status_supported=True,
            max_concurrency=1,
        ),
    ),
)

ActionKind = Literal["move_joints", "move_pose"]
GoalSender = Callable[[ActionKind, str, MoveJointsGoal | MovePoseGoal], None]
CancelSender = Callable[[ActionKind, str], None]


def goal_id_for_key(key: ToolExecutionKey) -> str:
    """Create the bounded MotionServer goal identity for one Tool attempt."""
    digest = hashlib.sha256(
        f"{key.invocation_id}\0{key.attempt_id}".encode()
    ).hexdigest()
    return f"tool-{digest}"


def _endpoint_error(code: str, message: str, *, retryable: bool = False) -> ToolEndpointError:
    return ToolEndpointError(ToolError(code=code, message=message, retryable=retryable))


def _number(value: Any, name: str, *, positive: bool = False) -> float:
    if isinstance(value, bool) or not isinstance(value, int | float):
        raise _endpoint_error("MOTION_INVALID_GOAL", f"{name} must be a number")
    result = float(value)
    if not math.isfinite(result) or (positive and result <= 0.0):
        qualifier = "finite and positive" if positive else "finite"
        raise _endpoint_error("MOTION_INVALID_GOAL", f"{name} must be {qualifier}")
    return result


def _optional_duration_ns(value: Any) -> int | None:
    if value is None:
        return None
    milliseconds = _number(value, "requested_duration_ms", positive=True)
    return int(round(milliseconds * 1_000_000))


def _strict(arguments: Mapping[str, Any], required: set[str], optional: set[str]) -> None:
    missing = sorted(required - set(arguments))
    unexpected = sorted(set(arguments) - required - optional)
    if missing or unexpected:
        raise _endpoint_error(
            "MOTION_INVALID_GOAL",
            f"invalid arguments: missing={missing}, unexpected={unexpected}",
        )


def _string(value: Any, name: str) -> str:
    if not isinstance(value, str) or not value.strip():
        raise _endpoint_error("MOTION_INVALID_GOAL", f"{name} must be a non-empty string")
    return value


def _float_list(value: Any, name: str) -> list[float]:
    if not isinstance(value, list) or not value:
        raise _endpoint_error("MOTION_INVALID_GOAL", f"{name} must be a non-empty array")
    return [_number(item, name) for item in value]


def _pose(value: Any) -> Pose:
    fields = {"x", "y", "z", "qx", "qy", "qz", "qw"}
    if not isinstance(value, Mapping) or set(value) != fields:
        raise _endpoint_error(
            "MOTION_INVALID_GOAL",
            "target_pose must contain exactly x, y, z, qx, qy, qz, qw",
        )
    return Pose(**{name: _number(value[name], f"target_pose.{name}") for name in fields})


def _goal(kind: ActionKind, arguments: Mapping[str, Any]) -> MoveJointsGoal | MovePoseGoal:
    common_optional = {"requested_duration_ms"}
    if kind == "move_joints":
        required = {
            "group_name",
            "joint_names",
            "positions",
            "velocity_scale",
            "acceleration_scale",
        }
        _strict(arguments, required, common_optional)
        names = arguments["joint_names"]
        if not isinstance(names, list) or any(
            not isinstance(name, str) or not name for name in names
        ):
            raise _endpoint_error(
                "MOTION_INVALID_GOAL", "joint_names must be a non-empty string array"
            )
        positions = _float_list(arguments["positions"], "positions")
        if len(names) != len(positions):
            raise _endpoint_error(
                "MOTION_INVALID_GOAL", "joint_names and positions lengths must match"
            )
        return MoveJointsGoal(
            group_name=_string(arguments["group_name"], "group_name"),
            joint_names=names,
            positions=positions,
            velocity_scale=_number(arguments["velocity_scale"], "velocity_scale", positive=True),
            acceleration_scale=_number(
                arguments["acceleration_scale"], "acceleration_scale", positive=True
            ),
            requested_duration_ns=_optional_duration_ns(arguments.get("requested_duration_ms")),
        )

    required = {
        "group_name",
        "reference_frame",
        "target_frame",
        "target_pose",
        "velocity_scale",
        "acceleration_scale",
        "position_tolerance_m",
        "orientation_tolerance_rad",
    }
    _strict(arguments, required, common_optional)
    return MovePoseGoal(
        group_name=_string(arguments["group_name"], "group_name"),
        reference_frame=_string(arguments["reference_frame"], "reference_frame"),
        target_frame=_string(arguments["target_frame"], "target_frame"),
        target_pose=_pose(arguments["target_pose"]),
        velocity_scale=_number(arguments["velocity_scale"], "velocity_scale", positive=True),
        acceleration_scale=_number(
            arguments["acceleration_scale"], "acceleration_scale", positive=True
        ),
        requested_duration_ns=_optional_duration_ns(arguments.get("requested_duration_ms")),
        position_tolerance_m=_number(
            arguments["position_tolerance_m"], "position_tolerance_m", positive=True
        ),
        orientation_tolerance_rad=_number(
            arguments["orientation_tolerance_rad"],
            "orientation_tolerance_rad",
            positive=True,
        ),
    )


def _jsonable(value: Any) -> Any:
    if hasattr(value, "model_dump"):
        return value.model_dump(mode="json")
    return value


@dataclass(slots=True)
class _Execution:
    key: ToolExecutionKey
    kind: ActionKind
    goal_id: str
    arguments_json: str
    emitter: ToolEventEmitter
    phase: str = "accepted"
    result: ToolResult | None = None
    last_event_ns: int | None = None


@dataclass(frozen=True, slots=True)
class ActionShutdownOutcome:
    """Best-effort shutdown state; cancellation admission is not terminal confirmation."""

    active_goal_id: str | None
    cancel_requested: bool
    terminal_confirmed: bool


class MotionActionEndpoint:
    """Action SPI implementation that delegates planning/execution to MotionServer."""

    def __init__(
        self,
        send_goal: GoalSender,
        send_cancel: CancelSender,
        *,
        result_cache_size: int = 128,
        feedback_interval_ms: int = 100,
        clock_ns: Callable[[], int] = time.monotonic_ns,
        epoch_ms: Callable[[], int] = lambda: time.time_ns() // 1_000_000,
    ) -> None:
        if result_cache_size < 1 or feedback_interval_ms < 1:
            raise ValueError("cache size and feedback interval must be positive")
        self._send_goal = send_goal
        self._send_cancel = send_cancel
        self._result_cache_size = result_cache_size
        self._feedback_interval_ns = feedback_interval_ms * 1_000_000
        self._clock_ns = clock_ns
        self._epoch_ms = epoch_ms
        self._executions: OrderedDict[ToolExecutionKey, _Execution] = OrderedDict()
        self._by_goal_id: dict[str, ToolExecutionKey] = {}
        self._active: ToolExecutionKey | None = None
        self._accepting = True

    async def start(
        self,
        request: ToolRequest,
        context: ToolContext,
        events: ToolEventEmitter,
    ) -> ToolAccepted:
        kind = context.operation
        if kind not in ("move_joints", "move_pose"):
            raise _endpoint_error("FORGE_PROTOCOL_UNKNOWN_OPERATION", f"unknown operation {kind!r}")
        if not self._accepting:
            raise _endpoint_error(
                "FORGE_ENDPOINT_STOPPING",
                "motion Action endpoint is shutting down and no longer accepts goals",
                retryable=True,
            )
        if context.deadline_ms is not None and self._epoch_ms() >= context.deadline_ms:
            raise _endpoint_error(
                "FORGE_DEADLINE_EXCEEDED",
                "the Tool invocation deadline has expired",
            )
        key = context.execution_key
        arguments_json = json.dumps(request.arguments, sort_keys=True, separators=(",", ":"))
        existing = self._executions.get(key)
        if existing is not None:
            if existing.kind != kind or existing.arguments_json != arguments_json:
                raise _endpoint_error(
                    "FORGE_EXECUTION_CONFLICT",
                    "execution key was already used with different action arguments",
                )
            return ToolAccepted(details={"goal_id": existing.goal_id, "idempotent": True})
        if self._active is not None:
            raise _endpoint_error(
                "BUSY",
                "motion server already has an active goal",
                retryable=True,
            )
        try:
            goal = _goal(kind, request.arguments)
        except ToolEndpointError:
            raise
        except (TypeError, ValueError) as exc:
            raise _endpoint_error("MOTION_INVALID_GOAL", str(exc)) from exc
        goal_id = goal_id_for_key(key)
        execution = _Execution(key, kind, goal_id, arguments_json, events)
        self._executions[key] = execution
        self._by_goal_id[goal_id] = key
        self._active = key
        try:
            self._send_goal(kind, goal_id, goal)
        except Exception:
            self._executions.pop(key, None)
            self._by_goal_id.pop(goal_id, None)
            self._active = None
            raise
        execution.phase = "running"
        return ToolAccepted(details={"goal_id": goal_id, "idempotent": False})

    def begin_shutdown(self) -> ActionShutdownOutcome:
        """Stop admission and request cancellation without claiming terminal completion."""
        self._accepting = False
        if self._active is None:
            return ActionShutdownOutcome(
                active_goal_id=None,
                cancel_requested=False,
                terminal_confirmed=True,
            )
        execution = self._executions[self._active]
        if execution.result is not None:
            return ActionShutdownOutcome(
                active_goal_id=execution.goal_id,
                cancel_requested=False,
                terminal_confirmed=True,
            )
        if execution.phase == "stopping":
            return ActionShutdownOutcome(
                active_goal_id=execution.goal_id,
                cancel_requested=True,
                terminal_confirmed=False,
            )
        try:
            self._send_cancel(execution.kind, execution.goal_id)
        except Exception:
            logger.exception("failed to request cancellation during Action endpoint shutdown")
            return ActionShutdownOutcome(
                active_goal_id=execution.goal_id,
                cancel_requested=False,
                terminal_confirmed=False,
            )
        execution.phase = "stopping"
        return ActionShutdownOutcome(
            active_goal_id=execution.goal_id,
            cancel_requested=True,
            terminal_confirmed=False,
        )

    async def cancel(
        self, key: ToolExecutionKey, reason: str | None = None
    ) -> ToolControlResponse:
        execution = self._executions.get(key)
        if execution is None:
            return ToolControlResponse(
                command="cancel",
                status="rejected",
                error=ToolError(
                    code="FORGE_EXECUTION_NOT_FOUND",
                    message="execution key is unknown",
                ),
            )
        if execution.result is not None:
            return ToolControlResponse(command="cancel", status="terminal")
        if execution.phase != "stopping":
            self._send_cancel(execution.kind, execution.goal_id)
            execution.phase = "stopping"
        return ToolControlResponse(
            command="cancel",
            status="accepted",
            details={"goal_id": execution.goal_id, "reason": reason},
        )

    async def status(self, key: ToolExecutionKey) -> ToolExecutionStatus:
        execution = self._executions.get(key)
        if execution is None:
            return ToolExecutionStatus(
                phase="unknown",
                error=ToolError(
                    code="FORGE_EXECUTION_NOT_FOUND",
                    message="execution key is unknown or its result expired",
                ),
            )
        return ToolExecutionStatus(
            phase=execution.phase,  # type: ignore[arg-type]
            error=execution.result.error
            if execution.phase in ("failed", "unknown") and execution.result is not None
            else None,
            details={"goal_id": execution.goal_id, "operation": execution.kind},
        )

    async def result(self, key: ToolExecutionKey) -> ToolResultResponse:
        execution = self._executions.get(key)
        if execution is None:
            return ToolResultResponse(status="not_found")
        if execution.result is None:
            return ToolResultResponse(status="pending")
        return ToolResultResponse(status="available", result=execution.result)

    async def handle_feedback(
        self, goal_id: str, feedback: MoveJointsFeedback | MovePoseFeedback
    ) -> None:
        execution = self._execution_for_goal(goal_id)
        if execution is None or execution.result is not None:
            return
        now_ns = self._clock_ns()
        if (
            execution.last_event_ns is not None
            and now_ns - execution.last_event_ns < self._feedback_interval_ns
        ):
            return
        execution.last_event_ns = now_ns
        event = ToolEvent(
            type="progress",
            data={
                "goal_id": goal_id,
                "phase": feedback.phase,
                "progress": feedback.progress,
                "elapsed_ms": feedback.elapsed_ns / 1_000_000,
                "message": feedback.message,
            },
        )
        try:
            await execution.emitter.emit(event)
        except Exception:
            logger.exception("failed to emit advisory motion progress event")

    async def handle_result(
        self,
        goal_id: str,
        result: MoveJointsResult | MovePoseResult,
        goal_status: str,
    ) -> None:
        execution = self._execution_for_goal(goal_id)
        if execution is None or execution.result is not None:
            return
        outputs = {"goal_id": goal_id, "motion_result": _jsonable(result)}
        code = result.error_code
        if goal_status == "succeeded" and code == "SUCCESS":
            execution.phase = "completed"
            execution.result = ToolResult(status="succeeded", outputs=outputs)
            event_type = "executor_completed"
        elif goal_status == "canceled" or code == "CANCELED":
            execution.phase = "cancelled"
            execution.result = ToolResult(status="cancelled", outputs=outputs)
            event_type = "cancelled"
        else:
            error = ToolError(code=code, message=result.message, retryable=code == "BUSY")
            execution.phase = "failed"
            execution.result = ToolResult(status="failed", outputs=outputs, error=error)
            event_type = "executor_failed"
        if self._active == execution.key:
            self._active = None
        try:
            await execution.emitter.emit(
                ToolEvent(type=event_type, data={"goal_id": goal_id, "error_code": code})
            )
        except Exception:
            logger.exception("failed to emit advisory motion terminal event")
        self._executions.move_to_end(execution.key)
        self._trim_cache()

    def _execution_for_goal(self, goal_id: str) -> _Execution | None:
        key = self._by_goal_id.get(goal_id)
        return None if key is None else self._executions.get(key)

    def _trim_cache(self) -> None:
        terminal = sum(item.result is not None for item in self._executions.values())
        while terminal > self._result_cache_size:
            key, execution = next(
                (key, item) for key, item in self._executions.items() if item.result is not None
            )
            self._executions.pop(key)
            self._by_goal_id.pop(execution.goal_id, None)
            terminal -= 1


__all__ = [
    "ACTION_DESCRIPTOR",
    "ACTION_ENDPOINT_ID",
    "ActionShutdownOutcome",
    "MotionActionEndpoint",
    "goal_id_for_key",
]
