"""Dora runners for the independent Motion Tool policy nodes."""

from __future__ import annotations

import asyncio
import logging
import time
import uuid
from collections.abc import Callable, Iterable
from typing import Any

import pyarrow as pa
from forge_msgs import (
    MoveJointsFeedback,
    MoveJointsGoal,
    MoveJointsResult,
    MovePoseFeedback,
    MovePoseGoal,
    MovePoseResult,
)

from forge_tool import ToolEndpointHandler, ToolEnvelope
from forge_tool._tool_message import ToolMessage
from forge_tool.dora import (
    DoraToolEndpointBinding,
    tool_envelope_to_message,
    tool_message_to_envelope,
)

from .action_endpoint import ACTION_DESCRIPTOR, MotionActionEndpoint
from .endpoint_lease import EndpointLease

logger = logging.getLogger(__name__)


def _parameters(event: dict[str, Any]) -> dict[str, Any]:
    metadata = event.get("metadata")
    if not isinstance(metadata, dict):
        return {}
    parameters = metadata.get("parameters")
    return parameters if isinstance(parameters, dict) else metadata


class _PolicyRunner:
    def __init__(
        self,
        node: Any,
        descriptor: Any,
        operations: dict[str, Any],
        *,
        endpoint_instance_id: str | None = None,
        clock: Callable[[], float] = time.monotonic,
    ) -> None:
        self.node = node
        self.endpoint_instance_id = endpoint_instance_id or str(uuid.uuid4())
        self.clock = clock
        self._async = asyncio.Runner()
        self.handler = ToolEndpointHandler(
            descriptor,
            endpoint_instance_id=self.endpoint_instance_id,
            operations=operations,
        )
        self.binding = DoraToolEndpointBinding(
            self.handler,
            event_sink=self._send_batch,
        )
        self.lease = EndpointLease(descriptor, self.endpoint_instance_id)

    async def _send_batch(self, batch: pa.RecordBatch) -> None:
        self.node.send_output("tool_out", batch)

    def _send_envelope(self, envelope: ToolEnvelope) -> None:
        self.node.send_output("tool_out", tool_envelope_to_message(envelope).to_arrow())

    def _drive_lease(self) -> None:
        request = self.lease.poll(self.clock())
        if request is not None:
            self._send_envelope(request)

    def _handle_tool(self, value: object) -> None:
        message = ToolMessage.from_arrow(value)
        envelope = tool_message_to_envelope(message)
        if envelope.message_type == "endpoint.registry.response":
            self.lease.acknowledge(envelope, self.clock())
            return
        batches = self._async.run(self.binding.dispatch_input(value))
        for batch in batches:
            self.node.send_output("tool_out", batch)

    def _prepare_stop(self) -> None:
        """Stop business admission before the endpoint lease is removed."""

    def _base_event(self, event: dict[str, Any]) -> int | None:
        event_type = event.get("type")
        if event_type == "STOP":
            self._prepare_stop()
            unregister = self.lease.stop()
            if unregister is not None:
                self._send_envelope(unregister)
            self._async.close()
            return 0
        if event_type == "ERROR":
            logger.error("Dora node error: %s", event.get("error", "unknown"))
            self._async.close()
            return 1
        return None

    def run(self, events: Iterable[dict[str, Any]]) -> int:
        self._drive_lease()
        for event in events:
            result = self.process_event(event)
            if result is not None:
                return result
        return 0

    def process_event(self, event: dict[str, Any]) -> int | None:
        raise NotImplementedError


class MotionActionPolicyRunner(_PolicyRunner):
    def __init__(
        self,
        node: Any,
        *,
        endpoint: MotionActionEndpoint | None = None,
        endpoint_instance_id: str | None = None,
        clock: Callable[[], float] = time.monotonic,
    ) -> None:
        self.endpoint = endpoint or MotionActionEndpoint(self._send_goal, self._send_cancel)
        super().__init__(
            node,
            ACTION_DESCRIPTOR,
            {"move_joints": self.endpoint, "move_pose": self.endpoint},
            endpoint_instance_id=endpoint_instance_id,
            clock=clock,
        )

    def _send_goal(
        self,
        kind: str,
        goal_id: str,
        goal: MoveJointsGoal | MovePoseGoal,
    ) -> None:
        self.node.send_output(f"{kind}_goal", goal.to_arrow(), metadata={"goal_id": goal_id})

    def _send_cancel(self, kind: str, goal_id: str) -> None:
        self.node.send_output(
            f"{kind}_cancel",
            pa.array([True], type=pa.bool_()),
            metadata={"goal_id": goal_id},
        )

    def _prepare_stop(self) -> None:
        outcome = self.endpoint.begin_shutdown()
        if outcome.active_goal_id is not None and not outcome.terminal_confirmed:
            logger.warning(
                (
                    "Action endpoint shutdown is ambiguous for goal_id=%s: "
                    "cancel_requested=%s; cancellation admission is not terminal confirmation"
                ),
                outcome.active_goal_id,
                outcome.cancel_requested,
            )

    def process_event(self, event: dict[str, Any]) -> int | None:
        terminal = self._base_event(event)
        if terminal is not None:
            return terminal
        if event.get("type") != "INPUT":
            return None
        self._drive_lease()
        input_id = event.get("id")
        try:
            if input_id == "tool_in":
                self._handle_tool(event["value"])
            elif input_id in ("move_joints_feedback", "move_pose_feedback"):
                goal_id = str(_parameters(event).get("goal_id", ""))
                feedback = (
                    MoveJointsFeedback.from_arrow(event["value"])
                    if input_id == "move_joints_feedback"
                    else MovePoseFeedback.from_arrow(event["value"])
                )
                self._async.run(self.endpoint.handle_feedback(goal_id, feedback))
            elif input_id in ("move_joints_result", "move_pose_result"):
                parameters = _parameters(event)
                goal_id = str(parameters.get("goal_id", ""))
                goal_status = str(parameters.get("goal_status", ""))
                result = (
                    MoveJointsResult.from_arrow(event["value"])
                    if input_id == "move_joints_result"
                    else MovePoseResult.from_arrow(event["value"])
                )
                self._async.run(self.endpoint.handle_result(goal_id, result, goal_status))
        except (TypeError, ValueError) as exc:
            logger.warning("ignored invalid action policy input: %s", exc)
        self._drive_lease()
        return None


def run_action_policy() -> int:
    from dora import Node

    node = Node()
    return MotionActionPolicyRunner(node).run(node)


__all__ = [
    "MotionActionPolicyRunner",
    "run_action_policy",
]
