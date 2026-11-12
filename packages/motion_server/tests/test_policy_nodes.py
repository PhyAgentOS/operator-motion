from __future__ import annotations

import asyncio

from forge_motion_server.action_endpoint import MotionActionEndpoint
from forge_motion_server.policy_nodes import MotionActionPolicyRunner
from forge_msgs import ToolMessage
from forge_tool import ToolContext, ToolExecutionKey, ToolRequest
from forge_tool.dora import tool_message_to_envelope


class _Events:
    async def emit(self, event) -> None:
        pass


class _Node:
    def __init__(self, calls) -> None:
        self.calls = calls

    def send_output(self, output_id, value, **kwargs) -> None:
        if output_id == "tool_out":
            envelope = tool_message_to_envelope(ToolMessage.from_arrow(value))
            self.calls.append(("tool", envelope.message_type))
        else:
            self.calls.append((output_id, kwargs.get("metadata")))


def _request() -> ToolRequest:
    return ToolRequest(
        {
            "group_name": "arm",
            "joint_names": ["j1"],
            "positions": [0.1],
            "velocity_scale": 0.5,
            "acceleration_scale": 0.5,
        }
    )


def test_action_policy_stop_requests_cancel_before_unregister() -> None:
    calls = []
    endpoint = MotionActionEndpoint(
        lambda *args: None,
        lambda kind, goal_id: calls.append(("cancel", kind, goal_id)),
    )
    key = ToolExecutionKey("invocation", "attempt")
    context = ToolContext(
        execution_key=key,
        tool_id="motion.move_joints",
        implementation_id="motion",
        endpoint_id="motion.server",
        operation="move_joints",
    )
    asyncio.run(endpoint.start(_request(), context, _Events()))
    runner = MotionActionPolicyRunner(
        _Node(calls),
        endpoint=endpoint,
        endpoint_instance_id="instance-1",
    )

    assert runner.process_event({"type": "STOP"}) == 0

    assert calls[0][0] == "cancel"
    assert calls[1] == ("tool", "endpoint.unregister")
    assert asyncio.run(endpoint.status(key)).phase == "stopping"
