from __future__ import annotations

import asyncio

from forge_motion_server.action_endpoint import ACTION_DESCRIPTOR, MotionActionEndpoint
from forge_motion_server.endpoint_lease import EndpointLease
from forge_msgs import MoveJointsResult

from forge_tool import (
    EndpointRegistryResponse,
    ToolAccepted,
    ToolContext,
    ToolEndpointHandler,
    ToolExecutionKey,
    ToolRequest,
    control_response_from_payload,
    invoke_response_from_payload,
    make_control_request_envelope,
    make_endpoint_registry_response_envelope,
    make_invoke_request_envelope,
    make_result_request_envelope,
    make_status_request_envelope,
    result_response_from_payload,
    status_response_from_payload,
    validate_registration_envelope,
)
from forge_tool._tool_message import ToolMessage
from forge_tool.dora import (
    DoraToolEndpointBinding,
    tool_envelope_to_message,
    tool_message_to_envelope,
)


def _context() -> ToolContext:
    return ToolContext(
        execution_key=ToolExecutionKey("invocation", "attempt"),
        tool_id="motion.move_joints",
        implementation_id="motion",
        endpoint_id="motion.server",
        operation="move_joints",
    )


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


def _arrow(envelope):
    return tool_envelope_to_message(envelope).to_arrow()


def _envelope(batch):
    return tool_message_to_envelope(ToolMessage.from_arrow(batch))


def test_public_action_binding_handles_lifecycle_and_events() -> None:
    sent_goals = []
    sent_cancels = []
    events = []
    endpoint = MotionActionEndpoint(
        lambda *args: sent_goals.append(args),
        lambda *args: sent_cancels.append(args),
    )
    handler = ToolEndpointHandler(
        ACTION_DESCRIPTOR,
        endpoint_instance_id="instance-1",
        operations={"move_joints": endpoint, "move_pose": endpoint},
    )
    binding = DoraToolEndpointBinding(
        handler,
        event_sink=lambda batch: _append_async(events, batch),
    )
    context = _context()
    invoke = make_invoke_request_envelope(
        _request(),
        context,
        request_id="invoke-1",
        endpoint_instance_id="instance-1",
    )

    invoke_batches = asyncio.run(binding.dispatch_input(_arrow(invoke)))
    assert invoke_batches == ()
    assert len(events) == 1
    invoke_response = _envelope(events.pop(0))
    accepted = invoke_response_from_payload(invoke_response.payload)
    assert isinstance(accepted, ToolAccepted)
    assert len(sent_goals) == 1

    status_request = make_status_request_envelope(
        context,
        request_id="status-1",
        endpoint_instance_id="instance-1",
    )
    status_response = _envelope(asyncio.run(binding.dispatch_input(_arrow(status_request)))[0])
    assert status_response_from_payload(status_response.payload).phase == "running"

    result_request = make_result_request_envelope(
        context,
        request_id="result-1",
        endpoint_instance_id="instance-1",
    )
    pending = _envelope(asyncio.run(binding.dispatch_input(_arrow(result_request)))[0])
    assert result_response_from_payload(pending.payload).status == "pending"

    cancel_request = make_control_request_envelope(
        "cancel",
        context,
        request_id="cancel-1",
        endpoint_instance_id="instance-1",
    )
    cancel_response = _envelope(asyncio.run(binding.dispatch_input(_arrow(cancel_request)))[0])
    assert control_response_from_payload(cancel_response.payload).status == "accepted"
    assert len(sent_cancels) == 1

    result = MoveJointsResult(
        error_code="CANCELED",
        message="canceled",
        elapsed_ns=1,
        joint_names=["j1"],
        final_positions=[0.0],
        final_position_errors=[0.1],
    )
    asyncio.run(endpoint.handle_result(accepted.details["goal_id"], result, "canceled"))
    available = _envelope(asyncio.run(binding.dispatch_input(_arrow(result_request)))[0])
    terminal = result_response_from_payload(available.payload)
    assert terminal.status == "available"
    assert terminal.result is not None and terminal.result.status == "cancelled"
    assert [_envelope(event).message_type for event in events] == ["tool.event"]


async def _append_async(items, item) -> None:
    items.append(item)


def test_endpoint_lease_registers_one_descriptor_and_renews_at_ttl_third() -> None:
    lease = EndpointLease(ACTION_DESCRIPTOR, "instance-1")

    registration = lease.poll(10.0)

    assert registration is not None
    assert registration.message_type == "endpoint.register"
    assert registration.endpoint_id == "motion.server"
    assert validate_registration_envelope(registration) == ACTION_DESCRIPTOR
    response = make_endpoint_registry_response_envelope(
        EndpointRegistryResponse(
            operation="register",
            status="accepted",
            registry_revision=1,
            lease_ttl_ms=3000,
        ),
        registration,
    )
    assert lease.acknowledge(response, 10.1) is True
    assert lease.poll(11.09) is None
    renewal = lease.poll(11.1)
    assert renewal is not None
    assert renewal.payload == registration.payload
