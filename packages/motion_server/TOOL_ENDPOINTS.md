# Motion Tool Endpoints

This package provides only the `motion.server` Action endpoint. Relative-to-absolute pose
resolution is provided by the standalone
[relative pose policy node](../../../policy_node/relative_pose_policy_node/README.md); see its
[Query contract](../../../policy_node/relative_pose_policy_node/TOOL_CONTRACTS.md) for relative
inputs, absolute outputs, frame semantics, and freshness behavior.

## `motion.server`

Descriptor: cancellable Action operations `move_joints` and `move_pose`, status supported,
concurrency 1 each with one shared MotionServer busy domain.

`move_joints` inputs:

- `group_name`, complete ordered `joint_names`, finite `positions`
- positive `velocity_scale`, `acceleration_scale`
- optional positive `requested_duration_ms`

`move_pose` inputs:

- `group_name`, `reference_frame`, `target_frame`
- `target_pose` with exactly `x,y,z,qx,qy,qz,qw`
- positive `velocity_scale`, `acceleration_scale`
- positive `position_tolerance_m`, `orientation_tolerance_rad`
- optional positive `requested_duration_ms`

The endpoint maps `(invocation_id, attempt_id)` deterministically to a bounded `goal_id`.
Duplicate starts observed by the endpoint are effect-idempotent; the common Forge handler owns
transport duplicate suppression and does not define an argument fingerprint. Concurrent different
attempts return retryable `BUSY`. Terminal results are cached. MotionServer error codes such as
`INVALID_GROUP`, `NO_FRESH_ROBOT_STATE`, `PLANNING_FAILED`, `BUSY`, and `CANCELED` remain the
Tool error/result code instead of being collapsed into a generic failure.

Feedback is emitted as low-rate `progress` events (10 Hz by default). Authoritative completion is
always available through Action `status` and `result`; events are advisory.

On policy shutdown, admission closes before the active goal receives a best-effort cancel request.
Cancel admission is not terminal confirmation. If the node exits before a MotionServer result is
observed, the outcome is ambiguous/unknown and must never be reported as stopped or cancelled.

## Endpoint and domain-service boundary

`MotionActionEndpoint` is the Forge Action SPI adapter. It validates the Tool wire shape and owns
Tool execution bookkeeping, but delegates every accepted physical goal to the existing Dora-native
`MotionServer`. IK, planning, controller execution, settling, and residual validation must not move
into the endpoint adapter or the Agent Skill.

The two layers intentionally retain different protocol state:

- Endpoint `_executions` cache: the Forge `status/result` view keyed by `ToolExecutionKey`.
- MotionServer result cache: authoritative Dora Motion Action replay keyed by `goal_id`.
- Endpoint `BUSY`: a fast Forge admission precheck that mirrors the single active motion known to
  this endpoint instance.
- MotionServer `BUSY`: the authoritative motion-domain decision, including goals submitted by
  native Dora callers that bypass the Tool endpoint.

Endpoint argument checks cover the strict Tool schema and conversion to `forge_msgs` goals.
Configured groups, joint ordering, frames, live robot state, reachability, and planning constraints
are validated authoritatively by MotionServer.

This mirrors the relative Query provider's `RelativePoseQueryEndpoint -> RelativePoseResolver`
composition. The Query pair runs in one process because it is a synchronous calculation; the
Motion pair currently preserves the pre-existing Dora Action process boundary for native reuse and
execution isolation.

## ToolSpec examples

```json
{
  "tool_id": "motion.move_pose",
  "implementation": {
    "endpoint_id": "motion.server",
    "operation": "move_pose",
    "semantics": "action"
  },
  "completion": {"type": "endpoint_result", "status": "succeeded"}
}
```

The endpoint node registers one descriptor and renews one endpoint-level Gateway lease. It does not
register one lease per operation.
