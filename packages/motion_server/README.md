# Forge Motion Server

A Dora-native `MoveJoints` and `MovePose` action server. It validates fresh robot state, plans a
smooth joint trajectory, delegates execution through `FollowJointTrajectory`, propagates cancel and
feedback, and verifies final joint or Cartesian residuals.

The server never emits `JointCommand`; actuator-facing execution remains the responsibility of a
joint trajectory controller.

## Architecture

```text
MoveJoints / MovePose
        -> MotionServer
        -> FollowJointTrajectory
        -> JointTrajectoryController
        -> JointCommand
        -> robot driver
```

`MovePose` uses `forge-kinematics` with the latest complete joint state as its IK seed. FK and IK use
the exact configured `base_frame -> tip_frame` relationship; the server never applies implicit tool
length compensation.

## Dora ports

Inputs:

- `tick`
- `joint_state`: `forge_msgs.JointState`
- `move_joints_goal`, `move_joints_cancel`
- `move_pose_goal`, `move_pose_cancel`
- `trajectory_feedback`, `trajectory_result` from the child controller action

Outputs:

- `move_joints_feedback`, `move_joints_result`
- `move_pose_feedback`, `move_pose_result`
- `trajectory_goal`, `trajectory_cancel`

Every Action event uses Dora `goal_id` metadata. Results additionally carry lowercase
`goal_status = succeeded | aborted | canceled`. Outbound Dora metadata is flat, while inbound
handling accepts both flat metadata and a `parameters` envelope for compatibility.

## Version 0.1 behavior

- one active motion goal across all configured groups;
- complete, exactly ordered joint goals; partial-joint goals are rejected;
- fresh complete state is required, while incoming `JointState` may contain extra/reordered joints;
- a goal is rejected if measured start velocity exceeds the configured tolerance;
- URDF position limits are authoritative through `forge-kinematics`;
- velocity and acceleration limits come from motion configuration;
- synchronized quintic time scaling generates complete positions and velocities with zero endpoint
  velocity and acceleration;
- a requested duration below the feasible minimum is rejected rather than silently changed;
- cancel is retried to close goal/cancel delivery races and becomes terminal only after the child
  trajectory action reports canceled;
- child rejection and child execution failure map to distinct parent error codes;
- missing child results trigger bounded execution timeout and downward cancellation;
- child success enters a short settling window so a final `JointState` arriving on another Dora edge
  can be used before joint or FK pose residual validation;
- BUSY, active duplicate, same-ID conflict, bounded result cache, and terminal retry semantics match
  the Forge joint trajectory controller.

This package does not provide collision checking, a planning scene, Cartesian paths, OMPL,
streaming servo control, or hard real-time motor control.

## Run

```bash
forge-motion-server --config motion_server.yaml
```

See `motion/examples/piper_fake_plant/` for a real four-process Dora smoke using Piper URDF
FK/IK, the Forge joint trajectory controller, and a fake position plant.

## Tool policy node

The Motion Action policy node exposes `motion.server/move_joints|move_pose` through the Forge Tool
protocol without changing `MotionServer` semantics:

```text
Forge Action Tool Wire
        -> MotionActionEndpoint
        -> Dora MoveJoints / MovePose Action
        -> MotionServer
             -> IK and motion planning
             -> FollowJointTrajectory orchestration
             -> settling and residual validation
```

`MotionActionEndpoint` is a protocol adapter: it owns Tool request parsing,
`(invocation_id, attempt_id)` lifecycle, status/result/cancel/event mapping, and conversion to Dora
Motion goals. It does not perform FK, IK, trajectory planning, controller orchestration, or final
residual validation. Those remain exclusively in `MotionServer`.

Unlike the one-shot Relative Pose Query, the existing Dora Motion Action is retained as a separate
process boundary so native Dora callers can use MotionServer without Forge Tool/Gateway
dependencies. This is a deployment decision, not a different layering model: both providers use an
Endpoint adapter over a protocol-independent domain implementation.

```bash
forge-motion-action-policy
```

It implements start, cancel, status, result, and rate-limited events while delegating all planning
and execution to the existing MotionServer Dora actions.

See [TOOL_ENDPOINTS.md](TOOL_ENDPOINTS.md) for the Action descriptor, strict arguments, outputs, and
ToolSpec example. Relative-to-absolute pose resolution is owned by the standalone
[relative pose policy node](../../../policy_node/relative_pose_policy_node/README.md); its
[Query contract](../../../policy_node/relative_pose_policy_node/TOOL_CONTRACTS.md) documents the
relative input and absolute target-pose result.
