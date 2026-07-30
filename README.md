# Forge Motion

Dora-native motion planning and execution orchestration for Forge robots.

```text
Manipulation / application goals
        -> MoveJoints / MovePose
        -> MotionServer
        -> FollowJointTrajectory
        -> actuator-facing controllers
```

The repository is a UV workspace of independently installable motion packages. Version 0.1 starts
with `forge-motion-server`; shared trajectory processing, collision checking, planning scenes, and
streaming servo packages should only be extracted when real additional consumers require them.

## Packages

- `packages/motion_server`: fresh-state validation, Forge FK/IK, joint-space time
  parameterization, parent/child Dora Action orchestration, cancel propagation, and final residual
  checking.

## Scope

Motion determines how a configured kinematic group reaches a joint or Cartesian target. It does not
contain object/manipulation semantics, hardware protocols, motor loops, or low-level
`JointCommand` generation.

See `examples/piper_fake_plant/` for the full Piper fake-plant integration smoke.
