# Piper MotionServer Fake Plant Smoke

This example validates the complete motion execution boundary in real Dora processes:

```text
MotionActionClient
  -> MoveJoints / MovePose
  -> MotionServer
  -> FollowJointTrajectory
  -> JointTrajectoryController
  -> JointCommand
  -> FakeJointPlant
  -> JointState
```

The example uses the canonical Piper URDF from `robots/agilex_piper`, including the fixed
`arm_base -> tcp` transform. It first moves from zero to a non-singular seed, then computes a small
Cartesian target through real `forge-kinematics` DLS IK.

The machine-checked smoke covers:

- `MoveJoints` success;
- `MovePose` through real Piper IK;
- final FK residual;
- parent-to-child cancel propagation;
- BUSY across `MoveJoints` and `MovePose`;
- active duplicate without replanning/restarting;
- terminal result cache retry;
- Arrow and Dora Action metadata across four processes;
- one terminal result per execution.

Prepare both workspaces, then run from the `motion` repository root:

```bash
uv sync --locked
uv run python examples/piper_fake_plant/smoke_test.py
```

If `dora` is not on `PATH`, pass `--dora /path/to/dora` or set `DORA_CLI`. The example expects the
sibling `controls`, `forge`, and `robots` repositories in the standard `framework/` checkout.
Generated Dora `out/` sessions and `smoke-result.json` are ignored.
