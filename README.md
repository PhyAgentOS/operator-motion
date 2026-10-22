# Forge Motion

[English](README.md) | [简体中文](README_zh.md)

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

## Node, Endpoint and Tool API

| Item | Current implementation |
| --- | --- |
| Production Nodes | 2: `motion_server` and `motion_action_policy`; source commands `forge-motion-server` and `forge-motion-action-policy` |
| Auxiliary programs | Examples such as fake plants are excluded from the production Node count |
| Tool integration | Dora `ToolMessage` provider; caller-facing access through Gateway |
| Direct HTTP Tool API | 0; the Nodes do not listen on HTTP; Gateway owns the invocation URL |
| Endpoints | 1: `motion.server`; its provider is `motion_action_policy`, not `motion_server` |
| Operations | 2: `move_joints` and `move_pose` (both Action, cancellable, status supported; each descriptor has maximum concurrency 1) |
| Gateway Tools | Deployment-defined in `tools.specs`; `robot-control-sim` binds 2 Tools to these operations |

| Tool ID (robot-control-sim configuration) | Endpoint / operation | Provider Node |
| --- | --- | --- |
| `sim.set_joint_positions` | `motion.server / move_joints` | `motion_action_policy` |
| `sim.move_eef` | `motion.server / move_pose` | `motion_action_policy` |

```text
Gateway -> motion_action_policy -> motion_server -> joint_trajectory_controller -> Adapter / simulator
```

`motion_action_policy` handles the Tool lifecycle and Wire argument translation;
`motion_server` handles fresh state, FK/IK, trajectories and controller orchestration.
They are separate Nodes: the word `server` in the Endpoint ID does not route the provider to the compute Node.
For arguments, units and validation, see the [Action endpoint](packages/motion_server/src/forge_motion_server/action_endpoint.py)
and [motion_server package guide](packages/motion_server/README.md). Joint values use rad/m according to joint type; pose positions use metres.

Compatibility: current dependency declarations are `dora-rs==0.4.1`, `forge-kinematics==1.0.1`, `forge-msgs==1.0.1`. See [pyproject.toml](packages/motion_server/pyproject.toml) and [uv.lock](uv.lock). The Tool protocol implementation is vendored in this repository. These declarations do not guarantee cross-version interoperability; also check the build revision and Skill lock when using artifacts.

Changes to Node entrypoints, Endpoints or Tool APIs must update this section and the associated configuration/contracts. Follow the [Node README contract](https://github.com/PhyAgentOS/forge/blob/master/CONTRIBUTING.md#node-readme-contract).

## Packages

- `packages/motion_server`: fresh-state validation, Forge FK/IK, joint-space time
  parameterization, parent/child Dora Action orchestration, cancel propagation, and final residual
  checking.

## Scope

Motion determines how a configured kinematic group reaches a joint or Cartesian target. It does not
contain object/manipulation semantics, hardware protocols, motor loops, or low-level
`JointCommand` generation.

See `examples/piper_fake_plant/` for the full Piper fake-plant integration smoke.

## Development

Use Python 3.12 and `uv`:

```bash
uv sync --frozen --all-groups
PYTHONPATH=src uv run pytest -q
```

`PYTHONPATH=src` exposes the vendored `forge-tool` protocol under
`src/forge_tool`; see `THIRD_PARTY_NOTICES.md` for its provenance. Optional
standalone executables can be built with `scripts/build_pyinstaller.sh`.

## License

[Apache License 2.0](LICENSE). Runtime dependencies are not distributed with
this source repository and retain their respective licenses; see
[NOTICE](NOTICE) and [THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md).
Review the complete bundled artifact before publishing standalone executables.
