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
Version 0.1.1 uses bounded local IK fallback and separates numerical position
accuracy from physical completion tolerance. A hardware-free JSON-lines
preflight helper is available as `python -m forge_motion_server.route_preflight`;
it adds no production Node, Endpoint or Tool operation.
They are separate Nodes: the word `server` in the Endpoint ID does not route the provider to the compute Node.
For arguments, units and validation, see the [Action endpoint](packages/motion_server/src/forge_motion_server/action_endpoint.py)
and [motion_server package guide](packages/motion_server/README.md). Joint values use rad/m according to joint type; pose positions use metres.

Compatibility: current dependency declarations are `dora-rs>=1.0.1,<2`, `forge-kinematics>=2.0.0,<3`, `forge-msgs>=2.0.0,<3`, and `scipy>=1.14,<2`. See [pyproject.toml](packages/motion_server/pyproject.toml) and [uv.lock](uv.lock). The Tool protocol implementation comes from the official `forge-tool[dora]` distribution. These declarations do not guarantee cross-version interoperability; also check the build revision and Skill lock when using artifacts.

Changes to Node entrypoints, Endpoints or Tool APIs must update this section and the associated configuration/contracts. Follow the [Node README contract](https://github.com/PhyAgentOS/forge/blob/master/CONTRIBUTING.md#node-readme-contract).

## Packages

- `packages/motion_server`: fresh-state validation, Forge FK/IK, joint-space time
  parameterization, parent/child Dora Action orchestration, cancel propagation, and final residual
  checking.

## Scope

Motion determines how a configured kinematic group reaches a joint or Cartesian target. It does not
contain object/manipulation semantics, hardware protocols, motor loops, or low-level
`JointCommand` generation.
The optional preflight helper interprets application waypoint labels outside the
MotionServer action core. It checks sequential IK and sampled TCP height, and
does not provide whole-robot collision checking.

The default planning budget is 1000 ms. Numerical IK position tolerance is
`min(completion_position_tolerance * 0.5, 0.002)` metres, with orientation cost
weight 0.01 and DLS damping 0.1. Up to eight numerical attempts, each limited to
80 ms, share the planning budget. These settings are defaults when omitted;
explicit configuration overrides them. See the [configuration and fallback
guide](packages/motion_server/README.md#ik-defaults-and-fallback).

See `examples/piper_fake_plant/` for the full Piper fake-plant integration smoke.

## Development

Use Python 3.12 and `uv`:

```bash
uv sync --frozen --all-groups
PYTHONPATH=src uv run pytest -q
```

Tool protocol code is supplied by the official `forge-tool[dora]` distribution. Optional
standalone executables can be built with `scripts/build_pyinstaller.sh`.

## Linux release artifacts

The manual **Build Ubuntu 20.04 binaries** workflow accepts a commit, branch or tag
in `ref` and checks that both package versions match `version`. It builds and
checks both CLIs in Ubuntu 20.04, then uploads the combined archive and separate
single-executable archives for `motion_server` and `motion_action_policy`.
Release assets also include SHA-256 checksums, the exact source commit and frozen
runtime requirements. Use the separate archives for single-node installation;
the combined archive contains both programs. Build success does not imply that
the independent dependency audit passed. Version 0.1.1 retains an inherited
Pillow 12.3.0 dependency whose audit currently fails; see the
[verification update](OPEN_SOURCE_AUDIT.md#011-verification-update-2026-11-09).

## License

[Apache License 2.0](LICENSE). Runtime dependencies are not distributed with
this source repository and retain their respective licenses; see
[NOTICE](NOTICE) and [THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md).
Review the complete bundled artifact before publishing standalone executables.

## Forge 2.x migration

This branch prepares node version `2.0.0` for Dora 1.x, frozen to Dora `1.0.1` for migration validation. It uses the official Forge 2.x distributions; Common is at least `2.1.0`. Regenerated locks and all executable entrypoints are part of the change.

Every change report must list repository, changes and test evidence, including per-node `dora run` records and a chained test containing `adapter-forge-gateway`. Unavailable hardware/model checks remain pending. Source PRs do not publish stable releases or synchronize GitLab; synchronization follows full acceptance.

ToolEndpoint implementations now come from `forge-tool[dora]>=2.0.0,<3`. The former repository-local Tool copy and private message imports are removed; messages use the public `forge_msgs.ToolMessage`.
