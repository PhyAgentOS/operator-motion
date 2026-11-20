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

Compatibility: current dependency declarations are `dora-rs==0.4.1`, `forge-kinematics==1.0.1`, `forge-msgs==1.0.1`, and `scipy>=1.14,<2`. See [pyproject.toml](packages/motion_server/pyproject.toml) and [uv.lock](uv.lock). The Tool protocol implementation is vendored in this repository. These declarations do not guarantee cross-version interoperability; also check the build revision and Skill lock when using artifacts.

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

`PYTHONPATH=src` exposes the vendored `forge-tool` protocol under
`src/forge_tool`; see `THIRD_PARTY_NOTICES.md` for its provenance. Optional
standalone executables can be built with `scripts/build_pyinstaller.sh`.

## License

[Apache License 2.0](LICENSE). Runtime dependencies are not distributed with
this source repository and retain their respective licenses; see
[NOTICE](NOTICE) and [THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md).
Review the complete bundled artifact before publishing standalone executables.

## Versioned releases

**Mandatory tag-first release rule:** merge the version/lock change, create an annotated version tag on that reviewed commit, build that exact tag, validate the artifacts, then publish. Moving branch names, untagged commit IDs and replacement of published tags are not release inputs.

| Release line | Source | Node/package version | Runtime |
| --- | --- | --- | --- |
| Forge 1.x | `stable/1.x` | `1.x`, tag `v1.X.Y` | Forge 1.x / Dora 0.4.x |
| Forge 2.x | `master` | `2.x`, tag `v2.X.Y` | Forge 2.x / Dora 1.x |

**Legacy normalization:** nodes previously at 0.x on `stable/1.x` release as **1.0.0**, retaining Forge 1.x / Dora 0.4.x dependencies and runtime behavior. Existing 1.x package versions are retained. Node/package majors now follow the Forge compatibility line; a 0.x tag is rejected.

Run **Build versioned release** from `master` for either line. First merge the version/lock changes into the selected source branch, then create an unused **annotated** tag on that reviewed commit. The tag must match package metadata. Never move existing tags or reuse a published version. Dispatch uses the workflow's reviewed tooling with the exact tagged source, so legacy tags need not contain the new release helper.

```bash
# Replace with an existing, reviewed and unpublished tag matching source metadata.
TAG=v1.0.0
gh workflow run release.yml --ref master -f release_line=1.x -f tag="$TAG"
TAG=v2.X.Y
gh workflow run release.yml --ref master -f release_line=2.x -f tag="$TAG"
```

The workflow verifies tag type, source commit, branch ancestry, source version and locked Forge/Dora compatibility. It builds source archives and wheel/sdist where supported, checks Python distribution metadata and SHA-256, and creates or attaches to a **draft GitHub Release**. Published releases/assets are not overwritten; review all build results and acceptance evidence before publishing. This workflow does not publish node packages to PyPI.

Set `build_binaries=true` to call the existing Ubuntu 20.04 binary workflow; its existing architecture, CLI and archive checks remain in force. You can also run `build-ubuntu20.yml` directly with `ref`, `version` and `release_line`; `ref` must be an annotated tag, not a moving branch. Binary artifacts are attached only after all selected jobs succeed.

The source archive preserves the tagged tree; external SDKs/submodules, model downloads and site configuration remain separate prerequisites. A dependency or SDK build failure is not successful release acceptance. Build success does not establish single-node or Gateway-chain `dora run`, hardware, real-model, sustained-run or rollback acceptance. Every acceptance chain must include `adapter-forge-gateway` from the same compatibility line. Forge 2.x is a **breaking change**: align Forge libraries, node versions and Dora across the entire deployment; keep 1.x on its maintenance branch.

## Dependency maintenance

This maintenance line uses Forge Msgs 1.3.1 and Pillow 13 while retaining Forge 1.x and Dora 0.4.x compatibility. Dependency vulnerability auditing remains enabled.
