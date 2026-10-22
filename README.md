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

## Node、Endpoint 与 Tool API / Capability summary

| 项目 | 当前实现 |
| --- | --- |
| 生产 Node | 2：`motion_server` 与 `motion_action_policy`；源码命令分别为 `forge-motion-server`、`forge-motion-action-policy` |
| 辅助程序 | fake plant 等示例不计入生产 Node 数 |
| Tool 接入方式 | Dora `ToolMessage` provider；通过 Gateway 暴露调用入口 |
| 直接 HTTP Tool API | 0；本 Node 不监听 HTTP，调用地址属于 Gateway |
| Endpoint 数 | 1：`motion.server`；provider 是 `motion_action_policy`，不是 `motion_server` |
| operation 数 | 2：`move_joints`、`move_pose`（均为 Action，可取消、可查状态；各自 descriptor 最大并发 1） |
| Gateway Tool 数 | 由 `tools.specs` 决定；`robot-control-sim` 为这两个 operation 配置 2 个 Tool |

| Tool ID（robot-control-sim 配置） | Endpoint / operation | provider Node |
| --- | --- | --- |
| `sim.set_joint_positions` | `motion.server / move_joints` | `motion_action_policy` |
| `sim.move_eef` | `motion.server / move_pose` | `motion_action_policy` |

```text
Gateway -> motion_action_policy -> motion_server -> joint_trajectory_controller -> Adapter / simulator
```

`motion_action_policy` 管 Tool 生命周期和 Wire 参数转换；`motion_server` 管新鲜状态、FK/IK、轨迹及控制器编排。
两者是两个 Node，不能因 Endpoint 名含 `server` 而把 provider 路由接到计算 Node。
参数、单位与校验以 [Action endpoint](packages/motion_server/src/forge_motion_server/action_endpoint.py)
和 [motion_server 包说明](packages/motion_server/README.md)为准；关节值按关节类型使用 rad/m，位姿位置为 m。

兼容性：当前依赖声明为 `dora-rs==0.4.1`、`forge-kinematics==1.0.1`、`forge-msgs==1.0.1`；以 [pyproject.toml](packages/motion_server/pyproject.toml) 与 [uv.lock](uv.lock) 为准。 Tool 协议实现为仓库内 vendored 代码。 这些声明不构成跨版本互操作保证；使用制品时还需核对 build revision 和 Skill lock。

后续修改 Node 入口、Endpoint 或 Tool API 时，请同步更新本节及对应配置/契约，遵循 [Node README 规范](https://github.com/PhyAgentOS/forge/blob/master/CONTRIBUTING.md#node-readme-contract)。

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
