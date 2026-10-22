# Forge Motion

[English](README.md) | [简体中文](README_zh.md)

面向 Forge 机器人的 Dora 原生运动规划与执行编排。

```text
操作 / 应用目标
        -> MoveJoints / MovePose
        -> MotionServer
        -> FollowJointTrajectory
        -> 面向执行器的控制器
```

仓库是由可独立安装的运动包组成的 uv 工作区。0.1 版本从 `forge-motion-server` 开始；
轨迹处理、碰撞检查、规划场景和流式伺服仅在确有额外使用方时再抽成共享包。

## Node、Endpoint 与 Tool API

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

## 包组成

- `packages/motion_server`：新鲜状态校验、Forge FK/IK、关节空间时间参数化、
  Dora 父子 Action 编排、取消传播及最终残差检查。

## 职责范围

Motion 决定配置的运动学组如何到达关节或笛卡尔目标，
不承担物体操作语义、硬件协议、电机闭环或底层 `JointCommand` 生成。

完整 Piper 仿真对象联调示例见 `examples/piper_fake_plant/`。

## 开发

使用 Python 3.12 和 `uv`：

```bash
uv sync --frozen --all-groups
PYTHONPATH=src uv run pytest -q
```

`PYTHONPATH=src` 暴露 `src/forge_tool` 中内置的 Tool 协议；来源见 `THIRD_PARTY_NOTICES.md`。
可选独立可执行文件通过 `scripts/build_pyinstaller.sh` 构建。

## 许可证

本仓库使用 [Apache License 2.0](LICENSE)。运行时依赖不随源码仓库分发，
保留各自许可证，详见 [NOTICE](NOTICE) 与 [THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md)。
发布独立可执行文件前审查完整打包制品。
