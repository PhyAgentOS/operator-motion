# Forge Motion

[English](README.md) | [简体中文](README_zh.md)

## BREAKING CHANGE：Forge 2.x / Dora 1.x 迁移

本次从 Forge 1.x / Dora 0.4.x 升级需要协同部署。节点对接前必须对齐 Forge 包、Dora CLI/coordinator/daemon、本节点及上下游节点的版本；不支持在 Forge 1.x 部署中只替换单个节点。可执行产物也需要使用匹配环境重新构建和部署。仍使用旧技术栈的部署请保留在 `stable/1.x`。

本轮验证的 Python 基线为 `forge-msgs==2.1.0`、`Pillow==13.0.0`、`forge-common==2.1.0`、按需使用的 `forge-robot` / `forge-policy` / `forge-kinematics` / `forge-tool` 为 `2.0.0`，以及 `dora-rs==1.0.1`。同时遵守本仓库的 Python 和 SDK 平台要求。Forge 各包族独立发布，无需强行使用同一个版本号。Rust 节点继续使用已发布的 `forge_msgs` crate `2.0.0` 和 Dora `1.0.1`；Python Msgs `2.1.0` 不要求升级到尚未发布的 Rust `2.1.0`。消息命名空间 `forge_msgs.v1` 保持不变。

这里的 breaking change 指旧技术栈的部署迁移，不代表真机、真实模型或长时间运行已完成验收。实际测试和未完成项见[本轮验证报告](migration/msgs-2.1.0-report.json)。

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
0.1.1 使用有界 IK 自动回退，并将数值位置求解精度与实际动作到位容差分开。
另提供不访问硬件的 JSON-lines 预检程序 `python -m forge_motion_server.route_preflight`；
它不新增生产 Node、Endpoint 或 Tool operation。
两者是两个 Node，不能因 Endpoint 名含 `server` 而把 provider 路由接到计算 Node。
参数、单位与校验以 [Action endpoint](packages/motion_server/src/forge_motion_server/action_endpoint.py)
和 [motion_server 包说明](packages/motion_server/README.md)为准；关节值按关节类型使用 rad/m，位姿位置为 m。

兼容性：当前依赖声明为 `dora-rs>=1.0.1,<2`、`forge-kinematics>=2.0.0,<3`、`forge-msgs>=2.0.0,<3`、`scipy>=1.14,<2`；以 [pyproject.toml](packages/motion_server/pyproject.toml) 与 [uv.lock](uv.lock) 为准。 Tool 协议实现来自正式 `forge-tool[dora]` 分发包。 这些声明不构成跨版本互操作保证；使用制品时还需核对 build revision 和 Skill lock。

后续修改 Node 入口、Endpoint 或 Tool API 时，请同步更新本节及对应配置/契约，遵循 [Node README 规范](https://github.com/PhyAgentOS/forge/blob/master/CONTRIBUTING.md#node-readme-contract)。

## 包组成

- `packages/motion_server`：新鲜状态校验、Forge FK/IK、关节空间时间参数化、
  Dora 父子 Action 编排、取消传播及最终残差检查。

## 职责范围

Motion 决定配置的运动学组如何到达关节或笛卡尔目标，
不承担物体操作语义、硬件协议、电机闭环或底层 `JointCommand` 生成。
可选预检程序在 MotionServer Action 核心之外解释应用层 waypoint 名称，
检查顺序 IK 和采样 TCP 高度，不提供全机器人碰撞检查。

默认规划总预算为 1000 ms，数值 IK 位置精度为
`min(动作到位容差 * 0.5, 0.002)` 米，姿态代价权重 0.01、DLS 阻尼 0.1。
最多八次数值搜索，每次上限 80 ms，并共享总预算。省略配置即使用这些默认值，
显式配置仍可覆盖。详见[配置与回退说明](packages/motion_server/README.md#ik-defaults-and-fallback)。

完整 Piper 仿真对象联调示例见 `examples/piper_fake_plant/`。

## 开发

使用 Python 3.12 和 `uv`：

```bash
uv sync --frozen --all-groups
PYTHONPATH=src uv run pytest -q
```

Tool 协议由正式 `forge-tool[dora]` 分发包提供；`THIRD_PARTY_NOTICES.md` 保留旧副本的来源记录。
可选独立可执行文件通过 `scripts/build_pyinstaller.sh` 构建。

## Linux 发布制品

手动运行 **Build Ubuntu 20.04 binaries** 工作流时，`ref` 可指定提交、分支或标签；
工作流校验两个包的版本都与 `version` 一致，在 Ubuntu 20.04 中构建并检查两个 CLI。
输出包括合并压缩包，以及分别仅包含 `motion_server`、`motion_action_policy` 的单可执行文件压缩包。
发布附件同时包含 SHA-256 校验值、精确源码提交和锁定的运行依赖清单。
单 Node 安装使用独立压缩包，合并包包含两个程序。构建成功不代表独立的依赖审计已通过。
0.1.1 的 Pillow 12.3.0 限制已在本 Forge 2.x 分支通过 Msgs 2.1.0 / Pillow 13.0.0 解除。
本轮结果见上方报告；[此前验证更新](OPEN_SOURCE_AUDIT.md#011-verification-update-2026-11-09) 保留为历史记录。

## 许可证

本仓库使用 [Apache License 2.0](LICENSE)。运行时依赖不随源码仓库分发，
保留各自许可证，详见 [NOTICE](NOTICE) 与 [THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md)。
发布独立可执行文件前审查完整打包制品。

## Forge 2.x 迁移

本分支准备节点版本 `2.0.0`，适配 Dora 1.x，迁移验证锁定 Dora `1.0.1`。使用正式 Forge 2.x 分发包；Common 至少为 `2.1.0`。此次更新包括依赖锁文件与各独立可执行入口。

每轮报告必须列出仓库、改动和测试证据，包括各节点的 `dora run` 记录，以及包含 `adapter-forge-gateway` 的串联测试。缺少硬件或模型的验收项保持待验证。源码 PR 不等于正式发布；整体验收通过后再同步 GitLab。
