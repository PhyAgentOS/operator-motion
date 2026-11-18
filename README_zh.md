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
0.1.1 使用有界 IK 自动回退，并将数值位置求解精度与实际动作到位容差分开。
另提供不访问硬件的 JSON-lines 预检程序 `python -m forge_motion_server.route_preflight`；
它不新增生产 Node、Endpoint 或 Tool operation。
两者是两个 Node，不能因 Endpoint 名含 `server` 而把 provider 路由接到计算 Node。
参数、单位与校验以 [Action endpoint](packages/motion_server/src/forge_motion_server/action_endpoint.py)
和 [motion_server 包说明](packages/motion_server/README.md)为准；关节值按关节类型使用 rad/m，位姿位置为 m。

兼容性：当前依赖声明为 `dora-rs==0.4.1`、`forge-kinematics==1.0.1`、`forge-msgs==1.0.1`、`scipy>=1.14,<2`；以 [pyproject.toml](packages/motion_server/pyproject.toml) 与 [uv.lock](uv.lock) 为准。 Tool 协议实现为仓库内 vendored 代码。 这些声明不构成跨版本互操作保证；使用制品时还需核对 build revision 和 Skill lock。

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

`PYTHONPATH=src` 暴露 `src/forge_tool` 中内置的 Tool 协议；来源见 `THIRD_PARTY_NOTICES.md`。
可选独立可执行文件通过 `scripts/build_pyinstaller.sh` 构建。

## Linux 发布制品

手动运行 **Build Ubuntu 20.04 binaries** 工作流时，`ref` 可指定提交、分支或标签；
工作流校验两个包的版本都与 `version` 一致，在 Ubuntu 20.04 中构建并检查两个 CLI。
输出包括合并压缩包，以及分别仅包含 `motion_server`、`motion_action_policy` 的单可执行文件压缩包。
发布附件同时包含 SHA-256 校验值、精确源码提交和锁定的运行依赖清单。
单 Node 安装使用独立压缩包，合并包包含两个程序。构建成功不代表独立的依赖审计已通过。
0.1.1 保留了已有的 Pillow 12.3.0 依赖，当前依赖审计未通过，详见
[验证更新](OPEN_SOURCE_AUDIT.md#011-verification-update-2026-11-09)。

## 许可证

本仓库使用 [Apache License 2.0](LICENSE)。运行时依赖不随源码仓库分发，
保留各自许可证，详见 [NOTICE](NOTICE) 与 [THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md)。
发布独立可执行文件前审查完整打包制品。

## 分版本发布

**发布规则（强制）：** 合并版本/锁文件修改 → 在已审查提交上创建 annotated 版本标签 → 从该标签精确构建 → 校验产物 → 发布。禁止使用移动分支、未打标签的提交构建 Release，禁止覆盖已发布标签。

| 维护线 | 源码 | 节点/包版本 | 运行时 |
| --- | --- | --- | --- |
| Forge 1.x | `stable/1.x` | `1.x`，标签 `v1.X.Y` | Forge 1.x / Dora 0.4.x |
| Forge 2.x | `master` | `2.x`，标签 `v2.X.Y` | Forge 2.x / Dora 1.x |

**旧版版本号统一：** `stable/1.x` 原为 0.x 的节点以 **1.0.0** 发布，保留 Forge 1.x / Dora 0.4.x 依赖与运行行为。已有 1.x 版本保留。节点/包主版本与 Forge 兼容线一致，不再接受 0.x 发布标签。

两条线均从 `master` 运行 **Build versioned release**。先将版本和锁文件修改通过 PR 合并至对应源码分支，再在已审查提交上创建未占用的 **annotated** 标签，标签必须匹配包元数据。禁止移动已有标签或复用已发布版本。工作流使用自身已审查的发布工具与精确标签源码，旧标签无需包含新发布脚本。

```bash
# 替换为已存在、已审查、未发布且与源码元数据一致的标签。
TAG=v1.0.0
gh workflow run release.yml --ref master -f release_line=1.x -f tag="$TAG"
TAG=v2.X.Y
gh workflow run release.yml --ref master -f release_line=2.x -f tag="$TAG"
```

流水线校验标签类型、源码提交、分支归属、源码版本及锁定的 Forge/Dora 兼容性；构建源码归档及项目支持的 wheel/sdist，检查 Python 分发元数据与 SHA-256，并创建或附加至 **GitHub Release 草稿**。不覆盖已发布 Release 或产物；发布前检查全部构建结果与验收依据。该流程不发布节点包至 PyPI。

设置 `build_binaries=true` 复用已有 Ubuntu 20.04 二进制工作流，保留原架构、CLI 和归档校验。也可直接运行 `build-ubuntu20.yml`，传入 `ref`、`version`、`release_line`；`ref` 必须为 annotated 标签，不能为移动分支。仅在所有选定任务通过后附加二进制产物。

源码归档保留标签中的文件；外部 SDK/submodule、模型下载与现场配置仍是独立前置条件。依赖或 SDK 构建失败不能视为发布验收通过。构建成功不代表单节点或串联 `dora run`、真机、真实模型、长稳态或回滚验收通过；每条验收链必须包含相同兼容线的 `adapter-forge-gateway`。Forge 2.x 属于 **breaking change**，整个部署的 Forge 库、节点版本与 Dora 必须对齐；1.x 保留在维护分支。
