# LW-only 仓库范围

本文档记录 LW-015 确立的仓库边界。它既是文件清单，也是扩展约定；若要新增
其他机器人，仍须单独提出产品范围变更并经过评审，不能将本文档视为直接授权。

## 保留的 LW 实现

| 类别 | 保留路径 |
|---|---|
| 策略 | `policy/LW/` |
| FSM | `src/rl_sar/fsm_robot/fsm_LW.hpp` |
| 实机适配器 | `src/rl_sar/include/rl_real_LW.hpp`、`src/rl_sar/src/rl_real_LW.cpp` |
| Sim2Sim 适配器 | `src/rl_sar/include/rl_sim_LW.hpp`、`src/rl_sar/src/rl_sim_LW.cpp` |
| 硬件 SDK | `src/rl_sar/library/thirdparty/robot_sdk/lfr/LW_sdk/` |
| 机器人描述 | `src/rl_sar_zoo/LW_description/` |
| 启动文件 | `src/rl_sar/launch/rl_real_LW.launch.py` |

LW 机器人描述以主仓库普通文件的形式内置。其来源和导入基线记录在
`src/rl_sar_zoo/README.md` 中，`LW_DESCRIPTION_MANIFEST.sha256` 覆盖该软件包的
全部文件。

## 保留的共享基础设施

- `src/rl_sar/library/core/` 下通用的强化学习配置、推理、观测、循环、日志和
  向量工具；
- 通用 FSM 接口、管理器、工厂和注册宏；
- `src/rl_sar/library/thirdparty/mujoco_simulate/` 下的 MuJoCo 仿真与界面支持；
- 通用仿真适配器和 Gazebo 启动骨架；保留它们不代表仓库仍支持任何已移除的
  机器人；
- 通用 `robot_msgs` 和 `robot_joint_controller` 软件包；它们已从当前 LW 目标的
  必需依赖中解除；
- 推理运行库和 MuJoCo 运行库的获取脚本；
- LW 使用的 `drewnoakes/joystick` 子模块。

## 已移除的具体实现

LW-015 已从策略、FSM 实现、机器人描述以及原有实机适配器中移除以下机器人
系列：

`a1`、`b2`、`b2w`、`g1`、`go2`、`go2w`、`gr1t1`、`gr1t2`、`l4w4`、
`lite3` 和 `tita`。

同时还移除了：

- Unitree 和 DeepRobotics SDK 子模块；
- 已跟踪的 Zhinao SDK；
- 未启用的非 LW CMake 目标定义和 SDK 配置；
- 仅用于 G1 的旧 motion loader 及 G1 专用观测分支；
- 硬编码的 A1 控制器启动示例；
- 用于克隆或更新原多机器人 zoo 的脚本。

## 外部内容和用户所有内容

`.agents/skills/inspect-context-compactions/` 是与 LW-015 无关的用户所有内容。
除非另行明确要求，否则该目录必须保持未跟踪状态。

构建、安装、日志、缓存和临时目录仍属于被忽略的构建产物，不是产品源码。

## 未来机器人扩展约定

未来新增机器人时，必须作为一项完整、连贯且经过评审的变更提交，并包含：

1. 有版本记录的策略和经过验证的配置；
2. 有来源记录且不存在非托管嵌套仓库的机器人描述；
3. 通过现有通用接口注册的机器人专用 FSM 工厂；
4. 机器人专用硬件或仿真适配器，以及它们实际需要的 SDK 依赖；
5. 对仓库范围允许列表和当前支持情况文档的更新；
6. 干净构建、仿真、安全和部署验收证据。

凡非该适配器必需的通用核心修改，均应作为独立评审事项提交。
