# rl_sar — LW

`rl_sar` 当前仅支持 LW 机器人，用于强化学习策略的 Sim2Sim 验证与实机部署。
仓库保留通用 FSM、推理、控制循环和仿真接口，以便以后在明确扩大产品范围时添加
新机器人，而无需恢复旧机器人实现。

[English](README.md)

> [!CAUTION]
> 实机运行可能造成机器人跌倒、设备损坏或人员受伤。必须先完成文档规定的
> Sim2Sim 和部署验收，准备机械支撑与物理急停，并保持安全隔离区域。

## 当前支持范围

| 机器人 | 策略运行时 | Sim2Sim | 实机 |
|---|---|---|---|
| LW | ONNX Runtime | MuJoCo / `rl_sim_LW` | ROS 2 / `rl_real_LW` |

当前跟踪四套正式 LW 策略：

- `policy/LW/robot_lab/leg_loco`
- `policy/LW/robot_lab/wheel_loco`
- `policy/LW/robot_lab/leg_to_wheel`
- `policy/LW/robot_lab/wheel_to_leg`

## 获取代码与依赖

克隆仓库时初始化保留的手柄子模块：

```bash
git clone --recursive https://github.com/bhwxzx/rl_sar.git
cd rl_sar
```

LW 描述已直接跟踪在 `src/rl_sar_zoo/LW_description` 中；构建过程不会下载
外部多机器人 zoo。`scripts/validate_lw_description.sh` 会校验其内容清单。

开发环境支持 Linux 上的 ROS 2 Foxy/Humble。主要依赖包括 C++17 编译器、
CMake、yaml-cpp、Eigen、Boost、TBB、OpenSSL、GLFW、ROS 2，以及由项目脚本
管理的推理与 MuJoCo 运行时。

## 开发构建

在仓库根目录加载 ROS 2 后构建：

```bash
source /opt/ros/humble/setup.bash
./build.sh
```

构建只校验本地已跟踪的 LW 描述，不会从网络克隆或更新机器人描述。

## Sim2Sim

构建并加载工作区后运行：

```bash
source install/setup.bash
ros2 run rl_sar rl_sim_LW
```

仿真器加载 `src/rl_sar_zoo/LW_description/mjcf/scene.xml`。terrain 场景与高度图
位于同一已跟踪包中，并由无界面的 MuJoCo 模型加载测试覆盖。

## 实机部署

构建、发布包校验、部署机验收、安全动作以及“先 Sim2Sim、后实机”的完整流程见
[docs/LW_BUILD_DEPLOYMENT_CN.md](docs/LW_BUILD_DEPLOYMENT_CN.md)。

ROS 2 启动入口为：

```bash
source install/setup.bash
ros2 launch rl_sar rl_real_LW.launch.py
```

未完成部署文档中的硬件侧检查时，不得启动实机节点。

## 仓库边界

- `policy/LW/`：当前策略配置和模型。
- `src/rl_sar/fsm_robot/fsm_LW.hpp`：当前机器人 FSM 与工厂。
- `src/rl_sar/src/rl_real_LW.cpp`：实机入口。
- `src/rl_sar/src/rl_sim_LW.cpp`：LW Sim2Sim 入口。
- `src/rl_sar/library/thirdparty/robot_sdk/lfr/LW_sdk/`：LW 硬件 SDK。
- `src/rl_sar_zoo/LW_description/`：已内置的 LW URDF、MJCF、mesh 和 terrain。
- `src/rl_sar/library/core/`：为未来受控扩展保留的通用控制、推理、FSM、循环和
  安全基础设施。

详细的删除/保留清单见
[docs/LW_REPOSITORY_SCOPE.md](docs/LW_REPOSITORY_SCOPE.md)。

## 添加未来机器人

添加机器人属于明确的产品范围变更，一次完整扩展应当：

1. 新增一个 `policy/<robot>/` 树，并提供经过校验的配置和版本化模型；
2. 新增可复现的 `<robot>_description` 包，记录来源且不得包含嵌套 Git 元数据；
3. 新增 `fsm_<robot>.hpp`，使用 `REGISTER_FSM_FACTORY` 注册，并从
   `fsm_all.hpp` 引入；
4. 新增机器人专用实机或仿真适配层，不把硬件行为塞进通用 FSM/SDK 核心；
5. 只增加该机器人实际需要的 SDK 依赖和 CMake 目标；
6. 扩展配置、状态转换、安全、仓库范围、干净构建及部署测试后才能声明支持。

## 许可证与来源

父仓库采用 Apache-2.0，见 [LICENSE](LICENSE)。LW 描述的来源仓库、导入提交、
包内许可证声明和保留的本地修改记录在
[src/rl_sar_zoo/README.md](src/rl_sar_zoo/README.md)。

项目保留对通用框架上游
[`fan-ziqi/rl_sar`](https://github.com/fan-ziqi/rl_sar)、LW zoo 分支
[`bhwxzx/rl_sar_zoo`](https://github.com/bhwxzx/rl_sar_zoo) 以及手柄子模块
[`drewnoakes/joystick`](https://github.com/drewnoakes/joystick) 的来源说明。
