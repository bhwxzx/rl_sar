# rl_sar — LW

`rl_sar` 当前仅支持 LW 机器人，用于强化学习策略的 Sim2Sim 验证与实机部署。
仓库保留通用 FSM、推理、控制循环和仿真接口，以便以后在明确扩大产品范围时添加
新机器人，而无需恢复旧机器人实现。

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

首次执行 `./build.sh` 时会检查 Debian/Ubuntu 系统包、当前 ROS 发行版组件、
推理运行时和仿真运行时。缺少系统包时会通过 `sudo apt-get` 自动安装，缺少
推理运行时或 MuJoCo 时会由项目脚本下载；因此首次构建需要网络并可能提示输入
sudo 密码。普通 x86_64 开发机构建会准备 LibTorch 和 ONNX Runtime；Jetson
真机构建只准备实际使用的 ONNX Runtime。可用下面的命令只查看系统包清单：

推理下载器只接受仓库清单固定的 Linux 平台、版本、官方 URL 和 SHA-256，归档
摘要、解压候选结构及 ELF 架构全部通过后才安装。已有的有效运行时若版本或来源
不匹配会被保留并使构建停止，不会静默升级或降级；系统 Python 环境中的
`torch`/`onnxruntime` 不属于该目录，不受影响。

这里的 LibTorch 是 C++ 版 PyTorch，供 Sim2Sim 可选的
`--use_actuator_net` 加载 `.pt` 执行器模型。训练脚本使用的 Python `torch`
不是编译依赖，不由 `build.sh` 通过 pip 修改用户 Python 环境。启用执行器网络时，
两个模型只会从 `--policy-root` 选定目录下的
`LW/robot_lab/motors/{leg,foot}_actuator_net.pt` 加载；缺失或契约不兼容会使
Sim2Sim 启动失败，不会回退到编译期策略目录。

```bash
ROS_DISTRO=humble scripts/install_build_dependencies.sh --print-packages
```

## 开发构建

在仓库根目录加载 ROS 2 后构建：

```bash
source /opt/ros/humble/setup.bash
./build.sh
```

构建只校验本地已跟踪的 LW 描述，不会从网络克隆或更新机器人描述。
指定包时会自动包含其工作区依赖，例如 `./build.sh fdilink_ahrs` 会先构建
`serial`，`./build.sh rl_sar` 会包含其声明的 IMU/串口依赖。

提交维护代码前应运行严格警告门禁：

```bash
scripts/validate_lw_strict_build.sh
```

脚本会创建临时 Debug 构建目录，以
`-Wall -Wextra -Wpedantic -Werror` 编译全部维护目标并运行完整 CTest，结束后
清理临时目录。vendored MuJoCo、joystick 和硬件 SDK 源码/头文件使用独立目标与
系统头边界，不会用第三方警告掩盖维护代码错误。

### Jetson 构建

默认不需要手工声明平台。构建入口、推理运行时脚本和 CMake 会在
Linux/aarch64 上检查 `/etc/nv_tegra_release`、`nvidia-l4t-core`、Tegra
系统库和 Jetson CUDA target，并输出统一的 `Jetson mode` 结果。建议真机使用
JetPack 6.2.2、Ubuntu 22.04 和 ROS 2 Humble。

LW 的 Jetson 真机和正式部署路径是 ONNX-only：首次构建自动下载 Linux
aarch64 ONNX Runtime，不安装或链接 PyTorch/LibTorch，也不启用 CUDA/TensorRT
推理。下载脚本和 CMake 会读取核心动态库的 ELF 架构；AArch64 设备会拒绝
x86-64 库，x86-64 主机也会拒绝 AArch64 库。因此不能把开发机的
`library/inference_runtime` 复制到 Jetson，应由 Jetson 自己运行 `./build.sh`
准备运行时。

只有自动检测所需的系统标志不可见时才应显式覆盖：

```bash
export IS_JETSON=true   # 强制按 Jetson 构建，只允许原生 Linux/aarch64
./build.sh
```

需要在 aarch64 非 Jetson 主机上明确禁用时可设置
`IS_JETSON=false`。变量只能使用 `true` 或 `false`；普通构建应保持未设置并
使用自动检测。

## Sim2Sim

构建并加载工作区后运行：

```bash
source install/setup.bash
ros2 run rl_sar rl_sim_LW
```

Sim2Sim 高频绘图遥测默认关闭。使用 `--enable-plot` 可按默认 100 Hz 恢复
`/LW_joint_states`，也可以指定 1–200 Hz 的整数频率：

```bash
ros2 run rl_sar rl_sim_LW --enable-plot --plot-rate-hz 50
```

`--plot-rate-hz` 必须与 `--enable-plot` 同时使用；无效或冲突的频率会让启动明确
失败。常规观察建议 50 Hz，普通诊断使用默认 100 Hz，200 Hz 仅用于短时逐控制
周期分析。

`--enable-plot` 只负责发布 `/LW_joint_states`，不会把数据写入磁盘。需要供
PlotJuggler 离线查看时，在另一个终端中录制：

```bash
mkdir -p bags
bag_dir="bags/lw_sim_$(date +%Y%m%d-%H%M%S)"
ros2 bag record \
    --output "$bag_dir" \
    /LW_joint_states
```

录制完成后先在 `ros2 bag record` 终端按 `Ctrl+C`，确保 bag 正常写入
`metadata.yaml`，然后检查并启动 PlotJuggler：

```bash
ros2 bag info "$bag_dir"
ros2 run plotjuggler plotjuggler
```

在 PlotJuggler 中使用 ROS 2 Bag 数据加载器打开 `$bag_dir` 对应的 bag。

仿真器加载 `src/rl_sar_zoo/LW_description/mjcf/scene.xml`。terrain 场景与高度图
位于同一已跟踪包中，并由无界面的 MuJoCo 模型加载测试覆盖。

## 实机部署

标准成功路径见[《LW 快速部署与实机启动指南》](docs/LW_QUICK_START_CN.md)；
构建原理、发布包校验、部署机验收、安全边界、故障排查以及“先 Sim2Sim、后
实机”的权威说明见
[《LW 编译与部署使用说明》](docs/LW_BUILD_DEPLOYMENT_CN.md)。

ROS 2 启动入口为：

```bash
source install/setup.bash
PYTHONDONTWRITEBYTECODE=1 ros2 launch rl_sar rl_real_LW.launch.py
```

必须保留 `PYTHONDONTWRITEBYTECODE=1`：正式 launch 目录是清单绑定的精确文件
集合，不能在启动时生成未经验证的字节码缓存。

该入口默认从控制终端 `/dev/tty` 启用真机键盘；数字键 `9` 可按 FSM 当前状态
请求 `GetDown`，且手柄断联锁存不会清除该键盘通道。无交互终端的受控部署必须
显式使用 `enable_keyboard:=false`，并按部署文档准备替代恢复和机械支撑措施。
当前键盘、手柄、速度摇杆、状态前提和 Sim2Sim 专用按键的完整说明见
[当前键盘和手柄映射](docs/LW_BUILD_DEPLOYMENT_CN.md#当前键盘和手柄映射)。

真机 `/LW_joint_states` 调试遥测默认关闭。受控调试时可按默认 50 Hz 启用，或用
`debug_publish_rate_hz:=<1–200 的整数>` 指定频率：

```bash
PYTHONDONTWRITEBYTECODE=1 ros2 launch rl_sar rl_real_LW.launch.py \
    enable_debug_publisher:=true debug_publish_rate_hz:=50
```

控制循环不会等待该可选消费者；发生争用或新帧覆盖时允许丢弃调试样本，且不会
给未变化的控制源帧重复刷新时间戳。

需要保存真机调试数据时，在同一控制机的第二个终端中录制。输出目录必须位于
受完整性保护的 `DEPLOY_PREFIX` 之外：

```bash
LW_BAG_ROOT="/absolute/writable/path/outside/DEPLOY_PREFIX"
mkdir -p "$LW_BAG_ROOT"
bag_dir="$LW_BAG_ROOT/lw_real_$(date +%Y%m%d-%H%M%S)"
ros2 bag record \
    --output "$bag_dir" \
    /LW_joint_states
```

正常采集结束时先在录包终端按 `Ctrl+C`，再检查并启动 PlotJuggler：

```bash
ros2 bag info "$bag_dir"
ros2 run plotjuggler plotjuggler
```

录制会在控制机上产生磁盘 I/O，应先确认磁盘空间并限制采集时长。rosbag2 故障
不会停止机器人，异常情况下必须优先急停和失能，不能为保存 bag 延迟安全操作。
调试结束后恢复默认关闭 publisher。

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

当前仓库边界由 `lw_repository_scope` 自动测试持续校验。

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
