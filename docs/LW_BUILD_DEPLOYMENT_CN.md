# LW 编译与部署使用说明

本文面向下面这种实际使用方式：

- **开发机**用于修改代码、模型和配置，并完成 Sim2Sim 仿真验证；
- **部署机**连接真实机器人，只进行正式部署验收和实机实验，同时也保留一份完整的 `rl_sar` 项目；
- 部署机在自己的 `rl_sar` 项目中，从开发机确认的 Git 提交生成一个固定的实机运行版本。

> [!CAUTION]
> 正常启动会连接传感器、串口和执行器。首次运行或更换代码、模型、配置、硬件环境后，必须先完成本文的离线验收。实机试运行时应将机器人可靠吊装或离地，保证急停可用，并安排人员现场监护。

## 1. 先理解三个目录

以下命令假设开发机和部署机的项目目录都是：

```text
/home/lfr/rl_sar
```

如果实际路径不同，只需修改命令中的 `RL_SAR_ROOT`。

部署机上会同时存在三类内容：

```text
/home/lfr/rl_sar/
├── src/、policy/、library/       # 完整项目：用于保存源码和外部推理库
└── build/lw_deployments/
    ├── <旧版本提交短哈希>/       # 以前验收通过的实机版本
    └── <当前提交短哈希>/         # 当前准备运行的实机版本
```

完整项目和部署版本并不冲突：

- 完整项目回答“有哪些源码可以修改和编译”；
- 部署版本回答“实机现在运行的究竟是哪一次提交、哪四个模型和哪些配置”。

部署版本不会取代完整项目。它只是把一次正式运行使用的可执行文件、模型和配置冻结下来，防止旧二进制、新源码、临时模型和不同版本配置混在一起。

## 2. 推荐流程总览

```text
开发机修改代码、配置和模型
        ↓
开发机编译并完成 LW Sim2Sim 验证
        ↓
提交验证通过的代码、配置和四个 ONNX 到 Git
        ↓
记录完整 commit 哈希并同步给部署机
        ↓
部署机取得该 commit，但不必切换当前工作区
        ↓
部署机在自己的 rl_sar 中执行干净构建
        ↓
部署包离线验收 + 动态库检查
        ↓
完成机械和人员安全检查
        ↓
部署机进行实机实验
```

推荐在部署机本地构建，而不是直接复制开发机编译出的二进制。这样可以使用部署机自己的 CPU 架构、ROS、编译环境和 `library/inference_runtime`，减少两台机器环境不一致造成的问题。

这里有两个不同的验收关口：

- 开发机的 **Sim2Sim 验证**检查策略和控制逻辑在仿真中的行为是否正确；
- 部署机的 **部署包离线验收**检查二进制、提交、模型和配置是否完整一致。

两者缺一不可。`--verify-deployment-only` 不运行仿真，也不能证明机器人行为正确。

## 3. 开发机上的步骤

### 第一步：准备 Sim2Sim 候选版本

进入开发机上的项目：

```bash
RL_SAR_ROOT=/home/lfr/rl_sar
cd "$RL_SAR_ROOT"
git status --short
```

完成代码、配置和模型修改，并按项目要求运行相应单元测试。LW Sim2Sim 和正式实机部署都使用以下四个 ONNX 模型：

```text
policy/LW/robot_lab/leg_loco/policy.onnx
policy/LW/robot_lab/leg_to_wheel/policy.onnx
policy/LW/robot_lab/wheel_loco/policy.onnx
policy/LW/robot_lab/wheel_to_leg/policy.onnx
```

它们分别负责腿式运动、腿转轮、轮式运动和轮转腿。Sim2Sim 还需要开发机上存在 LW 的 MuJoCo 场景，例如：

```text
src/rl_sar_zoo/LW_description/mjcf/scene.xml
```

### 第二步：在开发机编译 Sim2Sim

加载 ROS 2 环境并执行正常的开发构建：

```bash
source /opt/ros/humble/setup.bash
cd "$RL_SAR_ROOT"
./build.sh
```

第一次构建建议编译整个工作区。也可以使用 `./build.sh rl_sar` 做增量构建；
该命令会按 package manifest 自动包含 `rl_sar` 的工作区依赖，而不是只选择
一个可能缺少依赖的包。

LW 实机启动还需要工作区中的 IMU 驱动及其 ROS 串口库。首次构建后应确认三
个包都来自当前开发安装目录：

```bash
source "$RL_SAR_ROOT/install/setup.bash"
ros2 pkg prefix serial
ros2 pkg prefix fdilink_ahrs
ros2 pkg prefix rl_sar
```

仓库只保留 `./build.sh` 作为开发构建入口。不带包名时构建整个工作区；指定
`fdilink_ahrs` 或 `rl_sar` 时，colcon 会先构建其声明的 `serial` 和 IMU
依赖。首次运行会自动检查并安装缺失的 Debian/Ubuntu、ROS、推理和仿真依赖，
可能请求 sudo 权限和网络访问；具体清单见
[README_CN.md](../README_CN.md#获取代码与依赖)。

在 Jetson 上，`./build.sh` 会自动检查 Linux/aarch64、L4T/Tegra 和 Jetson
CUDA 标志，并在日志中输出 `Jetson mode: true`。只有容器等环境隐藏了这些
标志时才使用 `export IS_JETSON=true`；该强制模式在非 Linux/aarch64 主机上
会拒绝运行。普通 x86_64 和可明确排除 Jetson 的 aarch64 环境无需设置变量，
必要时可用 `IS_JETSON=false` 显式禁用。

建议 Jetson Orin NX 使用 JetPack 6.2.2、Ubuntu 22.04 和 ROS 2 Humble。Jetson
真机路径只下载并链接 Linux aarch64 ONNX Runtime，不安装 PyTorch/LibTorch，
也不启用 CUDA/TensorRT 推理。已有推理库会先校验 ELF 架构，不能把 x86_64
开发机的 `library/inference_runtime` 复制到 Jetson 使用。

这个开发构建与后面的正式部署构建用途不同：

- `./build.sh` 生成开发机上的 Sim2Sim 程序；
- `build_lw_deployment.sh` 在部署机生成实机正式运行版本。

### 第三步：在开发机完成 Sim2Sim 验证

构建成功后，在开发机启动 LW 的 MuJoCo Sim2Sim：

```bash
source /opt/ros/humble/setup.bash
source "$RL_SAR_ROOT/install/setup.bash"
ros2 run rl_sar rl_sim_LW
```

至少验证以下内容：

- 仿真能够正常启动，模型和 YAML 没有加载错误；
- 初始姿态和关节方向正确，机器人静止时没有明显异常动作；
- `leg_loco` 和 `wheel_loco` 都能正常进入并响应控制命令；
- `leg_to_wheel` 和 `wheel_to_leg` 两个转换能够完整执行；
- 状态切换后关节位置、速度和力矩没有明显跳变或持续发散；
- 连续运行期间没有崩溃、NaN、越界或控制周期异常；
- 本次准备交付的四个 ONNX 和配置与仿真实际加载的文件一致。

如果 Sim2Sim 暴露问题，应回到第一步修改并重新验证。只有 Sim2Sim 通过后，候选版本才能交给部署机进行实机实验。

> [!NOTE]
> Sim2Sim 只能降低实机风险，不能替代实机安全措施。仿真通过后，部署机仍必须完成部署包验收、动态库检查和机械安全检查。

### 第四步：提交 Sim2Sim 验证通过的内容

构建脚本只读取指定 Git 提交中的文件。没有提交的代码、配置和模型不会进入正式部署版本。

提交完成后检查：

```bash
git status --short
git show --stat --oneline HEAD
```

如果 `git status --short` 仍显示文件，应确认它们是否属于本次发布。不要为了让状态变干净而误删与本次发布无关的用户文件。

提交之后不要再修改本次发布使用的代码、模型或配置；如果又有修改，必须重新完成 Sim2Sim 并生成新的提交。

### 第五步：记录并同步已验证提交

记录准备部署的完整提交哈希：

```bash
SOURCE_COMMIT=$(git rev-parse HEAD)
echo "$SOURCE_COMMIT"
```

将该提交推送到部署机能够访问的 Git 仓库，或通过团队认可的方式把该提交同步到部署机。交付信息中必须保留上面输出的完整哈希，不能只说“使用最新版本”。

开发机交付时应同时记录 Sim2Sim 结果和完整提交哈希。开发机到这里完成正式交付准备；后续部署包构建和实机实验在部署机完成。

## 4. 部署机上的准备步骤

### 第一步：进入部署机自己的项目

```bash
RL_SAR_ROOT=/home/lfr/rl_sar
cd "$RL_SAR_ROOT"
git status --short
```

部署机可以保留自己的未跟踪文件或其他开发内容。后面的构建脚本会为指定提交创建临时干净工作树，不会直接从当前工作目录编译。不过，开始前仍应查看状态，避免误操作用户文件。

### 第二步：取得开发机指定的提交

先从 Git 仓库取得最新对象：

```bash
git fetch --all --tags
```

然后填写开发机提供的完整提交哈希并确认该提交已经存在。下面的哈希只是格式示例，执行时必须替换：

```bash
SOURCE_COMMIT=0123456789abcdef0123456789abcdef01234567
git cat-file -e "${SOURCE_COMMIT}^{commit}"
git show --stat --oneline "$SOURCE_COMMIT"
```

`git cat-file` 没有输出且返回成功，表示部署机已经取得该提交。构建脚本可以直接构建这个提交，因此不需要执行 `git checkout`，也不需要切换部署机当前分支。

### 第三步：检查部署机本地依赖

加载部署机安装的 ROS 2，并确认推理库存在：

```bash
source /opt/ros/humble/setup.bash

test -d "$RL_SAR_ROOT/library/inference_runtime/onnxruntime"
bash "$RL_SAR_ROOT/scripts/validate_inference_runtime.sh" \
    onnx "$RL_SAR_ROOT/library/inference_runtime/onnxruntime" "$(uname -m)"
```

还需要保证 `git`、`cmake`、`colcon`、C++ 编译器及
[README_CN.md](../README_CN.md#获取代码与依赖) 中的依赖可用。

> [!NOTE]
> 正式部署清单会校验 LW 可执行文件、模型、配置，以及项目自带的
> `fdilink_ahrs` 和 `serial` 运行文件，但不会打包基础 ROS、ONNX Runtime、
> Python、操作系统动态库和 USB 内核驱动。后者仍由部署机
> 的系统环境提供。

### 第四步：准备串口设备名和访问权限

LW 使用两条彼此独立的串口路径：

| 用途 | 固定设备名 | 访问程序 | 是否使用 ROS `serial` 包 |
| --- | --- | --- | --- |
| 右侧电机板 | `/dev/ttyLegRight` | `rl_real_LW` 内的 LWSDK | 否 |
| 左侧电机板 | `/dev/ttyLegLeft` | `rl_real_LW` 内的 LWSDK | 否 |
| IMU | `/dev/fdilink_ahrs` | `fdilink_ahrs/ahrs_driver_node` | 是 |

当前项目按现场使用要求将匹配到的 IMU 串口设置为 `0777`。规则安装完成后，
普通登录用户访问 `/dev/fdilink_ahrs` 不需要 root 权限，也不要求属于
`dialout` 组。该设置同时允许本机其他用户读写设备，部署机应限制非授权账户
登录和运行程序。

先用实际枚举出的设备节点确认 IMU 的 USB 属性；下面的 `/dev/ttyUSB0` 只是
示例，必须替换为本机设备：

```bash
udevadm info --query=property --name=/dev/ttyUSB0 \
    | grep -E 'ID_VENDOR_ID|ID_MODEL_ID|ID_SERIAL_SHORT'
```

确认设备确实匹配仓库脚本记录的 CP2102、CH9102 或 CH340 型号后，才可安装
IMU 规则：

```bash
sudo "$RL_SAR_ROOT/src/fdilink_ahrs_ROS2/wheeltec_udev.sh"
```

该脚本只建立 `/dev/fdilink_ahrs` 并将设备权限设置为 `0777`，不会在编译或
离线验收时自动运行。写入 `/etc/udev/rules.d` 和刷新 udev 仍属于系统管理
操作，因此安装规则时必须使用 `sudo`；安装后的普通串口访问不需要 root 或
`dialout`。CH340 通常没有可用于区分同型号设备的唯一序列号；
如果主机连接了多个 CH340，不应直接使用该通用规则，而应先制定能唯一识别
目标 IMU 的现场规则。

仓库目前没有足够的 USB VID、PID 和序列号信息来安全生成
`/dev/ttyLegRight`、`/dev/ttyLegLeft`。部署人员必须先分别读取两块电机板的
实际属性，再用不同的唯一序列号建立稳定别名；不要依赖可能随插拔顺序变化的
`/dev/ttyUSB0`、`/dev/ttyUSB1`，也不要把占位 VID/PID 直接写入系统规则。

重新插拔设备后，只做节点、权限和链接目标检查，不打开串口：

```bash
for device in /dev/ttyLegRight /dev/ttyLegLeft /dev/fdilink_ahrs; do
    test -e "$device" && test -r "$device" && test -w "$device"
    readlink -f "$device"
done
```

## 5. 在部署机生成正式运行版本

继续在部署机的 `/home/lfr/rl_sar` 中执行：

```bash
SHORT_COMMIT=$(git rev-parse --short "$SOURCE_COMMIT")
DEPLOY_PREFIX="$RL_SAR_ROOT/build/lw_deployments/$SHORT_COMMIT"

src/rl_sar/scripts/build_lw_deployment.sh \
    "$DEPLOY_PREFIX" \
    "$SOURCE_COMMIT"
```

脚本会自动完成以下工作：

1. 从 `SOURCE_COMMIT` 创建临时、干净的源码工作树；
2. 初始化该提交锁定的 Git 子模块；
3. 使用 `Release` 和 `LW_PRODUCTION_DEPLOYMENT=ON` 编译；
4. 安装 `serial`、`fdilink_ahrs`、`rl_real_LW`、LW 配置测量工具、五个
   YAML、四个 ONNX 和两个状态转换 CSV；
5. 生成记录源码提交、策略资源及项目内 IMU/串口运行文件 SHA-256 的
   `manifest.yaml`；
6. 拒绝关键部署文件中的符号链接；
7. 自动执行一次 `--verify-deployment-only` 离线验收。

输出目录必须不存在或为空。脚本不会覆盖以前的部署版本。如果同一个提交需要重新构建，应使用新的带后缀目录，例如 `${SHORT_COMMIT}_02`，并保留必要的旧版本供回滚。

主要产物位于：

```text
<DEPLOY_PREFIX>/
├── setup.bash
├── lib/libserial.a
├── lib/fdilink_ahrs/ahrs_driver_node
├── lib/rl_sar/lw_config_profiler
├── lib/rl_sar/profile_lw_runtime_config.py
├── lib/rl_sar/rl_real_LW
├── share/fdilink_ahrs/
│   ├── launch/ahrs_driver.launch.py
│   └── wheeltec_udev.sh
└── share/rl_sar/deployment/LW/
    ├── manifest.yaml
    └── policy/LW/
        ├── base.yaml
        └── robot_lab/
            ├── leg_loco/
            ├── leg_to_wheel/
            ├── wheel_loco/
            └── wheel_to_leg/
```

## 6. 在部署机进行部署包离线验收

构建脚本已经自动验收一次。正式启动前，建议在一个没有加载旧部署版本的新终端中再验收一次。

重新设置变量，因为新终端不会保留上一个终端中的变量：

```bash
RL_SAR_ROOT=/home/lfr/rl_sar
cd "$RL_SAR_ROOT"

# 必须与本次部署的提交一致
SOURCE_COMMIT=0123456789abcdef0123456789abcdef01234567
SHORT_COMMIT=$(git rev-parse --short "$SOURCE_COMMIT")
DEPLOY_PREFIX="$RL_SAR_ROOT/build/lw_deployments/$SHORT_COMMIT"

source /opt/ros/humble/setup.bash
source "$DEPLOY_PREFIX/setup.bash"
"$DEPLOY_PREFIX/lib/rl_sar/rl_real_LW" --verify-deployment-only
```

该命令只检查发布物，不会运行 Sim2Sim，不会创建 ROS 节点，也不会初始化手柄、串口或控制线程。它会检查：

- 当前可执行文件是否属于清单记录的源码提交；
- 可执行文件、五个 YAML、四个 ONNX 和两个 CSV 的哈希是否正确；
- `libserial`、AHRS 节点、launch、udev 辅助脚本和两个 ROS 包索引的哈希
  是否正确；
- 资源路径是否仍在部署目录内；
- 关键部署文件是否经过符号链接。

只有命令返回码为 `0` 才表示部署包完整性验收通过。它不能证明策略行为正确，因此必须确认该提交已经在开发机完成 Sim2Sim。任何报错都应停止部署，不能通过直接修改部署目录来绕过检查。

查看清单和动态库：

```bash
sed -n '1,220p' \
    "$DEPLOY_PREFIX/share/rl_sar/deployment/LW/manifest.yaml"

ldd "$DEPLOY_PREFIX/lib/rl_sar/rl_real_LW"
ldd "$DEPLOY_PREFIX/lib/rl_sar/lw_config_profiler"
ldd "$DEPLOY_PREFIX/lib/fdilink_ahrs/ahrs_driver_node"
```

`manifest.yaml` 中的 `source_commit` 应等于开发机交付的完整提交哈希。两次
`ldd` 输出中如果出现 `not found`，说明部署机缺少运行库，不得启动实机程序。
`rl_real_LW` 和 `lw_config_profiler` 应包含 `libonnxruntime.so`，且不得出现
`libtorch`、`libtorch_cpu` 或 `libc10`；正式部署构建脚本会自动检查这一约束。

还必须确认包解析没有回退到旧开发工作区：

```bash
for package in serial fdilink_ahrs rl_sar; do
    test "$(realpath -m "$(ros2 pkg prefix "$package")")" = "$DEPLOY_PREFIX"
done
```

## 7. 在部署机进行实机实验

启动前至少确认：

- 当前部署目录的 `--verify-deployment-only` 已通过；
- `manifest.yaml` 中的提交与本次交付记录一致；
- 该提交在开发机上的 Sim2Sim 验证记录已确认；
- `ldd` 没有缺失依赖；
- 机器人型号、关节映射、限位、初始姿态和四个策略版本正确；
- IMU、串口、执行器及手柄连接已分别验证；
- 机器人已可靠吊装或离地，运动范围内无人和障碍物；
- 硬件急停、电机失能方式及现场监护人员均已就位。

在刚才完成离线验收、并且 `DEPLOY_PREFIX` 指向正确版本的终端中执行：

```bash
ros2 launch rl_sar rl_real_LW.launch.py
```

正常启动会加载 AHRS 驱动并运行 `rl_real_LW`，随后可能访问真实硬件。不要跳过离线验收，也不要用正常启动命令测试部署包是否完整。

### 真机终端键盘

真机 launch 默认设置 `enable_keyboard:=true`。`rl_real_LW` 会直接打开启动该
launch 的控制终端 `/dev/tty`，切换为非规范、无回显的非阻塞输入，但保留
`Ctrl-C` 信号。终端配置会在正常退出、启动异常和对象销毁时恢复。键盘读取由
200 Hz 控制线程执行，不会启动另一个线程并发修改 FSM 输入。

数字键 `9` 是现有 FSM 的 `GetDown` 请求。手柄断联后，程序仍会永久锁住该
进程的 Gamepad 输入并把 `x/y/yaw` 置零，但不会清除终端键盘；确认下降路径
安全后仍可在这个终端按 `9` 请求受控趴下。其他数字键和 `P` 等键位仍按当前
FSM 状态解释，不能在机器人未吊装、人员位于运动范围内时试键。

启用键盘却没有可访问的控制终端时，真机节点会在控制循环启动前明确失败，
避免错误宣称存在键盘恢复通道。systemd、容器或其他明确无交互终端的部署必须
显式关闭：

```bash
ros2 launch rl_sar rl_real_LW.launch.py enable_keyboard:=false
```

关闭后不存在终端 `GetDown` 通道，必须在启动前准备独立的受控恢复方式、可靠
机械支撑和物理急停；不得仅因为节点仍在运行就假设手柄断联后可以安全恢复。

### 当前键盘和手柄映射

以下表格描述当前 LW 代码实际使用的映射，不是通用推荐键位。FSM 按键只在表中
列出的状态前提满足时生效；没有满足前提时，按键不会强制跳过起身、趴下或形态
切换过程。首次试键必须保持机器人可靠吊装或离地，并由现场人员掌握物理急停。

#### FSM 和模式切换

| 功能 | 键盘 | 手柄 | 生效状态和说明 |
| --- | --- | --- | --- |
| 腿式起身 | `0` | `A` | 在 Passive、轮式起身或 GetDown 状态请求 `RLFSMStateGetUp_Leg` |
| 进入腿式运动 | `1` | `RB` + 十字键上 | 仅在腿式起身完成后进入腿式 locomotion；腿式 locomotion 中重复请求保持当前状态 |
| 轮式起身 | `2` | `Y` | 在 Passive、腿式起身或 GetDown 状态请求 `RLFSMStateGetUp_Wheel` |
| 进入轮式运动 | `3` | `RB` + 十字键下 | 仅在轮式起身完成后进入轮式 locomotion；轮式 locomotion 中重复请求保持当前状态 |
| 腿式切换为轮式 | `4` | `RB` + 十字键左 | 仅在腿式 locomotion 中启动 `leg_to_wheel` 策略；完成后自动进入轮式 locomotion |
| 轮式切换为腿式 | `5` | `RB` + 十字键右 | 仅在轮式 locomotion 中启动 `wheel_to_leg` 策略；完成后自动进入腿式 locomotion |
| 受控趴下 | `9` | `B` | 起身完成后、任一 locomotion 或形态切换状态中请求 GetDown；Passive 中无动作 |
| 立即转入 Passive 阻尼 | `P` | `LB` + `X` | 任一非 Passive 状态转入 `RLFSMStatePassive`；这是 `Kp=0`、`Kd=5` 的阻尼命令，不等于电机失能或零执行器输出 |
| 切换导航模式标志 | `N` | `X` | 切换 `navigation_mode`；当前 LW 本地摇杆/键盘速度输入仍按下表处理 |

组合键写法表示先按住肩键，再按面键或推动十字键。`GetDown` 是受控运动，不是
急停；姿态、反馈或控制线程已经不可信时，运行时会按 S3/S4 硬失能，操作员也应
使用物理急停和可靠支撑。

#### 速度指令

| 功能 | 键盘 | 手柄 |
| --- | --- | --- |
| 增大/减小前向速度 `x` | `W` / `S`，每次按键变化 `+0.1` / `-0.1` | 左摇杆纵向：向前为正、向后为负 |
| 增大/减小横向速度 `y` | `A` / `D`，每次按键变化 `+0.1` / `-0.1` | 左摇杆横向：向左为正、向右为负 |
| 增大/减小偏航速度 `yaw` | `Q` / `E`，每次按键变化 `+0.1` / `-0.1` | 右摇杆横向：向左为正、向右为负 |
| 三轴速度清零 | `Space` | 将已使用的摇杆回中 |

键盘速度是离散累加值，不会因为松键自动回零；连续按键前应确认当前终端状态，
需要停止时按 `Space`。手柄速度按当前策略 `vel_command` 缩放；当前已提交的 LW
策略将横向上限 `vel_command[1]` 设为 `0.0`，因此默认配置下左摇杆横向不会
产生 `y` 指令。进入 S1 输入降级后，运行时会锁住 `x/y/yaw` 为零，但仍保留
上表的 FSM 恢复按键。

#### 仅 Sim2Sim 使用

| 功能 | 键盘 | 手柄 | 说明 |
| --- | --- | --- | --- |
| 重置到腿式初始姿态 | `R` | `RB` + `Y` | 调用 MuJoCo 的 `home_leg` keyframe；真机无此功能 |
| 重置到轮式初始姿态 | `T` | `RB` + `A` | 调用 MuJoCo 的 `home_wheel` keyframe；真机无此功能 |
| 暂停/继续仿真 | `Enter` | `RB` + `X` | 切换 MuJoCo 运行状态；真机无此功能 |

#### 手柄布局和未绑定输入

真机与默认 Sim2Sim 当前都按 `JOYSTICK_1` 布局解释 `/dev/input/js0`：按钮编号
`0/1/3/4` 分别作为 `A/B/X/Y`，`6/7` 作为 `LB/RB`，轴 `0/1/2` 分别作为
`LX/LY/RX`，轴 `6/7` 作为十字键横向/纵向。这里的 `A/B/X/Y` 是程序赋予
Linux joystick 编号的逻辑名称；不同品牌、连接方式或驱动可能报告不同编号，
必须在机器人断电或可靠吊装时先核对实际设备事件，不能只看手柄外壳标识。

除上表外，键盘解析器虽然能识别其他字母、数字、方向键和 `Escape`，手柄层也
能识别单独的 `LB/RB`、十字键、摇杆按下及其他组合，但当前 LW FSM 和适配器
没有为它们绑定动作。尤其是 `M/K`、`LB+A`、`LB+B` 当前不负责电机使能或
失能，单独十字键也不调整步频；不得依据通用 SDK 注释或其他机器人配置推测
这些按键的功能。

### 受控验证 IMU 话题

这一步不是自动化验收的一部分，因为它会真实打开 `/dev/fdilink_ahrs`。只能在
确认 IMU 型号、波特率 `921600`、设备别名和权限正确，并且尚未启动完整
`rl_real_LW` 时单独执行：

```bash
ros2 launch fdilink_ahrs ahrs_driver.launch.py
```

在另一个同样加载当前部署前缀的终端检查：

```bash
ros2 topic echo --once /imu
ros2 topic hz /imu
```

确认时间戳持续更新、姿态/角速度/加速度为有限值且发布频率稳定后，用
`Ctrl-C` 停止独立 AHRS 驱动。不要让独立 AHRS 驱动与完整 LW launch 同时
争用同一串口。

### 可选调试话题

正式启动默认不创建高频调试 publisher 或定时器，也不会在控制线程中复制调试
快照。如需临时观察当前/目标关节、IMU 和速度指令，可显式启用 250 Hz 的
`/LW_joint_states` 话题：

```bash
ros2 launch rl_sar rl_real_LW.launch.py enable_debug_publisher:=true
```

该话题仅用于受控调试和绘图；每条消息使用同一个控制周期的完整快照，并在
发布时刷新时间戳。调试结束后应恢复默认关闭状态，避免不必要的 ROS 发布负载。

### 实机控制循环的默认保护行为

LW 实机控制循环按绝对时间点以 5 ms 周期运行。某次执行过慢时，程序会跳过已经过期的周期，不会为了“补次数”而连续突发执行控制回调。终端状态和周期统计由 ROS 定时器输出，不在 200 Hz 电机命令线程中打印。

正式版本使用其清单内的 `policy/LW/base.yaml`。当前默认值的含义如下：

```yaml
control_loop_cpu: -1
control_loop_realtime_priority: 0
control_loop_require_realtime: false
control_loop_degraded_consecutive_misses: 3
control_loop_degraded_lateness: 0.02
control_loop_fatal_consecutive_misses: 0
control_loop_fatal_lateness: 0.0
```

- `cpu: -1` 表示暂不固定 CPU；`realtime_priority: 0` 表示使用普通的 `SCHED_OTHER` 调度。这是未经部署机测量前的可移植默认值。
- 连续错过 3 个控制周期，或者单次唤醒或执行晚到达到 20 ms，会进入“时序降级”状态。
- 时序降级会永久锁住本次进程的 `x/y/yaw` 为零，但保留 FSM 按钮输入。操作员仍可请求 `GetDown`；处理完现场安全后必须重启进程才能清除该锁存。
- 默认的两个 `fatal` 值为零，表示控制循环晚到只进入上述降级状态，不会自动硬失能。传感器过期、非法最终命令、姿态越界和控制回调异常等不可信故障仍保持硬失能。
- 每秒一条 `[Timing] loop_control` 日志会报告平均/最大唤醒晚到、最大截止时间晚到、最大回调执行时间、错过截止时间和跳过周期数。应把实机吊装测试时的这些日志保存到验收记录中。

不要直接修改已经生成的部署目录来调整参数。参数调整应在项目的 `policy/LW/base.yaml` 中完成，经过开发机测试和 Sim2Sim、提交，再由部署机从新提交生成新的部署版本。

### 吊装且不进入 Locomotion 的配置测量

可靠吊装并保持 FSM 在 Passive 时，可以完成两阶段的部署前测量：第一阶段在
目标 Jetson 上不连接任何硬件，运行与正式部署相同的四个 ONNX、观测/输出路径、
200 Hz 控制调度和 50 Hz 推理；第二阶段只观察真实 IMU 与两块电机板反馈，并
持续发送电机失能包。策略仍会在后台做 shadow inference，但其输出始终被丢弃，
不会进入腿式/轮式 Locomotion 或形态切换状态。

这两阶段可给出以下候选：

- `control_loop_cpu`、显式试验过的 `control_loop_realtime_priority`；
- 控制循环降级的连续丢周期数和单次晚到阈值；
- IMU/左右反馈的 `sensor_timeout`；
- 两个串口组成的完整命令包写入截止时间 `serial_write_timeout`。

吊装静态测量不能验证运动负载、接地冲击、电源压降、电机板看门狗和硬失能后的
机械结果。因此工具永远保留
`control_loop_fatal_consecutive_misses: 0`、
`control_loop_fatal_lateness: 0.0`，并保持
`control_loop_require_realtime: false`；这三项只能在单独的物理安全验证和部署权限
评审后人工决定。输出的其他值也只是吊装环境候选，不是自动生效的最终配置。

#### 第一阶段：无硬件 CPU 和调度测量

在目标部署机的新终端中加载本次部署前缀。以下命令不会初始化 ROS、IMU、手柄、
串口或执行器设备：

```bash
source /opt/ros/humble/setup.bash
source "$DEPLOY_PREFIX/setup.bash"

LW_PROFILER="$DEPLOY_PREFIX/lib/rl_sar/lw_config_profiler"
LW_PROFILE_TOOL="$DEPLOY_PREFIX/lib/rl_sar/profile_lw_runtime_config.py"
LW_POLICY_ROOT="$DEPLOY_PREFIX/share/rl_sar/deployment/LW/policy"
LW_PROFILE_DIR="$RL_SAR_ROOT/build/lw_profiles/$SHORT_COMMIT"

python3 "$LW_PROFILE_TOOL" collect-host \
    --profiler "$LW_PROFILER" \
    --policy-root "$LW_POLICY_ROOT" \
    --output-dir "$LW_PROFILE_DIR/host" \
    --duration-seconds 30 \
    --cpus allowed \
    --realtime-priorities 0
```

`--cpus allowed` 会逐一测量当前进程亲和掩码允许的逻辑 CPU；也可用
`--cpus=-1,0,1` 显式加入不绑定的对照组。正数实时优先级只应由部署负责人明确
选定后加入，例如 `--realtime-priorities 0,50`。未成功应用的亲和性或
`SCHED_FIFO` 报告不会被分析器选为候选。每个输出目录必须不存在或为空，工具
拒绝覆盖历史测量。

#### 第二阶段：吊装硬件观察

硬件阶段必须同时满足：机器人可靠吊装、运动范围隔离、现场人员掌握物理急停、
正常 `rl_real_LW` 已停止、两个电机板串口没有其他读写者。只单独启动 AHRS，
不要启动完整 LW launch：

```bash
ros2 launch fdilink_ahrs ahrs_driver.launch.py
```

在另一个已加载相同部署前缀的终端执行。确认字符串必须逐字匹配：

```bash
python3 "$LW_PROFILE_TOOL" collect-hardware \
    --profiler "$LW_PROFILER" \
    --policy-root "$LW_POLICY_ROOT" \
    --output "$LW_PROFILE_DIR/hardware.json" \
    --duration-seconds 60 \
    --cpu -1 \
    --realtime-priority 0 \
    --confirmation I_CONFIRM_LW_IS_SUSPENDED_AND_MOTORS_MUST_REMAIN_DISABLED
```

该模式不会创建手柄或键盘输入，也不会允许 Passive/shadow policy 命令进入串口。
它会先打开左右电机板串口并确认 20 个 `motors_disable=true` 包均完整写入，随后
启动独立的 5 ms 失能保活线程；只有失能写入和保活启动成功后才会预加载四个模型、
启动 ROS 观察和参数测算。初始化、测算和退出阶段不存在非失能命令发送路径；
任一失能包写入不完整都会终止测算，退出前仍尝试最终 20 个失能包。报告记录
初始失能写入与保活证明、IMU、左右电机板首帧等待、相邻有效帧间隔、结束时帧龄、
串口写耗时与失败次数。任何失能证明缺失、来源未出现、采样不足、串口写失败或
四个策略未完整运行，都会使候选分析失败。命令结束后停止独立 AHRS 驱动。

当前电机板反馈协议没有单独的“失能已执行”回执位，因此上述证明只表示上层失能
包已完整写入两个串口且程序没有发送非失能命令，不能替代 STM32 侧状态确认或
物理急停。底层是否实际进入失能状态仍应由固件行为和现场安全措施保证。

#### 生成仅供评审的候选

下面两个 `max-safe` 值不是脚本测出来的性能值，而是风险评估预先确定的硬上限；
必须由负责机械与控制安全的人员给出。示例中的占位符不能原样执行：

```bash
python3 "$LW_PROFILE_TOOL" analyze \
    --base-yaml "$LW_POLICY_ROOT/LW/base.yaml" \
    --reports "$LW_PROFILE_DIR"/host/*.json "$LW_PROFILE_DIR/hardware.json" \
    --output "$LW_PROFILE_DIR/candidate-review.json" \
    --max-safe-sensor-timeout-ms <评审确定的最大传感器时效毫秒> \
    --max-safe-control-gap-ms <评审确定的最大控制间断毫秒> \
    --minimum-hardware-samples 1000
```

分析器按截止时间丢失、最大晚到、最大执行时间和推理尾延迟排序 host 报告；传感器
候选同时覆盖首帧等待、P50/P99.9/最大帧间隔和结束帧龄；串口候选覆盖
P99.9/最大写耗时，并且必须小于一个 5 ms 控制周期。若不提供某个安全上限，
对应的现有值会保持不变并标记为需要人工测量或评审。

`candidate-review.json` 是 JSON（也是 YAML 1.2 可读的映射），包含
`review_only: true`，不会修改输入 `base.yaml` 或部署目录。必须人工查看报告、在
源码树的 `policy/LW/base.yaml` 中单独修改、重新执行 Sim2Sim 和测试、提交，再
从新提交生成新部署版本。不得把候选文件直接覆盖到当前部署包。

### 可选的 CPU 固定和实时优先级

只有在目标部署机上完成负载测量后，才应设置 `control_loop_cpu` 或正数的 `control_loop_realtime_priority`。先用 `lscpu` 确认可用 CPU，再确认运行账户具备设置 `SCHED_FIFO` 的权限；具体授权方式应遵守部署机的 systemd 或安全配置，不要仅为绕过权限错误而以 root 身份运行整个控制程序。

当设置了正数优先级但 `control_loop_require_realtime: false` 时，权限不足会记录警告并回退到 `SCHED_OTHER`；设为 `true` 后，实时调度失败会使控制回调在首次执行前拒绝启动。只要显式配置了 CPU 编号，CPU 固定失败就会拒绝启动，不受该开关影响。这些配置必须先在吊装状态下验收。

`control_loop_fatal_consecutive_misses` 和 `control_loop_fatal_lateness` 只应在取得部署机时序数据、确认电机板端看门狗行为并决定硬失能策略后显式启用；后一个值的单位是秒。启用任一严重阈值后，达到阈值会进入下文的 S4：锁死命令门、发送约 100 ms 的电机失能包并请求 ROS 关闭，机器人不会自动执行受控趴下。

### 实机运行时的安全分级

程序不再把所有异常都当成“立即断力并退出”。处理强度由故障发生在哪个环节、此时控制数据是否还可信来决定：

| 级别 | 触发情况 | 程序做什么 | 现场处理 |
| --- | --- | --- | --- |
| S0 诊断 | 串口解析错误，但新鲜有效反馈仍持续到达；力矩保护告警 | 记录告警，不改变控制 | 观察错误计数；若有效数据随后过期，会按 S4 处理 |
| S1 输入降级 | 手柄断开/手柄线程异常，或控制循环连续错过 3 周期/单次晚到 20 ms | 锁住速度指令为零，不断电、不关闭 ROS；时序降级时保留 FSM 按钮 | 可在判断安全后请求 `GetDown`；重启才能恢复该输入源 |
| S2 受控阻尼 | 推理线程异常，策略 action/输出非法，策略输出不完整或过期 | 不再接受策略输出；在下一个可执行的控制周期把命令覆盖为 `Kp=0`、`Kd=5`、速度与前馈力矩为零；若是控制线程当周期发现输出过期，则在当周期直接覆盖旧命令。随后由控制线程转入 `RLFSMStatePassive` | 扶稳机器人并停止实验；该故障不自动恢复，排查后重启 |
| S3 硬失能 | 电机板报告硬件故障 | 关闭命令门并发送 20 次失能包；不主动关闭 ROS，便于保留诊断 | 立即支撑机器人、使用物理急停/断电，不得原地恢复；排除硬件故障后重启 |
| S4 硬失能并退出 | 控制线程异常、传感器过期/反馈读取失败、反馈非法、FSM 丢失、受保护状态下姿态超过 75°、最终命令非法/发送不完整、显式启用的致命时序阈值 | 关闭命令门，发送 20 次失能包，然后请求 ROS 关闭 | 立即支撑机器人并使用物理急停/断电；不要期待自动趴下 |

启动阶段如果串口初始化失败、首次失能命令发送不完整或循环启动失败，程序会尽力发送失能命令后直接中止启动。正常退出仍是先关闭命令门、停止循环，再发送最终失能命令。

### 主机安全保护的边界

上述等级是主机程序的决策，不等于已经证明电机物理上失能。当前主机只能证明它已经调用串口发送失能帧；尚未在本项目中证明以下硬件事实：

- 电机板对 `motors_disable` 的确切执行时延、回执与故障时行为；
- 串口断线或主机死机后，板端看门狗是否会在有界时间内失能；
- `Kd=5` 的 Passive 阻尼在所有实机姿态和负载下是否都不会导致二次危险；
- 75° 姿态阈值与立即硬失能的组合是否适合实机的每一种受保护状态。

因此，首次实机验收前必须在机器人悬空、轮子和关节无法触及人员的条件下，逐项注入 S1–S4 故障，测量实际电机响应、串口失效和板端看门狗。在得到这些结果前，物理急停、可靠支撑和安全距离仍是必需条件，不能用软件分级代替。

## 8. 升级和回滚

### 升级

每次升级都重复同一流程：

1. 开发机完成修改、单元测试和 Sim2Sim；
2. 开发机提交验证通过的内容，并把 Sim2Sim 结果和完整提交哈希交给部署机；
3. 部署机取得该提交；
4. 部署机构建到新的 `build/lw_deployments/<提交短哈希>/`；
5. 部署机完成部署包、动态库和实机安全检查；
6. 加载新目录的 `setup.bash` 后进行实机实验。

不要覆盖旧部署目录，也不要在部署目录中替换模型、配置或可执行文件。需要修改任何受清单管理的文件时，应提交修改并生成新的部署版本。

### 回滚

回滚不需要切换整个项目的 Git 分支。打开新终端，将 `DEPLOY_PREFIX` 改为上一个已经验收的目录，重新执行：

```bash
source /opt/ros/humble/setup.bash
source "$DEPLOY_PREFIX/setup.bash"
"$DEPLOY_PREFIX/lib/rl_sar/rl_real_LW" --verify-deployment-only
```

验收和安全检查通过后，再执行实机启动命令。

## 9. 可选：从开发机复制部署版本

只有开发机和部署机的 CPU 架构、操作系统、ROS 版本、C/C++ ABI、推理库版本及相关路径兼容时，才考虑在开发机构建后复制完整部署前缀。

不能只复制 `rl_real_LW` 或 `deployment/LW`，必须复制整个 `<DEPLOY_PREFIX>`。复制到部署机后，仍必须重新执行 `--verify-deployment-only` 和 `ldd` 检查。

当前可执行文件可能记录构建机上的推理库搜索路径。因此，只要不能确认两台机器的环境兼容，就应使用本文推荐方案：在部署机自己的 `rl_sar` 项目中按指定提交本地构建。

不要把完整开发仓库中的 `library/inference_runtime` 从 x86_64 主机复制到
Jetson。即使目录结构完整，其 ELF 架构仍不兼容；当前构建会在 CMake 前明确
拒绝这种运行时。

## 10. 常见问题

### 为什么部署机已有完整项目还要生成部署版本

完整项目会继续变化，普通构建目录也可能残留旧文件。部署版本把一次正式运行使用的源码提交、二进制、四个模型和配置绑定在一起，并允许在启动前验证。它是完整项目中的“已验收运行版本”，不是项目代码的替代品。

### 部署机的离线验收能代替开发机 Sim2Sim 吗

不能。`--verify-deployment-only` 只确认文件没有缺失、篡改或版本混用，不执行策略推理闭环和机器人运动。策略行为、四种状态和两种转换必须先在开发机用 `rl_sim_LW` 完成 Sim2Sim 验证。

### 部署机必须切换到开发机的分支吗

不需要。只要部署机的 Git 仓库已经取得指定提交，构建脚本就能直接从该提交创建临时工作树。部署机当前分支和未提交文件不会进入部署版本。

### 输出目录不是空目录

脚本会报 `Output prefix must not exist or must be empty`。请选择新的版本化目录，不要覆盖已验收版本。

### ONNX Runtime 缺失

脚本会报 `ONNX Runtime dependency is missing`。检查部署机当前项目中的 `library/inference_runtime/onnxruntime` 是否完整。

### ONNX Runtime 架构不匹配

校验脚本会报告期望和实际 ELF `Machine`。删除或移出从其他 CPU 架构复制来的
`library/inference_runtime/onnxruntime`，然后在 Jetson 项目中重新运行
`./build.sh`，由下载脚本取得 Linux aarch64 版本；不要通过跳过校验继续链接。

### 临时工作树不干净或子模块初始化失败

确认指定提交中的 `.gitmodules` 和子模块提交可访问。构建依赖的源码或模型必须在指定提交中，不能依赖当前工作区的未跟踪文件。

### 清单、哈希或源码提交不匹配

部署目录可能被修改、复制不完整，或混入了其他版本的二进制和资源。停止使用该目录，从预期提交重新生成部署版本。

### ROS 找到了另一个工作区

关闭已经加载其他工作区的终端。在新终端中先加载基础 ROS，再加载目标部署目录的 `setup.bash`，然后确认：

```bash
ros2 pkg prefix serial
ros2 pkg prefix fdilink_ahrs
ros2 pkg prefix rl_sar
```

输出应与当前 `DEPLOY_PREFIX` 一致。

### 动态库缺失

使用 `ldd` 查找 `not found` 项，并在部署机安装 ABI 兼容的 ROS、推理库和系统依赖。仅有部署目录并不代表外部运行库已经齐全。

## 11. 相关文件

- 构建脚本：`src/rl_sar/scripts/build_lw_deployment.sh`
- 配置采集器：`src/rl_sar/src/lw_config_profiler.cpp`
- 配置候选分析器：`src/rl_sar/scripts/profile_lw_runtime_config.py`
- 清单生成器：`src/rl_sar/scripts/generate_lw_deployment_manifest.py`
- LW 实机启动文件：`src/rl_sar/launch/rl_real_LW.launch.py`
- LW 实机部署问题记录：`.learnings/LW_REAL_DEPLOYMENT_ISSUES.md` 中的
  `LW-010`、`LW-011` 和 `LW-017`
