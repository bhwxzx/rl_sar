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

第一次构建建议编译整个工作区。依赖已经构建完成后，也可以使用 `./build.sh rl_sar` 做增量构建。

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
test -d "$RL_SAR_ROOT/library/inference_runtime/libtorch"
```

还需要保证 `git`、`cmake`、`colcon`、C++ 编译器及 [README_CN.md](../README_CN.md#依赖) 中的依赖可用。

> [!NOTE]
> 正式部署清单会校验 LW 可执行文件、模型和配置，但不会打包或校验 ROS、LibTorch、ONNX Runtime、Python、操作系统动态库和硬件驱动。它们由部署机的项目和系统环境提供。

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
4. 安装 `rl_real_LW`、五个 YAML、四个 ONNX 和两个状态转换 CSV；
5. 生成记录源码提交和各文件 SHA-256 的 `manifest.yaml`；
6. 拒绝关键部署文件中的符号链接；
7. 自动执行一次 `--verify-deployment-only` 离线验收。

输出目录必须不存在或为空。脚本不会覆盖以前的部署版本。如果同一个提交需要重新构建，应使用新的带后缀目录，例如 `${SHORT_COMMIT}_02`，并保留必要的旧版本供回滚。

主要产物位于：

```text
<DEPLOY_PREFIX>/
├── setup.bash
├── lib/rl_sar/rl_real_LW
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
- 资源路径是否仍在部署目录内；
- 关键部署文件是否经过符号链接。

只有命令返回码为 `0` 才表示部署包完整性验收通过。它不能证明策略行为正确，因此必须确认该提交已经在开发机完成 Sim2Sim。任何报错都应停止部署，不能通过直接修改部署目录来绕过检查。

查看清单和动态库：

```bash
sed -n '1,220p' \
    "$DEPLOY_PREFIX/share/rl_sar/deployment/LW/manifest.yaml"

ldd "$DEPLOY_PREFIX/lib/rl_sar/rl_real_LW"
```

`manifest.yaml` 中的 `source_commit` 应等于开发机交付的完整提交哈希。`ldd` 输出中如果出现 `not found`，说明部署机缺少运行库，不得启动实机程序。

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

### 临时工作树不干净或子模块初始化失败

确认指定提交中的 `.gitmodules` 和子模块提交可访问。构建依赖的源码或模型必须在指定提交中，不能依赖当前工作区的未跟踪文件。

### 清单、哈希或源码提交不匹配

部署目录可能被修改、复制不完整，或混入了其他版本的二进制和资源。停止使用该目录，从预期提交重新生成部署版本。

### ROS 找到了另一个工作区

关闭已经加载其他工作区的终端。在新终端中先加载基础 ROS，再加载目标部署目录的 `setup.bash`，然后确认：

```bash
ros2 pkg prefix rl_sar
```

输出应与当前 `DEPLOY_PREFIX` 一致。

### 动态库缺失

使用 `ldd` 查找 `not found` 项，并在部署机安装 ABI 兼容的 ROS、推理库和系统依赖。仅有部署目录并不代表外部运行库已经齐全。

## 11. 相关文件

- 构建脚本：`src/rl_sar/scripts/build_lw_deployment.sh`
- 清单生成器：`src/rl_sar/scripts/generate_lw_deployment_manifest.py`
- LW 实机启动文件：`src/rl_sar/launch/rl_real_LW.launch.py`
- LW 实机部署问题记录：`.learnings/LW_REAL_DEPLOYMENT_ISSUES.md` 中的 `LW-010`
