# LW 编译与部署使用说明

本文说明如何从已提交的源码版本生成 LW 实机正式部署包、验证部署包、迁移安装目录，以及在完成安全检查后启动实机程序。

> [!CAUTION]
> 实机程序会连接传感器、串口和执行器。首次运行或更换模型、配置、硬件环境后，应先完成本文的离线验收，并在机器人可靠吊装或离地、急停可用且现场有人监护的条件下试运行。

## 1. 部署包保证什么

正式部署构建会：

- 从指定 Git 提交创建干净的临时工作树，并初始化该提交锁定的子模块；
- 使用 `Release` 和 `LW_PRODUCTION_DEPLOYMENT=ON` 构建；
- 安装 `rl_real_LW`、五个 YAML 配置、四个正式 ONNX 模型和两个状态转换 CSV；
- 生成 `manifest.yaml`，记录源码提交、构建类型、可执行文件及部署资源的 SHA-256；
- 拒绝部署目录中的符号链接，并自动执行一次只读部署验收。

四个正式模型分别用于：

- `leg_loco`：腿式运动；
- `leg_to_wheel`：腿式向轮式转换；
- `wheel_loco`：轮式运动；
- `wheel_to_leg`：轮式向腿式转换。

部署清单只约束 `rl_real_LW` 和上述 LW 策略资源。ROS、LibTorch、ONNX Runtime、Python、操作系统动态库及硬件驱动属于外部运行依赖，不包含在清单中，也不会随部署包自动打包。

## 2. 构建前准备

在构建机上确认：

1. 已安装并加载项目支持的 ROS 2 环境，例如 ROS 2 Humble；
2. `git`、`cmake`、`colcon`、C++ 编译器及 [README_CN.md](../README_CN.md#依赖) 中的依赖可用；
3. 仓库子模块配置有效；
4. 本地推理运行库存在：

   ```text
   library/inference_runtime/onnxruntime
   library/inference_runtime/libtorch
   ```

5. 准备部署的源码、配置和四个 ONNX 模型都已提交到 Git。

构建脚本只使用指定提交中的文件。当前工作区尚未提交的修改和未跟踪模型不会进入部署包。

## 3. 生成正式部署包

在仓库根目录执行：

```bash
cd /home/lfr/rl_sar
source /opt/ros/humble/setup.bash

SOURCE_REVISION=$(git rev-parse HEAD)
DEPLOY_PREFIX=/home/lfr/rl_sar/build/lw_deployment_$(git rev-parse --short "$SOURCE_REVISION")

src/rl_sar/scripts/build_lw_deployment.sh \
    "$DEPLOY_PREFIX" \
    "$SOURCE_REVISION"
```

脚本参数为：

```text
build_lw_deployment.sh <空的输出目录> [Git 提交]
```

- 第一个参数必须指向不存在或内容为空的目录；脚本不会覆盖已有部署包。
- 第二个参数可以是完整提交哈希、短哈希、标签或其他可解析为提交的 Git 引用；省略时使用 `HEAD`。
- 为便于审计，正式发布建议传入构建开始前保存的完整提交哈希。
- 推理运行库来自当前主工作区的 `library/inference_runtime`，不会从临时工作树复制。

构建成功后，脚本会打印源码提交、安装前缀、清单路径和启动命令。主要产物如下：

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

## 4. 离线验收

构建脚本会自动执行同样的检查。交付、复制或切换部署包后，应再次手动执行：

```bash
source /opt/ros/humble/setup.bash
source "$DEPLOY_PREFIX/setup.bash"
"$DEPLOY_PREFIX/lib/rl_sar/rl_real_LW" --verify-deployment-only
```

`--verify-deployment-only` 会验证：

- 当前可执行文件的哈希和编译时源码提交；
- 清单格式、构建类型及资源路径；
- 五个 YAML、四个 ONNX 和两个 CSV 的存在性与哈希；
- 部署资源没有越出部署根目录，也没有经过符号链接。

该模式不会创建 ROS 节点，也不会初始化手柄、串口或控制线程。命令返回码为 `0` 才表示验收通过；任何报错都应停止部署，重新从可信提交构建，不能直接修改安装目录来绕过检查。

可用以下命令查看部署包对应的源码版本和文件清单：

```bash
sed -n '1,220p' \
    "$DEPLOY_PREFIX/share/rl_sar/deployment/LW/manifest.yaml"
```

还应检查动态库是否全部可解析：

```bash
ldd "$DEPLOY_PREFIX/lib/rl_sar/rl_real_LW"
```

若输出中出现 `not found`，不得启动实机程序。

## 5. 迁移到运行机器或其他目录

必须复制完整安装前缀，不能只复制可执行文件或 `deployment/LW`：

```bash
RELOCATED_PREFIX=/opt/rl_sar/lw_release_001
sudo mkdir -p "$RELOCATED_PREFIX"
sudo cp -a "$DEPLOY_PREFIX/." "$RELOCATED_PREFIX/"
```

在一个未加载旧部署包的新终端中验证：

```bash
source /opt/ros/humble/setup.bash
source "$RELOCATED_PREFIX/setup.bash"

ros2 pkg prefix rl_sar
"$RELOCATED_PREFIX/lib/rl_sar/rl_real_LW" --verify-deployment-only
ldd "$RELOCATED_PREFIX/lib/rl_sar/rl_real_LW"
```

`ros2 pkg prefix rl_sar` 应输出新的安装前缀，只读验收应成功，并且 `ldd` 不应包含 `not found`。

部署资源查找支持安装前缀整体迁移，但这不等于部署包包含全部运行库。目标机器仍需具备兼容的 CPU 架构、ROS 发行版、C/C++ ABI、LibTorch、ONNX Runtime、系统动态库和硬件驱动。当前可执行文件还可能记录构建机上的推理库搜索路径，因此跨机器交付前必须在目标机检查 `ldd`，并按目标环境正确安装对应版本的推理库。

## 6. 实机启动

启动前至少确认：

- 已在当前部署目录执行 `--verify-deployment-only` 且通过；
- `ldd` 没有缺失依赖；
- 机器人型号、关节映射、限位、初始姿态和四个策略版本正确；
- IMU、串口、执行器及手柄连接已分别验证；
- 机器人已可靠吊装或离地，运动范围内无人和障碍物；
- 硬件急停、电机失能方式及现场监护人员均已就位。

确认后在新终端中执行：

```bash
source /opt/ros/humble/setup.bash
source "$DEPLOY_PREFIX/setup.bash"
ros2 launch rl_sar rl_real_LW.launch.py
```

正常启动会加载 AHRS 驱动并运行 `rl_real_LW`，随后可能访问真实硬件。不要用正常启动命令代替离线验收。

## 7. 发布、升级与回滚

建议将每个部署前缀视为不可变发布物：

1. 为发布选定一个已经提交并经过评审的 Git 提交；
2. 构建到一个全新的、带版本号或提交短哈希的目录；
3. 保存 `manifest.yaml`、完整提交哈希和验收记录；
4. 在目标机完成重定位、只读验收、动态库和硬件检查；
5. 通过切换所加载的安装前缀升级或回滚。

不要覆盖旧部署目录，也不要在部署目录内替换模型、配置或可执行文件。需要更新任何受清单管理的文件时，应提交修改并重新生成一个新部署包。回滚时加载并验证上一个已验收的完整前缀。

## 8. 常见问题

### 输出目录不是空目录

脚本会报 `Output prefix must not exist or must be empty`。请选择新的版本化目录；确认不再需要旧发布物后，再按团队的数据保留流程处理旧目录。

### ONNX Runtime 缺失

脚本会报 `ONNX Runtime dependency is missing`。检查当前主工作区的 `library/inference_runtime/onnxruntime`，以及相关子模块或本地依赖是否完整。

### 临时工作树不干净或子模块初始化失败

确认指定提交中的 `.gitmodules` 和子模块提交可访问。构建依赖的源码或模型必须先提交，不能依赖主工作区中的未跟踪文件。

### 清单、哈希或源码提交不匹配

部署包可能被修改、复制不完整，或混入了其他版本的二进制和资源。停止使用该目录，从预期提交重新构建完整部署包。

### 迁移后 ROS 找到旧目录

关闭已加载旧工作区的终端，在新终端依次加载基础 ROS 环境和新部署前缀，再用 `ros2 pkg prefix rl_sar` 确认解析结果。

### 目标机动态库缺失

使用 `ldd` 定位 `not found` 项，并安装与构建环境 ABI 兼容的 ROS、推理库和系统依赖。仅复制安装前缀不能解决外部动态库依赖。

## 9. 相关文件

- 构建脚本：`src/rl_sar/scripts/build_lw_deployment.sh`
- 清单生成器：`src/rl_sar/scripts/generate_lw_deployment_manifest.py`
- LW 实机启动文件：`src/rl_sar/launch/rl_real_LW.launch.py`
- LW 实机部署问题记录：`.learnings/LW_REAL_DEPLOYMENT_ISSUES.md` 中的 `LW-010`
