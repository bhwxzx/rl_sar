# LW 快速部署与实机启动指南

本文只给出 LW 从开发验证到实机启动的标准成功路径。设计依据、参数含义、异常
处理和安全边界以[《LW 编译与部署使用说明》](LW_BUILD_DEPLOYMENT_CN.md)为准；
两份文档有歧义时，停止操作并遵循完整说明。

> [!CAUTION]
> 未完成 Sim2Sim、部署包离线验收和现场安全检查时，不得启动实机节点。机器人
> 必须可靠吊装或离地，运动范围隔离，现场人员掌握物理急停；正常节点、参数测算
> 程序和其他工具不得同时读写两个电机板串口。

## 1. 先选择执行路径

| 当前情况 | 必须执行 |
| --- | --- |
| 首次实机部署 | 本文全部步骤，包括参数测算闭环 |
| 更换主机、内核、CPU/调度方案、IMU、串口设备、控制频率、策略模型、推理运行时或关键配置 | 重新执行参数测算闭环 |
| 已有可追溯的测算、评审和最终部署记录，且上述环境均未变化 | 可跳过第 5 节，从每日启动前验收继续 |
| 报告缺失、分析失败、评审关系不清或无法确认环境未变化 | 禁止启动；按首次部署重新测算 |

如果测算候选导致 `policy/LW/base.yaml` 改动，最终提交会与原测算候选提交不同。
交付记录必须保留“测算候选提交 → 原始报告 → 人工评审 → 最终配置提交”的关系，
不能只保存一个无法追溯来源的数值。

## 2. 统一设置路径

开发机和部署机分别在自己的项目中设置 `RL_SAR_ROOT`。`SOURCE_COMMIT` 必须是
已经通过开发机 Sim2Sim 的完整 40 位提交哈希，不能使用 `latest` 或口头约定的
“最新版本”。

```bash
RL_SAR_ROOT=/home/lfr/rl_sar
cd "$RL_SAR_ROOT"

SOURCE_COMMIT=REPLACE_WITH_FULL_40_CHARACTER_GIT_HASH
SHORT_COMMIT="${SOURCE_COMMIT:0:12}"
DEPLOY_PREFIX="$RL_SAR_ROOT/build/lw_deployments/$SHORT_COMMIT"
LW_PROFILE_DIR="$RL_SAR_ROOT/build/lw_profiles/$SHORT_COMMIT"
```

`SOURCE_COMMIT` 的占位符必须替换后才能执行。部署输出目录必须不存在或为空；
不要覆盖旧部署目录，同一提交重建时使用带后缀的新目录。

## 3. 开发机：构建、Sim2Sim 和提交

```bash
source /opt/ros/humble/setup.bash
cd "$RL_SAR_ROOT"
git status --short
./build.sh
scripts/validate_lw_strict_build.sh
source "$RL_SAR_ROOT/install/setup.bash"
ros2 run rl_sar rl_sim_LW
```

上述 `./build.sh` 不带参数时构建整个 ROS 工作区，是正式 Sim2Sim 验证的
标准入口。开发期可用 `./build.sh rl_sar` 指定包及其依赖；`-c`/`--clean`
会清理构建产物，`-mj`/`--mujoco` 是含 MuJoCo 的独立 CMake 构建，完整用法
可运行 `./build.sh --help`。该开发选项不能代替上述交付前的完整工作区构建。
严格构建默认并行度为 2；
资源受限或需要调整时可显式使用：

```bash
LW_STRICT_BUILD_JOBS=4 scripts/validate_lw_strict_build.sh
```

上述默认命令不创建 Sim2Sim Plot publisher、timer 或快照缓冲。需要观察
`/LW_joint_states` 时，显式启用 Plot（默认 100 Hz）：

```bash
ros2 run rl_sar rl_sim_LW --enable-plot
```

也可选择 1–200 Hz 的整数频率，例如常规观察使用 50 Hz：

```bash
ros2 run rl_sar rl_sim_LW --enable-plot --plot-rate-hz 50
```

`--plot-rate-hz` 必须与 `--enable-plot` 同时使用；缺失、非整数、越界或重复
冲突值会使 Sim2Sim 明确拒绝启动。

如需改用非编译期策略目录，使用 `--policy-root PATH`；如需启用可选的
TorchScript 执行器网络，追加 `--use_actuator_net`。同时使用的示例为：

```bash
ros2 run rl_sar rl_sim_LW \
    --policy-root /absolute/path/to/policy \
    --use_actuator_net
```

有效策略根必须包含四套 ONNX 策略；启用执行器网络时还必须包含
`LW/robot_lab/motors/{leg,foot}_actuator_net.pt`。任一所需模型缺失或契约
不兼容都会使启动失败。这两个参数可独立使用，也可与 Plot 参数组合。

至少确认四个 ONNX 策略均为本次候选版本，腿式、轮式和两个形态转换都能完整
运行，没有加载错误、NaN、越界、持续发散、明显跳变或控制周期异常。关闭仿真后
提交所有获批的代码、配置、模型和描述资源：

```bash
git status --short
git show --stat --oneline HEAD
SOURCE_COMMIT=$(git rev-parse HEAD)
echo "$SOURCE_COMMIT"
```

把完整哈希和 Sim2Sim 结果交给部署机。提交后若又修改任何发布资源，必须重新
测试、Sim2Sim 和提交。

## 4. 部署机：生成并离线验收候选部署包

在部署机自己的项目中新开终端：

```bash
source /opt/ros/humble/setup.bash
cd "$RL_SAR_ROOT"
git status --short
git fetch --all --tags
git cat-file -e "${SOURCE_COMMIT}^{commit}"
git show --stat --oneline "$SOURCE_COMMIT"

test -d "$RL_SAR_ROOT/library/inference_runtime/onnxruntime"
bash "$RL_SAR_ROOT/scripts/validate_inference_runtime.sh" \
    onnx "$RL_SAR_ROOT/library/inference_runtime/onnxruntime" "$(uname -m)"

src/rl_sar/scripts/build_lw_deployment.sh \
    "$DEPLOY_PREFIX" \
    "$SOURCE_COMMIT"
```

`validate_inference_runtime.sh` 的完整形式为
`<onnx|libtorch> <runtime-directory> [architecture]`；本正式部署路径必须保持
上述 `onnx` 和当前机器架构校验；支持的架构名为 `x86_64`/`amd64` 和
`aarch64`/`arm64`。`build_lw_deployment.sh` 的形式为
`<empty-output-prefix> [commit]`；脚本虽允许省略提交并使用 `HEAD`，但本可追溯部署
流程必须显式传入已验证的完整 `SOURCE_COMMIT`。

构建脚本必须成功验收原部署前缀及其迁移副本。随后在未加载其他工作区的新终端
重新执行第 2 节变量块，再执行：

```bash
source /opt/ros/humble/setup.bash
source "$DEPLOY_PREFIX/setup.bash"
"$DEPLOY_PREFIX/lib/rl_sar/rl_real_LW" --verify-deployment-only

for package in serial fdilink_ahrs rl_sar; do
    test "$(realpath -m "$(ros2 pkg prefix "$package")")" = "$DEPLOY_PREFIX"
done

ldd "$DEPLOY_PREFIX/lib/rl_sar/rl_real_LW"
ldd "$DEPLOY_PREFIX/lib/rl_sar/lw_config_profiler"
ldd "$DEPLOY_PREFIX/lib/fdilink_ahrs/ahrs_driver_node"
```

`--verify-deployment-only` 必须返回 `0`，三个包必须解析到当前
`DEPLOY_PREFIX`，`ldd` 不得出现 `not found`。任一检查失败都停止部署，不得
直接修改部署目录、伪造来源文件或从其他版本补拷二进制、模型和动态库。

## 5. 首次部署或环境变化：参数测算闭环

参数测算产生的是供人工评审的候选，不会自动修改 `base.yaml`，也不能替代
Sim2Sim、物理急停或 STM32 侧失能确认。
包装工具及 `collect-host`、`collect-hardware`、`analyze` 子命令都支持
`-h`/`--help`；帮助模式只显示参数后退出，不运行 profiler 或访问硬件。

### 5.1 无硬件 CPU 和调度测量

在目标部署机上执行。此阶段不会初始化 ROS、IMU、手柄、串口或执行器设备：

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

`--cpus allowed` 会测量当前进程允许的全部逻辑 CPU；也可使用
`--cpus=-1,0,1` 选择不绑定对照组和指定 CPU。`--realtime-priorities`
接受逗号分隔的非负整数；只有部署负责人已批准正数优先级时，才可加入
`--require-realtime` 要求该正数优先级的 `SCHED_FIFO` 应用失败即中止。
`--duration-seconds` 必须为正数，默认 30；本流程显式写出默认值以固定
可比较的采样时长。

输出目录必须不存在或为空。未经部署负责人批准，不要加入正数实时优先级。

### 5.2 吊装硬件观察

开始前逐项确认：

- [ ] 机器人可靠吊装，运动范围内无人和障碍物；
- [ ] 现场人员掌握物理急停；
- [ ] 正常 `rl_real_LW` 和完整 LW launch 已停止；
- [ ] 两个电机板串口没有其他读写者；
- [ ] 当前终端加载的是同一个 `DEPLOY_PREFIX`。

只单独启动 AHRS，不启动完整 LW launch：

```bash
ros2 launch fdilink_ahrs ahrs_driver.launch.py
```

当前 `ahrs_driver.launch.py` 没有项目自定义 launch 参数，因此这条命令不应
追加串口或话题覆盖值。

在另一个终端重新执行第 2 节变量块以及 5.1 开头的 `source`、`LW_PROFILER`、
`LW_PROFILE_TOOL`、`LW_POLICY_ROOT` 和 `LW_PROFILE_DIR` 赋值，再执行以下命令；
确认字符串必须逐字匹配：

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

`collect-hardware` 还支持以下可选设置：

| 参数 | 默认值 | 约束 |
| --- | --- | --- |
| `--right-port` | `/dev/ttyLegRight` | 右电机板串口 |
| `--left-port` | `/dev/ttyLegLeft` | 左电机板串口 |
| `--imu-topic` | `/imu` | 独立 AHRS 驱动的原始 IMU 话题 |
| `--ahrs-topic` | `/euler_angles` | 独立 AHRS 驱动的欧拉角话题 |
| `--require-realtime` | 关闭 | 只能与正数 `--realtime-priority` 同时使用 |

只有在稳定设备别名或话题配置已单独评审、且测量对象与最终部署一致时，
才能追加端口或话题覆盖参数。`--duration-seconds` 必须为正数，默认 60；
`--cpu` 必须是 `-1` 或非负整数，`--realtime-priority` 必须是非负整数。

执行 `collect-hardware` 的整个测算过程中，程序不会使能电机；唯一允许进入两个
电机板串口的命令是 `motors_disable=true`。它先完成双侧初始失能写入，再启动
独立的 5 ms 失能保活，策略只做 shadow inference，输出全部丢弃。命令结束后
停止独立 AHRS 驱动。

该证明只表示上层完整写入失能包且没有发送非失能命令，不表示 STM32 已经执行
失能，也不能证明电机物理上已经失能。整个阶段仍必须保持吊装、隔离和物理急停。

### 5.3 生成评审候选并返回开发闭环

四个上限必须由机械与控制安全负责人确定。先填写变量；任何变量为空都会在分析
前停止：

```bash
MAX_SAFE_SENSOR_TIMEOUT_MS=REPLACE_WITH_REVIEWED_VALUE
MAX_SAFE_TRUSTED_IMU_TIMEOUT_MS=REPLACE_WITH_REVIEWED_VALUE
MAX_SAFE_IMU_AHRS_PAIR_AGE_MS=REPLACE_WITH_REVIEWED_VALUE
MAX_SAFE_CONTROL_GAP_MS=REPLACE_WITH_REVIEWED_VALUE

(
    for value in \
        "$MAX_SAFE_SENSOR_TIMEOUT_MS" \
        "$MAX_SAFE_TRUSTED_IMU_TIMEOUT_MS" \
        "$MAX_SAFE_IMU_AHRS_PAIR_AGE_MS" \
        "$MAX_SAFE_CONTROL_GAP_MS"; do
        if [[ ! "$value" =~ ^[0-9]+([.][0-9]+)?$ \
              || "$value" =~ ^0+([.]0+)?$ ]]; then
            echo "四个 max-safe 值必须由安全负责人填写为有限正数" >&2
            exit 2
        fi
    done

    python3 "$LW_PROFILE_TOOL" analyze \
        --base-yaml "$LW_POLICY_ROOT/LW/base.yaml" \
        --reports "$LW_PROFILE_DIR"/host/*.json "$LW_PROFILE_DIR/hardware.json" \
        --output "$LW_PROFILE_DIR/candidate-review.json" \
        --max-safe-sensor-timeout-ms "$MAX_SAFE_SENSOR_TIMEOUT_MS" \
        --max-safe-trusted-imu-timeout-ms "$MAX_SAFE_TRUSTED_IMU_TIMEOUT_MS" \
        --max-safe-imu-ahrs-pair-age-ms "$MAX_SAFE_IMU_AHRS_PAIR_AGE_MS" \
        --max-safe-control-gap-ms "$MAX_SAFE_CONTROL_GAP_MS" \
        --minimum-hardware-samples 1000
)
```

`--reports` 接受一个或多个报告，但本流程要求同一部署的全部 host 报告和
一份 hardware 报告。四个 `--max-safe-*-ms` 在程序接口中可省略，但本实机
候选评审流程要求全部填写为由安全负责人确定的有限正数。
`--minimum-hardware-samples` 必须是正整数，默认 1000；输出文件不得已经存在。

`candidate-review.json` 仅供评审。不得把它直接覆盖到当前部署包。需要采用候选时：

1. 在开发机源码树中修改 `policy/LW/base.yaml`；
2. 重新执行测试和 Sim2Sim；
3. 创建并记录新的最终提交；
4. 部署机从新提交生成新的 `DEPLOY_PREFIX`；
5. 对新前缀重新执行第 4 节的全部离线验收；
6. 保存测算候选、原始报告、评审结论和最终提交之间的追溯记录。

## 6. 每次实机启动前

- [ ] 当前 `DEPLOY_PREFIX` 与交付记录一致；
- [ ] 已确认该最终提交的测试和 Sim2Sim 记录；
- [ ] 首次部署/环境变化所需的测算与人工评审已经闭环；
- [ ] `--verify-deployment-only` 刚刚返回 `0`；
- [ ] `ldd` 和 ROS 包前缀检查通过；
- [ ] `/dev/ttyLegRight`、`/dev/ttyLegLeft`、`/dev/fdilink_ahrs` 正确且可访问；
- [ ] 没有其他进程读取 IMU 或两个电机板串口；
- [ ] 机器人可靠吊装或离地，运动范围隔离；
- [ ] 物理急停可用，现场监护人员已就位。

在刚刚完成离线验收的同一终端启动：

```bash
PYTHONDONTWRITEBYTECODE=1 ros2 launch rl_sar rl_real_LW.launch.py
```

真机 launch 只提供三个项目自定义参数：

| 参数 | 默认值 | 用途 |
| --- | --- | --- |
| `enable_keyboard:=<boolean>` | `true` | 只接受 `true`/`false`；是否从控制终端读取真机键盘 |
| `enable_debug_publisher:=<boolean>` | `false` | 只接受 `true`/`false`；是否创建 `/LW_joint_states` 调试 publisher |
| `debug_publish_rate_hz:=<integer>` | `50` | 只接受 1–200；调试 publisher 频率，关闭 publisher 时仍校验参数但不创建资源 |

无交互终端的 systemd、容器或后台运行必须显式关闭键盘：

```bash
PYTHONDONTWRITEBYTECODE=1 ros2 launch rl_sar rl_real_LW.launch.py enable_keyboard:=false
```

关闭键盘后必须事先准备独立可控恢复方式、可靠机械支撑和物理急停。只有受控
短时调试才启用 publisher：

```bash
PYTHONDONTWRITEBYTECODE=1 ros2 launch rl_sar rl_real_LW.launch.py \
  enable_debug_publisher:=true debug_publish_rate_hz:=50
```

调试 publisher 只发布新的控制源帧，发生争用或新帧覆盖时允许丢弃样本，不会
阻塞 200 Hz 控制循环。调试结束后应恢复默认关闭。三个参数可同时指定。

必须保留 `PYTHONDONTWRITEBYTECODE=1`，否则 Python 可能在清单约束的生产 launch
目录生成 `__pycache__`，完整性检查会安全拒绝启动。正常 launch 会访问真实
IMU、串口和电机；不能用它测试部署包是否完整。

## 7. 停止、失败和回滚

- 任一检查或启动步骤报错：停止操作，不绕过校验，不现场修改部署包。
- 机器人出现非预期动作：立即使用物理急停，并停止 launch。
- 回滚：选择上一个完整、已验收的 `DEPLOY_PREFIX`，重新执行第 4 节离线验收和
  第 6 节安全清单；不能只替换可执行文件、模型或 YAML。
- 依赖、运行时、串口规则、清单、ROS 工作区或动态库问题，查阅完整说明的
  [常见问题](LW_BUILD_DEPLOYMENT_CN.md#10-常见问题)。
