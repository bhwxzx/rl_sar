# LW 实机部署问题记录

本文件是 LW 整改工作的唯一权威入口，记录当前待办顺序与历史索引。

## 执行规则

- 由用户逐项选择问题；修改代码前先说明具体方案并获得明确审批。
- 每次只处理一个已选问题，不捆绑后续问题或无关清理。
- 验证后只更新所选问题的状态、解决说明和验收证据。
- 编号永久保留，不因归档重新编号。若以后确认重复，保留原编号并标明合并去向。
- 状态：`pending` 待处理；`in_progress` 已批准实施；`resolved` 已验证完成；
  `deferred` 暂缓；`wont_fix` 不处理（须说明原因）。
- 历史归档仅提供追溯，不作为新的待办清单。具体审批和执行状态以本文件为准。

## 当前状态

截至 2026-09-10，历史 LW-001～LW-071 共 71 项均已结项。
LW-072 已完成批准范围内的修改与离线验证，当时无待处理条目；2026-09-12 新增已批准的 LW-073，见下文。GetDown 已取消
姿态角度限制，其他受保护状态姿态越限改为当周期进入锁存的 Passive 阻尼，
程序保持运行。
结项仅表示各项批准范围内的修改和所记录验证已完成，不代表所有实机风险已消除。

## 当前选定问题

### [LW-079] 固定输出回归采用绝对与相对误差容限

**状态**： resolved
**批准日期**：2026-09-17

- 用户在诊断后明确批准，将固定模型输出比较改为 `abs(actual - expected) <= 2e-6 + 1e-6 * abs(expected)`，显式拒绝 NaN/Inf，并输出期望值、实际值、绝对误差和允许误差。
- 仅修改配置回归测试中的固定输出比较、增加容差边界及非有限值验证；模型、固定基线、运行时和配置不变。基于 `d4ce20b`，开始前工作区干净。
- 诊断证据：Jetson C++ ONNX Runtime 1.22.0 下腿式索引 2/6 偏差分别约 `1.0133e-6`/`1.9073e-6`；重复推理一致，模型哈希吻合，另外三个策略最大偏差约 `2.3842e-7`。详见 `/tmp/lw-model-baseline-review-z1ybh54x/results.json`。
- 用户先批准实现与离线验证，随后明确要求 Git 提交并推送；不包含 Sim2Sim 或真实硬件操作。

#### 解决记录

- 解决时间：2026-09-17T20:57:46+08:00；基于 `d4ce20b`，实现与验证完成后按用户要求提交并推送。
- `test_lw_configuration_validation.cpp` 的固定输出比较使用 `2e-6 + 1e-6 * abs(expected)`，以 double 计算误差及允许误差，实际值和期望值均须有限；失败打印策略、索引、expected、actual、abs_error、allowed_error。
- 增加近零、正负 1、100 等幅值在容差内外相邻 float 值的边界检查，复现 Jetson 两个微小偏差可接受、明显偏差仍失败；NaN、正负 Inf 分别作为实际值、期望值或双方时均拒绝，核验失败信息包含四项数值字段。
- 使用现有非生产 build/rl_sar 重编译 `test_lw_configuration_validation` 成功；定向 CTest `lw_configuration_validation` **1/1 通过**（0.21 秒），完整执行四个模型固定输入回归及该测试内其它配置校验。未修改或跳过基线，不宣称全仓回归/Sim2Sim/硬件验收通过。
- 验证目录 `/tmp/lw079-4xca2kdv/`，保存 commands.json、validation.log、test-before.cpp、policy-hashes-before.json、verification.json 和 changes.patch；`policy/LW` 下全部文件哈希前后一致，固定基线块逐字节一致，`git diff --check` 通过。
- 该修复只影响回归测试判断和错误诊断，不改变正式推理、运行时安全阈值、模型或已提交的参数配置。

### [LW-078] 分析完成后打印候选参数摘要

**状态**： resolved
**批准日期**：2026-09-17

- 用户明确批准：保持现有 analyze 命令，在 JSON 成功保存和最终输入一致性校验通过后打印全部候选参数、秒/毫秒换算和评审状态；对保留原值、待人工评审或待硬件测量进行标识，失败时不打印成功摘要。
- 仅更新分析脚本、快速指南和本项记录，运行现有回归并核对终端与 JSON。候选算法、schema 和配置写入行为不变，不操作硬件。
- 开始时 `policy/LW/base.yaml` 和 `docs/LW_QUICK_START_CN.md` 已有用户修改；保留全部现有内容，仅在快速指南补充摘要说明。用户参数修改不属于本项。
- 用户随后明确授权本项 Git 提交和推送；既有部署包、用户参数配置和其它指南改动不纳入本项提交。

#### 解决记录

- 解决时间：2026-09-17T20:37:59+08:00；基于 `17a1e8d`，本记录随本项修改一并提交，标题为 `分析完成后打印候选参数摘要（LW-078）`。
- 分析器在 JSON 写入及最后一次输入一致性复核成功后，使用同一 result 打印 11 项参数、时间单位换算、bool 小写和评审状态，浮点格式使用 9 位有效数字以消除展示尾差；计算结果和 JSON 内容不变。
- 显示区分吊装候选、一般候选、仅已测选项、保留原值待硬件测量/人工评审，以及保持关闭待物理验证。完整命令参数保持兼容，报告路径继续输出；失败路径不输出成功摘要。
- 现有分析器回归 **22/22 通过**。使用 `17a1e8d` 真实 schema v4 报告在临时目录重做分析，与既有 candidate-review.json 完整 JSON 相等，11 个参数各显示一次；43.2/15/9.6/0.7/15 ms 显示正确。另核验省略人工上限、仅 host、输出文件已存在及输入缺失场景；失败不打印摘要、不覆盖旧文件。
- 证据：`/tmp/lw078-ffqw0cxd/` 中 commands.json、verification.json、各场景 stdout/stderr 和临时输出报告。只读取原始报告和部署包，不运行 profiler 或真实硬件。
- `policy/LW/base.yaml` 与修改前备份逐字节一致；快速指南去除本项新增四行说明后与用户原文一致。保留的用户修改备份及哈希在同一验证目录。`git diff --check` 通过。
- 当前变化位于源码分析脚本；已发布部署包中的脚本仍为原版本，可通过 LW_PROFILE_TOOL 指向源码脚本使用新摘要，或后续随新部署包发布。已有输出文件继续禁止覆盖。

### [LW-077] 修正吊装测算结束阶段的年龄统计

**状态**： resolved
**批准日期**：2026-09-17

- 用户在已收到具体方案后明确要求“继续LW-077的代码修改任务”，批准本项实现与离线验证。
- 范围：统一传感器采样截止，等待在途短记录事务后冻结入口；释放采样锁后计算统计，延后各策略推理分位数；保留真实反馈间隔、首样本延迟和结束年龄。
- 报告升级 schema v4，记录统一 steady 窗口和末次采样偏移；分析器校验共同截止及完整间隔总和，拒绝旧报告并要求重采。
- 保留独立失能保活、最终失能发送及失败检查；不修改 base.yaml、四项 MAX_SAFE 安全上限、模型或正式实机控制行为，不启动真实硬件。
- 基于 b9e821e，开始前工作区干净；此前 skill 修改已提交推送，本项不混入其它问题。用户随后明确授权本项 Git 提交和推送；发布新部署包仍不包含在本轮范围内。

#### 解决记录

- 解决时间：2026-09-17T19:58:18+08:00；基于 `b9e821e`，本记录随本项修改一并提交，标题为 `修正吊装测算统一截止与结束年龄统计（LW-077）`。
- `LWProfileSamplingWindow` 用短事务锁统一接收时间并处理正常/失败截止；同次左右反馈及单次 IMU 的原始、可信和配对记录保持完整。截止后不再接收统计，关闭操作幂等，不因退出和排序推迟截止。
- `lw_config_profile.hpp` 将样本捕获与分位数排序分离，源锁外排序；`snapshotSince(start, end)` 使用显式共同截止并拒绝越界时间。profiler 在最终策略 shutdown 前关闭窗口，首次 fail、构造异常和析构也收尾；全部推理分位数延后计算。串口失能保活仍独立运行，最终失能发送和写失败统计保留。
- profiler 和候选分析报告升级 schema v4；硬件报告保存 steady 窗口起止/时长和各源末次偏移。分析器以 1 微秒绝对容差核验共同截止及完整间隔总和；旧 schema 拒绝并提示重新采集。候选公式及人工安全上限要求不变。
- 验证目录 `/tmp/lw077-byb15f2e/`：`commands.json`、`build.log`、`tests.log`、`changes.patch`。使用系统 Python、ROS Humble 和现有非生产 `build/rl_sar`，`LW_PRODUCTION_DEPLOYMENT=OFF`、`LW_STRICT_WARNINGS=OFF`；不宣称生产 Release 或 strict 构建通过。编译 `test_lw_config_profile`、`lw_config_profiler` 成功，构建日志无 warning/error。
- 定向 CTest **4/4 通过**（1.25 秒）：`lw_config_profile`、`lw_runtime_config_analyzer`、`lw_config_profiler_help`、`lw_config_profiler_integration`。分析器 **22 项 Python 测试**通过（Windows 单独运行及 Jetson CTest）；`git diff --check` 通过。
- 新增 C++ 回归覆盖统一截止、统计延后不改变左右同刻年龄、拒绝迟到记录、重复关闭、未出现来源、窗口越界、并发在途 IMU 事务及独立样本快照；分析回归覆盖旧版本、缺失/矛盾时间、错误语义和时钟，以及真实 40 ms 结束年龄仍阻止突破 20 ms 人工上限。集成测试只运行 host-only、错误确认和不存在的临时串口路径，不访问真实硬件。
- 已同步两份中文部署指南。`policy/LW/base.yaml`、模型和既有部署包/报告未修改；未启动吊装或正式实机。后续需发布新部署包、重新采集 schema v4 主机及硬件报告后再评审候选；离线验证不代表现场 timing 改善幅度已测得。

### [LW-076] 新模型配置与测试基线同步

**状态**： resolved
**批准日期**：2026-09-17

- 用户明确批准本项最小方案：Wheel 历史索引从 5 帧改为从旧到新的 10 帧；配置回归测试的 Wheel 输入维度从 195 改为 390，同步 Leg/Wheel 固定推理输出、归档日期和模型哈希。
- 新模型归档：Leg `2026-09-16-15-45-12`，ONNX SHA-256 `c5d94cd109557baaf3b3a57b9e146a95555ea7c323b538fcf8830aea829f42eb`；Wheel `2026-09-16-15-12-04`，ONNX SHA-256 `b5f71d83ce6b8f9acdf73b8e7c4093fefc747db8ec9a08ae6bce72cdb5e9626e`。均已与归档清单和工作区权重核对一致。
- 保留用户已有的两份 ONNX、Leg 命令缩放 `[0.6,0,0.5]`、Wheel 命令缩放 `[0.8,0,0.8]` 和未跟踪的 `library/`。实施时发现 Wheel 历史索引已更新到批准值，保留该现有修改。运行时源码、PD、动作、历史/reset 实现及两种转换策略基线不变；保留维度校验和正式测试 `1e-6` 容差。
- 验收：根据新归档 JIT 权重及前向结构进行 NumPy 独立复算，再运行三项原失败测试和完整 54 项回归。独立复算门限 `1e-5`，不等同于 JIT 引擎执行。
- 边界：本项只处理已确认的尺寸和测试基线问题。归档声明环境历史重置，但缺少匹配版本的训练端逐项观测配置及 reset 填充证据，不能宣称完整部署契约已全部核验。不安装依赖，不训练，不启动实机或闭环评估。用户在验证完成后明确授权本地 Git 提交，将已验证的新模型及现有命令缩放一并纳入；未授权推送，未跟踪的 `library/` 不纳入提交。

#### 解决记录

- 解决时间：2026-09-17T16:29:49+08:00；基于 `1d31681`。本记录随本项修改一并提交，标题为 `同步 LW 新模型、历史配置与推理测试基线（LW-076）`；未推送。
- 生效文件：`policy/LW/robot_lab/wheel_loco/config.yaml` 使用现有已更新的 `[9,8,7,6,5,4,3,2,1,0]` 历史索引，保持 `time` 排列和单帧 39 维；`src/rl_sar/test/test_lw_configuration_validation.cpp` 更新 Wheel 输入为 390，并同步两份模型的固定输出、归档日期和 SHA-256；本记录只新增和更新 LW-076。
- 两份新 JIT 均为 ROADeploymentWrapper，当前帧取历史末尾，actor 拼接顺序为 current_obs、code_vel、hist_latent；actor_obs_normalizer 实际为 Identity。历史编码已包含在模型内，现有 C++ 历史容量和展平长度由配置驱动，本项未修改运行时源码。
- 独立验证：归档 ONNX/JIT 哈希及部署 ONNX 哈希匹配；从新 JIT 权重和已检查的前向结构进行 NumPy float64 复算，与现有 C++ ONNX Runtime 固定输入输出的最大绝对差为 Leg `1.5967867916799605e-6`、Wheel `3.2118387167656692e-6`，均低于独立核对门限 `1e-5`。正式固定输出测试仍为 `1e-6` 容差。
- 验证目录：`/tmp/lw076-20260917/`，含 `commands.json`、`crosscheck.py/json`、`probe.cpp`/`probe`、两份归档前向结构摘录以及构建、定向和完整测试日志。探针源码与二进制复用此前已检查的 C++ ORT 工具，独立复算按新 ROA 架构处理两个模型，未使用旧 Wheel DWAQ 计算路径。
- 构建：`cmake --build build/rl_sar --target test_lw_configuration_validation test_lw_runtime_parity lw_config_profiler -j2` 成功，日志未出现 warning/error。交接中的 `/tmp/lw-strict-build.KjwdkQ` 已不存在；此次使用现有普通 build（`LW_STRICT_WARNINGS=OFF`），不宣称完成 strict build 验证。
- 定向 CTest **3/3 通过**：`lw_runtime_parity`、`lw_configuration_validation`、`lw_config_profiler_integration`。随后 `ctest --test-dir build/rl_sar --output-on-failure` **54/54 通过**，10.81 秒。`git diff --check` 通过。
- 原有模型和命令缩放保留，未操作实际串口、启动实机或闭环评估。完整回归通过仅证明本项离线验收通过；逐项训练观测和 reset 填充的证据限制仍保留，不据此声明 hardware-ready。

### [LW-075] 关闭吊装测算逐条力矩告警

**状态**： resolved
**批准日期**：2026-09-17

- 用户明确批准本项方案；仅关闭吊装测算的逐条计算力矩超限打印，主机测算继续保持关闭。
- 实施范围：`lw_config_profiler.cpp` 复用现有 `print_torque_warnings` 开关并固定为 `false`；更新共享开关注释及两份部署指南。
- 保留超限检测和 `TorqueLimitWarning` 诊断事件；正式实机与 Sim2Sim 默认打印行为、策略输出、5 ms 失能保活、错误输出和失败退出处理不变。不增加开关参数或结束汇总。
- 验收范围：编译 profiler，运行现有共享运行时打印/检测一致性回归和 profiler 无硬件集成测试。实际吊装复测不包含在本次离线验证中。
- 本轮初始工作区干净；保留已有部署包和测算报告。用户随后于 2026-09-17 明确授权提交并推送；部署包发布不在本次范围内。
- 执行期间检测到用户同时保存快速指南，将主机测算命令改为 `--cpus -1`；已保留该调整，并在最新文档上补回本项说明。该 CPU 参数调整不属于本项代码修改。

#### 解决记录

- 解决时间： 2026-09-17T11:34:57+08:00
- 提交：本记录随本项修改一并提交，标题为 `关闭吊装测算逐条力矩告警（LW-075）`；基于 `df0d4eccd41a837c164ed4958fa0fcf049004ee2`。
- 修改文件：`src/rl_sar/src/lw_config_profiler.cpp`、`src/rl_sar/library/core/safety/lw_runtime_core.hpp`（仅注释）、`docs/LW_BUILD_DEPLOYMENT_CN.md`、`docs/LW_QUICK_START_CN.md` 及本记录。
- 实施：profiler 在共享推理调用前统一设置 `print_torque_warnings=false`。复用现有检测路径，不改变 `TorqueLimitWarning` 事件或输出计算；共享默认值仍为 `true`。串口失能、保活、失败退出、模型和配置均未修改。
- 隔离验证目录：`/tmp/lw075-8xb9hgeg`。`commands.json` 保存 CMake 配置、构建和 CTest 参数；`configure.log`、`build.log`、`tests.log` 保存执行结果。使用系统 Python、ROS Humble、Release 和非生产测试配置；未覆盖已有 build/install 程序或部署包。
- 编译 `lw_config_profiler`、`test_lw_runtime_parity` 成功；定向 CTest **3/3 通过**：`lw_runtime_parity`、`lw_config_profiler_help`、`lw_config_profiler_integration`。
- 既有 `testTorqueWarningPrintingDoesNotChangeDetection` 验证默认打印与静默模式的输出区别，同时确认超限诊断事件、非阻尼锁存状态及位置、速度、计算力矩输出一致。profiler 集成测试仅运行 host-only 和硬件拒绝启动路径；后者使用错误确认字符串或不存在的临时串口路径，未访问真实电机板。
- `git diff --check` 通过。验证不包含实际吊装、正式实机或 Sim2Sim 运行，也未测量硬件模式关闭打印后的性能改善幅度。
- 后续事项：提交新版本后按发布流程生成并离线验收新的部署包，再安排现场吊装复测；现有部署包和历史报告保持原样。

### [LW-074] 对齐四个 LW 策略的动作目标裁剪顺序

**状态**： resolved
**批准日期**：2026-09-15

- 用户确认所有训练策略的 `agent_cfg.clip_actions=None`，不存在外层原始动作裁剪，并明确批准简化方案；该事实来源于用户确认，不冒充本机历史训练运行时采集。
- 批准范围：Leg、Wheel、Leg→Wheel、Wheel→Leg 共用运行时删除原始动作裁剪，改为缩放/默认值偏置后裁剪位置或轮速度目标，PD 使用最终目标；保留原始 previous action 和已有观测裁剪。
- 不增加裁剪模式或配置开关；保留 `clip_actions_lower/upper` 名称和数值，明确其目标单位。保持模型、PD、动作缩放、默认姿态、历史、reset、步态相位、控制时序与场景。
- 本次仅源码、配置注释、离线测试和说明记录；不启动闭环评估、实机程序、训练，不安装依赖，不提交推送。
- 保留原有脏文件：两个 locomotion ONNX、`scene.xml`；保留未跟踪的 `library/` 与上下文检查技能目录。

#### 解决记录

- 解决日期：2026-09-15；用户随后授权提交。本记录随源码提交，标题为 `对齐 LW 四策略动作目标裁剪顺序（LW-074）`；未推送。
- `lw_runtime_core.hpp` 保留模型输出形状/有限性检查，删除原始动作限幅，下一帧 actions 仍经过既有观测裁剪。`rl_sdk.cpp::ComputeLWOutput` 按关节控制模式，在 float32 缩放和默认角偏置后使用现有上下界裁剪目标，再计算 PD 力矩；当前轮速度默认偏置仍为零。计算溢出保留为非有限输出，由既有 PolicyOutputInvalid 路径拒绝。
- 配套修改：四个策略 YAML 及 `lw_configuration_validation.hpp` 仅补充目标限幅语义注释；`docs/LW_BUILD_DEPLOYMENT_CN.md` 说明动作、previous action 与单位；`test_lw_runtime_parity.cpp` 更新裁剪回归测试。未新增配置字段或模式。
- 四策略分别覆盖六类目标（-101、-100、0、3、100、101），每类验证初始推理、重复输入去重、下一帧 previous action、历史仅推进一帧与重新激活清零；验证 raw action 不被修改、非零默认角、位置/轮速度目标与最终目标对应的 PD。使用动作专用观测夹具隔离该契约，同时保留原有实际模型推理及相位等回归测试。
- NaN、正负 Inf 原始动作仍触发 PolicyActionInvalid 与被动阻尼；有限原始动作在位置/轮速度缩放中溢出时，目标裁剪不掩盖 PolicyOutputInvalid。
- 隔离构建/日志目录：`/tmp/lw074-20260915-8z2m0il1`。精确编译和链接 argv 见 `commands.json`、`verification.json`；运行命令为该目录下对应测试二进制，各日志与测试同名。
- 测试 **5/5 通过**：`test_lw_runtime_parity`、`test_lw_configuration_validation`、`test_lw_allocation_bound`、`test_lw_control_safety`、`test_lw_policy_output_transport`。配置测试的固定模型输出基线使用 `git archive HEAD policy/LW` 提取的独立资源；运行时测试使用当前工作区资源，未替换模型或改写固定输出基线。
- `rl_real_LW.cpp`、`rl_sim_LW.cpp` 隔离编译通过；未更新 build/install 中已安装程序，未启动节点或物理仿真。实机和 Sim2Sim 需后续重新构建才会使用本次源码。
- 四份 YAML 与 HEAD 解析后逐值相同，仅注释变化；`git diff --check` 通过。原有两个 ONNX 的当前哈希仍为 `8d3ee152ca53c9f835318023e3095f868191c960c433c5a897317b2a7e38ce3b`（Leg）和 `1c08aa136ffd2039e1f21ddb435ec0fd411c726c18fa9462490fce25fc417ef1`（Wheel）。
- 验证边界：结项仅覆盖动作处理实现及离线验证；没有本次闭环或实物改善证据，不据此解释站立漂移或声明 hardware-ready。

### [LW-073] 修正 LW_Leg AMP-ROA 步态相位首帧时序偏移

**状态**： resolved
**批准日期**：2026-09-12

- 用户明确批准本次方案；仅处理步态相位提前一个策略周期的问题。
- 修正范围：共享运行时中声明 gait_phase 的策略先使用当前相位，成功发布后推进；无该观测的策略保留旧路径。保持 float32、历史、PD、reset 与动作处理。
- 配套诊断统一修正普通机身/惯性坐标系速度标签与计算，独立 A/B 构建；seed42，五用例每条件20秒，共200秒。
- 证据目录：`/home/lfr/sim2sim_test/sim2sim_diagnostics/20260912-202155-leg-phase-ab`。
- 联合 reset 另行处理；评估期间未操作实物、安装依赖或提交推送。已完成下述有限验证，不表示 hardware-ready。

#### 解决记录

- 修改完成：2026-09-12；用户随后授权提交。本记录与源码一并提交，标题为 `修正 LW Leg 步态相位首帧时序（LW-073）`；未推送。
- `lw_runtime_core.hpp`：仅声明 gait_phase 的策略使用当前相位并在输出成功发布后推进下一周期；保持原 float32 加法/回绕。旧输入去重与 activation 工作区重置保留，失败返回不推进相位。
- 首帧相位为0；2秒 A 输入[-0.15643037855625153,-0.9876889586448669]，B为[3.965692940255394e-06,-1]。20ms偏移去除；20秒内原 float32 递推相对理想时钟最大残差7.181微秒，未改用双精度或乘法时钟。
- 验证：隔离编译运行 `test_lw_runtime_parity` 通过，覆盖首帧、屏蔽恢复、重复输入、发布前失败及重新activation；`rl_sim_LW.cpp`、`rl_real_LW.cpp` 隔离编译通过，未启动实机节点。32帧 Wheel 实际模型合成状态 A/B 输入、原始动作、控制目标及旧相位记账逐字节相同，含重复调用和2次activation。
- 闭环：seed42，两条件各5例20秒，10/10完成；10000策略步、100000物理子步、40000 PD更新，无提前终止。关节/历史/previous action契约核验通过。stand 姿态、动作、PD和物理子步哈希相同；输入仅被屏蔽相位的正负零符号位不同。
- 表现：直行停车17–20秒净位移0.26341→0.20404m；前进转向停车0.000125→0.039854m，存在退化；不能宣称普遍改善。5–15秒纯转向仍明显不足。
- 诊断坐标修正：普通机身角速度为R.T×世界角速度，与freejoint旋转qvel一致；COM与原点速度分别保存。两条件统一，不改变传感器控制输入。
- 证据：上述目录下 report.md、input_validation.json、additional_audit.json、tests/ 与10例完整遥测/视频；联合reset和其他控制参数未处理，旧证据保持不变。

## 本次完成问题

<a id="lw-072"></a>

### [LW-072] 调整 GetDown 姿态保护范围及姿态越限阻尼动作

**状态**： resolved
**批准日期**：2026-09-10

- 问题与证据：用户在 Sim2Sim 的 `RLFSMStateGetDown` 中记录到
  `roll=-24.9037 deg, pitch=75.4931 deg, threshold=75 deg`，随后执行
  `hard-disable-and-shutdown` 并结束全部循环。当前 GetDown 被列入姿态保护，
  `AttitudeLimitExceeded` 属于 S4，且越限会提前结束当前控制周期。
- 批准范围：GetDown 移出角度保护；腿式/轮式行走及两种形态转换保留 75°
  阈值；姿态越限改为 S2，当周期转入 Passive 并发送 `Kp=0`、`Kd=5`、
  目标速度与前馈力矩为零的命令，保持程序运行。沿用 S2 锁存，恢复角度或
  按起身键不恢复运动，须重启。实机及 Sim2Sim 共用该逻辑。
- 保护边界：GetUp 和 Passive 继续免于角度限制；无效反馈、非法最终命令
  等原有终止保护保持有效。
- 验收标准：覆盖 GetDown 大角度、控制前越限、切入受保护状态后越限、
  正负横滚/俯仰、同周期阻尼交付、无退出请求、锁存及无效反馈保护。
- 历史关联：[LW-005](archive/LW_REAL_DEPLOYMENT_RESOLVED_001_066.md#lw-005)、
  [LW-016](archive/LW_REAL_DEPLOYMENT_RESOLVED_001_066.md#lw-016)。本项更新当前行为，
  不改写历史审批与验证记录。

#### 解决记录

- 解决日期：2026-09-10；本条记录与实现一并提交，提交标题为
  `调整 LW 姿态保护范围并将越限改为阻尼`。
- 实施：`lw_control_safety.hpp` 移除 GetDown 角度保护；
  `lw_safety_policy.hpp` 将姿态越限映射到 S2 Passive 阻尼；
  `lw_runtime_core.hpp` 在控制前及状态切换后发现越限时，均在当前周期
  消费 S2 锁存、切换 Passive 并交付阻尼命令。最终命令继续经过校验，
  无效反馈或无效最终命令仍触发终止保护。
- 修改文件：上述三份共享安全头文件；`test_lw_control_safety.cpp`、
  `test_lw_safety_policy.cpp`、`test_lw_runtime_parity.cpp`、
  `test_lw_mujoco_control_adapter.cpp`；
  `docs/LW_BUILD_DEPLOYMENT_CN.md` 及本记录。
- 构建：`cmake --build build/rl_sar -j2` 通过，包含 `rl_real_LW` 和
  `rl_sim_LW`。最终构建日志未出现编译警告或错误；
  `install/rl_sar/lib/rl_sar/rl_sim_LW` 仍链接到此次更新的构建产物。
- 验证：`ctest --test-dir build/rl_sar --output-on-failure` 为 **53/54**。
  姿态保护、安全决策、共享运行时及 MuJoCo 适配器测试全部通过；新增覆盖
  四个受保护状态的正负横滚/俯仰、控制前与切换后当周期阻尼、角度恢复及
  起身输入不能解除锁存、GetDown 大角度、NaN 反馈与阻尼中的 NaN 最终命令。
  MuJoCo 无图形界面测试确认姿态 S2 保持阻尼输出且不请求仿真退出。
- 验证限制：唯一失败项 `lw_configuration_validation` 报告
  `LW/robot_lab/leg_loco output differs at index 0`，源于用户已有未提交
  模型与固定输出基线不一致。将 HEAD 中策略资源提取至独立临时目录，
  仅在临时编译该测试时调整 `POLICY_DIR`，测试通过；未替换工作区模型或
  修改输出基线。最终完整回归仍如实记为 53/54。
- 证据：最终构建日志 `/tmp/lw072-final-build.log`、完整回归日志
  `/tmp/lw072-final-ctest.log`、已提交模型独立核对日志
  `/tmp/lw072-committed-model-o_0_x4uq/test.log`；`git diff --check` 通过。
- 边界：未启动交互式仿真、实机节点或串口设备，未进行实机动作验收。
  用户的 `policy/LW/robot_lab/leg_loco/policy.onnx`、未跟踪的根目录
  `library/` 和 `.agents/skills/inspect-context-compactions/` 均保留。
- 后续事项：本项无额外代码待办；S2 锁存按审批保留，需重启才能恢复运动。

## 已完成问题索引

详细问题说明、审批与验收证据按批次归档：

- [LW-001～LW-066](archive/LW_REAL_DEPLOYMENT_RESOLVED_001_066.md)
- [LW-067～LW-071](archive/LW_REAL_DEPLOYMENT_RESOLVED_067_071.md)

“结项记录提交”取自 Git 中将该问题状态写为 resolved 的提交，不保证是唯一实施提交；
分阶段实施与后续变更仍可通过 Git 历史追溯。

| 编号 | 问题 | 状态 | 结项记录提交 |
|---|---|---|---|
| [LW-001](archive/LW_REAL_DEPLOYMENT_RESOLVED_001_066.md#lw-001) | 循环故障安全退出与异常边界 | resolved | `11e57ccc` |
| [LW-002](archive/LW_REAL_DEPLOYMENT_RESOLVED_001_066.md#lw-002) | 传感器及通信就绪与数据时效门控 | resolved | `917aa770` |
| [LW-003](archive/LW_REAL_DEPLOYMENT_RESOLVED_001_066.md#lw-003) | 串口解析与发送的稳健性 | resolved | `8ac78e80` |
| [LW-004](archive/LW_REAL_DEPLOYMENT_RESOLVED_001_066.md#lw-004) | 有限状态机转换的正确性 | resolved | `15647aec` |
| [LW-005](archive/LW_REAL_DEPLOYMENT_RESOLVED_001_066.md#lw-005) | 命令有限值校验与主动保护 | resolved | `a2b79e74` |
| [LW-006](archive/LW_REAL_DEPLOYMENT_RESOLVED_001_066.md#lw-006) | 手柄断联与输入校验 | resolved | `a96dcf49` |
| [LW-007](archive/LW_REAL_DEPLOYMENT_RESOLVED_001_066.md#lw-007) | 跨线程状态一致性 | resolved | `a665d4ec` |
| [LW-008](archive/LW_REAL_DEPLOYMENT_RESOLVED_001_066.md#lw-008) | 策略输出的原子传递 | resolved | `633ca283` |
| [LW-009](archive/LW_REAL_DEPLOYMENT_RESOLVED_001_066.md#lw-009) | 轮转腿动作参考的更新频率 | resolved | `923182db` |
| [LW-010](archive/LW_REAL_DEPLOYMENT_RESOLVED_001_066.md#lw-010) | 可复现的部署产物 | resolved | `8272253f` |
| [LW-011](archive/LW_REAL_DEPLOYMENT_RESOLVED_001_066.md#lw-011) | 确定性的控制循环时序 | resolved | `9978230c` |
| [LW-012](archive/LW_REAL_DEPLOYMENT_RESOLVED_001_066.md#lw-012) | 动作加载器的稳健性与时间约定 | resolved | `190456dd` |
| [LW-013](archive/LW_REAL_DEPLOYMENT_RESOLVED_001_066.md#lw-013) | 配置与维度校验 | resolved | `6853b6cb` |
| [LW-014](archive/LW_REAL_DEPLOYMENT_RESOLVED_001_066.md#lw-014) | 调试与绘图发布隔离 | resolved | `6761b82b` |
| [LW-015](archive/LW_REAL_DEPLOYMENT_RESOLVED_001_066.md#lw-015) | 仅维护 LW 的仓库范围与未来机器人扩展边界 | resolved | `241f7d69` |
| [LW-016](archive/LW_REAL_DEPLOYMENT_RESOLVED_001_066.md#lw-016) | 安全动作适度性与恢复机制审查 | resolved | `6853b6cb` |
| [LW-017](archive/LW_REAL_DEPLOYMENT_RESOLVED_001_066.md#lw-017) | 可复现的 IMU 与串口运行环境部署 | resolved | `a65a5dcd` |
| [LW-018](archive/LW_REAL_DEPLOYMENT_RESOLVED_001_066.md#lw-018) | 统一构建入口与 Jetson 检测 | resolved | `a65a5dcd` |
| [LW-019](archive/LW_REAL_DEPLOYMENT_RESOLVED_001_066.md#lw-019) | 实机终端键盘恢复通道 | resolved | `379a5165` |
| [LW-020](archive/LW_REAL_DEPLOYMENT_RESOLVED_001_066.md#lw-020) | 架构匹配且仅使用 ONNX 的 Jetson 生产推理 | resolved | `cc07efc4` |
| [LW-021](archive/LW_REAL_DEPLOYMENT_RESOLVED_001_066.md#lw-021) | Sim2Sim 与实机运行行为一致性 | resolved | `25ba1927` |
| [LW-022](archive/LW_REAL_DEPLOYMENT_RESOLVED_001_066.md#lw-022) | 机器人悬吊状态下的实机性能分析与候选配置 | resolved | `57f184e6` |
| [LW-023](archive/LW_REAL_DEPLOYMENT_RESOLVED_001_066.md#lw-023) | 启动前禁能的安全边界 | resolved | `542dcc9b` |
| [LW-024](archive/LW_REAL_DEPLOYMENT_RESOLVED_001_066.md#lw-024) | 可等待线程结束且有界的 Sim2Sim 物理线程生命周期 | resolved | `efd339cd` |
| [LW-025](archive/LW_REAL_DEPLOYMENT_RESOLVED_001_066.md#lw-025) | 持久且可组合的键盘速度输入 | resolved | `1b29744f` |
| [LW-026](archive/LW_REAL_DEPLOYMENT_RESOLVED_001_066.md#lw-026) | 配置方案的来源追溯与可比性 | resolved | `5425b679` |
| [LW-027](archive/LW_REAL_DEPLOYMENT_RESOLVED_001_066.md#lw-027) | 可复现的 ONNX Runtime 部署依赖 | resolved | `83890c51` |
| [LW-028](archive/LW_REAL_DEPLOYMENT_RESOLVED_001_066.md#lw-028) | 信号安全的 Sim2Sim 退出请求 | resolved | `3a73aad7` |
| [LW-029](archive/LW_REAL_DEPLOYMENT_RESOLVED_001_066.md#lw-029) | Sim2Sim 执行器模型与策略根目录的一致性 | resolved | `2b325977` |
| [LW-030](archive/LW_REAL_DEPLOYMENT_RESOLVED_001_066.md#lw-030) | 命令受抑制时步态观测的一致性 | resolved | `095f13be` |
| [LW-031](archive/LW_REAL_DEPLOYMENT_RESOLVED_001_066.md#lw-031) | 受维护 LW 构建的零警告基线 | resolved | `769b6e89` |
| [LW-032](archive/LW_REAL_DEPLOYMENT_RESOLVED_001_066.md#lw-032) | IMU 与 AHRS 的端到端有效性及时效性 | resolved | `0fe6d291` |
| [LW-033](archive/LW_REAL_DEPLOYMENT_RESOLVED_001_066.md#lw-033) | 策略输入的来源追溯与时效性 | resolved | `71371e0d` |
| [LW-034](archive/LW_REAL_DEPLOYMENT_RESOLVED_001_066.md#lw-034) | 可信推理运行库的下载完整性 | resolved | `1f36ba63` |
| [LW-035](archive/LW_REAL_DEPLOYMENT_RESOLVED_001_066.md#lw-035) | 生产启动文件的完整性 | resolved | `f80b916d` |
| [LW-036](archive/LW_REAL_DEPLOYMENT_RESOLVED_001_066.md#lw-036) | 运行时执行器网络输出校验 | resolved | `381c3949` |
| [LW-037](archive/LW_REAL_DEPLOYMENT_RESOLVED_001_066.md#lw-037) | 完整的 Sim2Sim SIGTERM 与 ROS 退出流程 | resolved | `d0e1bad2` |
| [LW-038](archive/LW_REAL_DEPLOYMENT_RESOLVED_001_066.md#lw-038) | 内存分配有界的实机控制周期 | resolved | `6c2db924` |
| [LW-039](archive/LW_REAL_DEPLOYMENT_RESOLVED_001_066.md#lw-039) | 执行器模型路径限制在策略根目录内 | resolved | `eb64b3d8` |
| [LW-040](archive/LW_REAL_DEPLOYMENT_RESOLVED_001_066.md#lw-040) | 安全的 RL 多态析构约定 | resolved | `267fb263` |
| [LW-041](archive/LW_REAL_DEPLOYMENT_RESOLVED_001_066.md#lw-041) | 按需启用 Sim2Sim 绘图发布 | resolved | `476e112a` |
| [LW-042](archive/LW_REAL_DEPLOYMENT_RESOLVED_001_066.md#lw-042) | 非阻塞且保证源数据时效的实机调试遥测 | resolved | `205cc70d` |
| [LW-043](archive/LW_REAL_DEPLOYMENT_RESOLVED_001_066.md#lw-043) | 移除 Sim2Sim 执行器模型运行路径并保留离线训练 | resolved | `37188881` |
| [LW-044](archive/LW_REAL_DEPLOYMENT_RESOLVED_001_066.md#lw-044) | 完整接收 FDILink 帧并显式报告失败 | resolved | `cc54c5f4` |
| [LW-045](archive/LW_REAL_DEPLOYMENT_RESOLVED_001_066.md#lw-045) | 显式且跨架构安全的 FDILink 载荷解码 | resolved | `559ac3eb` |
| [LW-046](archive/LW_REAL_DEPLOYMENT_RESOLVED_001_066.md#lw-046) | 发布前拒绝语义无效的 FDILink 样本 | resolved | `a9f981dd` |
| [LW-047](archive/LW_REAL_DEPLOYMENT_RESOLVED_001_066.md#lw-047) | 分类处理 FDILink 8 位序号异常，避免虚构丢帧 | resolved | `1fda05ad` |
| [LW-048](archive/LW_REAL_DEPLOYMENT_RESOLVED_001_066.md#lw-048) | 将已安装的 ONNX Runtime 文件绑定到获准的归档包 | resolved | `6ef1b97e` |
| [LW-049](archive/LW_REAL_DEPLOYMENT_RESOLVED_001_066.md#lw-049) | 使保留的 Gazebo 控制器执行有界、等待 URDF 就绪且内存分配稳定 | resolved | `ef0af1d3` |
| [LW-050](archive/LW_REAL_DEPLOYMENT_RESOLVED_001_066.md#lw-050) | 统一 ONNX 动态批次约定与缓存张量资源 | resolved | `0ce245a3` |
| [LW-051](archive/LW_REAL_DEPLOYMENT_RESOLVED_001_066.md#lw-051) | 固定 MuJoCo 下载摘要并实现原子安装 | resolved | `af0b0c0c` |
| [LW-052](archive/LW_REAL_DEPLOYMENT_RESOLVED_001_066.md#lw-052) | 加固通用 rl_sim 的手柄边界检查与临时文件生命周期 | resolved | `11183fc1` |
| [LW-053](archive/LW_REAL_DEPLOYMENT_RESOLVED_001_066.md#lw-053) | 缓存 rl_sim_LW 调试消息布局 | resolved | `d85f6808` |
| [LW-054](archive/LW_REAL_DEPLOYMENT_RESOLVED_001_066.md#lw-054) | 在控制线程启动前预加载形态转换动作资源 | resolved | `79bf0b76` |
| [LW-055](archive/LW_REAL_DEPLOYMENT_RESOLVED_001_066.md#lw-055) | 移除实机控制截止时间路径上的阻塞同步 | resolved | `fa3fe9f3` |
| [LW-056](archive/LW_REAL_DEPLOYMENT_RESOLVED_001_066.md#lw-056) | 使受维护的完整控制周期保持内存分配稳定 | resolved | `151a3e97` |
| [LW-057](archive/LW_REAL_DEPLOYMENT_RESOLVED_001_066.md#lw-057) | 校验 MuJoCo 控制适配器布局并测试实际安全动作 | resolved | `39748434` |
| [LW-058](archive/LW_REAL_DEPLOYMENT_RESOLVED_001_066.md#lw-058) | 将受维护的 C++ 目标与未使用的 Python 运行环境隔离 | resolved | `6b610110` |
| [LW-059](archive/LW_REAL_DEPLOYMENT_RESOLVED_001_066.md#lw-059) | 在所有部分构造失败路径上停止 Sim2Sim 工作线程 | resolved | `bf21601e` |
| [LW-060](archive/LW_REAL_DEPLOYMENT_RESOLVED_001_066.md#lw-060) | 在 ObservationBuffer 中统一使用历史帧计数域 | resolved | `e8669452` |
| [LW-061](archive/LW_REAL_DEPLOYMENT_RESOLVED_001_066.md#lw-061) | 统一动作参考校验与运行时门控的语义 | resolved | `161feb00` |
| [LW-062](archive/LW_REAL_DEPLOYMENT_RESOLVED_001_066.md#lw-062) | 在推理热路径中复用连续缓冲区 | resolved | `e620d0b8` |
| [LW-063](archive/LW_REAL_DEPLOYMENT_RESOLVED_001_066.md#lw-063) | 共享 ONNX Runtime 环境且保持模型隔离 | resolved | `de1f55fc` |
| [LW-064](archive/LW_REAL_DEPLOYMENT_RESOLVED_001_066.md#lw-064) | 移除或正确实现具有误导性的 FDILink CRC32 接口 | resolved | `fea0ff9a` |
| [LW-065](archive/LW_REAL_DEPLOYMENT_RESOLVED_001_066.md#lw-065) | 恢复 FDILink 代码规范检查与软件包元数据的干净基线 | resolved | `e3596f48` |
| [LW-066](archive/LW_REAL_DEPLOYMENT_RESOLVED_001_066.md#lw-066) | 确保依赖查找顺序正确并将构建设置限定到目标 | resolved | `83e22d55` |

| <a id="lw-067"></a>[LW-067](archive/LW_REAL_DEPLOYMENT_RESOLVED_067_071.md#lw-067) | 修复构建目标检查对合法编译配置的误报 | resolved | `39a3d22` |
| <a id="lw-068"></a>[LW-068](archive/LW_REAL_DEPLOYMENT_RESOLVED_067_071.md#lw-068) | 将只读动作裁剪配置校验集中到所属边界 | resolved | `76dc6bb` |
| <a id="lw-069"></a>[LW-069](archive/LW_REAL_DEPLOYMENT_RESOLVED_067_071.md#lw-069) | 合并基础配置及启动超时参数的重复校验 | resolved | `eeb1d16` |
| <a id="lw-070"></a>[LW-070](archive/LW_REAL_DEPLOYMENT_RESOLVED_067_071.md#lw-070) | 将 ONNX 私有缓存不变量检查集中到加载阶段 | resolved | `8f32b25` |
| <a id="lw-071"></a>[LW-071](archive/LW_REAL_DEPLOYMENT_RESOLVED_067_071.md#lw-071) | 减少源码写法绑定和重复生命周期测试断言 | resolved | `d03c6bd` |

## 解决记录模板

完成所选问题后填写以下信息，明确实际验证范围：

```markdown
**状态**： resolved

### 解决记录
- 解决时间： YYYY-MM-DDTHH:MM:SS+08:00
- 提交： <提交后填写真实哈希；尚未提交时如实注明>
- 批准范围： <用户批准的范围>
- 修改文件： <文件路径>
- 验证： <命令、结果及验证限制>
- 后续事项： <问题编号；不修改其他问题状态>
```
