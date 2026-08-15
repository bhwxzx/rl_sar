# Learnings

## [LRN-20260814-001] correction

**Logged**: 2026-08-14T20:17:53+08:00
**Priority**: high
**Status**: promoted
**Area**: docs

### Summary
只有代码修改方案必须在末尾明确说明方案解决的问题和具体优势。

### Details
用户指出，仅列实施步骤、文件范围和验证方法不足以支持方案审批。方案结尾还应
把技术动作映射回实际问题，说明消除了哪些故障路径，以及所选做法在安全性、
可维护性、可测试性或变更边界方面的优势，便于用户判断方案是否值得批准。该
要求只适用于请求审批的代码修改方案；Git 提交结果、任务完成汇报、状态更新等
不是代码修改方案，不应机械追加该段落。

### Suggested Action
以后提出任何代码修改方案时，以独立的“解决的问题与方案优势”段落收尾；内容
必须具体对应当前问题，不能只使用“更安全”“更可靠”等泛化表述。非方案类
消息按其实际目的简洁汇报，不追加这一固定段落。

### Metadata
- Source: user_feedback
- Related Files: AGENTS.md
- Tags: proposal, approval, communication, rationale, benefits
- Pattern-Key: workflow.code_proposal_explains_solution_and_benefits
- Recurrence-Count: 2
- First-Seen: 2026-08-14
- Last-Seen: 2026-08-14

**Promoted**: AGENTS.md

---

## [LRN-20260815-005] best_practice

**Logged**: 2026-08-15T17:45:00+08:00
**Priority**: medium
**Status**: resolved
**Area**: infra

### Summary
ROS 2-only 工作区应直接跟踪普通 `package.xml`，构建清理不得通过切换清单链接来改写源码状态。

### Details
同时保留 `package.ros1.xml`、`package.ros2.xml` 并让 `package.xml` 成为动态链接，
会把 ROS 版本选择引入源码树状态：全量清理必须特殊保护或重建链接，包级构建也
可能遗留与当前环境不匹配的清单。项目明确只支持 ROS 2 后，应把 ROS 2 清单内容
固化为普通 `package.xml`，删除版本切换逻辑，并让全量与包级清理只处理生成物。
这样 `colcon` 始终读取稳定清单，`git status` 不会因构建或清理发生源码类型变化。

### Suggested Action
ROS 2-only 项目统一使用 Ament CMake 和普通 `package.xml`；测试同时断言清单不是
符号链接、旧版清单不存在、全量及包级清理均保留源码清单，并从空构建树执行一次
全量 isolated Colcon 构建验证迁移结果。

### Metadata
- Source: task_outcome
- Related Files: build.sh, src/rl_sar/package.xml, src/robot_joint_controller/package.xml, src/robot_msgs/package.xml
- Tags: ROS2, colcon, package-manifest, clean, source-safety
- Pattern-Key: build.ros2_only_standard_manifest
- Recurrence-Count: 1
- First-Seen: 2026-08-15
- Last-Seen: 2026-08-15

### Resolution
- **Resolved**: 2026-08-15T17:45:00+08:00
- **Commit/PR**: 未提交（按用户要求保留工作区修改）
- **Notes**: 三个双清单包已改为普通 ROS 2 `package.xml`，清理逻辑不再触碰源码清单；工作流测试、空树全量构建、CTest 和严格构建均通过。

---

## [LRN-20260815-001] correction

**Logged**: 2026-08-15T13:30:22+08:00
**Priority**: medium
**Status**: resolved
**Area**: docs

### Summary
新增运行参数时必须同步检查完整说明、README 和面向操作者的快速开始文档。

### Details
LW-041 已在中英文 README 和完整编译部署说明中记录 `--enable-plot`
与 `--plot-rate-hz`，但遗漏了实际 Sim2Sim 交付流程直接引用的
`docs/LW_QUICK_START_CN.md`。只核对主 README 和详细文档不足以保证
操作者能在标准成功路径中发现新参数。

### Suggested Action
每次新增或修改用户可见的运行时参数，在完成前使用参数名全局检索，
并逐项核对 README、完整使用说明、快速开始及其它实际操作入口的命令、
默认值、取值范围和组合约束。

### Metadata
- Source: user_feedback
- Related Files: docs/LW_QUICK_START_CN.md, docs/LW_BUILD_DEPLOYMENT_CN.md, README.md, README_CN.md
- Tags: LW, CLI, documentation, quick-start, synchronization
- Pattern-Key: docs.runtime_parameter_operator_entrypoints
- Recurrence-Count: 1
- First-Seen: 2026-08-15
- Last-Seen: 2026-08-15

### Resolution
- **Resolved**: 2026-08-15T13:30:22+08:00
- **Commit/PR**: 本提交
- **Notes**: 已在 LW 快速开始的 Sim2Sim 标准流程中补充默认关闭、
  `--enable-plot`、`--plot-rate-hz`、1–200 Hz 范围及错误参数拒绝语义。

---

## [LRN-20260815-002] correction

**Logged**: 2026-08-15T14:26:04+08:00
**Priority**: low
**Status**: resolved
**Area**: docs

### Summary
当前项目只面向中文使用者，根 README 和项目操作文档无需维持英文副本。

### Details
在评估是否用中文替换根 `README.md` 时，曾默认建议保留英文版以服务国际使用
者。用户明确补充：当前项目只针对中文使用者，且 `docs/` 中的项目操作文档均
为中文。因此正确的仓库约定是以中文 `README.md` 作为唯一根入口；不应仅为
假设的英文受众继续维护重复的英文 README。迁移时仍需同步现有
`README_CN.md` 引用、语言切换链接和构建测试参数，避免悬空路径或重复内容。

### Suggested Action
将现有中文 README 内容迁移到 `README.md`，删除英文版和不再需要的
`README_CN.md`，同步更新文档及测试中对 `README_CN.md` 的引用，并验证仓库
不再存在旧文件名或失效的语言切换链接。

### Metadata
- Source: user_feedback
- Related Files: README.md, README_CN.md, docs/LW_BUILD_DEPLOYMENT_CN.md, src/rl_sar/CMakeLists.txt
- Tags: documentation, Chinese, audience, README, localization
- Pattern-Key: docs.project_audience_chinese_only
- Recurrence-Count: 1
- First-Seen: 2026-08-15
- Last-Seen: 2026-08-15

### Resolution
- **Resolved**: 2026-08-15T14:26:04+08:00
- **Commit/PR**: 本提交
- **Notes**: 中文内容已迁移为唯一根 `README.md`，英文根 README 和重复的
  `README_CN.md` 已移除；部署文档及构建测试引用已同步，旧文件名和语言切换
  链接全局检查无残留，相关构建与测试通过。

---

## [LRN-20260815-003] knowledge_gap

**Logged**: 2026-08-15T14:38:03+08:00
**Priority**: high
**Status**: resolved
**Area**: infra

### Summary
`build.sh --cmake` 当前不是可用的 LW 硬件构建，且不会可靠关闭 MuJoCo。

### Details
帮助和部署文档把 `-m`/`--cmake` 描述为“不含 MuJoCo 的 CMake 硬件构建”，
但脚本实际只用 `-DUSE_CMAKE=ON` 配置复用的 `cmake_build`。当前
`rl_real_LW`、ROS 调试 publisher、ROS 依赖和安装规则均位于
`NOT USE_CMAKE` 分支，因此该模式不会生成可启动的 LW 真机程序；它主要构建
共享库及不依赖 ROS 的测试目标。脚本也未传入 `-DUSE_MUJOCO=OFF`，所以同一
目录曾由 `--mujoco` 配置后，再运行 `--cmake` 会继续沿用缓存中的
`USE_MUJOCO=ON`。现有 `cmake_build/CMakeCache.txt` 即为
`USE_CMAKE=ON`、`USE_MUJOCO=ON`，目标清单含 `rl_sim_mujoco` 而不含
`rl_real_LW`；目录中的旧真机二进制只是历史残留，不能证明当前模式有效。

### Suggested Action
先明确是否仍需要独立的非 ROS CMake 诊断入口。若不需要，删除
`--cmake` 及误导性文档，仅保留 ROS 开发构建、MuJoCo 构建和正式部署构建；
若需要，则将其明确命名为非 ROS 核心/测试构建、每次显式设置
`USE_MUJOCO=OFF`，并增加目标集合与缓存切换测试。不要继续称其为硬件部署
构建，除非它真正生成并验证当前 `rl_real_LW` 交付物。

### Metadata
- Source: conversation
- Related Files: build.sh, src/rl_sar/CMakeLists.txt, docs/LW_BUILD_DEPLOYMENT_CN.md, docs/LW_QUICK_START_CN.md
- Tags: build, CMake, hardware, MuJoCo, cache, documentation
- Pattern-Key: build.cmake_mode_matches_claimed_targets
- Recurrence-Count: 1
- First-Seen: 2026-08-15
- Last-Seen: 2026-08-15

### Resolution
- **Resolved**: 2026-08-15T15:49:28+08:00
- **Commit/PR**: 本提交
- **Notes**: 已删除 `build.sh` 的 `-m`/`--cmake` 参数、独立执行分支、帮助
  文本和误导性无 ROS 提示，并同步两份操作文档。`--mujoco` 及其所需的
  `USE_CMAKE=ON`/`USE_MUJOCO=ON` 保持不变；回归测试验证删除的参数被拒绝、
  MuJoCo 入口仍保留。

---

## [LRN-20260815-004] best_practice

**Logged**: 2026-08-15T16:11:48+08:00
**Priority**: high
**Status**: resolved
**Area**: infra

### Summary
项目应只保留一个受支持的 LW Sim2Sim 构建入口，并让构建、测试和文档共同固定该边界。

### Details
历史 `build.sh --mujoco` 通过独立的 `USE_CMAKE`/`USE_MUJOCO` CMake 分支构建
`rl_sim_mujoco`，与标准 ROS 工作空间中的 `rl_sim_LW` 形成第二套仿真实现。
该入口绕开了当前受维护的共享运行时、生命周期、安全适配和 ROS 集成，容易在主
Sim2Sim 路径继续演进时静默陈旧。项目实际需要的 MuJoCo 依赖准备、vendor 库、
无窗口生命周期测试和标准仿真目标均已由完整工作空间构建覆盖，因此额外入口没有
独立交付价值，反而扩大了代码、文档和验证表面。

### Suggested Action
将不带包名的完整 `./build.sh` 和其中生成的 `rl_sim_LW` 作为唯一受支持的 LW
Sim2Sim 构建/运行路径。修改构建系统时同时用回归测试固定三类约束：历史 CLI、
CMake 选项和独立源文件不存在；标准 `rl_sim_LW` 与 MuJoCo vendor 目标存在；
操作文档只引导完整工作空间验证。保留共享依赖准备和测试资源，不把删除旧入口误
扩展为删除标准 MuJoCo 能力。

### Metadata
- Source: conversation
- Related Files: build.sh, src/rl_sar/CMakeLists.txt, src/rl_sar/test/test_build_workflow.py, src/rl_sar/test/test_lw_sim_lifecycle_integration.py, docs/LW_BUILD_DEPLOYMENT_CN.md, docs/LW_QUICK_START_CN.md
- Tags: LW, Sim2Sim, MuJoCo, build-entry, lifecycle, maintenance
- See Also: LRN-20260815-003
- Pattern-Key: build.single_supported_sim2sim_entry
- Recurrence-Count: 1
- First-Seen: 2026-08-15
- Last-Seen: 2026-08-15

### Resolution
- **Resolved**: 2026-08-15T16:11:48+08:00
- **Commit/PR**: 未提交（按用户要求）
- **Notes**: 已删除历史 `--mujoco`、`USE_CMAKE`/`USE_MUJOCO`、
  `rl_sim_mujoco` 源文件和目标；标准 `rl_sim_LW`、MuJoCo 依赖、vendor 库及
  自动测试保留。现有构建与全新严格构建的 45/45 CTest 均通过。

---

## [LRN-20260811-003] correction

**Logged**: 2026-08-11T17:57:11+08:00
**Priority**: high
**Status**: pending
**Area**: tests

### Summary
Sim2Sim 安全一致性不能只记录真机将采取的动作，还应实际执行对应的仿真阻尼或
主动执行器归零。

### Details
LW-021 初版方案只要求仿真 shadow safety sink 记录 Passive 阻尼、硬失能和关闭
决定。用户指出，仿真本身可以进入阻尼模式或把动作输出归零，因此只记录不足以
验证故障后的机器人动力学响应。不同安全等级必须保持语义区分：S2 应复用批准的
Passive 命令（当前关节位置、`dq=0`、`tau=0`、`Kp=0`、`Kd=5`），让 MuJoCo
产生速度阻尼；S3/S4 才锁存停止接受新命令并将所有主动执行器输出置零。把阻尼
和零主动输出视为同一动作会掩盖真实的安全行为差异。

### Suggested Action
为仿真提供可执行且可观测的安全适配层：S1 抑制速度但保留恢复输入，S2 实际
应用共享 Passive 阻尼命令，S3/S4 锁存主动输出为零，并在 S4 记录关闭请求供
测试检查。确定性 parity 测试同时断言安全决定、锁存状态、最终 `RobotCommand`
和 MuJoCo 主动执行器输出。

### Metadata
- Source: user_feedback
- Related Files: .learnings/LW_REAL_DEPLOYMENT_ISSUES.md, src/rl_sar/src/rl_sim_LW.cpp, src/rl_sar/library/core/safety/lw_control_safety.hpp
- Tags: LW, Sim2Sim, safety, passive-damping, hard-disable, parity
- Pattern-Key: test.sim_safety_actions_are_executed
- Recurrence-Count: 1
- First-Seen: 2026-08-11
- Last-Seen: 2026-08-11

---

## [LRN-20260813-001] correction

**Logged**: 2026-08-13T12:28:00+08:00
**Priority**: high
**Status**: resolved
**Area**: docs

### Summary
安全关键操作文档必须直接说明测算期间电机是否会被软件使能，并区分命令保证与硬件状态证明。

### Details
LW 吊装硬件观察说明原本描述了只发送 `motors_disable=true`、不允许 Passive 或
shadow policy 命令进入串口，但没有用直接结论回答“测算过程中电机是否会使能”。
用户指出这一歧义。实现能够证明测算程序不会发送使能或普通控制命令：初始化、
5 ms 保活和退出序列只使用独立失能包，策略命令在 `SetCommand()` 中丢弃。由于
反馈协议没有“失能已执行”回执，这个软件命令保证仍不能证明 STM32 已执行失能或
电机物理状态已经失能。

### Suggested Action
安全步骤应先用醒目结论明确“程序是否会发送使能命令”，随后单独说明无硬件回执
造成的证明边界，并继续要求吊装、隔离和物理急停，避免用委婉的实现描述代替操作
人员真正需要回答的安全问题。

### Metadata
- Source: user_feedback
- Related Files: docs/LW_BUILD_DEPLOYMENT_CN.md, src/rl_sar/src/lw_config_profiler.cpp
- Tags: LW, hardware-observe, motor-disable, safety, documentation
- Pattern-Key: docs.safety_command_guarantee_vs_physical_state
- Recurrence-Count: 1
- First-Seen: 2026-08-13
- Last-Seen: 2026-08-13

### Resolution
- **Resolved**: 2026-08-13T12:28:00+08:00
- **Commit/PR**: 本提交
- **Notes**: 吊装硬件观察步骤已明确测算程序全程不会发送使能或普通控制命令，
  并将该软件保证与“STM32 已执行失能、电机物理失能”的不可证明状态分开表述。

---

## [LRN-20260729-001] best_practice

**Logged**: 2026-07-29T13:00:09+08:00
**Priority**: high
**Status**: promoted
**Area**: config
**Promoted**: AGENTS.md

### Summary
修改任何代码前，必须先获得用户的明确审批。

### Details
用户要求将“修改代码前必须先经过审批”记录为项目级工作规范。只读检查、分析和提出修改方案不属于代码修改；在用户明确批准之前，不得编辑源码、测试、脚本或其他代码文件。

### Suggested Action
每次需要修改代码时，先说明拟修改的范围和内容并请求审批；收到用户明确同意后再开始写入。

### Metadata
- Source: user_feedback
- Related Files: AGENTS.md
- Tags: approval, code-change, workflow, user-control
- Pattern-Key: workflow.code_change_approval
- Recurrence-Count: 1
- First-Seen: 2026-07-29
- Last-Seen: 2026-07-29

---

## [LRN-20260811-002] correction

**Logged**: 2026-08-11T17:02:00+08:00
**Priority**: medium
**Status**: resolved
**Area**: infra

### Summary
精简 Jetson 生产推理后端时，必须保留新开发机首次构建自动准备 PyTorch C++
运行时的能力。

### Details
LW-020 将 Jetson 真机和正式部署收敛到实际使用的 ONNX 后端。用户补充确认，
这不能改变新开发机执行 `build.sh` 时自动安装相关依赖的既有契约。当前项目的
构建依赖是 LibTorch（C++ 版 PyTorch），用于非 Jetson Sim2Sim 可选的
`--use_actuator_net`；Python `torch` 只由训练脚本使用，并不是编译依赖。
因此后端精简必须按用途和平台生效，不能全局删除 LibTorch 引导。

### Suggested Action
Jetson 真机与正式部署保持 ONNX-only；非 Jetson 开发构建继续默认下载 LibTorch
和 ONNX Runtime，并用测试与文档固定这条分支契约。不要让生产部署的最小依赖
集合反向定义完整开发环境。

### Metadata
- Source: user_feedback
- Related Files: build.sh, scripts/download_inference_runtime.sh, README_CN.md
- Tags: build, bootstrap, LibTorch, PyTorch, Jetson, production
- See Also: LRN-20260811-001
- Pattern-Key: build.production_reduction_preserves_development_bootstrap
- Recurrence-Count: 1
- First-Seen: 2026-08-11
- Last-Seen: 2026-08-11

### Resolution

- **Resolved**: 2026-08-11T17:09:09+08:00
- **Commit/PR**: 本提交
- **Notes**: LW-020 仅在 Jetson 真机和正式部署中移除 LibTorch；普通
  x86_64 开发机首次运行 `build.sh` 仍自动准备 LibTorch 与 ONNX Runtime，
  并由构建流程测试固定该平台分支契约。

---

## [LRN-20260729-002] best_practice

**Logged**: 2026-07-29T14:04:42+08:00
**Priority**: high
**Status**: promoted
**Area**: config
**Promoted**: AGENTS.md

### Summary
LW 实机部署问题必须按优先级逐项审批、逐项修改和逐项验证。

### Details
实机部署审查发现的问题相互关联且包含安全关键路径。用户要求后续由其指定修复项，每次只修改一个问题，不能一次性合并全部修复。完整问题顺序、状态和验收标准记录在 `.learnings/LW_REAL_DEPLOYMENT_ISSUES.md`。

### Suggested Action
收到后续修改请求时，先确认用户指定的 `LW-XXX`，说明该项的修改范围并获得明确审批；只完成该项，验证后更新其状态和证据。

### Metadata
- Source: user_feedback
- Related Files: AGENTS.md, .learnings/LW_REAL_DEPLOYMENT_ISSUES.md
- Tags: real-robot, deployment, staged-remediation, approval, safety
- Pattern-Key: workflow.lw_staged_remediation
- Recurrence-Count: 1
- First-Seen: 2026-07-29
- Last-Seen: 2026-07-29

---

## [LRN-20260729-003] knowledge_gap

**Logged**: 2026-07-29T21:20:56+08:00
**Priority**: high
**Status**: resolved
**Area**: backend

### Summary
FSM 支持键盘状态不等于真机程序提供了可用的终端键盘控制通道。

### Details
LW FSM 接受 `current_keyboard`，核心库也实现了 `KeyboardInterface()`，但
`rl_real_LW.cpp` 没有启动该接口或键盘线程。因此，手柄断联闭锁后不能假设
操作员仍可通过终端触发 GetDown。与此同时，退出真机程序会发送最终失能
命令；机器人直立时依靠重启恢复存在摔倒风险。

### Suggested Action
审查安全恢复路径时，从输入设备一直追踪到实际入口和线程启动点，并验证
命令能够到达 FSM；在提供独立受控 GetDown 通道之前，将机械支撑作为
手柄断联后关闭或重启程序的前置条件。

### Metadata
- Source: conversation
- Related Files: src/rl_sar/src/rl_real_LW.cpp, src/rl_sar/fsm_robot/fsm_LW.hpp, src/rl_sar/library/core/rl_sdk/rl_sdk.cpp
- Tags: real-robot, joystick, keyboard, getdown, shutdown, recovery
- Pattern-Key: safety.verify_end_to_end_recovery_path
- Recurrence-Count: 1
- First-Seen: 2026-07-29
- Last-Seen: 2026-07-29

### Resolution
- **Resolved**: 2026-08-11T15:31:22+08:00
- **Commit/PR**: 本提交
- **Notes**: LW-019 为真机默认启用由控制线程轮询的 `/dev/tty` 键盘通道；
  手柄断联锁存继续清零速度和 Gamepad，但保留数字键 `9` 的 `GetDown` 请求。
  终端以 RAII 恢复，headless 部署必须显式关闭并承担文档列明的替代恢复措施。

---

## [LRN-20260731-001] best_practice

**Logged**: 2026-07-31T15:29:07+08:00
**Priority**: high
**Status**: promoted
**Area**: config
**Promoted**: AGENTS.md

### Summary
同一会话自动压缩上下文过多时，应形成持久化交接文档并建议在新会话继续。

### Details
用户要求长任务在多次上下文自动压缩后，不再仅依赖压缩摘要维持连续性。
同一会话出现第三次可见的自动压缩时，应先完成不可分割的当前操作，再将
目标、已完成提交、用户决策与审批、约束、验证结果、需保留的未提交文件、
未解决问题和下一步写入 `.learnings/session_handoffs/`。随后向用户说明
连续性风险，建议开启新会话，并提供可直接复制的续接 prompt。

### Suggested Action
只统计会话中明确可见的压缩事件。达到三次时生成带时间和主题的交接文档，
向用户提供文档链接及新会话 prompt；若用户选择继续当前会话，则每次新增
压缩后刷新该文档。

### Metadata
- Source: user_feedback
- Related Files: AGENTS.md, .learnings/session_handoffs/
- Tags: context-compaction, handoff, long-session, continuity, prompt
- Pattern-Key: workflow.context_compaction_handoff
- Recurrence-Count: 1
- First-Seen: 2026-07-31
- Last-Seen: 2026-07-31

---

## [LRN-20260731-002] best_practice

**Logged**: 2026-07-31T16:15:16+08:00
**Priority**: medium
**Status**: resolved
**Area**: config

### Summary
Codex Accounts Manager 报 HTTPS 代理不支持 SOCKS 时，应检查 HTTPS 代理变量是否缺失并回退到了 `ALL_PROXY`。

### Details
虚拟机环境同时存在 `HTTP_PROXY=http://...` 和 `ALL_PROXY=socks://...`，
但没有 `HTTPS_PROXY`。Codex Accounts Manager 为 HTTPS 请求选择代理时回退
到了 `ALL_PROXY`，随后因只接受 `http://` 和 `https://` 代理 URL 而拒绝
启动认证请求。

排查时只输出环境变量名和协议，避免泄露代理凭据。将 `HTTPS_PROXY` 临时
指向已有的 HTTP 代理并清空大小写的 `ALL_PROXY` 后，使用 `curl` 访问
`https://auth.openai.com/` 得到代理隧道 `CONNECT 200`，证明该 HTTP 代理
路径可用；目标站点根路径随后返回 `403` 不代表代理连接失败。

VS Code 扩展读取的是扩展宿主进程环境。在终端中修改变量后，需要彻底重启
VS Code；若扩展运行在 Remote SSH 端，还应终止远端 VS Code Server 并重新
连接。不能只把纯 SOCKS 端口的 URL 从 `socks://` 改写成 `http://`，除非
该监听端口确实支持 HTTP CONNECT 或 mixed 模式。

### Suggested Action
为启动 VS Code/远端扩展宿主的环境设置有效的
`HTTPS_PROXY=http://<host>:<http-or-mixed-port>`，同步设置小写变量，并
取消 `ALL_PROXY`/`all_proxy` 后重启扩展宿主。重启后再次验证 Codex 登录；
确认成功后将本条状态更新为 `resolved`。

### Metadata
- Source: conversation
- Related Files: N/A (VS Code/Codex runtime environment)
- Tags: codex, vscode, proxy, socks, https-proxy, virtual-machine, authentication
- Pattern-Key: config.codex_https_proxy_fallback
- Recurrence-Count: 1
- First-Seen: 2026-07-31
- Last-Seen: 2026-07-31

### Resolution
- **Resolved**: 2026-07-31T16:16:20+08:00
- **Commit/PR**: N/A (runtime environment configuration)
- **Notes**: 用户确认按建议调整代理环境并重启插件后，Codex Accounts Manager 已恢复正常。

---

## [LRN-20260801-001] knowledge_gap

**Logged**: 2026-08-01T12:09:28+08:00
**Priority**: medium
**Status**: promoted_to_skill
**Area**: config
**Skill-Path**: .agents/skills/inspect-context-compactions/

### Summary
上下文压缩阈值必须通过专用 inspector 核验，不能根据模型可见摘要或手工读取 rollout 内容推断。

### Details
精确计数属于 `.agents/skills/inspect-context-compactions/` 的职责。该技能以
只读方式定位当前会话，验证顶层压缩记录、连续窗口编号、镜像事件和会话
身份，并且不会输出消息或替换历史。`self-improvement` 只保留本晋升记录，
不实现 rollout 解析或压缩计数。

### Suggested Action
涉及压缩阈值或会话交接时，运行 inspector 并先检查其 `status`；仅信任
`available` 结果，遇到 `unavailable` 或 `inconsistent` 时不得推断精确次数。

### Metadata
- Source: conversation
- Related Files: AGENTS.md, .agents/skills/inspect-context-compactions/
- Tags: codex, context-compaction, inspector, session-handoff
- See Also: LRN-20260731-001
- Pattern-Key: workflow.context_compaction_counting
- Recurrence-Count: 1
- First-Seen: 2026-08-01
- Last-Seen: 2026-08-01

### Resolution
- **Resolved**: 2026-08-01T12:09:28+08:00
- **Commit/PR**: N/A
- **Notes**: 计数与隐私校验职责已晋升到 `inspect-context-compactions` 技能；项目交接规则改为使用其结构化结果。

---

## [LRN-20260804-001] correction

**Logged**: 2026-08-04T16:18:00+08:00
**Priority**: high
**Status**: resolved
**Area**: docs

### Summary
LW 发布流程必须把 Sim2Sim 行为验证明确放在开发机，部署机只进行部署验收和实机实验。

### Details
先前的编译部署文档只把开发机描述为修改、测试和提交，没有明确要求在开发机
构建并运行 `rl_sim_LW` 完成 Sim2Sim 验证，容易让读者误以为部署机的
`--verify-deployment-only` 可以覆盖仿真验证。正确职责是：开发机完成代码、
模型和配置修改以及 Sim2Sim 行为验证；验证通过的 Git 提交再交给部署机；
部署机从该提交生成正式部署版本，完成清单和动态库检查后才进行实机实验。

### Suggested Action
在 LW 编译部署文档中把开发机 Sim2Sim 设为正式交付前置门槛，给出构建、
启动和检查步骤；明确部署机只读验收只验证发布物完整性，不能代替策略行为验证。

### Metadata
- Source: user_feedback
- Related Files: docs/LW_BUILD_DEPLOYMENT_CN.md, README_CN.md
- Tags: LW, sim2sim, deployment, real-robot, release-gate, documentation
- Pattern-Key: workflow.lw_dev_sim2sim_deploy_real
- Recurrence-Count: 1
- First-Seen: 2026-08-04
- Last-Seen: 2026-08-04

### Resolution
- **Resolved**: 2026-08-04T16:21:00+08:00
- **Commit/PR**: 本次文档提交
- **Notes**: 中文部署文档已增加开发机 `rl_sim_LW` 构建、启动和行为检查步骤，并明确部署机的只读验收不能代替 Sim2Sim，部署机只进行部署验收和实机实验。

---

## [LRN-20260804-002] correction

**Logged**: 2026-08-04T16:47:00+08:00
**Priority**: high
**Status**: resolved
**Area**: config

### Summary
`src/rl_sar_zoo/` 必须纳入 LW-only 清理和版本跟踪范围，不能继续作为外部未跟踪用户目录排除。

### Details
先前的 LW-015 方案把 `src/rl_sar_zoo/` 视为清理范围之外的用户资产，用户
明确纠正为：同样删除其中的非 LW 机器人描述，并将保留的 LW 仿真资源纳入
版本跟踪。当前目录是约 588 MB 的嵌套 Git 仓库，LW 的三个 MJCF 文件已有
本地修改，另有未跟踪 terrain 资源。直接从根仓库执行普通 `git add` 可能只
形成不完整的嵌套仓库引用，因此实施前必须盘点并批准正式集成策略，同时保留
所有当前 LW 修改。

### Suggested Action
在 LW-015 中纳入 zoo 清理、LW 资源保留、来源/许可证和可复现获取要求；
删除前明确选择清理后内置到根仓库或配置为正式固定版本子模块，并逐项核对
当前 LW 修改和新增资源，禁止静默丢弃。

### Metadata
- Source: user_feedback
- Related Files: .learnings/LW_REAL_DEPLOYMENT_ISSUES.md, src/rl_sar_zoo/
- Tags: LW, robot-description, nested-git, tracking, cleanup, sim2sim
- See Also: LRN-20260804-001
- Pattern-Key: workflow.lw_zoo_tracking_scope
- Recurrence-Count: 1
- First-Seen: 2026-08-04
- Last-Seen: 2026-08-04

### Resolution
- **Resolved**: 2026-08-04T16:49:00+08:00
- **Commit/PR**: N/A（随本次范围记录提交）
- **Notes**: LW-015 已将整个 zoo 纳入盘点、非 LW 描述清理和可复现版本跟踪范围，并要求保留当前 LW MJCF/terrain 修改、上游来源及许可证；实际暂未删除或暂存文件。

---

## [LRN-20260804-003] correction

**Logged**: 2026-08-04T17:55:02+08:00
**Priority**: medium
**Status**: resolved
**Area**: docs

### Summary
权威问题表只能有一套按优先级排列的总顺序，状态筛选不能被描述成另一套“剩余事项顺序”。

### Details
在新增 LW-016 后，表格本身已按 P0、P1、P2 排列，并在同一优先级内按实机
风险和依赖排序；但回复和记录措辞同时使用“剩余事项首位”和“总顺序第 11”，
容易让人误解为存在两套独立顺序。正确语义是：`Order` 始终是唯一权威优先级
顺序，`Status` 只表示完成状态；过滤掉 `resolved` 行只是查看下一项工作，不会
产生新的排序规则。

### Suggested Action
维护权威任务表时，明确先按优先级分组、再在同级内按风险和依赖排序；汇报时
直接给出总表位置，并可补充“过滤已解决项后是下一项”，但不得称为另一套顺序。

### Metadata
- Source: user_feedback
- Related Files: .learnings/LW_REAL_DEPLOYMENT_ISSUES.md
- Tags: issue-register, priority, ordering, status, documentation
- Pattern-Key: workflow.authoritative_issue_priority_order
- Recurrence-Count: 1
- First-Seen: 2026-08-04
- Last-Seen: 2026-08-04

### Resolution
- **Resolved**: 2026-08-04T17:55:02+08:00
- **Commit/PR**: this commit
- **Notes**: 问题记录已明确 `Order` 是唯一权威优先级顺序，状态仅用于筛选；LW-016 在 P1 内因安全风险排在 LW-013 前。

---

## [LRN-20260809-001] correction

**Logged**: 2026-08-09T11:47:00+08:00
**Priority**: medium
**Status**: resolved
**Area**: config

### Summary
运行参数注释必须逐项说明单位、特殊值、组合触发逻辑和触发后的行为。

### Details
用户指出控制循环配置虽然已有中文概述，但仍无法从配置文件直接判断每个参数
的具体含义。对部署和安全相关参数，只说明总体目的并不够；注释还需区分逻辑
CPU 编号与实时优先级，解释 `-1`、`0`、`false` 等特殊值，说明计数单位和时间
单位、多个阈值之间的“或”关系、锁存是否需要重启清除，以及启用致命阈值后
会采取的具体动作。

### Suggested Action
在 `policy/LW/base.yaml` 中紧邻各控制循环参数补充可独立阅读的中文注释，并以
实际实现为准说明连续跳过周期数、截止时间晚到秒数、实时调度回退和安全动作。

### Metadata
- Source: user_feedback
- Related Files: policy/LW/base.yaml, src/rl_sar/library/core/loop/loop.hpp
- Tags: configuration, comments, realtime, timing, safety
- Pattern-Key: docs.config_operational_semantics
- Recurrence-Count: 1
- First-Seen: 2026-08-09
- Last-Seen: 2026-08-09

### Resolution
- **Resolved**: 2026-08-09T11:52:00+08:00
- **Commit/PR**: 本次配置注释修改
- **Notes**: `base.yaml` 已逐项说明 CPU 绑定、实时调度回退、计数与时间单位、阈值“或”关系、锁存清除方式以及致命时序动作，未改变任何参数值或控制逻辑。

---

## [LRN-20260809-002] correction

**Logged**: 2026-08-09T15:23:00+08:00
**Priority**: medium
**Status**: resolved
**Area**: docs

### Summary
讨论离散动作时间轴时必须区分“第几帧”的自然编号与相对起始时间，不能用“第 0 帧”混淆两者。

### Details
在 LW-012 方案中使用“第 0 帧发生在 0 秒”的表述，用户指出动作数据只有第 1
帧、第 2 帧等自然编号。真正需要决定的是：第 1 帧相对动作激活时刻是位于
`t=0`，还是位于 `t=1/fps`；若选择后者，还必须定义 `[0, 1/fps]` 内控制器
输出什么参考。用户随后提供了实际 CSV 生成代码：目标时间轴从 `t=0` 开始，
但导出前执行 `csv_data[1:-1]` 删除绝对起点和终点。因此当前 CSV 第一行确实
对应原轨迹的 `t=1/fps`，第 N 行对应 `t=N/fps`；数组下标从 0 开始不能替代
这一生成端时间语义。

### Suggested Action
LW-012 应记录生成端契约并保留 `duration=N/fps`；运行时明确首帧前区间的参考，
从 `t=1/fps` 起按 `frame=t*fps-1` 对齐 60 Hz 样本，再统一速度、FSM 完成判断
和测试。

### Metadata
- Source: user_feedback
- Related Files: src/rl_sar/library/core/motion_loader/motion_loader_lw.cpp, .learnings/LW_REAL_DEPLOYMENT_ISSUES.md
- Tags: LW, motion, frame-index, timestamp, terminology
- Pattern-Key: docs.motion_frame_number_vs_timestamp
- Recurrence-Count: 1
- First-Seen: 2026-08-09
- Last-Seen: 2026-08-09

### Resolution
- **Resolved**: 2026-08-09T15:53:18+08:00
- **Commit/PR**: 本提交（基线 `6853b6c`）
- **Notes**: LW-012 现以 `motion_time_offset_frames` 明确区分自然帧编号、数组下标和相对动作时间；当前 CSV 配置为偏移 1，首个样本前保持第一行，并通过偏移 0/1、时间边界、插值、速度及错误输入测试。

---

## [LRN-20260811-001] correction

**Logged**: 2026-08-11T13:17:46+08:00
**Priority**: medium
**Status**: resolved
**Area**: infra

### Summary
项目的统一首次构建入口应自动准备缺失依赖，而不是只报告系统包前置条件。

### Details
在 LW-018 初版实现中，删除 `build_LW.sh` 后曾计划把系统包安装改成人工步骤。
用户明确指出，新环境第一次运行 `build.sh` 的预期就是自动下载和安装相关库。
正确契约是：默认检查并自动安装缺失的 Debian/Ubuntu 与 ROS 构建包，继续自动
下载项目管理的推理/仿真运行时；已满足的依赖不得重复安装，首次安装可以请求
sudo 和网络。测试或受控离线环境可以显式跳过系统包安装，但不能改变默认行为。

### Suggested Action
由 `build.sh` 在编译前调用一个可独立检查的系统依赖安装器，再准备
LibTorch、ONNX Runtime 和按需 MuJoCo；文档明确首次运行的联网和 sudo 行为，
测试覆盖包清单、跳过开关和调用顺序。

### Metadata
- Source: user_feedback
- Related Files: build.sh, scripts/install_build_dependencies.sh, README_CN.md
- Tags: build, bootstrap, dependencies, first-run, Jetson
- Pattern-Key: build.first_run_bootstraps_dependencies
- Recurrence-Count: 1
- First-Seen: 2026-08-11
- Last-Seen: 2026-08-11

### Resolution
- **Resolved**: 2026-08-11T13:31:54+08:00
- **Commit/PR**: 本提交
- **Notes**: `build.sh` 现默认在编译前检查并自动安装缺失的系统与 ROS 构建包，
  随后准备推理运行时及按需 MuJoCo；依赖齐全时不会调用包管理器。隔离测试已
  验证缺包安装、依赖齐全跳过、包清单及执行顺序，中文文档同步说明联网、
  sudo 和受控测试跳过方式。

---
