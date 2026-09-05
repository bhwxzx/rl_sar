# LW Real-Deployment Issue Register

本文件是 LW 整改工作的唯一权威入口，记录当前待办顺序与历史索引。

## 执行规则

- 由用户逐项选择问题；修改代码前先说明具体方案并获得明确审批。
- 每次只处理一个已选问题，不捆绑后续问题或无关清理。
- 验证后只更新所选问题的状态、解决说明和验收证据。
- 编号永久保留，不因归档重新编号。若以后确认重复，保留原编号并标明合并去向。
- 状态：`pending` 待处理；`in_progress` 已批准实施；`resolved` 已验证完成；
  `deferred` 暂缓；`wont_fix` 不处理（须说明原因）。
- 历史归档仅提供追溯，不作为新的待办清单。具体审批和执行状态以本文件为准。

## 当前待办

LW-067～LW-070 已完成；当前待办为 LW-071，尚未获实施审批。
下表保留本轮处理进度，由用户选择下一项。

| 顺序 | ID | 优先级 | 状态 | 问题 |
|---:|---|---|---|---|
| 1 | [LW-067](#lw-067) | P2 / medium | resolved | 修复构建目标检查对合法编译配置的误报 |
| 2 | [LW-068](#lw-068) | P2 / low | resolved | 将只读动作裁剪配置校验集中到所属边界 |
| 3 | [LW-069](#lw-069) | P2 / low | resolved | 合并基础配置及启动超时参数的重复校验 |
| 4 | [LW-070](#lw-070) | P2 / low | resolved | 将 ONNX 私有缓存不变量检查集中到加载阶段 |
| 5 | [LW-071](#lw-071) | P2 / low | pending | 减少源码写法绑定和重复生命周期测试断言 |

## 本轮审查依据与边界

- 审查日期：2026-09-05；代码基线：`83e22d5`。
- 已有验证目录的 rl_sar Debug CTest 52/52、FDILink CTest 12/12 通过。
  全新 CMake 配置分别复现额外 `CMAKE_CXX_FLAGS=-Wall` 和
  `CMAKE_NO_SYSTEM_FROM_IMPORTED=ON` 引起的目标检查误报。
- 顺序依据：先处理已复现的测试错误，再处理热路径重复扫描、启动配置规则、
  ONNX 内部不变量和结构测试维护成本。未测量性能收益。
- 保留推理前后及消费端时效、FSM 转换后姿态、模型动作与最终电机命令、
  FDILink 帧完整性/载荷语义/接收端时效等不同阶段的必要检查及行为测试。

---

<a id="lw-067"></a>

## [LW-067] Fix target-scope test false positives for valid compiler configurations

**Priority**: P2 / medium
**Status**: resolved
**Dependencies**: LW-066

### Problem and Evidence

- `src/rl_sar/test/test_lw_build_target_scope.py:40` recognizes a standalone
  include-path token but not the equivalent `-I/path` spelling.
- Line 50 requires warning options to be absent when strict mode is off,
  including options explicitly supplied by the user or toolchain.
- Fresh configure with `LW_STRICT_WARNINGS=OFF` and `CMAKE_CXX_FLAGS=-Wall`
  succeeds, but the checker rejects test_lw_joinable_worker for `-Wall`.
- Fresh configure with `CMAKE_NO_SYSTEM_FROM_IMPORTED=ON` succeeds, but the
  checker misreports inference_runtime's ONNX header/layout contract.

### Intended Scope

- Correct compiler-argument parsing and distinguish project warning policy
  from user/toolchain flags; check actual dependency and ABI requirements.
- Limit changes to the target-scope checker and its necessary regression
  coverage/CMake wiring. General source-text test cleanup belongs to LW-071.

### Acceptance Criteria

- Both reproduced valid configurations pass the relevant scope checks.
- Missing ONNX ABI definitions, genuinely leaked dependencies, and missing
  project strict-warning settings are still detected by negative cases.
- Ordinary Debug and strict checks remain green; no production RPATH or
  application behavior changes.

### Resolution (2026-09-05)

- 用户明确批准仅实施 LW-067；变更与本验收记录一并提交。
- 参数检查统一解析分离/连写的 include、define、undefine 选项，支持
  相对路径和带空格路径。SDK/ONNX 隔离检查使用解析后的路径与宏。
- CMake 仅在 Linux BUILD_TESTING 下导出各编译目标实际 COMPILE_OPTIONS；
  严格警告同时核对目标属性与最终命令，允许用户通过 CMAKE_CXX_FLAGS
  额外启用警告。检查仍能拒绝项目给 vendor 施加的严格警告，以及被用户
  同名选项掩盖的项目警告缺失。保留必需目标存在性和原有 ELF/RPATH 检查。
- 修改 CMake 测试接线和 test_lw_build_target_scope.py，新增
  test_lw_build_target_scope_regressions.py；5 项针对性回归覆盖等价参数
  写法、额外用户警告、缺失/取消 ONNX 宏、实际依赖泄漏、项目严格警告缺失
  及 vendor 警告泄漏。正负例均通过。
- 全新 `/tmp/lw067-debug` 和 `/tmp/lw067-strict` 构建成功，完整 CTest
  各 **53/53** 通过。全新 `/tmp/lw067-user-warnings` 使用
  `LW_STRICT_WARNINGS=OFF; CMAKE_CXX_FLAGS=-Wall`，全新
  `/tmp/lw067-normal-includes` 使用 `CMAKE_NO_SYSTEM_FROM_IMPORTED=ON`；
  两者完整构建成功，构建工作流、运行时链接、目标作用域及解析回归共
  **4/4** 定向 CTest 各自通过。最终保留目标存在性检查后，四种配置的
  两项作用域相关测试再次各 **2/2** 通过。
- git diff --check 通过。生产 RPATH、C++ 控制代码和策略资产未改动；
  未进行新的生产部署或硬件验证。用户技能目录保持原样，LW-068～LW-071
  未实施。

---

<a id="lw-068"></a>

## [LW-068] Validate immutable action-clipping configuration at its owning boundary

**Priority**: P2 / low
**Status**: resolved
**Dependencies**: LW-013, LW-038

### Problem and Evidence

- `src/rl_sar/library/core/safety/lw_runtime_core.hpp:604` checks empty bounds,
  vector lengths, and finite values on every inference cycle.
- `src/rl_sar/library/core/rl_sdk/lw_configuration_validation.cpp:510`
  already validates both vectors and lower/upper ordering before publication
  through the read-only policy definition.
- The maintained production path provides no new configuration input between
  those validations. Runtime cost reduction has not been quantified.

### Intended Scope

- Establish the validated configuration contract at loading/activation and
  remove repeated hot-path checks of those same immutable properties.
- Preserve validation of newly inferred actions and the clipping operation.
  Do not alter model output, policy files, or safety-event handling generally.

### Acceptance Criteria

- Missing, malformed, nonfinite, and reversed clipping bounds fail before
  inference workers consume a policy definition.
- Finite actions retain identical clipping results; invalid model actions
  retain the existing safety response.
- Configuration, inference/runtime parity, and allocation regressions pass.

### Resolution (2026-09-05)

- 用户明确批准仅实施 LW-068；变更与本验收记录一并提交。
- 已核对真机、Sim2Sim 与 profiler 的正式调用路径：PreloadModel 先调用
  ValidateLWPolicyConfiguration，再由 PreloadLWPolicyContext 发布只读定义，
  ActivateLWPolicy 只激活已预加载的定义。保留现有加载校验，不新增重复入口。
- lw_runtime_core.hpp 删除每轮对裁剪上下限长度/有限值的重复扫描及空数组
  跳过分支，直接使用已验证上下限裁剪。每轮模型动作尺寸/有限值检查、
  裁剪后的输出检查和安全响应保持原样。
- 在已有配置测试中覆盖两侧边界缺失、非数组、空数组、长度错误、NaN/Inf、
  上下限颠倒及上下限相等；在已有运行时测试中注入固定模型输出，核对区间内、
  边界上和越界动作的裁剪结果，并验证 NaN、正负 Inf 触发
  PolicyActionInvalid/PassiveDamping 且不发布策略输出。
- 配置、运行时一致性及分配约束定向 CTest **3/3** 通过；在已有
  `/tmp/lw067-debug` 和 `/tmp/lw067-strict` 目录重建全部目标，两者完整
  CTest 各 **53/53** 通过。git diff --check 通过。
- 修改限于运行时核心、上述两份测试及本项记录；策略资产、控制时序与其他
  问题未修改，未访问真实硬件。未测量性能收益，也未进行新的生产部署验证。

---

<a id="lw-069"></a>

## [LW-069] Consolidate repeated base-configuration and startup timeout validation

**Priority**: P2 / low
**Status**: resolved
**Dependencies**: LW-013, LW-038

### Problem and Evidence

- `src/rl_sar/src/rl_real_LW.cpp:106` validates the base configuration, then
  repeats finite/positive checks for sensor_timeout, trusted_imu_timeout,
  imu_ahrs_pair_max_age, and serial_write_timeout.
- The same four rules exist in
  `src/rl_sar/library/core/rl_sdk/lw_configuration_validation.cpp:345`.
- ValidateLWPolicyConfiguration also revalidates the base configuration on
  each policy preload, despite the maintained startup path having validated it.

### Intended Scope

- Centralize these rules and reuse validated base results in maintained
  startup/preload paths, while preserving a checked entry for raw YAML callers.
- Preserve independent checks after time-unit conversion, especially rejection
  of durations rounded to zero, and all startup-disable lifecycle guarantees.

### Acceptance Criteria

- Every affected invalid configuration still fails before worker startup with
  an actionable diagnostic; raw-YAML callers cannot bypass validation.
- Valid real/profiler/simulation configurations retain their existing values.
- Startup, configuration, and profiler regressions pass; no worker sequencing
  or serial command changes are bundled.

### Resolution (2026-09-05)

- 用户明确批准仅实施 LW-069；变更与本验收记录一并提交。
- 四个超时参数由 ValidateLWBaseConfiguration 统一解析、校验并保留到
  LWBaseRuntimeConfiguration；真机启动删除重复的有限值/正数检查，
  真机和 profiler 直接使用类型化结果。原有时长转换、接收端非正时长检查、
  启动禁用及工作线程顺序保持不变。
- 新增 LWValidatedBaseConfiguration，构造时克隆并验证基础 YAML，私有只读
  快照与运行时结果绑定。真机、仿真及 profiler 安装此对象；PreloadModel
  复用它校验各策略，不再逐策略全量校验基础配置。原始 YAML 策略入口仍先
  完整验证基础配置，策略自身及合并配置的必要校验未删除。
- 原运行时数值 setter 会清除已验证快照；未安装快照的预加载路径仍必须
  验证 YAML，避免将未经验证的数值或过期快照当成校验凭据。基础配置来源
  或返回的合并 YAML 被修改时，不会影响已安装快照。
- 配置测试覆盖四个参数缺失、错误类型、零/负值、NaN/正负 Inf、原始 YAML
  入口拒绝错误基础配置、快照隔离与预加载复用；四个现有策略的合并配置
  在两种入口间保持一致，类型化超时值与 YAML 一致。时间边界测试覆盖
  IMU、motor、IMU/AHRS 及串口超时由极小正秒数转换为零后的拒绝行为。
- 复用 `/tmp/lw067-debug`、`/tmp/lw067-strict` 重建全部目标（包括真机、
  仿真和 profiler）；最终完整 CTest 各 **53/53** 通过，包含配置、启动、
  profiler、运行时一致性和分配约束回归。git diff --check 通过。
- 生命周期源码测试仅同步本项校验入口名称，保留原顺序断言，未实施 LW-071。
  未修改策略资产、串口命令或其他问题，未访问真实硬件，未测量性能收益。

---

<a id="lw-070"></a>

## [LW-070] Consolidate ONNX private-cache invariant checks at model load

**Priority**: P2 / low
**Status**: resolved
**Dependencies**: LW-050, LW-063

### Problem and Evidence

- `src/rl_sar/library/core/inference_runtime/inference_runtime.cpp:198`
  checks four private metadata/name containers on every forwardInto call.
- Line 382 checks the output metadata count again in validateOutput.
- setup_input_output_info validates the single-input/output contract; load
  sets loaded_ only after successful setup. These private caches do not change
  during normal inference.

### Intended Scope

- Consolidate internal cache invariants at successful load and preserve
  consistent state after failed or replacement loads.
- Retain public tensor-view pointer/count/size checks. Removing checks on
  actual ONNX execution results is not implied by this issue.
- Do not change shared-environment ownership, model isolation, or reload
  concurrency contracts.

### Acceptance Criteria

- Invalid models fail loading; failed reloads leave the model unusable rather
  than exposing partial caches. Valid inference remains numerically identical.
- Invalid caller buffers and execution failures remain detectable.
- Inference contracts, model isolation/lifetime, and runtime regressions pass.

### Resolution (2026-09-05)

- 用户明确批准仅实施 LW-070；变更与本验收记录一并提交。
- 将 forwardInto 中四个私有元数据/名称容器的数量检查移到
  setup_input_output_info 末尾，在 load 设置 loaded_ 之前建立缓存契约；
  删除 validateOutput 中重复的输出元数据数量检查。
- 保留模型已加载、调用方输入数量、空指针和缓冲区大小检查；保留实际
  ONNX 输出的张量类型、float32 类型及元素数量校验和执行异常传播。
  通用 Model::forward 接口检查、共享环境、模型隔离及并发约定均未修改。
- 加载前及失败后的 reset_loaded_state 保持原样。扩展已有重载测试，覆盖
  会话创建失败、输入元数据拒绝、输出元数据拒绝；每次失败均确认 loaded
  为 false、元数据清空、forward/forwardInto 拒绝执行，随后加载不同维度
  的合法模型并验证数值输出，排除残留缓存影响。
- 在已有缓冲区测试中补充输入视图数组为空、输入数据为空和输出缓冲区
  过长的拒绝用例；复用已有非法模型、输入数量/尺寸、输出为空/过短、
  调用方存储保持、并发模型生命周期与合法模型数值回归。
- 复用 `/tmp/lw067-debug`、`/tmp/lw067-strict` 重建全部目标，完整 CTest
  各 **53/53** 通过（包括推理契约、配置、运行时一致性、分配约束和
  profiler 回归）；git diff --check 通过。
- 修改仅涉及推理实现、已有推理测试与本项记录；未处理 LW-071，未访问
  真实硬件，未测量性能收益。

---

<a id="lw-071"></a>

## [LW-071] Reduce source-spelling coupling and duplicate lifecycle test assertions

**Priority**: P2 / low
**Status**: pending
**Dependencies**: LW-059, LW-066, LW-067

### Problem and Evidence

- `src/rl_sar/test/test_lw_real_startup_disable_integration.py:23` matches
  literal source text including indentation and newlines.
- `src/rl_sar/test/test_build_workflow.py:150` and subsequent checks require
  exact CMake statement spelling rather than only the resulting contract.
- `src/rl_sar/test/test_lw_sim_lifecycle_integration.py:162` repeats shutdown
  ordering already implied by the immediately preceding test's worker order
  and last-worker-before-physics assertions.

### Intended Scope

- Consolidate duplicate assertions and reduce exact source-spelling checks
  in the identified tests, using existing behavior/artifact checks where they
  cover the same requirement. Keep narrowly necessary wiring checks.
- Preserve startup/rollback/shutdown coverage; do not delete whole safety
  suites or introduce a broad production-code refactor just for testing.
- LW-067 owns generated compiler-argument parsing and its reproduced errors.

### Acceptance Criteria

- Behavior-preserving whitespace/layout changes do not fail the affected
  checks; duplicate lifecycle assertions have one clear owner.
- Regressions in worker shutdown/rollback ordering and relevant dependency
  contracts still fail meaningful checks, not merely text snapshots.
- Relevant lifecycle, build-workflow, and runtime-linkage suites pass.

---

## 已完成问题索引

LW-001～LW-066 均已结项，详细原因、审批与验收证据见
[完整历史归档](archive/LW_REAL_DEPLOYMENT_RESOLVED_001_066.md)。
“结项记录提交”取自 Git 中将该问题状态写为 resolved 的提交，不保证是唯一实施提交；
分阶段实施与后续变更仍可通过 Git 历史追溯。

| ID | 问题 | 状态 | 结项记录提交 |
|---|---|---|---|
| [LW-001](archive/LW_REAL_DEPLOYMENT_RESOLVED_001_066.md#lw-001) | Fail-safe loop shutdown and exception boundary | resolved | `11e57ccc` |
| [LW-002](archive/LW_REAL_DEPLOYMENT_RESOLVED_001_066.md#lw-002) | Sensor and communication readiness/freshness gate | resolved | `917aa770` |
| [LW-003](archive/LW_REAL_DEPLOYMENT_RESOLVED_001_066.md#lw-003) | Serial parser and transmitter robustness | resolved | `8ac78e80` |
| [LW-004](archive/LW_REAL_DEPLOYMENT_RESOLVED_001_066.md#lw-004) | FSM transition correctness | resolved | `15647aec` |
| [LW-005](archive/LW_REAL_DEPLOYMENT_RESOLVED_001_066.md#lw-005) | Finite command validation and active protection | resolved | `a2b79e74` |
| [LW-006](archive/LW_REAL_DEPLOYMENT_RESOLVED_001_066.md#lw-006) | Joystick disconnect and input validation | resolved | `a96dcf49` |
| [LW-007](archive/LW_REAL_DEPLOYMENT_RESOLVED_001_066.md#lw-007) | Coherent cross-thread state | resolved | `a665d4ec` |
| [LW-008](archive/LW_REAL_DEPLOYMENT_RESOLVED_001_066.md#lw-008) | Atomic policy output transport | resolved | `633ca283` |
| [LW-009](archive/LW_REAL_DEPLOYMENT_RESOLVED_001_066.md#lw-009) | Wheel-to-leg motion reference rate | resolved | `923182db` |
| [LW-010](archive/LW_REAL_DEPLOYMENT_RESOLVED_001_066.md#lw-010) | Reproducible deployment artifacts | resolved | `8272253f` |
| [LW-011](archive/LW_REAL_DEPLOYMENT_RESOLVED_001_066.md#lw-011) | Deterministic control-loop timing | resolved | `9978230c` |
| [LW-012](archive/LW_REAL_DEPLOYMENT_RESOLVED_001_066.md#lw-012) | Motion-loader robustness and time convention | resolved | `190456dd` |
| [LW-013](archive/LW_REAL_DEPLOYMENT_RESOLVED_001_066.md#lw-013) | Configuration and dimension validation | resolved | `6853b6cb` |
| [LW-014](archive/LW_REAL_DEPLOYMENT_RESOLVED_001_066.md#lw-014) | Debug and plot publishing isolation | resolved | `6761b82b` |
| [LW-015](archive/LW_REAL_DEPLOYMENT_RESOLVED_001_066.md#lw-015) | LW-only repository scope and future robot extension boundary | resolved | `241f7d69` |
| [LW-016](archive/LW_REAL_DEPLOYMENT_RESOLVED_001_066.md#lw-016) | Safety-action proportionality and recovery audit | resolved | `6853b6cb` |
| [LW-017](archive/LW_REAL_DEPLOYMENT_RESOLVED_001_066.md#lw-017) | Reproducible IMU and serial runtime deployment | resolved | `a65a5dcd` |
| [LW-018](archive/LW_REAL_DEPLOYMENT_RESOLVED_001_066.md#lw-018) | Unified build entry point and Jetson detection | resolved | `a65a5dcd` |
| [LW-019](archive/LW_REAL_DEPLOYMENT_RESOLVED_001_066.md#lw-019) | Real-robot terminal keyboard recovery channel | resolved | `379a5165` |
| [LW-020](archive/LW_REAL_DEPLOYMENT_RESOLVED_001_066.md#lw-020) | Architecture-safe ONNX-only Jetson production inference | resolved | `cc07efc4` |
| [LW-021](archive/LW_REAL_DEPLOYMENT_RESOLVED_001_066.md#lw-021) | Sim2Sim and real-runtime behavioral parity | resolved | `25ba1927` |
| [LW-022](archive/LW_REAL_DEPLOYMENT_RESOLVED_001_066.md#lw-022) | Suspended real-runtime profiling and configuration candidates | resolved | `57f184e6` |
| [LW-023](archive/LW_REAL_DEPLOYMENT_RESOLVED_001_066.md#lw-023) | Disable-before-startup safety boundary | resolved | `542dcc9b` |
| [LW-024](archive/LW_REAL_DEPLOYMENT_RESOLVED_001_066.md#lw-024) | Joinable and bounded Sim2Sim physics lifecycle | resolved | `efd339cd` |
| [LW-025](archive/LW_REAL_DEPLOYMENT_RESOLVED_001_066.md#lw-025) | Persistent and composable keyboard velocity input | resolved | `1b29744f` |
| [LW-026](archive/LW_REAL_DEPLOYMENT_RESOLVED_001_066.md#lw-026) | Configuration-profile provenance and comparability | resolved | `5425b679` |
| [LW-027](archive/LW_REAL_DEPLOYMENT_RESOLVED_001_066.md#lw-027) | Reproducible ONNX Runtime deployment dependency | resolved | `83890c51` |
| [LW-028](archive/LW_REAL_DEPLOYMENT_RESOLVED_001_066.md#lw-028) | Signal-safe Sim2Sim shutdown request | resolved | `3a73aad7` |
| [LW-029](archive/LW_REAL_DEPLOYMENT_RESOLVED_001_066.md#lw-029) | Sim2Sim actuator-model policy-root consistency | resolved | `2b325977` |
| [LW-030](archive/LW_REAL_DEPLOYMENT_RESOLVED_001_066.md#lw-030) | Coherent inhibited-command gait observation | resolved | `095f13be` |
| [LW-031](archive/LW_REAL_DEPLOYMENT_RESOLVED_001_066.md#lw-031) | Warning-clean maintained LW build | resolved | `769b6e89` |
| [LW-032](archive/LW_REAL_DEPLOYMENT_RESOLVED_001_066.md#lw-032) | End-to-end IMU and AHRS validity and freshness | resolved | `0fe6d291` |
| [LW-033](archive/LW_REAL_DEPLOYMENT_RESOLVED_001_066.md#lw-033) | Policy input provenance and freshness | resolved | `71371e0d` |
| [LW-034](archive/LW_REAL_DEPLOYMENT_RESOLVED_001_066.md#lw-034) | Trusted inference-runtime download integrity | resolved | `1f36ba63` |
| [LW-035](archive/LW_REAL_DEPLOYMENT_RESOLVED_001_066.md#lw-035) | Production launch-file integrity | resolved | `f80b916d` |
| [LW-036](archive/LW_REAL_DEPLOYMENT_RESOLVED_001_066.md#lw-036) | Runtime actuator-network output validation | resolved | `381c3949` |
| [LW-037](archive/LW_REAL_DEPLOYMENT_RESOLVED_001_066.md#lw-037) | Complete Sim2Sim SIGTERM and ROS shutdown | resolved | `d0e1bad2` |
| [LW-038](archive/LW_REAL_DEPLOYMENT_RESOLVED_001_066.md#lw-038) | Allocation-bounded real control cycle | resolved | `6c2db924` |
| [LW-039](archive/LW_REAL_DEPLOYMENT_RESOLVED_001_066.md#lw-039) | Actuator-model policy-root containment | resolved | `eb64b3d8` |
| [LW-040](archive/LW_REAL_DEPLOYMENT_RESOLVED_001_066.md#lw-040) | Safe polymorphic RL destruction contract | resolved | `267fb263` |
| [LW-041](archive/LW_REAL_DEPLOYMENT_RESOLVED_001_066.md#lw-041) | Opt-in Sim2Sim plot publishing | resolved | `476e112a` |
| [LW-042](archive/LW_REAL_DEPLOYMENT_RESOLVED_001_066.md#lw-042) | Nonblocking, source-fresh real debug telemetry | resolved | `205cc70d` |
| [LW-043](archive/LW_REAL_DEPLOYMENT_RESOLVED_001_066.md#lw-043) | Retire the Sim2Sim actuator-model runtime while preserving offline training | resolved | `37188881` |
| [LW-044](archive/LW_REAL_DEPLOYMENT_RESOLVED_001_066.md#lw-044) | Complete and failure-visible FDILink frame ingestion | resolved | `cc54c5f4` |
| [LW-045](archive/LW_REAL_DEPLOYMENT_RESOLVED_001_066.md#lw-045) | Explicit and architecture-safe FDILink payload decoding | resolved | `559ac3eb` |
| [LW-046](archive/LW_REAL_DEPLOYMENT_RESOLVED_001_066.md#lw-046) | Reject semantically invalid FDILink samples before publication | resolved | `a9f981dd` |
| [LW-047](archive/LW_REAL_DEPLOYMENT_RESOLVED_001_066.md#lw-047) | Classify FDILink 8-bit sequence anomalies without inventing frame loss | resolved | `1fda05ad` |
| [LW-048](archive/LW_REAL_DEPLOYMENT_RESOLVED_001_066.md#lw-048) | Bind installed ONNX Runtime bytes to the approved archive | resolved | `6ef1b97e` |
| [LW-049](archive/LW_REAL_DEPLOYMENT_RESOLVED_001_066.md#lw-049) | Make retained Gazebo controllers bounded, URDF-ready, and allocation-stable | resolved | `ef0af1d3` |
| [LW-050](archive/LW_REAL_DEPLOYMENT_RESOLVED_001_066.md#lw-050) | Unify the ONNX dynamic-batch contract and cached tensor resources | resolved | `0ce245a3` |
| [LW-051](archive/LW_REAL_DEPLOYMENT_RESOLVED_001_066.md#lw-051) | Make MuJoCo downloads digest-pinned and installation atomic | resolved | `af0b0c0c` |
| [LW-052](archive/LW_REAL_DEPLOYMENT_RESOLVED_001_066.md#lw-052) | Harden generic rl_sim joystick bounds and temporary-file lifecycle | resolved | `11183fc1` |
| [LW-053](archive/LW_REAL_DEPLOYMENT_RESOLVED_001_066.md#lw-053) | Cache the rl_sim_LW debug message layout | resolved | `d85f6808` |
| [LW-054](archive/LW_REAL_DEPLOYMENT_RESOLVED_001_066.md#lw-054) | Preload morphology-transition motion assets before control workers | resolved | `79bf0b76` |
| [LW-055](archive/LW_REAL_DEPLOYMENT_RESOLVED_001_066.md#lw-055) | Remove blocking synchronization from the real control deadline path | resolved | `fa3fe9f3` |
| [LW-056](archive/LW_REAL_DEPLOYMENT_RESOLVED_001_066.md#lw-056) | Make the maintained complete control cycle allocation-stable | resolved | `151a3e97` |
| [LW-057](archive/LW_REAL_DEPLOYMENT_RESOLVED_001_066.md#lw-057) | Validate the MuJoCo control adapter layout and test actual safety actions | resolved | `39748434` |
| [LW-058](archive/LW_REAL_DEPLOYMENT_RESOLVED_001_066.md#lw-058) | Isolate maintained C++ targets from the unused Python runtime | resolved | `6b610110` |
| [LW-059](archive/LW_REAL_DEPLOYMENT_RESOLVED_001_066.md#lw-059) | Stop Sim2Sim workers on every partial-construction failure path | resolved | `bf21601e` |
| [LW-060](archive/LW_REAL_DEPLOYMENT_RESOLVED_001_066.md#lw-060) | Use the history-frame domain consistently in ObservationBuffer | resolved | `e8669452` |
| [LW-061](archive/LW_REAL_DEPLOYMENT_RESOLVED_001_066.md#lw-061) | Make motion-reference validation and runtime gating semantically consistent | resolved | `161feb00` |
| [LW-062](archive/LW_REAL_DEPLOYMENT_RESOLVED_001_066.md#lw-062) | Reuse contiguous buffers throughout the inference hot path | resolved | `e620d0b8` |
| [LW-063](archive/LW_REAL_DEPLOYMENT_RESOLVED_001_066.md#lw-063) | Share the ONNX Runtime environment without weakening model isolation | resolved | `de1f55fc` |
| [LW-064](archive/LW_REAL_DEPLOYMENT_RESOLVED_001_066.md#lw-064) | Remove or correctly implement the misleading FDILink CRC32 API | resolved | `fea0ff9a` |
| [LW-065](archive/LW_REAL_DEPLOYMENT_RESOLVED_001_066.md#lw-065) | Restore a clean FDILink lint and package-metadata baseline | resolved | `e3596f48` |
| [LW-066](archive/LW_REAL_DEPLOYMENT_RESOLVED_001_066.md#lw-066) | Make dependency discovery ordered and build settings target-scoped | resolved | `83e22d55` |

## 解决记录模板

完成所选问题后填写以下信息，明确实际验证范围：

```markdown
**Status**: resolved

### Resolution
- Resolved: YYYY-MM-DDTHH:MM:SS+08:00
- Commit: <提交后填写真实哈希；尚未提交时如实注明>
- Approved Scope: <用户批准的范围>
- Changed Files: <文件路径>
- Verification: <命令、结果及验证限制>
- Remaining Follow-ups: <问题编号；不修改其他问题状态>
```
