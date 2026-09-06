# LW 已完成问题归档：LW-067～LW-071

[返回权威问题记录](../LW_REAL_DEPLOYMENT_ISSUES.md)

- 归档日期：2026-09-06；核对基线：`d03c6bd`。
- 保存本轮 5 项问题的完整审查背景、方案、审批与验收证据；当前状态以主文档为准。
- 各项 Resolution 的结项提交已按 Git 历史补齐。
- 原问题行号、调用路径、临时目录及“后续问题未实施”等文字属于当时的历史语境，
  不代表当前状态，也不代表已完成实机安全验证。

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

- 用户明确批准仅实施 LW-067；结项提交：`39a3d22`。
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

- 用户明确批准仅实施 LW-068；结项提交：`76dc6bb`。
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

- 用户明确批准仅实施 LW-069；结项提交：`eeb1d16`。
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

- 用户明确批准仅实施 LW-070；结项提交：`8f32b25`。
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
**Status**: resolved
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

### Resolution (2026-09-06)

- 用户明确批准仅实施 LW-071；结项提交：`d03c6bd`。
- 新增小型 lw_source_checks 测试辅助模块：必要的 C++ 接线检查按词法片段
  匹配，允许空白/换行变化并排除注释；CMake 依赖发现检查读取平坦声明的
  命令与参数，不要求固定缩进、换行、命令大小写或等价的参数顺序。
  此辅助模块不是通用解析器，也不以源码顺序检查替代实际行为测试。
- 真机参数声明、启动禁用接线及命令门关闭检查改用上述辅助函数；删除
  已由 debug publisher 行为测试覆盖的重复序号比较写法断言，保留非阻塞
  发布接线检查。launch 接线及其他未涉及断言未做泛化清理。
- 将真机/仿真工作线程关闭及后端停止顺序统一归属共享生命周期测试，
  删除重复的仿真析构顺序测试和真机最终禁用顺序断言；启动失败回滚与
  命令门关闭仍分别保留，不合并不同安全语义。
- 删除 ONNX 宏传播的精确 CMake 声明断言，由现有 lw_build_target_scope
  检查实际编译命令；继续运行 lw_runtime_linkage 检查实际链接产物。
  保留解释器发现先于 ament/使用、系统 fmt 选择、禁止 Python 开发组件、
  禁止全局编译/RPATH 污染等必要源码约束，未修改 LW-067 的检查器。
- 在临时副本中执行正反例：已调整的调用布局和 CMake 声明重排通过；交换
  真机/仿真回滚或析构关闭顺序、提前停止后端、注释掉命令门关闭、追加
  Python Development、延后解释器发现、加入全局编译选项或 RPATH 均被拒绝。
  副本不会改动生产源码或仓库 CMakeLists.txt。
- 三项定向 CTest **3/3** 通过；复用 `/tmp/lw067-debug`、
  `/tmp/lw067-strict` 构建并运行完整 CTest，各 **53/53** 通过，包含
  生命周期、构建流程、实际依赖/链接、调试发布及运行时回归。
  git diff --check 通过。
- 修改限于三份现有 Python 测试、一个测试辅助模块及本项记录；生产代码、
  构建配置、策略资产均未修改，未访问真实硬件。

---
