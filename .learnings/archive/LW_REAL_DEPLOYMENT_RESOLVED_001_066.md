# LW 已完成问题归档：LW-001～LW-066

[返回权威问题记录](../LW_REAL_DEPLOYMENT_ISSUES.md)

- 归档日期：2026-09-05；核对基线：`83e22d5`。
- 本归档保存 66 项已完成问题的完整问题说明、审批决策及验收证据。
  当前待办、执行顺序和通用规则统一维护在主文档。
- 结项提交由 Git blame 的 resolved 状态行核对；原“本提交”等占位文字已补齐。
  LW-061 详情及提交 `161feb00` 确认已完成，原汇总表的 pending 已修正。
  LW-065 的“尚未 Git 提交”已按 `e3596f48` 修正。
- 以下审查背景、原始方案和后续事项均属于当时的历史语境。
  用户调整过范围时，以该项解决记录中的明确审批和最终验收结论为准；
  历史方案不代表当前仍需实施。
- 文件行号、临时构建目录和隔离验证快照保留为历史证据，不保证路径仍存在，
  也不代表当前正式部署。仓库路径均相对于仓库根目录。

## 审查基线

- 审查日期：2026-07-29
- 审查时仓库 HEAD：`e4a2765`
- 主要入口：`src/rl_sar/src/rl_real_LW.cpp`
- 范围：实机入口、FSM、核心 RL 运行时、循环实现、LW 串口 SDK、手柄输入、动作加载器、策略 YAML/ONNX/CSV、CMake 和 ROS 启动集成。
- 审查方法：静态检查、`cppcheck`、文件系统及构建产物检查、离线 ONNX Runtime 元数据检查。
- 审查期间未启动机器人节点或串口设备。
- 已验证 ONNX 维度：
  - `leg_loco`：`410 -> 10`
  - `wheel_loco`：`195 -> 10`
  - `leg_to_wheel`：`59 -> 10`
  - `wheel_to_leg`：`59 -> 10`
- 两份转换动作 CSV 文件均保持每行 17 列一致。
- 部署基线并不干净：
  - 三份已跟踪的 LW ONNX 文件有修改。
  - `wheel_to_leg/policy.onnx` 未被跟踪。
  - 已安装的 `rl_real_LW` 指向 2026-07-10 构建的二进制，早于当前源码在 2026-07-23 的修改。

## 优先级调整说明

- 更新日期：2026-08-04
- 当前产品范围仅限 LW，同时仓库应保留清晰、有文档说明的边界，供日后扩展其他机器人。
- `Order` 列是唯一权威优先顺序。按 `Priority` 分组，同一优先级内由实机安全影响和依赖关系决定顺序。`Status` 不构成另一套排序；过滤已解决行只是显示下一项待办。
- P1 中 `LW-016` 排在 `LW-013` 之前，因为超过触发条件所需强度的安全动作本身可能导致跌倒或阻止受控恢复。审查必须保留确有必要的硬停止，不能无差别弱化保护。
- 现有问题编号保持稳定，仅调整权威执行顺序和基于风险的优先级。

## 整改后复审补充

- 审查日期：2026-08-12
- 审查时仓库 HEAD：`57f184e`
- 审查范围：`6761b82` 至 `57f184e` 的全部变更，包括共享 LW 运行时、实机及 Sim2Sim 适配器、输入处理、部署包、Jetson 构建路径和悬吊状态配置性能分析器。
- 验证：当前根构建完成，完整 CTest **31/31** 通过；配置分析器单元测试 **8/8**、Bash 语法检查及 `git diff --check` 通过。还执行了启用警告的构建与定向 `cppcheck` 检查。
- 未启动机器人节点或串口设备，也未发送电机命令。
- 审查确认了下文待办问题 `LW-023` 至 `LW-031`。

## 2026-08-13 全面审查补充

- 审查日期：2026-08-13
- 审查时仓库 HEAD：`769b6e8`
- 审查范围：项目自有 FDILink IMU 驱动、实机和 Sim2Sim 入口、共享运行时与安全核心、策略输出传递、可选执行器模型、生产部署包及启动集成、依赖安装脚本、配置与自动化测试。
- 验证：全新严格 `-Wall -Wextra -Wpedantic -Werror` 构建及完整 CTest **38/38** 通过；所有已跟踪 Bash 和 Python 文件语法检查通过；仓库内 22 份 LW 描述资源全部通过 SHA-256 清单核验；还执行了 `cppcheck` 与 `git diff --check`。
- 原根构建已过期，出现两项非源码故障；全新严格构建确立了当前源码基线。
- ASan/UBSan 构建完成，但完整检测运行不能作为项目结论：ROS Humble/Conda 运行环境组合在外部 `librcl`/`rcutils` 内引发分配器不匹配，随后一项测试停滞。临时构建已移除。
- 未启动 MuJoCo 图形界面、ROS 实机节点、串口设备、IMU 或电机。
- 审查确认了下文新增待办问题 `LW-032` 至 `LW-041`。

## 2026-08-18 跟进审查补充

- 审查日期：2026-08-18
- 审查时仓库 HEAD：`1fda05a`
- 审查范围：已安装 ONNX Runtime 的来源、保留的 Gazebo 关节控制器、通用 ONNX 推理形状处理、MuJoCo 依赖安装、通用 `rl_sim` 手柄与控制器启动、可选 `rl_sim_LW` 调试发布。
- 审查方法：静态检查实现、配置和已有测试。本次仅更新记录，未运行仿真器、ROS 节点、依赖下载、构建或测试套件。
- 未启动串口设备、IMU、手柄、仿真器、实机或电机。
- 早期草案的 `LW-044` 发现已由当前 `LW-044` 解决。由于 `LW-045` 至 `LW-047` 也是现有已解决条目，用户批准为其余六项发现分配唯一编号 `LW-048` 至 `LW-053`，不覆盖已解决历史。
- 审查确认了下文新增待办问题 `LW-048` 至 `LW-053`。

## 2026-08-20 全面复审补充

- 审查日期：2026-08-18；记录日期：2026-08-20
- 审查时仓库 HEAD：`d85f680`
- 审查范围：实机节点启动与安全路径、受维护的 200 Hz 控制热路径和跨线程交接、策略/配置/ONNX 集成、保留的 MuJoCo 适配器假设及自动化测试覆盖。
- 验证：完成静态检查和定向 `cppcheck`；补建缺失的测试目标后，受维护构建的完整 CTest **48/48** 通过。未启动 ROS 节点、仿真图形界面、串口设备、IMU、手柄、实机或电机。
- 审查未发现需要重新打开已解决问题的理由。四项新确认的问题记为 `LW-054` 至 `LW-057`；其顺序先考虑部署风险，再考虑完整控制周期验证所需的依赖。
- 用户所有且未跟踪的 `.agents/skills/inspect-context-compactions/` 目录已保留，不属于本次审查记录的修改范围。

## 2026-08-20 整改后全面复审补充

- 审查及记录日期：2026-08-20
- 审查时仓库 HEAD：`3974843`
- 审查范围：受维护的实机与 Sim2Sim 启动及生命周期路径、共享运行时和策略观测约定、推理热路径、保留的 FDILink 驱动、CMake 与依赖集成、Python 和 shell 工具，以及 `LW-058` 后全部可用自动化测试。
- 验证：全新严格构建完成，完整 CTest **50/50** 通过。强制重新配置过期的 FDILink 构建目录后，五项功能测试全部通过；完整套件为 **8/13**，因为 copyright、cpplint、flake8、lint_cmake 和 uncrustify 检查仍失败。受维护代码的 `cppcheck`、Python AST 解析和 Bash 语法检查未发现其他正确性故障。
- 未启动 ROS 节点、MuJoCo 图形界面、串口设备、IMU、手柄、实机或电机。本次未运行内存与未定义行为检测构建，因此不新增此类验证结论。
- 审查确认下文 `LW-059` 至 `LW-066`。权威顺序先按严重程度，再按正确性先于优化的依赖，最后按运行时相关性。关闭缺陷所需的测试归属该缺陷，不重复列为新问题。
- 用户所有且未跟踪的 `.agents/skills/inspect-context-compactions/` 目录保持原样。

---

<a id="lw-001"></a>

## [LW-001] 循环故障安全退出与异常边界

**优先级**： P0 / 严重
**状态**： resolved
**结项记录提交**： `11e57ccc`
**依赖**： 无

### 问题

`LoopFunc::start()` 将线程分离，因此即便接口打印循环已结束，`shutdown()` 也无法等待线程真正结束。析构时，仍在运行的控制周期可能在禁能包后再次发送使能命令，或访问已销毁的 `RL_Real` 对象。从循环回调逸出的异常也会终止进程，且无法保证发送安全命令。

### 证据

- `src/rl_sar/library/core/loop/loop.hpp:32-60`
- `src/rl_sar/src/rl_real_LW.cpp:118-124`
- `src/rl_sar/library/core/loop/loop.hpp:73-93`

### 计划范围

- 保持循环线程可等待结束。
- 明确定义确定性的退出顺序。
- 在发送最终禁能命令前停止所有产生命令的线程。
- 在线程边界捕获回调异常并触发故障安全路径。
- 确保只有在任何线程都无法重新使能电机后，才尝试最终禁能操作。

### 验收标准

- `shutdown()` 等待到相应回调不可能再运行。
- 最终禁能命令后不能再发送控制命令。
- 人为触发回调异常会记录故障并安全退出，而非调用 `std::terminate`。
- 通过模拟 SDK 测试验证正常退出和异常退出的命令顺序。

### 解决证据

- 解决日期：2026-07-29
- `LoopFunc` 现拥有可等待结束的线程，拒绝重复启动，在线程边界捕获回调异常，并在 `shutdown()` 或析构时等待线程结束。
- `RL_Real` 仅在初始化完成后启动工作线程，先停止产生命令的循环，再通过带锁存的串行化命令门发送最终禁能帧。
- 循环异常会记录来源循环及异常，关闭命令门，尝试紧急禁能并请求 ROS 退出。
- `test_loop_lifecycle` 覆盖等待线程结束、幂等退出、重复启动、异常报告、正常及异常退出时的命令顺序。
- 验证命令与方法：
  - `cmake --build build/rl_sar --target test_loop_lifecycle rl_real_LW -j2`
  - `ctest --test-dir build/rl_sar --output-on-failure -R '^loop_lifecycle$'`
- 结果：`rl_real_LW` 构建成功；`loop_lifecycle` 通过（1/1）。
- 按计划未运行硬件。

---

<a id="lw-002"></a>

## [LW-002] 传感器及通信就绪与数据时效门控

**优先级**： P0 / 严重
**状态**： resolved
**结项记录提交**： `917aa770`
**依赖**： LW-001

### 问题

`InitSerial()` 的返回值被忽略。没有 IMU 时 `GetState()` 提前返回，但 `RobotControl()` 仍推进 FSM 并发送命令。左右控制板任意一侧更新就被视为电机反馈已更新，且电机反馈与 IMU 数据都没有时效超时。ROS 启动使用固定两秒延迟，而非就绪条件。

### 证据

- `src/rl_sar/src/rl_real_LW.cpp:45-60`
- `src/rl_sar/src/rl_real_LW.cpp:219-240`
- `src/rl_sar/src/rl_real_LW.cpp:393-445`
- `src/rl_sar/library/thirdparty/robot_sdk/lfr/LW_sdk/LW_sdk.hpp:213-217`
- `src/rl_sar/library/thirdparty/robot_sdk/lfr/LW_sdk/LW_sdk.hpp:270-277`
- `src/rl_sar/launch/rl_real_LW.launch.py:27-34`

### 计划范围

- 任一串口失败时拒绝启动控制。
- 独立跟踪左右反馈时间戳和有效性。
- 跟踪 IMU 时间戳，校验四元数和陀螺仪数据时效。
- 所有必需输入均有效之前，保持电机禁能。
- 任一必需来源过期时进入故障安全路径。
- 以明确就绪条件替代或补充固定启动延迟。

### 验收标准

- 缺少 IMU、右控制板或左控制板中的任一项，均阻止电机使能。
- 运行中断开任一必需来源后，在规定超时内进入安全状态。
- 重连行为明确且有测试，不得悄然恢复运动。
- 启动日志准确指出缺失的就绪条件。

### 解决证据

- 解决日期：2026-07-29
- 任一工作循环启动前，两个串口都必须成功初始化；失败时指出受影响侧，尝试禁能所有可通信侧并终止启动。
- `LWSDK::RecvFdData()` 现分别报告左右有效帧更新。
- IMU 到达和每块控制板的有效反馈均使用独立单调时钟时间戳，以及可配置的 `sensor_timeout`（`0.1 s`）。
- 三个来源全部具有时效之前，控制循环保持禁能等待，准确报告缺失来源且不推进 FSM。
- 曾经达到就绪后，任一来源超时都会永久锁存命令门，在约 `100 ms` 内尽力发送 20 帧禁能命令，并请求 ROS 退出；重连不能清除锁存。
- 已移除固定两秒启动延迟，由真实传感器就绪条件控制激活。
- `test_sensor_readiness` 覆盖启动时各来源缺失、超时边界、运行时每个来源单独超时及重连后永久锁存。
- 验证命令与方法：
  - `cmake --build build/rl_sar --target test_sensor_readiness test_loop_lifecycle rl_real_LW -j2`
  - `ctest --test-dir build/rl_sar --output-on-failure -R '^(loop_lifecycle|sensor_readiness)$'`
  - `python3 -m py_compile src/rl_sar/launch/rl_real_LW.launch.py`
  - 对实机入口与就绪测试进行定向 `cppcheck` 检查
- 结果：`rl_real_LW` 构建成功；两个选定 CTest 均通过；`cppcheck` 未报告新的正确性警告。
- 按计划未运行硬件。
- 安全边界：在 `LW-003` 验证串口完整写入之前，主机侧禁能仍只是尽力执行。应对线缆断开、主机故障和掉电，仍需控制板本地通信看门狗与物理急停。

---

<a id="lw-003"></a>

## [LW-003] 串口解析与发送的稳健性

**优先级**： P0 / 严重
**状态**： resolved
**结项记录提交**： `8ac78e80`
**依赖**： LW-001

### 问题

RX 缓冲区超过 4096 字节后被清空，随即又用于无符号减法，形成越界路径。串口配置失败可能留下仍打开的非负文件描述符。TX 代码将任意正数 `write()` 返回值视为成功，且未一致处理部分写入、`EAGAIN` 或发送失败。

### 证据

- `src/rl_sar/library/thirdparty/robot_sdk/lfr/LW_sdk/LW_sdk.hpp:91-129`
- `src/rl_sar/library/thirdparty/robot_sdk/lfr/LW_sdk/LW_sdk.hpp:131-189`
- `src/rl_sar/library/thirdparty/robot_sdk/lfr/LW_sdk/LW_sdk.hpp:219-268`

### 计划范围

- 清空过大的 RX 缓冲区后立即返回或安全重新同步。
- 任一配置失败后关闭串口并使其描述符失效。
- 处理整包写入、部分写入、中断及非阻塞重试策略。
- 向调用方返回按控制板区分的结构化 RX/TX 状态。

### 验收标准

- 大于 4096 字节的损坏输入不能导致对空缓冲区的访问。
- 解析器测试覆盖噪声、分包、多包、CRC 错误和恢复。
- 仅当完整包已写入每块必需控制板时，命令才算成功。
- 串口设置失败不得留下看似可用的描述符。

### 解决证据

- 解决日期：2026-07-29
- 串口初始化先配置局部候选描述符，仅在全部必需设置成功后发布给 `LWSDK`。所有失败路径关闭候选描述符，返回失败操作及 `errno`；不支持低延迟 ioctl 仍仅记为非致命警告。
- 反馈使用有界流式解析器，保留不完整帧头及帧，逐步丢弃噪声，以 `memcpy` 复制包字节，校验帧尾和 CRC，解析所有可用帧，仅应用最新有效帧。
- 每次接收每侧最多读取 4096 字节，独立返回字节数、包数、丢弃、CRC、格式及读取错误状态。
- 命令发送逐侧跟踪偏移，重试部分写入与 `EINTR`，以 `poll()` 等待 `EAGAIN/EWOULDBLOCK`，在共享的可配置 `serial_write_timeout`（`0.002 s`）内服务两侧；仅当两个完整包均进入内核串口队列时成功。
- `RL_Real` 对启动禁能、等待态禁能、电机保护禁能、正常控制、紧急禁能及最终退出均检查结构化结果。正常发送失败在释放 `CommandGate` 后才处理，避免进入故障安全前出现递归加锁死锁。
- `test_lw_serial_sdk` 覆盖拆分帧头及帧、噪声、多帧、CRC 错误、帧尾错误、8192 字节损坏输入与恢复、PTY 双侧 RX/TX 与映射、重复配置失败无描述符泄漏、非阻塞部分写入、共享截止时间超时，以及一侧失败时继续向健康侧发送。
- 验证命令与方法：
  - `cmake --build build/rl_sar --target test_lw_serial_sdk test_sensor_readiness test_loop_lifecycle rl_real_LW rl_sim_LW -j2`
  - `ctest --test-dir build/rl_sar --output-on-failure --repeat until-fail:20 -R '^(lw_serial_sdk|sensor_readiness|loop_lifecycle)$'`
  - 独立使用 `-Wall -Wextra -Wpedantic` 编译
  - 独立运行 AddressSanitizer 和 UndefinedBehaviorSanitizer
  - 定向 `cppcheck` 检查
- 结果：两个 LW 可执行文件均构建成功；三项选定测试各连续 20 次通过；内存与未定义行为检测未发现故障；描述符泄漏回归通过；`cppcheck` 未报告新的正确性警告。
- 按计划未运行硬件。
- 安全边界：主机完整 `write()` 仅证明内核接受数据包，不证明控制器收到或执行。端到端保证仍需控制板应答、板内通信看门狗及物理急停。

---

<a id="lw-004"></a>

## [LW-004] 有限状态机转换的正确性

**优先级**： P0 / 严重
**状态**： resolved
**结项记录提交**： `15647aec`
**依赖**： LW-001

### 问题

两个形态转换状态都可能返回不存在的状态名 `RLFSMStateGetUp`。随后 `FSM::Run()` 调用 `states_.at(next)` 并抛出异常。当时实现的循环没有安全异常边界。被动模式帮助文本也与实际手柄映射不符。

### 证据

- `src/rl_sar/fsm_robot/fsm_LW.hpp:460-475`
- `src/rl_sar/fsm_robot/fsm_LW.hpp:542-556`
- `src/rl_sar/library/core/fsm/fsm.hpp:62-95`
- `src/rl_sar/fsm_robot/fsm_LW.hpp:15-44`

### 计划范围

- 将无效目标名称替换为预期的已注册状态名称。
- 在使用 `.at()` 前校验转换目标。
- 使显示给操作员的说明符合实际映射。
- 增加覆盖所有状态及其接受输入的转换表测试。

### 验收标准

- 所有 `CheckChange()` 返回结果都属于工厂注册状态集。
- 在每个状态按下所有受支持输入均不能抛出异常。
- 无效外部请求被拒绝，且不离开当前安全状态。

### 解决证据

- 解决日期：2026-07-29
- 从两种形态转换状态中移除了无效 `RLFSMStateGetUp` 分支。按用户明确要求，腿转轮和轮转腿期间忽略 `0/A`，不会以起身状态中断正在执行的动作。
- 形态转换仍接受 `P/LB_X` 进入被动模式、`9/B` 进入趴下；动作成功结束仍请求对应的已注册行走状态。
- `FSM::Run()` 现于进入转换模式前使用 `find()` 解析 `CheckChange()` 目标。未注册目标会被记录并拒绝，FSM 保持当前状态，因此转换路径不再通过 `unordered_map::at()` 抛出异常。
- 被动模式操作提示现符合实际轮式起身映射：键盘 `2` 或手柄 `Y`。
- `test_lw_fsm_transitions` 覆盖八个已注册 LW 状态、每个接受的键盘/手柄转换、两个输入枚举的所有值、形态转换期间忽略 `0/A`、工厂注册一致性，以及拒绝无效内部和外部目标。
- 验证命令与方法：
  - `cmake --build build/rl_sar --target test_lw_fsm_transitions rl_real_LW rl_sim_LW -j2`
  - `ctest --test-dir build/rl_sar --output-on-failure --repeat until-fail:20 -R '^(loop_lifecycle|sensor_readiness|lw_serial_sdk|lw_fsm_transitions)$'`
  - 对 FSM 核心、LW FSM 和转换测试进行定向 `cppcheck` 检查
- 结果：两个 LW 可执行文件构建成功；四项选定测试各连续 20 次通过；`cppcheck` 仅报告已有风格建议，无新的正确性警告。
- 按计划未运行硬件。

---

<a id="lw-005"></a>

## [LW-005] 命令有限值校验与主动保护

**优先级**： P0 / 严重
**状态**： resolved
**结项记录提交**： `a2b79e74`
**依赖**： LW-001, LW-002

### 问题

`TorqueProtect()` 仅打印警告。策略动作限幅为 ±100，经缩放后允许很大的位置与轮速目标。NaN 检查仅覆盖右腿动作目标，不拒绝无穷值或错误增益，且仍发送数据包。姿态保护存在延迟，也未覆盖全部动作状态。

### 证据

- `src/rl_sar/library/core/rl_sdk/rl_sdk.cpp:358-375`
- `src/rl_sar/library/core/rl_sdk/rl_sdk.cpp:386-436`
- `src/rl_sar/src/rl_real_LW.cpp:320-336`
- `src/rl_sar/src/rl_real_LW.cpp:447-479`
- `src/rl_sar/library/thirdparty/robot_sdk/lfr/LW_sdk/LW_sdk.hpp:244-267`
- `policy/LW/robot_lab/*/config.yaml`

### 计划范围

- 对 IMU 四元数与陀螺仪、电机反馈位置/速度/估计力矩、原始策略动作、计算后策略输出、最终机器人命令及底层串口命令字段，均要求通过 `std::isfinite()`。
- 拒绝负比例或微分增益，但不设最大增益。
- 无效策略帧在入队或发送前拒绝。
- 75 度横滚/俯仰故障安全仅用于趴下、两种行走及两种形态转换状态。
- 被动状态和两种起身状态不应用姿态保护。
- 保持现有 `[-100, 100]` 动作裁剪、目标位置/速度行为及仅警告的预测力矩保护不变。

### 验收标准

- 任一受校验控制阶段出现 NaN 或无穷值，触发永久故障安全路径。
- 负增益触发永久故障安全路径。
- 无效底层命令值不会写入任何串口。
- 横滚或俯仰超过 75 度时，仅在指定五个受保护状态触发故障安全；被动及两种起身状态不受姿态保护影响。
- 很大但有限的动作、目标、增益和预测力矩，不因幅值本身触发故障安全；原动作裁剪保持不变。

### 解决证据

- 解决日期：2026-07-29
- IMU 四元数/陀螺仪以及双侧电机位置、速度和估计力矩，在进入控制器前检查大小正确且数值有限。
- 原始模型动作在既有裁剪操作前校验；计算后的位置、速度和力矩输出在入队前校验；最终 `RobotCommand` 在打开命令门前校验。
- 串口 SDK 在构包前独立拒绝非有限动作/增益字段和负增益，返回双侧 `EINVAL`，不向任一串口写入字节。
- 姿态保护在控制前及 FSM 运行后再次判断，确保进入受保护状态后生成的首条命令不能绕过 75 度横滚/俯仰限制。仅对趴下、腿式行走、轮式行走、腿转轮和轮转腿启用保护。
- 按获准范围，被动和两种起身状态不启用姿态保护。不增加位置/速度范围、最大增益或预测力矩故障安全；保留 `[-100, 100]` 动作裁剪及仅警告的 `TorqueProtect()` 行为。
- `test_lw_control_safety` 覆盖有限反馈、动作、输出、最终命令、负增益、无幅值限制，以及准确的受保护和不受保护状态集合。
- `test_lw_serial_sdk` 验证 NaN、无穷值及负增益产生零串口字节，而很大的有限增益仍可发送。
- 验证命令与方法：
  - `cmake --build build/rl_sar --target rl_real_LW rl_sim_LW test_lw_control_safety test_lw_serial_sdk test_lw_fsm_transitions test_sensor_readiness test_loop_lifecycle -j2`
  - `ctest --test-dir build/rl_sar --output-on-failure --repeat until-fail:20 -R '^(lw_control_safety|lw_serial_sdk|lw_fsm_transitions|sensor_readiness|loop_lifecycle)$'`
  - 独立使用 `-Wall -Wextra -Wpedantic` 编译
  - 对串口 SDK 测试独立运行 AddressSanitizer 和 UndefinedBehaviorSanitizer
  - 定向 `cppcheck` 检查
- 结果：两个 LW 可执行文件构建成功；五项选定测试各连续 20 次通过；内存与未定义行为检测无故障；`cppcheck` 仅报告已有构造初始化性能建议。
- 按计划未运行硬件。

---

<a id="lw-006"></a>

## [LW-006] 手柄断联与输入校验

**优先级**： P0 / 严重
**状态**： resolved
**结项记录提交**： `a96dcf49`
**依赖**： LW-001, LW-002

### 问题

手柄读取无法区分无事件与断联。缓存轴值维持非零，无线手柄丢失可能无限保留最后的速度命令。按钮和轴编号作为数组索引使用，却没有边界检查。

### 证据

- `src/rl_sar/src/rl_real_LW.cpp:481-587`
- `src/rl_sar/library/thirdparty/joystick/joystick.cc:47-68`

### 计划范围

- 不增加持续按压的使能按钮，不改变现有按钮、轴、缩放或 5% 死区行为。
- 区分非阻塞设备空闲与 EOF、断联、部分读取及其他读取失败。
- 手柄不可用时永久锁存关闭手柄输入，清空所有缓存按钮和轴，并强制速度命令为零。
- 保持电机控制和当前 FSM 活跃，不关闭命令门、不禁能电机、不退出 ROS。
- 重连后不自动接受输入，恢复手柄输入须重启节点。
- 访问数组前校验按钮及轴索引。

### 验收标准

- `EAGAIN`、`EWOULDBLOCK` 和 `EINTR` 不会将空闲手柄误判为断联。
- EOF、无效描述符、设备错误及部分读取将手柄输入永久锁存为不可用。
- 锁存手柄故障清空缓存输入，并在控制路径强制 `x/y/yaw` 和 Gamepad 状态为零或无。
- 手柄丢失不发送电机禁能命令、不停止当前 FSM，也不退出 ROS。
- 已停止或缺失的手柄不能通过过期 Gamepad 状态触发起身、行走或形态转换。
- 越界事件编号被忽略并记录，不发生数组外内存访问。
- 原死区和有效输入行为不变。

### 解决证据

- 解决日期：2026-07-29
- LW 专用非阻塞手柄读取器现分别报告完整事件、无数据、EOF/断联及格式错误/读取错误，未修改第三方手柄子模块。
- LW 实机和仿真输入路径使用固定大小的按钮/轴数组，每次接收事件都在索引访问前校验。
- 启动时打开失败或运行时出现终止性读取结果，均清空全部缓存手柄状态，并永久锁存手柄输入不可用。后续调用不能清除锁存或恢复 Gamepad 命令。
- 200 Hz 控制路径在锁存期间清零 `x/y/yaw` 及两个 Gamepad 状态字段。策略路径独立发布零命令观测，保证下一推理帧不会复用之前的手柄速度。
- 按获准安全行为，手柄丢失不调用 `EnterFailSafe()`、不关闭 `CommandGate`、不发送电机禁能包，也不请求 ROS 退出。当前 FSM 和电机支撑保持运行。故障门不清空键盘状态，但当时的实机可执行文件不启动 `KeyboardInterface()`，因此终端键盘输入不能作为恢复路径。
- 原轴归一化、`vel_command` 缩放、输入映射和严格 5% 死区不变。
- `test_lw_joystick_safety` 覆盖空闲描述符、完整事件、EOF、无效描述符、空目标、部分读取、按钮/轴边界、5% 死区边界、完整缓存清空及永久故障锁存。
- 验证命令与方法：
  - `cmake --build build/rl_sar --target test_lw_joystick_safety rl_real_LW rl_sim_LW -j2`
  - `ctest --test-dir build/rl_sar --output-on-failure --repeat until-fail:20 -R '^(loop_lifecycle|sensor_readiness|lw_serial_sdk|lw_fsm_transitions|lw_control_safety|lw_joystick_safety)$'`
  - 独立使用 `-Wall -Wextra -Wpedantic` 编译
  - 独立运行 AddressSanitizer 和 UndefinedBehaviorSanitizer
  - 定向 `cppcheck` 检查
- 结果：两个 LW 可执行文件构建成功；六项选定测试各连续 20 次通过；内存与未定义行为检测无故障；`cppcheck` 仅报告已有的构造初始化性能建议。
- 按计划未运行硬件。
- 安全边界：无线链路丢失后，若接收器仍存在且不报告释放事件或设备/读取错误，仅凭 `/dev/input/js*` 不能证明手持控制器已断联。
- 安全边界：机器人站立时停止或重启实机程序会发送最终电机禁能命令，可能导致跌倒。手柄故障锁存后，退出前必须机械支撑机器人；LW-006 不提供进程内的受控 GetDown 路径。

---

<a id="lw-007"></a>

## [LW-007] 跨线程状态一致性

**优先级**： P1 / 高
**状态**： resolved
**结项记录提交**： `a665d4ec`
**依赖**： LW-001, LW-002

### 问题

策略线程复制 `robot_state` 时锁定 `state_mutex`，但控制线程写入时不持有该互斥锁。`control`、`params`、`rl_init_done`、`episode_length_buf`、绘图数据和 `motion_loader_lw` 也在缺乏一致同步策略的情况下共享。策略切换可能暴露仅部分更新的配置与状态。

### 证据

- `src/rl_sar/src/rl_real_LW.cpp:127-203`
- `src/rl_sar/src/rl_real_LW.cpp:257-341`
- `src/rl_sar/src/rl_real_LW.cpp:393-445`
- `src/rl_sar/fsm_robot/fsm_LW.hpp:242-558`
- `src/rl_sar/library/core/rl_sdk/rl_sdk.cpp:317-356`

### 计划范围

- 明确控制状态、机器人状态、活动策略配置及动作参考的所有权。
- 线程间发布不可变快照，或使用一致的锁与原子操作方案。
- 将策略激活与停用实现为原子状态转换。
- 确保调试发布读取一致快照。

### 验收标准

- 不得在未使用文档约定同步机制的情况下访问共享可变控制数据。
- 策略切换不能将旧观测与新配置或模型数据混用。
- ThreadSanitizer 或等效压力测试在 LW 控制路径中不报告数据竞争。

### 解决证据

- 解决日期：2026-07-29
- 200 Hz 控制线程现唯一拥有 `control`、FSM、`robot_state`、`robot_command` 及 LW 动作加载器。手柄线程通过受保护邮箱发布一致速度和带序号的 Gamepad 输入；仿真键盘读取也在控制线程中运行。
- 每个推理帧读取一份合并的机器人状态与控制快照。调试回调读取专用完整快照而非实时控制数据，仿真执行器网络使用代次匹配的推理快照。
- 四份策略 YAML 配置与预加载模型在工作线程启动前组装为只读策略定义。激活时原子发布定义、模型和动作长度上下文，并使用单调递增代次；推理线程整帧保持同一上下文，仅在代次变化时重置私有观测、历史和输出工作区。
- 形态参考与推理进度均为不可变且带代次标签的快照。拒绝其他代次的参考或进度值，保证策略帧不能混用新模型/配置和旧动作参考。
- 快照缓冲区复制到保留存储，而非每个控制周期转移临时向量所有权，从而减少 200 Hz 路径的内存分配抖动。
- `test_lw_runtime_sync` 对整帧发布、策略上下文原子替换和一致且有序的输入进行压力测试：一个写入者，适用时四个并发读取者，每个线程迭代 50,000 次。
- 验证命令与方法：
  - `cmake --build build/rl_sar --target rl_sdk test_lw_runtime_sync test_lw_fsm_transitions test_lw_control_safety test_lw_joystick_safety rl_real_LW rl_sim_LW -j2`
  - `ctest --test-dir build/rl_sar --output-on-failure -R 'loop_lifecycle|sensor_readiness|lw_serial_sdk|lw_fsm_transitions|lw_control_safety|lw_joystick_safety|lw_runtime_sync'`
  - `ctest --test-dir build/rl_sar --output-on-failure --repeat until-fail:20 -R 'lw_fsm_transitions|lw_runtime_sync'`
  - 额外连续独立运行 `test_lw_runtime_sync` 共 50 次
  - 独立运行 AddressSanitizer 和 UndefinedBehaviorSanitizer
  - 使用 `-Wall -Wextra -Wpedantic` 构建核心库、同步测试和两个 LW 可执行文件
- 结果：两个 LW 可执行文件及所有选定目标构建成功；七项选定 CTest 通过；FSM/同步测试各连续 20 次 CTest 通过；同步测试额外 50 次通过；内存与未定义行为检测无故障。严格警告输出仅含已有初始化、未使用参数及第三方警告。
- ThreadSanitizer 插桩编译成功，但其运行时不能在该容器启动（`ThreadSanitizer: unexpected memory mapping`）；因此以并发压力测试、重复运行及 ASan/UBSan 作为可用的等效验证。
- 按计划未运行硬件。
- 范围边界：现有位置、速度和力矩三个队列仍相互独立且没有版本标记。完整推理帧内部绑定同一代次，但过期或交叉配对的队列消费仍归属 LW-008，本项按计划未修改。

---

<a id="lw-008"></a>

## [LW-008] 策略输出的原子传递

**优先级**： P1 / 高
**状态**： resolved
**结项记录提交**： `633ca283`
**依赖**： LW-007

### 问题

位置、速度和力矩分别推入独立队列。消费端可能在速度可用前弹出位置，因短路求值将其丢弃，随后配对来自不同推理帧的输出。策略切换期间队列既不清空，也没有版本标记。

### 证据

- `src/rl_sar/src/rl_real_LW.cpp:320-333`
- `src/rl_sar/library/core/rl_sdk/rl_sdk.hpp:205-214`
- `src/rl_sar/library/core/rl_sdk/rl_sdk.cpp:693-713`

### 计划范围

- 将分离队列替换为包含位置、速度、力矩、时间戳、序号及策略代次的整帧。
- 实时控制优先采用明确的最新帧语义。
- 拒绝过期帧及不属于活动策略代次的帧。

### 验收标准

- 消费端不可能观察到部分策略输出。
- 策略转换不能消费前一策略的帧。
- 推理延迟时对过期帧的响应有明确文档。

### 解决证据

- 解决日期：2026-07-31
- LW 策略推理现将位置、速度、力矩、源时间戳、全局序号、推理帧及策略代次发布为一份不可变的最新帧快照。实机和仿真生产端不再通过三个独立队列发布 LW 输出。
- 发布时拒绝不完整载荷及非活动代次。激活与停用会清空最新帧槽位，消费端应用帧前按当前活动代次校验。策略切换竞争可能留下仍可见的旧帧，但其代次不能通过消费端检查，因此不能形成控制命令。
- 四个 LW FSM 状态和仿真执行器网络路径均消费同一份一致帧。通用、基于队列的 `RLControl()` 路径继续供非 LW 机器人使用，并有兼容性回归测试。
- 从推理开始前一刻起，以 `steady_clock` 衡量数据时效。允许的最大帧龄为三个策略周期（`3 * dt * decimation`，当时为 60 ms）。恰好在边界时帧仍有效；超过边界后消费端保持最后已应用命令与增益，限频警告，并在下一份完整且未过期的帧到达时自动恢复。不会仅因推理输出延迟而切换被动模式、禁能电机或退出 ROS。
- `test_lw_policy_output_transport` 覆盖部分帧及非活动代次拒绝、最新帧覆盖、代次清空、时效边界、并发读取和非 LW 队列兼容性。
- 验证命令与方法：
  - 构建 `rl_sdk`、`rl_real_LW`、`rl_sim_LW`、`test_lw_policy_output_transport` 及已有 LW 测试目标
  - 全部八项选定 CTest：`loop_lifecycle`、`sensor_readiness`、`lw_serial_sdk`、`lw_fsm_transitions`、`lw_control_safety`、`lw_joystick_safety`、`lw_runtime_sync` 和 `lw_policy_output_transport`
  - `lw_fsm_transitions`、`lw_runtime_sync` 和 `lw_policy_output_transport` 各连续运行 20 次 CTest
  - 对 `test_lw_policy_output_transport` 运行 AddressSanitizer 和 UndefinedBehaviorSanitizer
  - 使用 `-Wall -Wextra -Wpedantic` 构建核心库、新传输测试和两个 LW 可执行文件；仅报告已有初始化、未使用参数和第三方警告
- 按计划未运行硬件。

---

<a id="lw-009"></a>

## [LW-009] 轮转腿动作参考的更新频率

**优先级**： P1 / 高
**状态**： resolved
**结项记录提交**： `923182db`
**依赖**： LW-007, LW-008

### 问题

`wheel_to_leg/config.yaml` 指定 `motion_fps: 60.0`，但 FSM 按 `1 / (dt * decimation)` 创建动作加载器，当时为 50 Hz。这会拉长动作参考时长，并改变计算所得的参考速度。

### 证据

- `src/rl_sar/fsm_robot/fsm_LW.hpp:478-535`
- `policy/LW/robot_lab/wheel_to_leg/config.yaml:62-65`
- `src/rl_sar/library/core/motion_loader/motion_loader_lw.cpp:5-15`

### 计划范围

- 两个转换方向均一致使用配置的动作文件帧率。
- 将控制器推理频率与动作文件采样频率分开。
- 按训练与导出约定验证插值和速度缩放。

### 验收标准

- 两个转换加载器均报告配置的 60 Hz 源频率。
- 参考时长与速度符合 CSV 离线计算。
- 回归测试比较选定时间戳与预期插值。

### 解决证据

- 解决日期：2026-07-31
- 两个形态转换状态现从不可变策略配置的 `motion_fps` 字段获取动作源频率。轮转腿不再从独立的 20 ms 策略推理周期推导 50 Hz 源频率。
- 策略推理仍为 50 Hz。`MotionLoaderLW` 继续在每个推理时间戳对配置的 60 Hz 动作源插值，因此不改变控制循环或推理调度。
- 离线验证得到腿转轮 167 帧、轮转腿 170 帧。按当时加载器约定，配置的 60 Hz 时长分别为 2.78333 s 和 2.83333 s；此前轮转腿被拉长到 3.4 s。关节参考速度现使用 `(q[n+1] - q[n]) * 60`，不再按 50 Hz 缩放。
- `test_lw_motion_reference_rate` 验证源 FPS 独立于策略 `dt * decimation`、两份 YAML 均选择 60 Hz、CSV 帧数和宽度符合预期、时长及前向差分速度与离线计算一致、中点插值正确且可到达末帧。
- 验证命令与方法：
  - 构建 `test_lw_motion_reference_rate`、`test_lw_fsm_transitions`、`rl_real_LW` 和 `rl_sim_LW`
  - 全部九项选定 CTest：`loop_lifecycle`、`sensor_readiness`、`lw_serial_sdk`、`lw_fsm_transitions`、`lw_control_safety`、`lw_joystick_safety`、`lw_runtime_sync`、`lw_policy_output_transport` 和 `lw_motion_reference_rate`
  - `lw_fsm_transitions` 和 `lw_motion_reference_rate` 各连续运行 20 次 CTest
  - 使用 `-Wall -Wextra -Wpedantic` 构建新测试和两个 LW 可执行文件；仅报告已有初始化、未使用参数及第三方警告
  - 对新测试进行定向 `cppcheck` 检查，无发现
- 按计划未运行硬件。
- 范围边界：`MotionLoaderLW` 仍沿用现有 `num_frames * dt` 时长约定。定义并修正该约定归属 LW-012，本项按计划未修改。

---

<a id="lw-010"></a>

## [LW-010] 可复现的部署产物

**优先级**： P1 / 高
**状态**： resolved
**结项记录提交**： `8272253f`
**依赖**： LW-004, LW-005, LW-009

### 问题

已安装启动器指向的可执行文件早于当前源码。可执行文件嵌入工作区绝对路径 `POLICY_DIR`，因此加载的是工作树内可变模型，而非有版本的已安装策略集合。当前 LW 模型文件也有修改或未跟踪状态。

### 证据

- `src/rl_sar/CMakeLists.txt:42-46`
- `src/rl_sar/CMakeLists.txt:627-655`
- `src/rl_sar/CMakeLists.txt:672-678`
- 文件系统与构建状态记录在上文审查基线中。

### 计划范围

- 通过已安装软件包共享目录或明确部署包解析策略。
- 在干净工作树中构建，记录源码提交、配置哈希和模型哈希。
- 预期部署包不完整或不匹配时，启动失败。
- 确保启动文件选择新构建的目标。

### 验收标准

- 部署清单标识二进制、源码提交、YAML、ONNX 和 CSV 哈希。
- 移动已安装工作区不影响策略查找。
- 有未提交修改或未跟踪的生产模型文件不会被悄然加载。
- 启动的二进制包含全部获准源码修改。

### 解决证据

- 解决日期：2026-08-04
- 实现提交：
  - `8803e6d` 增加经验证的部署包、获准 ONNX 模型、清单生成器、策略根目录隔离、失败时拒绝启动以及测试。
  - `8d18532` 在干净临时工作树中初始化固定版本的 Git 子模块。
  - `2cf311a` 增加仅验证的启动路径，执行相同部署检查，但不初始化 ROS、手柄、串口或控制线程。
- `build_lw_deployment.sh` 在干净的分离工作树中以 `Release` 模式构建提交 `2cf311adf55f3a2afffd4c18751cc11877a218de`，并将无符号链接的部署包安装到 `build/lw010_release_2cf311a/`。
- 生成清单记录源码提交、已安装 `rl_real_LW` 的 SHA-256 `5d7d92057fa6f1aced0e0f71d9a383754e42a5bcbc25d71c0b01ee2c72e0a667`，以及五份 YAML 文件、四个 ONNX 模型和两份转换动作 CSV 文件的哈希。
- 已安装可执行文件和部署树均无符号链接。实机可执行文件使用明确且已验证的策略根目录，不再包含 `/home/lfr/rl_sar/policy`。
- 完整安装前缀复制到 `build/lw010_relocated_2cf311a/`。加载重定位后的环境后，`ros2 pkg prefix rl_sar` 解析到该目录；`rl_real_LW --verify-deployment-only` 通过全部二进制、源码提交、清单、路径包含关系及资源哈希检查。
- `test_lw_deployment_bundle` 覆盖合法和重定位部署包、资源缺失或篡改、可执行文件篡改、源码提交不匹配、符号链接和非规范路径。Python 生成器测试覆盖完整清单、拒绝符号链接、非 Release 构建及无效提交。
- `rl_real_LW` 构建成功，并通过 11 项选定 LW CTest：`loop_lifecycle`、`sensor_readiness`、`lw_serial_sdk`、`lw_deployment_bundle`、`lw_deployment_manifest_generator`、`lw_fsm_transitions`、`lw_control_safety`、`lw_joystick_safety`、`lw_runtime_sync`、`lw_policy_output_transport` 和 `lw_motion_reference_rate`。
- 定向 `-Wall -Wextra -Wpedantic -Werror`、AddressSanitizer、UndefinedBehaviorSanitizer、`cppcheck`、Python 语法与测试、shell 语法及 `git diff --check` 均通过。
- 按计划未运行硬件。
- 范围边界：清单绑定已安装可执行文件及本项所需的全部 LW 策略与配置产物。兼容的外部 ROS、推理运行库、Python 和操作系统共享库仍是部署前提，本清单不将其纳入仓库或计算哈希。

---

<a id="lw-016"></a>

## [LW-016] 安全动作适度性与恢复机制审查

**优先级**： P1 / 高
**状态**： resolved
**依赖**： LW-001, LW-002, LW-003, LW-005, LW-006, LW-011

### 问题

LW 安全行为分散积累在循环异常、传感器时效、串口交付、反馈和命令校验、姿态限制、电机保护、手柄丢失、时序降级、启动及退出路径中。目前没有统一决策矩阵证明每种触发条件都使用了严重程度和恢复策略合适的动作。

`EnterFailSafe()` 当时会锁存关闭命令门，在约 100 ms 内发送 20 个硬禁能包，并请求 ROS 退出。只有立即撤去力矩比维持受控状态更安全时，这才合适。若用于瞬态或可恢复条件，同样的行为可能使站立机器人跌倒，或剥夺操作员请求 `GetDown` 的机会。反之，弱化确属致命故障的响应又可能让不安全命令继续生效。因此必须基于证据审查完整边界与动作适度性，而非一概放宽。

### 证据

- `src/rl_sar/src/rl_real_LW.cpp:492-746`
- `src/rl_sar/src/rl_real_LW.cpp:776-787`
- `src/rl_sar/src/rl_real_LW.cpp:1298-1344`
- `src/rl_sar/library/core/loop/command_gate.hpp`
- `src/rl_sar/library/core/safety/lw_control_safety.hpp`
- `src/rl_sar/library/core/safety/lw_joystick_safety.hpp`
- `LW-001` 至 `LW-006` 及 `LW-011` 的解决证据与已记录安全边界。

### 计划范围

- 盘点所有警告、速度清零、输入源锁存、状态转换请求、阻尼或禁能命令发送、`CommandGate` 关闭、`EnterFailSafe()` 调用及 ROS 退出路径。
- 对每种触发条件记录原始故障、机器人/FSM 状态、持续或去抖规则、硬件假设、所选动作、操作员选项、重启要求，以及采取和不采取动作的最坏可信后果。
- 明确定义动作层级，例如仅诊断、有界命令削减、速度清零锁存并允许操作员请求 `GetDown`、受控被动/阻尼，以及立即硬禁能并退出。不能假定每一层级在每个状态都安全。
- 审查阈值、滞回、瞬态处理、重试上限、锁存永久性和恢复权限。可恢复事件不得在没有记录理由的情况下悄然升级为不可逆动作。
- 为每条硬禁能路径提供具体物理安全依据，说明为何在相关状态下，立即撤去力矩比保持支撑或允许受控下降更安全。
- 将主机保证与电机板看门狗、应答、电源、线缆断开、物理急停及机械支撑假设分开。
- 为获准决策矩阵增加定向测试。任何行为改变仍限于 `LW-016`，实施前须按常规获得明确代码审批。

### 验收标准

- 审查后的决策矩阵覆盖每项生产 LW 安全触发条件和每种终止性安全动作；任何调用点都不得保留隐含的严重程度或恢复语义。
- 每个 `EnterFailSafe()` 或硬禁能调用点均有证据表明：对其故障及适用 FSM 状态，立即撤去力矩是可用响应中危险最小的。
- 瞬态或降级条件使用的动作不得强于获准矩阵，锁存不得更久，操作员干预不得更多。
- 保留 FSM 输入的情形须明确 `GetDown` 何时仍安全且可用；状态或命令数据损坏时，不得宣称无法保证的受控恢复。
- 日志区分降级、可恢复、锁存和致命状态，并说明是否需要重启、`GetDown`、硬件禁能或物理支撑。
- 自动化测试强制触发每项条件，验证命令门状态、电机包顺序、ROS 退出行为、输入保留或清空、锁存持续性及允许的恢复动作，不弱化无关保护。
- 部署文档说明尚需成立的控制板看门狗与物理急停假设，并记录仍需悬吊机器人验证的行为。

### 解决证据

- 解决时间：2026-08-09T11:32:37+08:00
- 提交：`6853b6cb`
- 新增 `lw_safety_policy.hpp`，作为 27 种运行时与生命周期事件的唯一生产决策矩阵。明确分配 S0 诊断、S1 输入降级、S2 受控阻尼、S3 硬禁能、S4 硬禁能并退出 ROS、启动中止及有序退出动作。监督器单调锁存严重程度，并去重重复诊断事件。
- 手柄丢失和手柄循环异常保留电机/FSM 运行，仅抑制输入。控制时序降级保留获准的速度清零锁存及操作员请求 `GetDown` 的路径。
- 推理循环异常、无效策略动作/输出/配置，以及缺失、不完整、代次不匹配、序号回退或过期的策略输出，现均永久停止接受策略，应用 `Kp=0`、`Kd=5`、速度和力矩为零的 Passive 阻尼命令。控制线程发现过期输出时，在当周期覆盖旧命令；其他 S2 事件最迟在下一个可执行控制周期应用。FSM 转换仅由控制线程请求并完成。
- 电机板硬件故障现触发一次性 S3 硬禁能锁存，ROS 保持运行以供诊断。控制或未知循环异常、显式启用的致命时序阈值、过期或无效反馈、缺失 FSM 状态、受保护状态姿态越限、无效或空最终命令、读取失败及不完整发送仍属 S4。硬禁能与 ROS 退出锁存相互独立，后续 S4 事件仍可升级现有 S3 锁存。
- 启动串口、初始禁能及循环启动失败，仍尽力禁能后中止启动。正常退出顺序不变。只要持续收到有效且未过期的帧，解析错误与力矩限值报告仍仅作诊断；反馈时效是升级边界。
- 新增 `test_lw_safety_policy`，扩展 `test_lw_control_safety` 与 `test_lw_policy_output_transport`，覆盖完整矩阵顺序和动作、按来源区分的循环处理、单调升级、重复事件去重、发送失败升级、正常退出、Passive 阻尼内容，以及有界初始等待与过期输出行为。
- Debug 验证构建了 `rl_real_LW` 和 `rl_sim_LW`，全部 13 项已注册 CTest 通过。全新 Release 配置构建两个可执行文件，三项直接受影响测试通过；这三项也通过 UndefinedBehaviorSanitizer。
- 纯安全策略测试通过 AddressSanitizer 与 UndefinedBehaviorSanitizer 联合检测。不宣称完整 `rl_sdk` 的 ASan 运行通过：本机链接测试间歇性无限占用一个 CPU，终止时 ASan 递归打印 `DEADLYSIGNAL`，没有可用应用堆栈。普通、Release、UBSan 及 50,000 帧并发运行全部通过。
- 仅将仓库原有 `reorder`、聚合初始化及未使用参数警告类别保留为警告后，新测试、受影响测试、核心 SDK 和 `rl_real_LW` 通过严格 `-Wall -Wextra -Wpedantic -Werror` 构建。定向 `cppcheck` 仅发现已有 `CSVInit(std::string)` 性能建议和构造/线程绑定启发式提示，无新正确性发现。`git diff --check` 通过。
- `docs/LW_BUILD_DEPLOYMENT_CN.md` 以操作员语言说明 S0-S4 行为，明确区分主机包发送保证与电机应答、板内看门狗、物理急停、机械支撑、Passive 阻尼和 75 度姿态假设。
- 按计划未运行硬件。首次落地实验前，S1-S4 故障注入、`motors_disable` 应答与延迟、串口丢失后的看门狗行为、`Kd=5` Passive 阻尼及 75 度 S4 响应，仍需在物理急停、支撑和隔离距离就绪的情况下进行悬吊验证。

---

<a id="lw-013"></a>

## [LW-013] 配置与维度校验

**优先级**： P1 / 高
**状态**： resolved
**结项记录提交**： `6853b6cb`
**依赖**： LW-005, LW-010

### 问题

控制路径假定每个 YAML 向量长度正确、每个映射和索引有效。控制启动前，未显式按配置检查模型输入输出大小。`InitObservations()` 还将 `w,x,y,z` 四元数初始化为 `{0,0,0,1}`，而非单位四元数 `{1,0,0,0}`。

### 证据

- `src/rl_sar/library/core/rl_sdk/rl_sdk.cpp:221-235`
- `src/rl_sar/library/core/rl_sdk/rl_sdk.cpp:317-375`
- `src/rl_sar/src/rl_real_LW.cpp:127-203`
- `src/rl_sar/src/rl_real_LW.cpp:447-479`
- `policy/LW/base.yaml`
- `policy/LW/robot_lab/*/config.yaml`

### 计划范围

- 校验必需键、向量长度、映射唯一性、索引范围、正时序值和有限限值。
- 按计算得到的观测与动作维度验证 ONNX 输入输出形状。
- 修正单位四元数初始化。

### 验收标准

- 无效配置在串口命令循环启动前失败。
- 当前全部 LW 配置通过独立校验测试。
- 模型维度不匹配产生明确启动错误。

### 解决证据

- 解决日期：2026-08-09
- 新增共享 LW 配置校验器，在 `rl_real_LW` 和 `rl_sim_LW` 中均先于策略预加载、串口初始化及工作循环启动运行。缺失键、类型错误、无效时序、非有限或大小错误向量、重复/越界映射、不支持观测项、无效历史设置和不一致动作限值，均以配置路径和具体字段原因报错。
- 按实际观测列表计算策略观测维度，并与 `num_observations` 对照；历史选择决定真实模型输入大小。四个正式 LW 策略验证为 `410 -> 10`、`195 -> 10`、`59 -> 10` 和 `59 -> 10`。
- ONNX Runtime 现暴露不可变输入输出张量元数据。LW 预加载要求各一个 float32 二阶输入及输出，接受批次大小 1 或动态批次，检查特征维度，并在两次预热推理中验证输出长度与有限性。配置或模型失败现在抛出异常，不再仅记录后忽略。
- 将共享 `w,x,y,z` 观测四元数的单位初始化从 `{0,0,0,1}` 修正为 `{1,0,0,0}`。
- `test_lw_configuration_validation` 覆盖全部当前 YAML 和正式 ONNX 模型、必需字段、向量大小、有限值、映射唯一性和边界、观测名称与维度、历史设置、模型维度不匹配及四元数初始化。
- 验证命令与方法：
  - 在已有 Debug 构建中编译 `test_lw_configuration_validation`、`rl_real_LW` 和 `rl_sim_LW`；
  - 完整 Debug CTest **14/14** 通过；
  - 全新 Debug 和 Release 构建均编译 `test_lw_configuration_validation`、`rl_real_LW` 和 `rl_sim_LW`；
  - 全新 Debug 和 Release 的 `lw_configuration_validation` CTest 各 **1/1** 通过；
  - 定向 `cppcheck` 检查校验器及测试，无警告；
  - `git diff --check`。
- 未启动串口设备、仿真界面或实机。

---

<a id="lw-011"></a>

## [LW-011] 确定性的控制循环时序

**优先级**： P1 / 高
**状态**： resolved
**依赖**： LW-001, LW-007

### 问题

LW-010 已保证 Release 生产部署包，但循环计时仍将回调时长截断为整数毫秒并使用相对休眠。回调没有明确实时调度策略，未测量或处理截止时间违约，行走和转换状态还在 200 Hz 控制路径中进行终端输入输出及刷新。

### 证据

- `src/rl_sar/CMakeLists.txt:34-40`
- `src/rl_sar/library/core/loop/loop.hpp:105-135`
- `src/rl_sar/library/core/logger/logger.hpp:47-92`
- `src/rl_sar/fsm_robot/fsm_LW.hpp:291-295`
- `src/rl_sar/fsm_robot/fsm_LW.hpp:371-375`
- `src/rl_sar/fsm_robot/fsm_LW.hpp:464-482`
- `src/rl_sar/fsm_robot/fsm_LW.hpp:570-588`

### 计划范围

- 保留 LW-010 的 Release 生产构建保证。
- 以高分辨率时长按绝对截止时间调度。
- 定义超时检测与安全超时策略。
- 将终端输入输出及进度格式化移出控制线程。
- 决定并记录 CPU 亲和性与实时优先级要求。

### 验收标准

- 生产构建使用获准的优化配置。
- 时序测试报告控制及推理抖动、最大延迟和截止时间违约。
- 阻塞终端或 ROS 调试输入输出不能延迟电机命令生成。

### 解决证据

- 解决时间：2026-08-04T17:35:45+08:00
- 提交：`9978230c`
- 批准范围：高分辨率绝对时间调度，不突发补跑；时序指标与可配置 CPU 亲和性/`SCHED_FIFO`；连续跳过三个周期或一次截止时间晚到 20 ms 后，软降级为速度清零；默认关闭致命时序故障安全；实机与 Sim2Sim 的周期 LW 终端输出从控制回调移到 ROS 定时器。
- 修改文件：`policy/LW/base.yaml`、`docs/LW_BUILD_DEPLOYMENT_CN.md`、`src/rl_sar/CMakeLists.txt`、`src/rl_sar/fsm_robot/fsm_LW.hpp`、两套 LW 头文件和源码、`library/core/loop/loop.hpp`、`rl_sdk.hpp/.cpp`、`lw_runtime_sync.hpp`、`test_loop_timing.cpp` 及 `test_lw_runtime_sync.cpp`。
- `LoopFunc` 现按 `steady_clock` 绝对截止时间调度，跳过过期周期而非补跑回调，暴露唤醒/截止时间/执行统计，首次回调前校验调度设置，支持可选亲和性和 `SCHED_FIFO`，并明确必须成功与允许回退的行为。
- LW 实机控制在获准降级阈值下将 `x/y/yaw` 锁存为零，同时保留 FSM 按钮事件，以便操作员请求 `GetDown`。只有有意配置非零致命阈值后，时序触发的 `EnterFailSafe()` 才会启用。
- 周期 FSM 状态通过一致、非阻塞的单写入者邮箱传递。实机和 Sim2Sim 的 ROS 定时器在 200 Hz 回调外进行格式化及终端输入输出；实机控制还报告每秒时序汇总、调度回退与降级警告。
- 在 `build/lw011_release_current` 全新 Release 配置中验证：`rl_real_LW` 和 `rl_sim_LW` 均构建成功，全部 12 项已注册测试通过。新 `loop_timing` 测试连续 10 次通过，报告控制/推理平均及最大唤醒晚到、最大截止时间晚到、最大执行时间、截止时间违约数和跳过周期数。
- `test_loop_timing` 独立 `-Wall -Wextra -Wpedantic` 编译及 AddressSanitizer、UndefinedBehaviorSanitizer 运行通过。定向 `cppcheck` 仅报告已有 `CSVInit(std::string)` 按值传参性能建议。
- 按计划未运行硬件，也未调整部署主机的 `SCHED_FIFO` 或 CPU 亲和性。在目标主机测量和控制板看门狗行为完成审查前，致命时序阈值保持为零。

---

<a id="lw-012"></a>

## [LW-012] 动作加载器的稳健性与时间约定

**优先级**： P2 / 中
**状态**： resolved
**依赖**： LW-009

### 问题

动作时长按 `num_frames * dt` 计算，但首帧至末帧的间隔通常为 `(num_frames - 1) * dt`。单帧文件会解引用空速度向量，CSV 行长度不一致也未在按索引访问前拒绝。

### 证据

- `src/rl_sar/library/core/motion_loader/motion_loader_lw.cpp:5-15`
- `src/rl_sar/library/core/motion_loader/motion_loader_lw.cpp:18-79`
- `src/rl_sar/library/core/motion_loader/motion_loader_lw.cpp:81-118`

### 计划范围

- 明确定义帧时间约定。
- 拒绝空、单帧、含非有限值或行宽不一致的动作数据。
- 按约定计算速度和时长。

### 验收标准

- 时长测试覆盖 0、1、2 及多帧。
- 格式错误的 CSV 在任何索引访问前失败。
- 验证首帧、中间及末帧时间戳插值。

### 解决证据

- 解决时间：2026-08-09T15:53:18+08:00
- 提交：`190456dd`（基线 `6853b6c`）
- 加载器现使用显式非负 `motion_time_offset_frames` 约定。CSV 第 `i` 行采样时刻为 `(motion_time_offset_frames + i) / motion_fps`；两份当前转换配置均设偏移 `1`，因为导出器移除了原 `t=0` 行。未来保留该行的 CSV 可选择偏移 `0`，无需再修改加载器。
- 早于首个保留采样时刻时保持首行，晚于末采样时刻时保持末行；插值与前向差分速度均使用配置动作频率。FSM 完成判定现使用 `motion_time >= motion_length`。
- 启动校验拒绝无效帧率、偏移、关节数参数，空或单行文件、空白或格式错误字段、非有限值、行宽不一致、非预期关节数及无效四元数。加载后的四元数归一化，并预分配速度存储。
- `test_lw_motion_loader` 覆盖偏移 `0` 与 `1`、时长、首个/精确/中间/最终时间戳、边界保持、速度、四元数归一化及格式错误输入。参考频率与配置测试覆盖两组正式 60 Hz CSV/配置及新字段。
- 已有构建的定向 CTest **3/3** 通过；完整套件 **15/15** 通过。
- 全新 Debug 和 Release 配置分别位于 `build/lw012_debug_clean` 与 `build/lw012_release_clean`，均构建 `test_lw_motion_loader`、`test_lw_motion_reference_rate`、`test_lw_configuration_validation`、`rl_real_LW` 和 `rl_sim_LW`；三项选定测试在两种配置下各 **3/3** 通过。
- `git diff --check` 通过。定向 `cppcheck` 无正确性警告，仅一项非阻塞的测试夹具初始化列表性能建议。
- 未启动串口设备、仿真界面或实机。

---

<a id="lw-015"></a>

## [LW-015] 仅维护 LW 的仓库范围与未来机器人扩展边界

**优先级**： P2 / 中
**状态**： resolved
**结项记录提交**： `241f7d69`
**依赖**： LW-010

### 问题

当前仓库仍携带 11 种非 LW 机器人的生产策略、11 个非 LW FSM 头文件、5 个非 LW 实机入口及多个厂商 SDK/子模块。若干非 LW SDK 即便可执行目标已禁用，仍由 CMake 配置。这增加了仅维护 LW 产品的依赖、构建、审查及维护范围。
`src/rl_sar_zoo/` 还是约 588 MB 的嵌套 Git 仓库，含 LW 及另外 11 种机器人描述目录。其 LW MJCF/地形存在当前 Sim2Sim 流程所需的本地改动，却未被父仓库跟踪。盲目删除或直接在父仓库暂存，可能丢失改动、记录不可用的嵌入仓库，或移除未来机器人扩展所需的通用仿真与核心设施。

### 证据

- `policy/{a1,b2,b2w,g1,go2,go2w,gr1t1,gr1t2,l4w4,lite3,tita}/`
- `src/rl_sar/fsm_robot/fsm_*.hpp`
- `src/rl_sar/src/rl_real_{a1,g1,go2,l4w4,lite3}.cpp`
- `src/rl_sar/library/thirdparty/robot_sdk/{unitree,deeprobotics,zhinao}/`
- `.gitmodules`
- `src/rl_sar/CMakeLists.txt:355-422`
- `README.md` 和 `README_CN.md`
- `scripts/download_robot_descriptions.sh`
- `src/rl_sar_zoo/`，包括其嵌套 Git 元数据和 LW 本地改动

### 计划范围

- 形成明确的受跟踪清单，将机器人相关路径分类为删除、保留为共享基础设施、为 LW 保留或外部/用户所有。
- 删除已跟踪非 LW 策略、FSM 实现、实机入口、厂商 SDK/子模块、停用 CMake 目标及过时构建/下载和文档引用。
- 保留 LW 策略/FSM/SDK、通用 RL 运行时、循环与推理基础设施、FSM 工厂/注册机制，以及未来机器人集成所需的通用仿真设施。
- 将 `fsm_all.hpp` 缩减为当前 LW 注册，同时保留清晰、有文档的未来机器人注册扩展点。
- 将 `src/rl_sar_zoo/` 纳入获准清单。保留当前 `LW_description` MJCF 改动和新地形资源，移除非 LW 描述及不属于获准 LW 仿真输入的无关编辑器/下载产物。
- 暂存 zoo 前，选定并记录一种有效父仓库集成策略：清理嵌套 Git 元数据后以普通文件纳入 LW 描述；或配置正确固定版本的子模块，其提交含获准 LW 改动。不得创建无人管理的嵌入仓库 gitlink。
- 为保留的 LW 描述资源保存上游来源及适用许可和版本信息。
- 保持当前 LW Sim2Sim 和生产部署流程可用。
- 更新 `scripts/download_robot_descriptions.sh` 及构建行为，让干净检出准确取得已跟踪或固定版本的 LW 描述，不悄然下载原多机器人资源库。
- 更新主文档，说明当前受支持机器人为 LW，同时保留未来新增机器人的简要指南。
- 不将通用核心重新设计或 LW 运行行为变更混入本次仓库范围清理。

### 验收标准

- 删除前经批准的清单标识每个需删除或保留的已跟踪路径与子模块，包括 zoo 的每个顶层条目及当前修改/未跟踪 LW 资源。
- 不残留对已删除机器人或厂商 SDK 的活动构建/运行引用，显式历史、许可或未来扩展说明除外。
- 干净父仓库检出可复现获准 `src/rl_sar_zoo/LW_description`，不依赖无人管理的嵌套 Git 仓库、手工下载，也不缺失本地 MJCF/地形改动。
- `src/rl_sar_zoo/` 中不再有非 LW 描述目录。
- 干净 ROS 2 开发构建生成 `rl_sim_LW`，LW Sim2Sim 启动路径可用。
- 干净生产构建生成 `rl_real_LW`，部署清单及 `--verify-deployment-only` 验收通过。
- 删除后所有现有 LW 单元和回归测试通过。
- 文档明确未来新增机器人所需的有限文件与接口，不要求恢复无关旧实现。
- 无关且用户所有的 `.agents/skills/inspect-context-compactions/` 目录保持原样，不进入清理提交；所有获准 LW zoo 文件均明确跟踪或固定版本，不悄然遗漏。

### 解决证据

- **解决日期**：2026-08-09
- 用户在任何清理前批准了仅 LW 清单及资源纳入仓库方案。已移除非 LW 策略、FSM、实机适配器、SDK/子模块、示例、停用构建接线及旧多机器人资源下载器。通用 FSM、推理、循环、控制、MuJoCo、手柄、消息和控制器扩展基础设施仍记录于 `README.md`，`test_lw_repository_scope.py` 持续约束保留的仅 LW 边界。
- `src/rl_sar_zoo/LW_description` 现为父仓库普通内容，基于 zoo 提交 `349d14a700ecf248b3cdbec5e7bac30882b66e62`，已记录上游与许可证来源。22 文件 SHA-256 清单通过，无嵌套 Git 元数据或非 LW 描述残留；原有五份 LW MJCF/地形改动与清理前备份逐字节一致。地形 PNG SHA-256 为 `6ffaba9cff2aa28640a72c578e9fbf1cea8a878b8cc26eafcccf985d78d75a2a`。
- 清理前完整 zoo Git 包与 LW 工作树归档已在 `/tmp/lw015-zoo-backup-aTLmdg/` 验证；SHA-256 分别为 `90dd2a285b47179378a30e68b1785d44edcd6e8d01fd63dbbf868ea0dbbb10d1` 和 `dd8d5a4ffbbdc9f7aa28ab5d6ffe7cc5328d319845b0e8d749498fda18058231`。
- 描述校验器在工作树及分离的干净检出中通过。反例拒绝清单文件改动、非 LW 描述和嵌套 Git 元数据。独立 `lw_description` 软件包可配置、构建并安装 MJCF、地形、网格及 URDF 资源。
- 已有构建及全新 Debug/Release 构建均通过全部 17 项 CTest。另一个分离干净检出的 Debug ROS 2 构建生成 `lw_description`、`rl_sim_LW` 和 `rl_real_LW`，随后 CTest **17/17** 通过。
- 分离干净检出的 Release 生产构建生成部署清单，`rl_real_LW --verify-deployment-only` 通过。仅初始化固定版本手柄子模块，没有下载机器人描述或使用非 LW SDK。
- `git diff --check`、Bash/Python 语法和定向 `cppcheck` 通过；`shellcheck` 与 `cmakelint` 不可用。未启动串口设备、仿真界面或实机。

---

<a id="lw-014"></a>

## [LW-014] 调试与绘图发布隔离

**优先级**： P2 / 低
**状态**： resolved
**依赖**： LW-007, LW-011

### 问题

生产构建在编译期无条件启用绘图发布，即便不需要调试输出，也创建 250 Hz ROS 定时器与发布器。LW-007 已保证数据源一致和非阻塞发布，但消息时间戳仍仅初始化一次，未随每个样本刷新。

### 证据

- `src/rl_sar/include/rl_real_LW.hpp:4`
- `src/rl_sar/src/rl_real_LW.cpp:129-178`
- `src/rl_sar/src/rl_real_LW.cpp:218-309`

### 计划范围

- 通过构建或运行时设置按需启用绘图。
- 保留 LW-007 引入的一致快照与非阻塞实时发布器。
- 每条消息刷新时间戳。
- 生产运行关闭绘图时，不创建绘图发布器或定时器。

### 验收标准

- 生产控制可在完全关闭绘图时运行。
- 启用绘图不引入控制路径数据竞争或阻塞。
- 发布消息携带当前时间戳及内部一致的样本。

### 解决记录

- **解决时间**： 2026-08-11T12:01:35+08:00
- **提交**： `6761b82b`
- **批准范围**： 将 LW 实机调试发布改为默认关闭的显式 ROS/启动文件
  参数；保留 LW-007 的一致快照和非阻塞发布边界；逐条刷新消息时间戳；关闭时
  不创建 发布器、定时器或控制线程调试快照。
- **修改文件**： `src/rl_sar/library/core/debug/lw_debug_publisher.*`、
  `src/rl_sar/include/rl_real_LW.hpp`、`src/rl_sar/src/rl_real_LW.cpp`、
  `src/rl_sar/launch/rl_real_LW.launch.py`、`src/rl_sar/CMakeLists.txt`、
  `src/rl_sar/test/test_lw_debug_publisher.cpp`、
  `docs/LW_BUILD_DEPLOYMENT_CN.md`。
- **验证**： `lw_debug_publisher` 与 `lw_runtime_sync` 定向 CTest 在现有、
  全新 Debug 和全新 Release 三个构建中均通过；三套完整 CTest 均为 18/18；
  `rl_real_LW` 与 `rl_sim_LW` 在 Debug/Release 全新构建 中构建成功；
  `cppcheck`、Python/Bash 语法、启动文件默认参数检查及 `git diff --check`
  通过；分离干净工作树 Release 部署构建、清单生成和
  `--verify-deployment-only` 通过。未启动串口、仿真界面或实机。
- **后续事项**： 无

---

<a id="lw-017"></a>

## [LW-017] 可复现的 IMU 与串口运行环境部署

**优先级**： P1 / 高
**状态**： resolved
**依赖**： LW-010, LW-015

### 问题

开发辅助脚本会构建 ROS `serial` 库和 `fdilink_ahrs`，但正式生产构建脚本只选择 `rl_sar`。安装后的 LW 启动文件无条件解析 `fdilink_ahrs`，因此全新或重定位后的生产前缀可能通过 `--verify-deployment-only`，却仍因缺少 IMU 包而在实机节点启动前失败。包清单还遗漏了 AHRS CMake 配置所需的依赖，部署指南也未提供完整的设备别名设置与包存在性检查流程。

### 证据

- `build_LW.sh:7-8`
- `src/rl_sar/scripts/build_lw_deployment.sh:50-52`
- `src/rl_sar/launch/rl_real_LW.launch.py:14-17`
- `src/rl_sar/package.ros2.xml:11-19`
- `src/fdilink_ahrs_ROS2/package.xml:10-25`
- `src/fdilink_ahrs_ROS2/CMakeLists.txt:22-34`
- `src/fdilink_ahrs_ROS2/wheeltec_udev.sh:1-13`
- `src/rl_sar/src/rl_real_LW.cpp:171-172`

### 计划范围

- 完整声明项目自有的 IMU 和串口依赖图。
- 将 `serial`、`fdilink_ahrs` 和 `rl_sar` 构建并安装到同一个干净的生产前缀。
- 将已安装的 IMU/串口运行时文件纳入部署完整性检查。
- 提供非自动执行的 IMU udev 辅助脚本，使用操作员批准的 `0777` 设备模式，并记录电机板与 IMU 各自不同的串口路径，不臆造电机 USB ID。
- ROS、操作系统库、USB 驱动和实际硬件激活不纳入部署清单及自动化验收范围。

### 验收标准

- 干净的生产前缀无需加载开发工作区环境，即可解析 `serial`、`fdilink_ahrs` 和 `rl_sar`。
- `--verify-deployment-only` 拒绝缺失、被修改、越出部署目录或为符号链接的项目自有 IMU/串口运行时文件。
- 两个生产可执行文件均无无法解析的动态库。
- 中文部署指南包含构建、包检查、权限、稳定设备名、离线验证以及受控 `/imu` 验证步骤。
- 自动化验证不打开串口设备，也不启动 ROS 节点。

### 解决记录

- **解决时间**： 2026-08-11T12:34:05+08:00
- **提交**： `a65a5dcd`
- **批准范围**： 补全 LW 正式部署中的 `serial`、`fdilink_ahrs` 依赖
  图和同前缀安装；将项目内 IMU/串口运行文件加入清单哈希与离线校验；提供
  非自动执行、设备权限为用户指定 `0777` 的 IMU udev 辅助脚本；用中文记录
  电机板与 IMU 串口的构建、设备别名、权限和受控验证步骤。安装规则仍需
  root，安装后访问 IMU 不要求 root 或 `dialout`。
- **修改文件**： `src/fdilink_ahrs_ROS2/{package.xml,CMakeLists.txt,wheeltec_udev.sh}`、
  `src/rl_sar/package.ros2.xml`、`src/rl_sar/scripts/{build_lw_deployment.sh,generate_lw_deployment_manifest.py}`、
  `src/rl_sar/library/core/deployment/lw_deployment_bundle.*`、
  `src/rl_sar/CMakeLists.txt`、`src/rl_sar/test/{test_generate_lw_deployment_manifest.py,test_lw_deployment_bundle.cpp,test_lw_runtime_dependencies.py}`、
  `docs/LW_BUILD_DEPLOYMENT_CN.md`、`.learnings/LW_REAL_DEPLOYMENT_ISSUES.md`。
- **验证**： 现有构建及从空目录创建的 Debug、Release 构建均通过新增
  三项定向 CTest 和完整 19/19 CTest；两个干净构建均构建了 `serial`、
  `fdilink_ahrs`、`rl_real_LW` 和 `rl_sim_LW`。临时克隆中的分离 HEAD 的干净工作树 Release 正式构建安装了三个 ROS 包，生成结构版本 2 清单并通过
  `--verify-deployment-only`；完整前缀重定位后，三个包均解析到新前缀，清单
  校验再次通过，两个运行程序的 `ldd` 均无 `not found`。Python/Bash 语法、
  XML 依赖测试、`cppcheck` 和 `git diff --check` 通过；`shellcheck`、
  `cmakelint` 不可用。未打开串口、未安装 udev 规则、未启动 ROS 节点、仿真
  界面或实机。用户随后明确要求将 IMU 设备模式恢复为 `0777` 且普通访问不
  依赖 `dialout`；四条规则、中文文档和批准范围已同步，Bash 语法、规则内容
  断言、三项部署定向 CTest 及 `git diff --check` 再次通过。
- **后续事项**： 无

---

<a id="lw-018"></a>

## [LW-018] 统一构建入口与 Jetson 检测

**优先级**： P2 / 中
**状态**： resolved
**依赖**： LW-015, LW-017

### 问题

`build_LW.sh` 通过多次独立的 `--packages-select` 调用重复实现常规构建入口。因此，全新工作区可能在某个包的工作区依赖尚未安装或尚未加载其环境时就选择构建该包。该辅助脚本还交互式安装系统包，并依靠注释提醒 Jetson 用户导出 `IS_JETSON=true`。Jetson 检测并不一致：推理下载脚本直接使用该变量而不确定其取值，Jetson 安装脚本执行更广泛的本地检查，而 CMake 只检查 `/etc/nv_tegra_release`。

### 证据

- `build_LW.sh:3-9`
- `build.sh:96-135`
- `scripts/download_inference_runtime.sh:16-19,73-104`
- `scripts/install_pytorch_jetson.sh:30-42`
- `src/rl_sar/CMakeLists.txt:134-139`
- `docs/LW_BUILD_DEPLOYMENT_CN.md:119-122`

### 计划范围

- 删除 `build_LW.sh`，仅保留 `build.sh` 作为开发构建入口。
- 使选定包的构建包含完整的工作区依赖闭包，并在选包前暴露所有对应 ROS 版本的包清单。
- 根据 Linux/aarch64 硬件标志自动确定 Jetson 模式，并支持经过验证的 `IS_JETSON=true|false` 显式覆盖，确保各处一致。
- 首次构建时自动安装缺失的 Debian/Ubuntu 和 ROS 构建依赖；依赖齐全时不执行包管理器操作。

### 验收标准

- 干净环境中的无参数构建按依赖顺序成功构建全部工作区包。
- 干净环境中选定 `fdilink_ahrs` 和 `rl_sar` 的构建包含 `serial` 及其余已声明的工作区依赖闭包。
- 无需 Jetson 硬件即可覆盖原生 Jetson、非 Jetson aarch64、x86_64、有效覆盖以及无效或不兼容覆盖。
- Shell、CMake、推理环境准备流程及文档报告相同的 Jetson 模式。
- 首次构建安装缺失的已声明系统/ROS 依赖，已有完整环境跳过包管理器操作。

### 解决记录

- **解决时间**： 2026-08-11T13:31:54+08:00
- **提交**： `a65a5dcd`
- **批准范围**： 删除重复的 `build_LW.sh`，将 `build.sh` 作为唯一开发
  构建入口；使选包构建包含完整工作区依赖闭包；统一 Shell、CMake 与推理运行时
  的 Jetson 自动检测及显式覆盖；首次构建默认自动安装缺失的 Debian/Ubuntu、
  ROS 构建依赖并继续准备项目运行库，依赖齐全时不调用包管理器。
- **修改文件**： `build.sh`（并删除 `build_LW.sh`）、
  `scripts/{common.sh,detect_jetson.sh,download_inference_runtime.sh,install_build_dependencies.sh,install_pytorch_jetson.sh}`、
  `src/rl_sar/CMakeLists.txt`、
  `src/rl_sar/test/{test_build_workflow.py,test_jetson_detection.py}`、
  `README_CN.md`、`docs/LW_BUILD_DEPLOYMENT_CN.md`、
  `.learnings/{LEARNINGS.md,LW_REAL_DEPLOYMENT_ISSUES.md}`。
- **验证**： 隔离测试覆盖 x86_64、无 Jetson 标志的 aarch64、三类 Jetson
  标志、有效 `IS_JETSON` 覆盖、无效及不兼容覆盖；假 `dpkg-query`/`apt-get`/
  `sudo` 验证缺包时执行更新和安装，依赖齐全时完全跳过包管理器。最终源码的
  干净的 `./build.sh fdilink_ahrs` 按 `serial -> fdilink_ahrs` 构建 2 包，干净的
  `./build.sh rl_sar` 按 `serial -> fdilink_ahrs -> rl_sar` 构建 3 包，无参数
  Debug 构建全部 6 包；独立 Debug、Release 的定向 CTest 和全量 21/21 CTest
  均通过。分离 HEAD 的干净工作树 Release 正式部署构建安装三个 ROS 包、生成
  结构版本 2 清单并通过 `--verify-deployment-only`。Bash/Python 语法、
  `cppcheck` 和 `git diff --check` 通过；`shellcheck`、`cmakelint` 不可用。
  验收未在宿主机实际执行 apt 安装，也未使用 Jetson 或实机硬件。
- **后续事项**： 无

---

<a id="lw-019"></a>

## [LW-019] 实机终端键盘恢复通道

**优先级**： P1 / 高
**状态**： resolved
**依赖**： LW-006, LW-007, LW-016

### 问题

LW FSM 支持键盘触发的状态转换，共享 SDK 也实现了终端按键解码，但 `rl_real_LW` 从不轮询键盘。因此，当手柄故障将 Gamepad 锁存为不可用后，文档所述保留的 `GetDown` 通道无法通过终端触达。在实机可执行文件中启动仿真器独立的键盘循环，又会与 200 Hz 控制线程产生竞争；而 ROS 2 启动的子进程也不能把经管道连接的标准输入当作交互终端。

### 证据

- `src/rl_sar/src/rl_real_LW.cpp:220-290,752-841`
- `src/rl_sar/src/rl_sim.cpp:152-180`
- `src/rl_sar/library/core/rl_sdk/rl_sdk.cpp:915-1037`
- `src/rl_sar/fsm_robot/fsm_LW.hpp:119-121`
- `.learnings/LEARNINGS.md:59-87`

### 计划范围

- 默认启用 `rl_real_LW` 的终端键盘输入，并为无交互终端部署提供显式启动参数。
- 从进程的控制终端非阻塞读取，并在所有正常或异常退出路径恢复 termios 状态。
- 仅在实机控制线程中轮询并应用键盘输入，不为 `Control` 增加第二个写入方。
- 保留手柄故障锁存与速度清零行为，同时保留键盘 FSM 事件，包括已有的 `9` 到 `GetDown` 的映射。
- 记录交互终端要求；请求键盘输入但无可用控制终端时，启动失败。

### 验收标准

- 从控制终端调用 `ros2 launch rl_sar rl_real_LW.launch.py` 时，默认启用键盘输入。
- 终端输入为非阻塞模式，关闭规范输入和回显但保留信号，并在析构时恢复原始 termios 状态。
- 控制线程在评估 FSM 转换前消费键盘事件，不存在并发写入 `Control` 的键盘工作线程。
- 已锁存的手柄故障不会清除终端 `GetDown` 事件。
- `enable_keyboard:=false` 支持显式选择无交互终端部署；启用键盘但无控制终端时，在控制循环启动前失败。

### 解决记录

- **解决时间**： 2026-08-11T15:31:22+08:00
- **提交**： `379a5165`
- **批准范围**： 真机默认启用 `/dev/tty` 终端键盘，以非阻塞 RAII 方式
  配置和恢复 termios；只由 200 Hz 控制线程读取并更新 FSM 输入，不创建并发
  键盘线程；保留手柄断联后的键盘 `GetDown` 通道；为无交互终端提供显式
  `enable_keyboard:=false`，启用但无控制终端时在控制循环启动前失败。
- **修改文件**： `src/rl_sar/library/core/safety/lw_terminal_keyboard.hpp`、
  `src/rl_sar/library/core/rl_sdk/rl_sdk.{hpp,cpp}`、
  `src/rl_sar/include/rl_real_LW.hpp`、`src/rl_sar/src/rl_real_LW.cpp`、
  `src/rl_sar/launch/rl_real_LW.launch.py`、`src/rl_sar/CMakeLists.txt`、
  `src/rl_sar/test/{test_lw_terminal_keyboard.cpp,test_lw_real_keyboard_integration.py}`、
  `README_CN.md`、`docs/LW_BUILD_DEPLOYMENT_CN.md`、
  `.learnings/{LEARNINGS.md,LW_REAL_DEPLOYMENT_ISSUES.md}`。
- **验证**： 伪终端测试验证描述符非阻塞、关闭 `ICANON/ECHO`、保留
  `ISIG`、析构恢复原始 termios、数字键 `9` 与方向键解析以及无终端拒绝；
  集成测试验证键盘在 FSM 前由控制线程轮询、未新增 `loop_keyboard`、手柄故障
  门不清除键盘、启动配置默认启用和文档契约。现有工作区及干净的 Debug、
  Release 的定向测试与全量 23/23 CTest 均通过，两个干净构建均构建全部
  6 包；`ros2 launch ... --show-args` 确认 `enable_keyboard` 默认 `true`。
  分离 HEAD 的干净工作树 Release 正式部署构建和 `--verify-deployment-only`
  通过。Python/C++ 严格语法、`cppcheck`（仅定点抑制既有 `CSVInit` 按值传参
  提示）和 `git diff --check` 通过；`clang-tidy`、`cmakelint` 不可用。未打开
  真机串口、未启动 ROS 节点或电机控制，也未在真实控制终端上进行实机试键。
- **后续事项**： 无

---

<a id="lw-020"></a>

## [LW-020] 架构匹配且仅使用 ONNX 的 Jetson 生产推理

**优先级**： P1 / 高
**状态**： resolved
**依赖**： LW-010, LW-018

### 问题

Jetson 构建会先准备 LibTorch，再准备 ONNX Runtime，尽管所有 LW 生产策略都是 ONNX。PyTorch 引导安装流程把所有 L4T R36 版本映射到同一个 CUDA 12.6 wheel，并遗漏文档中要求的 PyTorch 前置依赖，因此无关的 Torch 故障也可能中断全新的实机构建。已有推理运行时还仅凭目录结构就被接受；将 x86_64 开发树复制到 Jetson 会因此复用不兼容的 ELF 库。最后，只要本地碰巧存在 LibTorch 目录，生产 `rl_real_LW` 就会链接 Torch。

### 证据

- `build.sh:18-31`
- `scripts/download_inference_runtime.sh:49-85,173-184,281-349`
- `scripts/install_pytorch_jetson.sh:30-99,102-181`
- `src/rl_sar/CMakeLists.txt:121-277,436-449`
- `policy/LW/robot_lab/*/config.yaml:2`

### 计划范围

- 使原生 Jetson 构建和所有正式部署仅使用 ONNX，同时保留非 Jetson Sim2Sim 的可选 LibTorch 执行器网络支持。
- 在 CMake 或部署流程使用 Linux 推理库之前，验证已有和新下载库的 ELF 机器类型。
- 拒绝显式的 Jetson LibTorch 请求，不根据 L4T 主版本猜测 PyTorch wheel。
- Jetson 和生产配置必须使用有效的 ONNX Runtime，并验证安装后的实机可执行文件没有 Torch 运行时依赖。
- 记录受支持的 Jetson 路径以及禁止复用 x86_64 推理产物的要求。

### 验收标准

- Jetson 上的 `./build.sh` 仅准备 Linux aarch64 ONNX Runtime，绝不调用 PyTorch 安装脚本。
- Linux x86_64 和 AArch64 运行时库仅在匹配架构上被接受；ELF 文件缺失、损坏或不匹配时，在链接前失败。
- Jetson 和 `LW_PRODUCTION_DEPLOYMENT=ON` 构建设置 `USE_TORCH=OFF`；非 Jetson 开发构建保留可选 Torch 支持。
- 生产 `rl_real_LW` 依赖 ONNX Runtime，且没有 `libtorch`、`libtorch_cpu` 或 `libc10` 动态依赖。
- 文档、定向测试、全量测试、静态检查以及干净的 Debug 和 Release 构建均符合生产环境仅使用 ONNX 的约定。

### 解决记录

- **解决时间**： 2026-08-11T17:09:09+08:00
- **提交**： `cc07efc4`
- **批准范围**： Jetson 真机与正式部署仅准备并链接 ONNX Runtime，拒绝
  Jetson LibTorch 请求和错误 ELF 架构；非 Jetson 开发构建继续在新环境中
  自动准备 LibTorch 与 ONNX Runtime，保留 `rl_sim_LW --use_actuator_net`；
  正式部署验证 `rl_real_LW` 不依赖 Torch。Python 训练环境的 `torch` 不属于
  本项编译依赖范围。
- **修改文件**： `build.sh`、
  `scripts/{download_inference_runtime.sh,validate_inference_runtime.sh}`（并删除
  `scripts/install_pytorch_jetson.sh`）、`src/rl_sar/CMakeLists.txt`、
  `src/rl_sar/scripts/build_lw_deployment.sh`、
  `src/rl_sar/test/{test_build_workflow.py,test_inference_runtime_architecture.py}`、
  `README_CN.md`、`docs/LW_BUILD_DEPLOYMENT_CN.md`、
  `.learnings/{LEARNINGS.md,LW_REAL_DEPLOYMENT_ISSUES.md}`。
- **验证**： 架构校验单元测试 9/9、构建流程测试 12/12、Jetson 检测
  测试 8/8 通过；模拟 Jetson 显式请求 LibTorch 会在下载前失败，x86_64
  `all` 路径继续验证并准备 LibTorch 与 ONNX Runtime。现有构建及干净的
  Debug、Release 构建均通过全量 24/24 CTest；最终源码在三套构建目录中的
  LW-020 定向 CTest 均为 2/2。临时干净克隆的 Release 正式部署构建和
  `--verify-deployment-only` 通过，最终 `rl_real_LW` 依赖 ONNX Runtime，
  `readelf`/`ldd` 未发现 `libtorch`、`libtorch_cpu` 或 `libc10`。Bash/Python
  语法与 `git diff --check` 通过；未修改 C++ 源文件，`shellcheck`、
  `cmakelint` 不可用。验收未在 Jetson、串口或真机上运行 ROS 节点或电机控制。
- **后续事项**： 无

---

<a id="lw-021"></a>

## [LW-021] Sim2Sim 与实机运行行为一致性

**优先级**： P1 / 高
**状态**： resolved
**依赖**： LW-007, LW-008, LW-011, LW-016, LW-020

### 问题

`rl_sim_LW` 和 `rl_real_LW` 使用相同的 LW FSM、策略定义、观测/输出辅助组件、策略输出传输机制、名义控制周期、推理周期、关节顺序和手柄映射。不过，它们在两个独立入口源文件中各自实现运行流水线。仿真器从状态获取直接经过 `StateController()` 到命令应用；实机可执行文件则额外执行与硬件无关的策略动作/输出验证、过期输出回退、最终命令验证、循环异常处理及控制时序安全动作。

默认的 `RL::HandleLWPolicyOutputFault()` 不执行任何操作，因此在 Sim2Sim 中，过期或不完整的策略输出可能使上一条命令被保留，而实机可执行文件会请求 Passive 阻尼。仿真器还在控制和调试回调中读取、重置和写入 `mjData`，却未获取物理线程使用的 `sim->mtx`（它包围 `mj_step()` 操作）。因此，无法保证其观测与命令构成一致的物理步快照。

此外，仿真器加载可变的源码树策略，可选用 Torch 执行器网络，始终启用绘图路径，而且没有实机运行时的时序或安全事件回调。现有测试分别验证共享组件，但没有确定性的端到端测试向两条运行路径输入相同状态/输入序列，并比较 FSM、观测、策略、命令以及与硬件无关的安全结果。因此，通过 Sim2Sim 运行可以作为正常行为的证据，但目前不能据此证明与实机部署的行为等价。

### 证据

- `src/rl_sar/src/rl_sim_LW.cpp:46-195,395-852,853-948`
- `src/rl_sar/src/rl_real_LW.cpp:100-298,456-898,899-1203`
- `src/rl_sar/include/rl_sim_LW.hpp:4-141`
- `src/rl_sar/include/rl_real_LW.hpp:8-172`
- `src/rl_sar/library/core/rl_sdk/rl_sdk.hpp:353-357`
- `src/rl_sar/library/core/rl_sdk/rl_sdk.cpp:1207-1260`
- `src/rl_sar/library/thirdparty/mujoco_simulate/mujoco_utils.hpp:332-428`
- `src/rl_sar/CMakeLists.txt:774-845`
- 现有 CTest 注册表：24 项组件测试，没有 Sim2Sim/实机一致性测试。

### 计划范围

- 提取一个由两个可执行文件共用的平台无关 LW 运行时核心，负责输入应用、FSM 执行、策略输入发布、推理、策略动作/输出验证、过期输出处理、最终命令验证以及与硬件无关的安全决策。
- 将传感器获取、执行器命令交付、MuJoCo 物理/UI、串口通信、ROS 接线及平台生命周期保留在精简的实机/仿真适配器中。不削弱或重新解释已批准的 LW-016 安全矩阵。
- 为仿真器提供安全适配器，既记录相同决策，也执行其 MuJoCo 等效动作，不打开串口或关闭无关宿主进程。S1 抑制操作员速度，同时保留批准的 FSM 恢复输入；S2 通过仿真器执行器路径施加完全相同的共享 Passive 命令（`q=current`、`dq=0`、`tau=0`、`Kp=0`、`Kd=5`）；S3/S4 锁存关闭命令接收，并将所有活动 MuJoCo 执行器输出置零。S4 还记录所请求的终止关闭状态，同时允许测试框架检查最终轨迹。
- 通过锁定 `sim->mtx` 或发布不可变的逐步快照，使 MuJoCo 状态获取与命令应用同物理线程保持一致。消除未同步的重置、读取、调试和命令访问。
- 增加确定性的无窗口回放框架，向共享核心提供相同的 `RobotState`、操作员输入、时间戳、策略帧和注入故障，并比较完整可观测轨迹。
- 使用部署包标识的精确策略/配置资产进行一致性测试，不默许可变源码树与部署资产存在差异。
- 将仿真专用的重置/暂停控制以及可选执行器动力学建模保留在共享控制器核心之外，明确视为适配器行为，不作为一致性测试覆盖的实机运行行为。
- 本问题不修改策略模型、YAML 值、FSM 转换语义、串口协议、电机增益、安全阈值、CUDA/TensorRT 支持或硬件激活行为。

### 验收标准

- `rl_sim_LW` 和 `rl_real_LW` 在每个平台无关的控制和推理阶段调用同一实现；源码级重复仅限于文档注明的适配器职责。
- 对 Passive、起身、趴下、运动和形态转换的每种场景，相同回放输入在明确记录的浮点容差内产生相同 FSM 状态、策略代际、观测向量、策略帧和 `RobotCommand` 值。
- NaN/Inf 策略动作和输出、不完整或过期策略帧、回退序号、推理异常、控制异常及时序降级，在仿真与实机运行时测试中产生相同的硬件无关安全决策和锁存状态。
- Sim2Sim 在可见行为和数值上执行每个安全等级的批准动作：S1 将命令速度置零但不清除恢复按钮，S2 进入并保持 `Kp=0`、`Kd=5` 的 Passive 阻尼，S3/S4 在终止锁存后产生零活动执行器作用力。测试不得将 Passive 阻尼和执行器零输出视为等效动作。
- 通过适配器契约注入 IMU 缺失、串口交付失败和电机板故障等硬件特定事件；测试无需访问硬件即可验证实机终止动作以及仿真器对应的执行和记录动作。
- 仿真器回调不得与物理线程并发访问可变的 `mjModel`/`mjData`。压力测试覆盖状态快照、命令写入、重置和可选调试发布。
- 策略/配置哈希与部署包不同时，一致性测试框架失败；使用精确的已提交部署包时通过。
- 现有 LW 测试在当前构建、干净 Debug 和干净 Release 构建中持续通过；两个可执行文件均成功构建；正式 Release 部署及 `--verify-deployment-only` 通过。
- 自动化验收保持无窗口运行，不打开串口设备、不启动实机 ROS 节点、不发送电机命令，也不声称完成物理验证。

### 解决记录

- **解决时间**： 2026-08-11T18:43:25+08:00
- **提交**： `25ba1927`
- **批准范围**： 将 Sim2Sim 与真机入口的平台无关控制、推理、输出校验、
  时序和安全决策收敛到同一运行时核心；仿真执行与真机同义的 S1 速度抑制、
  S2 Passive 阻尼以及 S3/S4 零执行器输出，并保留恢复按键；同步 MuJoCo
  读写；加入真实 ONNX 推理、故障注入、部署策略哈希和并发压力回放。传感器、
  执行器、串口、ROS、MuJoCo 生命周期及仿真专用复位/暂停/执行器网络继续由
  各自适配层负责；未改变策略、FSM 语义、串口协议、增益或安全阈值。
- **修改文件**： `src/rl_sar/library/core/safety/{lw_runtime_core.hpp,lw_loop_config.hpp}`、
  `src/rl_sar/include/{rl_real_LW.hpp,rl_sim_LW.hpp}`、
  `src/rl_sar/src/{rl_real_LW.cpp,rl_sim_LW.cpp}`、`src/rl_sar/CMakeLists.txt`、
  `src/rl_sar/scripts/{build_lw_deployment.sh,verify_lw_policy_parity.py}`、
  `src/rl_sar/test/{test_lw_runtime_parity.cpp,test_lw_mujoco_synchronization.cpp,test_verify_lw_policy_parity.py,test_lw_real_keyboard_integration.py}`、
  `.learnings/{LEARNINGS.md,LW_REAL_DEPLOYMENT_ISSUES.md}`。
- **验证**： 共享核心的确定性双适配器测试覆盖 FSM/命令一致性、真实
  `leg_loco` ONNX 推理的完整观测/动作/输出回放，以及 S1-S4、
  NaN/Inf、过期/缺帧/代际错误、控制与推理异常和时序故障；MuJoCo 无窗口
  四线程压力测试覆盖物理步进、状态快照、命令/复位和调试快照。当前工作树、
  干净的 Debug 和干净的 Release 均成功构建两个入口并通过全量 27/27 CTest。
  隔离临时克隆的仅使用 ONNX 的 Release 正式部署构建、策略源文件/部署清单
  SHA-256 一致性检查及 `--verify-deployment-only` 全部通过，部署二进制无
  LibTorch 依赖。Python 编译、Bash 语法和 `git diff --check` 通过。所有验收
  均为无窗口运行；未打开串口、未启动真机 ROS 节点、未发送电机命令，也未在
  Jetson 或真实机器人上执行物理验证。
- **后续事项**： 无

---

<a id="lw-022"></a>

## [LW-022] 机器人悬吊状态下的实机性能分析与候选配置

**优先级**： P1 / 高
**状态**： resolved
**依赖**： LW-011, LW-016, LW-020, LW-021

### 问题

当前 LW 的传感器时效性、成对串口写入截止时间、控制循环 CPU 亲和性、实时调度和时序降级默认值，是按可移植的保守默认值选定的，并非在目标 Jetson 上测得。现有诊断仅暴露控制循环的汇总最大值，不保留传感器到达间隔或串口入队延迟分布。此外，处于 Passive 的悬吊运行会使所有运动策略保持不活跃，其 CPU 负载无法代表生产推理负载。

从空载运行结果盲目推导生产安全阈值并不安全。尤其是，`control_loop_require_realtime` 属于运行策略，而致命时序阈值取决于经过验证的板端看门狗及物理硬失能行为；两者都不能仅凭延迟样本选定。

### 计划范围

- 增加仅主机分析器，运行部署包中精确的四个 LW ONNX 策略、观测/输出路径、运动参考、200 Hz 控制调度和 50 Hz 推理，不打开 ROS、手柄、IMU、串口或执行器设备。
- 增加独立的悬吊硬件观察模式；没有显式确认标志就拒绝启动，绝不进入 LW 运动 FSM 状态，且只发送 `motors_disable=true` 数据包，同时测量 IMU、左右有效反馈间隔及成对串口入队延迟。
- 在机器可读报告中记录有界分布统计、控制循环启动/时序状态、测试的 CPU/优先级、策略身份、目标环境及失败计数。
- 增加编排器/分析器，对允许的 CPU 及显式请求的实时优先级进行排名，随后生成仅供评审的 JSON 报告，其中包含兼容 YAML 的候选覆盖配置。绝不修改 `policy/LW/base.yaml` 或部署包。
- 在建议相关阈值前，必须由操作员提供最大安全传感器年龄和最大安全控制间断。保持致命时序阈值关闭，并将 `control_loop_require_realtime` 标记为人工部署决策。
- 记录悬吊影子测量只生成暂定的软件和通信候选参数，不构成最终动态或物理安全验证。

### 验收标准

- 默认测量仅在主机运行，不进行硬件 I/O；硬件模式要求精确、可审计的确认，在策略预加载前确认初始失能连发，并持续仅发送电机失能包。
- 四个部署策略均执行真实 ONNX 推理，不使 FSM 离开 Passive，也不允许策略输出到达执行器。
- 报告按适用情况包含推理、传感器间隔和串口写入的样本数、分位数/最大值统计，以及循环错过周期、亲和性和调度应用结果。
- 候选生成拒绝不足或失败的测量，绝不启用致命时序，绝不静默要求 SCHED_FIFO，且仅写入用户选择的新输出路径。
- 单元测试覆盖统计、安全门控、候选边界、输入配置不可变以及畸形报告。现有测试和两个 LW 可执行文件继续构建并通过验证，无需访问硬件。

### 解决记录

- **解决时间**： 2026-08-11T19:54:33+08:00
- **提交**： `57f184e6`
- **批准范围**： 增加两阶段 LW 部署参数测量。仅主机阶段在目标主机
  运行四个正式 ONNX、共享控制/观测/输出路径及 200 Hz/50 Hz 线程而不访问
  硬件；吊装硬件观察阶段保持 Passive、丢弃策略输出并只发送电机
  失能包，采集 IMU、左右反馈及成对串口写时序。分析器只生成需人工评审的
  候选报告，不修改 `base.yaml`，不自动启用实时强制或致命时序阈值。
  根据底层控制器上电即使能的现场约束，硬件模式在任何模型预加载前确认左右板
  初始失能包完整写入，并以独立 5 ms 保活持续只发送失能包；失败时不得开始或
  继续测算。反馈协议没有失能回执位，因此不把完整串口写入误称为物理执行确认。
- **修改文件**： `src/rl_sar/src/lw_config_profiler.cpp`、
  `src/rl_sar/library/core/safety/lw_config_profile.hpp`、
  `src/rl_sar/scripts/{profile_lw_runtime_config.py,build_lw_deployment.sh,generate_lw_deployment_manifest.py}`、
  `src/rl_sar/library/core/deployment/lw_deployment_bundle.cpp`、
  `src/rl_sar/test/{test_lw_config_profile.cpp,test_lw_config_profiler_integration.py,test_profile_lw_runtime_config.py,test_lw_deployment_bundle.cpp}`、
  `src/rl_sar/CMakeLists.txt`、`docs/LW_BUILD_DEPLOYMENT_CN.md`、
  `.learnings/LW_REAL_DEPLOYMENT_ISSUES.md`。
- **验证**： 当前工作树 Debug 全目标构建成功，`rl_sim_LW`、
  `rl_real_LW` 和 `lw_config_profiler` 均完成链接；完整 31/31 CTest 通过。
  集成测试实际加载四个正式 ONNX 并分别执行推理，确认仅主机报告没有串口
  写或硬件命令，错误确认字符串在任何设备初始化前被拒绝；使用正确确认字符串
  但不可用串口时，采集器在任何 ONNX 预加载前失败。统计和分析器测试
  覆盖滚动上限、分位数、首帧/结束帧龄、配置输入不变、安全上限、采样不足、
  畸形字段、策略缺失及输出不可覆盖。隔离临时 Git 验证快照成功生成
  仅使用 ONNX 的 Release 正式部署，策略哈希、部署清单、动态库和
  `--verify-deployment-only` 均通过；部署包内采集器再次完成四策略仅主机
  推理，且采集器/分析器均纳入清单。Python 编译、Bash 语法和
  `git diff --check` 通过。未访问串口、未启动真机 ROS 节点、未发送电机命令；
  目标 Jetson 的 CPU/实时数据及吊装硬件数据必须按中文文档现场采集，不能由
  当前无硬件验收替代。
- **后续事项**： 无

---

<a id="lw-023"></a>

## [LW-023] 启动前禁能的安全边界

**优先级**： P0 / 严重
**状态**： resolved
**依赖**： LW-010, LW-017, LW-020

### 问题

部署的 STM32 在上电时使能电机，但正常实机节点直到完成 ROS/输入设置、YAML 验证以及全部四个策略和策略上下文的预加载后，才打开两个电机板串口并发送首个失能包。因此，在串口初始化前发生模型缺失、配置无效、终端故障、分配失败或其他异常，可能使已上电硬件一直保持使能，上层却没有尝试失能。LW-022 硬件分析器已经建立更严格的预加载前失能边界，但正常部署入口尚未如此。

### 证据

- `src/rl_sar/src/rl_real_LW.cpp:44-127`
- `src/rl_sar/src/rl_real_LW.cpp:143-169`
- `src/rl_sar/src/lw_config_profiler.cpp:401-425`

### 计划范围

- 保留硬件访问前的离线部署完整性验证。
- 验证后，在策略预加载或其他耗时/可能失败的运行时初始化前，建立并确认实机节点的电机失能输出。
- 从任一串口打开后开始，所有构造函数失败路径都应确保失能输出安全。
- 不削弱现有启动、命令门或最终失能检查。

### 验收标准

- 两个串口均可用时，必须先向两块板完整写入失能包，之后才开始任何模型预加载。
- 串口初始化后的任何失败都关闭命令交付，并在释放端口前尝试有界的最终失能序列。
- 部分完成的串口初始化不能启动任何工作循环或模型预加载。
- 测试无需访问物理硬件即可证明操作顺序和构造失败清理行为。
- 部署指南准确说明最早阶段不可避免的使能区间，以及仍然需要物理隔离和急停。

### 解决记录

- **解决时间**： 2026-08-12T12:50:24+08:00
- **提交**： `542dcc9b`
- **批准范围**： 在正式部署完整性校验通过后、ROS/终端/YAML/FSM/模型及
  `RL_Real` 堆分配之前，以栈上 RAII 守卫打开左右电机板串口并确认 20 个双侧
  完整失能包，随后以独立 5 ms 线程持续只发送失能；全部运行资源准备完成后
  等待保活线程结束、应用已验证的运行时写超时并补发交接失能，才允许启动工作循环。
  任一部分串口初始化或后续构造失败均关闭命令门、停止保活并尝试最终 20 包
  失能。保留部署离线校验、统一串口命令门和正常退出顺序，不把主机完整写入
  称为 STM32 物理执行确认。
- **修改文件**： `src/rl_sar/library/core/safety/lw_startup_disable.hpp`、
  `src/rl_sar/include/rl_real_LW.hpp`、`src/rl_sar/src/rl_real_LW.cpp`、
  `src/rl_sar/test/{test_lw_startup_disable.cpp,test_lw_real_startup_disable_integration.py}`、
  `src/rl_sar/CMakeLists.txt`、`docs/LW_BUILD_DEPLOYMENT_CN.md`、
  `.learnings/LW_REAL_DEPLOYMENT_ISSUES.md`。
- **验证**： 当前 Debug 全目标构建成功，完整 33/33 CTest 通过；启动失能
  PTY 生命周期测试和正式入口顺序集成测试连续 20 轮通过。全新 Release 构建
  成功生成 `rl_real_LW`，启动失能、正式入口顺序、串口 SDK、循环生命周期和
  安全策略测试 5/5 通过。新增 C++ 测试通过独立
  `-Wall -Wextra -Wpedantic -Werror` 构建、AddressSanitizer/
  UndefinedBehaviorSanitizer 和定向 `cppcheck`；Python 编译及
  `git diff --check` 通过。隔离分离 HEAD 的临时验证提交成功生成仅使用 ONNX 的
  Release 正式部署，策略哈希、部署清单、动态依赖和
  `rl_real_LW --verify-deployment-only` 均通过，临时工作树随后已清理。
  全部自动化仅使用 PTY 或离线路径；未访问真实串口、未启动真机 ROS 节点、
  未发送真实电机命令，也未把主机 `write()` 当作硬件失能回执。部署文档明确
  保留上电至首批失能写入之间的不可避免窗口，以及可靠吊装、机械隔离和物理
  急停要求。
- **后续事项**： 无

---

<a id="lw-024"></a>

## [LW-024] 可等待线程结束且有界的 Sim2Sim 物理线程生命周期

**优先级**： P1 / 高
**状态**： resolved
**依赖**： LW-001, LW-021

### 问题

`rl_sim_LW` 分离 MuJoCo 物理线程，并向其传入自有 `Simulate` 对象的裸指针。析构函数不等待该线程，因此正常退出或构造函数异常可能在物理清理仍访问 `sim` 时就销毁它。模型加载失败时，启动流程还会无限等待全局 MuJoCo 数据，因为物理线程没有结果通道，轮询循环也没有失败或超时条件。

### 证据

- `src/rl_sar/src/rl_sim_LW.cpp:74-112`
- `src/rl_sar/src/rl_sim_LW.cpp:250-269`
- `src/rl_sar/library/thirdparty/mujoco_simulate/mujoco_utils.hpp:436-459`

### 计划范围

- 保持物理线程可联结，并明确所有权。
- 将模型/数据初始化的成功或失败结果传递给构造函数。
- 限制启动等待时长，并确保部分构造失败不会遗留访问已销毁状态的工作线程。
- 保留 LW-021 建立的共享 LW 运行时行为。

### 验收标准

- 所有正常退出、信号请求退出、安全请求退出和异常退出，都在销毁 `Simulate`、`mjData` 或 `mjModel` 状态前停止并联结物理线程。
- 无效或无法加载的场景在启动时失败并给出诊断，不无限等待。
- 重复启动/退出及注入初始化故障，在适当的消毒器或确定性生命周期压力测试下通过。
- 不存在保留 `RL_Real` 或 `Simulate` 成员指针的分离线程。

### 解决记录

- **解决时间**： 2026-08-12T13:33:29+08:00
- **提交**： `efd339cd`
- **批准范围**： 以 RAII 生命周期对象同步加载并唯一持有初始
  `mjModel`/`mjData`，再通过有界启动握手启动可联结的物理工作线程；删除
  `rl_sim_LW` 的分离线程和全局 `d` 无限轮询。为尚未启动或正在退出
  的渲染循环增加可取消模型交接，所有正常、窗口、安全请求和异常路径均先停止
  业务循环、唤醒并联结物理工作线程，之后才释放 MuJoCo 资源和 `Simulate`。
  线程异常跨边界传播到主线程；无效场景保留文件名和 MuJoCo 诊断。保持 LW-021
  的共享运行时及 S1–S4 安全动作语义，不处理 LW-028 的信号处理器重构。
- **修改文件**： `src/rl_sar/library/core/simulation/lw_joinable_worker.hpp`、
  `src/rl_sar/library/thirdparty/mujoco_simulate/{mujoco_utils.hpp,simulate.h,simulate.cc}`、
  `src/rl_sar/include/rl_sim_LW.hpp`、`src/rl_sar/src/rl_sim_LW.cpp`、
  `src/rl_sar/test/{test_lw_joinable_worker.cpp,test_lw_mujoco_lifecycle.cpp,test_lw_sim_lifecycle_integration.py}`、
  `src/rl_sar/test/data/lw024_minimal.xml`、`src/rl_sar/CMakeLists.txt`、
  `.learnings/LW_REAL_DEPLOYMENT_ISSUES.md`。
- **验证**： 当前 Debug 全目标构建成功，`rl_sim_LW` 完成链接，完整
  36/36 CTest 通过。全新 Release 构建成功，可联结工作线程、生命周期入口、
  MuJoCo 同步和真实无窗口 MuJoCo 生命周期测试 4/4 通过。真实 MuJoCo 测试
  覆盖无效场景诊断、渲染接管前取消、联结、资源清空及幂等停止，连续 50 轮
  确定性压力通过；通用工作线程覆盖正常停止、启动超时、注入启动/运行异常和
  200 次内部重复生命周期，并在 AddressSanitizer/UndefinedBehaviorSanitizer
  下连续 20 轮通过。ASan/UBSan 版本 `rl_sim_LW` 构建成功；严格警告构建在仅
  降级 MuJoCo 既有 reorder、缺失初始化、未使用参数/函数和 sign-compare
  类别后通过。定向 `cppcheck`、Python 集成测试和 `git diff --check` 通过。
  未启动图形窗口、未访问真机硬件，也未改变 LW-028 信号处理器。
- **后续事项**： 无

---

<a id="lw-025"></a>

## [LW-025] 持久且可组合的键盘速度输入

**优先级**： P1 / 高
**状态**： resolved
**依赖**： LW-006, LW-019, LW-021

### 问题

每个控制周期都会在处理键盘前，将手柄邮箱的 `x/y/yaw` 复制到共享控制状态。随后 `W/S/A/D/Q/E` 对该状态的修改只持续当前 5 ms 周期；下个周期又会用手柄值覆盖。这与文档所述的键盘速度持久性行为相矛盾，而且 50 Hz 推理循环可能完全错过这一短脉冲。因此，`Space` 也无法提供文档所述针对键盘输入的持久停止语义。

### 证据

- `src/rl_sar/src/rl_real_LW.cpp:875-885`
- `src/rl_sar/src/rl_sim_LW.cpp:957-967`
- `src/rl_sar/library/core/rl_sdk/rl_sdk.cpp:96-125`
- `docs/LW_BUILD_DEPLOYMENT_CN.md:474-485`

### 计划范围

- 为键盘速度命令提供持久状态，并明确其相对于手柄速度的合并/优先级规则。
- 保持 FSM 按钮事件为边沿触发，保留手柄故障和时序降级的零速度锁存。
- 在实机和 Sim2Sim 适配器中应用相同的硬件无关行为。

### 验收标准

- 键盘速度阶跃持续对后续控制和推理周期可见，直到按文档规则修改或清除。
- `Space`、手柄断联和时序降级可靠地将全部三个速度轴强制置零。
- 键盘与手柄输入切换遵循一条有文档说明的确定性优先级规则，不重新出现陈旧值。
- 自动化一致性测试在类实机和类 Sim2Sim 框架中覆盖持久性、清零、来源切换及 FSM 恢复按钮。

### 解决记录

- **解决时间**： 2026-08-12T15:39:32+08:00
- **提交**： `1b29744f`
- **批准范围**： 用户明确决定以取消 LW 键盘速度功能替代原持久化与
  输入仲裁方案。真实机和 Sim2Sim 的共享 LW 运行核心均禁止
  `W/S/A/D/Q/E/Space` 修改 `x/y/yaw`，摇杆成为唯一人工速度来源；键盘
  FSM/模式事件（包括数字键 `9` 的 `GetDown`）继续进入状态机，手柄断联和
  控制时序降级的锁存零速语义保持不变。通用非 LW `StateController` 默认行为
  未改变。
- **修改文件**： `src/rl_sar/library/core/rl_sdk/rl_sdk.{hpp,cpp}`、
  `src/rl_sar/library/core/safety/lw_runtime_core.hpp`、
  `src/rl_sar/test/{test_lw_runtime_parity.cpp,test_lw_real_keyboard_integration.py}`、
  `docs/LW_BUILD_DEPLOYMENT_CN.md`、
  `.learnings/LW_REAL_DEPLOYMENT_ISSUES.md`。
- **验证**： 当前 Debug 工作树完整构建真实机和 Sim2Sim 入口并通过
  36/36 CTest；定向测试覆盖七个原速度键不改变摇杆三轴、键盘事件仍到达 FSM、
  S1 保留恢复键且锁住零速，以及非 LW 默认行为未被改变。Release 构建两个入口
  并通过终端键盘、真机键盘集成和运行时一致性测试；严格警告配置构建两个入口
  和一致性测试（仅输出该配置已知且已降级的 MuJoCo/旧适配层警告）；ASan/UBSan
  运行时一致性测试通过。Python 严格语法和 `git diff --check` 通过。未打开真机
  串口、未启动 ROS 节点、MuJoCo 图形窗口或电机控制，也未进行实机试键。
- **后续事项**： 无

---

<a id="lw-026"></a>

## [LW-026] 配置方案的来源追溯与可比性

**优先级**： P1 / 高
**状态**： resolved
**依赖**： LW-010, LW-022

### 问题

测量器记录了 `source_commit`、`policy_root`、主机身份和时长，但分析器未验证或比较这些字段。因此，来自不同提交或不同策略树的主机和硬件报告可以与无关的 `base.yaml` 合并。精确策略检查使用集合，因而接受重复策略记录；累计错过周期数的排名也不要求测量时长可比。因此，混合或陈旧的报告集可能为错误的部署生成看似合理的评审文件。

### 证据

- `src/rl_sar/src/lw_config_profiler.cpp:930-983`
- `src/rl_sar/scripts/profile_lw_runtime_config.py:166-187`
- `src/rl_sar/scripts/profile_lw_runtime_config.py:294-320`
- `src/rl_sar/scripts/profile_lw_runtime_config.py:234-287`

### 计划范围

- 将每个被分析报告及选定的基础配置绑定到一个精确、可评审的部署身份。
- 要求每个批准策略恰有一条记录，并拒绝重复。
- 要求时长可比，或显式归一化每个依赖时长的排名指标。
- 保留仅供评审的输出以及现有物理安全限制。

### 验收标准

- 分析器拒绝混合源码提交、策略根/资产、不兼容的基础配置、模式以及不可比的报告时长。
- 四条策略记录必须恰为按批准顺序排列的四条唯一记录，或采用另一种经过显式验证的规范表示。
- 将每个接受的身份字段复制到候选报告，使人工能够将候选追溯到测量及基础文件。
- 测试覆盖混合提交、混合策略根/资产、重复策略、时长不匹配、陈旧基础输入及有效的同部署报告。

### 解决记录

- **解决时间**： 2026-08-12T16:59:22+08:00
- **提交**： `5425b679`
- **批准范围**： 测量器报告升级为结构版本 v2，在模型加载前固定源码提交、
  结构化主机身份、规范化策略根、每策略时长以及固定顺序的 11 个批准策略资产
  SHA-256，并在写报告前复核资产未变化。分析器仅接受同一提交、主机、策略根和
  资产摘要的报告，要求四个策略恰好各一条且顺序一致，要求全部主机报告具有
  相同测量时长，并将硬件时长独立记录；`--base-yaml` 必须是该身份中的
  `LW/base.yaml` 且摘要一致。候选结构版本 v2 复制部署身份、基础配置和每个输入报告
  的路径/摘要/模式/时长，继续保持仅供评审、致命时序关闭及原物理安全限制。
- **修改文件**： `src/rl_sar/src/lw_config_profiler.cpp`、
  `src/rl_sar/scripts/profile_lw_runtime_config.py`、`src/rl_sar/CMakeLists.txt`、
  `src/rl_sar/test/{test_profile_lw_runtime_config.py,test_lw_config_profiler_integration.py}`、
  `docs/LW_BUILD_DEPLOYMENT_CN.md`、
  `.learnings/LW_REAL_DEPLOYMENT_ISSUES.md`。
- **验证**： 当前 Debug 工作树完整构建并通过 36/36 CTest；分析器 17 项
  单元测试覆盖旧结构版本、混合提交/主机/策略根/资产、重复/缺失/乱序策略、
  主机时长不一致、独立硬件时长、陈旧基础配置、模式矛盾、输入变化及合法
  同部署组合。真实测量器集成测试核对结构化身份、有序资产摘要、每策略时长、
  主机无硬件输出以及硬件确认/缺失串口拒绝。Release 和严格警告配置均构建
  测量器并通过定向分析器/测量器测试；ASan/UBSan 测量器构建成功，
  主机测量及错误确认、缺失串口两个拒绝路径分别在受控单进程中通过。ASan 下
  同一 Python 测试连续启动多个 ONNX 子进程会出现环境性进程启动洪泛/10 秒超时，
  普通 Debug、Release、严格警告及各独立 ASan 路径均未复现功能失败。Python
  严格语法、定向 `cppcheck` 和 `git diff --check` 通过。未访问真机串口、未启动
  硬件观察、ROS 节点或电机控制。
- **后续事项**： 无

---

<a id="lw-027"></a>

## [LW-027] 可复现的 ONNX Runtime 部署依赖

**优先级**： P1 / 高
**状态**： resolved
**依赖**： LW-010, LW-017, LW-020

### 问题

生产 LW 可执行文件内嵌了指向仓库 ONNX Runtime 目录的构建主机绝对 RPATH。生成的部署前缀既不包含该运行时，也未在清单中记录其哈希。因此，移动前缀可能破坏启动；替换外部运行时则可能改变执行行为，即使 `--verify-deployment-only` 仍接受该部署包。

### 证据

- `src/rl_sar/CMakeLists.txt:295-320`
- `src/rl_sar/scripts/generate_lw_deployment_manifest.py:25-36`
- `src/rl_sar/library/core/deployment/lw_deployment_bundle.cpp:38-53`
- `docs/LW_BUILD_DEPLOYMENT_CN.md:739-749`

### 计划范围

- 明确 ONNX Runtime 是随包部署还是作为显式外部已验证依赖，并一致执行该约定。
- 消除对未经验证、可变的构建树库路径的依赖。
- 将每个必需运行时库及相关身份纳入部署生成、完整性检查和重定位测试。

### 验收标准

- 部署必须通过可重定位的运行时搜索路径携带所需 ONNX Runtime 库，或在具有记录身份的外部运行时不存在时拒绝启动。
- 运行时库与可执行文件和策略一起接受架构及完整性验证。
- 重定位完整的批准部署前缀不依赖原始源码树路径。
- 必需运行时库被篡改或遗漏时，部署验证在 ROS、串口或电机初始化前失败。

### 解决记录

- **解决时间**： 2026-08-12T17:24:40+08:00
- **提交**： `83890c51`
- **批准范围**： 正式部署将实际使用的 ONNX Runtime 主库和共享执行提供程序库
  作为普通文件安装到 `lib/rl_sar/onnxruntime/`，两个生产可执行文件仅以
  `$ORIGIN/onnxruntime` 相对 RPATH 加载它们。清单升级为结构版本 v3，
  固定 ONNX Runtime 版本、规范化 CPU 架构、精确两库集合和各自 SHA-256；
  C++ 启动校验拒绝缺失、多余、符号链接、目录逃逸、错误 ELF64 架构和哈希
  不匹配。部署脚本拒绝源码树 ONNX RPATH 和部署外解析结果，并自动验收原部署
  前缀及完整重定位副本。
- **修改文件**： `src/rl_sar/CMakeLists.txt`、
  `src/rl_sar/cmake/install_lw_deployment.cmake.in`、
  `src/rl_sar/library/core/deployment/{lw_deployment_bundle.cpp,lw_deployment_bundle.hpp}`、
  `src/rl_sar/scripts/{build_lw_deployment.sh,generate_lw_deployment_manifest.py}`、
  `src/rl_sar/test/{test_build_workflow.py,test_generate_lw_deployment_manifest.py,test_lw_deployment_bundle.cpp,test_verify_lw_policy_parity.py}`、
  `docs/LW_BUILD_DEPLOYMENT_CN.md`、
  `.learnings/LW_REAL_DEPLOYMENT_ISSUES.md`。
- **验证**： 当前 Debug 工作树完整构建并通过 36/36 CTest；清单
  生成器 13 项测试和 C++ 部署包 18 条路径覆盖版本/架构、精确文件集合、缺失、
  符号链接、错误 ELF 架构、篡改、重定位及既有策略/运行文件约束。Release 和
  严格警告配置均构建主程序、配置测量工具与部署包测试并通过 4 项定向 CTest；
  ASan/UBSan 部署包测试通过。真实 `LW_PRODUCTION_DEPLOYMENT=ON` Release
  部署生成成功，清单记录 ONNX Runtime 1.22.0/x86_64 和两个库摘要；
  `readelf` 仅见 `$ORIGIN/onnxruntime`，`ldd` 对两个生产可执行文件均解析到
  当前部署前缀，原前缀和临时重定位副本的 `--verify-deployment-only` 均通过。
  真实失效注入中，缺执行提供程序库、缺主库和替换执行提供程序库内容分别由启动校验、
  动态加载器和 SHA-256 校验在硬件初始化前拒绝。Python 严格语法、shell 语法、
  定向 `cppcheck` 和 `git diff --check` 通过。未访问 ROS 设备、真机串口或电机。
- **后续事项**： 无

---

<a id="lw-028"></a>

## [LW-028] 信号安全的 Sim2Sim 退出请求

**优先级**： P2 / 中
**状态**： resolved
**依赖**： LW-024

### 问题

Sim2Sim 的 `SIGINT` 处理器通过 `std::cout` 写入，并解引用全局 `RL_Real` 指针以访问 `Simulate` 对象。C++ 流、共享对象生命周期检查和一般对象访问均不具备异步信号安全性；在这些操作期间到达的信号可能引发死锁或与析构产生竞争。

### 证据

- `src/rl_sar/src/rl_sim_LW.cpp:74-75`
- `src/rl_sar/src/rl_sim_LW.cpp:1257-1277`

### 计划范围

- 将信号处理器限制为仅使用信号安全的通知机制。
- 在普通线程上下文中执行日志记录、执行器清零、ROS 关闭及对象访问。
- 与 LW-024 引入的可联结生命周期协调。

### 验收标准

- 安装的处理器不执行 C++ 流操作、分配、加锁或对象图访问。
- 在启动、稳定仿真和关闭期间反复注入 SIGINT，均确定性地正常退出，无释放后使用或死锁。
- 正常关闭窗口及安全关闭路径保留既有行为。

### 解决记录

- **解决时间**： 2026-08-12T18:21:42+08:00
- **提交**： `3a73aad7`
- **批准范围**： 删除 Sim2Sim 的异步 `SIGINT` 处理器和全局
  `RL_Real*`。主线程在创建 ROS、MuJoCo 或工作线程前阻塞 `SIGINT`，专用可
  联结线程通过 `sigtimedwait` 同步接收并把一次性请求交给普通线程协调器；
  对象构造前收到的请求会锁存，绑定后通过 `weak_ptr` 调用既有
  `RequestSimulationStop()`。ROS 仅管理 `SIGTERM`。渲染循环返回后才记录日志、
  关闭 ROS 并联结线程；等待线程停止后保持 `SIGINT` 阻塞至进程退出，消除关闭
  尾部恢复默认动作的竞争窗。窗口关闭和既有安全停止路径保持不变。
- **修改文件**： `src/rl_sar/src/rl_sim_LW.cpp`、
  `src/rl_sar/include/rl_sim_LW.hpp`、
  `src/rl_sar/library/core/simulation/lw_signal_shutdown.hpp`、
  `src/rl_sar/test/{test_lw_signal_shutdown.cpp,test_lw_sim_lifecycle_integration.py}`、
  `src/rl_sar/CMakeLists.txt`、`docs/LW_BUILD_DEPLOYMENT_CN.md`、
  `.learnings/LW_REAL_DEPLOYMENT_ISSUES.md`。
- **验证**： 当前 Debug 工作树完整构建并通过 37/37 CTest。新增信号
  测试覆盖构造前锁存、稳态重复请求、无信号有界关闭、关闭期间并发信号、回调
  异常保留、原掩码恢复和进程退出前持续阻塞；源码集成测试确认不再存在自定义
  异步处理器或全局对象指针，并保留 MuJoCo 待加载唤醒及窗口关闭测试。
  Release 和严格警告配置均构建 `rl_sim_LW` 与信号测试并通过 3 项定向 CTest；
  ASan/UBSan 信号测试连续 5 次通过，既有 MuJoCo 生命周期测试单独受控运行通过。
  真实 `rl_sim_LW` 进程连续 5 轮承受从稳态持续到进程退出的 SIGINT 洪泛，每轮
  679–733 次，均打印正常退出日志并返回 0。ThreadSanitizer 在本机运行时初始化
  即因 `unexpected memory mapping` 退出，未执行测试代码；定向 `cppcheck`、
  Python 严格语法和 `git diff --check` 通过。进程在进入 `main()` 前仍遵循
  操作系统默认 SIGINT 行为，该主函数执行前区间无法由程序内线程机制消除。未启动
  真机节点、未访问串口或电机。
- **后续事项**： 无

---

<a id="lw-029"></a>

## [LW-029] Sim2Sim 执行器模型与策略根目录的一致性

**优先级**： P2 / 中
**状态**： resolved
**依赖**： LW-010, LW-021

### 问题

`rl_sim_LW --policy-root` 重定向四个主要 LW 策略，但可选执行器网络模型仍通过编译时的 `POLICY_DIR` 加载。因此，同一进程可能把一个目录树的主要策略与另一个目录树的执行器模型混用，或者即使选定策略根完整，也在重定位后失败。

### 证据

- `src/rl_sar/src/rl_sim_LW.cpp:54-67`
- `src/rl_sar/src/rl_sim_LW.cpp:134-152`

### 计划范围

- 通过选定并验证的策略根解析每个与 Sim2Sim 策略相关的资产。
- 请求 `--use_actuator_net` 时，任一模型缺失或不兼容都明确失败。
- 除非另行批准，可选执行器网络不纳入实机部署范围。

### 验收标准

- 显式选择策略根后，任何 `--use_actuator_net` 资产路径均不回退到编译时 `POLICY_DIR`。
- 测试无需运行 GUI 即可证明重定位、缺失模型拒绝及模型选择一致性。
- 启动诊断标明精确解析后的执行器模型路径。

### 解决记录

- **解决时间**： 2026-08-12T19:32:04+08:00
- **提交**： `2b325977`
- **批准范围**： 将 Sim2Sim 可选执行器模型解析集中到独立组件，并以
  `SetPolicyRoot()` 保存的规范化策略根作为唯一来源。启用
  `--use_actuator_net` 时固定解析
  `LW/robot_lab/motors/{leg,foot}_actuator_net.pt`，逐项记录最终绝对路径，
  任一文件缺失、TorchScript 加载失败、拒绝 6 维输入、未产生恰好一个输出或
  产生非有限输出都会抛出带模型路径的启动错误；未启用该选项时仍不要求这两个
  可选资产。未修改真机执行路径。
- **修改文件**： `src/rl_sar/library/core/simulation/lw_actuator_models.{hpp,cpp}`、
  `src/rl_sar/src/rl_sim_LW.cpp`、`src/rl_sar/include/rl_sim_LW.hpp`、
  `src/rl_sar/test/{test_lw_actuator_models.cpp,test_lw_sim_lifecycle_integration.py}`、
  `src/rl_sar/CMakeLists.txt`、`README.md`、`README_CN.md`、
  `docs/LW_BUILD_DEPLOYMENT_CN.md`、`.learnings/LW_REAL_DEPLOYMENT_ISSUES.md`。
- **验证**： 当前 Debug 工作树完整构建并通过 38/38 CTest。新增无 GUI
  测试把仓库实际 leg/foot `.pt` 文件复制到临时搬迁策略根并从该根成功加载、
  预热，同时确认所选根缺少 foot 文件而编译期根仍有同名文件时会明确拒绝、
  不会回退；伪模型覆盖未加载、非 TorchScript、6 维输入不兼容、多输出和非有限
  输出。当前仓库两个原始模型也通过相同 6→1 契约。独立 Release 构建
  `rl_sim_LW` 和测试目标并通过 2/2 定向 CTest；新组件与测试在
  `-Wall -Wextra -Wpedantic -Werror` 下构建并通过。ASan/UBSan 测试逻辑完成，
  禁用泄漏检测后无地址或未定义行为报告；启用 LeakSanitizer 时测试已打印通过，
  但进程退出阶段在 Conda `libstdc++` 的 `std::filesystem` 路径中报告 707 字节
  残留。定向 `cppcheck`、Python 严格语法和 `git diff --check` 通过。未启动
  MuJoCo GUI、真机节点，未访问串口或电机。
- **后续事项**： 无

---

<a id="lw-030"></a>

## [LW-030] 命令受抑制时步态观测的一致性

**优先级**： P2 / 中
**状态**： resolved
**依赖**： LW-021, LW-025

### 问题

外部输入故障或安全监督器抑制输入时，推理观测发布零值 `commands`，但 `command_norm` 和运动步态相位仍根据抑制前的局部控制快照计算。因此，同一推理帧中可能同时包含零速度命令与非零运动步态相位。

### 证据

- `src/rl_sar/library/core/safety/lw_runtime_core.hpp:377-411`

### 计划范围

- 在应用全部抑制和外部故障规则后，从同一有效命令推导所有依赖命令的观测字段。
- 对有效非零命令保留当前步态相位时序。
- 保持实机、Sim2Sim 和主机分析器的推理行为一致。

### 验收标准

- 有效命令被置零时，同一推理帧始终产生文档约定的静止步态观测。
- 测试覆盖正常运动、手柄故障、时序降级和外部输入故障，不依赖偶然调度。
- 对有效命令，正常策略输入/输出一致性保持不变。

### 解决记录

- **解决时间**： 2026-08-12T19:50:43+08:00
- **提交**： `095f13be`
- **批准范围**： `LWRuntimeCore::runInferenceCycle()` 从每帧已发布的控制
  快照构造一次有效命令；外部输入故障或安全监督器输入抑制会统一清零有效
  `x/y/yaw`。策略 `commands`、命令范数、运动判定和最终 `gait_phase` 全部读取
  该有效命令，保证同一帧的零速度对应 `{0, 0}` 静止相位。按用户决定，内部
  相位时钟在短暂抑制期间保持连续而不重置；有效非零命令的阈值、推进速度和
  正弦/余弦计算顺序不变。实机、Sim2Sim 和主机测量器继续共用同一路径。
- **修改文件**： `src/rl_sar/library/core/safety/lw_runtime_core.hpp`、
  `src/rl_sar/test/test_lw_runtime_parity.cpp`、
  `docs/LW_BUILD_DEPLOYMENT_CN.md`、`.learnings/LW_REAL_DEPLOYMENT_ISSUES.md`。
- **验证**： 当前 Debug 工作树完整构建并通过 38/38 CTest。共享运行时
  测试先发布非零策略输入，再于推理前分别注入 `JoystickUnavailable`、
  `ControlTimingDegraded` 和 `external_input_fault=true`，确定性确认每种情况的
  当帧 `commands={0,0,0}`、`gait_phase={0,0}`；正常路径保持原命令和精确相位
  数值，临时外部抑制后的恢复帧证明内部时钟没有重置。现有真实 ONNX
  实机/Sim2Sim 推理奇偶性继续通过。独立 Release 构建成功链接
  `rl_real_LW`、`rl_sim_LW` 和 `lw_config_profiler`，运行时奇偶性与测量器
  集成测试 2/2 通过。完整严格 `-Werror` 构建被 LW-031 已登记的既有
  `ObservationBuffer -Wreorder` 阻断；仅将该类别降为非致命后，本次目标在
  `-Wall -Wextra -Wpedantic -Werror` 其余规则下构建并通过。ASan/UBSan、定向
  `cppcheck` 和 `git diff --check` 通过。未启动 MuJoCo GUI、真机节点，未访问
  串口或电机。
- **后续事项**： 无

---

<a id="lw-031"></a>

## [LW-031] 受维护 LW 构建的零警告基线

**优先级**： P2 / 低
**状态**： resolved
**依赖**： 无

### 问题

启用警告的构建暴露了维护代码中的警告，包括 `ObservationBuffer` 成员初始化顺序不匹配，以及范围循环测试将 `std::string` 引用绑定到临时转换结果。后者使现有严格 `-Werror` 构建在检查完全部目标前就中止。这削弱了编译器警告作为回归门禁的价值，也可能使新缺陷淹没在第三方 MuJoCo 警告噪声中。

### 证据

- `src/rl_sar/library/core/observation_buffer/observation_buffer.hpp:55-62`
- `src/rl_sar/library/core/observation_buffer/observation_buffer.cpp:14-22`
- `src/rl_sar/test/test_lw_fsm_transitions.cpp:66-71`

### 计划范围

- 修正维护中的 LW 源码和测试警告，不改变行为。
- 对维护目标应用严格警告设置，同时将随仓库提供的第三方代码视为独立范围的依赖。
- 在验证流程中增加可复现的零警告检查。

### 验收标准

- 维护中的 LW 源码和测试目标使用批准的警告集合及 `-Werror` 编译。
- 第三方警告不压制或掩盖维护代码失败。
- 常规构建及完整 CTest 套件持续通过。

### 解决记录

- **解决时间**： 2026-08-12T20:18:05+08:00
- **提交**： `769b6e89`
- **批准范围**： 修正维护代码中全部审计警告且不改变行为：按声明顺序
  初始化 `ObservationBuffer`，FSM 测试按值构造字符串，实机/Sim2Sim 的
  `LowCmd`/`LowState` 使用完整值初始化，并明确标记实机构造函数保留参数未
  使用。新增默认关闭的 `LW_STRICT_WARNINGS` CMake 选项，对 GCC/Clang 维护
  目标施加 `-Wall -Wextra -Wpedantic -Werror`，对 MSVC 使用 `/W4 /WX`。
  joystick 与 MuJoCo simulate 源码拆分为独立第三方代码库，硬件 SDK、joystick
  和 MuJoCo 头文件通过 `SYSTEM` 边界传播；仅第三方代码编译目标在严格模式关闭
  自身诊断，不修改第三方源码，也不豁免维护目标。新增临时目录一键严格构建与
  全量 CTest 脚本。
- **修改文件**： `src/rl_sar/CMakeLists.txt`、
  `src/rl_sar/library/core/observation_buffer/observation_buffer.cpp`、
  `src/rl_sar/include/{rl_real_LW.hpp,rl_sim_LW.hpp}`、
  `src/rl_sar/src/rl_real_LW.cpp`、
  `src/rl_sar/test/{test_lw_fsm_transitions.cpp,test_build_workflow.py}`、
  `scripts/validate_lw_strict_build.sh`、`README.md`、`README_CN.md`、
  `docs/LW_BUILD_DEPLOYMENT_CN.md`、`.learnings/LW_REAL_DEPLOYMENT_ISSUES.md`。
- **验证**： 初始完整 `-Wall -Wextra -Wpedantic` 审计统计 873 条诊断：
  维护代码包含 `-Wreorder`、`-Wrange-loop-construct`、聚合初始化和未使用参数，
  其余 850 余条来自随仓库提供的第三方 MuJoCo/joystick。修复后运行
  `scripts/validate_lw_strict_build.sh`，全新 Debug 严格构建的所有维护库、测试、
  `rl_real_LW`、`rl_sim_LW` 和 `lw_config_profiler` 均在 `-Werror` 下成功，第三方代码
  输出无警告噪声，随后 38/38 CTest 通过；脚本自动清理临时目录。普通 Debug
  全构建及 38/38 CTest 同样通过。独立 Release 成功构建实机、Sim2Sim、MuJoCo
  生命周期和 FSM 目标，3/3 定向 CTest 通过。构建工作流静态测试 13/13、定向
  `cppcheck`、Python 严格语法、shell 语法和 `git diff --check` 通过。未启动
  MuJoCo GUI、真机节点，未访问串口或电机。
- **后续事项**： 无

---

<a id="lw-032"></a>

## [LW-032] IMU 与 AHRS 的端到端有效性及时效性

**优先级**： P0 / 严重
**状态**： resolved
**依赖**： LW-002, LW-003, LW-005, LW-017

### 问题

项目内 FDILink 驱动可能发布一个带新时间戳的 `/imu` 消息，其姿态却未初始化，或来自无限陈旧的 AHRS 帧。其串口超时和首序列标志未初始化，各次串口读取也未在解析共享帧缓冲区前拒绝短读。实机控制器将收到这一组合 ROS 消息视为角速度和姿态都新鲜的证明。反馈验证会拒绝非有限四元数值，但接受 `[0, 0, 0, 0]` 这样的有限非四元数，而姿态转换会把它报告为安全的零横滚和零俯仰。

### 证据

- `src/fdilink_ahrs_ROS2/include/ahrs_driver.h:63-81`
- `src/fdilink_ahrs_ROS2/src/ahrs_driver.cpp:8-50`
- `src/fdilink_ahrs_ROS2/src/ahrs_driver.cpp:74-101`
- `src/fdilink_ahrs_ROS2/src/ahrs_driver.cpp:110-205`
- `src/fdilink_ahrs_ROS2/src/ahrs_driver.cpp:303-440`
- `src/fdilink_ahrs_ROS2/src/ahrs_driver.cpp:444-495`
- `src/rl_sar/src/rl_real_LW.cpp:614-695`
- `src/rl_sar/library/core/safety/lw_control_safety.hpp:87-119`
- `src/rl_sar/library/core/safety/lw_control_safety.hpp:243-280`
- `scripts/validate_lw_strict_build.sh:25-31`
- `src/rl_sar/test/test_lw_runtime_dependencies.py:22-34`

### 计划范围

- 保持第三方 FDLink 包不变。将其经过 CRC 和帧尾检查的 `/euler_angles` 发布路径作为独立 AHRS 存活事件，同时保留文档所述限制：ROS 话题不暴露串口读取数量、设备时间戳或序号。
- 在正常实机启动中，将 FDLink `/imu` 和 `/euler_angles` 重映射到明确不可信的内部话题；不允许原始 `/imu` 直接满足控制器就绪条件。
- 每个有限 AHRS 事件只授予一次短时有效授权，且仅用于紧随其后的一个 IMU 消息。拒绝缺失、已消费、时间回退或超龄授权，不无限刷新缓存姿态。
- 在就绪或姿态保护允许命令前，拒绝非有限、零模、缩放不合理或其他无效四元数，以及非有限角速度；归一化接受的四元数。
- 独立配置电机反馈时效性、可信 IMU 时效性和 IMU/AHRS 配对年龄。保留 100 ms 为明确未经批准的测量前占位值，不从设备名义 400 Hz 或策略 50 Hz 频率推断上限。
- 扩展悬吊硬件测量，记录原始 IMU、有效 AHRS、可信 IMU、配对年龄及双侧反馈分布；生成仅供评审的候选参数前，要求操作员分别提供安全上限。

### 验收标准

- 原始 FDLink `/imu` 不能直接产生控制器就绪样本；从未收到、无效、已消费或陈旧的 AHRS 授权均阻止其通过。
- 一个 AHRS 事件最多授权一个后续 IMU 消息，且必须位于配置的含端点配对年龄边界内。
- 包括零四元数在内的有限无效四元数会阻止命令激活；非有限姿态、欧拉角数据或角速度也在就绪前被拒绝。
- 启动时没有可信配对，继续保持现有仅失能等待状态；Ready 后丢失可信配对，在 `trusted_imu_timeout` 到期时锁存现有硬失能并关闭动作。
- 缺少原始 IMU、有效 AHRS、可信 IMU、配对样本或双侧反馈证明的硬件报告不能生成候选参数。
- 自动化测试无需打开物理串口设备或发送电机命令，即可验证守卫、独立超时、配置、启动重映射、测量器报告格式及分析器拒绝路径；完整严格构建持续通过。

### 解决记录

- **解决时间**： 2026-08-13T14:04:40+08:00
- **提交**： `0fe6d291`
- **批准范围**： 按用户明确选择不修改第三方
  `src/fdilink_ahrs_ROS2`。完整 LW 启动配置将 FDLink 的 `/imu`、
  `/euler_angles` 重映射为 `/fdilink/raw_imu`、`/fdilink/raw_euler`；
  `rl_sar` 自有守卫以有限 AHRS 事件一次性授权下一帧原始 IMU，限制配对
  时效，拒绝未授权、重复使用、时间倒退、非有限数据、零模或异常缩放四元数，
  并在交给就绪检查前归一化。电机反馈、可信 IMU、IMU/AHRS 配对分别使用
  `sensor_timeout`、`trusted_imu_timeout`、`imu_ahrs_pair_max_age`。硬件测算器
  复用同一守卫并输出三类 IMU/AHRS 分布及配对时延；分析器要求对应的独立
  操作员安全上限，报告结构版本升级至 v3。
- **修改文件**： `policy/LW/base.yaml`、`src/rl_sar/CMakeLists.txt`、
  `src/rl_sar/include/rl_real_LW.hpp`、
  `src/rl_sar/launch/rl_real_LW.launch.py`、
  `src/rl_sar/library/core/rl_sdk/lw_configuration_validation.cpp`、
  `src/rl_sar/library/core/safety/{lw_imu_ahrs_guard.hpp,sensor_readiness.hpp}`、
  `src/rl_sar/src/{rl_real_LW.cpp,lw_config_profiler.cpp}`、
  `src/rl_sar/scripts/profile_lw_runtime_config.py`、
  `src/rl_sar/test/{test_lw_imu_ahrs_guard.cpp,test_sensor_readiness.cpp,`
  `test_profile_lw_runtime_config.py,test_lw_config_profiler_integration.py,`
  `test_lw_real_startup_disable_integration.py}`、
  `docs/LW_BUILD_DEPLOYMENT_CN.md`、
  `.learnings/LW_REAL_DEPLOYMENT_ISSUES.md`。
- **验证**： 普通 Debug 全构建成功且 39/39 CTest 通过；全新
  `scripts/validate_lw_strict_build.sh` 在
  `-Wall -Wextra -Wpedantic -Werror` 下完成所有维护目标并通过 39/39 CTest；
  测量器/分析器定向 18/18 Python 测试、启动配置 Python 语法和加载构造、
  `git diff --check` 通过。未启动 ROS 节点、IMU、串口、MuJoCo GUI 或电机，
  `src/fdilink_ahrs_ROS2` 无任何修改。
- **已接受的限制**： 外部守卫能证明 FDLink 刚经过其 AHRS CRC/帧尾
  检查并发布事件，但第三方无 Header 的 `/euler_angles` 不暴露设备时间戳、
  序号或 `serial::read()` 实际长度，因此不能从 ROS 层独立证明每次底层读取均
  完整。按用户决定，允许少量重复旧姿态，并以一次性授权、配对时限、内容校验
  和可信输出超时阻止无限续命；若以后要求逐字节串口完整性证明，必须另行批准
  FDLink 最小接口补丁或 `rl_sar` 串口代理。
- **后续事项**： 当前两个新增 IMU 时限均为 100 ms 未测量占位值。
  正式部署前必须在目标机吊装采集结构版本 v3 硬件报告，由现场安全评审分别给出
  最大电机反馈时效、最大可信 IMU 时效、最大配对时延和最大控制间断，再审查候选；
  不得从名义 400 Hz 或 50 Hz 策略频率直接推定。

---

<a id="lw-033"></a>

## [LW-033] 策略输入的来源追溯与时效性

**优先级**： P1 / 高
**状态**： resolved
**依赖**： LW-007, LW-008, LW-011, LW-021, LW-032

### 问题

`LWPolicyInputSnapshot` 包含机器人状态和控制值，但没有采集时间或序号。推理循环根据推理周期而非输入快照设置策略输出的 `source_time`。因此，如果控制循环停滞而推理继续运行，基于同一陈旧机器人状态反复产生的输出仍可能看似新鲜。控制恢复后，`EvaluateLWPolicyOutput()` 可能接受其中一个输出，即使其源状态已经超过配置的输出年龄上限。

### 证据

- `src/rl_sar/library/core/rl_sdk/rl_sdk.hpp:222-237`
- `src/rl_sar/library/core/safety/lw_runtime_core.hpp:356-399`
- `src/rl_sar/library/core/safety/lw_runtime_core.hpp:452-459`
- `src/rl_sar/library/core/safety/lw_runtime_core.hpp:579-587`
- `src/rl_sar/library/core/rl_sdk/rl_sdk.cpp:17-43`
- `src/rl_sar/library/core/rl_sdk/rl_sdk.cpp:1213-1255`
- `policy/LW/base.yaml:23-38`

### 计划范围

- 为每个策略输入快照增加单调采集时间和序号/代际身份。
- 将输入来源信息传递到完整策略输出帧。
- 以状态采集时间定义时效性，拒绝重复、回退、未来时间或超龄的输入和输出。
- 使陈旧输入处理与现有按严重程度分级的安全策略以及控制循环时序降级行为协调。
- 测试控制生产者停滞而推理消费者持续运行、恢复、策略代际切换及年龄边界行为。

### 验收标准

- 基于旧状态快照新计算的输出，绝不会仅因最近运行过推理而被归类为就绪。
- 控制路径不能消费源状态超过批准上限或属于另一策略代际的输出。
- 测试确定性地复现控制停滞场景，并验证实机和 Sim2Sim 适配器批准的回退及恢复语义。
- 现有完整一致帧和运行时一致性测试持续通过。

### 解决记录

- **解决时间**： 2026-08-13T14:41:44+08:00
- **提交**： `71371e0d`
- **批准范围**： 每个 `LWPolicyInputSnapshot` 现在携带活动策略代际、全局
  单调序号以及 `GetState()` 完成时的单调时钟采集时间；推理线程按代际验证输入，
  对每个输入最多消费一次，重复输入不会推进策略帧、相位、历史、输出或进度。
  策略输出携带源输入序号与源状态采集时间，输入检查和推理完成后的二次检查均
  使用既有 `3 * dt * decimation` 数据年龄上限（当前配置为 60 ms），而不是以
  推理完成时间刷新年龄。输出传输和控制消费者拒绝缺失、回退或重复的新来源；
  200 Hz 控制循环仍允许在年龄窗口内保持同一个完整 50 Hz 输出。策略切换期间的
  旧代际输入和暂时重复输入只跳过；不完整、来源回退、未来时间或过期输入触发
  新的 `PolicyInputUnavailable` S2 Passive 阻尼锁存，必须重启，不改变既有硬失能
  分级或配置阈值。
- **修改文件**： `src/rl_sar/library/core/rl_sdk/rl_sdk.hpp`、
  `src/rl_sar/library/core/rl_sdk/rl_sdk.cpp`、
  `src/rl_sar/library/core/safety/lw_runtime_core.hpp`、
  `src/rl_sar/library/core/safety/lw_safety_policy.hpp`、
  `src/rl_sar/test/test_lw_policy_output_transport.cpp`、
  `src/rl_sar/test/test_lw_runtime_parity.cpp`、
  `src/rl_sar/test/test_lw_safety_policy.cpp`、
  `docs/LW_BUILD_DEPLOYMENT_CN.md`、
  `.learnings/LW_REAL_DEPLOYMENT_ISSUES.md`。
- **验证**： 普通 Debug 完整构建成功且 39/39 CTest 通过；全新
  `scripts/validate_lw_strict_build.sh` 在
  `-Wall -Wextra -Wpedantic -Werror` 下完成所有维护目标并通过 39/39 CTest；
  `git diff --check` 通过。定向测试覆盖 60 ms 年龄边界、未来/过期/重复/回退
  输入、重复和回退输出来源、控制生产者停滞而推理消费者继续运行、停滞后的 S2
  实机/Sim2Sim 一致性、重复输入后新输入恢复、策略代际切换，以及同一有效输出被
  多个控制调用保持使用。未启动 ROS 节点、IMU、串口、MuJoCo GUI 或电机，
  `src/fdilink_ahrs_ROS2` 无任何修改。
- **后续事项**： 无

---

<a id="lw-034"></a>

## [LW-034] 可信推理运行库的下载完整性

**优先级**： P1 / 高
**状态**： resolved
**依赖**： LW-020, LW-027

### 问题

推理运行时引导流程固定了版本字符串和 HTTPS URL，但未在解压前固定或验证可信归档摘要。结构与 ELF 架构验证可能接受具有预期布局的替换归档。随后，部署清单只对实际下载到的字节计算哈希；这可以发现后续修改，却无法证明这些字节来自批准的上游发行版本。

### 证据

- `scripts/download_inference_runtime.sh:62-69`
- `scripts/download_inference_runtime.sh:112-157`
- `scripts/download_inference_runtime.sh:202-259`
- `scripts/validate_inference_runtime.sh:15-100`
- `src/rl_sar/scripts/generate_lw_deployment_manifest.py:38-41`
- `src/rl_sar/scripts/generate_lw_deployment_manifest.py:194-208`

### 计划范围

- 为引导流程使用的每个受支持操作系统/架构归档固定一个经过评审的 SHA-256 摘要。
- 在解压或替换现有运行时前验证完整下载归档。
- 摘要数据缺失、格式错误或不匹配时采取拒绝策略，并安全清理不完整的临时产物。
- 使版本、架构、URL、预期摘要及已部署库哈希之间的关联可供评审。
- 使用合成的有效和篡改归档增加离线测试；常规测试套件不要求网络下载。

### 验收标准

- 下载的推理归档在摘要与所选平台的固定值匹配前，不得解压或安装。
- 归档改变一个字节，就必须在替换任何已批准运行时目录前被拒绝。
- 不受支持的平台/版本组合明确失败，不使用未经验证的回退。
- 部署生成继续绑定已安装运行时库，且所有引导流程测试无需网络访问即可通过。

### 解决记录

- **解决时间**： 2026-08-13T15:49:27+08:00
- **提交**： `1f36ba63`
- **批准范围**： 推理运行时支持矩阵收紧为 Linux x86_64 的 LibTorch
  2.3.0/ONNX Runtime 1.22.0，以及 Linux aarch64 的 ONNX Runtime 1.22.0；
  Darwin、Windows 和其他未审查组合不再使用回退 URL。清单固定运行时类型、
  版本、操作系统、规范化架构、精确官方 HTTPS URL、归档名/格式/根目录和完整归档
  SHA-256。三个摘要于 2026-08-13 从清单中的精确 PyTorch/ONNX Runtime 官方
  URL 流式读取完整归档并计算：LibTorch x86_64
  `f60009d2a74b6c8bdb174e398c70d217b7d12a4d3d358cd1db0690b32f6e193b`，
  ONNX x86_64
  `8344d55f93d5bc5021ce342db50f62079daf39aaafb5d311a451846228be49b3`，
  ONNX aarch64
  `bb76395092d150b52c7092dc6b8f2fe4d80f0f3bf0416d2f269193e347e24702`。
  下载器使用独立临时归档，摘要匹配前不解压；候选在同一文件系统的隔离目录完成
  安全路径、精确根目录、结构和 ELF 架构验证并写入来源证明后才替换结构损坏的
  目录。任一失败均清理候选并保留旧目录；结构有效但来源缺失、版本更高或来源
  不匹配的现有运行时会明确停止并要求单独审查升级，不自动覆盖。正式生产 CMake
  要求来源证明匹配仓库清单；部署清单升级为结构版本 v4，绑定批准的 ONNX
  归档名/URL/SHA-256、来源文件哈希及实际部署库哈希，运行时验证器同时与编译进
  二进制的 x86_64/aarch64 批准归档身份比对。用户 Python 环境中的包不受影响。
- **修改文件**： `.gitignore`、`README.md`、`README_CN.md`、
  `docs/LW_BUILD_DEPLOYMENT_CN.md`、
  `scripts/inference_runtime_archives.json`、
  `scripts/manage_inference_runtime.py`、
  `scripts/download_inference_runtime.sh`、`src/rl_sar/CMakeLists.txt`、
  `src/rl_sar/cmake/install_lw_deployment.cmake.in`、
  `src/rl_sar/scripts/generate_lw_deployment_manifest.py`、
  `src/rl_sar/library/core/deployment/lw_deployment_bundle.hpp`、
  `src/rl_sar/library/core/deployment/lw_deployment_bundle.cpp`、
  `src/rl_sar/test/test_inference_runtime_download_integrity.py`、
  `src/rl_sar/test/test_build_workflow.py`、
  `src/rl_sar/test/test_generate_lw_deployment_manifest.py`、
  `src/rl_sar/test/test_lw_deployment_bundle.cpp`、
  `.learnings/LW_REAL_DEPLOYMENT_ISSUES.md`。
- **验证**： 普通 Debug 完整构建成功且 40/40 CTest 通过；全新
  `scripts/validate_lw_strict_build.sh` 在
  `-Wall -Wextra -Wpedantic -Werror` 下完成全部维护目标并通过 40/40 CTest；
  Python 严格语法、Shell 语法和 `git diff --check` 通过。新增 7 项纯离线测试
  使用合成 TGZ/ELF 覆盖正确安装及精确来源、单字节归档篡改在解压前拒绝、候选
  结构失败、结构有效但无来源时拒绝自动替换、无效旧目录只在候选完全验证后
  替换、重复/不支持清单失败和三项生产支持矩阵。清单生成与 C++ 启动验证
  测试覆盖来源缺失/不匹配、未批准版本/归档、来源文件及部署库篡改。正常测试套件
  未执行网络下载；未启动 ROS 节点、IMU、串口、MuJoCo GUI 或电机。
- **已接受的限制**： 固定摘要是对指定官方 HTTPS 资产完整字节的项目审查
  信任锚，不是上游数字签名。以后升级版本必须单独审查新 URL 与摘要并重新完成
  全量构建、Sim2Sim 和部署验收，不能把更高版本静默视为当前批准版本。
- **后续事项**： 无

---

<a id="lw-035"></a>

## [LW-035] 生产启动文件的完整性

**优先级**： P2 / 中
**状态**： resolved
**依赖**： LW-010, LW-017, LW-019, LW-023, LW-027, LW-034

### 问题

生产操作说明通过 `rl_real_LW.launch.py` 启动机器人，该文件控制是否启动 FDILink 节点，以及是否启用键盘/调试通道。生产清单的精确运行时文件集合未包含已安装的 `rl_sar` 启动文件。因此，在生成部署包后更改或替换该文件，并不会使 `--verify-deployment-only` 失败，即使已验证的可执行文件可能以实质不同的依赖或参数启动。

### 证据

- `src/rl_sar/launch/rl_real_LW.launch.py:10-45`
- `src/rl_sar/CMakeLists.txt:1144-1149`
- `src/rl_sar/scripts/build_lw_deployment.sh:64-80`
- `src/rl_sar/scripts/build_lw_deployment.sh:145-149`
- `src/rl_sar/scripts/generate_lw_deployment_manifest.py:25-36`
- `src/rl_sar/library/core/deployment/lw_deployment_bundle.cpp:39-53`
- `src/rl_sar/test/test_lw_deployment_bundle.cpp:33-44`

### 计划范围

- 将实际安装的真机启动文件及其依赖的所有项目自有子启动文件纳入生产清单获准的运行时文件集合。
- 对这些文件保留符号链接、目录包含关系、精确集合和 SHA-256 检查。
- 为缺失、变更、额外添加和符号链接形式的启动资产补充生成器及运行时验证器测试。
- 开发专用和 Sim2Sim 启动资产继续排除在生产集合之外，除非另有充分理由。

### 验收标准

- 文档规定的生产启动路径通过密码学校验绑定到与 `rl_real_LW` 相同的源码提交和部署包。
- 必需启动文件缺失、被修改、逃逸出目录边界或为符号链接时，离线部署验证必须在访问硬件前失败。
- 干净的迁移后生产前缀仍能通过验证，并在该前缀内解析全部必需的 ROS 包。

### 解决记录

- **解决时间**： 2026-08-14T18:57:31+08:00
- **提交**： `f80b916d`
- **获准范围**： `LW_PRODUCTION_DEPLOYMENT=ON` 时只安装项目自有的
  `share/rl_sar/launch/rl_real_LW.launch.py`，不再把 Gazebo 启动文件 或
  `worlds` 带入正式前缀；开发构建保持原有安装范围。清单 的
  `runtime_files` 同时绑定该 启动文件 及
  `share/ament_index/resource_index/packages/rl_sar`，并继续绑定其引用的
  FDLink 启动文件。生成器和 C++ 离线验证器都要求 `share/rl_sar/launch`
  是前缀内的真实目录、只含批准的单一普通文件，且 清单 路径集合和
  SHA-256 完全匹配；缺失、修改、额外文件/目录、路径逃逸以及文件或目录符号
  链接均失败。生产构建在原始与迁移前缀内使用
  `PYTHONDONTWRITEBYTECODE=1 ros2 launch ... --show-args` 验证 `rl_sar`、
  FDLink 和 启动文件 参数解析而不启动节点。所有正式启动文档及构建输出均固定
  `PYTHONDONTWRITEBYTECODE=1`，避免 Python 在精确集合目录内生成或使用未绑定的
  `__pycache__`；省略时额外缓存会被完整性检查安全拒绝。清单 格式未变化，
  保持 格式版本 v4。
- **修改文件**： `README.md`、`README_CN.md`、
  `docs/LW_BUILD_DEPLOYMENT_CN.md`、`src/rl_sar/CMakeLists.txt`、
  `src/rl_sar/scripts/build_lw_deployment.sh`、
  `src/rl_sar/scripts/generate_lw_deployment_manifest.py`、
  `src/rl_sar/library/core/deployment/lw_deployment_bundle.cpp`、
  `src/rl_sar/test/test_build_workflow.py`、
  `src/rl_sar/test/test_generate_lw_deployment_manifest.py`、
  `src/rl_sar/test/test_lw_deployment_bundle.cpp`、
  `.learnings/LW_REAL_DEPLOYMENT_ISSUES.md`。
- **验证**： 三项定向 CTest
  `lw_deployment_bundle`、`lw_deployment_manifest_generator` 和
  `lw_build_workflow` 全部通过；普通 Debug 完整构建成功且 40/40 CTest 通过；
  全新 `scripts/validate_lw_strict_build.sh` 在
  `-Wall -Wextra -Wpedantic -Werror` 下构建成功并通过 40/40 CTest。第一次最终
  严格运行中与本项无关的 `lw_debug_publisher` 首次发布时序测试单次失败，其余
  39 项通过；未修改该测试，第二次全新严格构建和 40 项测试全部通过。另在隔离
  临时 Git 仓库中下载并核验 LW-034 批准的 ONNX Runtime，正式生成生产前缀；
  原始前缀和复制迁移前缀均完成 ROS 包解析、`--show-args` 与
  `--verify-deployment-only`，生产 启动文件 目录只含
  `rl_real_LW.launch.py` 且不存在 `worlds`。临时目录随后移入系统回收站。
  Python/Shell 语法和 `git diff --check` 通过；未启动 ROS 节点、IMU、串口、
  MuJoCo GUI 或电机，`src/fdilink_ahrs_ROS2` 无修改。
- **后续事项**： 无

---

<a id="lw-036"></a>

## [LW-036] 运行时执行器网络输出校验

**优先级**： P2 / 中
**状态**： resolved
**依赖**： LW-005, LW-013, LW-016, LW-021, LW-029

### 问题

可选的 Sim2Sim 执行器模型只在启动时用零输入检查一次，但之后每次推理结果都直接以 `output[0]` 索引，未验证其大小或有限性。空的动态结果可能造成越界访问；NaN 或无穷大可能进入 `actuator_net_tau_`，随后进入 MuJoCo 控制。该钩子在共享机器人命令校验之后运行，因此辅助力矩位于既有最终命令有限值校验边界之外。

### 证据

- `src/rl_sar/library/core/simulation/lw_actuator_models.cpp:63-103`
- `src/rl_sar/src/rl_sim_LW.cpp:666-749`
- `src/rl_sar/library/core/safety/lw_runtime_core.hpp:317-331`
- `src/rl_sar/src/rl_sim_LW.cpp:910-935`
- `src/rl_sar/test/test_lw_actuator_models.cpp:89-155`
- `src/rl_sar/test/test_lw_actuator_models.cpp:214-251`

### 计划范围

- 每次调用执行器模型时都验证输出数量及每个输出值，而不只在预热时验证。
- 将推理异常和无效动态输出统一交由一个明确且处置程度适当的 Sim2Sim 安全事件处理，阻止写入任何无效力矩。
- 在所有适配器钩子执行后、写入 MuJoCo 控制前，验证最终合成的执行器力矩。
- 增加有状态的模拟模型，使其通过预热后返回空结果、超长结果、非有限值或抛出异常。

### 验收标准

- 执行器模型结果的形状和值通过校验前，不得索引或应用该结果。
- 运行时结果为空、大小错误、含非有限值或抛出异常时，执行获准的安全动作，不产生未定义行为或无效 MuJoCo 控制值。
- 有效的可选执行器模型保留现有行为，真机命令路径保持不变。

### 解决记录

- **解决时间**： 2026-08-14T20:06:59+08:00
- **提交**： `381c3949`
- **获准范围**： 执行器模型启动预热与每次 Sim2Sim 运行时调用复用同一
  6 输入、单输出和有限性校验入口，捕获标准及未知推理异常。每个控制周期先
  使旧执行器网络 代际 失效，在独立候选帧中完成全部 腿部/脚部 关节推理，
  只有全部成功才原子提交力矩和 代际；策略输出无效或 S2 回退 时不会
  复用旧网络力矩。最终网络/前馈或 MuJoCo PD 力矩先在预分配缓冲区中全部形成，
  验证有限并限幅后才统一写入 `mj_data->ctrl`。新增仅由 Sim2Sim 发出的
  `SimulationActuatorCommandInvalid`，映射为 S4
  `HardDisableAndShutdown`，立即清零 MuJoCo 执行器并停止仿真；真机命令路径
  未修改。
- **修改文件**： `src/rl_sar/library/core/simulation/lw_actuator_models.hpp`、
  `src/rl_sar/library/core/simulation/lw_actuator_models.cpp`、
  `src/rl_sar/library/core/safety/lw_safety_policy.hpp`、
  `src/rl_sar/include/rl_sim_LW.hpp`、`src/rl_sar/src/rl_sim_LW.cpp`、
  `src/rl_sar/test/test_lw_actuator_models.cpp`、
  `src/rl_sar/test/test_lw_safety_policy.cpp`、
  `src/rl_sar/test/test_lw_sim_lifecycle_integration.py`、
  `.learnings/LW_REAL_DEPLOYMENT_ISSUES.md`。
- **验证**： 定向构建 `test_lw_actuator_models`、
  `test_lw_safety_policy` 和 `rl_sim_LW` 成功；三项定向 CTest 连续 20 轮通过。
  有状态 模拟模型 覆盖预热后有效输出、空输出、超长输出、NaN、Inf 和异常；
  事务缓冲测试覆盖失败时 代际 清除且不泄漏部分结果，最终力矩测试覆盖
  有限限幅和无效候选不产生部分写入。当前 Debug 完整构建成功且 40/40 CTest
  通过；全新 `scripts/validate_lw_strict_build.sh` 在
  `-Wall -Wextra -Wpedantic -Werror` 下完成全部维护目标并通过 40/40 CTest。
  定向 `cppcheck` 和 `git diff --check` 通过。未启动 ROS 节点、MuJoCo GUI、
  IMU、串口或电机，用户所有的未跟踪技能目录未修改。
- **后续事项**： 无

---

<a id="lw-037"></a>

## [LW-037] 完整的 Sim2Sim SIGTERM 与 ROS 退出流程

**优先级**： P2 / 中
**状态**： resolved
**依赖**： LW-024, LW-028

### 问题

Sim2Sim 屏蔽 SIGINT 并通过 `LWSigintWaiter` 处理，但将 SIGTERM 交给 ROS 信号处理器。如果 SIGTERM 使 `rclcpp::spin()` 正常返回，ROS 工作线程不会请求停止仿真；只有异常路径会调用 `RequestSimulationStop()`。主线程可能一直阻塞在 MuJoCo `RenderLoop()` 中，直到手动关闭窗口。

### 证据

- `src/rl_sar/library/core/simulation/lw_signal_shutdown.hpp:76-212`
- `src/rl_sar/src/rl_sim_LW.cpp:1265-1305`
- `src/rl_sar/src/rl_sim_LW.cpp:1319-1346`
- `src/rl_sar/test/test_lw_sim_lifecycle_integration.py:77-110`
- `src/rl_sar/test/test_lw_signal_shutdown.cpp:53-153`

### 计划范围

- 使每次 ROS 自旋正常退出、SIGTERM、SIGINT、ROS 关闭请求及 ROS 线程异常，都请求同一个幂等仿真停止操作。
- 保留信号安全的处理方式，以及 ROS、渲染、物理和业务工作线程既定的等待退出顺序。
- 增加不需要 GUI 的无界面生命周期测试，覆盖 SIGTERM/ROS 正常退出。

### 验收标准

- SIGTERM 能使 Sim2Sim 在有界时间内正常终止，无需操作窗口。
- ROS 执行器正常返回后，不能留下仍在运行的 `RenderLoop()`。
- 重复或并发的关闭请求保持幂等，全部工作线程均被等待退出，保存的工作线程错误仍会传播。

### 解决记录

- **解决时间**： 2026-08-14T20:28:39+08:00
- **提交**： `d0e1bad2`
- **获准范围**： 新增可测试的 ROS 工作线程 包装器，使正常 自旋 返回与异常
  都在保存 工作线程 异常后请求同一个 `LWSimShutdownCoordinator`；SIGINT 继续由
  同步等待线程安全处理，SIGTERM、显式 ROS 关闭、正常 自旋 返回、ROS
  工作线程 异常和主线程生命周期异常统一进入幂等仿真停止路径。保留现有
  `RequestExit()` 唤醒机制、工作线程等待退出 顺序和异常传播顺序，未修改第三方
  MuJoCo 代码。
- **修改文件**： `src/rl_sar/CMakeLists.txt`、
  `src/rl_sar/library/core/simulation/lw_signal_shutdown.hpp`、
  `src/rl_sar/src/rl_sim_LW.cpp`、
  `src/rl_sar/test/test_lw_signal_shutdown.cpp`、
  `src/rl_sar/test/test_lw_ros_shutdown.cpp`、
  `src/rl_sar/test/test_lw_sim_lifecycle_integration.py`、
  `.learnings/LW_REAL_DEPLOYMENT_ISSUES.md`。
- **验证**： 定向构建 `test_lw_signal_shutdown`、
  `test_lw_ros_shutdown` 和 `rl_sim_LW` 成功；正常 ROS 关闭、真实进程内
  SIGTERM、工作线程 异常及并发重复停止请求等四项定向 CTest 各连续 20 轮通过。
  当前 Debug 完整构建成功且 42/42 CTest 通过；全新
  `scripts/validate_lw_strict_build.sh` 在
  `-Wall -Wextra -Wpedantic -Werror` 下完成全部维护目标并通过 42/42 CTest。
  定向 `cppcheck`、Python 语法检查和 `git diff --check` 通过。未启动 MuJoCo
  GUI、真实机器人、IMU、串口或电机，用户所有的未跟踪技能目录未修改。
- **后续事项**： 无

---

<a id="lw-038"></a>

## [LW-038] 内存分配有界的实机控制周期

**优先级**： P2 / 中
**状态**： resolved
**依赖**： LW-007, LW-011, LW-013, LW-031, LW-033

### 问题

200 Hz 真机控制路径反复将 YAML 值解码为新的向量。`GetState()` 对每个自由度都请求三次 `joint_mapping`，命令下发和 FSM 插值也会在控制周期内请求容器类型的配置值。此外，快照发布还会在互斥锁保护下复制由向量承载的机器人状态。尽管已有时序监控和严格构建门禁，这些操作仍将分配器和锁的延迟引入了确定性循环。

### 证据

- `src/rl_sar/library/core/rl_sdk/rl_sdk.hpp:160-176`
- `src/rl_sar/src/rl_real_LW.cpp:699-703`
- `src/rl_sar/src/rl_real_LW.cpp:717-758`
- `src/rl_sar/fsm_robot/fsm_LW.hpp:86-98`
- `src/rl_sar/fsm_robot/fsm_LW.hpp:160-171`
- `src/rl_sar/library/core/safety/lw_runtime_sync.hpp:10-42`
- `src/rl_sar/library/core/safety/lw_runtime_core.hpp:579-587`

### 计划范围

- 在工作线程启动前，将不可变的基础/策略配置解码并验证后存入持久保留的强类型存储。
- 从真机控制和 FSM 执行路径中移除重复的 YAML 查找和容器构造。
- 跨线程状态传输复用有界存储，或记录并测试一种可证明有界的替代方案。
- 增加分配计数或等效的确定性回归测试；修改部署时序阈值前，使用既有悬吊分析工具测量目标主机上的影响。

### 验收标准

- 启动及策略激活后，稳态真机控制迭代不会因配置读取或状态传输而分配内存。
- 不在命令截止时间路径中引入无界的锁等待。
- 缓存值始终绑定到当前已验证的策略代际，并在获准的策略切换时原子刷新。
- 运行时一致性、安全行为和完整严格测试持续通过；提出任何阈值建议前，记录目标主机时序证据。

### 解决记录

- **解决时间**： 2026-08-14T21:20:17+08:00
- **提交**： `6c2db924`
- **获准范围**： 将 基础 和 策略 YAML 在校验阶段解码为强类型、只读运行时配置，
  并把 策略 配置与不可变定义及激活 代际 原子绑定；真实控制、LW FSM、
  共享推理运行时和配置 分析工具 改用缓存值。策略输入状态传输复用预分配快照，
  控制侧使用非阻塞 `tryPublish()`，仅成功发布时递增序号，争用或跳帧继续由现有
  新鲜度与代际检查处理。增加稳定态分配计数、确定性锁争用、配置解码和代际绑定
  回归测试；未修改时序阈值，未启动硬件、ROS、串口或电机。
- **修改文件**： `src/rl_sar/CMakeLists.txt`、
  `src/rl_sar/fsm_robot/fsm_LW.hpp`、
  `src/rl_sar/library/core/rl_sdk/lw_configuration_validation.cpp`、
  `src/rl_sar/library/core/rl_sdk/lw_configuration_validation.hpp`、
  `src/rl_sar/library/core/rl_sdk/rl_sdk.cpp`、
  `src/rl_sar/library/core/rl_sdk/rl_sdk.hpp`、
  `src/rl_sar/library/core/safety/lw_runtime_core.hpp`、
  `src/rl_sar/library/core/safety/lw_runtime_sync.hpp`、
  `src/rl_sar/src/lw_config_profiler.cpp`、
  `src/rl_sar/src/rl_real_LW.cpp`、
  `src/rl_sar/src/rl_sim_LW.cpp`、
  `src/rl_sar/test/test_lw_allocation_bound.cpp`、
  `src/rl_sar/test/test_lw_configuration_validation.cpp`、
  `src/rl_sar/test/test_lw_runtime_parity.cpp`、
  `src/rl_sar/test/test_lw_runtime_sync.cpp`、
  `.learnings/LW_REAL_DEPLOYMENT_ISSUES.md`。
- **验证**： 定向维护目标与 `rl_real_LW`、`rl_sim_LW`、
  `lw_config_profiler` 构建成功；`lw_allocation_bound`、`lw_runtime_sync`、
  `lw_configuration_validation`、`lw_runtime_parity`、`lw_fsm_transitions` 和
  `lw_control_safety` 各连续 20 轮通过，其中稳定态 10,000 次配置访问及策略输入
  快照传输的分配计数为零，锁争用测试验证控制侧发布立即返回且恢复后快照一致，
  代际测试验证切换时强类型定义原子替换。Debug 完整套件 43/43 通过；最终
  `scripts/validate_lw_strict_build.sh` 在
  `-Wall -Wextra -Wpedantic -Werror` 下完成全部目标并通过 43/43 CTest；定向
  `cppcheck`（仅精确抑制未修改的 `CSVInit(std::string)` 既有
  `passedByValue` 提示）和 `git diff --check` 通过。仅在宿主机运行的分析工具 实际加载、
  预热并运行 4 个正式 ONNX 策略各 0.2 秒，报告为 `failed=false`、
  `commands_sent=none`。没有依据该短时宿主机结果调整阈值；未来任何时序阈值建议
  仍须先取得目标 Jetson/硬件证据。
- **后续事项**： 无

---

<a id="lw-039"></a>

## [LW-039] 执行器模型路径限制在策略根目录内

**优先级**： P2 / 低
**状态**： resolved
**依赖**： LW-010, LW-029

### 问题

`ResolveLWActuatorModelPaths()` 会规范化选定的策略根目录，但对模型路径只做词法拼接，并调用会跟随符号链接的 `is_regular_file()` 检查。因此，以符号链接提供的腿部或脚部模型可能解析到选定策略根目录之外，而诊断信息仍显示根目录内的词法路径。

### 证据

- `src/rl_sar/library/core/simulation/lw_actuator_models.cpp:13-29`
- `src/rl_sar/library/core/simulation/lw_actuator_models.cpp:32-60`
- `src/rl_sar/test/test_lw_actuator_models.cpp:157-212`

### 计划范围

- 拒绝可选执行器模型路径中任一组件为符号链接的情况。
- 加载前规范化每个模型路径，并证明其位于选定的规范化策略根目录内。
- 测试直接文件符号链接、中间目录符号链接，以及逃逸到外部其他方面均有效的 TorchScript 文件的情况。

### 验收标准

- 无法通过符号链接或路径别名加载选定策略根目录之外的可选执行器模型。
- 失败诊断标明被拒绝的词法路径和解析路径，不得静默回退到编译时策略目录。
- 正常迁移后的真实目录仍能加载当前两个模型。

### 解决记录

- **解决时间**： 2026-08-15T12:37:03+08:00
- **提交**： `eb64b3d8`
- **获准范围**： 继续以所选规范化 策略根目录 为唯一边界；对固定 腿部/脚部
  模型相对路径逐级执行非跟随式 `symlink_status()` 检查，拒绝最终模型文件和任一
  中间目录的符号链接。每个模型必须成功规范化、通过按路径组件比较的根目录包含
  检查，并且最终节点是普通非链接文件后才返回。错误同时报告词法路径、解析后
  路径（无法解析时报告明确原因）及命中的链接组件；机器人名额外拒绝 `.`、`..`
  和带父路径的别名。未增加编译期 `POLICY_DIR` 回退，未改变模型 6→1 契约、
  推理输出或真机路径。
- **修改文件**：
  `src/rl_sar/library/core/simulation/lw_actuator_models.cpp`、
  `src/rl_sar/test/test_lw_actuator_models.cpp`、
  `.learnings/LW_REAL_DEPLOYMENT_ISSUES.md`。
- **验证**： `test_lw_actuator_models` 与 `rl_sim_LW` 定向构建成功；
  `lw_actuator_models` 连续 20 轮通过，覆盖指向策略根外有效 TorchScript 的直接
  文件链接、中间目录链接、`..` 路径别名、缺失模型拒绝，以及普通搬迁目录中
  两个当前模型的成功解析、加载和 6→1 预热。完整 Debug 构建成功并通过 43/43
  CTest；全新 `scripts/validate_lw_strict_build.sh` 在
  `-Wall -Wextra -Wpedantic -Werror` 下完成全部目标并通过 43/43 CTest；定向
  `cppcheck` 和 `git diff --check` 通过。未启动 MuJoCo GUI、ROS、真机、串口或
  电机，用户未跟踪技能目录保持未修改。
- **后续事项**： 无

---

<a id="lw-040"></a>

## [LW-040] 安全的 RL 多态析构约定

**优先级**： P2 / 低
**状态**： resolved
**依赖**： LW-015, LW-024, LW-031

### 问题

`RL` 是具有虚运行时方法的多态基类，但其析构函数不是虚函数。目前未发现通过 `RL*` 删除对象的情况，因此这不是正在发生的生命周期故障；不过，文档规定的扩展边界允许未来的所有者通过基类销毁派生的真机或 Sim2Sim 对象，从而跳过其线程、串口或仿真清理。

### 证据

- `src/rl_sar/library/core/rl_sdk/rl_sdk.hpp:298-302`
- `src/rl_sar/library/core/rl_sdk/rl_sdk.hpp:320-365`
- `src/rl_sar/include/rl_real_LW.hpp`
- `src/rl_sar/include/rl_sim_LW.hpp:38-47`

### 计划范围

- 明确并确保所有权/析构契约安全，通常采用虚的 noexcept 析构函数，除非选择并强制实施更严格的非多态所有权限制。
- 验证派生类析构函数仍按要求的顺序执行现有的有界关闭流程。
- 为所选契约增加有针对性的编译/运行时回归测试。

### 验收标准

- 通过受支持的基类所有权类型删除 LW 运行时对象，不可能引发未定义行为或清理不完整。
- 除获准的析构契约外，变更不改变运行时行为、对象所有权或扩展注册。
- 严格警告检查、生命周期测试及完整 CTest 套件持续通过。

### 解决记录

- **解决时间**： 2026-08-15T12:51:07+08:00
- **提交**： `267fb263`
- **获准范围**： 将多态 `RL` 基类的公开析构函数改为
  `virtual noexcept = default`，真实和 Sim2Sim 的 `RL_Real` 析构函数显式声明为
  `noexcept override`，不改变析构函数体、对象创建方式、所有权或扩展注册。新增
  独立编译/运行时测试，静态验证虚析构和 `noexcept` 契约，并通过
  `std::unique_ptr<RL>` 销毁轻量派生探针，确认派生析构函数体及派生成员清理均
  完整执行。既有真实节点关门、工作线程 停止、最终失能顺序和 Sim2Sim 工作线程、
  物理生命周期停止顺序保持不变。
- **修改文件**： `src/rl_sar/CMakeLists.txt`、
  `src/rl_sar/include/rl_real_LW.hpp`、
  `src/rl_sar/include/rl_sim_LW.hpp`、
  `src/rl_sar/library/core/rl_sdk/rl_sdk.hpp`、
  `src/rl_sar/src/rl_real_LW.cpp`、
  `src/rl_sar/src/rl_sim_LW.cpp`、
  `src/rl_sar/test/test_lw_rl_destruction.cpp`、
  `.learnings/LW_REAL_DEPLOYMENT_ISSUES.md`。
- **验证**： `test_lw_rl_destruction`、`rl_real_LW` 和 `rl_sim_LW`
  定向构建成功；`lw_rl_destruction`、`lw_signal_shutdown`、
  `lw_sim_lifecycle_integration`、`lw_mujoco_lifecycle` 和
  `lw_real_startup_disable_integration` 各连续 20 轮通过，验证基类所有权虚派发、
  派生成员清理，以及真实/Sim2Sim 原有有界停止顺序。完整 Debug 构建成功并通过
  44/44 CTest；全新 `scripts/validate_lw_strict_build.sh` 在
  `-Wall -Wextra -Wpedantic -Werror` 下完成全部目标并通过 44/44 CTest；定向
  `cppcheck` 和 `git diff --check` 通过。未启动 ROS 节点、MuJoCo GUI、真机、
  串口或电机，用户未跟踪技能目录保持未修改。
- **后续事项**： 无

---

<a id="lw-041"></a>

## [LW-041] 按需启用 Sim2Sim 绘图发布

**优先级**： P2 / 低
**状态**： resolved
**依赖**： LW-014, LW-021, LW-031

### 问题

Sim2Sim 头文件无条件定义 `PLOT`，因此每个受维护的 `rl_sim_LW` 构建都会创建一个 `/joint_states` 发布器和 2 ms 墙钟定时器。这使高频诊断工作成为默认仿真运行时的一部分，即使没有消费者需要该话题，也可能干扰一致性、时序和性能分析运行。

### 证据

- `src/rl_sar/include/rl_sim_LW.hpp:4-10`
- `src/rl_sar/include/rl_sim_LW.hpp:119-124`
- `src/rl_sar/src/rl_sim_LW.cpp:223-232`
- `src/rl_sar/src/rl_sim_LW.cpp:278-432`

### 计划范围

- 将头文件中始终开启的宏替换为默认关闭、需显式启用的 Sim2Sim 运行时或构建选项。
- 选项关闭时，不创建发布器、定时器、消息缓冲区或回调。
- 将操作员状态报告与高频绘图遥测分离。
- 为关闭和启用两种模式增加配置及生命周期测试。

### 验收标准

- 默认启动 Sim2Sim 时不执行 500 Hz 绘图发布工作。
- 通过文档说明的显式启用方式恢复既有绘图话题和载荷。
- 真机调试发布及共享控制/安全一致性保持不变。

### 解决记录

- **解决时间**： 2026-08-15T13:17:13+08:00
- **提交**： `476e112a`
- **获准范围**： 删除 Sim2Sim 头文件中始终启用的 `PLOT` 宏和未使用的
  matplotlib/绘图循环 声明，新增默认关闭的 `--enable-plot` 运行时开关；
  启用时默认 100 Hz，并允许通过 `--plot-rate-hz <integer>` 选择 1–200 Hz。
  频率参数必须与开关同时使用，缺失、非整数、越界或重复冲突值在启动时
  明确拒绝，参数顺序不受限制。默认模式不分配绘图快照缓冲、不创建 发布器
  或 定时器，也不安装控制周期快照回调；显式启用后恢复既有
  `/LW_joint_states` 话题 和 载荷，启动日志输出有效 话题/频率。100 ms
  操作员状态定时器 保持独立，真机 绘图/调试 路径及共享控制、安全逻辑
  不变。
- **修改文件**： `README.md`、`README_CN.md`、
  `docs/LW_BUILD_DEPLOYMENT_CN.md`、`docs/LW_QUICK_START_CN.md`、
  `src/rl_sar/CMakeLists.txt`、
  `src/rl_sar/include/rl_sim_LW.hpp`、
  `src/rl_sar/library/core/simulation/lw_sim_plot_config.hpp`、
  `src/rl_sar/src/rl_sim_LW.cpp`、
  `src/rl_sar/test/test_lw_sim_lifecycle_integration.py`、
  `src/rl_sar/test/test_lw_sim_plot_config.cpp`、
  `.learnings/LEARNINGS.md`、`.learnings/LW_REAL_DEPLOYMENT_ISSUES.md`。
- **验证**： `test_lw_sim_plot_config` 与 `lw_sim_lifecycle_integration` 各连续
  20 轮通过，覆盖默认关闭、显式启用、1–200 Hz 边界、无效或冲突参数拒绝、
  绘图资源按需创建和 操作员状态 独立性。完整 Debug 构建成功并通过
  45/45 CTest；全新 `scripts/validate_lw_strict_build.sh` 在
  `-Wall -Wextra -Wpedantic -Werror` 下完成全部目标并通过 45/45
  CTest；定向 `cppcheck`、Python 生命周期集成检查 和 `git diff --check`
  通过。README、完整部署说明和快速开始文档中的 绘图 参数名称、默认值、
  范围和约束已交叉核对。未启动 MuJoCo GUI、ROS 节点、真机、串口或电机，
  用户未跟踪技能目录保持未修改。
- **后续事项**： 无

---

<a id="lw-042"></a>

## [LW-042] 非阻塞且保证源数据时效的实机调试遥测

**优先级**： P2 / 中
**状态**： resolved
**依赖**： LW-007, LW-011, LW-014, LW-038

### 问题

可选启用的真机调试发布器不在默认控制路径中，但其启用路径未完全满足 LW-014 记录的非阻塞契约。200 Hz 控制回调调用 `publishSnapshot()`，该方法使用阻塞的 `LWSnapshotBuffer::publish()`，而 ROS 定时器也通过同一个互斥锁读取。因此，被抢占的读取线程可能延迟控制线程。

发布器定时器还固定为 4 ms（250 Hz），快于 5 ms（200 Hz）的源控制周期。缓冲区不携带源序号或采集时间，`publishOnce()` 可能重复读取同一快照，并每次赋予新的发布时间戳。因此，消费者可能将重复的控制数据视为看似新鲜的 250 Hz 样本，同时在真机调试期间产生可避免的 ROS 工作。启动接口只提供启用开关，现有测试则使用一小时定时器手动驱动 `publishOnce()`，因此未覆盖争用、重复抑制或生产运行频率。

### 证据

- `policy/LW/base.yaml:1-3`
- `src/rl_sar/src/rl_real_LW.cpp:229-247`
- `src/rl_sar/src/rl_real_LW.cpp:541-593`
- `src/rl_sar/library/core/debug/lw_debug_publisher.cpp:137-160`
- `src/rl_sar/library/core/safety/lw_runtime_sync.hpp:14-43`
- `src/rl_sar/launch/rl_real_LW.launch.py:39-48`
- `src/rl_sar/test/test_lw_debug_publisher.cpp:35-44`
- `src/rl_sar/test/test_lw_debug_publisher.cpp:158-187`

### 计划范围

- 增加有文档说明的整数 ROS/启动参数，用于设定真机调试发布频率，默认 50 Hz，接受 1 到 200 Hz。既有启用开关保持默认关闭，启用后报告实际话题/频率。
- 使控制侧快照交接严格非阻塞；发生争用时必须丢弃调试快照，不得等待定时器持有的互斥锁。
- 为每个被接受的快照附加单调递增的源序号（必要时增加采集时间元数据），仅发布比上次已发布帧更新的源帧，避免将旧样本重新打时间戳后当作新遥测。
- 保留既有 `/LW_joint_states` 话题、载荷映射、一致的快照边界、深度为一的实时发布器和逐消息时间戳。
- 增加针对配置、争用、新鲜度、生命周期和启动文件的测试，不启动真机节点，也不访问 ROS 硬件、串口或电机。同步全部面向操作员的参数文档。

### 验收标准

- 真机调试关闭时，无论未启用的频率配置如何，都不存在调试发布器、定时器、快照交接或控制周期复制。
- 真机调试启用时，200 Hz 控制回调绝不等待调试消费者；锁争用仅表现为丢弃调试样本。
- 实际调试频率可显式配置为 1 到 200 Hz，默认 50 Hz；无效值在工作循环开始前使启动失败。
- 定时器回调对每个被接受的源序号最多发布一次，不会为未变化的控制快照反复刷新时间戳并将其作为新鲜数据。
- 既有话题和载荷保持兼容，共享控制、安全、启动失能、操作员状态和 Sim2Sim 行为保持不变。

### 解决记录

- **解决时间**： 2026-08-15T14:18:41+08:00
- **提交**： `205cc70d`
- **获准范围**： 为真机 启动文件/ROS 节点新增默认 50 Hz、只接受 1–200
  整数的 `debug_publish_rate_hz` 参数，并在任何 工作线程 启动前无条件校验；保持
  `enable_debug_publisher` 默认关闭，关闭时不创建 发布器、定时器 或快照交接，
  也不在控制周期复制调试数据。启用时将控制侧快照交接改为 `tryPublish()`，锁
  争用立即丢弃调试帧而不等待；成功交接的帧携带单调源序号，定时器 只发布比上次
  已发序号更新的最新帧，避免给未变化数据重复刷新时间戳。保留既有
  `/LW_joint_states`、26 字段 载荷、深度为一的实时发布器、完整快照
  边界和发布时钟时间戳；Sim2Sim、控制、安全、启动失能和 操作员状态
  行为不变。四份操作文档已同步参数、边界和丢帧语义。
- **修改文件**： `README.md`、`README_CN.md`、
  `docs/LW_BUILD_DEPLOYMENT_CN.md`、`docs/LW_QUICK_START_CN.md`、
  `src/rl_sar/include/rl_real_LW.hpp`、
  `src/rl_sar/launch/rl_real_LW.launch.py`、
  `src/rl_sar/library/core/debug/lw_debug_publisher.cpp`、
  `src/rl_sar/library/core/debug/lw_debug_publisher.hpp`、
  `src/rl_sar/src/rl_real_LW.cpp`、
  `src/rl_sar/test/test_lw_debug_publisher.cpp`、
  `src/rl_sar/test/test_lw_real_startup_disable_integration.py`、
  `.learnings/LW_REAL_DEPLOYMENT_ISSUES.md`。
- **验证**： `lw_debug_publisher`、`lw_runtime_sync` 和
  `lw_real_startup_disable_integration` 各连续 20 轮通过，覆盖默认关闭、
  1/50/200 Hz 有效边界、负数/0/201 无效边界、非阻塞 `tryPublish` 接线、同一
  源帧至多发布一次、新源帧继续发布、载荷 和时间戳兼容，以及参数校验早于
  工作线程 启动。完整 Debug 构建成功并通过 45/45 CTest；全新
  `scripts/validate_lw_strict_build.sh` 在 `-Wall -Wextra -Wpedantic -Werror`
  下完成全部目标并通过 45/45 CTest。定向 `cppcheck`、Python 语法、启动文件
  `--show-args`、四份文档参数交叉核对和 `git diff --check` 通过。未启动真机
  节点、AHRS、串口、电机或 MuJoCo GUI；用户未跟踪技能目录
  保持未修改。
- **后续事项**： 无

---

<a id="lw-043"></a>

## [LW-043] 移除 Sim2Sim 执行器模型运行路径并保留离线训练

**优先级**： P2 / 中
**状态**： resolved
**依赖**： LW-020, LW-021, LW-036

### 问题

可选的 `rl_sim_LW --use_actuator_net` 路径用 TorchScript 执行器网络输出替换选定的 MuJoCo PD 加前馈力矩。这形成了第二条 Sim2Sim 执行路径，并且仅为这一可选功能保留了 C++ LibTorch 构建、下载、校验和运行时依赖。项目仍需要独立的 Python 训练及评估流程和两个已跟踪的 `.pt` 资产，但这些离线资产并不要求仿真器加载或执行 TorchScript。

### 证据

- `src/rl_sar/src/rl_sim_LW.cpp`
- `src/rl_sar/include/rl_sim_LW.hpp`
- `src/rl_sar/library/core/simulation/lw_actuator_models.{hpp,cpp}`
- `src/rl_sar/library/core/inference_runtime/inference_runtime.{hpp,cpp}`
- `src/rl_sar/CMakeLists.txt`
- `scripts/download_inference_runtime.sh`
- `src/rl_sar/scripts/actuator_net.py`
- `policy/LW/robot_lab/motors/{leg,foot}_actuator_net.pt`

### 计划范围

- 让 Sim2Sim 的所有关节都使用既有 MuJoCo PD 加前馈力矩路径，并在启动时明确拒绝已移除的 `--use_actuator_net` 选项。
- 将最终有限值和力矩限幅校验移入独立于执行器模型的模块，继续保证在任何 MuJoCo 控制数组修改前完成校验。
- 移除 C++ LibTorch 后端及其构建/下载/校验接口，保留 ONNX Runtime 策略后端。
- 保留 Python 执行器模型训练/评估脚本、其安装规则、Python `torch` 用法和两个已跟踪的模型资产，均不作修改。
- 更新受维护的测试和当前操作文档，不重写历史已解决问题记录。

### 验收标准

- 受维护的 Sim2Sim 代码均不能加载执行器模型或替换 PD 加前馈力矩；旧选项在 ROS、MuJoCo 或工作线程启动前失败。
- 无效的最终候选力矩仍会触发 `SimulationActuatorCommandInvalid`，且发生在任何 `mj_data->ctrl` 写入之前。
- 受维护的 C++ 构建和运行时管理工具只使用 ONNX，不需要 LibTorch。
- 训练脚本、其安装规则及两个 `.pt` 文件逐字节保持不变，并仍可用于 Python 训练/评估流程。
- 干净构建、完整 CTest 套件及严格构建均成功完成。

### 解决记录

- **解决时间**： 2026-08-16T12:35:00+08:00
- **提交**： `37188881`
- **获准范围**： 删除 `rl_sim_LW --use_actuator_net` 对 MuJoCo 底层力矩的
  运行时接管，所有关节统一使用 PD+前馈；旧参数在任何 ROS、MuJoCo
  或 工作线程 启动前明确拒绝。将最终力矩有限值/限幅/事务性写入校验迁入
  与执行器模型无关的 `lw_sim_torque_validation`，并删除已无消费者的共享
  `before_command_delivery` 钩子。C++ 推理、CMake、下载、来源清单和
  验证工具收敛为 仅使用 ONNX；Python 训练/评估脚本、安装规则以及两个
  `.pt` 资产保持不变。当前 764 MB LibTorch 目录未删除，已可恢复地迁至
  `/home/lfr/rl_sar-runtime-backups/20260816-actuator-runtime-retirement/libtorch`，
  不覆盖既有备份。
- **修改文件**： `.gitignore`、`README.md`、`build.sh`、
  `docs/{LW_BUILD_DEPLOYMENT_CN.md,LW_QUICK_START_CN.md}`、
  `scripts/{download_inference_runtime.sh,inference_runtime_archives.json,manage_inference_runtime.py,validate_inference_runtime.sh}`、
  `src/rl_sar/CMakeLists.txt`、`src/rl_sar/include/rl_sim_LW.hpp`、
  `src/rl_sar/library/core/inference_runtime/inference_runtime.{hpp,cpp}`、
  `src/rl_sar/library/core/safety/lw_runtime_core.hpp`、
  `src/rl_sar/library/core/simulation/{lw_actuator_models.hpp,lw_actuator_models.cpp,lw_sim_plot_config.hpp,lw_sim_torque_validation.hpp,lw_sim_torque_validation.cpp}`、
  `src/rl_sar/src/{rl_sim_LW.cpp,rl_real_LW.cpp,lw_config_profiler.cpp}`、
  `src/rl_sar/test/{test_build_workflow.py,test_inference_runtime.cpp,test_inference_runtime_architecture.py,test_inference_runtime_download_integrity.py,test_lw_actuator_models.cpp,test_lw_runtime_parity.cpp,test_lw_sim_lifecycle_integration.py,test_lw_sim_plot_config.cpp,test_lw_sim_torque_validation.cpp}`、
  `.learnings/{LEARNINGS.md,LW_REAL_DEPLOYMENT_ISSUES.md}`。
- **验证**： 修改前基线 45/45 CTest 通过，其中旧 LibTorch 测试已对
  当前两个 TorchScript 模型完成加载及 6 维输入/单值输出契约验证。
  定向 7 项 CTest 通过；旧 `--use_actuator_net` 实际运行在 GUI/ROS 启动前
  以非零状态和明确诊断退出。执行全量 `./build.sh --clean` 后 `./build.sh`
  从零构建 6 个包成功，干净构建 45/45 CTest 通过；
  `scripts/validate_lw_strict_build.sh` 在 `-Wall -Wextra -Wpedantic -Werror`
  下构建全部维护目标并通过 45/45 CTest。`ldd` 确认
  `rl_sim_LW`、`rl_real_LW` 和 `lw_config_profiler` 均不依赖
  `libtorch`/`libc10`。Bash/Python 语法、ONNX 架构/下载完整性、通用力矩
  校验、源码/文档交叉检查和 `git diff --check` 通过。训练脚本和两个
  模型 SHA-256 与基线完全一致，安装后脚本存在；当前机器的现有
  Python 环境均未安装 `torch`，因此未新增依赖也未重复 Python 前向推理。
  未启动真机节点、AHRS、串口、电机或 MuJoCo GUI；用户未跟踪技能目录保持未修改。
- **后续事项**： 无

---

<a id="lw-044"></a>

## [LW-044] 完整接收 FDILink 帧并显式报告失败

**优先级**： P1 / 高
**状态**： resolved
**依赖**： LW-017, LW-032

### 问题

生产 FDILink 节点将未初始化的 `serial_timeout_` 传给串口库，并在初始化前读取 `frist_sn_`。其接收循环记录读取字节数，但短读后仍继续解析，因此部分帧可能在 CRC 校验和发布前与陈旧或未初始化的存储内容拼接。串口打开失败还会以 `exit(0)` 终止，使启动系统和服务监督机制无法发现必需运行时的故障。

### 证据

- `src/fdilink_ahrs_ROS2/include/ahrs_driver.h:63-81`
- `src/fdilink_ahrs_ROS2/src/ahrs_driver.cpp:74-101`
- `src/fdilink_ahrs_ROS2/src/ahrs_driver.cpp:110-205`
- `src/fdilink_ahrs_ROS2/src/ahrs_driver.cpp:302-440`
- `src/serial_ros2/src/impl/unix.cc:533-606`

### 计划范围

- 将基于联合体的零散解析替换为固定容量、增量输入的协议解析器；只有类型/长度获准、帧头 CRC8 有效、载荷 CRC16 有效且具备所需结束标记的完整帧才输出。
- 使任意读取分片、前导噪声、畸形帧、超时复位及确定性重新同步均可在不使用 ROS 硬件的情况下测试。
- 初始化所有序号和传感器缓存状态，仅在完整帧校验后更新序号统计；对应缓存尚未接收其帧时，绝不基于该缓存发布 IMU 姿态或磁场样本。
- 使用确定性的有界串口读取超时，确保关闭流程不会被不确定的等待拖住。
- 串口打开失败、断开及接收异常时返回失败，正常 ROS 关闭时仍返回成功状态。
- 保留生产话题、重映射、坐标变换、真机运行时 IMU/AHRS 守卫、电机处理、策略及串口库。

### 验收标准

- 任何未初始化标量或传感器缓存都不会影响串口设置、序号统计、CRC 校验或已发布消息。
- 不完整的帧头或帧体，包括在帧的每个边界处分片的字节，在完整接收前绝不发布；超时会丢弃不完整帧，不将其与之后的字节拼接。
- 类型/长度、CRC8、CRC16 或结束标记无效的帧被拒绝，后续有效帧无需重启节点即可恢复接收。
- 第一个完整 AHRS 包到达前不发布 IMU 包，第一个完整 IMU 包到达前不发布磁场样本。
- 串口不可用时及时以非零状态退出；正常 ROS 关闭会关闭串口，并在有界读取超时内成功退出。
- `fdilink_ahrs` 包、完整 LW 构建/测试套件、严格维护构建、部署相关检查、语法检查和 `git diff --check` 均通过，且不访问真实串口设备、IMU 或电机。

### 解决记录

- **解决时间**： 2026-08-18T12:46:04+08:00
- **提交**： `cc54c5f4`
- **获准范围**： 将 FDILink 串口接收改为固定容量、增量式完整帧解析；
  只有类型、长度、CRC8、CRC16 和结束标记全部有效时才更新序号并发布。
  初始化串口、序号及传感器缓存状态，丢弃超时或短读留下的不完整帧；在
  AHRS/IMU 对应缓存首次有效前禁止发布依赖该缓存的消息。串口读取使用
  20 ms 有界超时，串口打开或接收异常以非零状态退出，正常 ROS 关闭在
  有界等待后成功退出；保持既有话题、坐标变换和下游真机安全守卫不变。
- **修改文件**： `src/fdilink_ahrs_ROS2/CMakeLists.txt`、
  `src/fdilink_ahrs_ROS2/package.xml`、
  `src/fdilink_ahrs_ROS2/include/{ahrs_driver.h,fdilink_frame_parser.h}`、
  `src/fdilink_ahrs_ROS2/src/{ahrs_driver.cpp,fdilink_frame_parser.cpp}`、
  `src/fdilink_ahrs_ROS2/test/{test_fdilink_frame_parser.cpp,test_fdilink_process_lifecycle.py}`、
  `.learnings/LW_REAL_DEPLOYMENT_ISSUES.md`。
- **验证**： 隔离构建 `serial`、`fdilink_ahrs`、`rl_sar` 三个包成功；
  FDILink 的 6 个 GTest 用例覆盖全部支持帧型、每个截断边界、噪声与连续帧、
  非法类型/长度/CRC8/CRC16/结束标记、超时复位和后续恢复，2 个 Python
  进程用例验证不存在的串口及时非零退出，以及空 PTY 上 SIGTERM 在 1 秒内
  成功退出。两项 CTest 均通过。变更目标在
  `-Wall -Wextra -Wpedantic -Werror` 下构建并再次通过两项 CTest；完整 LW
  CTest 45/45 通过，包含部署包、清单、运行时依赖、策略资产、构建工作流
  和启动禁用集成检查。`scripts/validate_lw_strict_build.sh` 构建全部维护目标
  并通过 45/45 CTest；`git diff --check` 通过。所有验证均未访问真机串口、
  AHRS、IMU 或电机；用户已有快速启动文档、错误记录和未跟踪技能目录保持不变。
- **后续事项**： 无

---

<a id="lw-045"></a>

## [LW-045] 显式且跨架构安全的 FDILink 载荷解码

**优先级**： P1 / 高
**状态**： resolved
**依赖**： LW-044

### 问题

LW-044 验证完整帧后，驱动将其字节复制到紧凑布局联合体的 `read_tmp` 成员，再读取非活动的 `frame` 成员。载荷从字节偏移七开始，因此其中的 `float`、`double` 和 `int64_t` 成员也未对齐。生产传感器边界因而依赖编译器的联合体类型双关扩展、紧凑布局成员行为、原生 IEEE-754 布局和主机字节序。

### 证据

- `src/fdilink_ahrs_ROS2/include/fdilink_data_struct.h:6-186`
- `src/fdilink_ahrs_ROS2/src/ahrs_driver.cpp:18-40`
- `src/fdilink_ahrs_ROS2/src/ahrs_driver.cpp:213-385`

### 计划范围

- 根据明确的小端字节偏移，将每种受支持的 FDILink 载荷解码为普通对齐值类型；使用整数组装及 `memcpy` 传递 IEEE-754 位模式，不使用类型强制转换或紧凑布局对象。
- 让驱动缓存已解码的 IMU、AHRS、INSGPS 和大地坐标位置值，并使用已验证的帧序号进行序号统计。
- 移除已不再使用的紧凑布局结构体、联合体和布局大小断言。
- 保留帧校验、话题、消息字段、单位、坐标变换、发布顺序、超时行为及下游守卫。
- 载荷取值范围和非有限值处理策略不纳入本问题。

### 验收标准

- 生产 FDILink 载荷均不通过紧凑布局对象、非活动联合体成员、`reinterpret_cast` 或主机原生多字节读取来解释。
- 固定小端字节向量能解码全部四种受支持载荷类型的每个字段，包括有符号值、`float`、`double` 和 `int64_t`。
- 错误类型、错误载荷长度和截断帧被拒绝，且不修改调用者的解码输出。
- 既有解析器、进程生命周期、ROS 话题映射、坐标变换及序号行为保持不变。
- 变更目标通过将警告视为错误的构建及未定义行为/对齐检查器构建；FDILink 测试、完整 LW 套件、部署相关检查、语法检查和 `git diff --check` 均在不使用真实硬件的情况下通过。

### 解决记录

- **解决时间**： 2026-08-18T13:44:26+08:00
- **提交**： `559ac3eb`
- **获准范围**： 将四类已验证 FDILink 帧按明确的小端字段偏移解码到
  普通对齐值对象；整数字节先组合为无符号数，IEEE-754 浮点通过 `memcpy`
  转移位模式，有符号 64 位值按协议补码转换。驱动只在完整解码成功后更新缓存，
  并直接使用已验证帧序号。删除 紧凑布局 结构、联合体 覆盖读取和布局尺寸断言；
  保持 CRC/帧解析、话题、单位、坐标变换、发布顺序和下游安全守卫不变，未加入
  数值范围或 NaN/Inf 策略。
- **修改文件**： `src/fdilink_ahrs_ROS2/CMakeLists.txt`、
  `src/fdilink_ahrs_ROS2/include/{ahrs_driver.h,fdilink_payload_decoder.h}`、
  `src/fdilink_ahrs_ROS2/include/fdilink_data_struct.h`（删除）、
  `src/fdilink_ahrs_ROS2/src/{ahrs_driver.cpp,fdilink_payload_decoder.cpp}`、
  `src/fdilink_ahrs_ROS2/test/test_fdilink_payload_decoder.cpp`、
  `.learnings/LW_REAL_DEPLOYMENT_ISSUES.md`。
- **验证**： 固定小端字节向量逐字段验证 IMU、AHRS、INSGPS 和
  大地坐标位置 的全部 `float`、`double` 与 `int64_t` 字段，并验证错误
  类型、错误长度和截断帧不修改输出。`serial`、`fdilink_ahrs`、`rl_sar`
  三包隔离构建成功；解析器、解码器和进程生命周期 3/3 CTest 通过，完整 LW
  CTest 45/45 通过。变更协议库、驱动和两项 C++ 测试在
  `-Wall -Wextra -Wpedantic -Werror -fsanitize=undefined,alignment`
  下构建，3/3 CTest 通过且无 检查器 报告；
  `scripts/validate_lw_strict_build.sh` 构建全部维护目标并通过 45/45 CTest。
  源码检查确认 FDILink 生产路径不再包含 紧凑布局/联合体 覆盖或
  `reinterpret_cast` 解码，`git diff --check` 通过。未访问真机串口、AHRS、
  IMU 或电机；用户未跟踪技能目录保持未修改。
- **后续事项**： 无

---

<a id="lw-046"></a>

## [LW-046] 发布前拒绝语义无效的 FDILink 样本

**优先级**： P2 / 中
**状态**： resolved
**依赖**： LW-045

### 问题

帧即使通过类型、长度、CRC、结束标记和跨架构安全解码检查，仍可能包含 NaN、无穷大、无效四元数范数或不可能的大地坐标。目前驱动会立即提交解码值，因此这类帧可能覆盖有效缓存并发布无效 ROS 消息。LW 真机控制路径会独立拒绝无效的 AHRS/四元数/陀螺仪输入，但其他 ROS 消费者和非控制用途的 FDILink 话题不受该守卫保护。

### 证据

- `src/fdilink_ahrs_ROS2/src/ahrs_driver.cpp:188-349`
- `src/fdilink_ahrs_ROS2/include/fdilink_payload_decoder.h:11-85`
- `src/rl_sar/library/core/safety/lw_imu_ahrs_guard.hpp:56-127`
- `src/rl_sar/src/rl_real_LW.cpp:630-665`

### 计划范围

- 在修改缓存或发布前验证解码候选，对 IMU、磁场、INSGPS、里程计、速度和大地坐标话题实际发布的每个字段检查有限性。
- 要求 AHRS 欧拉角/四元数字段为有限值，并采用下游既有的 0.9 到 1.1 四元数范数范围。
- 验证大地纬度位于 `[-pi/2, pi/2]`、经度位于 `[-pi, pi]`；不添加未经审查的运动或磁场幅值限制。
- 将磁场有效性与 IMU 运动有效性分开处理，并将 GPS/INSGPS 故障与全部 IMU/AHRS 缓存及发布决策隔离。
- 结构有效的帧即使载荷未通过语义校验，仍更新序号统计；语义拒绝与 CRC 失败分别计数，并输出限频诊断。
- 保留话题、坐标变换、串口解析、下游守卫、电机处理和配置。

### 验收标准

- 当前填充的任何 FDILink ROS 消息字段都不会输出非有限值，被拒绝的候选也不会替换对应的有效缓存。
- 错误 AHRS 候选仅使 AHRS 就绪状态失效；错误 IMU 运动候选仅使 IMU/磁场就绪状态失效；错误磁场字段不会抑制有效 IMU 消息。
- 无效 GPS 或 INSGPS 数据仅抑制自身输出消息，不能改变 IMU/AHRS 缓存、标志或输出。
- 四元数和大地坐标边界值被接受，超出获准范围的值被拒绝。
- 语义拒绝诊断有界，并与 CRC 错误区分。
- FDILink 测试、将警告视为错误的构建和检查器构建、完整 LW 套件、严格维护构建、部署相关检查、语法检查和 `git diff --check` 均在不使用真实硬件的情况下通过。

### 解决记录

- **解决时间**： 2026-08-18T16:13:32+08:00
- **提交**： `a9f981dd`
- **获准范围**： 对解码后的 IMU 运动、磁场、AHRS、INSGPS 和
  大地坐标位置 候选执行发布字段语义校验；AHRS 使用与下游一致的
  0.9–1.1 四元数范数，地理纬度限制为 `[-pi/2, pi/2]`、经度限制为
  `[-pi, pi]`，其余字段只检查有限值。候选先验证后提交；IMU 运动与磁场
  独立有效，GPS/INSGPS 拒绝不访问任何 IMU/AHRS 缓存或标志。结构有效但
  语义无效的帧仍更新传输序号，使用独立计数和 1 秒节流诊断，不混入 CRC
  错误。保持话题、坐标变换、串口解析、下游守卫、电机和配置不变。
- **修改文件**： `src/fdilink_ahrs_ROS2/CMakeLists.txt`、
  `src/fdilink_ahrs_ROS2/include/{ahrs_driver.h,fdilink_payload_validation.h}`、
  `src/fdilink_ahrs_ROS2/src/{ahrs_driver.cpp,fdilink_payload_validation.cpp}`、
  `src/fdilink_ahrs_ROS2/test/test_fdilink_payload_validation.cpp`、
  `.learnings/LW_REAL_DEPLOYMENT_ISSUES.md`。
- **验证**： 新增 9 个语义校验 GTest，用 NaN、正 Inf、负 Inf 逐一
  覆盖全部实际发布字段，验证 IMU 运动/磁场独立、未发布字段不误伤、四元数
  可表示边界内外、经纬度精确边界和越界。`serial`、`fdilink_ahrs`、
  `rl_sar` 三包隔离构建成功；FDILink 解析器、解码器、语义校验和进程生命
  周期 4/4 CTest 通过，完整 LW CTest 45/45 通过。变更协议库、驱动和三项
  C++ 测试在 `-Wall -Wextra -Wpedantic -Werror` 与
  `-fsanitize=undefined,alignment` 下构建，4/4 CTest 通过且无 检查器
  报告；`scripts/validate_lw_strict_build.sh` 构建全部维护目标并通过 45/45
  CTest。源码检查确认 CRC/语义计数分离、诊断节流，以及 GPS/INSGPS 分支
  不修改 IMU/AHRS 状态；`git diff --check` 通过。未访问真机串口、AHRS、
  IMU、GPS 或电机；用户未跟踪技能目录保持未修改。
- **后续事项**： 无

---

<a id="lw-047"></a>

## [LW-047] 分类处理 FDILink 8 位序号异常，避免虚构丢帧

**优先级**： P2 / 低
**状态**： resolved
**依赖**： LW-044

### 问题

当前序号计数器只要发现收到的序号与下一个预期值不同，就执行无符号 8 位减法。因此，一个重复帧会凭空增加 255 个丢失帧，设备复位或向后跳变也可能增加大量虚假丢帧。长时间部署后，有符号 `int` 总数还可能溢出。这些诊断无法区分已确认的前向缺帧、重复帧和不连续跳变。

### 证据

- `src/fdilink_ahrs_ROS2/src/ahrs_driver.cpp:458-481`
- `src/fdilink_ahrs_ROS2/include/ahrs_driver.h:61-65`

### 计划范围

- 在纯逻辑、独立测试的组件中跟踪相邻结构有效帧的 8 位序号模差。
- 将差值 1（包括 255 到 0 回绕）视为顺序正常；差值 2 到 127 视为前向缺帧，确认缺失帧数恰为 `delta - 1`；差值 0 视为重复；差值 128 到 255 视为不连续。
- 不连续时重建基线，使后续递增可以正常恢复。
- 分别维护确认丢帧、重复和不连续的饱和 64 位总计数，并且只在调试模式输出限频的分类诊断。
- 序号观测继续在语义校验前进行，禁止序号分类改变解码、缓存、发布或控制。

### 验收标准

- 首帧、普通递增和 255 到 0 回绕不报告丢帧。
- 单个或多个前向缺帧只统计无歧义的缺失帧。
- 重复帧、向后/复位跳变和有歧义的半区间跳变不增加确认丢帧，而是计入各自类别。
- 不连续后的帧根据新基线评估。
- 计数器在 `uint64_t` 最大值处饱和，不发生回绕。
- FDILink 测试、将警告视为错误的构建及未定义行为/对齐检查器构建、完整 LW 套件、严格维护构建和 `git diff --check` 均在不使用真实硬件的情况下通过。

### 解决记录

- **解决时间**： 2026-08-18T16:31:45+08:00
- **提交**： `1fda05ad`
- **获准范围**： 新增纯 C++ 的 8 位序号跟踪器，将首次、顺序、明确前向
  缺帧、重复和不连续分别分类；255 到 0 视为正常递增，模差 2–127 只累计
  `delta - 1` 个确认缺帧，模差 0 计为重复，模差 128–255 计为不连续并以
  当前帧重建基线。三个统计量使用共享的饱和 `uint64_t` 加法。驱动对所有结构
  有效帧继续在语义校验前更新序号，仅在 调试 模式输出 1 秒节流的分类诊断；
  序号结果不改变解码、缓存、发布或控制。
- **修改文件**： `src/fdilink_ahrs_ROS2/CMakeLists.txt`、
  `src/fdilink_ahrs_ROS2/include/{ahrs_driver.h,fdilink_sequence_tracker.h}`、
  `src/fdilink_ahrs_ROS2/src/{ahrs_driver.cpp,fdilink_sequence_tracker.cpp}`、
  `src/fdilink_ahrs_ROS2/test/test_fdilink_sequence_tracker.cpp`、
  `.learnings/LW_REAL_DEPLOYMENT_ISSUES.md`。
- **验证**： 新增 7 个 GTest，覆盖首次与正常递增、255 到 0 回绕、单个
  和多个确认缺帧、重复、后退与半区间跳变、不连续后恢复，以及饱和计数加法。
  `serial`、`fdilink_ahrs`、`rl_sar` 三包隔离构建成功；FDILink 解析器、
  解码器、语义校验、序号跟踪和进程生命周期 5/5 CTest 通过，完整 LW CTest
  45/45 通过。协议库、驱动和四项 C++ 测试在
  `-Wall -Wextra -Wpedantic -Werror` 与
  `-fsanitize=undefined,alignment` 下构建，4/4 CTest 通过且无 检查器
  报告；`scripts/validate_lw_strict_build.sh` 构建全部维护目标并通过 45/45
  CTest。源码检查确认五个结构有效帧分支仍在语义校验前调用序号跟踪，旧的
  有符号丢帧计数和原始基线字段已删除；`git diff --check` 通过。未访问真机
  串口、AHRS、IMU、GPS 或电机；用户未跟踪技能目录保持未修改。
- **后续事项**： 无

---

<a id="lw-048"></a>

## [LW-048] 将已安装的 ONNX Runtime 文件绑定到获准的归档包

**优先级**： P1 / 高
**状态**： resolved
**依赖**： LW-027, LW-034

### 问题

下载路径在解压前验证获准归档的 SHA-256，但安装后的来源文件仅记录目录元数据。后续检查将这份自行声明的来源信息与目录比较，并验证结构和 ELF 架构，却不能证明已安装共享库的字节仍与获准归档一致。部署清单生成时会对当前安装的任意库字节计算哈希，因此，即使来源文件未变，被篡改的库仍可能成为新的内部自洽部署基线。

### 证据

- `scripts/manage_inference_runtime.py:168-200`
- `scripts/manage_inference_runtime.py:260-297`
- `src/rl_sar/scripts/generate_lw_deployment_manifest.py:74-116`
- `src/rl_sar/scripts/generate_lw_deployment_manifest.py:251-284`

### 计划范围

- 将每个获准平台归档对应的精确 ONNX Runtime 安装文件集合及字节摘要，加入可信运行时目录或经过同等审查的不可变记录。
- 在安装、日常运行时检查、构建配置、部署清单生成和部署验证阶段，记录并核验获准的安装字节身份。
- 即使版本、架构和来源元数据看起来仍然正确，也拒绝缺失、额外、被替换或为符号链接的运行时文件。
- 保留获准的 ONNX Runtime 版本、架构选择、下载 URL、部署布局和既有归档摘要检查。

### 验收标准

- 从每个获准归档解压且未经修改的运行时，能够通过安装、重复检查、生产构建和部署验证。
- 保持来源文件不变而修改已安装库的一个字节，会使运行时检查和清单生成失败，从而无法接受该部署。
- 安装文件摘要集合缺失、格式错误、重复或不一致的目录条目被拒绝。
- 既有错误归档、错误架构、不安全路径和符号链接测试持续通过，并增加安装字节篡改的端到端测试。
- 完整严格 LW 测试套件及 `git diff --check` 在不下载生产运行时或访问真实硬件的情况下通过。

### 解决记录

- **解决时间**： 2026-08-18T17:02:39+08:00
- **提交**： `6ef1b97e`
- **获准范围**： 将可信运行时目录升级为 格式版本 2，为 x64 与 aarch64
  批准归档中的版本化 `libonnxruntime.so.1.22.0` 和
  `libonnxruntime_providers_shared.so` 固定归档内路径、部署路径及逐文件
  SHA-256。运行时管理器在安装候选及日常检查时验证精确文件集合、普通文件
  类型、批准字节和两级主库符号链接；部署清单生成器只接受目录中批准的库
  字节；部署端再把清单摘要与按架构编译进验证器的批准摘要比较，拒绝同时
  篡改库文件和清单的自洽伪造。继续接受既有 格式版本 1 来源 文件，保持
  版本、下载地址、架构选择、部署布局和归档摘要检查不变。
- **修改文件**： `scripts/inference_runtime_archives.json`、
  `scripts/manage_inference_runtime.py`、`src/rl_sar/CMakeLists.txt`、
  `src/rl_sar/library/core/deployment/lw_deployment_bundle.cpp`、
  `src/rl_sar/scripts/generate_lw_deployment_manifest.py`、
  `src/rl_sar/test/test_generate_lw_deployment_manifest.py`、
  `src/rl_sar/test/test_inference_runtime_download_integrity.py`、
  `src/rl_sar/test/test_lw_deployment_bundle.cpp`、
  `.learnings/LW_REAL_DEPLOYMENT_ISSUES.md`。
- **验证**： 下载两个现有目录锁定的官方归档并确认归档 SHA-256，提取
  四个生产库摘要；当前 x64 已安装运行时与批准归档逐字节一致，aarch64
  目录选择返回对应批准文件集。运行时管理器 10/10、部署清单生成器 20/20
  Python 测试通过，覆盖安装后单字节篡改、符号链接改向、缺失或畸形摘要、
  重复及越界路径；C++ 测试覆盖库与清单同步篡改仍被固定批准摘要拒绝。
  `scripts/validate_lw_strict_build.sh` 构建全部维护目标并通过 45/45 CTest；
  使用不挂接分支的临时验证提交运行正式部署构建脚本，三包构建、描述检查、
  策略一致性、依赖/RPATH/包前缀、清单以及原位置和重定位后的部署验证全部
  通过。正式生产验证器对同步修改 提供程序 库与清单摘要的副本按预期失败。
  Python 编译、JSON 语法和 `git diff --check` 通过；未访问真机串口、AHRS、
  IMU、手柄、电机或仿真 GUI，现有运行时未重装，用户未跟踪技能目录保持
  未修改。
- **后续事项**： LW-049, LW-050, LW-051, LW-052, LW-053

---

<a id="lw-049"></a>

## [LW-049] 使保留的 Gazebo 控制器执行有界、等待 URDF 就绪且内存分配稳定

**优先级**： P2 / 中
**状态**： resolved
**依赖**： LW-015

### 问题

两个保留的 ROS 2 控制器都调用了 `std::clamp`，却没有赋值使用其返回值，因此位置、速度和力矩限制目前不起作用。它们在异步请求 URDF 后立即报告配置成功，因此激活或更新时可能解引用缺失的关节或限制对象。关节组控制器还会在每次更新时构造三个大小等于关节数的向量，两个控制器也都直接除以传入的周期，未先要求该时长为有限正值。

### 证据

- `src/robot_joint_controller/ros2/src/robot_joint_controller.cpp:28-86`
- `src/robot_joint_controller/ros2/src/robot_joint_controller.cpp:151-229`
- `src/robot_joint_controller/ros2/src/robot_joint_controller_group.cpp:28-123`
- `src/robot_joint_controller/ros2/src/robot_joint_controller_group.cpp:214-299`

### 计划范围

- 在单关节和关节组控制器中，应用位置、速度、前馈力矩和最终计算力矩的限幅返回值。
- 只有有界 URDF 获取完成、解析了全部请求关节并验证所需限制数据后，配置才能成功；超时、关闭、响应畸形、缺失关节或缺失限制时返回生命周期错误。
- 索引或计算速度和力矩前，验证接口数量、命令大小、命令字段有限性及更新周期为有限正值。
- 在更新周期外预先设置并复用关节组控制器的临时/状态存储，使受维护的更新路径不发生大小随关节数量变化的分配。
- 保留控制器话题、消息类型、接口名称、停止哨兵值、ROS 2 生命周期集成和保留的 Gazebo 扩展边界。

### 验收标准

- 超出 URDF 位置、速度或力矩限制的命令，在命令接口处被限幅，包括最终的 PD 加前馈力矩。
- 全部关节限制就绪前，配置不能报告成功；机器人描述缺失或畸形时干净失败，不发生空指针访问。
- 拒绝零、负数或非有限周期，以及非有限或大小错误的命令，且不写入命令接口。
- 激活和预热后，关节组更新使用预分配临时存储；回归测试能够检测更新周期中的分配。
- 定向控制器测试、条件允许时的 Gazebo 冒烟测试、严格 LW 套件、将警告视为错误的构建和 `git diff --check` 均在不使用真实电机的情况下通过。

### 解决记录

- **解决时间**： 2026-08-18T17:44:25+08:00
- **提交**： `ef0af1d3`
- **获准范围**： 用户决定删除已退出产品范围的 Gazebo 仿真功能，而非
  继续修复其控制器。删除完整 `robot_joint_controller` 包、仅由该路径使用的
  `robot_msgs` 包、未构建的旧通用 `RL_Sim` 源码、Gazebo 启动文件 和 世界文件；
  移除 CMake 中残留的 Gazebo 编译及安装入口，并把仓库范围测试改为拒绝这些
  路径重新出现。保留 MuJoCo `rl_sim_LW`、LW MJCF、真机运行时、策略、FSM
  和安全核心不变。
- **修改文件**： 删除 `src/robot_joint_controller/`、`src/robot_msgs/`、
  `src/rl_sar/{include/rl_sim.hpp,src/rl_sim.cpp,launch/gazebo.launch.py,worlds/}`；
  修改 `src/rl_sar/CMakeLists.txt`、
  `src/rl_sar/library/core/rl_sdk/rl_sdk.cpp`、
  `src/rl_sar/test/{test_build_workflow.py,test_generate_lw_deployment_manifest.py,test_lw_deployment_bundle.cpp,test_lw_repository_scope.py}`
  和本记录。
- **验证**： 仓库范围测试通过并验证所有退役路径不存在；部署清单生成器
  20/20、构建流程 27/27 通过。`scripts/validate_lw_strict_build.sh` 以
  `-Wall -Wextra -Wpedantic -Werror` 构建全部维护目标并通过完整 45/45 CTest，
  其中 `rl_sim_LW` 与三个 MuJoCo 测试成功。Colcon 只发现 `serial`、
  `fdilink_ahrs`、`rl_sar`、`lw_description`，维护源码中无 Gazebo、控制器或
  专用消息引用（禁止重新引入的测试断言除外）。Python 编译和
  `git diff --check` 通过；批准清单中的旧 构建/安装 产物及 Gazebo 安装
  符号链接已删除。未启动 Gazebo、MuJoCo GUI、ROS 真机节点或任何硬件。
- **后续事项**： LW-050, LW-051, LW-052, LW-053

---

<a id="lw-050"></a>

## [LW-050] 统一 ONNX 动态批次约定与缓存张量资源

**优先级**： P2 / 中
**状态**： resolved
**依赖**： LW-013

### 问题

LW 配置校验允许固定为一的批次或动态批次维度。模型加载另行将缓存形状向量中的 `-1` 维度归一为一，但推理忽略这些向量，再次向会话查询原始形状，可能将 `-1` 传给 `CreateTensor`。该类还保存了可复用的 `Ort::MemoryInfo`，却在每次前向调用时重新构造一个；此外，在索引第零个元素前，也未验证外层输入向量的数量。这些并存的契约可能使已被接受的模型在推理时失败或行为不一致。

### 证据

- `src/rl_sar/library/core/inference_runtime/inference_runtime.hpp:90-104`
- `src/rl_sar/library/core/inference_runtime/inference_runtime.cpp:35-42`
- `src/rl_sar/library/core/inference_runtime/inference_runtime.cpp:92-123`
- `src/rl_sar/library/core/inference_runtime/inference_runtime.cpp:143-230`
- `src/rl_sar/library/core/rl_sdk/lw_configuration_validation.cpp:276-307`

### 计划范围

- 定义唯一且明确的单样本契约：恰好一个 float32 输入和一个 float32 输出，秩为二，批次固定为一，特征维度为固定正数。依据获准的产品决策，在模型加载时拒绝所有动态维度，包括动态批次。
- 加载时一次性缓存已验证的静态形状，并用于每个输入张量；在同一边界拒绝动态维度或任何其他不受支持的形状。
- 访问缓冲区或调用 ONNX Runtime 前验证输入数量及元素数量，并使用经过检查的正运行时维度计算输出大小。
- 使用一个缓存的 CPU `Ort::MemoryInfo`、缓存的节点名称和唯一权威形状表示；删除或实际使用当前冗余的形状状态。
- 保留当前策略维度、数值结果、单批次行为、会话线程设置和公开模型抽象，除非需要最小限度的契约澄清。

### 验收标准

- 固定 `[1, features]` 模型作为单样本执行；动态 `[-1, features]` 及其他非静态形状在加载时被拒绝。
- 空、多项、大小错误、类型错误、秩错误、动态特征和不受支持批次的输入/模型，在推理前确定性失败。
- 前向调用不再每次查询原始动态输入形状或构造新的 `Ort::MemoryInfo`。
- 输出提取拒绝无效或溢出的运行时形状，并返回与已验证输出元素数量完全一致的结果。
- 合成固定/动态模型测试、全部四个部署 LW 策略、完整严格套件、检查器检查和 `git diff --check` 均通过。

### 解决记录

- **解决时间**： 2026-08-18T18:06:30+08:00
- **提交**： `0ce245a3`
- **获准范围**： 将 ONNX 推理契约统一为恰好一个 float32、秩 2、静态
  `[1, features]` 输入和输出，按用户确认的一般策略部署形态拒绝动态 批次、
  动态特征维、其他 批次、类型、秩 或输入输出数量。模型加载时缓存唯一的
  张量元数据和节点名称，推理复用既有 CPU `Ort::MemoryInfo`，调用运行时前验证
  外层输入数量和元素数量，返回结果前验证实际输出类型、形状及经溢出检查的
  元素数量；失败的替换加载清空旧会话及元数据。
- **修改文件**： `src/rl_sar/library/core/inference_runtime/inference_runtime.hpp`、
  `src/rl_sar/library/core/inference_runtime/inference_runtime.cpp`、
  `src/rl_sar/library/core/rl_sdk/lw_configuration_validation.cpp`、
  `src/rl_sar/CMakeLists.txt`、`src/rl_sar/test/test_inference_runtime.cpp`、
  `src/rl_sar/test/test_lw_configuration_validation.cpp` 和本记录。
- **验证**： 通过运行时生成的离线 ONNX protobuf 模型验证静态 批次 1
  推理，并覆盖动态输入 批次、动态输入及输出特征维、批次 2、错误 秩、
  int64、多输入、多输出、输入数量及长度错误和失败重载清理。目标 CTest 2/2 通过；
  ASan+UBSan 测试通过且无报告；无 ONNX 宏路径以
  `-Wall -Wextra -Wpedantic -Werror` 单独编译通过；
  `scripts/validate_lw_strict_build.sh` 严格构建全部维护目标并通过完整 46/46
  CTest，其中配置契约覆盖全部四个实际部署策略。`git diff --check` 通过；
  未启动 Gazebo、MuJoCo GUI、ROS 真机节点或访问任何硬件，用户未跟踪技能
  目录保持未修改。
- **后续事项**： LW-051, LW-052, LW-053

---

<a id="lw-051"></a>

## [LW-051] 固定 MuJoCo 下载摘要并实现原子安装

**优先级**： P2 / 中
**状态**： resolved
**依赖**： 无

### 问题

MuJoCo 引导脚本按平台选择发布包 URL，却从不校验可信的归档摘要。它复用一个名称可预测的暂存目录，并将第一个匹配的解压目录直接移到最终位置。当现有安装看起来不完整或版本标记不同时，脚本会在下载、解压和候选安装校验成功前删除它。因此，损坏、被替换、并发或中断的下载可能移除可用的开发依赖，或安装未经批准的字节内容。

### 证据

- `scripts/download_mujoco.sh:32-57`
- `scripts/download_mujoco.sh:65-130`
- `scripts/download_mujoco.sh:145-234`
- `scripts/download_mujoco.sh:248-283`

### 计划范围

- 为每个受支持的 MuJoCo 操作系统/架构候选包维护经过审核的 SHA-256 摘要及精确归档名称和根目录名称。
- 下载到唯一的私有暂存目录，解压前校验归档，拒绝不安全或有歧义的归档布局，并在不触碰当前安装的情况下完整校验候选安装。
- 通过同一文件系统内的原子重命名安装，并提供回滚和清理语义；任何操作失败时均保留先前有效的安装。
- 使并发调用安全失败，或使用独立候选安装，避免共享部分完成的状态。
- 保留 MuJoCo 3.2.7、受支持平台、最终 `library/mujoco` 布局、macOS 后处理和正常的 CMake 发现机制。

### 验收标准

- 每个获准归档摘要对应的归档在匹配平台上成功安装；任何字节变化或平台不匹配均在解压/安装前被拒绝。
- 下载、解压、校验或最终替换失败时，先前有效的 MuJoCo 目录仍可用，且仅移除本次调用自己的候选安装。
- 并发候选安装不能使用、覆盖或删除彼此的暂存数据。
- 拒绝归档路径穿越、链接逃逸、有歧义的顶层根目录和不完整候选安装。
- 离线夹具测试覆盖成功、篡改、中断/回滚和并发；Bash 语法、相关构建测试及 `git diff --check` 通过。

### 解决记录

- **解决时间**： 2026-08-18T18:36:12+08:00
- **提交**： `af0b0c0c`
- **批准范围**： 为 MuJoCo 3.2.7 的 Linux x86_64、Linux aarch64、
  Windows x86_64 和 macOS universal2 官方资产固定精确名称、URL、归档格式、
  根布局、必需文件和 SHA-256。下载脚本改为清单驱动的薄包装器；独立管理器
  在解压前校验摘要，使用调用专属候选目录，拒绝路径穿越、绝对路径、链接
  逃逸、特殊或重复条目及模糊根布局，并在同文件系统内通过安装锁、唯一备份、
  原子重命名和失败回滚切换最终目录。保留版本、平台范围、`library/mujoco`
  布局、`VERSION_NUMBER`、macOS 安装名称/签名处理和 CMake 发现方式。
- **修改文件**： `.gitignore`、`scripts/mujoco_archives.json`、
  `scripts/manage_mujoco.py`、`scripts/download_mujoco.sh`、
  `src/rl_sar/test/test_mujoco_download_integrity.py`、
  `src/rl_sar/CMakeLists.txt` 和本记录。
- **验证**： 从 Google DeepMind 官方 GitHub 发布页 下载全部四个不同
  资产并二次计算 SHA-256，结果与清单一致；使用真实归档在 `/tmp` 中完成
  Linux x86_64、Linux aarch64 和 Windows x86_64 安装及来源检查，macOS DMG
  在 Linux 上仅验证官方字节和目录项，未尝试挂载。Python 3.10 与 3.13 的
  离线完整性测试均为 15/15，通过合法安装、Windows 反斜杠布局、篡改和平台
  错配、下载失败、路径穿越、绝对路径、链接逃逸、多根目录、不完整候选、
  最终重命名回滚、并发锁及危险目标符号链接覆盖。Bash、Python 和 JSON
  语法检查通过；`scripts/validate_lw_strict_build.sh` 严格构建全部维护目标并
  通过完整 47/47 CTest，`git diff --check` 通过。现有 `library/mujoco` 未安装、
  替换或删除，未启动 MuJoCo 图形界面、ROS 节点或访问硬件，用户未跟踪技能目录
  保持未修改。
- **后续事项**： LW-052, LW-053

---

<a id="lw-052"></a>

## [LW-052] 加固通用 rl_sim 的手柄边界检查与临时文件生命周期

**优先级**： P2 / 低
**状态**： resolved
**依赖**： LW-049

### 问题

通用 ROS/Gazebo 模拟器访问 Joy 按钮索引直至 10、轴索引直至 7，却不检查消息长度，因此过短或映射不同的 Joy 消息会导致越界访问。控制器启动时写入名称可预测的共享文件 `/tmp/robot_joint_controller_params.yaml`，通过 shell 命令调用生成器，并且仅在一条成功路径结束后删除文件。并发模拟器可能覆盖彼此的参数，而 fork/生成器失败可能遗留过期文件或替换无关路径。

### 证据

- `src/rl_sar/src/rl_sim.cpp:142-188`
- `src/rl_sar/src/rl_sim.cpp:272-317`
- `src/rl_sar/include/rl_sim.hpp:69-90`

### 计划范围

- 在每次按索引映射前检查 Joy 按钮和轴的最少数量，拒绝过短样本且不改变当前命令状态。
- 将 F710 映射移入小型可测试辅助函数，保留现有按钮组合、方向键符号和速度轴映射。
- 安全创建唯一私有参数文件，并在成功、生成器失败、fork 失败、异常和对象销毁时精确删除该文件。
- 使用参数向量执行控制器生成器，替代 shell 字符串拼接；校验子进程终止情况，并确保生成的 YAML 对配置的关节名称有效。
- 保留通用 `rl_sim` 话题、控制器名称、受支持 ROS 发行版的生成器选择，以及 LW 真机/Sim2Sim 手柄路径。

### 验收标准

- 空或长度不足的 Joy 消息不会越界访问数组或改变命令；完整 F710 消息保留全部现有映射。
- 两次并发启动使用不同参数文件，不能覆盖或移除彼此的数据。
- 父进程的每条失败路径均删除自己的临时文件，同时不触碰预先存在或无关的路径。
- 生成器 exec 失败、信号终止和非零退出均报告为启动失败，且不遗留临时文件。
- 定向辅助函数/生命周期测试、内存与未定义行为检测、受维护构建和 `git diff --check` 通过，且不启动 Gazebo 或真实硬件。

### 解决记录

- **解决时间**： 2026-08-18T18:43:47+08:00
- **提交**： `11183fc1`
- **批准范围**： 本项不恢复或修补已退役的通用 ROS/Gazebo `rl_sim`。
  LW-049 的提交 `ef0af1d` 已按用户决定删除 `src/rl_sar/src/rl_sim.cpp`、
  `src/rl_sar/include/rl_sim.hpp`、`robot_joint_controller`、`robot_msgs`、
  Gazebo 启动文件和世界文件，从而整体移除本项描述的 ROS Joy 越界、共享
  `/tmp/robot_joint_controller_params.yaml` 及 shell 生成器 生命周期风险。
  保留的 MuJoCo `rl_sim_LW` 使用独立的固定容量手柄安全路径，不在本项范围内。
- **修改文件**： 仅 `.learnings/LW_REAL_DEPLOYMENT_ISSUES.md`。
- **验证**： `test_lw_repository_scope.py` 通过，并持续断言通用
  `rl_sim`、Gazebo、控制器和专用消息路径不得重新出现；维护源码及脚本中无
  `robot_joint_controller_params.yaml` 或 controller-manager 生成器 调用。
  当前严格构建基线在 LW-051 后为完整 47/47 CTest 通过，其中仓库范围、
  MuJoCo 和手柄安全测试均通过；本项没有可运行的已退役代码，因此无需新增
  内存与未定义行为检测目标。`git diff --check` 通过；未启动 Gazebo、MuJoCo 图形界面、ROS
  节点或访问硬件，用户未跟踪技能目录保持未修改。
- **后续事项**： LW-053

---

<a id="lw-053"></a>

## [LW-053] 缓存 rl_sim_LW 调试消息布局

**优先级**： P2 / 低
**状态**： resolved
**依赖**： LW-041

### 问题

启用可选 Sim2Sim 绘图发布后，每次定时器回调都会重建四个静态名称向量，将其合并为第五个向量，创建新的 `JointState`，调整全部三个数值数组的大小，并复制关节映射和轮关节索引配置。布局和配置在回调之间不会变化，因此这些工作以配置的最高 200 Hz 调试频率造成可避免的重复分配和复制。

### 证据

- `src/rl_sar/src/rl_sim_LW.cpp:204-215`
- `src/rl_sar/src/rl_sim_LW.cpp:261-329`
- `src/rl_sar/src/rl_sim_LW.cpp:331-419`
- `src/rl_sar/include/rl_sim_LW.hpp:114-127`

### 计划范围

- 启用绘图发布时，一次性构建并校验固定调试字段名称布局、数值数组大小、关节映射、轮关节索引和偏移。
- 在定时器回调中复用预先定长的消息/布局，发布前仅更新时间戳和数值样本。
- 保持快照交接和 MuJoCo 访问同步不变，不将调试工作移入控制周期。
- 保留 `/LW_joint_states` 话题、精确字段顺序和值、默认需显式启用的设置、受支持的 1-200 Hz 频率，以及 MuJoCo 数据不可用时的行为。

### 验收标准

- 发布的名称、数组长度、偏移、轮关节处理、步态数据及跟踪数据，在字节内容和顺序上均与当前布局等价。
- 非法映射、轮关节索引、自由度数量或派生偏移在调试初始化时一次性失败，避免回调索引风险。
- 初始化和预热后，重复布局准备及字段填充不发生向量扩容或配置解码；分配回归测试覆盖受维护的回调辅助函数。
- 禁用绘图时，仍不构建调试缓冲区、发布器、定时器或缓存消息布局。
- Sim2Sim 调试测试、同步/生命周期测试、严格构建及 `git diff --check` 通过，且不打开 MuJoCo 图形界面。

### 解决记录

- **解决时间**： 2026-08-18T19:12:21+08:00
- **提交**： `d85f6808`
- **批准范围**： 仅缓存可选 MuJoCo `rl_sim_LW` 的调试消息布局、配置和
  MuJoCo 站点/传感器 查询结果；保持 `/LW_joint_states`、43 字段顺序与数值
  语义、1-200 Hz 开关、快照交接和 MuJoCo 互斥边界不变，不修改真机
  `LWDebugPublisher`。
- **修改文件**： `.learnings/LW_REAL_DEPLOYMENT_ISSUES.md`、
  `src/rl_sar/CMakeLists.txt`、`src/rl_sar/include/rl_sim_LW.hpp`、
  `src/rl_sar/src/rl_sim_LW.cpp`、
  `src/rl_sar/library/core/simulation/lw_sim_debug_message.hpp`、
  `src/rl_sar/library/core/simulation/lw_sim_debug_message.cpp`、
  `src/rl_sar/test/test_lw_sim_debug_message.cpp`、
  `src/rl_sar/test/test_lw_sim_lifecycle_integration.py`。
- **验证**： 新增单元测试逐项核对 43 个名称、数组长度、当前/目标
  关节值、轮关节速度目标、步态与跟踪字段、缺失可选传感器清零、非法配置
  拒绝及预热后 10000 次填充零分配；定向构建和测试通过。完整
  `LW_STRICT_WARNINGS=ON` 构建及 48/48 CTest 通过，新增测试在
  AddressSanitizer/UndefinedBehaviorSanitizer 下通过，`git diff --check`
  通过。未启动 MuJoCo 图形界面、ROS 节点或访问任何硬件，用户未跟踪技能目录
  保持未修改。
- **后续事项**： 无

---

<a id="lw-054"></a>

## [LW-054] 在控制线程启动前预加载形态转换动作资源

**优先级**： P1 / 高
**状态**： resolved
**依赖**： LW-009, LW-011, LW-012, LW-038

### 问题

两个形态转换状态都在 `Enter()` 回调中构造 `MotionLoaderLW`。这些回调在 200 Hz 控制线程中执行，但构造过程会解析路径、打开并解析完整 CSV、扩展行和向量存储、推导速度并输出诊断。因此，腿转轮或轮转腿请求可能因文件系统延迟及无界解析/分配而阻塞有截止时间要求的执行路径，同时电机板仍保持上一条命令。

### 证据

- `src/rl_sar/fsm_robot/fsm_LW.hpp:407-448`
- `src/rl_sar/fsm_robot/fsm_LW.hpp:517-558`
- `src/rl_sar/library/core/motion_loader/motion_loader_lw.cpp:91-255`
- `src/rl_sar/test/test_lw_fsm_transitions.cpp:303-386`

当前转换测试仅执行 `CheckChange()`，不会进入目标状态，因此无法检测启动后的文件系统访问或解析。

### 计划范围

- 在可失败的启动阶段、任何产生命令的线程启动前，加载并完整校验两个转换 CSV 资源。
- 将不可变的预处理动作数据绑定到策略/状态定义，使 `Enter()` 仅执行有界游标复位和偏航角/参考对齐。
- 保留当前帧率、偏移、插值、关节顺序、策略和安全语义；任一资源无效时，在命令送达前使启动失败。
- 增加执行真实转换状态进入路径的测试，证明启动后不再依赖文件。

### 验收标准

- 成功启动后，两个转换方向均不在 `Enter()` 或其他控制周期回调中打开或解析 CSV。
- 转换状态进入路径经测量为有界执行，不进行完整动作分配，同时两条动作轨迹保持数值等价。
- 缺失、格式错误或不兼容的转换资源在控制线程和电机命令送达开始前导致失败。
- 转换状态进入回归、受维护严格构建、内存与未定义行为检测以及 `git diff --check` 通过，且不访问真实硬件。

### 解决记录

- **解决时间**： 2026-08-20T15:05:58+08:00
- **提交**： `79bf0b76`
- **批准范围**： 将 CSV 解析、完整轨迹校验、速度表计算和动作数据分配
  从 `leg_to_wheel`/`wheel_to_leg` 的 200 Hz FSM `Enter()` 移到
  `PreloadLWPolicyContext()`；策略定义持有不可变预处理动作，运行时复用启动期
  创建的独立播放游标。`Enter()` 只选择已准备游标、复位时间/偏航角 对齐并执行
  原有策略激活；成功路径不再访问动作文件或输出重置日志。真实机预加载继续受
  启动失能守卫保护并早于运行态交接和工作线程，MuJoCo 入口同样早于其工作线程。
  保持 FPS、时间偏移、插值、关节顺序、ONNX、FSM 和 S1-S4 行为不变；未处理
  LW-055 的同步或 LW-056 的逐周期快照分配。
- **修改文件**： `.learnings/LW_REAL_DEPLOYMENT_ISSUES.md`、
  `src/rl_sar/fsm_robot/fsm_LW.hpp`、
  `src/rl_sar/library/core/motion_loader/motion_loader_lw.{hpp,cpp}`、
  `src/rl_sar/library/core/rl_sdk/rl_sdk.{hpp,cpp}`、
  `src/rl_sar/test/test_lw_motion_loader.cpp`、
  `src/rl_sar/test/test_lw_fsm_transitions.cpp`、
  `src/rl_sar/test/test_lw_real_startup_disable_integration.py`。
- **验证**： 当前 Debug 完整构建成功并重新链接 `rl_real_LW`、
  `rl_sim_LW` 和 `lw_config_profiler`，完整 48/48 CTest 通过。新增回归在临时
  策略根目录 中预加载两个真实转换策略后删除 CSV，再执行两侧实际 `Enter()`，
  均复用原播放游标并发布正确尺寸的首帧引用；缺失 CSV 在策略上下文预加载时
  失败且不发布定义或游标。动作单测验证不可变预处理数据在源文件删除后仍可由
  两个独立游标按原时间/速度语义播放。全新 `LW_STRICT_WARNINGS=ON` 完整构建
  通过；两项定向测试在 AddressSanitizer/UndefinedBehaviorSanitizer 下通过。
  定向 `cppcheck` 未发现新增问题，Python 语法和 `git diff --check` 通过。
  未启动 ROS 节点、MuJoCo 图形界面、串口、IMU、摇杆、真机或电机，用户未跟踪技能
  目录保持未修改。
- **后续事项**： LW-055, LW-056, LW-057

---

<a id="lw-055"></a>

## [LW-055] 移除实机控制截止时间路径上的阻塞同步

**优先级**： P1 / 高
**状态**： resolved
**结项记录提交**： `fa3fe9f3`
**依赖**： LW-007, LW-008, LW-011, LW-032, LW-033, LW-038

### 问题

真机 200 Hz 控制回调仍通过阻塞互斥锁获取输入，通过 `realtime_tools::RealtimeBox` 读取 IMU/AHRS 数据（其受支持的 ROS Humble 实现也获取阻塞互斥锁），并使用原子 `shared_ptr` 快照传输策略激活/输出。在部署使用的标准库中，这些共享指针操作并非无锁。如果低优先级的手柄、ROS 或推理写入线程持有某个内部锁时被抢占，`SCHED_FIFO` 控制线程可能遭遇无界优先级反转，错过其 5 ms 截止时间。

### 证据

- `src/rl_sar/library/core/safety/lw_runtime_sync.hpp:58-136`
- `src/rl_sar/include/rl_real_LW.hpp:114-126`
- `src/rl_sar/src/rl_real_LW.cpp:547-565`
- `src/rl_sar/src/rl_real_LW.cpp:630-710`
- `src/rl_sar/src/rl_real_LW.cpp:822-833`
- `src/rl_sar/library/core/safety/lw_runtime_core.hpp:273-338`
- `src/rl_sar/library/core/safety/lw_runtime_core.hpp:617-636`
- `src/rl_sar/library/core/rl_sdk/rl_sdk.cpp:1338-1426`
- `src/rl_sar/test/test_lw_runtime_sync.cpp:128-162`
- `src/rl_sar/test/test_lw_runtime_sync.cpp:245-310`
- `/opt/ros/humble/include/realtime_tools/realtime_tools/realtime_box.hpp:45-72`

现有测试证明了快照一致性及策略输入发布器的尝试加锁回退，但未暂停每个剩余写入方并证明控制侧读取在固定时间界限内返回。

### 计划范围

- 将控制侧阻塞读取替换为适用于单个实时读取方的有界非阻塞最新值快照，采用显式序列号或修订号校验，并在适用处使用固定存储。
- 若写入方正在写入，保留上一份一致的手柄、IMU/AHRS、激活或策略输出样本，由现有新鲜度/代次检查判断运行是否仍然安全。
- 保留命令生成的单一所有者、激活代次、新鲜度规则、安全锁存和故障动作语义。
- 为真机控制回调使用的每条交接通道增加对抗性争用测试，不仅覆盖策略输入发布器。

### 验收标准

- 控制侧的输入、IMU/AHRS、激活或策略输出读取均不能等待互斥锁或其他由写入方持有的阻塞原语。
- 测试在发布中途暂停每个写入方，证明控制读取迅速返回新的完整一致样本或上一份完整一致样本。
- 仍不能出现撕裂样本、跨代次输出或接受过期数据；现有 S1-S4 行为不变。
- 截止时间压力测试、受支持环境下的 ThreadSanitizer、受维护严格构建及 `git diff --check` 通过，且不启动真机节点或硬件。

### 解决证据

- 已解决：2026-08-20
- 新增固定三槽、单生产者/单消费者最新值传输。带标记的无锁 32 位中间索引转移槽位的独占所有权，使读取方在生产者填充另一槽位时保留上一份一致槽位，且从不等待写入方持有的互斥锁或引用计数操作。
- 真机和仿真手柄交接现在使用 SPSC 传输及控制侧拥有的缓存快照。真机 IMU 回调发布归一化的固定大小四元数/陀螺仪样本，不再通过 `realtime_tools::RealtimeBox` 发布 ROS 消息；控制回调以非阻塞方式读取上一份一致样本。
- 策略激活和动作参考发布使用控制到推理的 SPSC 通道，策略进度和完整策略输出帧则使用推理到控制的 SPSC 通道。策略定义仍由不可变的预加载定义映射拥有，所有可变大小帧槽位都在线程启动前设定大小。
- 停用和策略切换不再需要清空共享指针快照。读取方通过现有激活代次、来源序列号、时间戳、完整性和最大年龄检查拒绝保留样本。转换状态保留控制侧拥有且不回退的帧计数器，使第零帧在首次匹配的推理进度发布前仍可使用。
- 同步回归在生产者向其私有后槽复制期间将其暂停，验证 SPSC 读取方在 500 ms 内返回上一份一致样本，随后观测已完成的发布。其他 SPSC、手柄邮箱、策略输出、策略代次、输入来源和过期数据压力测试验证一致的最新值行为。
- 受维护 Debug 构建及全新 `LW_STRICT_WARNINGS=ON` 构建均完成，包括 `rl_real_LW`、`rl_sim_LW` 和 `lw_config_profiler`；两套完整 48/48 CTest 均通过。同步、策略输出和运行时一致性测试连续运行 20 次通过，`loop_timing` 连续运行 20 次通过。
- 独立 AddressSanitizer/UndefinedBehaviorSanitizer 同步测试及定向插桩的策略输出/运行时一致性测试通过，未发现检测器问题。ThreadSanitizer 插桩编译成功，但运行时无法在该容器中启动（`ThreadSanitizer: unexpected memory
  mapping`），包括禁用 PIE 后重试，因此不宣称取得环境不支持的 TSAN 结果。定向 `cppcheck` 仅报告既有 `CSVInit` 按值传参性能建议；`git diff --check` 通过。
- 未启动 ROS 节点、MuJoCo 图形界面、串口设备、IMU、手柄、真机或电机。用户拥有的未跟踪上下文压缩检查技能目录保持原样。
- **后续事项**： LW-056, LW-057

---

<a id="lw-056"></a>

## [LW-056] 使受维护的完整控制周期保持内存分配稳定

**优先级**： P2 / 中
**状态**： resolved
**结项记录提交**： `151a3e97`
**依赖**： LW-038, LW-054, LW-055

### 问题

现有分配回归覆盖独立的快照/插值辅助函数，未覆盖受维护的 `runControlCycle()` 编排。正常周期仍构建校验描述符向量、复制 FSM 状态名称字符串，并在转换动作期间反复创建向量/共享指针参考对象。MuJoCo 适配器还在高频状态、命令和手柄路径中反复解码不变的 YAML 向量。这些开销可能增长或退化，却没有任何测试观测实际控制流程中的分配。

### 证据

- `src/rl_sar/library/core/safety/lw_control_safety.hpp:87-119`
- `src/rl_sar/library/core/safety/lw_control_safety.hpp:151-195`
- `src/rl_sar/library/core/fsm/fsm.hpp:20-88`
- `src/rl_sar/fsm_robot/fsm_LW.hpp:462-484`
- `src/rl_sar/fsm_robot/fsm_LW.hpp:572-594`
- `src/rl_sar/library/core/rl_sdk/rl_sdk.cpp:154-172`
- `src/rl_sar/library/core/rl_sdk/rl_sdk.cpp:753-766`
- `src/rl_sar/library/core/motion_loader/motion_loader_lw.cpp:289-350`
- `src/rl_sar/src/rl_sim_LW.cpp:740-855`
- `src/rl_sar/src/rl_sim_LW.cpp:1017-1066`
- `src/rl_sar/test/test_lw_allocation_bound.cpp:123-175`

### 计划范围

- 将逐周期的校验描述符容器和 FSM 状态名称复制替换为固定/静态描述符及不分配内存的转换标识。
- 复用固定转换参考存储或保留的槽位，避免每个转换动作周期都分配向量/共享所有权对象。
- 让保留的 MuJoCo 适配器使用已校验的类型化 LW 运行时配置，避免反复解码 YAML 向量。
- 将分配检测扩展到预热后的完整正常控制周期及两个转换 `Run()` 路径；明确允许异常故障路径为丰富诊断进行分配。

### 验收标准

- 预热后成功的类真机控制周期和转换动作周期不发生项目代码拥有的动态内存分配。
- MuJoCo 状态/命令/手柄热路径不反复解码 YAML 或复制配置向量。
- FSM 转换、校验诊断、动作值和运行时安全决策保持行为等价。
- 完整周期分配测试、转换测试、严格及内存与未定义行为检测构建，以及 `git diff --check` 通过，且不访问硬件或打开图形界面。

### 解决证据

- 已解决：2026-08-20
- 成功的命令和反馈校验现在遍历栈上的固定描述符，并记录紧凑字段标识。仅在异常拒绝路径中生成人可读的诊断字符串。FSM 转换请求使用非拥有字符串视图，与保留的状态名称比较，不构造逐周期字符串。
- 转换动作加载器原地填充预先定长的关节、速度、根四元数和锚点四元数缓冲区。直接填充并发布控制侧拥有的动作参考生产者槽位，避免临时向量和共享所有权对象构造，同时保留插值、四元数归一化、代次和来源序列号行为。
- 保留的 MuJoCo 适配器在状态、命令和手柄热路径中，现使用启动时已校验的类型化基础配置获取自由度数量、关节映射、轮关节掩码、扭矩限制、步态命令和速度缩放。可选绘图回调也填充保留的预先定长快照，不再重建由向量存储的快照。LW-057 的传感器/执行器布局校验按范围约定保持不变。
- 分配回归现在执行 10,000 个预热后的完整控制周期，包括生产运行时核心编排及钩子聚合对象，报告项目代码拥有的分配为零。它还预加载真实 `leg_to_wheel` 和 `wheel_to_leg` 策略，反复执行两个实际转换状态的 `Run()` 路径，均为零分配。动作加载器测试验证每个预先定长填充接口与返回值的兼容接口保持数值等价，并拒绝大小不正确的缓冲区。
- 受维护 Debug 构建及全新 `LW_STRICT_WARNINGS=ON` 构建均完成，包括 `rl_real_LW` 和 `rl_sim_LW`；两套完整 48/48 CTest 均通过。随后，分配界限、FSM 转换、运行时一致性和动作加载器测试各连续运行 20 次通过（共 80 次）。MuJoCo 生命周期源码检查 11/11 通过，Python 编译通过。
- 四项定向测试的全新 AddressSanitizer/UndefinedBehaviorSanitizer 构建完成。但运行结果不能用作检测结论：重复运行可能非确定性地进入递归的 `AddressSanitizer:DEADLYSIGNAL` 循环，包括不加载 ONNX 的动作加载器测试。禁用 ASan 信号拦截后暴露间歇性的原始 SIGSEGV，而非有效报告。受影响可执行文件通过项目全局 Python 链接和 RPATH，将系统检测运行库与 Conda Python/libstdc++ 混用，因此不宣称检测运行通过或发现项目代码问题；相同测试已在完整 Debug 和严格测试集中通过。定向 `cppcheck` 仅报告有意采用的轻量 `std::string_view` 按值传递建议，以及既有 `CSVInit`/测试夹具性能建议。`git diff --check` 通过。
- 未启动 ROS 节点、MuJoCo 图形界面、串口设备、IMU、手柄、真机或电机。用户拥有的未跟踪上下文压缩检查技能目录保持原样。
- **后续事项**： LW-057

---

<a id="lw-057"></a>

## [LW-057] 校验 MuJoCo 控制适配器布局并测试实际安全动作

**优先级**： P2 / 中
**状态**： resolved
**结项记录提交**： `39748434`
**依赖**： LW-021, LW-036, LW-053, LW-056

### 问题

保留的 MuJoCo 适配器假定关节位置、速度、扭矩、四元数和陀螺仪数据位于固定连续的 `sensordata` 数据块，并将策略关节映射复用为执行器 `ctrl` 索引。当前场景碰巧符合这些假设，但重排或插入传感器/执行器可能悄然向策略输入错误状态，或向错误执行器发送命令。当前描述测试仅检查总体数量和选定的足部力传感器名称，而运行时一致性测试使用合成轨迹，没有使用实际 MuJoCo 适配器和安全动作写入器。

### 证据

- `src/rl_sar/src/rl_sim_LW.cpp:509-568`
- `src/rl_sar/src/rl_sim_LW.cpp:740-855`
- `src/rl_sar_zoo/LW_description/mjcf/LW.xml:134-190`
- `src/rl_sar/test/test_lw_description.cpp:9-65`
- `src/rl_sar/test/test_lw_runtime_parity.cpp:123-144`
- `src/rl_sar/test/test_lw_runtime_parity.cpp:654-709`
- `src/rl_sar/test/test_lw_sim_lifecycle_integration.py`

### 计划范围

- 启动时按名称解析所需关节、IMU 和执行器对象；校验其 MuJoCo 对象类型、维度、数据地址及完整的一一对应策略关节映射，然后缓存显式读写索引。
- 场景不兼容时使启动失败，不依赖传感器或执行器总数，并保留当前场景的数值顺序。
- 提取可无界面测试的 MuJoCo 适配器边界，输入实际状态并应用实际运行时安全决策，无需启动 ROS 或图形界面。
- 测试模型所有执行器上的 S2 阻尼输出及 S3/S4 清零，包括 S4 退出传播。

### 验收标准

- 任一必需传感器或执行器发生重排、缺失、重复或维度变化时，要么由按名称缓存的映射处理，要么在物理/控制线程启动前被拒绝。
- 状态读取和命令写入仅使用已校验的缓存地址，不假设策略关节索引等于 MuJoCo 执行器索引。
- 无界面测试执行实际适配器，证明 S2 写入预期阻尼扭矩、S3/S4 将全部 `nu` 控制量清零，且 S4 请求退出。
- 描述、适配器、生命周期、严格及内存与未定义行为检测测试，以及 `git diff --check` 通过，且不打开图形界面、ROS 节点或真实硬件。

### 解决证据

- 已解决：2026-08-20
- 已校验的基础运行时配置现在保留定义硬件顺序的唯一关节名称。策略索引 `i` 经 `joint_mapping[i]` 解析到具名 MuJoCo 关节，不再复用为原始传感器或执行器索引。
- 无界面的 `LWMuJoCoControlAdapter` 按名称解析并缓存所有必需关节、位置/速度/扭矩传感器、执行器、IMU 四元数和陀螺仪。启动时拒绝缺失、重复、类型错误、维度错误、越界或绑定错误的对象，以及存在歧义的执行器。
- MuJoCo 生命周期现在在加载模型/数据后、发布它们或启动其线程前执行布局校验。Sim2Sim 构造函数先校验 YAML，通过该启动前回调构建适配器，仅在模型契约成功后才启动手柄/控制线程。
- `GetState` 和 `SetCommand` 仅使用缓存地址及执行器 ID。现有事务式扭矩校验仍位于所有 `ctrl` 写入之前，状态、命令和安全适配器路径均已验证无分配。
- 实际安全写入器现在通过无界面适配器执行：S2 经正常命令路径生成预期被动阻尼扭矩，S3 将全部 `model->nu` 控制量清零且不退出，S4 将全部控制量清零并传播 `simulation_running=false`、`run=0` 和 `exitrequest=1`。
- 无界面测试加载两个受维护场景和合成的重排布局。测试证明能够处理无关传感器/执行器插入及非恒等策略映射，同时在线程启动前拒绝缺失、重复、类型错误、维度错误和绑定错误的布局。
- 受维护 Debug 构建及全新 `LW_STRICT_WARNINGS=ON` 构建均完成，包括 `rl_real_LW` 和 `rl_sim_LW`；两套完整 50/50 CTest 均通过。`git diff --check` 也通过。
- 全新仅 ASan 和仅 UBSan 构建分别对 `lw_runtime_parity`、`lw_allocation_bound`、`lw_mujoco_control_adapter` 和 `lw_mujoco_lifecycle` 的每项测试连续重复运行 20 次。全新 ASan/UBSan 联合构建对相同测试各重复运行 10 次。全部 200 次均在启用检测器信号拦截时通过，未发现检测器问题或递归 `DEADLYSIGNAL`。
- 隔离的干净验证克隆配置正式 `LW_PRODUCTION_DEPLOYMENT=ON` Release 模式，并构建 `rl_real_LW` 和 `lw_config_profiler`，确认保留的关节名称配置不会使生产构建退化。
- 未启动 ROS 节点、MuJoCo 图形界面、串口设备、IMU、手柄、真机或电机。用户拥有的未跟踪上下文压缩检查技能目录保持原样。
- **修改文件**： `.learnings/LW_REAL_DEPLOYMENT_ISSUES.md`,
  `src/rl_sar/CMakeLists.txt`, `src/rl_sar/include/rl_sim_LW.hpp`,
  `src/rl_sar/library/core/rl_sdk/lw_configuration_validation.cpp`,
  `src/rl_sar/library/core/rl_sdk/lw_configuration_validation.hpp`,
  `src/rl_sar/library/core/simulation/lw_mujoco_control_adapter.cpp`,
  `src/rl_sar/library/core/simulation/lw_mujoco_control_adapter.hpp`,
  `src/rl_sar/library/thirdparty/mujoco_simulate/mujoco_utils.hpp`,
  `src/rl_sar/src/rl_sim_LW.cpp`,
  `src/rl_sar/test/test_lw_configuration_validation.cpp`,
  `src/rl_sar/test/test_lw_description.cpp`,
  `src/rl_sar/test/test_lw_mujoco_control_adapter.cpp`,
  `src/rl_sar/test/test_lw_mujoco_lifecycle.cpp`，以及
  `src/rl_sar/test/test_lw_sim_lifecycle_integration.py`。
- **后续事项**： 无

---

<a id="lw-058"></a>

## [LW-058] 将受维护的 C++ 目标与未使用的 Python 运行环境隔离

**优先级**： P2 / 中
**状态**： resolved
**结项记录提交**： `6b610110`
**依赖**： LW-027, LW-050, LW-056

### 问题

软件包全局要求 Python 开发和 NumPy 组件，尽管受维护的 C++ 源码均未包含 `matplotlibcpp` 或调用 Python C API。两个 `rl_sdk` 变体都公开链接 Python 嵌入/模块/NumPy 目标，并将所选 Python 库目录追加到全局安装 RPATH。在当前 Conda 环境中，这会使每个依赖它们的控制和测试可执行文件同时加载 Conda Python、libstdc++、系统 GCC 11 ASan/UBSan 运行库及获准的预构建 ONNX Runtime。内存与未定义行为检测进程随后间歇性陷入递归的 `AddressSanitizer:DEADLYSIGNAL`，包括不加载 ONNX 的测试，因此表面上的 CPU 停滞无法提供可用的项目代码检测结论。

### 证据

- `src/rl_sar/CMakeLists.txt:323-325`
- `src/rl_sar/CMakeLists.txt:446-450`
- `src/rl_sar/CMakeLists.txt:529-543`
- `src/rl_sar/CMakeLists.txt:592-609`
- `src/rl_sar/test/test_build_workflow.py`
- `src/rl_sar/test/test_lw_sim_lifecycle_integration.py:170-173`
- LW-056 内存与未定义行为检测的解决证据

### 计划范围

- 仅将 Python 保留为受维护 Python 脚本和测试的解释器；从 C++ 控制目标中移除未使用的 Python 开发、模块、NumPy 和 matplotlib 头文件/链接传播。
- 不再将所选 Python 库目录加入软件包 RPATH，同时保留获准的 ONNX Runtime RPATH 和运行库来源检查。
- 增加源码配置及构建产物回归，证明代表性受维护 C++ 目标不会引入 Python 运行库依赖。
- 在全新构建目录中分别重新运行 ASan 和 UBSan。仅当运行库链接清理后，零分配计数测试的全局分配器重载仍有独立兼容问题时，才将该测试单独处理。

### 验收标准

- 受维护的 C++ 控制程序及代表性测试二进制不依赖 `libpython`，且项目配置的 RPATH 中无 Python/Conda 目录。
- Python 构建/部署/集成测试仍通过发现的解释器运行，ONNX Runtime 加载和部署完整性保持完好。
- 全新 ASan 和 UBSan 运行完成并提供可用结果，不再陷入递归信号报告循环；任何剩余外部运行库限制均单独隔离并报告，不禁用检测器的信号拦截。
- 完整 Debug 和严格测试集、定向检测器重复运行、依赖回归及 `git diff --check` 通过，且不运行 ROS 节点、模拟器图形界面或真实硬件。

### 解决证据

- 已解决：2026-08-20
- 软件包现在仅要求受维护脚本和测试使用的 Python 解释器。`rl_sdk` 和 `rl_sdk_lw_deployment` 不再传播 Python 嵌入、扩展模块或 NumPy 目标；已移除未使用的全局 `matplotlibcpp` 头文件目录及 Python 库 RPATH 注入。
- Linux 配置在 ROS 依赖之前解析系统架构的 `fmt` 软件包。这可防止已激活的 Conda shell 替换为自身的 `fmt` 软件包，并通过 CMake 自动构建 RPATH 重新引入 Conda 库目录，同时仍与受维护部署使用的 Ubuntu/ROS Humble 系统库一致。
- 构建工作流回归拒绝恢复 Python 开发、模块、NumPy、matplotlib 或 Python RPATH 配置，同时保留获准的 ONNX Runtime RPATH。新增 ELF 回归检查 FSM 测试及全部可用真机/仿真可执行文件是否依赖 `libpython`，并在 Conda 内配置时检查运行时搜索路径是否位于 `CONDA_PREFIX` 之下。
- 受维护 Debug 构建及全新 `LW_STRICT_WARNINGS=ON` 构建均完成，包括 `rl_real_LW` 和 `rl_sim_LW`；两套完整 49/49 CTest 均通过。最终格式调整后，源码配置和构建产物链接测试也通过。
- 全新仅 ASan、仅 UBSan 和 ASan/UBSan 联合构建分别对 `lw_fsm_transitions`、`lw_runtime_parity`、`lw_allocation_bound` 和 `lw_motion_loader` 的每项测试连续重复运行 20 次。全部 240 次均在启用检测器信号拦截时通过，未发现检测器问题或递归 `DEADLYSIGNAL`；分配计数器无需特殊变体。
- 隔离的干净验证克隆配置正式 `LW_PRODUCTION_DEPLOYMENT=ON` Release 模式，重新校验获准 ONNX 运行库，并构建 `rl_real_LW` 和 `lw_config_profiler`。ELF 检查未发现 `libpython` 或 Conda 路径；构建 RPATH 保留获准 ONNX 路径及必需系统/ROS 目录。Python AST 解析及 `git diff --check` 通过。
- 未启动 ROS 节点、MuJoCo 图形界面、串口设备、IMU、手柄、真机或电机。用户拥有的未跟踪上下文压缩检查技能目录保持原样。
- **修改文件**： `.learnings/LW_REAL_DEPLOYMENT_ISSUES.md`,
  `src/rl_sar/CMakeLists.txt`, `src/rl_sar/test/test_build_workflow.py`,
  `src/rl_sar/test/test_lw_runtime_linkage.py`。
- **后续事项**： LW-057

---

<a id="lw-059"></a>

## [LW-059] 在所有部分构造失败路径上停止 Sim2Sim 工作线程

**优先级**： P1 / 高
**状态**： resolved
**依赖**： LW-024, LW-057

### 问题

Sim2Sim 构造函数先启动手柄、推理和控制线程，再初始化仍可能失败的操作员状态定时器、可选绘图发布器与定时器、可选 CSV 日志器。现有 catch 块仅覆盖三个 `start()` 调用。若后续初始化抛出异常，不会调用完整对象的析构函数。由于线程句柄的声明早于回调访问的大多数状态成员，部分构造失败的栈展开会先销毁后声明的成员，之后线程句柄才最终停止线程。因此回调可能与运行时核心、MuJoCo 适配器、手柄邮箱、绘图缓冲区或其他成员的销毁发生竞争。

### 证据

- `src/rl_sar/src/rl_sim_LW.cpp:205-255`
- `src/rl_sar/src/rl_sim_LW.cpp:259-268`
- `src/rl_sar/include/rl_sim_LW.hpp:72-155`
- `src/rl_sar/test/test_lw_sim_lifecycle_integration.py`

### 计划范围

- 确保线程可能运行前，其依赖的所有可失败初始化都已完成；或安装构造保护，在成员栈展开开始前同步停止全部已启动线程。
- 保留正常退出顺序、线程时序、绘图按需启用行为、物理线程生命周期和现有安全事件语义。
- 在线程启动后每个尚存的可失败初始化边界注入确定性故障。

### 验收标准

- 每个注入的构造失败都在线程回调可访问的成员被销毁前，停止并等待全部已启动线程结束。
- 部分构造清理开始后无回调继续运行，无线程被分离或无限等待。
- 正常启动与退出行为不变。
- 生命周期、严格构建、受支持的内存与未定义行为检测测试以及 `git diff --check` 通过，且不打开图形界面、ROS 节点或硬件设备。

### 解决记录

- **解决时间**： 2026-08-20T18:48:16+08:00
- **提交**： `bf21601e`
- **批准范围**： 保持真机现有生命周期模式，不引入 Sim2Sim 专用的
  二阶段启动或线程组抽象。将 Sim2Sim 操作员状态定时器、可选 绘图发布器/定时器 和可选 CSV 日志器 的全部可抛初始化移到首个业务 工作线程
  启动之前，使 手柄、推理、控制 的启动/回滚块成为构造函数最后
  阶段。启动顺序和反向停止顺序与真机保持一致，未修改真机运行代码、控制核心、
  策略、FSM、MuJoCo 数值或安全动作。
- **修改文件**： `.learnings/LW_REAL_DEPLOYMENT_ISSUES.md`、
  `src/rl_sar/src/rl_sim_LW.cpp`、
  `src/rl_sar/test/test_lw_sim_lifecycle_integration.py`。
- **验证**： Python 生命周期集成测试扩展为 13 项并全部通过；新增断言
  同时检查真机和 Sim2Sim 的资源准备必须早于 工作线程 启动、共同启动顺序必须为
  手柄/推理/控制、异常回滚和析构停止顺序必须为
  控制/推理/手柄，并确保后端关闭发生在 等待线程结束之后。当前 Debug 构建
  重新链接 `rl_sim_LW`，完整 50/50 CTest 通过；全新
  `LW_STRICT_WARNINGS=ON` 构建及完整 50/50 CTest 通过。隔离
  AddressSanitizer/UndefinedBehaviorSanitizer 构建成功编译 `rl_sim_LW`，
  `lw_sim_lifecycle_integration` 和真实 `LoopFunc` 的 `loop_lifecycle` 测试
  2/2 通过。Python 语法和 `git diff --check` 通过。未启动 ROS 节点、
  MuJoCo 图形界面、串口、IMU、摇杆、真机或电机；用户未跟踪技能目录保持未修改。
- **后续事项**： LW-060, LW-061, LW-062, LW-063, LW-064,
  LW-065, LW-066

---

<a id="lw-060"></a>

## [LW-060] 在 ObservationBuffer 中统一使用历史帧计数域

**优先级**： P2 / 中
**状态**： resolved
**依赖**： LW-013, LW-050

### 问题

`ObservationBuffer::get_obs_vec()` 的文档及后续使用将 `obs_ids` 定义为 `[0, history_length)` 范围的历史帧索引，但输出大小预计算却按 `obs_dims.size()` 校验索引，并累加 `obs_dims[obs_id]`，把它当成观测项索引。因此，合法历史请求在所选帧索引大于等于观测项数量时可能返回空向量。碰巧使用较小帧索引的配置虽能得到正确值，却会预留错误容量，可能导致推理线程重新分配内存。

### 证据

- `src/rl_sar/library/core/observation_buffer/observation_buffer.cpp:100-132`
- `src/rl_sar/library/core/observation_buffer/observation_buffer.cpp:134-179`
- `src/rl_sar/library/core/rl_sdk/lw_configuration_validation.cpp:584-607`
- `src/rl_sar/library/core/safety/lw_runtime_core.hpp:545-561`
- `src/rl_sar/test/test_observation_buffer.cpp:1-67`

### 计划范围

- 每个请求索引仅按历史帧范围校验。
- 根据 `num_envs`、所选帧数及完整观测宽度，以防溢出的算术计算准确输出长度。
- 保留文档约定的时间优先和观测项优先顺序。
- 增加稀疏历史、单观测项缓冲区选择第 1 帧、无效索引、准确输出维度及两种优先模式的断言。
- 调用方拥有的缓冲区复用留给依赖本项的优化 `LW-062`。

### 验收标准

- 校验器接受的每个历史列表，在两种优先模式下都生成与模型约定完全一致的输入长度和顺序。
- 对至少包含两帧历史的单观测项缓冲区，`{1}` 合法，不能与第 1 个观测项混淆。
- 无效帧索引由明确且有测试的约定处理，不再仅悄然影响容量预留计算。
- 配置、观测缓冲区、推理、严格构建、受支持的内存与未定义行为检测测试及 `git diff --check` 通过。

### 解决记录

- **解决时间**： 2026-08-20T19:00:56+08:00
- **提交**： `e8669452`
- **批准范围**： `get_obs_vec()` 统一把 `obs_ids` 作为 历史帧
  索引，先按 `[0, history_length)` 完整校验，再按环境数、选中帧数和完整单帧
  观测 宽度进行溢出安全的精确容量计算。非法帧索引现在在生成任何部分
  输出前抛出 `std::out_of_range`；观测项 维度累加溢出在缓冲区分配前
  抛出 `std::overflow_error`。保持 时间优先/观测项优先 排列、请求帧顺序以及返回新 向量
  的现有 API，不实施 `LW-062` 的 调用方拥有/原地写入 缓冲区优化。
- **修改文件**： `.learnings/LW_REAL_DEPLOYMENT_ISSUES.md`、
  `src/rl_sar/CMakeLists.txt`、
  `src/rl_sar/library/core/observation_buffer/observation_buffer.{hpp,cpp}`、
  `src/rl_sar/test/test_observation_buffer.cpp`、
  `src/rl_sar/test/test_lw_configuration_validation.cpp`。
- **验证**： 原先仅打印结果且未注册的 观测缓冲区 示例已改为
  正式断言测试并加入 CTest，覆盖 时间优先/观测项优先 精确顺序、单观测项选择第 1 帧、
  稀疏历史、多 环境、空请求、上下界非法索引和维度溢出。配置测试验证
  合法 `{9}` 稀疏历史只产生一帧模型输入。定向测试 2/2 通过；当前 Debug
  完整构建及 51/51 CTest 通过；全新 `LW_STRICT_WARNINGS=ON` 构建及 51/51
  CTest 通过。隔离 AddressSanitizer/UndefinedBehaviorSanitizer 构建中的
  `observation_buffer` 和 `lw_configuration_validation` 2/2 通过。定向
  `cppcheck` 无告警，`git diff --check` 通过。未启动 ROS 节点、MuJoCo 图形界面、
  串口、IMU、摇杆、真机或电机；用户未跟踪技能目录保持未修改。
- **后续事项**： LW-061, LW-062, LW-063, LW-064, LW-065,
  LW-066

---

<a id="lw-061"></a>

## [LW-061] 统一动作参考校验与运行时门控的语义

**优先级**： P2 / 中
**状态**： resolved
**依赖**： LW-009, LW-012, LW-013, LW-054, LW-055

### 问题

配置校验将动作命令、动作锚点朝向和相位观测均归为需要动作资源。这适用于加载参考数据或获取动作时长。但要求当前代次动作参考快照的运行时标志，仅由 `whole_body_tracking/motion_command` 启用。仅含锚点朝向的策略同样解引用快照，却被允许在参考缺失或代次不匹配时继续执行，生成零值或过期朝向特征。相位使用预加载的动作长度，不会解引用逐周期快照；因此资源需求与实时参考需求是不同约定，目前却用一个局部标志和一个不完整的运行时标志表达。

### 证据

- `src/rl_sar/library/core/rl_sdk/lw_configuration_validation.cpp:562-568`
- `src/rl_sar/library/core/rl_sdk/lw_configuration_validation.cpp:618-641`
- `src/rl_sar/library/core/rl_sdk/lw_configuration_validation.cpp:699-712`
- `src/rl_sar/library/core/safety/lw_runtime_core.hpp:402-413`
- `src/rl_sar/library/core/rl_sdk/rl_sdk.cpp:408-472`
- `policy/LW/robot_lab/leg_to_wheel/config.yaml:4-5`
- `policy/LW/robot_lab/wheel_to_leg/config.yaml:4-5`

### 计划范围

- 分别表示并校验预加载动作资源需求与代次匹配的实时动作参考快照需求。
- 所有读取快照的观测（包括锚点朝向）均须获得当前代次快照；相位仅依赖有效动作时长，不增加无用的逐周期快照读取。
- 保留当前转换策略输出和动作时序。
- 增加仅动作命令、仅锚点、仅相位及组合策略的配置矩阵，覆盖代次不匹配和参考缺失。

### 验收标准

- 无观测会解引用缺失或跨代次的参考。
- 仅锚点策略等待正确参考，不再悄然输出零值或过期特征。
- 仅相位策略获得已验证的非零动作时长，推理周期不要求本来不会使用的实时快照。
- 现有转换资源数值等价，配置、运行时一致性、严格构建及受支持的内存与未定义行为检测测试通过。

### 解决记录

- **解决时间**： 2026-08-24T13:25:06+08:00
- **提交**： `161feb00`
- **批准范围**： 按用户调整后的明确审批，删除当前四个正式策略均未使用的
  `RoboMimic_Deploy/phase` 观测契约，保留含义不同的 `gait_phase`。动作命令 或 锚点朝向 任一出现时均设置现有
  `needs_motion_reference`，以同一标志完成 动作资源 预加载和推理期 实时参考 门控；不再新增 仅相位资源 标志。需要 参考 的推理只接受
  当前策略代际且所需 关节/锚点 载荷尺寸完整的快照，缺失、不完整或跨代时在
  推进帧、历史和 ONNX 前返回；非动作 策略不读取 参考通道。保持
  正式 YAML、ONNX、CSV、FSM 转换时序、动作加载器 数值规则和 S1-S4 行为
  不变，未处理 LW-062 或后续问题。
- **修改文件**： `src/rl_sar/library/core/rl_sdk/lw_configuration_validation.cpp`、
  `src/rl_sar/library/core/rl_sdk/rl_sdk.{hpp,cpp}`、
  `src/rl_sar/library/core/safety/lw_runtime_core.hpp`、
  `src/rl_sar/test/test_lw_configuration_validation.cpp`、
  `src/rl_sar/test/test_lw_runtime_parity.cpp`、
  `.learnings/LW_REAL_DEPLOYMENT_ISSUES.md`。
- **验证**： 定向构建配置校验、运行时一致性、动作参考频率、FSM 转换、`rl_real_LW` 和 `rl_sim_LW` 成功，4/4 定向 CTest 通过。新增配置
  矩阵覆盖 仅动作命令、仅锚点、组合、非动作 及已删除相位 的
  明确拒绝；运行时回归覆盖缺失、不完整、跨代和正确同代 参考、非动作
  无 参考 推理，以及两个正式转换 ONNX 的 真机/Sim2Sim 输出一致性。当前
  Debug 完整 51/51 CTest 通过；全新 `LW_STRICT_WARNINGS=ON` 构建全部维护目标
  并通过 51/51 CTest。全新 AddressSanitizer/UndefinedBehaviorSanitizer 构建中，
  `lw_configuration_validation` 与 `lw_runtime_parity` 各连续 5 次通过且无报告。
  定向 `cppcheck` 仅报告未修改的 `CSVInit(std::string)` 既有
  `passedByValue` 提示，`git diff --check` 通过。未启动 ROS 节点、MuJoCo 图形界面、
  串口、IMU、摇杆、真机或电机；用户未跟踪技能目录保持未修改。
- **后续事项**： LW-062, LW-063, LW-064, LW-065, LW-066

---

<a id="lw-062"></a>

## [LW-062] 在推理热路径中复用连续缓冲区

**优先级**： P2 / 低
**状态**： resolved
**依赖**： LW-050, LW-056, LW-060

### 问题

推理周期仍会创建中间 `vector<vector<float>>` 观测项，将其展平为新向量，通过另一返回向量值的接口取得历史，再为 `Model::forward()` 将输入包装为临时嵌套向量，最后将 ONNX 输出复制到新分配的向量。这些操作虽不在 200 Hz 命令循环内，却在每轮策略推理中发生，增加堆内存操作与延迟波动。现有内存分配回归覆盖受维护的控制周期，未覆盖该推理流水线。

### 证据

- `src/rl_sar/library/core/rl_sdk/rl_sdk.cpp:357-475`
- `src/rl_sar/library/core/observation_buffer/observation_buffer.cpp:108-179`
- `src/rl_sar/library/core/safety/lw_runtime_core.hpp:533-561`
- `src/rl_sar/library/core/inference_runtime/inference_runtime.cpp:195-209`
- `src/rl_sar/library/core/inference_runtime/inference_runtime.cpp:315-336`
- `src/rl_sar/test/test_lw_allocation_bound.cpp`

### 计划范围

- 缓存已验证的观测偏移，直接写入预先定长的连续缓冲区。
- 增加调用方拥有或原地写入的历史与推理输出接口，在策略激活时确定容量，而非每次推理时分配。
- 对受维护的单输入模型约定避免临时嵌套输入容器，同时保留明确的多输入扩展边界。
- 测量预热后推理侧项目代码的内存分配和延迟分布，不将不透明的 ONNX Runtime 内部分配误归因于项目代码。

### 验收标准

- 预热后的观测组装、历史展平、输入包装和输出提取不发生逐周期的项目自有动态内存分配。
- 四个受维护模型保持相同的输入顺序、维度和既有数值容差内的输出值。
- 策略切换时，在缓冲区首次使用前安全调整其大小。
- 内存分配、推理约定、运行时一致性、严格构建、受支持的内存与未定义行为检测测试及 `git diff --check` 通过。

### 解决记录

- **解决时间**： 2026-08-24T14:03:14+08:00
- **提交**： `e620d0b8`
- **批准范围**： 配置验证阶段缓存类型化观测项、偏移和已验证模型维度；
  策略切换边界据此一次性调整平坦观测、历史输入、动作、策略输出与 轨迹
  发布缓冲区。观测装配改为直接写入连续缓冲区，栈上固定尺寸四元数中间量保持
  原公式与顺序；历史缓冲改为连续环形存储并原地展开，保留 `time`、`term`、
  稀疏索引和多环境语义。模型主接口以非拥有 张量视图 写入调用方输出，当前
  ONNX 单输入契约仍显式校验 `input_count=1`，输入/输出 张量 直接绑定预分配
  存储；旧 返回值的 `forward()` 仅作为非热路径兼容包装。动作裁剪、轮腿
  输出、策略输出传输和 推理轨迹 发布均复用保留容量。未共享或调整
  `Ort::Env`，未处理 LW-063 或后续问题；ONNX Runtime 内部不透明分配不计作
  项目侧零分配结论。
- **修改文件**： `src/rl_sar/library/core/inference_runtime/inference_runtime.{hpp,cpp}`、
  `src/rl_sar/library/core/observation_buffer/observation_buffer.{hpp,cpp}`、
  `src/rl_sar/library/core/rl_sdk/lw_configuration_validation.{hpp,cpp}`、
  `src/rl_sar/library/core/rl_sdk/rl_sdk.{hpp,cpp}`、
  `src/rl_sar/library/core/safety/lw_runtime_core.hpp`、
  `src/rl_sar/test/test_inference_runtime.cpp`、
  `src/rl_sar/test/test_lw_allocation_bound.cpp`、
  `src/rl_sar/test/test_lw_configuration_validation.cpp`、
  `src/rl_sar/test/test_lw_runtime_parity.cpp`、
  `src/rl_sar/test/test_observation_buffer.cpp`、
  `.learnings/LW_REAL_DEPLOYMENT_ISSUES.md`。
- **验证**： 四个正式 ONNX 模型以修改前 `161feb0` 的确定输入输出保存
  十六进制基线，修改后 40 个输出均在 `1e-6` 容差内一致；四套配置的 39/41/59
  维观测顺序和数值与旧装配算法一致，历史输入维度分别保持 195/410/59，策略
  连续切换正常。无分配假模型覆盖预热后的控制输入、观测、历史、模型调用、动作
  后处理、策略输出和 轨迹 发布，四策略各 100 个完整周期均为 0 次项目侧动态
  分配。旧/新 仅主机性能分析器 各采集每策略 101 个样本，p50 从
  `232.558/274.076/200.850/204.401 us` 降至
  `150.212/206.094/160.534/200.000 us`；该短时同机数据仅作观测证据，不设硬
  阈值。当前 Debug 完整 51/51 CTest 通过；全新 `LW_STRICT_WARNINGS=ON` 构建
  全部维护目标并通过 51/51 CTest。全新 ASan+UBSan 构建中，内存分配、
  推理契约、运行时一致性、配置、观测缓冲区 和
  策略输出传输 六项连续 5 轮、共 30 次通过且无报告。定向 `cppcheck`
  仅报告明确留给 LW-063 的既有 `Ort::Env` 初始化提示和未修改的
  `CSVInit(std::string)` `passedByValue` 提示；`git diff --check` 通过。未启动
  ROS 节点、MuJoCo 图形界面、串口、IMU、摇杆、真机或电机；用户未跟踪技能目录保持
  未修改。
- **后续事项**： LW-063, LW-064, LW-065, LW-066

---

<a id="lw-063"></a>

## [LW-063] 共享 ONNX Runtime 环境且保持模型隔离

**优先级**： P2 / 低
**状态**： resolved
**依赖**： LW-027, LW-050, LW-058

### 问题

每个 `ONNXModel` 都创建并拥有独立 `Ort::Env`，但运行时环境是进程级基础设施，而部署会预加载多个模型。这导致启动资源重复且生命周期不清晰。该环境还在构造函数体内赋值，而不是与对象其他部分一同初始化。会话、模型元数据和张量缓冲区仍需独立拥有。

### 证据

- `src/rl_sar/library/core/inference_runtime/inference_runtime.cpp:85-110`
- `src/rl_sar/library/core/inference_runtime/inference_runtime.hpp:79-101`
- `src/rl_sar/library/core/rl_sdk/lw_configuration_validation.cpp:752-778`

### 计划范围

- 为所有受维护的 ONNX 会话提供所有权明确、生命周期覆盖进程的共享 `Ort::Env`，同时隔离会话与模型特有可变状态。
- 使初始化与销毁顺序确定且线程安全。
- 测量启动和资源影响，仅在有证据时保留优化，避免引入不必要的全局状态抽象。

### 验收标准

- 并发创建、推理及销毁所有受维护模型会话时，会话不会超出共享环境生命周期，也不共享可变会话数据。
- 模型校验、动态批次行为、运行库来源和数值输出不变。
- 定向回归验证环境生命周期及多模型销毁；推理、严格构建和受支持的内存与未定义行为检测测试通过。

### 解决记录

- **解决时间**： 2026-08-24T14:30:44+08:00
- **提交**： `de1f55fc`
- **批准范围**： `ONNXModel` 现在通过 C++17 线程安全的函数局部静态
  `shared_ptr` 获取进程级、只读 `Ort::Env`；每个模型保留一份共享所有权引用，
  且环境成员声明在 会话 之前，确保每个独立 会话 先析构、环境引用后
  释放，即使跨静态析构顺序也不会让存活 会话 悬空。`Ort::Session`、局部
  `SessionOptions`、节点名称、张量元数据、尺寸缓存和推理缓冲仍由各模型独立
  持有。未改变模型工厂、单输入输出/静态 批次 契约、线程数、运行时 来源
  或数值路径，未处理 LW-064 或后续问题。
- **修改文件**： `src/rl_sar/library/core/inference_runtime/inference_runtime.{hpp,cpp}`、
  `src/rl_sar/test/test_inference_runtime.cpp`、`src/rl_sar/CMakeLists.txt`、
  `.learnings/LW_REAL_DEPLOYMENT_ISSUES.md`。
- **验证**： 新增回归在共享环境首次初始化前同时创建四个不同维度的
  合成 ONNX 会话，并连续 8 轮并发加载、推理、交错销毁部分 会话，
  再验证剩余 会话 的元数据和数值未交叉污染。四个正式策略的既有配置、
  模型契约、预热和固定输出回归保持通过。相对 `e620d0b` 的 分离检出基线，
  同机交替 31 组四策略验证进程样本中，耗时中位数从 `63.526 ms` 降至
  `60.623 ms`（约 `-4.6%`），峰值 RSS 中位数从 `28,876 KiB` 降至
  `28,520 KiB`（`-356 KiB`）。当前 Debug 完整 51/51 CTest 通过；全新
  `LW_STRICT_WARNINGS=ON` 构建全部维护目标并通过 51/51 CTest。全新
  ASan+UBSan 构建中，配置 与 推理生命周期 两项各连续 5 次、
  共 10 次通过且无报告。定向 `cppcheck` 在仅抑制供应商
  `onnxruntime_float16.h` 的端序预处理误报后无项目告警，`git diff --check`
  通过。未启动 ROS 节点、MuJoCo 图形界面、串口、IMU、摇杆、真机或电机；用户未
  跟踪技能目录保持未修改。
- **后续事项**： LW-064, LW-065, LW-066

---

<a id="lw-064"></a>

## [LW-064] 移除或正确实现具有误导性的 FDILink CRC32 接口

**优先级**： P2 / 低
**状态**： resolved
**依赖**： LW-044, LW-045

### 问题

`CRC32_Table()` 声明为 32 位校验和接口，却用 16 位整数保存状态，并索引 CRC16 表。因此实际计算的是 CRC16 后再扩展结果，而非 CRC32。受维护调用点均未使用此函数，当前驱动协议不受影响；但未来调用方会因公开名称误导而悄然收到无效校验和。

### 证据

- `src/fdilink_ahrs_ROS2/include/crc_table.h:8`
- `src/fdilink_ahrs_ROS2/src/crc_table.cpp:140-158`
- 全仓库调用点搜索仅发现声明和定义。

### 计划范围

- 确认受支持的 FDILink 协议无需此接口。
- 若为死代码则移除声明与定义；仅在有明确外部兼容要求时，才实现规格完整的 CRC32 变体。
- 如保留，记录多项式、初值、反射规则和最终异或，并验证标准及协议特定的已知答案测试向量。

### 验收标准

- 代码不再暴露名称为 CRC32、实际计算 CRC16 的函数。
- 现有 CRC8/CRC16 包校验不变。
- 解析器、载荷测试及新增已知答案测试在严格构建和受支持的内存与未定义行为检测构建下通过。

### 解决记录

- **解决时间**： 2026-08-24T15:06:25+08:00
- **提交**： `fea0ff9a`
- **批准范围**： 仓库调用点、帧解析器和构建安装边界确认当前支持的
  IMU、AHRS、INSGPS、大地坐标位置 及忽略帧均只使用头部 CRC8 和载荷
  CRC16，`fdilink_protocol` 与其头文件也不作为已安装公共接口发布。因此按
  批准方案删除无调用的 `CRC32_Table()` 声明、实际重复 CRC16 算法的错误实现，
  以及从未被引用的 256 项 CRC32 查找表；未猜测或新增任何 CRC32 变体，未调整
  CRC8/CRC16 签名、算法、帧结构或解析行为，未处理 LW-065 或后续问题。
- **修改文件**： `src/fdilink_ahrs_ROS2/include/crc_table.h`、
  `src/fdilink_ahrs_ROS2/src/crc_table.cpp`、
  `src/fdilink_ahrs_ROS2/test/test_fdilink_frame_parser.cpp`、
  `.learnings/LW_REAL_DEPLOYMENT_ISSUES.md`。
- **验证**： 新增独立标准已知答案，固定 CRC8/MAXIM 的
  `"123456789" -> 0xA1` 和 CRC16/XMODEM 的 `"123456789" -> 0x31C3`；
  当前普通构建的解析、解码、语义校验、序号跟踪和进程生命周期 5/5 CTest
  通过。全新 `-Wall -Wextra -Wpedantic -Werror` 构建协议库、AHRS 驱动和
  四个 C++ 功能测试成功，5/5 功能 CTest 通过；全包 全部目标 另行暴露未修改
  `imu_tf.cpp` 使用 ROS 废弃头文件的既有告警，保持给后续 FDILink 清理而未并入
  本项。沿用 LW-046/LW-047 的受支持 `-fsanitize=undefined,alignment`
  口径，五项测试各连续 5 次、共 25 次通过且无报告；补充 ASan+UBSan 下四个
  纯 C++ 测试各 5 次、共 20 次通过，ROS 进程测试受未修改的 ROS/Conda
  分配器 ABI `new-delete-type-mismatch` 限制。当前 `cppcheck` 通过，仓库搜索
  和普通/UBSan 静态库符号表均只保留 CRC8/CRC16，`git diff --check` 通过。
  未启动 ROS 节点、访问串口、AHRS、IMU、真机或电机；用户未跟踪技能目录保持
  未修改。
- **后续事项**： LW-065, LW-066

---

<a id="lw-065"></a>

## [LW-065] 恢复 FDILink 代码规范检查与软件包元数据的干净基线

**优先级**： P2 / 低
**状态**： resolved
**结项记录提交**： `e3596f48`
**依赖**： LW-044, LW-045, LW-046, LW-047, LW-064

### 问题

强制重新配置后，FDILink 五项功能测试全部通过，但八项 ament 规范检查中五项失败：copyright、cpplint、flake8、lint_cmake 和 uncrustify。软件包清单仍使用版本 `0.0.0`，描述、维护者邮箱及许可证字段仍有 TODO。当前尚未据此证明运行时缺陷，但持续集成一直失败，基线噪声掩盖未来退化，软件包来源信息也不适合发布。

### 证据

- `src/fdilink_ahrs_ROS2/package.xml:5-8`
- `src/fdilink_ahrs_ROS2/launch/ahrs_driver.launch.py`
- `src/fdilink_ahrs_ROS2/launch/imu_tf.launch.py`
- `src/fdilink_ahrs_ROS2/CMakeLists.txt:53`
- `src/fdilink_ahrs_ROS2/CMakeLists.txt:121`
- 重新配置后的 FDILink CTest 结果为 8/13 通过；五项功能测试通过，上述五项代码规范检查失败。

### 计划范围

- 使用用户确认的值建立准确的软件包所有权、描述、许可证、维护者和版本元数据。
- 仅在 FDILink 包内进行保持行为不变的格式与版权清理。
- 保持启动参数、话题名称、协议解码和进程行为不变，使规范检查失败重新具有可操作性。

### 验收标准

- 按用户 2026-09-05 的范围调整，本地验收不要求版权声明：移除独立版权测试，仅过滤 cpplint 的 legal/copyright 类别。其余 12 项 CTest 全部通过。
- 软件包元数据无 TODO 占位，符合仓库实际许可与所有权决策。
- 格式调整前后，启动文件生成相同节点、参数和话题。
- 不捆绑无关 rl_sar 格式或行为修改。

### 解决记录 (2026-09-05)

- 用户批准实施 LW-065，并明确维护者为 `liufengrong
  <1044867193@qq.com>`；用户表示无法提供 FDILink 原始代码来源或许可证。
  用户随后明确“无需版权声明，对我来说可以使用即可”，因此调整本地验收范围，
  按上述 12 项检查完成本项。已随 `e3596f48` 提交。
- 已整理包内 C++、启动文件 和 CMake 格式，修正头文件保护宏、头文件包含顺序及
  命名空间导入；日志计数使用 `PRIu64` 与原有 uint64 值匹配。保留既有全局
  参数字符串的存储与生命周期，仅对这两行说明并限定 `runtime/string`代码规范检查例外。uncrustify 显式按 CPP 解析包含命名空间和模板的 .h 文件，避免与
  cpplint 的 C++ 格式规则冲突。按用户要求停用独立版权检查，新增包内
  CPPLINT.cfg 仅过滤 legal/copyright，其余检查继续启用。
- package.xml 已更新版本 0.1.0、驱动描述和用户确认的维护者；许可证字段仍
  如实填写 `License not declared`，未补写原始代码版权或推定授权声明。
- imu_tf.cpp 已改用 tf2_geometry_msgs.hpp；全新全包
  `-Wall -Wextra -Wpedantic -Werror -fsanitize=undefined,alignment` 构建成功，
  包括两个节点与四个 C++ 测试。五项功能测试均通过且无 UBSan 报告。
  初次全部 CTest 为 11/13，仅版权声明相关检查失败；应用用户明确调整后的
  验收范围，普通构建和上述严格警告/UBSan 构建重新配置后的全部 CTest
  均为 12/12 通过。
- 两个启动文件的 Node 与 LaunchDescription 调用参数经修改前后 AST 对比
  一致；保留 imu_tf 启动文件 原有缺少 executable 的状态。git diff --check
  通过。生命周期测试仅使用伪终端/不存在的设备路径，未访问真实硬件。
- 原始代码来源与授权仍未核实；本地验收豁免不表示获得了新的授权。
  用户未跟踪技能目录保持原样，LW-066 未处理。

---

<a id="lw-066"></a>

## [LW-066] 确保依赖查找顺序正确并将构建设置限定到目标

**优先级**： P2 / 低
**状态**： resolved
**结项记录提交**： `83e22d55`
**依赖**： LW-018, LW-027, LW-058

### 问题

生产 ONNX 来源校验命令在本 CMake 文件调用 `find_package(Python3)` 之前使用 `${Python3_EXECUTABLE}`。当前 ROS/ament 环境碰巧足够早地填充该变量，使已验证构建成功，但这是隐式顺序依赖。软件包还保留全局编译定义、头文件目录、链接目录、链接器选项及累积 RPATH，可能泄漏到无关库和测试，使依赖来源难以判断。

### 证据

- `src/rl_sar/CMakeLists.txt:105-107`
- `src/rl_sar/CMakeLists.txt:195`
- `src/rl_sar/CMakeLists.txt:236-259`
- `src/rl_sar/CMakeLists.txt:311-334`
- `src/rl_sar/CMakeLists.txt:457-477`
- `src/rl_sar/CMakeLists.txt:531-548`
- `src/rl_sar/CMakeLists.txt:947-983`

### 计划范围

- 在首次使用前发现 Python 解释器，使生产来源校验独立于偶然存在的 ament 缓存状态。
- 将受维护的编译定义、头文件路径、链接搜索路径、链接选项和运行时路径移到最小所属目标范围。
- 保留获准 ONNX/MuJoCo 运行库来源、Jetson 行为、安装布局及 `LW-058` 确立的无 Python 运行库链接保证。
- 增加不继承旧 CMake 缓存的全新配置与构建检查。

### 验收标准

- 全新生产配置在任何来源校验命令使用前，解析并执行预期 Python 解释器。
- 代表性控制和测试目标仅暴露所需编译、链接及 RPATH 属性，不重新引入 Conda 或非预期依赖。
- Debug、严格构建、生产 Release、Jetson 配置、运行时链接及构建流程回归在干净构建目录中通过。
- 生成的部署路径和已安装运行库解析保持不变。

### 解决记录 (2026-09-05)

- 用户明确批准实施 LW-066；仅修改 rl_sar 的 CMake、构建工作流测试，新增
  生成目标属性回归测试，并更新本项记录；这些变更与本记录一并提交。
- 显式发现 Python 解释器 已移到 ament 及生产 来源 命令之前。
  全新 Debug/生产配置自动找到系统 Python 3.10；严格构建显式选用 Conda
  Python 3.13.9，仍不向 C++ 目标引入 Python 运行库。
- 去掉全局定义、头文件目录、链接搜索目录、链接器选项和累计 RPATH。
  各库公开自身头文件与实际依赖；USE_ONNX 由 inference_runtime PUBLIC
  传播以保持公开类布局一致，POLICY_DIR 仅由开发版 rl_sdk PUBLIC 传播，
  模拟器源码路径宏仅属于 rl_sim_LW；删除未使用的全局 Boost 宏。严格警告
  PRIVATE 应用于本目录受维护编译目标，供应商目标不继承这些警告选项。
- yaml-cpp 通过显式 CONFIG 查找和导入目标链接；Linux 使用系统 多架构
  配置目录，与已有系统 fmt 隔离策略一致。ONNX/MuJoCo 链接选项仅传播到
  实际消费者；构建 RPATH 由目标依赖生成，安装路径仅设置到对应安装目标。
  生产真机与性能分析器 保留原有 `$ORIGIN/onnxruntime` 和安装布局。
- 新增 lw_build_target_scope，读取 compile_commands.json 和 ELF 动态段，
  检查头文件/ABI 宏、维护与 供应商警告范围、普通目标依赖隔离，以及代表性
  二进制的 ONNX/MuJoCo RPATH 与无 Python/Conda 链接约束。构建工作流测试
  同时锁定 Python 发现顺序和禁止全局设置。用旧构建产物作负对照时，
  新检查按预期拒绝旧的全局 ONNX 设置。
- 全新 `/tmp/lw066-debug` 与 `/tmp/lw066-strict` 构建成功；两者完整 CTest
  均为 **52/52** 通过。严格构建使用 `LW_STRICT_WARNINGS=ON`。
  独立干净副本 `/tmp/lw066-production.Jq9VTu/repo` 的生产 Release 构建
  成功，完整 CTest **48/48** 通过；配置未传入 Python3_EXECUTABLE，ONNX
  来源 校验正常执行。隔离副本验证快照为 `10131b1`，不是主分支提交。
- 在该隔离副本运行现有 build_lw_deployment.sh，重新构建 serial、FDILink
  和 rl_sar；安装及迁移后的包解析、启动文件参数解析、动态库解析、策略哈希
  和 `--verify-deployment-only` 均通过。readelf 确认生产 RPATH 精确为
  `$ORIGIN/onnxruntime`；ldd 确认 ONNX 位于包内，yaml-cpp/fmt 来自系统，
  不含 libpython/Conda/MuJoCo。保留验证包于
  `/tmp/lw066-production.Jq9VTu/bundle`，并非正式发布包。
- 全新 `/tmp/lw066-jetson-config` 用平台检测测试覆盖项进入 Jetson 模式，
  未生成 MuJoCo/rl_sim_LW 目标；Jetson 检测、运行库架构、构建工作流三项
  CTest **3/3** 通过。缺失 ONNX 的全新 Jetson 配置按预期失败。本机为
  x86_64，这只是配置分支回归，不宣称已完成 aarch64 编译或 Jetson 实机验证。
- git diff --check 通过。未改动策略或控制逻辑，未访问真实串口、IMU、
  电机或启动 MuJoCo 图形界面；部署进程仅运行验证模式。用户未跟踪技能目录
  保持原样，未纳入修改或提交；未处理其他问题。

---
