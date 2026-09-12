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
