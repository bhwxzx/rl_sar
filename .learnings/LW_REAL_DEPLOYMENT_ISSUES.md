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

## 当前状态

截至 2026-09-06，已记录的 LW-001～LW-071 共 71 项均已结项，当前无待处理条目。
结项仅表示各项批准范围内的修改和所记录验证已完成，不代表所有实机风险已消除。
近期关于超时阈值、故障分级及恢复方式的讨论尚未形成获批修改项，本次整理不新增
编号、不重新打开历史问题，也不调整安全参数或处理策略。

## 已完成问题索引

详细问题说明、审批与验收证据按批次归档：

- [LW-001～LW-066](archive/LW_REAL_DEPLOYMENT_RESOLVED_001_066.md)
- [LW-067～LW-071](archive/LW_REAL_DEPLOYMENT_RESOLVED_067_071.md)

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

| <a id="lw-067"></a>[LW-067](archive/LW_REAL_DEPLOYMENT_RESOLVED_067_071.md#lw-067) | 修复构建目标检查对合法编译配置的误报 | resolved | `39a3d22` |
| <a id="lw-068"></a>[LW-068](archive/LW_REAL_DEPLOYMENT_RESOLVED_067_071.md#lw-068) | 将只读动作裁剪配置校验集中到所属边界 | resolved | `76dc6bb` |
| <a id="lw-069"></a>[LW-069](archive/LW_REAL_DEPLOYMENT_RESOLVED_067_071.md#lw-069) | 合并基础配置及启动超时参数的重复校验 | resolved | `eeb1d16` |
| <a id="lw-070"></a>[LW-070](archive/LW_REAL_DEPLOYMENT_RESOLVED_067_071.md#lw-070) | 将 ONNX 私有缓存不变量检查集中到加载阶段 | resolved | `8f32b25` |
| <a id="lw-071"></a>[LW-071](archive/LW_REAL_DEPLOYMENT_RESOLVED_067_071.md#lw-071) | 减少源码写法绑定和重复生命周期测试断言 | resolved | `d03c6bd` |

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
