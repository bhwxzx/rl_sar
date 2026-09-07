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
