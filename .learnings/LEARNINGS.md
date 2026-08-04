# Learnings

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
**Status**: pending
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
