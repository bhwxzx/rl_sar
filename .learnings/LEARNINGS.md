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
