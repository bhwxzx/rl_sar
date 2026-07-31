# Agent Instructions

## Code Change Approval

- Before modifying any source code, tests, scripts, or other code files, first describe the intended changes and obtain the user's explicit approval.
- Do not infer approval merely from a request to investigate, diagnose, review, or propose a fix.
- Read-only inspection and analysis may proceed without approval.

## LW Real-Deployment Remediation

- Use `.learnings/LW_REAL_DEPLOYMENT_ISSUES.md` as the authoritative issue order and status record.
- Work on only the single `LW-XXX` issue explicitly selected by the user.
- Do not bundle later issues, opportunistic refactors, or unrelated cleanup into an approved change.
- After verification, update only the selected issue's status and resolution evidence.

## Context Compaction Handoff

- Count only context compactions that are explicitly visible in the current
  session; do not guess an unseen count.
- Treat the third automatic context compaction in one session as the handoff
  threshold.
- At that threshold, finish any in-progress atomic operation, then create
  `.learnings/session_handoffs/YYYYMMDD-HHMM-<topic>.md` before starting more
  substantial work.
- The handoff document must record the current objective, completed work and
  commits, user decisions and approvals, applicable constraints, verification
  results, dirty or user-owned files that must be preserved, unresolved issues,
  blockers, and the exact next step.
- Tell the user that repeated compaction may reduce continuity, recommend
  continuing in a new session, link the handoff document, and provide a
  copyable prompt that instructs the new session to read `AGENTS.md`, the
  handoff document, and any authoritative task record before continuing.
- Fill and provide this prompt template:

  ```text
  请在 /home/lfr/rl_sar 继续上一会话的任务。开始前请完整阅读
  AGENTS.md、<handoff-document> 和 <authoritative-task-record>，然后运行
  git status --short 核对工作区并保留交接文档标记的用户文件。不要重复
  已完成或已提交的工作。当前应继续：<exact-next-step>。修改代码前必须
  按 AGENTS.md 先说明方案并获得我的明确审批。
  ```

- If the user explicitly chooses to remain in the current session, continue,
  but refresh the handoff document after each additional visible compaction.
