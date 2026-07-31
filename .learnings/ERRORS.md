# Errors

## [ERR-20260731-001] apply_patch

**Logged**: 2026-07-31T16:16:20+08:00
**Priority**: medium
**Status**: resolved
**Area**: config

### Summary
更新学习条目状态时，非唯一补丁上下文短暂命中了相邻条目。

### Error
```text
目标 LRN-20260731-002 仍为 pending；相邻条目的状态和 Resolution 区块被误改。
```

### Context
- 操作：将 `LRN-20260731-002` 标记为 `resolved` 并增加验证证据。
- 原因：首个补丁仅匹配通用的 `**Status**: pending` 和邻近 Metadata 尾部，没有在每个修改块中包含唯一条目 ID。
- 发现方式：写入后使用 `rg` 检查目标条目、全部状态及 Resolution 区块位置。
- 修复：恢复相邻条目原值，并以各自条目 ID 为上下文重新应用精确补丁。

### Suggested Fix
修改结构相同的 Markdown 日志时，每个补丁块都应包含唯一条目 ID；写入后检查目标条目和相邻条目的状态与区块位置。

### Metadata
- Reproducible: yes
- Related Files: .learnings/LEARNINGS.md
- See Also: N/A

### Resolution
- **Resolved**: 2026-07-31T16:16:20+08:00
- **Commit/PR**: N/A
- **Notes**: 已恢复误改内容并验证只有 `LRN-20260731-002` 获得本次 Resolution。

---
