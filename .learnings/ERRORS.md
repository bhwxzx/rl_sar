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

## [ERR-20260818-001] python-compileall-readonly-review

**Logged**: 2026-08-18T11:55:31+08:00
**Priority**: low
**Status**: resolved
**Area**: tests

### Summary
只读代码审查中使用 `python3 -m compileall`，在源码测试目录生成了忽略的
`__pycache__`，违反了审查不写工作区的约束。

### Error
```text
scripts/__pycache__
src/rl_sar/test/__pycache__
```

### Context
- 操作：批量检查 Python 语法。
- 原因：`compileall` 默认把 `.pyc` 写到各源码目录。
- 影响：只产生缓存文件，没有修改跟踪文件；交接标记的 `.agents` 技能缓存未触碰。

### Suggested Fix
只读审查应设置 `PYTHONDONTWRITEBYTECODE=1` 并使用 AST 解析，或把 `py_compile`
输出显式定向到临时目录；不得对源码树直接运行默认 `compileall`。

### Metadata
- Reproducible: yes
- Related Files: scripts/, src/rl_sar/test/
- See Also: N/A

### Resolution
- **Resolved**: 2026-08-18T11:55:31+08:00
- **Commit/PR**: N/A
- **Notes**: 已精确删除本次生成的两个缓存目录，并确认未触碰用户技能目录。

---

## [ERR-20260816-001] rg-shell-quoting

**Logged**: 2026-08-16T15:44:04+08:00
**Priority**: low
**Status**: resolved
**Area**: docs

### Summary
包含 Markdown 反引号的 `rg` 搜索模式被放入双引号，导致 Bash 将其解释为未闭合的命令替换。

### Error
```text
/bin/bash: -c: 行 1: 寻找匹配的 ``' 时遇到了未预期的 EOF
/bin/bash: -c: 行 6: 语法错误：未预期的文件结束符
```

### Context
- 操作：核对正式部署文档中是否仍有旧措辞。
- 原因：把包含反引号的多个搜索模式放进 Bash 双引号字符串。
- 影响：只读搜索未执行，没有修改文件。

### Suggested Fix
搜索 Markdown 反引号时使用单引号，并优先使用 `rg -F -e 'pattern'` 分别传入固定字符串模式。

### Metadata
- Reproducible: yes
- Related Files: docs/LW_BUILD_DEPLOYMENT_CN.md
- See Also: N/A

### Resolution
- **Resolved**: 2026-08-16T15:44:04+08:00
- **Commit/PR**: N/A
- **Notes**: 已改用单引号固定字符串模式重新核对，命令成功且未发现残留旧措辞。

---
