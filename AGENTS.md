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
