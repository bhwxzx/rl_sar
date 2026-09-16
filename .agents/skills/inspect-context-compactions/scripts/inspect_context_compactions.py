#!/usr/bin/env python3
"""Inspect one Codex rollout and report a verified compaction count."""

from __future__ import annotations

import argparse
import json
import os
import re
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Any


DEFAULT_THRESHOLD = 5
DEFAULT_STABILITY_RETRIES = 2
DEFAULT_RETRY_DELAY_SECONDS = 0.05
THREAD_ID_PATTERN = re.compile(r"[A-Za-z0-9][A-Za-z0-9._-]*\Z")


def _default_sessions_root() -> Path:
    codex_home = os.environ.get("CODEX_HOME")
    if codex_home:
        return Path(codex_home).expanduser() / "sessions"
    return Path.home() / ".codex" / "sessions"


def _base_result(thread_id: str | None, threshold: int) -> dict[str, Any]:
    return {
        "version": 1,
        "provider": "codex",
        "status": "unavailable",
        "checked_at": datetime.now(timezone.utc).isoformat(),
        "thread_id": thread_id,
        "rollout_path": None,
        "compaction_count": None,
        "window_numbers": [],
        "compaction_timestamps": [],
        "threshold": threshold,
        "threshold_reached": None,
        "event_cross_check": {
            "context_compacted_count": None,
            "matches": None,
        },
        "errors": [],
    }


def _resolve_rollout(
    sessions_root: Path,
    thread_id: str | None,
    rollout_path: Path | None,
) -> tuple[Path | None, list[str]]:
    if rollout_path is not None:
        resolved = rollout_path.expanduser().resolve()
        if not resolved.is_file():
            return None, ["rollout_not_found"]
        return resolved, []
    if not thread_id:
        return None, ["codex_thread_id_unavailable"]
    if THREAD_ID_PATTERN.fullmatch(thread_id) is None:
        return None, ["invalid_thread_id"]
    root = sessions_root.expanduser().resolve()
    if not root.is_dir():
        return None, ["sessions_root_not_found"]
    matches = sorted(root.rglob(f"rollout-*-{thread_id}.jsonl"))
    if not matches:
        return None, ["rollout_not_found"]
    if len(matches) != 1:
        return None, ["multiple_rollouts_match_thread_id"]
    return matches[0].resolve(), []


def _rollout_signature(path: Path) -> tuple[int, int] | None:
    try:
        stat = path.stat()
    except OSError:
        return None
    return stat.st_size, stat.st_mtime_ns


def _has_retryable_tail_error(report: dict[str, Any]) -> bool:
    errors = report.get("errors")
    if not isinstance(errors, list):
        return False
    return any(
        error == "context_compacted_event_count_mismatch"
        or (isinstance(error, str) and error.startswith("invalid_json_line:"))
        for error in errors
    )


def _inspect_rollout_once(
    path: Path,
    thread_id: str | None,
    threshold: int,
) -> dict[str, Any]:
    result = _base_result(thread_id, threshold)
    result["rollout_path"] = str(path)
    errors: list[str] = []
    session_ids: set[str] = set()
    window_numbers: list[int] = []
    compaction_timestamps: list[str | None] = []
    context_compacted_count = 0

    try:
        with path.open("r", encoding="utf-8") as stream:
            for line_number, raw_line in enumerate(stream, 1):
                if not raw_line.strip():
                    continue
                try:
                    record = json.loads(raw_line)
                except json.JSONDecodeError:
                    errors.append(f"invalid_json_line:{line_number}")
                    continue
                if not isinstance(record, dict):
                    errors.append(f"non_object_record:{line_number}")
                    continue
                record_type = record.get("type")
                payload = record.get("payload")
                if record_type == "session_meta" and isinstance(payload, dict):
                    session_id = payload.get("id") or payload.get("session_id")
                    if isinstance(session_id, str) and session_id:
                        session_ids.add(session_id)
                elif record_type == "compacted":
                    if not isinstance(payload, dict):
                        errors.append(f"invalid_compacted_payload:{line_number}")
                        continue
                    window_number = payload.get("window_number")
                    if (
                        isinstance(window_number, bool)
                        or not isinstance(window_number, int)
                        or window_number < 1
                    ):
                        errors.append(f"invalid_window_number:{line_number}")
                        continue
                    window_numbers.append(window_number)
                    timestamp = record.get("timestamp")
                    compaction_timestamps.append(
                        timestamp if isinstance(timestamp, str) else None
                    )
                elif (
                    record_type == "event_msg"
                    and isinstance(payload, dict)
                    and payload.get("type") == "context_compacted"
                ):
                    context_compacted_count += 1
    except OSError:
        result["errors"] = ["rollout_read_failed"]
        return result

    if not session_ids:
        errors.append("session_meta_missing")
    elif len(session_ids) != 1:
        errors.append("multiple_session_ids_in_rollout")
    else:
        session_id = next(iter(session_ids))
        if thread_id and thread_id != session_id:
            errors.append("rollout_thread_id_mismatch")
        elif not thread_id:
            result["thread_id"] = session_id

    expected_windows = list(range(1, len(window_numbers) + 1))
    if window_numbers != expected_windows:
        errors.append("compaction_window_sequence_invalid")
    event_matches = context_compacted_count == len(window_numbers)
    if not event_matches:
        errors.append("context_compacted_event_count_mismatch")

    result["window_numbers"] = window_numbers
    result["compaction_timestamps"] = compaction_timestamps
    result["event_cross_check"] = {
        "context_compacted_count": context_compacted_count,
        "matches": event_matches,
    }
    result["errors"] = errors
    if errors:
        result["status"] = "inconsistent"
        return result

    count = len(window_numbers)
    result["status"] = "available"
    result["compaction_count"] = count
    result["threshold_reached"] = count >= threshold
    return result


def _inspect_rollout(
    path: Path,
    thread_id: str | None,
    threshold: int,
    *,
    stability_retries: int,
    retry_delay_seconds: float,
) -> dict[str, Any]:
    last_result: dict[str, Any] | None = None
    changed_during_last_read = False
    for attempt in range(stability_retries + 1):
        signature_before = _rollout_signature(path)
        last_result = _inspect_rollout_once(path, thread_id, threshold)
        signature_after = _rollout_signature(path)
        changed_during_last_read = signature_before != signature_after
        should_retry = changed_during_last_read or _has_retryable_tail_error(
            last_result
        )
        if not should_retry:
            return last_result
        if attempt < stability_retries and retry_delay_seconds:
            time.sleep(retry_delay_seconds)

    assert last_result is not None
    if changed_during_last_read:
        result = _base_result(thread_id, threshold)
        result["rollout_path"] = str(path)
        result["errors"] = ["rollout_changed_during_read"]
        return result
    return last_result


def inspect_context_compactions(
    *,
    sessions_root: Path,
    threshold: int = DEFAULT_THRESHOLD,
    thread_id: str | None = None,
    rollout_path: Path | None = None,
    stability_retries: int = DEFAULT_STABILITY_RETRIES,
    retry_delay_seconds: float = DEFAULT_RETRY_DELAY_SECONDS,
) -> dict[str, Any]:
    """Return a content-free compaction count for one exact Codex thread."""
    if threshold < 1:
        raise ValueError("threshold must be a positive integer")
    if stability_retries < 0:
        raise ValueError("stability_retries must be non-negative")
    if retry_delay_seconds < 0:
        raise ValueError("retry_delay_seconds must be non-negative")
    resolved, errors = _resolve_rollout(sessions_root, thread_id, rollout_path)
    if resolved is None:
        result = _base_result(thread_id, threshold)
        result["errors"] = errors
        return result
    return _inspect_rollout(
        resolved,
        thread_id,
        threshold,
        stability_retries=stability_retries,
        retry_delay_seconds=retry_delay_seconds,
    )


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--thread-id")
    parser.add_argument("--sessions-root", default=str(_default_sessions_root()))
    parser.add_argument("--rollout")
    parser.add_argument("--threshold", type=int, default=DEFAULT_THRESHOLD)
    parser.add_argument(
        "--stability-retries",
        type=int,
        default=DEFAULT_STABILITY_RETRIES,
    )
    parser.add_argument(
        "--retry-delay-seconds",
        type=float,
        default=DEFAULT_RETRY_DELAY_SECONDS,
    )
    args = parser.parse_args()
    if args.threshold < 1:
        parser.error("--threshold must be a positive integer")
    if args.stability_retries < 0:
        parser.error("--stability-retries must be non-negative")
    if args.retry_delay_seconds < 0:
        parser.error("--retry-delay-seconds must be non-negative")

    thread_id = args.thread_id
    if thread_id is None and args.rollout is None:
        thread_id = os.environ.get("CODEX_THREAD_ID")
    report = inspect_context_compactions(
        sessions_root=Path(args.sessions_root),
        threshold=args.threshold,
        thread_id=thread_id,
        rollout_path=Path(args.rollout) if args.rollout else None,
        stability_retries=args.stability_retries,
        retry_delay_seconds=args.retry_delay_seconds,
    )
    print(json.dumps(report, indent=2, sort_keys=True, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
