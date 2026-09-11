#!/usr/bin/env python3
"""Query immutable tuning history without changing files or external state."""

from __future__ import annotations

from evidence_provenance import require_source_path

import argparse
import hashlib
import json
import os
import re
import stat
from datetime import datetime
from pathlib import Path
from typing import Any

from capture_effective_training_config import (
    DEFAULT_MAX_DIFF_ENTRIES,
    EffectiveConfigError,
    compare_effective_configs,
    load_and_validate_effective_config,
    load_run_identity,
)
from capture_run_identity import RunIdentityError, validate_run_identity
from record_tuning_experience import (
    ExperienceError,
    SLUG_RE,
    validate_effective_config_binding,
    validate_event,
    validate_event_evidence,
)


REPO_ROOT = Path(__file__).resolve().parents[4]
DEFAULT_ROOT = REPO_ROOT / "learnings" / "policy_tuning"
DEFAULT_MAX_EVENTS = 10_000
DEFAULT_MAX_EVENT_BYTES = 4 * 1024 * 1024
SHA256_RE = re.compile(r"^[0-9a-f]{64}$")
FINGERPRINT_FIELDS = (
    "observation_fingerprint",
    "reward_fingerprint",
    "deployment_fingerprint",
)


class ExperienceQueryError(ValueError):
    """Raised when a history query is unsafe, unbounded, or inconsistent."""


def _reject_symlink_components(path: Path, *, label: str) -> None:
    if not path.is_absolute():
        raise ExperienceQueryError(f"{label} must be absolute")
    current = Path(path.anchor)
    for component in path.parts[1:]:
        current /= component
        if current.is_symlink():
            raise ExperienceQueryError(
                f"{label} contains a symlinked path component: {current}"
            )


def _validate_nonempty(name: str, value: Any) -> None:
    if not isinstance(value, str) or not value:
        raise ExperienceQueryError(f"{name} must be a non-empty string")


def _validate_query_inputs(
    *,
    task: str,
    algorithm: str,
    host_id: str,
    observation_fingerprint: str,
    reward_fingerprint: str,
    deployment_fingerprint: str,
    max_events: int,
    max_event_bytes: int,
    max_diff_entries: int,
) -> None:
    for name, value in (("task", task), ("algorithm", algorithm), ("host-id", host_id)):
        if not isinstance(value, str) or not SLUG_RE.fullmatch(value):
            raise ExperienceQueryError(f"{name} must be a safe ASCII identifier")
    for name, value in (
        ("observation-fingerprint", observation_fingerprint),
        ("reward-fingerprint", reward_fingerprint),
        ("deployment-fingerprint", deployment_fingerprint),
    ):
        _validate_nonempty(name, value)
    if isinstance(max_events, bool) or not isinstance(max_events, int) or max_events <= 0:
        raise ExperienceQueryError("max-events must be a positive integer")
    if (
        isinstance(max_event_bytes, bool)
        or not isinstance(max_event_bytes, int)
        or max_event_bytes <= 0
    ):
        raise ExperienceQueryError("max-event-bytes must be a positive integer")
    if (
        isinstance(max_diff_entries, bool)
        or not isinstance(max_diff_entries, int)
        or max_diff_entries <= 0
    ):
        raise ExperienceQueryError("max-diff-entries must be a positive integer")


def _is_unknown(value: Any) -> bool:
    return value is None or (
        isinstance(value, str) and value.strip().casefold() == "unknown"
    )


def _event_paths(root: Path, task: str, max_events: int) -> list[Path]:
    task_dir = root / task
    _reject_symlink_components(task_dir, label="task history directory")
    if not task_dir.exists():
        return []
    if not task_dir.is_dir():
        raise ExperienceQueryError("task history path must be a directory")

    event_paths: list[Path] = []
    with os.scandir(task_dir) as run_entries:
        for run_entry in sorted(run_entries, key=lambda entry: entry.name):
            run_path = Path(run_entry.path)
            if run_entry.is_symlink():
                raise ExperienceQueryError(
                    f"task history contains a symlinked run directory: {run_path}"
                )
            if not run_entry.is_dir(follow_symlinks=False):
                continue
            if not SLUG_RE.fullmatch(run_entry.name):
                raise ExperienceQueryError(
                    f"task history contains an unsafe run directory: {run_path}"
                )
            event_directories = [run_path]
            nested = run_path / "events"
            _reject_symlink_components(nested, label="events directory")
            if nested.is_dir():
                event_directories.append(nested)
            for event_directory in event_directories:
                with os.scandir(event_directory) as candidate_entries:
                    for candidate in sorted(candidate_entries, key=lambda entry: entry.name):
                        if candidate.name.startswith(".") or not candidate.name.endswith(".json"):
                            continue
                        candidate_path = Path(candidate.path)
                        if candidate.is_symlink():
                            raise ExperienceQueryError(
                                f"task history contains a symlinked event file: {candidate_path}"
                            )
                        if not candidate.is_file(follow_symlinks=False):
                            continue
                        event_paths.append(candidate_path)
                        if len(event_paths) > max_events:
                            raise ExperienceQueryError(
                                f"history exceeds max-events={max_events}; narrow the query"
                            )
    return event_paths


def _read_stable_event(path: Path, max_event_bytes: int) -> tuple[bytes, os.stat_result]:
    before = path.lstat()
    if stat.S_ISLNK(before.st_mode) or not stat.S_ISREG(before.st_mode):
        raise ExperienceQueryError(f"event is not a regular non-symlinked file: {path}")
    if before.st_size > max_event_bytes:
        raise ExperienceQueryError(
            f"event exceeds max-event-bytes={max_event_bytes}: {path}"
        )
    flags = os.O_RDONLY | getattr(os, "O_NOFOLLOW", 0)
    descriptor = os.open(path, flags)
    try:
        opened = os.fstat(descriptor)
        if (opened.st_dev, opened.st_ino) != (before.st_dev, before.st_ino):
            raise ExperienceQueryError(f"event changed before read: {path}")
        with os.fdopen(descriptor, "rb", closefd=False) as stream:
            encoded = stream.read(max_event_bytes + 1)
    finally:
        os.close(descriptor)
    after = path.lstat()
    stable_fields = ("st_dev", "st_ino", "st_size", "st_mtime_ns")
    if any(getattr(before, field) != getattr(after, field) for field in stable_fields):
        raise ExperienceQueryError(f"event changed during read: {path}")
    if len(encoded) > max_event_bytes:
        raise ExperienceQueryError(
            f"event exceeds max-event-bytes={max_event_bytes}: {path}"
        )
    return encoded, after


def _timestamp_slug(timestamp: str) -> str:
    return re.sub(r"[^0-9A-Za-z]+", "-", timestamp).strip("-")


def _validate_storage_binding(event: dict[str, Any], path: Path, task: str) -> None:
    if event["version"] == 5:
        if path.parent.name != "events":
            raise ExperienceQueryError("version-5 event must be under events/")
        run_id = path.parent.parent.name
    else:
        run_id = path.parent.name
    if event["task"] != task or event["run_id"] != run_id:
        raise ExperienceQueryError(
            "event scope does not match its task/run history directory"
        )
    expected_name = (
        f"{_timestamp_slug(event['recorded_at'])}__{event['event_id']}.json"
    )
    if path.name != expected_name:
        raise ExperienceQueryError("event filename does not match recorded_at/event_id")


def _extract_evidence_refs(value: Any, json_path: str = "evidence") -> list[dict[str, Any]]:
    references: list[dict[str, Any]] = []
    if isinstance(value, dict):
        direct_path = value.get("path")
        if isinstance(direct_path, str) and Path(direct_path).is_absolute():
            direct_hash = value.get("sha256")
            references.append(
                {
                    "json_path": f"{json_path}.path",
                    "path": direct_path,
                    "sha256": (
                        direct_hash
                        if isinstance(direct_hash, str)
                        and SHA256_RE.fullmatch(direct_hash)
                        else None
                    ),
                }
            )
        for key in sorted(value):
            child = value[key]
            child_json_path = f"{json_path}.{key}"
            if (
                key.endswith("_path")
                and isinstance(child, str)
                and Path(child).is_absolute()
            ):
                companion = value.get(f"{key[:-5]}_sha256")
                references.append(
                    {
                        "json_path": child_json_path,
                        "path": child,
                        "sha256": (
                            companion
                            if isinstance(companion, str)
                            and SHA256_RE.fullmatch(companion)
                            else None
                        ),
                    }
                )
            references.extend(_extract_evidence_refs(child, child_json_path))
    elif isinstance(value, list):
        for index, child in enumerate(value):
            references.extend(_extract_evidence_refs(child, f"{json_path}[{index}]"))
    unique = {
        (item["json_path"], item["path"], item["sha256"]): item
        for item in references
    }
    return [unique[key] for key in sorted(unique)]


def _classify_event(
    event: dict[str, Any],
    *,
    algorithm: str,
    host_id: str,
    observation_fingerprint: str,
    reward_fingerprint: str,
    deployment_fingerprint: str,
) -> tuple[str, list[str]]:
    event_host = (
        event["run_identity"]["host_id"] if event["version"] >= 2 else None
    )
    query_values = {
        "algorithm": algorithm,
        "host_id": host_id,
        "observation_fingerprint": observation_fingerprint,
        "reward_fingerprint": reward_fingerprint,
        "deployment_fingerprint": deployment_fingerprint,
    }
    event_values = {
        "algorithm": event["algorithm"],
        "host_id": event_host,
        **{field: event["context"].get(field) for field in FINGERPRINT_FIELDS},
    }
    conflicts: list[str] = []
    unknowns: list[str] = []
    if event["version"] < 3:
        unknowns.append("event_effective_config_unknown")
    for field in query_values:
        query_value = query_values[field]
        event_value = event_values[field]
        if _is_unknown(query_value):
            unknowns.append(f"query_{field}_unknown")
        elif _is_unknown(event_value):
            unknowns.append(f"event_{field}_unknown")
        elif query_value != event_value:
            conflicts.append(f"{field}_mismatch")
    if event["version"] < 3:
        return "unknown", conflicts + unknowns
    if conflicts:
        return "conflicting", conflicts + unknowns
    if unknowns:
        return "unknown", unknowns
    return "compatible", []


def _configuration_comparison_eligible(
    event: dict[str, Any],
    *,
    algorithm: str,
    host_id: str,
    observation_fingerprint: str,
    deployment_fingerprint: str,
) -> bool:
    if event["version"] not in {3, 4, 5}:
        return False
    event_values = (
        event["algorithm"],
        event["run_identity"]["host_id"],
        event["context"].get("observation_fingerprint"),
        event["context"].get("deployment_fingerprint"),
    )
    query_values = (
        algorithm,
        host_id,
        observation_fingerprint,
        deployment_fingerprint,
    )
    return not any(_is_unknown(value) for value in (*event_values, *query_values)) and (
        event_values == query_values
    )


def _event_sort_key(item: dict[str, Any]) -> tuple[float, str, str]:
    timestamp = item["recorded_at"].replace("Z", "+00:00")
    return (
        datetime.fromisoformat(timestamp).timestamp(),
        item["event_id"],
        item["event_path"],
    )


def query_tuning_experience(
    root: Path,
    *,
    run_identity: dict[str, Any],
    effective_config_path: Path,
    effective_config_sha256: str,
    observation_fingerprint: str,
    deployment_fingerprint: str,
    max_events: int = DEFAULT_MAX_EVENTS,
    max_event_bytes: int = DEFAULT_MAX_EVENT_BYTES,
    max_diff_entries: int = DEFAULT_MAX_DIFF_ENTRIES,
) -> dict[str, Any]:
    """Return deterministic history classifications without changing state."""
    try:
        validate_run_identity(run_identity)
    except RunIdentityError as exc:
        raise ExperienceQueryError(str(exc)) from exc
    task = run_identity["task"]
    algorithm = run_identity["algorithm"]
    host_id = run_identity["host_id"]
    _reject_symlink_components(root, label="history root")
    if not root.is_dir():
        raise ExperienceQueryError("history root must be an existing directory")
    try:
        require_source_path(effective_config_path, root / task / run_identity["run_id"], "config")
    except ValueError as exc:
        raise ExperienceQueryError(str(exc)) from exc
    try:
        current_config, current_config_source = load_and_validate_effective_config(
            effective_config_path,
            expected_sha256=effective_config_sha256,
            run_identity=run_identity,
        )
    except EffectiveConfigError as exc:
        raise ExperienceQueryError(str(exc)) from exc
    reward_fingerprint = current_config["fingerprints"]["reward"]
    _validate_query_inputs(
        task=task,
        algorithm=algorithm,
        host_id=host_id,
        observation_fingerprint=observation_fingerprint,
        reward_fingerprint=reward_fingerprint,
        deployment_fingerprint=deployment_fingerprint,
        max_events=max_events,
        max_event_bytes=max_event_bytes,
        max_diff_entries=max_diff_entries,
    )
    event_paths = _event_paths(root, task, max_events)
    buckets: dict[str, list[dict[str, Any]]] = {
        "compatible": [],
        "conflicting": [],
        "unknown": [],
    }
    invalid_events: list[dict[str, str | None]] = []
    for path in event_paths:
        encoded: bytes | None = None
        event_hash: str | None = None
        try:
            encoded, _ = _read_stable_event(path, max_event_bytes)
            event_hash = hashlib.sha256(encoded).hexdigest()
            event = json.loads(encoded.decode("utf-8"))
            if not isinstance(event, dict):
                raise ExperienceQueryError("event must be a JSON object")
            try:
                validate_event(event)
            except ExperienceError as exc:
                raise ExperienceQueryError(str(exc)) from exc
            _validate_storage_binding(event, path, task)
            classification, reasons = _classify_event(
                event,
                algorithm=algorithm,
                host_id=host_id,
                observation_fingerprint=observation_fingerprint,
                reward_fingerprint=reward_fingerprint,
                deployment_fingerprint=deployment_fingerprint,
            )
            event_host = (
                event["run_identity"]["host_id"]
                if event["version"] >= 2
                else None
            )
            comparison_eligible = _configuration_comparison_eligible(
                event,
                algorithm=algorithm,
                host_id=host_id,
                observation_fingerprint=observation_fingerprint,
                deployment_fingerprint=deployment_fingerprint,
            )
            parameter_diff = None
            if comparison_eligible:
                historical_config = validate_effective_config_binding(root, event)
                parameter_diff = compare_effective_configs(
                    historical_config,
                    current_config,
                    max_diff_entries=max_diff_entries,
                )
                config_verification = {
                    "status": "verified",
                    "effective_config_fingerprint": historical_config["fingerprints"][
                        "effective_config"
                    ],
                    "reward_fingerprint": historical_config["fingerprints"]["reward"],
                }
                evidence_status = validate_event_evidence(
                    root,
                    event,
                    current_config=historical_config,
                    max_diff_entries=max_diff_entries,
                )
            elif event["version"] < 3:
                config_verification = {"status": "legacy_event_not_verifiable"}
                evidence_status = {
                    "event_evidence_complete": False,
                    "outcome_evidence_complete": False,
                    "reasons": ["legacy_event_contract"],
                }
            else:
                config_verification = {"status": "not_checked_context_mismatch"}
                evidence_status = {
                    "event_evidence_complete": None,
                    "outcome_evidence_complete": None,
                    "reasons": ["event_evidence_not_checked_context_mismatch"],
                }
            context_compatible = classification == "compatible"
            candidate = (
                context_compatible
                and evidence_status["outcome_evidence_complete"] is True
            )
            buckets[classification].append(
                {
                    "event_path": str(path),
                    "event_sha256": event_hash,
                    "version": event["version"],
                    "event_id": event["event_id"],
                    "event_type": event["event_type"],
                    "recorded_at": event["recorded_at"],
                    "task": event["task"],
                    "run_id": event["run_id"],
                    "algorithm": event["algorithm"],
                    "host_id": event_host,
                    "context": event["context"],
                    "parameters": event["parameters"],
                    "evidence_refs": _extract_evidence_refs(event["evidence"]),
                    "effective_config_verification": config_verification,
                    "parameter_diff": parameter_diff,
                    "context_compatible": context_compatible,
                    "event_evidence_complete": evidence_status[
                        "event_evidence_complete"
                    ],
                    "outcome_evidence_complete": evidence_status[
                        "outcome_evidence_complete"
                    ],
                    "tuning_candidate_evidence": candidate,
                    "evidence_completeness_reasons": evidence_status["reasons"],
                    "analysis": {
                        "summary": event["analysis"].get("summary"),
                        "confidence": event["analysis"]["confidence"],
                    },
                    "next_suggestion": event["next_suggestion"],
                    "classification_reasons": reasons,
                }
            )
        except (
            EffectiveConfigError,
            ExperienceError,
            ExperienceQueryError,
            OSError,
            UnicodeDecodeError,
            json.JSONDecodeError,
        ) as exc:
            invalid_events.append(
                {
                    "event_path": str(path),
                    "event_sha256": event_hash,
                    "error": str(exc),
                }
            )

    for events in buckets.values():
        events.sort(key=_event_sort_key)
    invalid_events.sort(key=lambda item: item["event_path"])
    query_context_complete = not any(
        _is_unknown(value)
        for value in (
            algorithm,
            host_id,
            observation_fingerprint,
            reward_fingerprint,
            deployment_fingerprint,
        )
    )
    candidate_events = [
        item
        for item in buckets["compatible"]
        if item["tuning_candidate_evidence"] is True
    ]
    if not query_context_complete:
        support_status = "query_context_incomplete"
    elif invalid_events:
        support_status = "history_invalid"
    elif candidate_events:
        support_status = "candidate_outcome_history_available"
    elif buckets["compatible"]:
        support_status = "context_compatible_outcome_incomplete"
    else:
        support_status = "no_context_compatible_history"
    confidence_counts = {"low": 0, "medium": 0, "high": 0}
    for item in buckets["compatible"]:
        confidence_counts[item["analysis"]["confidence"]] += 1

    return {
        "version": 2,
        "read_only": True,
        "query": {
            "root": str(root),
            "run_id": run_identity["run_id"],
            "task": task,
            "algorithm": algorithm,
            "host_id": host_id,
            "observation_fingerprint": observation_fingerprint,
            "reward_fingerprint": reward_fingerprint,
            "deployment_fingerprint": deployment_fingerprint,
            "max_events": max_events,
            "max_event_bytes": max_event_bytes,
            "max_diff_entries": max_diff_entries,
            "current_effective_config": {
                "path": str(effective_config_path),
                "sha256": current_config_source["sha256"],
                "effective_config_fingerprint": current_config["fingerprints"][
                    "effective_config"
                ],
            },
        },
        "scan": {
            "event_files": len(event_paths),
            "valid_events": sum(len(events) for events in buckets.values()),
            "invalid_events": len(invalid_events),
            "complete": not invalid_events,
        },
        "summary": {
            "compatible": len(buckets["compatible"]),
            "context_compatible": len(buckets["compatible"]),
            "event_evidence_complete": sum(
                item["event_evidence_complete"] is True
                for events in buckets.values()
                for item in events
            ),
            "outcome_evidence_complete": sum(
                item["outcome_evidence_complete"] is True
                for events in buckets.values()
                for item in events
            ),
            "tuning_candidate_evidence": len(candidate_events),
            "conflicting": len(buckets["conflicting"]),
            "unknown": len(buckets["unknown"]),
            "invalid": len(invalid_events),
            "compatible_confidence_counts": confidence_counts,
        },
        "historical_support": {
            "status": support_status,
            "query_context_complete": query_context_complete,
            "context_compatibility_is_not_outcome_evidence": True,
            "only_complete_outcomes_are_candidate_evidence": True,
            "direct_parameter_change_supported": False,
        },
        "candidate_events": candidate_events,
        "compatible_events": buckets["compatible"],
        "conflicting_events": buckets["conflicting"],
        "unknown_events": buckets["unknown"],
        "invalid_events": invalid_events,
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", default=str(DEFAULT_ROOT))
    parser.add_argument("--run-identity", required=True)
    parser.add_argument("--effective-config", required=True)
    parser.add_argument("--effective-config-sha256", required=True)
    parser.add_argument("--observation-fingerprint", required=True)
    parser.add_argument("--deployment-fingerprint", required=True)
    parser.add_argument("--max-events", type=int, default=DEFAULT_MAX_EVENTS)
    parser.add_argument(
        "--max-event-bytes",
        type=int,
        default=DEFAULT_MAX_EVENT_BYTES,
    )
    parser.add_argument(
        "--max-diff-entries",
        type=int,
        default=DEFAULT_MAX_DIFF_ENTRIES,
    )
    args = parser.parse_args()
    try:
        result = query_tuning_experience(
            Path(args.root),
            run_identity=load_run_identity(Path(args.run_identity)),
            effective_config_path=Path(args.effective_config),
            effective_config_sha256=args.effective_config_sha256,
            observation_fingerprint=args.observation_fingerprint,
            deployment_fingerprint=args.deployment_fingerprint,
            max_events=args.max_events,
            max_event_bytes=args.max_event_bytes,
            max_diff_entries=args.max_diff_entries,
        )
    except (EffectiveConfigError, ExperienceQueryError, OSError) as exc:
        parser.error(str(exc))
    print(
        json.dumps(
            result,
            indent=2,
            sort_keys=True,
            ensure_ascii=False,
            allow_nan=False,
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
