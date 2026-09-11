#!/usr/bin/env python3
"""Collect profile-driven, non-mutating health evidence for a training process."""

from __future__ import annotations

import argparse
import json
import math
import os
import re
import subprocess
import time
from pathlib import Path
from typing import Any

from algorithm_profiles import (
    DEFAULT_REGISTRY_PATH,
    ProfileError,
    load_registry,
    resolve_profile,
)


ANSI_RE = re.compile(r"\x1b\[[0-?]*[ -/]*[@-~]")


class HealthEvidenceError(ValueError):
    """Raised when a previous health snapshot cannot be loaded."""


def write_new_absolute_output(path: Path, encoded: str) -> None:
    """Write a health snapshot without following symlinks or overwriting a file."""
    if not path.is_absolute():
        raise HealthEvidenceError("--output must be a new absolute path")
    current = Path(path.anchor)
    for component in path.parts[1:-1]:
        current /= component
        if current.is_symlink():
            raise HealthEvidenceError(
                f"--output contains a symlinked path component: {current}"
            )
    if not path.parent.is_dir():
        raise HealthEvidenceError("--output parent directory does not exist")
    if path.exists() or path.is_symlink():
        raise HealthEvidenceError("--output already exists")
    try:
        with path.open("x", encoding="utf-8") as stream:
            stream.write(encoded)
    except FileExistsError as exc:
        raise HealthEvidenceError("--output already exists") from exc


def _read_tail(path: Path, byte_count: int = 1024 * 1024) -> str:
    with path.open("rb") as stream:
        stream.seek(0, os.SEEK_END)
        size = stream.tell()
        stream.seek(max(0, size - byte_count))
        return stream.read().decode("utf-8", errors="replace")


def _latest_log_progress(
    path: Path,
    patterns: list[dict[str, Any]],
) -> dict[str, Any] | None:
    text = ANSI_RE.sub("", _read_tail(path))
    latest: tuple[int, dict[str, Any]] | None = None
    for pattern in patterns:
        for match in re.finditer(pattern["regex"], text):
            target_text = match.groupdict().get("target")
            candidate = {
                "name": pattern["name"],
                "current": int(match.group("current")),
                "target": int(target_text) if target_text is not None else None,
                "completion_offset": pattern["completion_offset"],
            }
            if latest is None or match.start() > latest[0]:
                latest = (match.start(), candidate)
    return latest[1] if latest else None


def tensorboard_progress(path: Path | None) -> dict[str, Any]:
    if path is None:
        return {
            "path": None,
            "available": None,
            "step": None,
            "wall_time_unix": None,
            "tag": None,
            "error": None,
        }
    try:
        from tensorboard.backend.event_processing.event_accumulator import (
            EventAccumulator,
        )

        accumulator = EventAccumulator(str(path), size_guidance={"scalars": 1})
        accumulator.Reload()
        latest = None
        for tag in accumulator.Tags().get("scalars", []):
            events = accumulator.Scalars(tag)
            if not events:
                continue
            event = events[-1]
            candidate = (float(event.wall_time), int(event.step), tag)
            if latest is None or candidate > latest:
                latest = candidate
        if latest is None:
            return {
                "path": str(path),
                "available": False,
                "step": None,
                "wall_time_unix": None,
                "tag": None,
                "error": "no scalar events",
            }
        wall_time, step, tag = latest
        return {
            "path": str(path),
            "available": True,
            "step": step,
            "wall_time_unix": wall_time,
            "tag": tag,
            "error": None,
        }
    except Exception as exc:
        # TensorBoard readers can raise backend-specific corruption errors.
        # This source is auxiliary evidence and must not break reconciliation.
        return {
            "path": str(path),
            "available": False,
            "step": None,
            "wall_time_unix": None,
            "tag": None,
            "error": str(exc),
        }


def _process_info(pid: int | None, expected_pattern: str | None) -> dict[str, Any]:
    if pid is None:
        return {"pid": None, "alive": None, "matches_expected": None, "cmdline": None}
    proc_dir = Path("/proc") / str(pid)
    alive = proc_dir.exists()
    cmdline: str | None = None
    matches: bool | None = None
    if alive:
        try:
            raw = (proc_dir / "cmdline").read_bytes()
            cmdline = " ".join(
                part.decode("utf-8", errors="replace")
                for part in raw.split(b"\0")
                if part
            )
        except OSError:
            cmdline = None
        matches = (
            expected_pattern in cmdline
            if expected_pattern and cmdline is not None
            else None
        )
    return {
        "pid": pid,
        "alive": alive,
        "matches_expected": matches,
        "cmdline": cmdline,
    }


def _gpu_info(gpu_index: int | None) -> dict[str, Any]:
    if gpu_index is None:
        return {
            "index": None,
            "available": None,
            "utilization_percent": None,
            "memory_used_mb": None,
            "error": None,
        }
    command = [
        "nvidia-smi",
        f"--id={gpu_index}",
        "--query-gpu=utilization.gpu,memory.used",
        "--format=csv,noheader,nounits",
    ]
    try:
        result = subprocess.run(
            command,
            check=True,
            capture_output=True,
            text=True,
            timeout=5,
        )
        first_line = result.stdout.strip().splitlines()[0]
        utilization, memory_used = [
            part.strip() for part in first_line.split(",", maxsplit=1)
        ]
        return {
            "index": gpu_index,
            "available": True,
            "utilization_percent": float(utilization),
            "memory_used_mb": float(memory_used),
            "error": None,
        }
    except (FileNotFoundError, subprocess.SubprocessError, ValueError, IndexError) as exc:
        return {
            "index": gpu_index,
            "available": False,
            "utilization_percent": None,
            "memory_used_mb": None,
            "error": str(exc),
        }


def _finite_number(value: Any) -> bool:
    return (
        not isinstance(value, bool)
        and isinstance(value, (int, float))
        and math.isfinite(float(value))
    )


def load_previous_health(path: Path) -> dict[str, Any]:
    """Load a health snapshot without accepting non-object JSON."""
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except FileNotFoundError as exc:
        raise HealthEvidenceError(
            f"previous health does not exist: {path}"
        ) from exc
    except json.JSONDecodeError as exc:
        raise HealthEvidenceError(
            f"invalid previous health JSON at line {exc.lineno}: {exc.msg}"
        ) from exc
    if not isinstance(value, dict):
        raise HealthEvidenceError("previous health must be a JSON object")
    return value


def _same_path(left: Any, right: Path) -> bool:
    if not isinstance(left, str) or not left:
        return False
    return Path(left).expanduser().resolve() == right.expanduser().resolve()


def _compare_progress(
    current: int | None,
    previous: int | None,
) -> dict[str, Any]:
    comparable = current is not None and previous is not None
    return {
        "current": current,
        "previous": previous,
        "comparable": comparable,
        "advanced": comparable and current > previous,
        "unchanged": comparable and current == previous,
        "regressed": comparable and current < previous,
    }


def _build_comparison(
    *,
    profile: dict[str, Any],
    log_path: Path,
    pid: int | None,
    timestamp: float,
    log_progress: dict[str, Any] | None,
    tensorboard: dict[str, Any],
    previous_health: dict[str, Any] | None,
    previous_log_progress: int | None,
    previous_tensorboard_step: int | None,
    previous_observed_at: float | None,
) -> dict[str, Any]:
    errors: list[str] = []
    source = "none"
    previous_log_name: str | None = None

    if previous_health is not None:
        source = "previous_health"
        if previous_health.get("profile_id") != profile["id"]:
            errors.append("previous_profile_mismatch")
        previous_log = previous_health.get("log")
        if not isinstance(previous_log, dict) or not _same_path(
            previous_log.get("path"), log_path
        ):
            errors.append("previous_log_path_mismatch")
        previous_process = previous_health.get("process")
        if not isinstance(previous_process, dict) or previous_process.get("pid") != pid:
            errors.append("previous_pid_mismatch")
        previous_timestamp = previous_health.get("timestamp_unix")
        if not _finite_number(previous_timestamp) or float(previous_timestamp) >= timestamp:
            errors.append("previous_timestamp_not_earlier")
        else:
            previous_observed_at = float(previous_timestamp)

        previous_progress = previous_health.get("progress")
        if isinstance(previous_progress, dict):
            previous_log_value = previous_progress.get("log")
            if isinstance(previous_log_value, dict):
                previous_log_name = previous_log_value.get("name")
                candidate = previous_log_value.get("current")
                if isinstance(candidate, int) and not isinstance(candidate, bool):
                    previous_log_progress = candidate
            previous_tensorboard = previous_progress.get("tensorboard")
            if isinstance(previous_tensorboard, dict):
                candidate = previous_tensorboard.get("step")
                if isinstance(candidate, int) and not isinstance(candidate, bool):
                    previous_tensorboard_step = candidate
    elif any(
        value is not None
        for value in (
            previous_log_progress,
            previous_tensorboard_step,
            previous_observed_at,
        )
    ):
        source = "explicit"
        if previous_observed_at is not None and (
            not _finite_number(previous_observed_at)
            or float(previous_observed_at) >= timestamp
        ):
            errors.append("previous_timestamp_not_earlier")

    log_current = log_progress["current"] if log_progress else None
    log_comparison = _compare_progress(log_current, previous_log_progress)
    log_comparison["name"] = log_progress.get("name") if log_progress else None
    log_comparison["previous_name"] = previous_log_name
    if (
        previous_log_name is not None
        and log_progress is not None
        and previous_log_name != log_progress.get("name")
    ):
        errors.append("previous_log_progress_kind_mismatch")
        log_comparison["comparable"] = False
        log_comparison["advanced"] = False
        log_comparison["unchanged"] = False
        log_comparison["regressed"] = False

    tensorboard_step = tensorboard.get("step")
    tensorboard_comparison = _compare_progress(
        tensorboard_step if isinstance(tensorboard_step, int) else None,
        previous_tensorboard_step,
    )
    source_comparisons = [
        item
        for item in (log_comparison, tensorboard_comparison)
        if item["comparable"]
    ]
    advanced = any(item["advanced"] for item in source_comparisons)
    regressed = any(item["regressed"] for item in source_comparisons)
    unchanged = bool(source_comparisons) and all(
        item["unchanged"] for item in source_comparisons
    )
    if advanced and regressed:
        errors.append("progress_sources_disagree")
    baseline_available = (
        previous_log_progress is not None
        or previous_tensorboard_step is not None
    )
    if source != "none" and baseline_available and not source_comparisons:
        errors.append("previous_progress_not_comparable")
    comparable = bool(source_comparisons) and not errors
    elapsed_seconds = (
        timestamp - float(previous_observed_at)
        if _finite_number(previous_observed_at)
        and float(previous_observed_at) < timestamp
        else None
    )
    return {
        "source": source,
        "baseline_available": baseline_available,
        "identity_valid": not errors if source == "previous_health" else None,
        "comparable": comparable,
        "advanced": comparable and advanced,
        "unchanged": comparable and unchanged,
        "regressed": regressed,
        "elapsed_seconds": elapsed_seconds,
        "log": log_comparison,
        "tensorboard": tensorboard_comparison,
        "errors": errors,
    }


def collect_health(
    log_path: Path,
    profile: dict[str, Any],
    stale_after_seconds: int,
    pid: int | None,
    expected_process_pattern: str | None,
    gpu_index: int | None,
    low_gpu_utilization_percent: float,
    tensorboard_path: Path | None = None,
    previous_log_progress: int | None = None,
    previous_tensorboard_step: int | None = None,
    previous_observed_at: float | None = None,
    now: float | None = None,
    previous_health: dict[str, Any] | None = None,
) -> dict[str, Any]:
    """Collect evidence and classify state without changing the process."""
    timestamp = time.time() if now is None else now
    log_exists = log_path.is_file()
    log_mtime: float | None = None
    log_age: float | None = None
    log_stale: bool | None = None
    log_progress: dict[str, Any] | None = None
    if log_exists:
        log_mtime = log_path.stat().st_mtime
        log_age = max(0.0, timestamp - log_mtime)
        log_stale = log_age >= stale_after_seconds
        log_progress = _latest_log_progress(
            log_path,
            profile["progress_patterns"],
        )

    tensorboard = tensorboard_progress(tensorboard_path)
    process = _process_info(pid, expected_process_pattern)
    gpu = _gpu_info(gpu_index)

    comparison = _build_comparison(
        profile=profile,
        log_path=log_path,
        pid=pid,
        timestamp=timestamp,
        log_progress=log_progress,
        tensorboard=tensorboard,
        previous_health=previous_health,
        previous_log_progress=previous_log_progress,
        previous_tensorboard_step=previous_tensorboard_step,
        previous_observed_at=previous_observed_at,
    )
    log_advanced = comparison["log"]["advanced"] and comparison["comparable"]
    tensorboard_advanced = (
        comparison["tensorboard"]["advanced"] and comparison["comparable"]
    )
    tensorboard_age = (
        max(0.0, timestamp - tensorboard["wall_time_unix"])
        if tensorboard["wall_time_unix"] is not None
        else None
    )
    tensorboard_recent = (
        tensorboard_age is not None and tensorboard_age < stale_after_seconds
    )
    progress_advanced = comparison["advanced"]
    completed = (
        log_progress is not None
        and log_progress["target"] is not None
        and log_progress["current"] + log_progress["completion_offset"]
        >= log_progress["target"]
    )
    progress_stale = (
        comparison["unchanged"]
        and comparison["elapsed_seconds"] is not None
        and comparison["elapsed_seconds"] >= stale_after_seconds
    )
    gpu_utilization = gpu.get("utilization_percent")
    gpu_utilization_available = (
        gpu.get("available") is True and _finite_number(gpu_utilization)
    )
    gpu_utilization_low = (
        gpu_utilization_available
        and float(gpu_utilization) <= low_gpu_utilization_percent
    )
    confirmed_stall = (
        progress_stale
        and process["alive"] is True
        and process["matches_expected"] is not False
        and not completed
    )
    activity_without_progress = (
        confirmed_stall
        and gpu_utilization_available
        and float(gpu_utilization) > low_gpu_utilization_percent
    )

    evidence: list[str] = []
    if comparison["errors"]:
        state = "unknown"
        evidence.extend(comparison["errors"])
    elif process["matches_expected"] is False:
        state = "unknown"
        evidence.append("pid_does_not_match_expected_process")
    elif comparison["regressed"]:
        state = "unknown"
        evidence.append("training_progress_regressed")
    elif completed:
        state = "completed"
        evidence.append("profile_progress_reached_target")
    elif process["alive"] is False:
        state = "stopped"
        evidence.append("process_not_alive")
    elif progress_advanced:
        state = "healthy"
        evidence.append("training_progress_advanced")
    elif confirmed_stall:
        state = "stalled"
        evidence.extend(["training_progress_stale", "process_still_alive"])
        if activity_without_progress:
            evidence.append("activity_without_progress")
        elif gpu_utilization_low:
            evidence.append("gpu_utilization_low")
        else:
            evidence.append("gpu_activity_unavailable")
    elif (
        not comparison["baseline_available"]
        and (log_progress is not None or tensorboard["step"] is not None)
    ):
        state = "observing"
        evidence.append("initial_progress_baseline_recorded")
        if tensorboard_recent:
            evidence.append("tensorboard_recency_is_auxiliary_only")
    elif not log_exists and tensorboard["available"] is not True:
        state = "unknown"
        evidence.append("progress_sources_missing")
    else:
        state = "suspect"
        evidence.append("training_progress_not_confirmed")
        if log_stale:
            evidence.append("log_stale")
        if gpu["available"] is True and gpu["utilization_percent"] > low_gpu_utilization_percent:
            evidence.append("gpu_activity_is_auxiliary_only")

    latest_iteration = (
        log_progress["current"]
        if log_progress and log_progress["name"] == "learning_iteration"
        else None
    )
    target_iteration = (
        log_progress["target"]
        if log_progress and log_progress["name"] == "learning_iteration"
        else None
    )
    return {
        "version": 1,
        "timestamp_unix": timestamp,
        "profile_id": profile["id"],
        "state": state,
        "activity_without_progress": activity_without_progress,
        "auto_recovery_candidate": state == "stalled" and gpu_utilization_low,
        "evidence": evidence,
        "progress": {
            "log": log_progress,
            "log_advanced": log_advanced,
            "tensorboard": tensorboard,
            "tensorboard_age_seconds": tensorboard_age,
            "tensorboard_advanced": tensorboard_advanced,
            "previous_observed_at": (
                timestamp - comparison["elapsed_seconds"]
                if comparison["elapsed_seconds"] is not None
                else None
            ),
            "stale": progress_stale,
        },
        "comparison": comparison,
        "baseline_for_next_check": {
            "profile_id": profile["id"],
            "timestamp_unix": timestamp,
            "log_path": str(log_path),
            "pid": pid,
            "log_progress": log_progress,
            "tensorboard_step": tensorboard.get("step"),
        },
        "log": {
            "path": str(log_path),
            "exists": log_exists,
            "mtime_unix": log_mtime,
            "age_seconds": log_age,
            "stale": log_stale,
            "latest_iteration": latest_iteration,
            "target_iteration": target_iteration,
        },
        "process": process,
        "gpu": gpu,
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--log", required=True)
    parser.add_argument("--profile-id", default="generic")
    parser.add_argument("--registry", default=str(DEFAULT_REGISTRY_PATH))
    parser.add_argument("--tensorboard")
    parser.add_argument("--stale-after-seconds", required=True, type=int)
    parser.add_argument("--pid", type=int)
    parser.add_argument("--expected-process-pattern")
    parser.add_argument("--gpu-index", type=int)
    parser.add_argument("--low-gpu-utilization-percent", type=float, default=5.0)
    parser.add_argument("--previous-health")
    parser.add_argument("--previous-log-progress", type=int)
    parser.add_argument("--previous-tensorboard-step", type=int)
    parser.add_argument("--previous-observed-at", type=float)
    parser.add_argument("--now", type=float)
    parser.add_argument("--output")
    args = parser.parse_args()
    if args.previous_health and any(
        value is not None
        for value in (
            args.previous_log_progress,
            args.previous_tensorboard_step,
            args.previous_observed_at,
        )
    ):
        parser.error(
            "--previous-health cannot be combined with explicit previous progress options"
        )
    previous_health = None
    if args.previous_health:
        previous_health_path = Path(args.previous_health)
        if not previous_health_path.is_absolute():
            parser.error("--previous-health must be absolute")
        try:
            previous_health = load_previous_health(previous_health_path)
        except HealthEvidenceError as exc:
            parser.error(str(exc))
    try:
        profile = resolve_profile(
            load_registry(args.registry),
            args.profile_id,
        )
    except ProfileError as exc:
        parser.error(str(exc))
    result = collect_health(
        log_path=Path(args.log),
        profile=profile,
        stale_after_seconds=args.stale_after_seconds,
        pid=args.pid,
        expected_process_pattern=args.expected_process_pattern,
        gpu_index=args.gpu_index,
        low_gpu_utilization_percent=args.low_gpu_utilization_percent,
        tensorboard_path=Path(args.tensorboard) if args.tensorboard else None,
        previous_log_progress=args.previous_log_progress,
        previous_tensorboard_step=args.previous_tensorboard_step,
        previous_observed_at=args.previous_observed_at,
        now=args.now,
        previous_health=previous_health,
    )
    encoded = json.dumps(result, indent=2, sort_keys=True, ensure_ascii=False) + "\n"
    if args.output:
        try:
            write_new_absolute_output(Path(args.output), encoded)
        except HealthEvidenceError as exc:
            parser.error(str(exc))
    else:
        print(encoded, end="")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
