#!/usr/bin/env python3
"""Inventory stable checkpoints and compare bounded Play evidence without selecting autonomously."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import sys
import re
import time
from pathlib import Path
from typing import Any


CHECKPOINT_RE = re.compile(r"^model_(?P<step>\d+)\.pt$")


class SelectionError(ValueError):
    """Raised when checkpoint or evaluation evidence is invalid."""


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _load_object(path: Path, label: str) -> dict[str, Any]:
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except FileNotFoundError as exc:
        raise SelectionError(f"{label} does not exist: {path}") from exc
    except json.JSONDecodeError as exc:
        raise SelectionError(
            f"invalid {label} JSON at line {exc.lineno}: {exc.msg}"
        ) from exc
    if not isinstance(value, dict):
        raise SelectionError(f"{label} must be a JSON object")
    if label == "evaluation result" and value.get("version") in {2, 3}:
        sys.path.insert(0, str(Path(__file__).resolve().parents[4] / "scripts/reinforcement_learning/rsl_rl"))
        from policy_evaluation_evidence import validate_evaluation_bundle
        try:
            validate_evaluation_bundle(path)
        except (ValueError, OSError) as exc:
            raise SelectionError(str(exc)) from exc
    return value


def inventory_checkpoints(
    run_dir: Path,
    *,
    stable_age_seconds: float = 30.0,
    now: float | None = None,
) -> list[dict[str, Any]]:
    if not run_dir.is_absolute() or not run_dir.is_dir():
        raise SelectionError("run directory must be an existing absolute directory")
    observed_at = time.time() if now is None else now
    entries: list[dict[str, Any]] = []
    for path in run_dir.glob("model_*.pt"):
        match = CHECKPOINT_RE.fullmatch(path.name)
        if match is None or path.is_symlink() or not path.is_file():
            continue
        stat = path.stat()
        age = max(0.0, observed_at - stat.st_mtime)
        stable = stat.st_size > 0 and age >= stable_age_seconds
        entries.append(
            {
                "path": str(path.resolve()),
                "step": int(match.group("step")),
                "size_bytes": stat.st_size,
                "mtime_unix": stat.st_mtime,
                "age_seconds": age,
                "stable": stable,
                "sha256": _sha256(path) if stable else None,
            }
        )
    return sorted(entries, key=lambda item: (item["step"], item["path"]))


def _parse_metric_specs(raw_specs: list[str]) -> dict[str, str]:
    metrics: dict[str, str] = {}
    for raw in raw_specs:
        try:
            name, direction = raw.split(":", 1)
        except ValueError as exc:
            raise SelectionError("metrics must use NAME:maximize or NAME:minimize") from exc
        if not name or direction not in {"maximize", "minimize"} or name in metrics:
            raise SelectionError(f"invalid or duplicate metric specification: {raw}")
        metrics[name] = direction
    if not metrics:
        raise SelectionError("at least one metric specification is required")
    return metrics


def _nearest_checkpoint(
    checkpoints: list[dict[str, Any]], progress: int
) -> dict[str, Any]:
    return min(checkpoints, key=lambda item: (abs(item["step"] - progress), -item["step"]))


def shortlist_checkpoints(
    inventory: list[dict[str, Any]],
    *,
    maximum: int,
    summary: dict[str, Any] | None = None,
    training_metrics: dict[str, str] | None = None,
) -> list[dict[str, Any]]:
    stable = [item for item in inventory if item["stable"]]
    if not stable:
        return []
    if maximum < 1:
        raise SelectionError("maximum shortlist size must be positive")
    selected: dict[str, dict[str, Any]] = {stable[-1]["path"]: stable[-1]}
    records = summary.get("records", []) if isinstance(summary, dict) else []
    if not isinstance(records, list):
        raise SelectionError("summary.records must be an array")
    for metric, direction in (training_metrics or {}).items():
        eligible = [
            record
            for record in records
            if isinstance(record, dict)
            and isinstance(record.get("progress"), int)
            and not isinstance(record.get(metric), bool)
            and isinstance(record.get(metric), (int, float))
            and math.isfinite(float(record[metric]))
        ]
        if not eligible:
            continue
        best = (max if direction == "maximize" else min)(
            eligible, key=lambda record: float(record[metric])
        )
        checkpoint = _nearest_checkpoint(stable, best["progress"])
        selected[checkpoint["path"]] = checkpoint
    if len(selected) < maximum and len(stable) > 1:
        slots = min(maximum, len(stable))
        for index in range(slots):
            position = round(index * (len(stable) - 1) / max(slots - 1, 1))
            selected[stable[position]["path"]] = stable[position]
            if len(selected) >= maximum:
                break
    ordered = sorted(selected.values(), key=lambda item: item["step"])
    if len(ordered) > maximum:
        newest = ordered[-1]
        ordered = ordered[: maximum - 1] + [newest]
    return ordered


def _dominates(
    left: dict[str, float], right: dict[str, float], directions: dict[str, str]
) -> bool:
    no_worse = True
    strictly_better = False
    for metric, direction in directions.items():
        left_value = left[metric]
        right_value = right[metric]
        if direction == "maximize":
            no_worse &= left_value >= right_value
            strictly_better |= left_value > right_value
        else:
            no_worse &= left_value <= right_value
            strictly_better |= left_value < right_value
    return no_worse and strictly_better


def compare_evaluations(
    shortlist: list[dict[str, Any]],
    evaluation_results: list[dict[str, Any]],
    metric_directions: dict[str, str],
) -> dict[str, Any]:
    by_path: dict[str, dict[str, Any]] = {}
    incomplete_telemetry: dict[str, dict[str, Any]] = {}
    for result in evaluation_results:
        checkpoint_path = result.get("checkpoint_path")
        metrics = result.get("metrics")
        if result.get("status") != "completed" or not isinstance(checkpoint_path, str) or not isinstance(metrics, dict):
            continue
        resolved_checkpoint = str(Path(checkpoint_path).resolve())
        telemetry_required = (
            result.get("telemetry_required_for_complete_assessment") is True
            or result.get("runner") == "OnPolicyRunnerAmpROA"
        )
        if telemetry_required and result.get("telemetry_status") != "complete":
            missing = result.get("missing_required_signals", [])
            incomplete_telemetry[resolved_checkpoint] = {
                "telemetry_status": result.get("telemetry_status"),
                "missing_required_signals": missing if isinstance(missing, list) else [],
            }
            continue
        values: dict[str, float] = {}
        for metric in metric_directions:
            value = metrics.get(metric)
            if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(float(value)):
                break
            values[metric] = float(value)
        if len(values) == len(metric_directions):
            by_path[resolved_checkpoint] = {
                "checkpoint_path": resolved_checkpoint,
                "metrics": values,
                "result_path": result.get("result_path"),
            }
    eligible = [by_path[item["path"]] for item in shortlist if item["path"] in by_path]
    missing = [item["path"] for item in shortlist if item["path"] not in by_path]
    if missing:
        return {
            "status": "evaluation_required",
            "recommended_checkpoint": None,
            "pareto_front": [],
            "missing_evaluations": missing,
            "incomplete_telemetry": incomplete_telemetry,
            "pending_user_selection": True,
        }
    pareto = [
        candidate
        for candidate in eligible
        if not any(
            other is not candidate
            and _dominates(other["metrics"], candidate["metrics"], metric_directions)
            for other in eligible
        )
    ]
    recommended = pareto[0]["checkpoint_path"] if len(pareto) == 1 else None
    return {
        "status": "single_pareto_recommendation" if recommended else "user_tradeoff_required",
        "recommended_checkpoint": recommended,
        "pareto_front": pareto,
        "missing_evaluations": [],
        "incomplete_telemetry": {},
        "pending_user_selection": True,
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run_dir")
    parser.add_argument("--summary")
    parser.add_argument("--training-metric", action="append", default=[])
    parser.add_argument("--evaluation-result", action="append", default=[])
    parser.add_argument("--evaluation-metric", action="append", default=[])
    parser.add_argument("--max-candidates", type=int, default=5)
    parser.add_argument("--stable-age-seconds", type=float, default=30.0)
    parser.add_argument("--output")
    args = parser.parse_args()
    try:
        training_metrics = _parse_metric_specs(args.training_metric) if args.training_metric else {}
        evaluation_metrics = _parse_metric_specs(args.evaluation_metric) if args.evaluation_metric else {}
        inventory = inventory_checkpoints(
            Path(args.run_dir), stable_age_seconds=args.stable_age_seconds
        )
        shortlist = shortlist_checkpoints(
            inventory,
            maximum=args.max_candidates,
            summary=_load_object(Path(args.summary), "summary") if args.summary else None,
            training_metrics=training_metrics,
        )
        comparison = (
            compare_evaluations(
                shortlist,
                [_load_object(Path(path), "evaluation result") for path in args.evaluation_result],
                evaluation_metrics,
            )
            if evaluation_metrics
            else {
                "status": "evaluation_required",
                "recommended_checkpoint": None,
                "pareto_front": [],
                "missing_evaluations": [item["path"] for item in shortlist],
                "incomplete_telemetry": {},
                "pending_user_selection": True,
            }
        )
    except SelectionError as exc:
        parser.error(str(exc))
    report = {
        "version": 1,
        "advisory_only": True,
        "inventory": inventory,
        "shortlist": shortlist,
        "comparison": comparison,
    }
    encoded = json.dumps(report, indent=2, sort_keys=True, ensure_ascii=False, allow_nan=False)
    if args.output:
        output = Path(args.output)
        if not output.is_absolute():
            parser.error("--output must be absolute")
        output.write_text(encoded + "\n", encoding="utf-8")
    else:
        print(encoded)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
