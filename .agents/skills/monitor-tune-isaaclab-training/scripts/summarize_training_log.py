#!/usr/bin/env python3
"""Parse profile-driven training console logs into bounded JSON summaries."""

from __future__ import annotations

import argparse
import json
import math
import re
from collections import deque
from pathlib import Path
from statistics import fmean
from typing import Any

from algorithm_profiles import (
    DEFAULT_REGISTRY_PATH,
    ProfileError,
    load_registry,
    normalize_metric_name,
    resolve_profile,
)


ANSI_RE = re.compile(r"\x1b\[[0-?]*[ -/]*[@-~]")
COMPUTATION_RE = re.compile(
    r"Computation:\s+([+-]?(?:\d+(?:\.\d*)?|\.\d+))\s+steps/s "
    r"\(collection:\s*([+-]?(?:\d+(?:\.\d*)?|\.\d+))s,\s*"
    r"learning\s+([+-]?(?:\d+(?:\.\d*)?|\.\d+))s\)"
)
VALUE_RE = re.compile(
    r"^\s*([^:]+):\s*([+-]?(?:(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][+-]?\d+)?|nan|inf|-inf))s?\s*$",
    re.IGNORECASE,
)


class SummaryOutputError(ValueError):
    """Raised when a summary output path is unsafe."""


def write_new_absolute_output(path: Path, encoded: str) -> None:
    """Write a summary without following symlinks or overwriting a file."""
    if not path.is_absolute():
        raise SummaryOutputError("--output must be a new absolute path")
    current = Path(path.anchor)
    for component in path.parts[1:-1]:
        current /= component
        if current.is_symlink():
            raise SummaryOutputError(
                f"--output contains a symlinked path component: {current}"
            )
    if not path.parent.is_dir():
        raise SummaryOutputError("--output parent directory does not exist")
    if path.exists() or path.is_symlink():
        raise SummaryOutputError("--output already exists")
    try:
        with path.open("x", encoding="utf-8") as stream:
            stream.write(encoded)
    except FileExistsError as exc:
        raise SummaryOutputError("--output already exists") from exc


def _progress_match(
    line: str,
    patterns: list[dict[str, Any]],
) -> tuple[str, int, int | None] | None:
    for pattern in patterns:
        match = re.search(pattern["regex"], line)
        if not match:
            continue
        current = int(match.group("current"))
        target_text = match.groupdict().get("target")
        target = int(target_text) if target_text is not None else None
        return pattern["name"], current, target
    return None


class StreamingLogSummary:
    """Incrementally parse training output and expose bounded snapshots."""

    def __init__(
        self,
        path: Path,
        last: int,
        profile: dict[str, Any],
    ) -> None:
        self.path = path
        self.profile = profile
        self.rows: deque[dict[str, Any]] = deque(maxlen=last)
        self.current: dict[str, Any] | None = None
        self.non_finite: deque[dict[str, Any]] = deque(maxlen=1000)
        self.lines_seen = 0

    def feed_line(self, raw_line: str) -> dict[str, bool]:
        """Consume one line and report completed-progress/non-finite events."""
        self.lines_seen += 1
        line = ANSI_RE.sub("", raw_line.rstrip("\n"))
        progress = _progress_match(
            line,
            self.profile["progress_patterns"],
        )
        completed = False
        if progress:
            if self.current is not None:
                self.rows.append(self.current)
                completed = True
            progress_name, progress_value, target = progress
            self.current = {
                "progress_name": progress_name,
                "progress": progress_value,
                "target_progress": target,
            }
            if progress_name == "learning_iteration":
                self.current["iteration"] = progress_value
                self.current["target_iteration"] = target
            return {"completed": completed, "non_finite": False}
        if self.current is None:
            return {"completed": False, "non_finite": False}
        computation_match = COMPUTATION_RE.search(line)
        if computation_match:
            self.current["steps_per_second"] = float(
                computation_match.group(1)
            )
            self.current["collection_time_seconds"] = float(
                computation_match.group(2)
            )
            self.current["learning_time_seconds"] = float(
                computation_match.group(3)
            )
            return {"completed": False, "non_finite": False}
        value_match = VALUE_RE.match(line)
        if not value_match:
            return {"completed": False, "non_finite": False}
        label = value_match.group(1).strip()
        metric = self.profile["metric_aliases"].get(
            label,
            normalize_metric_name(label),
        )
        if not metric:
            return {"completed": False, "non_finite": False}
        value = float(value_match.group(2))
        if math.isfinite(value):
            self.current[metric] = value
            return {"completed": False, "non_finite": False}
        self.current[metric] = None
        self.non_finite.append(
            {
                "progress": self.current["progress"],
                "metric": metric,
            }
        )
        return {"completed": False, "non_finite": True}

    def finish(self) -> None:
        """Finalize the current progress record exactly once."""
        if self.current is not None:
            self.rows.append(self.current)
            self.current = None

    def snapshot(self, include_current: bool = False) -> dict[str, Any]:
        """Return the same schema as parse_log without mutating parser state."""
        retained = list(self.rows)
        if include_current and self.current is not None:
            retained.append(dict(self.current))
            retained = retained[-self.rows.maxlen :]
        aggregate: dict[str, float] = {}
        excluded = {
            "progress",
            "target_progress",
            "iteration",
            "target_iteration",
        }
        metric_names = sorted(
            {
                key
                for row in retained
                for key, value in row.items()
                if key not in excluded
                and isinstance(value, (int, float))
                and not isinstance(value, bool)
            }
        )
        for metric in metric_names:
            values = [
                float(row[metric])
                for row in retained
                if isinstance(row.get(metric), (int, float))
            ]
            if values:
                aggregate[metric] = fmean(values)
        latest = retained[-1] if retained else None
        return {
            "log_path": str(self.path),
            "profile_id": self.profile["id"],
            "window_size": len(retained),
            "first_progress": retained[0]["progress"] if retained else None,
            "last_progress": latest["progress"] if latest else None,
            "first_iteration": retained[0].get("iteration") if retained else None,
            "last_iteration": latest.get("iteration") if latest else None,
            "latest": latest,
            "mean": aggregate,
            "non_finite_metrics": list(self.non_finite),
            "records": retained,
            "iterations": retained,
        }


def parse_log(
    path: Path,
    last: int,
    profile: dict[str, Any],
) -> dict[str, Any]:
    """Parse a log while retaining only the requested number of progress records."""
    parser = StreamingLogSummary(path, last, profile)
    with path.open("r", encoding="utf-8", errors="replace") as stream:
        for raw_line in stream:
            parser.feed_line(raw_line)
    parser.finish()
    return parser.snapshot()


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("log", help="Training stdout/stderr log")
    parser.add_argument("--last", type=int, default=100)
    parser.add_argument("--profile-id", default="generic")
    parser.add_argument("--registry", default=str(DEFAULT_REGISTRY_PATH))
    parser.add_argument("--output", required=True)
    args = parser.parse_args()
    if not 1 <= args.last <= 10000:
        parser.error("--last must be between 1 and 10000")
    try:
        profile = resolve_profile(load_registry(args.registry), args.profile_id)
    except ProfileError as exc:
        parser.error(str(exc))
    result = parse_log(Path(args.log), args.last, profile)
    encoded = json.dumps(
        result,
        indent=2,
        sort_keys=True,
        ensure_ascii=False,
        allow_nan=False,
    )
    try:
        write_new_absolute_output(Path(args.output), encoded + "\n")
    except SummaryOutputError as exc:
        parser.error(str(exc))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
