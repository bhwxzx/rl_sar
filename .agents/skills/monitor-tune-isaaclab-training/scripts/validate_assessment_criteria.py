#!/usr/bin/env python3
"""Validate an assessment criteria v2 file without modifying it."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

from assessment_criteria import CriteriaError, SCOPE_FIELDS, inspect_criteria_file


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("criteria", help="absolute path to the criteria JSON")
    parser.add_argument("--task")
    parser.add_argument("--run-id")
    parser.add_argument("--backend")
    parser.add_argument("--profile-id")
    parser.add_argument("--algorithm")
    parser.add_argument("--runner")
    args = parser.parse_args()

    path = Path(args.criteria)
    if not path.is_absolute():
        parser.error("criteria path must be absolute")
    values = {
        "task": args.task,
        "run_id": args.run_id,
        "backend": args.backend,
        "profile_id": args.profile_id,
        "algorithm": args.algorithm,
        "runner": args.runner,
    }
    supplied = [value is not None for value in values.values()]
    if any(supplied) and not all(supplied):
        parser.error("scope options must be supplied together: " + ", ".join(SCOPE_FIELDS))
    try:
        _, report = inspect_criteria_file(
            path,
            expected_scope=values if all(supplied) else None,
        )
    except CriteriaError as exc:
        parser.error(str(exc))
    print(json.dumps(report, indent=2, sort_keys=True, ensure_ascii=False))
    return 0 if report["status"] in {"approved", "draft"} else 1


if __name__ == "__main__":
    raise SystemExit(main())
