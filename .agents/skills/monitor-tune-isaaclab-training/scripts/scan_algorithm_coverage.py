#!/usr/bin/env python3
"""Scan robot_lab sources for algorithms and runners lacking specific profiles."""

from __future__ import annotations

import argparse
import ast
import json
import sys
from pathlib import Path
from typing import Any

from algorithm_profiles import (
    DEFAULT_REGISTRY_PATH,
    ProfileError,
    load_registry,
    match_profile,
)


def _string_assignments(tree: ast.AST, name: str) -> set[str]:
    values: set[str] = set()
    for node in ast.walk(tree):
        target_name: str | None = None
        value_node: ast.AST | None = None
        if isinstance(node, ast.Assign) and len(node.targets) == 1:
            target = node.targets[0]
            target_name = target.id if isinstance(target, ast.Name) else None
            value_node = node.value
        elif isinstance(node, ast.AnnAssign):
            target_name = node.target.id if isinstance(node.target, ast.Name) else None
            value_node = node.value
        if (
            target_name == name
            and isinstance(value_node, ast.Constant)
            and isinstance(value_node.value, str)
        ):
            values.add(value_node.value)
    return values


def _runner_comparisons(tree: ast.AST) -> set[str]:
    runners: set[str] = set()
    for node in ast.walk(tree):
        if not isinstance(node, ast.Compare) or len(node.ops) != 1:
            continue
        left = node.left
        comparator = node.comparators[0]
        if (
            isinstance(node.ops[0], ast.Eq)
            and isinstance(left, ast.Attribute)
            and isinstance(left.value, ast.Name)
            and left.value.id == "agent_cfg"
            and left.attr == "class_name"
            and isinstance(comparator, ast.Constant)
            and isinstance(comparator.value, str)
        ):
            runners.add(comparator.value)
    return runners


def _string_dict_keys(tree: ast.AST, name: str) -> set[str]:
    keys: set[str] = set()
    for node in ast.walk(tree):
        if (
            isinstance(node, ast.Assign)
            and len(node.targets) == 1
            and isinstance(node.targets[0], ast.Name)
            and node.targets[0].id == name
            and isinstance(node.value, ast.Dict)
        ):
            for key in node.value.keys:
                if isinstance(key, ast.Constant) and isinstance(key.value, str):
                    keys.add(key.value)
    return keys


def _parse(path: Path) -> ast.AST:
    try:
        return ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
    except (FileNotFoundError, SyntaxError, UnicodeDecodeError) as exc:
        raise ProfileError(f"cannot parse {path}: {exc}") from exc


def scan(
    registry: dict[str, Any],
    entrypoint: Path,
    evaluation_entrypoint: Path,
    config_root: Path,
    backend: str,
) -> dict[str, Any]:
    """Return runner and config identities without a specific profile."""
    entrypoint_runners = sorted(_runner_comparisons(_parse(entrypoint)))
    evaluation_runners = sorted(
        _string_dict_keys(_parse(evaluation_entrypoint), "RUNNER_CLASSES")
    )
    configured_identities: list[dict[str, str]] = []
    for path in sorted(config_root.rglob("*.py")):
        try:
            class_names = _string_assignments(_parse(path), "class_name")
        except ProfileError:
            continue
        runners = sorted(name for name in class_names if "Runner" in name)
        algorithms = sorted(
            name
            for name in class_names
            if name not in runners
            and (
                name.endswith("PPO")
                or name == "PPO"
                or name == "Distillation"
            )
        )
        for runner in runners:
            for algorithm in algorithms:
                configured_identities.append(
                    {
                        "backend": backend,
                        "algorithm": algorithm,
                        "runner": runner,
                        "source": str(path),
                    }
                )

    specific_runner_classes = {
        runner
        for profile in registry["profiles"]
        if not profile["is_generic"]
        for runner in profile["match"]["runner_classes"]
        if runner != "*"
    }
    uncovered_runners = sorted(set(entrypoint_runners) - specific_runner_classes)
    uncovered_evaluation_runners = sorted(
        set(entrypoint_runners) - set(evaluation_runners)
    )
    unmatched_identities: list[dict[str, str]] = []
    for identity in configured_identities:
        matched = match_profile(
            registry,
            identity["backend"],
            identity["algorithm"],
            identity["runner"],
        )
        if matched["is_generic"]:
            unmatched_identities.append(
                {**identity, "matched_profile": matched["id"]}
            )

    return {
        "schema_version": 1,
        "backend": backend,
        "entrypoint": str(entrypoint),
        "entrypoint_runners": entrypoint_runners,
        "evaluation_entrypoint": str(evaluation_entrypoint),
        "evaluation_entrypoint_runners": evaluation_runners,
        "specific_profile_runners": sorted(specific_runner_classes),
        "uncovered_entrypoint_runners": uncovered_runners,
        "uncovered_evaluation_runners": uncovered_evaluation_runners,
        "configured_identities": configured_identities,
        "unmatched_configured_identities": unmatched_identities,
        "upgrade_candidate_required": bool(
            uncovered_runners
            or uncovered_evaluation_runners
            or unmatched_identities
        ),
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--entrypoint",
        default="scripts/reinforcement_learning/rsl_rl/train.py",
    )
    parser.add_argument(
        "--config-root",
        default="source/robot_lab/robot_lab",
    )
    parser.add_argument(
        "--evaluation-entrypoint",
        default="scripts/reinforcement_learning/rsl_rl/evaluate_policy.py",
    )
    parser.add_argument("--backend", default="rsl_rl")
    parser.add_argument("--registry", default=str(DEFAULT_REGISTRY_PATH))
    parser.add_argument("--fail-on-uncovered", action="store_true")
    args = parser.parse_args()
    try:
        result = scan(
            load_registry(args.registry),
            Path(args.entrypoint),
            Path(args.evaluation_entrypoint),
            Path(args.config_root),
            args.backend,
        )
    except ProfileError as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 2
    print(json.dumps(result, indent=2, sort_keys=True, ensure_ascii=False))
    if args.fail_on_uncovered and result["upgrade_candidate_required"]:
        return 3
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
