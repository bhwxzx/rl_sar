#!/usr/bin/env python3
"""Inspect a policy-storage Git worktree without modifying it."""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import subprocess
import sys
from pathlib import Path
from typing import Any


class StorageError(ValueError):
    """Raised when a policy-storage worktree is unsafe or invalid."""


def _git(root: Path, *args: str) -> str:
    result = subprocess.run(
        ["git", "-C", str(root), *args],
        check=False,
        capture_output=True,
        text=True,
    )
    if result.returncode != 0:
        detail = result.stderr.strip() or result.stdout.strip()
        raise StorageError(f"git {' '.join(args)} failed: {detail}")
    return result.stdout.strip()


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def inspect_storage(root: Path, hash_artifacts: bool = False) -> dict[str, Any]:
    """Return a bounded inventory of exact policy artifact directories."""
    if not root.is_absolute():
        raise StorageError("storage root must be absolute")
    if not root.is_dir():
        raise StorageError(f"storage root is not a directory: {root}")
    resolved_root = root.resolve()
    git_root = Path(_git(resolved_root, "rev-parse", "--show-toplevel")).resolve()
    if git_root != resolved_root:
        raise StorageError(
            "storage root must be the exact top level of its Git worktree"
        )

    status_lines = [
        line
        for line in _git(
            resolved_root,
            "status",
            "--porcelain=v1",
            "--untracked-files=all",
        ).splitlines()
        if line
    ]
    entries: list[dict[str, Any]] = []
    for directory, dirnames, filenames in os.walk(
        resolved_root,
        followlinks=False,
    ):
        current = Path(directory)
        if current == resolved_root:
            dirnames[:] = [
                name for name in dirnames if name != ".git"
            ]
        if current.name == ".git" or ".git" in current.parts:
            dirnames[:] = []
            continue
        names = set(filenames)
        if "policy.pt" not in names and "policy.onnx" not in names:
            continue
        relative_dir = current.relative_to(resolved_root).as_posix()
        if relative_dir == ".":
            continue
        entry: dict[str, Any] = {
            "relative_dir": relative_dir,
            "has_policy_pt": "policy.pt" in names,
            "has_policy_onnx": "policy.onnx" in names,
            "has_description": "策略说明.txt" in names,
            "has_manifest": "archive_manifest.json" in names,
            "complete_required_pair": {
                "policy.pt",
                "policy.onnx",
            } <= names,
            "contains_symlink": any(
                (current / name).is_symlink() for name in filenames
            ),
        }
        description_path = current / "策略说明.txt"
        if description_path.is_file() and not description_path.is_symlink():
            description = description_path.read_text(
                encoding="utf-8",
                errors="replace",
            )
            entry["description"] = description[:8000]
            entry["description_truncated"] = len(description) > 8000
        if hash_artifacts:
            hashes: dict[str, str] = {}
            for name in ("policy.pt", "policy.onnx"):
                artifact = current / name
                if artifact.is_file() and not artifact.is_symlink():
                    hashes[name] = _sha256(artifact)
            entry["sha256"] = hashes
        entries.append(entry)

    entries.sort(key=lambda item: item["relative_dir"])
    collections = sorted(
        {
            "/".join(entry["relative_dir"].split("/")[:-1])
            for entry in entries
            if "/" in entry["relative_dir"]
        }
    )
    return {
        "version": 1,
        "storage_root": str(resolved_root),
        "git_branch": _git(resolved_root, "branch", "--show-current"),
        "git_commit": _git(resolved_root, "rev-parse", "HEAD"),
        "git_clean": not status_lines,
        "git_status": status_lines,
        "collections": collections,
        "entry_count": len(entries),
        "entries": entries,
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("storage_root", help="Absolute policy-storage Git root")
    parser.add_argument(
        "--hash-artifacts",
        action="store_true",
        help="Compute SHA-256 for policy.pt and policy.onnx",
    )
    parser.add_argument("--output", help="Optional JSON output path")
    args = parser.parse_args()
    try:
        report = inspect_storage(
            Path(args.storage_root),
            hash_artifacts=args.hash_artifacts,
        )
    except (OSError, StorageError) as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 2
    encoded = json.dumps(
        report,
        indent=2,
        sort_keys=True,
        ensure_ascii=False,
        allow_nan=False,
    )
    if args.output:
        output = Path(args.output)
        if not output.is_absolute():
            parser.error("--output must be absolute")
        if output.exists():
            parser.error("--output already exists")
        if not output.parent.is_dir():
            parser.error("--output parent directory does not exist")
        output.write_text(encoded + "\n", encoding="utf-8")
    else:
        print(encoded)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
