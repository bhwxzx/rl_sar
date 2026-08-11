#!/usr/bin/env python3

import argparse
from pathlib import Path
import re
import sys

from generate_lw_deployment_manifest import POLICY_FILES, sha256_file


ENTRY_PATH = re.compile(r'^  - path: "([^"]+)"$')
ENTRY_HASH = re.compile(r'^    sha256: "([0-9a-f]{64})"$')


def read_policy_records(manifest_path: Path) -> dict[str, str]:
    if manifest_path.is_symlink() or not manifest_path.is_file():
        raise RuntimeError(
            f"deployment manifest must be a regular non-symlink file: {manifest_path}"
        )
    records: dict[str, str] = {}
    in_policy_files = False
    pending_path: str | None = None
    for line in manifest_path.read_text(encoding="utf-8").splitlines():
        if line == "files:":
            in_policy_files = True
            continue
        if line == "runtime_files:":
            break
        if not in_policy_files:
            continue
        path_match = ENTRY_PATH.fullmatch(line)
        if path_match:
            if pending_path is not None:
                raise RuntimeError(f"manifest hash missing for: {pending_path}")
            pending_path = path_match.group(1)
            continue
        hash_match = ENTRY_HASH.fullmatch(line)
        if hash_match and pending_path is not None:
            if pending_path in records:
                raise RuntimeError(f"duplicate manifest policy path: {pending_path}")
            records[pending_path] = hash_match.group(1)
            pending_path = None
    if pending_path is not None:
        raise RuntimeError(f"manifest hash missing for: {pending_path}")
    return records


def verify_policy_parity(policy_root: Path, manifest_path: Path) -> None:
    if policy_root.is_symlink() or not policy_root.is_dir():
        raise RuntimeError(
            f"source policy root must be a real directory: {policy_root}"
        )
    records = read_policy_records(manifest_path)
    if set(records) != set(POLICY_FILES):
        missing = sorted(set(POLICY_FILES) - set(records))
        extra = sorted(set(records) - set(POLICY_FILES))
        raise RuntimeError(
            f"deployment policy file set differs; missing={missing}, extra={extra}"
        )

    for manifest_path_string in POLICY_FILES:
        relative = Path(manifest_path_string).relative_to("policy")
        current = policy_root
        for component in relative.parts:
            current = current / component
            if current.is_symlink():
                raise RuntimeError(
                    f"source policy path contains a symbolic link: {current}"
                )
        source_asset = policy_root / relative
        if source_asset.is_symlink() or not source_asset.is_file():
            raise RuntimeError(
                f"source policy asset must be a regular non-symlink file: {source_asset}"
            )
        actual_hash = sha256_file(source_asset)
        if actual_hash != records[manifest_path_string]:
            raise RuntimeError(
                f"policy SHA-256 mismatch for {manifest_path_string}: "
                f"source={actual_hash}, deployment={records[manifest_path_string]}"
            )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Verify source Sim2Sim policies match an LW deployment manifest."
    )
    parser.add_argument("--policy-root", required=True, type=Path)
    parser.add_argument("--manifest", required=True, type=Path)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    try:
        verify_policy_parity(args.policy_root, args.manifest)
    except (OSError, RuntimeError, ValueError) as error:
        print(f"LW policy parity verification failed: {error}", file=sys.stderr)
        return 1
    print("LW source and deployment policy hashes match")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
