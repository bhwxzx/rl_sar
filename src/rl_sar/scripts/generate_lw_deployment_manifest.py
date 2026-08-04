#!/usr/bin/env python3

import argparse
import hashlib
import os
from pathlib import Path
import re
import tempfile


POLICY_FILES = (
    "policy/LW/base.yaml",
    "policy/LW/robot_lab/leg_loco/config.yaml",
    "policy/LW/robot_lab/leg_loco/policy.onnx",
    "policy/LW/robot_lab/leg_to_wheel/config.yaml",
    "policy/LW/robot_lab/leg_to_wheel/leg_to_wheel_transform_60hz.csv",
    "policy/LW/robot_lab/leg_to_wheel/policy.onnx",
    "policy/LW/robot_lab/wheel_loco/config.yaml",
    "policy/LW/robot_lab/wheel_loco/policy.onnx",
    "policy/LW/robot_lab/wheel_to_leg/config.yaml",
    "policy/LW/robot_lab/wheel_to_leg/policy.onnx",
    "policy/LW/robot_lab/wheel_to_leg/wheel_to_leg_transform_60hz.csv",
)


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as source:
        for chunk in iter(lambda: source.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def require_regular_file(path: Path, description: str) -> None:
    if path.is_symlink() or not path.is_file():
        raise RuntimeError(
            f"{description} must be a regular non-symlink file: {path}"
        )


def require_no_symlink_components(root: Path, relative_path: Path) -> None:
    current = root
    for component in relative_path.parts:
        current = current / component
        if current.is_symlink():
            raise RuntimeError(f"bundle path contains a symbolic link: {current}")


def render_manifest(
    source_commit: str,
    build_type: str,
    executable_sha256: str,
    file_hashes: list[tuple[str, str]],
) -> str:
    lines = [
        "schema_version: 1",
        f'source_commit: "{source_commit}"',
        f'build_type: "{build_type}"',
        "executable:",
        '  name: "rl_real_LW"',
        f'  sha256: "{executable_sha256}"',
        "files:",
    ]
    for relative_path, digest in file_hashes:
        lines.extend(
            (
                f'  - path: "{relative_path}"',
                f'    sha256: "{digest}"',
            )
        )
    return "\n".join(lines) + "\n"


def generate_manifest(
    install_prefix: Path,
    source_commit: str,
    build_type: str,
) -> Path:
    if not re.fullmatch(r"[0-9a-f]{40}", source_commit):
        raise RuntimeError("source commit must be a full lowercase Git SHA-1")
    if build_type != "Release":
        raise RuntimeError("LW production deployment requires build type Release")

    prefix = install_prefix.resolve(strict=True)
    executable = prefix / "lib" / "rl_sar" / "rl_real_LW"
    require_regular_file(executable, "installed executable")

    bundle_root = prefix / "share" / "rl_sar" / "deployment" / "LW"
    if bundle_root.is_symlink() or not bundle_root.is_dir():
        raise RuntimeError(
            f"LW deployment bundle is not a real directory: {bundle_root}"
        )

    file_hashes = []
    for relative_string in POLICY_FILES:
        relative_path = Path(relative_string)
        require_no_symlink_components(bundle_root, relative_path)
        asset = bundle_root / relative_path
        require_regular_file(asset, "deployment asset")
        resolved_asset = asset.resolve(strict=True)
        try:
            resolved_asset.relative_to(bundle_root.resolve(strict=True))
        except ValueError as error:
            raise RuntimeError(
                f"deployment asset escapes bundle: {relative_string}"
            ) from error
        file_hashes.append((relative_string, sha256_file(asset)))

    manifest_path = bundle_root / "manifest.yaml"
    manifest_text = render_manifest(
        source_commit,
        build_type,
        sha256_file(executable),
        file_hashes,
    )
    with tempfile.NamedTemporaryFile(
        mode="w",
        encoding="utf-8",
        dir=bundle_root,
        prefix="manifest.yaml.",
        delete=False,
    ) as temporary:
        temporary.write(manifest_text)
        temporary.flush()
        os.fsync(temporary.fileno())
        temporary_path = Path(temporary.name)
    os.chmod(temporary_path, 0o644)
    os.replace(temporary_path, manifest_path)
    return manifest_path


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Generate a verified LW production deployment manifest."
    )
    parser.add_argument("--install-prefix", required=True, type=Path)
    parser.add_argument("--source-commit", required=True)
    parser.add_argument("--build-type", required=True)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    try:
        manifest = generate_manifest(
            args.install_prefix,
            args.source_commit,
            args.build_type,
        )
    except (OSError, RuntimeError) as error:
        print(f"LW deployment manifest generation failed: {error}", file=os.sys.stderr)
        return 1
    print(f"Generated LW deployment manifest: {manifest}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
