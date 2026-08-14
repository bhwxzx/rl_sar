#!/usr/bin/env python3

import argparse
import hashlib
import json
import os
from pathlib import Path
import re
import tempfile


ORIGIN_FILENAME = "origin.json"

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

RUNTIME_FILES = (
    "lib/libserial.a",
    "lib/fdilink_ahrs/ahrs_driver_node",
    "lib/rl_sar/lw_config_profiler",
    "lib/rl_sar/profile_lw_runtime_config.py",
    "share/ament_index/resource_index/packages/fdilink_ahrs",
    "share/ament_index/resource_index/packages/rl_sar",
    "share/ament_index/resource_index/packages/serial",
    "share/fdilink_ahrs/launch/ahrs_driver.launch.py",
    "share/fdilink_ahrs/package.xml",
    "share/fdilink_ahrs/wheeltec_udev.sh",
    "share/rl_sar/launch/rl_real_LW.launch.py",
    "share/serial/package.xml",
)

PRODUCTION_LAUNCH_DIRECTORY = "share/rl_sar/launch"
PRODUCTION_LAUNCH_FILES = {
    "share/rl_sar/launch/rl_real_LW.launch.py",
}

ONNX_RUNTIME_FILES = (
    "lib/rl_sar/onnxruntime/libonnxruntime.so.1",
    "lib/rl_sar/onnxruntime/libonnxruntime_providers_shared.so",
)

ARCHITECTURE_ALIASES = {
    "amd64": "x86_64",
    "x86_64": "x86_64",
    "arm64": "aarch64",
    "aarch64": "aarch64",
}

ELF_MACHINE_BY_ARCHITECTURE = {
    "x86_64": 62,
    "aarch64": 183,
}


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as source:
        for chunk in iter(lambda: source.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def load_approved_onnx_origin(
    catalog_path: Path,
    origin_path: Path,
    version: str,
    architecture: str,
) -> dict[str, str]:
    try:
        catalog = json.loads(catalog_path.read_text(encoding="utf-8"))
        origin = json.loads(origin_path.read_text(encoding="utf-8"))
    except (OSError, ValueError) as error:
        raise RuntimeError(f"cannot read ONNX Runtime provenance: {error}") from error
    if not isinstance(catalog, dict) or catalog.get("schema_version") != 1:
        raise RuntimeError("unsupported inference-runtime catalog schema")
    archives = catalog.get("archives")
    if not isinstance(archives, list):
        raise RuntimeError("inference-runtime catalog has no archive list")
    matches = [
        entry
        for entry in archives
        if isinstance(entry, dict)
        and entry.get("kind") == "onnx"
        and entry.get("version") == version
        and entry.get("os") == "Linux"
        and entry.get("architecture") == architecture
    ]
    if len(matches) != 1:
        raise RuntimeError("ONNX Runtime catalog selection is missing or ambiguous")
    expected = {"schema_version": 1, **matches[0]}
    if origin != expected:
        raise RuntimeError("installed ONNX Runtime provenance differs from the approved catalog")
    digest = matches[0].get("sha256")
    url = matches[0].get("url")
    archive_name = matches[0].get("archive_name")
    if (
        not isinstance(digest, str)
        or re.fullmatch(r"[0-9a-f]{64}", digest) is None
        or not isinstance(url, str)
        or not url.startswith("https://")
        or not isinstance(archive_name, str)
        or Path(archive_name).name != archive_name
    ):
        raise RuntimeError("approved ONNX Runtime catalog entry is malformed")
    return matches[0]


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


def normalize_architecture(architecture: str) -> str:
    normalized = ARCHITECTURE_ALIASES.get(architecture.lower())
    if normalized is None:
        raise RuntimeError(
            f"unsupported ONNX Runtime architecture: {architecture}"
        )
    return normalized


def require_elf_architecture(path: Path, architecture: str) -> None:
    with path.open("rb") as source:
        header = source.read(20)
    if len(header) != 20 or header[:4] != b"\x7fELF" or header[4] != 2:
        raise RuntimeError(f"ONNX Runtime library is not ELF64: {path}")
    if header[5] == 1:
        byte_order = "little"
    elif header[5] == 2:
        byte_order = "big"
    else:
        raise RuntimeError(f"ONNX Runtime library has invalid ELF encoding: {path}")
    machine = int.from_bytes(header[18:20], byte_order)
    expected_machine = ELF_MACHINE_BY_ARCHITECTURE[architecture]
    if machine != expected_machine:
        raise RuntimeError(
            "ONNX Runtime architecture mismatch: "
            f"expected {architecture}, ELF machine is {machine} in {path}"
        )


def render_manifest(
    source_commit: str,
    build_type: str,
    executable_sha256: str,
    onnx_version: str,
    onnx_architecture: str,
    onnx_archive: dict[str, str],
    onnx_origin_hash: str,
    onnx_hashes: list[tuple[str, str]],
    policy_hashes: list[tuple[str, str]],
    runtime_hashes: list[tuple[str, str]],
) -> str:
    lines = [
        "schema_version: 4",
        f'source_commit: "{source_commit}"',
        f'build_type: "{build_type}"',
        "executable:",
        '  name: "rl_real_LW"',
        f'  sha256: "{executable_sha256}"',
        "onnx_runtime:",
        f'  version: "{onnx_version}"',
        f'  architecture: "{onnx_architecture}"',
        "  archive:",
        f'    name: "{onnx_archive["archive_name"]}"',
        f'    url: "{onnx_archive["url"]}"',
        f'    sha256: "{onnx_archive["sha256"]}"',
        "  provenance:",
        f'    path: "lib/rl_sar/onnxruntime/{ORIGIN_FILENAME}"',
        f'    sha256: "{onnx_origin_hash}"',
        "  libraries:",
    ]
    for relative_path, digest in onnx_hashes:
        lines.extend(
            (
                f'    - path: "{relative_path}"',
                f'      sha256: "{digest}"',
            )
        )
    lines.append("files:")
    for relative_path, digest in policy_hashes:
        lines.extend(
            (
                f'  - path: "{relative_path}"',
                f'    sha256: "{digest}"',
            )
        )
    lines.append("runtime_files:")
    for relative_path, digest in runtime_hashes:
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
    onnx_version: str,
    onnx_architecture: str,
    runtime_catalog: Path,
) -> Path:
    if not re.fullmatch(r"[0-9a-f]{40}", source_commit):
        raise RuntimeError("source commit must be a full lowercase Git SHA-1")
    if build_type != "Release":
        raise RuntimeError("LW production deployment requires build type Release")
    if not re.fullmatch(r"[0-9]+\.[0-9]+\.[0-9]+", onnx_version):
        raise RuntimeError("ONNX Runtime version must use major.minor.patch")
    normalized_architecture = normalize_architecture(onnx_architecture)

    prefix = install_prefix.resolve(strict=True)
    executable = prefix / "lib" / "rl_sar" / "rl_real_LW"
    require_regular_file(executable, "installed executable")

    bundle_root = prefix / "share" / "rl_sar" / "deployment" / "LW"
    if bundle_root.is_symlink() or not bundle_root.is_dir():
        raise RuntimeError(
            f"LW deployment bundle is not a real directory: {bundle_root}"
        )

    onnx_directory = prefix / "lib" / "rl_sar" / "onnxruntime"
    if onnx_directory.is_symlink() or not onnx_directory.is_dir():
        raise RuntimeError(
            f"ONNX Runtime directory must be a real directory: {onnx_directory}"
        )
    origin_path = onnx_directory / ORIGIN_FILENAME
    require_regular_file(origin_path, "ONNX Runtime provenance")
    approved_origin = load_approved_onnx_origin(
        runtime_catalog,
        origin_path,
        onnx_version,
        normalized_architecture,
    )
    actual_onnx_files = {
        entry.relative_to(prefix).as_posix() for entry in onnx_directory.iterdir()
    }
    expected_onnx_files = set(ONNX_RUNTIME_FILES) | {
        f"lib/rl_sar/onnxruntime/{ORIGIN_FILENAME}"
    }
    if actual_onnx_files != expected_onnx_files:
        raise RuntimeError(
            "ONNX Runtime directory does not contain the exact approved library set"
        )

    onnx_hashes = []
    for relative_string in ONNX_RUNTIME_FILES:
        relative_path = Path(relative_string)
        require_no_symlink_components(prefix, relative_path)
        library = prefix / relative_path
        require_regular_file(library, "ONNX Runtime library")
        resolved_library = library.resolve(strict=True)
        try:
            resolved_library.relative_to(prefix)
        except ValueError as error:
            raise RuntimeError(
                f"ONNX Runtime library escapes install prefix: {relative_string}"
            ) from error
        require_elf_architecture(library, normalized_architecture)
        onnx_hashes.append((relative_string, sha256_file(library)))

    policy_hashes = []
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
        policy_hashes.append((relative_string, sha256_file(asset)))

    launch_directory = prefix / PRODUCTION_LAUNCH_DIRECTORY
    if launch_directory.is_symlink() or not launch_directory.is_dir():
        raise RuntimeError(
            f"production launch directory must be a real directory: {launch_directory}"
        )
    actual_launch_files = {
        entry.relative_to(prefix).as_posix() for entry in launch_directory.iterdir()
    }
    if actual_launch_files != PRODUCTION_LAUNCH_FILES:
        raise RuntimeError(
            "production launch directory does not contain the exact approved file set"
        )

    runtime_hashes = []
    for relative_string in RUNTIME_FILES:
        relative_path = Path(relative_string)
        require_no_symlink_components(prefix, relative_path)
        asset = prefix / relative_path
        require_regular_file(asset, "runtime dependency")
        resolved_asset = asset.resolve(strict=True)
        try:
            resolved_asset.relative_to(prefix)
        except ValueError as error:
            raise RuntimeError(
                f"runtime dependency escapes install prefix: {relative_string}"
            ) from error
        runtime_hashes.append((relative_string, sha256_file(asset)))

    manifest_path = bundle_root / "manifest.yaml"
    manifest_text = render_manifest(
        source_commit,
        build_type,
        sha256_file(executable),
        onnx_version,
        normalized_architecture,
        approved_origin,
        sha256_file(origin_path),
        onnx_hashes,
        policy_hashes,
        runtime_hashes,
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
    parser.add_argument("--onnx-version", required=True)
    parser.add_argument("--onnx-architecture", required=True)
    parser.add_argument("--runtime-catalog", required=True, type=Path)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    try:
        manifest = generate_manifest(
            args.install_prefix,
            args.source_commit,
            args.build_type,
            args.onnx_version,
            args.onnx_architecture,
            args.runtime_catalog,
        )
    except (OSError, RuntimeError) as error:
        print(f"LW deployment manifest generation failed: {error}", file=os.sys.stderr)
        return 1
    print(f"Generated LW deployment manifest: {manifest}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
