#!/usr/bin/env python3

import argparse
import hashlib
import json
import os
from pathlib import Path, PurePosixPath
import re
import shutil
import stat
import subprocess
import sys
import tarfile
import tempfile
import uuid
import zipfile


ORIGIN_NAME = ".rl_sar_runtime_origin.json"
ENTRY_FIELDS = {
    "kind",
    "version",
    "os",
    "architecture",
    "archive_name",
    "archive_format",
    "root_directory",
    "url",
    "sha256",
}
ARCHITECTURE_ALIASES = {
    "amd64": "x86_64",
    "x86_64": "x86_64",
    "arm64": "aarch64",
    "aarch64": "aarch64",
}


class RuntimeErrorWithCode(RuntimeError):
    def __init__(self, message: str, code: int = 1):
        super().__init__(message)
        self.code = code


def normalize_architecture(value: str) -> str:
    normalized = ARCHITECTURE_ALIASES.get(value.lower())
    if normalized is None:
        raise RuntimeErrorWithCode(
            f"unsupported inference-runtime architecture: {value}"
        )
    return normalized


def require_string(entry: dict, field: str) -> str:
    value = entry.get(field)
    if not isinstance(value, str) or not value:
        raise RuntimeErrorWithCode(f"catalog field {field} must be non-empty")
    if "\t" in value or "\n" in value or "\r" in value:
        raise RuntimeErrorWithCode(f"catalog field {field} contains whitespace control data")
    return value


def load_catalog(path: Path) -> list[dict[str, str]]:
    try:
        data = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as error:
        raise RuntimeErrorWithCode(f"cannot read inference-runtime catalog: {error}") from error
    if not isinstance(data, dict) or data.get("schema_version") != 1:
        raise RuntimeErrorWithCode("unsupported inference-runtime catalog schema")
    archives = data.get("archives")
    if not isinstance(archives, list) or not archives:
        raise RuntimeErrorWithCode("inference-runtime catalog must contain archives")

    result = []
    identities = set()
    for raw_entry in archives:
        if not isinstance(raw_entry, dict) or set(raw_entry) != ENTRY_FIELDS:
            raise RuntimeErrorWithCode("catalog archive entry has an unexpected field set")
        entry = {field: require_string(raw_entry, field) for field in ENTRY_FIELDS}
        if entry["kind"] not in {"onnx", "libtorch"}:
            raise RuntimeErrorWithCode(f"unsupported runtime kind in catalog: {entry['kind']}")
        if entry["os"] != "Linux":
            raise RuntimeErrorWithCode(f"unsupported runtime OS in catalog: {entry['os']}")
        entry["architecture"] = normalize_architecture(entry["architecture"])
        if entry["archive_format"] not in {"zip", "tgz"}:
            raise RuntimeErrorWithCode("catalog archive format must be zip or tgz")
        if not re.fullmatch(r"[0-9]+\.[0-9]+\.[0-9]+", entry["version"]):
            raise RuntimeErrorWithCode("catalog runtime version must use major.minor.patch")
        if not re.fullmatch(r"[0-9a-f]{64}", entry["sha256"]):
            raise RuntimeErrorWithCode("catalog SHA-256 must be 64 lowercase hex characters")
        if not entry["url"].startswith("https://"):
            raise RuntimeErrorWithCode("catalog URL must use HTTPS")
        for field in ("archive_name", "root_directory"):
            if Path(entry[field]).name != entry[field] or entry[field] in {".", ".."}:
                raise RuntimeErrorWithCode(f"catalog {field} must be one path component")
        identity = (
            entry["kind"],
            entry["version"],
            entry["os"],
            entry["architecture"],
        )
        if identity in identities:
            raise RuntimeErrorWithCode("catalog contains a duplicate platform/version entry")
        identities.add(identity)
        result.append(entry)
    return result


def select_entry(args: argparse.Namespace) -> dict[str, str]:
    architecture = normalize_architecture(args.architecture)
    matches = [
        entry
        for entry in load_catalog(args.catalog)
        if entry["kind"] == args.kind
        and entry["version"] == args.version
        and entry["os"] == args.os_name
        and entry["architecture"] == architecture
    ]
    if len(matches) != 1:
        raise RuntimeErrorWithCode(
            "unsupported or ambiguous inference-runtime platform/version: "
            f"{args.kind} {args.version} {args.os_name} {architecture}"
        )
    return matches[0]


def origin_document(entry: dict[str, str]) -> dict[str, object]:
    return {"schema_version": 1, **entry}


def load_origin(path: Path) -> dict:
    if path.is_symlink() or not path.is_file():
        raise RuntimeErrorWithCode(f"runtime provenance is missing or not regular: {path}")
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as error:
        raise RuntimeErrorWithCode(f"runtime provenance is malformed: {path}") from error
    if not isinstance(value, dict):
        raise RuntimeErrorWithCode(f"runtime provenance is not an object: {path}")
    return value


def run_validator(
    validator: Path,
    kind: str,
    runtime_dir: Path,
    architecture: str,
    os_name: str,
) -> bool:
    environment = os.environ.copy()
    environment["RL_SAR_RUNTIME_VALIDATION_OS"] = os_name
    result = subprocess.run(
        ["bash", str(validator), kind, str(runtime_dir), architecture],
        check=False,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
        env=environment,
    )
    return result.returncode == 0


def destination_for(runtime_root: Path, kind: str) -> Path:
    return runtime_root / ("onnxruntime" if kind == "onnx" else "libtorch")


def check_runtime(args: argparse.Namespace, entry: dict[str, str]) -> None:
    destination = destination_for(args.runtime_root, entry["kind"])
    origin_path = destination / ORIGIN_NAME
    if origin_path.exists() or origin_path.is_symlink():
        if load_origin(origin_path) != origin_document(entry):
            raise RuntimeErrorWithCode(
                "installed runtime provenance differs from the approved catalog; "
                "review the version change explicitly",
                3,
            )
    elif run_validator(
        args.validator,
        entry["kind"],
        destination,
        entry["architecture"],
        entry["os"],
    ):
        raise RuntimeErrorWithCode(
            "installed runtime has no approved archive provenance; refusing automatic replacement",
            3,
        )
    else:
        raise RuntimeErrorWithCode("installed runtime is missing or structurally invalid", 4)

    if not run_validator(
        args.validator,
        entry["kind"],
        destination,
        entry["architecture"],
        entry["os"],
    ):
        raise RuntimeErrorWithCode("approved runtime failed structural or architecture validation", 4)
    print(f"approved {entry['kind']} runtime is ready: {destination}")


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as source:
        for chunk in iter(lambda: source.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def safe_member_path(name: str) -> PurePosixPath:
    path = PurePosixPath(name)
    if path.is_absolute() or ".." in path.parts or not path.parts:
        raise RuntimeErrorWithCode(f"archive contains an unsafe path: {name}")
    return path


def safe_link_target(member_name: str, target: str) -> None:
    target_path = PurePosixPath(target)
    if target_path.is_absolute():
        raise RuntimeErrorWithCode(f"archive link has an absolute target: {target}")
    components = list(PurePosixPath(member_name).parent.parts)
    for component in target_path.parts:
        if component in {"", "."}:
            continue
        if component == "..":
            if not components:
                raise RuntimeErrorWithCode(f"archive link escapes its root: {member_name}")
            components.pop()
        else:
            components.append(component)


def extract_archive(archive: Path, destination: Path, archive_format: str) -> None:
    if archive_format == "zip":
        with zipfile.ZipFile(archive) as source:
            for member in source.infolist():
                safe_member_path(member.filename)
                mode = member.external_attr >> 16
                if stat.S_ISLNK(mode):
                    safe_link_target(
                        member.filename,
                        source.read(member).decode("utf-8"),
                    )
        subprocess.run(
            ["unzip", "-q", str(archive), "-d", str(destination)],
            check=True,
        )
        return
    with tarfile.open(archive, mode="r:gz") as source:
        for member in source.getmembers():
            safe_member_path(member.name)
            if member.issym() or member.islnk():
                safe_link_target(member.name, member.linkname)
            elif not (member.isfile() or member.isdir()):
                raise RuntimeErrorWithCode("archive contains an unsupported entry")
        source.extractall(destination)


def write_origin(candidate: Path, entry: dict[str, str]) -> None:
    origin = candidate / ORIGIN_NAME
    with origin.open("x", encoding="utf-8") as output:
        json.dump(origin_document(entry), output, indent=2, sort_keys=True)
        output.write("\n")


def install_runtime(args: argparse.Namespace, entry: dict[str, str]) -> None:
    if args.archive.is_symlink() or not args.archive.is_file():
        raise RuntimeErrorWithCode(f"downloaded archive is not a regular file: {args.archive}")
    actual_digest = sha256_file(args.archive)
    if actual_digest != entry["sha256"]:
        raise RuntimeErrorWithCode(
            "downloaded archive SHA-256 mismatch: "
            f"expected {entry['sha256']}, found {actual_digest}"
        )

    args.runtime_root.mkdir(parents=True, exist_ok=True)
    stage = Path(tempfile.mkdtemp(prefix=".runtime-candidate.", dir=args.runtime_root))
    destination = destination_for(args.runtime_root, entry["kind"])
    backup = args.runtime_root / f".runtime-backup.{entry['kind']}.{uuid.uuid4().hex}"
    replaced_old = False
    installed = False
    try:
        extract_archive(args.archive, stage, entry["archive_format"])
        candidate = stage / entry["root_directory"]
        top_level = list(stage.iterdir())
        if len(top_level) != 1 or top_level[0] != candidate or not candidate.is_dir():
            raise RuntimeErrorWithCode("archive does not contain the exact approved root directory")
        if not run_validator(
            args.validator,
            entry["kind"],
            candidate,
            entry["architecture"],
            entry["os"],
        ):
            raise RuntimeErrorWithCode("candidate runtime failed structural or architecture validation")
        write_origin(candidate, entry)

        if destination.exists() or destination.is_symlink():
            if destination.is_symlink() or not destination.is_dir():
                raise RuntimeErrorWithCode(
                    f"refusing to replace non-directory runtime path: {destination}"
                )
            origin_path = destination / ORIGIN_NAME
            existing_origin = None
            if origin_path.exists() or origin_path.is_symlink():
                existing_origin = load_origin(origin_path)
            existing_valid = run_validator(
                args.validator,
                entry["kind"],
                destination,
                entry["architecture"],
                entry["os"],
            )
            if existing_origin is not None and existing_origin != origin_document(entry):
                raise RuntimeErrorWithCode(
                    "installed runtime provenance differs from the approved catalog; "
                    "refusing automatic replacement"
                )
            if existing_valid:
                raise RuntimeErrorWithCode(
                    "installed runtime is structurally valid but lacks matching approved provenance; "
                    "refusing automatic replacement"
                )
            os.replace(destination, backup)
            replaced_old = True

        try:
            os.replace(candidate, destination)
            installed = True
        except Exception:
            if replaced_old and not destination.exists():
                os.replace(backup, destination)
                replaced_old = False
            raise
        if replaced_old:
            shutil.rmtree(backup)
            replaced_old = False
        print(
            f"installed approved {entry['kind']} {entry['version']} for "
            f"{entry['os']} {entry['architecture']}"
        )
    finally:
        if replaced_old and not destination.exists() and backup.exists():
            os.replace(backup, destination)
        elif backup.exists():
            shutil.rmtree(backup)
        shutil.rmtree(stage, ignore_errors=True)
        if not installed and destination.exists() and destination.is_dir():
            # Existing destinations are deliberately retained on every failure.
            pass


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Manage pinned inference runtimes.")
    parser.add_argument("action", choices=("select", "check", "install"))
    parser.add_argument("--catalog", type=Path, required=True)
    parser.add_argument("--kind", choices=("onnx", "libtorch"), required=True)
    parser.add_argument("--version", required=True)
    parser.add_argument("--os", dest="os_name", required=True)
    parser.add_argument("--architecture", required=True)
    parser.add_argument("--runtime-root", type=Path)
    parser.add_argument("--validator", type=Path)
    parser.add_argument("--archive", type=Path)
    return parser


def main() -> int:
    args = build_parser().parse_args()
    try:
        entry = select_entry(args)
        if args.action == "select":
            print("\t".join((entry["archive_name"], entry["url"], entry["sha256"])))
            return 0
        if args.runtime_root is None or args.validator is None:
            raise RuntimeErrorWithCode("check/install require --runtime-root and --validator")
        if args.action == "check":
            check_runtime(args, entry)
            return 0
        if args.archive is None:
            raise RuntimeErrorWithCode("install requires --archive")
        install_runtime(args, entry)
        return 0
    except RuntimeErrorWithCode as error:
        print(f"error: {error}", file=sys.stderr)
        return error.code
    except (
        OSError,
        subprocess.CalledProcessError,
        tarfile.TarError,
        UnicodeDecodeError,
        zipfile.BadZipFile,
    ) as error:
        print(f"error: inference-runtime operation failed: {error}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
