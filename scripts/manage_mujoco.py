#!/usr/bin/env python3

import argparse
from contextlib import contextmanager
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


ORIGIN_NAME = ".rl_sar_mujoco_origin.json"
VERSION_NAME = "VERSION_NUMBER"
ENTRY_FIELDS = {
    "version",
    "os",
    "architecture",
    "archive_name",
    "archive_format",
    "root_directory",
    "url",
    "sha256",
    "required_files",
}
ARCHITECTURE_ALIASES = {
    "amd64": "x86_64",
    "x86_64": "x86_64",
    "arm64": "aarch64",
    "aarch64": "aarch64",
}


class MujocoError(RuntimeError):
    def __init__(self, message: str, code: int = 1):
        super().__init__(message)
        self.code = code


def normalize_architecture(value: str) -> str:
    normalized = ARCHITECTURE_ALIASES.get(value.lower())
    if normalized is None:
        raise MujocoError(f"unsupported MuJoCo architecture: {value}")
    return normalized


def require_string(entry: dict, field: str) -> str:
    value = entry.get(field)
    if not isinstance(value, str) or not value:
        raise MujocoError(f"catalog field {field} must be non-empty")
    if any(character in value for character in "\0\t\n\r"):
        raise MujocoError(f"catalog field {field} contains control data")
    return value


def require_relative_path(value: object, field: str) -> str:
    if not isinstance(value, str) or not value:
        raise MujocoError(f"catalog field {field} must be non-empty")
    path = PurePosixPath(value)
    if (
        path.is_absolute()
        or ".." in path.parts
        or not path.parts
        or path.as_posix() != value
    ):
        raise MujocoError(f"catalog field {field} must be a normalized relative path")
    return value


def expected_archive_name(entry: dict[str, object]) -> str:
    version = entry["version"]
    os_name = entry["os"]
    architecture = entry["architecture"]
    if os_name == "Linux":
        return f"mujoco-{version}-linux-{architecture}.tar.gz"
    if os_name == "Windows" and architecture == "x86_64":
        return f"mujoco-{version}-windows-x86_64.zip"
    if os_name == "Darwin":
        return f"mujoco-{version}-macos-universal2.dmg"
    raise MujocoError(
        f"unsupported MuJoCo platform: {os_name} {architecture}"
    )


def expected_required_files(entry: dict[str, object]) -> set[str]:
    version = entry["version"]
    if entry["os"] == "Linux":
        return {
            "include/mujoco/mujoco.h",
            f"lib/libmujoco.so.{version}",
            "bin/simulate",
        }
    if entry["os"] == "Windows":
        return {
            "include/mujoco/mujoco.h",
            "bin/mujoco.dll",
            "lib/mujoco.lib",
            "bin/simulate.exe",
        }
    return {
        "Headers/mujoco.h",
        f"Versions/Current/libmujoco.{version}.dylib",
    }


def validate_platform_entry(entry: dict[str, object]) -> None:
    archive_name = expected_archive_name(entry)
    if entry["archive_name"] != archive_name:
        raise MujocoError(
            f"catalog archive name does not match the approved platform: {archive_name}"
        )
    expected_url = (
        "https://github.com/google-deepmind/mujoco/releases/download/"
        f"{entry['version']}/{archive_name}"
    )
    if entry["url"] != expected_url:
        raise MujocoError("catalog URL is not the exact official MuJoCo release asset")

    if entry["os"] == "Linux":
        expected_format = "tgz"
        expected_root = f"mujoco-{entry['version']}"
    elif entry["os"] == "Windows":
        expected_format = "zip"
        expected_root = "."
    else:
        expected_format = "dmg"
        expected_root = "MuJoCo.app/Contents/Frameworks/MuJoCo.framework"
    if entry["archive_format"] != expected_format:
        raise MujocoError("catalog archive format does not match the platform")
    if entry["root_directory"] != expected_root:
        raise MujocoError("catalog root directory does not match the platform")
    if set(entry["required_files"]) != expected_required_files(entry):
        raise MujocoError("catalog required files do not match the platform contract")


def load_catalog(path: Path) -> list[dict[str, object]]:
    try:
        document = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as error:
        raise MujocoError(f"cannot read MuJoCo catalog: {error}") from error
    if not isinstance(document, dict) or document.get("schema_version") != 1:
        raise MujocoError("unsupported MuJoCo catalog schema")
    archives = document.get("archives")
    if not isinstance(archives, list) or not archives:
        raise MujocoError("MuJoCo catalog must contain archives")

    result = []
    identities = set()
    for raw_entry in archives:
        if not isinstance(raw_entry, dict) or set(raw_entry) != ENTRY_FIELDS:
            raise MujocoError("catalog archive entry has an unexpected field set")
        entry: dict[str, object] = {
            field: require_string(raw_entry, field)
            for field in ENTRY_FIELDS - {"required_files"}
        }
        if not re.fullmatch(r"[0-9]+\.[0-9]+\.[0-9]+", entry["version"]):
            raise MujocoError("catalog version must use major.minor.patch")
        if entry["version"] != "3.2.7":
            raise MujocoError("catalog version must remain pinned to 3.2.7")
        if entry["os"] not in {"Linux", "Windows", "Darwin"}:
            raise MujocoError(f"unsupported MuJoCo OS: {entry['os']}")
        entry["architecture"] = normalize_architecture(entry["architecture"])
        if not re.fullmatch(r"[0-9a-f]{64}", entry["sha256"]):
            raise MujocoError("catalog SHA-256 must be 64 lowercase hex characters")
        if Path(entry["archive_name"]).name != entry["archive_name"]:
            raise MujocoError("catalog archive name must be one path component")

        raw_required = raw_entry["required_files"]
        if not isinstance(raw_required, list) or not raw_required:
            raise MujocoError("catalog required_files must be a non-empty list")
        required_files = [
            require_relative_path(value, "required file") for value in raw_required
        ]
        if len(required_files) != len(set(required_files)):
            raise MujocoError("catalog contains duplicate required files")
        entry["required_files"] = required_files
        validate_platform_entry(entry)

        identity = (entry["version"], entry["os"], entry["architecture"])
        if identity in identities:
            raise MujocoError("catalog contains a duplicate platform/version entry")
        identities.add(identity)
        result.append(entry)
    return result


def select_entry(args: argparse.Namespace) -> dict[str, object]:
    architecture = normalize_architecture(args.architecture)
    matches = [
        entry
        for entry in load_catalog(args.catalog)
        if entry["version"] == args.version
        and entry["os"] == args.os_name
        and entry["architecture"] == architecture
    ]
    if len(matches) != 1:
        raise MujocoError(
            "unsupported or ambiguous MuJoCo platform/version: "
            f"{args.version} {args.os_name} {architecture}"
        )
    return matches[0]


def origin_document(entry: dict[str, object]) -> dict[str, object]:
    return {"schema_version": 1, **entry}


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as source:
        for chunk in iter(lambda: source.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def archive_member_path(name: str) -> PurePosixPath:
    normalized = name.replace("\\", "/")
    canonical = normalized.rstrip("/")
    path = PurePosixPath(normalized)
    if (
        "\0" in name
        or path.is_absolute()
        or re.match(r"^[A-Za-z]:/", normalized)
        or ".." in path.parts
        or not path.parts
        or path.as_posix() == "."
        or path.as_posix() != canonical
    ):
        raise MujocoError(f"archive contains an unsafe path: {name}")
    return path


def resolved_link_path(
    member: PurePosixPath, target: str, *, hard_link: bool
) -> PurePosixPath:
    normalized = target.replace("\\", "/")
    target_path = PurePosixPath(normalized)
    if "\0" in target or target_path.is_absolute():
        raise MujocoError(f"archive link has an unsafe target: {target}")
    components = [] if hard_link else list(member.parent.parts)
    for component in target_path.parts:
        if component in {"", "."}:
            continue
        if component == "..":
            if not components:
                raise MujocoError(f"archive link escapes its root: {member}")
            components.pop()
        else:
            components.append(component)
    if not components:
        raise MujocoError(f"archive link has an empty target: {member}")
    return PurePosixPath(*components)


def filesystem_path(root: Path, relative: PurePosixPath) -> Path:
    return root.joinpath(*relative.parts)


def reject_link_ancestors(
    member_paths: set[PurePosixPath], link_paths: set[PurePosixPath]
) -> None:
    for member in member_paths:
        for parent in member.parents:
            if parent == PurePosixPath("."):
                break
            if parent in link_paths:
                raise MujocoError(
                    f"archive member is nested below a link: {member}"
                )


def extract_tgz(archive: Path, destination: Path) -> None:
    with tarfile.open(archive, mode="r:gz") as source:
        records = []
        member_paths: set[PurePosixPath] = set()
        link_paths: set[PurePosixPath] = set()
        for member in source.getmembers():
            relative = archive_member_path(member.name)
            if relative in member_paths:
                raise MujocoError(f"archive contains a duplicate path: {relative}")
            member_paths.add(relative)
            if member.issym() or member.islnk():
                resolved_link_path(relative, member.linkname, hard_link=member.islnk())
                link_paths.add(relative)
            elif not (member.isfile() or member.isdir()):
                raise MujocoError(f"archive contains an unsupported entry: {relative}")
            records.append((member, relative))
        reject_link_ancestors(member_paths, link_paths)

        for member, relative in records:
            if member.isdir():
                filesystem_path(destination, relative).mkdir(
                    parents=True, exist_ok=True
                )
        for member, relative in records:
            if not member.isfile():
                continue
            target = filesystem_path(destination, relative)
            target.parent.mkdir(parents=True, exist_ok=True)
            archive_file = source.extractfile(member)
            if archive_file is None:
                raise MujocoError(f"cannot read archive member: {relative}")
            with archive_file, target.open("xb") as output:
                shutil.copyfileobj(archive_file, output)
            target.chmod(member.mode & 0o777)
        for member, relative in records:
            if not member.islnk():
                continue
            target = filesystem_path(destination, relative)
            target.parent.mkdir(parents=True, exist_ok=True)
            linked = filesystem_path(
                destination,
                resolved_link_path(relative, member.linkname, hard_link=True),
            )
            if not linked.is_file() or linked.is_symlink():
                raise MujocoError(f"archive hard link target is not a regular file: {relative}")
            os.link(linked, target)
        for member, relative in records:
            if not member.issym():
                continue
            target = filesystem_path(destination, relative)
            target.parent.mkdir(parents=True, exist_ok=True)
            os.symlink(member.linkname, target)


def extract_zip(archive: Path, destination: Path) -> None:
    with zipfile.ZipFile(archive) as source:
        records = []
        member_paths: set[PurePosixPath] = set()
        for member in source.infolist():
            relative = archive_member_path(member.filename)
            if relative in member_paths:
                raise MujocoError(f"archive contains a duplicate path: {relative}")
            member_paths.add(relative)
            mode = member.external_attr >> 16
            file_type = stat.S_IFMT(mode)
            if stat.S_ISLNK(mode):
                raise MujocoError(f"ZIP archive contains a symbolic link: {relative}")
            if member.flag_bits & 0x1:
                raise MujocoError(f"ZIP archive contains an encrypted member: {relative}")
            if file_type not in {0, stat.S_IFREG, stat.S_IFDIR}:
                raise MujocoError(f"ZIP archive contains an unsupported entry: {relative}")
            records.append((member, relative, mode))

        for member, relative, mode in records:
            target = filesystem_path(destination, relative)
            if (
                member.is_dir()
                or member.filename.endswith(("/", "\\"))
                or stat.S_ISDIR(mode)
            ):
                target.mkdir(parents=True, exist_ok=True)
                continue
            target.parent.mkdir(parents=True, exist_ok=True)
            with source.open(member) as archive_file, target.open("xb") as output:
                shutil.copyfileobj(archive_file, output)
            permissions = mode & 0o777
            if permissions:
                target.chmod(permissions)


def path_is_within(path: Path, root: Path) -> bool:
    try:
        path.relative_to(root)
        return True
    except ValueError:
        return False


def validate_tree_links(root: Path) -> None:
    resolved_root = root.resolve(strict=True)
    for directory, directory_names, file_names in os.walk(root, followlinks=False):
        directory_path = Path(directory)
        for name in [*directory_names, *file_names]:
            candidate = directory_path / name
            if not candidate.is_symlink():
                continue
            try:
                resolved = candidate.resolve(strict=True)
            except OSError as error:
                raise MujocoError(f"candidate contains a broken link: {candidate}") from error
            if not path_is_within(resolved, resolved_root):
                raise MujocoError(f"candidate link escapes its root: {candidate}")


def validate_required_files(candidate: Path, entry: dict[str, object]) -> None:
    if candidate.is_symlink() or not candidate.is_dir():
        raise MujocoError("candidate MuJoCo root is not a regular directory")
    validate_tree_links(candidate)
    resolved_root = candidate.resolve(strict=True)
    for relative_name in entry["required_files"]:
        required = filesystem_path(candidate, PurePosixPath(relative_name))
        try:
            resolved = required.resolve(strict=True)
        except OSError as error:
            raise MujocoError(f"candidate required file is missing: {relative_name}") from error
        if not path_is_within(resolved, resolved_root):
            raise MujocoError(f"candidate required file escapes its root: {relative_name}")
        if not stat.S_ISREG(resolved.stat().st_mode):
            raise MujocoError(f"candidate required path is not a regular file: {relative_name}")


def write_metadata(candidate: Path, entry: dict[str, object]) -> None:
    version_path = candidate / VERSION_NAME
    origin_path = candidate / ORIGIN_NAME
    if version_path.exists() or version_path.is_symlink():
        raise MujocoError(f"archive unexpectedly contains {VERSION_NAME}")
    if origin_path.exists() or origin_path.is_symlink():
        raise MujocoError(f"archive unexpectedly contains {ORIGIN_NAME}")
    version_path.write_text(f"{entry['version']}\n", encoding="utf-8")
    with origin_path.open("x", encoding="utf-8") as output:
        json.dump(origin_document(entry), output, indent=2, sort_keys=True)
        output.write("\n")


def load_origin(path: Path) -> dict:
    if path.is_symlink() or not path.is_file():
        raise MujocoError("MuJoCo archive provenance is missing or not regular")
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as error:
        raise MujocoError(f"MuJoCo archive provenance is malformed: {error}") from error
    if not isinstance(value, dict):
        raise MujocoError("MuJoCo archive provenance must be an object")
    return value


def validate_installation(destination: Path, entry: dict[str, object]) -> None:
    validate_required_files(destination, entry)
    version_path = destination / VERSION_NAME
    if version_path.is_symlink() or not version_path.is_file():
        raise MujocoError("MuJoCo version marker is missing or not regular")
    if version_path.read_text(encoding="utf-8").strip() != entry["version"]:
        raise MujocoError("MuJoCo version marker does not match the approved version")
    if load_origin(destination / ORIGIN_NAME) != origin_document(entry):
        raise MujocoError("MuJoCo archive provenance differs from the approved catalog")


def extract_dmg(
    archive: Path, stage: Path, candidate: Path, entry: dict[str, object]
) -> None:
    if sys.platform != "darwin":
        raise MujocoError("DMG extraction requires macOS")
    mount_point = stage / "mount"
    mount_point.mkdir()
    attached = False
    try:
        subprocess.run(
            [
                "hdiutil",
                "attach",
                str(archive),
                "-mountpoint",
                str(mount_point),
                "-nobrowse",
                "-readonly",
                "-quiet",
            ],
            check=True,
        )
        attached = True
        source = filesystem_path(
            mount_point, PurePosixPath(entry["root_directory"])
        )
        if source.is_symlink() or not source.is_dir():
            raise MujocoError("DMG does not contain the exact approved framework path")
        shutil.copytree(source, candidate, symlinks=True)
    finally:
        if attached:
            subprocess.run(
                ["hdiutil", "detach", str(mount_point), "-quiet"],
                check=True,
            )

    dylib = candidate / f"Versions/Current/libmujoco.{entry['version']}.dylib"
    subprocess.run(
        [
            "install_name_tool",
            "-id",
            f"@rpath/libmujoco.{entry['version']}.dylib",
            str(dylib),
        ],
        check=True,
    )
    subprocess.run(["codesign", "--force", "--sign", "-", str(dylib)], check=True)


def prepare_candidate(
    archive: Path, stage: Path, entry: dict[str, object]
) -> Path:
    candidate = stage / "candidate"
    if entry["archive_format"] == "dmg":
        extract_dmg(archive, stage, candidate, entry)
        return candidate

    extracted = stage / "extracted"
    extracted.mkdir()
    if entry["archive_format"] == "tgz":
        extract_tgz(archive, extracted)
    else:
        extract_zip(archive, extracted)

    root_directory = entry["root_directory"]
    if root_directory == ".":
        os.replace(extracted, candidate)
        return candidate
    top_level = list(extracted.iterdir())
    source = filesystem_path(extracted, PurePosixPath(root_directory))
    if len(top_level) != 1 or top_level[0] != source or not source.is_dir():
        raise MujocoError("archive does not contain the exact approved root directory")
    os.replace(source, candidate)
    extracted.rmdir()
    return candidate


@contextmanager
def installation_lock(runtime_root: Path):
    lock_path = runtime_root / ".mujoco-install.lock"
    open_flags = os.O_CREAT | os.O_RDWR
    if hasattr(os, "O_NOFOLLOW"):
        open_flags |= os.O_NOFOLLOW
    try:
        lock_descriptor = os.open(lock_path, open_flags, 0o600)
    except OSError as error:
        raise MujocoError(f"cannot safely open MuJoCo installation lock: {error}") from error
    if not stat.S_ISREG(os.fstat(lock_descriptor).st_mode):
        os.close(lock_descriptor)
        raise MujocoError("MuJoCo installation lock is not a regular file")
    lock_file = os.fdopen(lock_descriptor, "a+b")
    locked = False
    try:
        if os.name == "nt":
            import msvcrt

            if lock_path.stat().st_size == 0:
                lock_file.write(b"\0")
                lock_file.flush()
            lock_file.seek(0)
            try:
                msvcrt.locking(lock_file.fileno(), msvcrt.LK_NBLCK, 1)
            except OSError as error:
                raise MujocoError("another MuJoCo installation is in progress", 5) from error
        else:
            import fcntl

            try:
                fcntl.flock(lock_file.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
            except OSError as error:
                raise MujocoError("another MuJoCo installation is in progress", 5) from error
        locked = True
        yield
    finally:
        if locked:
            if os.name == "nt":
                import msvcrt

                lock_file.seek(0)
                msvcrt.locking(lock_file.fileno(), msvcrt.LK_UNLCK, 1)
            else:
                import fcntl

                fcntl.flock(lock_file.fileno(), fcntl.LOCK_UN)
        lock_file.close()


def destination_is_approved(
    destination: Path, entry: dict[str, object]
) -> bool:
    try:
        validate_installation(destination, entry)
        return True
    except (MujocoError, OSError, UnicodeError):
        return False


def check_runtime(args: argparse.Namespace, entry: dict[str, object]) -> None:
    destination = args.runtime_root / "mujoco"
    if not destination.exists() and not destination.is_symlink():
        raise MujocoError("MuJoCo installation is missing", 4)
    if destination.is_symlink() or not destination.is_dir():
        raise MujocoError(f"refusing unsafe MuJoCo destination: {destination}", 3)
    try:
        validate_installation(destination, entry)
    except (MujocoError, OSError, UnicodeError) as error:
        raise MujocoError(f"installed MuJoCo is not approved: {error}", 4) from error
    print(f"approved MuJoCo {entry['version']} is ready: {destination}")


def verify_archive(archive: Path, entry: dict[str, object]) -> None:
    if archive.is_symlink() or not archive.is_file():
        raise MujocoError(f"downloaded archive is not a regular file: {archive}")
    actual_digest = sha256_file(archive)
    if actual_digest != entry["sha256"]:
        raise MujocoError(
            "downloaded archive SHA-256 mismatch: "
            f"expected {entry['sha256']}, found {actual_digest}"
        )


def install_runtime(args: argparse.Namespace, entry: dict[str, object]) -> None:
    verify_archive(args.archive, entry)
    args.runtime_root.mkdir(parents=True, exist_ok=True)
    if args.runtime_root.is_symlink() or not args.runtime_root.is_dir():
        raise MujocoError("MuJoCo runtime root must be a regular directory")

    stage = Path(tempfile.mkdtemp(prefix=".mujoco-candidate.", dir=args.runtime_root))
    destination = args.runtime_root / "mujoco"
    backup = args.runtime_root / f".mujoco-backup.{uuid.uuid4().hex}"
    previous_moved = False
    installed = False
    try:
        candidate = prepare_candidate(args.archive, stage, entry)
        validate_required_files(candidate, entry)
        write_metadata(candidate, entry)
        validate_installation(candidate, entry)

        with installation_lock(args.runtime_root):
            if destination_is_approved(destination, entry):
                print(f"approved MuJoCo {entry['version']} is already installed")
                return
            if destination.exists() or destination.is_symlink():
                if destination.is_symlink() or not destination.is_dir():
                    raise MujocoError(
                        f"refusing to replace unsafe MuJoCo destination: {destination}",
                        3,
                    )
                os.replace(destination, backup)
                previous_moved = True
            try:
                os.replace(candidate, destination)
                installed = True
                validate_installation(destination, entry)
            except BaseException:
                if destination.exists() or destination.is_symlink():
                    failed = stage / "failed-candidate"
                    os.replace(destination, failed)
                if previous_moved and backup.exists():
                    os.replace(backup, destination)
                    previous_moved = False
                installed = False
                raise
            if previous_moved:
                try:
                    shutil.rmtree(backup)
                except OSError as error:
                    print(
                        f"warning: installed MuJoCo but could not remove backup {backup}: {error}",
                        file=sys.stderr,
                    )
                previous_moved = False
        print(
            f"installed approved MuJoCo {entry['version']} for "
            f"{entry['os']} {entry['architecture']}"
        )
    finally:
        if previous_moved and not destination.exists() and backup.exists():
            os.replace(backup, destination)
        shutil.rmtree(stage, ignore_errors=True)
        if not installed and backup.exists() and not destination.exists():
            os.replace(backup, destination)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Manage pinned MuJoCo archives.")
    parser.add_argument("action", choices=("select", "check", "install"))
    parser.add_argument("--catalog", type=Path, required=True)
    parser.add_argument("--version", required=True)
    parser.add_argument("--os", dest="os_name", required=True)
    parser.add_argument("--architecture", required=True)
    parser.add_argument("--runtime-root", type=Path)
    parser.add_argument("--archive", type=Path)
    return parser


def main() -> int:
    args = build_parser().parse_args()
    try:
        entry = select_entry(args)
        if args.action == "select":
            print("\t".join((entry["archive_name"], entry["url"], entry["sha256"])))
            return 0
        if args.runtime_root is None:
            raise MujocoError("check/install require --runtime-root")
        if args.action == "check":
            check_runtime(args, entry)
            return 0
        if args.archive is None:
            raise MujocoError("install requires --archive")
        install_runtime(args, entry)
        return 0
    except MujocoError as error:
        print(f"error: {error}", file=sys.stderr)
        return error.code
    except (
        OSError,
        subprocess.CalledProcessError,
        tarfile.TarError,
        UnicodeError,
        zipfile.BadZipFile,
    ) as error:
        print(f"error: MuJoCo operation failed: {error}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
