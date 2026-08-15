#!/usr/bin/env python3

import hashlib
import re
import sys
from pathlib import Path


REMOVED_ROBOTS = {
    "a1",
    "b2",
    "b2w",
    "g1",
    "go2",
    "go2w",
    "gr1t1",
    "gr1t2",
    "l4w4",
    "lite3",
    "tita",
}

ROS2_PACKAGE_DIRECTORIES = (
    "src/rl_sar",
    "src/robot_joint_controller",
    "src/robot_msgs",
)


def require(condition: bool, message: str, errors: list[str]) -> None:
    if not condition:
        errors.append(message)


def directory_names(path: Path) -> set[str]:
    return {entry.name for entry in path.iterdir() if entry.is_dir()}


def verify_manifest(zoo: Path, errors: list[str]) -> None:
    manifest = zoo / "LW_DESCRIPTION_MANIFEST.sha256"
    require(manifest.is_file(), f"missing manifest: {manifest}", errors)
    if not manifest.is_file():
        return

    expected_files: set[Path] = set()
    for line_number, line in enumerate(
        manifest.read_text(encoding="utf-8").splitlines(), start=1
    ):
        match = re.fullmatch(r"([0-9a-f]{64})  (LW_description/.+)", line)
        if not match:
            errors.append(f"invalid manifest line {line_number}: {line}")
            continue
        expected_hash, relative_name = match.groups()
        relative_path = Path(relative_name)
        expected_files.add(relative_path)
        asset = zoo / relative_path
        if not asset.is_file():
            errors.append(f"manifest asset is missing: {relative_path}")
            continue
        actual_hash = hashlib.sha256(asset.read_bytes()).hexdigest()
        if actual_hash != expected_hash:
            errors.append(f"manifest hash mismatch: {relative_path}")

    actual_files = {
        path.relative_to(zoo)
        for path in (zoo / "LW_description").rglob("*")
        if path.is_file()
    }
    require(
        actual_files == expected_files,
        "LW_description files differ from the manifest",
        errors,
    )


def main() -> int:
    if len(sys.argv) != 2:
        print(f"usage: {sys.argv[0]} <repository-root>", file=sys.stderr)
        return 2

    root = Path(sys.argv[1]).resolve()
    errors: list[str] = []

    require(directory_names(root / "policy") == {"LW"},
            "policy/ must contain only LW", errors)

    fsm_files = {path.name for path in (root / "src/rl_sar/fsm_robot").glob("fsm_*.hpp")}
    require(fsm_files == {"fsm_LW.hpp", "fsm_all.hpp"},
            f"unexpected FSM files: {sorted(fsm_files)}", errors)

    real_sources = {path.name for path in (root / "src/rl_sar/src").glob("rl_real_*.cpp")}
    require(real_sources == {"rl_real_LW.cpp"},
            f"unexpected real-robot sources: {sorted(real_sources)}", errors)
    real_headers = {path.name for path in (root / "src/rl_sar/include").glob("rl_real_*.hpp")}
    require(real_headers == {"rl_real_LW.hpp"},
            f"unexpected real-robot headers: {sorted(real_headers)}", errors)

    sdk_root = root / "src/rl_sar/library/thirdparty/robot_sdk"
    require(directory_names(sdk_root) == {"lfr"},
            f"unexpected robot SDK directories: {sorted(directory_names(sdk_root))}", errors)

    submodule_paths = {
        match.group(1).strip()
        for match in re.finditer(
            r"^\s*path\s*=\s*(.+)$",
            (root / ".gitmodules").read_text(encoding="utf-8"),
            flags=re.MULTILINE,
        )
    }
    require(
        submodule_paths == {"src/rl_sar/library/thirdparty/joystick"},
        f"unexpected submodules: {sorted(submodule_paths)}",
        errors,
    )

    zoo = root / "src/rl_sar_zoo"
    require(
        {entry.name for entry in zoo.iterdir()}
        == {"LW_description", "LW_DESCRIPTION_MANIFEST.sha256", "README.md"},
        "src/rl_sar_zoo contains unexpected top-level entries",
        errors,
    )
    require(not any(zoo.rglob(".git")), "nested Git metadata found in zoo", errors)
    require((zoo / "LW_description/package.xml").is_file(),
            "lw_description package.xml is missing", errors)
    require(not (zoo / "LW_description/package.ros2.xml").exists(),
            "legacy package.ros2.xml is still present", errors)
    verify_manifest(zoo, errors)

    require(not (root / "scripts/download_robot_descriptions.sh").exists(),
            "legacy zoo downloader still exists", errors)
    require((root / "scripts/validate_lw_description.sh").is_file(),
            "LW description validator is missing", errors)

    for relative_directory in ROS2_PACKAGE_DIRECTORIES:
        package_directory = root / relative_directory
        package_manifest = package_directory / "package.xml"
        require(package_manifest.is_file(),
                f"{relative_directory}/package.xml is missing", errors)
        require(not package_manifest.is_symlink(),
                f"{relative_directory}/package.xml must be a regular file", errors)
        for legacy_manifest in ("package.ros1.xml", "package.ros2.xml"):
            require(not (package_directory / legacy_manifest).exists(),
                    f"legacy manifest remains: {relative_directory}/{legacy_manifest}",
                    errors)

    require(not (root / "src/robot_joint_controller/ros").exists(),
            "ROS 1 controller sources are still present", errors)
    require(not (root / "src/rl_sar/launch/gazebo.launch").exists(),
            "ROS 1 Gazebo launch file is still present", errors)

    ros2_only_files = [
        root / "build.sh",
        root / "src/fdilink_ahrs_ROS2/CMakeLists.txt",
        root / "src/rl_sar/CMakeLists.txt",
        root / "src/rl_sar/include/rl_sim.hpp",
        root / "src/rl_sar/src/rl_sim.cpp",
        root / "src/robot_joint_controller/CMakeLists.txt",
        root / "src/robot_msgs/CMakeLists.txt",
    ]
    legacy_ros_pattern = re.compile(
        r"\b(?:catkin|noetic|USE_ROS1|package\.ros1\.xml|package\.ros2\.xml)\b",
        flags=re.IGNORECASE,
    )
    for path in ros2_only_files:
        if legacy_ros_pattern.search(path.read_text(encoding="utf-8")):
            errors.append(f"ROS 1 compatibility remains in {path.relative_to(root)}")

    implementation_files = [
        root / "src/rl_sar/CMakeLists.txt",
        root / "src/rl_sar/fsm_robot/fsm_all.hpp",
        root / "src/rl_sar/library/core/rl_sdk/rl_sdk.cpp",
        root / "src/rl_sar/library/core/rl_sdk/rl_sdk.hpp",
    ]
    forbidden_pattern = re.compile(
        r"(?<![A-Za-z0-9_])(" + "|".join(sorted(REMOVED_ROBOTS)) + r")(?![A-Za-z0-9_])",
        flags=re.IGNORECASE,
    )
    for path in implementation_files:
        if forbidden_pattern.search(path.read_text(encoding="utf-8")):
            errors.append(f"removed robot identifier remains in {path.relative_to(root)}")

    if errors:
        for error in errors:
            print(f"FAIL: {error}", file=sys.stderr)
        return 1

    print("LW repository scope validation passed")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
