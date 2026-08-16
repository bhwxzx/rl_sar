#!/usr/bin/env python3

from pathlib import Path
import os
import shutil
import subprocess
import sys
import tempfile
import unittest


BUILD_SCRIPT = Path(sys.argv.pop(1)).resolve()
REMOVED_LW_SCRIPT = Path(sys.argv.pop(1)).resolve()
README = Path(sys.argv.pop(1)).resolve()
DEPLOYMENT_GUIDE = Path(sys.argv.pop(1)).resolve()
QUICK_START_GUIDE = Path(sys.argv.pop(1)).resolve()
DEPENDENCY_INSTALLER = Path(sys.argv.pop(1)).resolve()
RUNTIME_DOWNLOADER = Path(sys.argv.pop(1)).resolve()
REMOVED_JETSON_INSTALLER = Path(sys.argv.pop(1)).resolve()
CMAKE_FILE = Path(sys.argv.pop(1)).resolve()
DEPLOYMENT_BUILDER = Path(sys.argv.pop(1)).resolve()
STRICT_BUILD_VALIDATOR = Path(sys.argv.pop(1)).resolve()


class BuildWorkflowTests(unittest.TestCase):
    CLEAN_PACKAGES = ("serial", "fdilink_ahrs", "rl_sar", "lw_description")

    def create_clean_fixture(
        self, workspace: Path, install_layout: str = "isolated"
    ) -> dict[str, str]:
        (workspace / "scripts").mkdir(parents=True)
        shutil.copy2(BUILD_SCRIPT, workspace / "build.sh")
        shutil.copy2(
            BUILD_SCRIPT.parent / "scripts" / "common.sh",
            workspace / "scripts" / "common.sh",
        )

        for package in self.CLEAN_PACKAGES:
            package_dir = workspace / "src" / package
            package_dir.mkdir(parents=True)
            (package_dir / "package.xml").write_text(
                f"<package><name>{package}</name></package>\n",
                encoding="utf-8",
            )
            for artifact_root in ("build", "install"):
                artifact = workspace / artifact_root / package
                artifact.mkdir(parents=True)
                (artifact / "keep.txt").write_text(package, encoding="utf-8")

        install_root = workspace / "install"
        (install_root / ".colcon_install_layout").write_text(
            f"{install_layout}\n", encoding="utf-8"
        )
        for shared_directory in ("log", "cmake_build", "logs"):
            directory = workspace / shared_directory
            directory.mkdir()
            (directory / "keep.txt").write_text(
                shared_directory, encoding="utf-8"
            )

        fake_bin = workspace / "fake-bin"
        fake_bin.mkdir()
        fake_colcon = fake_bin / "colcon"
        fake_colcon.write_text(
            "#!/bin/bash\n"
            "set -e\n"
            "if [[ \" $* \" == *\" --packages-above \"* ]]; then\n"
            "  if [[ \" $* \" == *\" serial \"* ]]; then\n"
            "    printf '%s\\n' serial fdilink_ahrs rl_sar\n"
            "  elif [[ \" $* \" == *\" fdilink_ahrs \"* ]]; then\n"
            "    printf '%s\\n' fdilink_ahrs rl_sar\n"
            "  elif [[ \" $* \" == *\" rl_sar \"* ]]; then\n"
            "    printf '%s\\n' rl_sar\n"
            "  else\n"
            "    printf '%s\\n' lw_description\n"
            "  fi\n"
            "else\n"
            "  printf '%s\\n' serial fdilink_ahrs rl_sar lw_description\n"
            "fi\n",
            encoding="utf-8",
        )
        fake_colcon.chmod(0o755)

        environment = os.environ.copy()
        environment.update(
            {
                "PATH": f"{fake_bin}:{environment['PATH']}",
                "ROS_DISTRO": "humble",
            }
        )
        return environment

    def run_clean(
        self,
        workspace: Path,
        arguments: list[str],
        environment: dict[str, str],
        answer: str = "",
    ) -> subprocess.CompletedProcess[str]:
        return subprocess.run(
            ["bash", str(workspace / "build.sh"), *arguments],
            check=False,
            cwd=workspace.parent,
            env=environment,
            input=answer,
            text=True,
            capture_output=True,
        )

    def test_lw_wrapper_was_removed(self) -> None:
        self.assertFalse(REMOVED_LW_SCRIPT.exists())

    def test_jetson_pytorch_installer_was_removed(self) -> None:
        self.assertFalse(REMOVED_JETSON_INSTALLER.exists())

    def test_obsolete_standalone_cmake_modes_were_removed(self) -> None:
        content = BUILD_SCRIPT.read_text(encoding="utf-8")
        self.assertNotIn("run_cmake_build()", content)
        self.assertNotIn("-m|--cmake", content)
        self.assertNotIn("cmake_mode", content)
        self.assertNotIn("--cmake option", content)
        self.assertNotIn("run_mujoco_build()", content)
        self.assertNotIn("-mj|--mujoco", content)
        self.assertNotIn("mujoco_mode", content)

        result = subprocess.run(
            ["bash", str(BUILD_SCRIPT), "--help"],
            check=True,
            text=True,
            capture_output=True,
        )
        self.assertNotIn("--cmake", result.stdout)
        self.assertNotIn("--mujoco", result.stdout)

        for document in (DEPLOYMENT_GUIDE, QUICK_START_GUIDE):
            document_content = document.read_text(encoding="utf-8")
            self.assertNotIn("--cmake", document_content)
            self.assertNotIn("--mujoco", document_content)

        cmake = CMAKE_FILE.read_text(encoding="utf-8")
        self.assertNotIn("USE_CMAKE", cmake)
        self.assertNotIn("USE_MUJOCO", cmake)
        self.assertNotIn("rl_sim_mujoco", cmake)
        self.assertIn("add_executable(rl_sim_LW", cmake)
        self.assertIn("lw_mujoco_simulate_vendor", cmake)

    def test_selected_builds_include_dependency_closure(self) -> None:
        content = BUILD_SCRIPT.read_text(encoding="utf-8")
        self.assertNotIn("--packages-select", content)
        self.assertNotIn("--merge-install", content)
        self.assertIn("colcon build --symlink-install", content)
        self.assertIn('--packages-up-to "${packages[@]}"', content)
        self.assertNotIn("catkin", content.lower())
        self.assertNotIn("package.ros1.xml", content)
        self.assertNotIn("package.ros2.xml", content)

        build_function = content[content.index("run_ros_build()") :]
        build_function = build_function[: build_function.index("# Clean Functions")]
        self.assertLess(
            build_function.index("detect_incompatible_build_artifacts"),
            build_function.index("--packages-up-to"),
        )

    def test_clean_help_and_documentation_match_isolated_layout(self) -> None:
        result = subprocess.run(
            ["bash", str(BUILD_SCRIPT), "--help"],
            check=True,
            text=True,
            capture_output=True,
        )
        self.assertIn("selected packages and reverse dependencies", result.stdout)
        self.assertIn("--clean package1", result.stdout)

        deployment = DEPLOYMENT_GUIDE.read_text(encoding="utf-8")
        quick_start = QUICK_START_GUIDE.read_text(encoding="utf-8")
        for document in (deployment, quick_start):
            self.assertIn("isolated", document)
            self.assertNotIn("merged install", document.lower())
            self.assertNotIn("--merge-install", document)
            self.assertIn("./build.sh --clean", document)
        self.assertIn("./build.sh --clean serial", deployment)
        self.assertIn("./build.sh --clean rl_sar", quick_start)

    def test_selected_clean_removes_only_package_closure(self) -> None:
        with tempfile.TemporaryDirectory(
            prefix="lw-package-clean-test-"
        ) as temporary:
            workspace = Path(temporary) / "workspace"
            environment = self.create_clean_fixture(workspace)

            result = self.run_clean(
                workspace, ["--clean", "serial"], environment, "y\n"
            )

            self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
            for package in ("serial", "fdilink_ahrs", "rl_sar"):
                self.assertIn(f"build/{package}", result.stdout)
                self.assertIn(f"install/{package}", result.stdout)
                self.assertFalse((workspace / "build" / package).exists())
                self.assertFalse((workspace / "install" / package).exists())
            self.assertTrue((workspace / "build/lw_description/keep.txt").is_file())
            self.assertTrue((workspace / "install/lw_description/keep.txt").is_file())
            self.assertTrue((workspace / "log/keep.txt").is_file())
            self.assertTrue((workspace / "cmake_build/keep.txt").is_file())
            for package in self.CLEAN_PACKAGES:
                manifest = workspace / "src" / package / "package.xml"
                self.assertTrue(manifest.is_file())
                self.assertFalse(manifest.is_symlink())

    def test_selected_clean_rejects_unknown_package_before_deletion(self) -> None:
        with tempfile.TemporaryDirectory(
            prefix="lw-package-clean-test-"
        ) as temporary:
            workspace = Path(temporary) / "workspace"
            environment = self.create_clean_fixture(workspace)

            result = self.run_clean(
                workspace, ["--clean", "../rl_sar"], environment
            )

            self.assertNotEqual(result.returncode, 0)
            self.assertIn("Invalid package name '../rl_sar'", result.stdout)
            for package in self.CLEAN_PACKAGES:
                self.assertTrue((workspace / "build" / package / "keep.txt").is_file())
                self.assertTrue((workspace / "install" / package / "keep.txt").is_file())

    def test_selected_clean_rejects_merged_install_without_changes(self) -> None:
        with tempfile.TemporaryDirectory(
            prefix="lw-package-clean-test-"
        ) as temporary:
            workspace = Path(temporary) / "workspace"
            environment = self.create_clean_fixture(workspace, "merged")

            result = self.run_clean(
                workspace, ["--clean", "rl_sar"], environment
            )

            self.assertNotEqual(result.returncode, 0)
            self.assertIn("requires an isolated Colcon install layout", result.stdout)
            self.assertIn("./build.sh --clean", result.stdout)
            for package in self.CLEAN_PACKAGES:
                self.assertTrue((workspace / "build" / package / "keep.txt").is_file())
                self.assertTrue((workspace / "install" / package / "keep.txt").is_file())

    def test_selected_clean_rejects_symlinked_artifact_root(self) -> None:
        with tempfile.TemporaryDirectory(
            prefix="lw-package-clean-test-"
        ) as temporary:
            workspace = Path(temporary) / "workspace"
            environment = self.create_clean_fixture(workspace)
            external_build = workspace / "external-build"
            (workspace / "build").rename(external_build)
            (workspace / "build").symlink_to(
                external_build, target_is_directory=True
            )

            result = self.run_clean(
                workspace, ["--clean", "rl_sar"], environment
            )

            self.assertNotEqual(result.returncode, 0)
            self.assertIn("refuses a symlinked artifact root", result.stdout)
            for package in self.CLEAN_PACKAGES:
                self.assertTrue((external_build / package / "keep.txt").is_file())
                self.assertTrue((workspace / "install" / package / "keep.txt").is_file())

    def test_selected_clean_cancellation_preserves_workspace(self) -> None:
        with tempfile.TemporaryDirectory(
            prefix="lw-package-clean-test-"
        ) as temporary:
            workspace = Path(temporary) / "workspace"
            environment = self.create_clean_fixture(workspace)

            result = self.run_clean(
                workspace, ["--clean", "rl_sar"], environment, "n\n"
            )

            self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
            self.assertIn("Clean operation cancelled", result.stdout)
            for package in self.CLEAN_PACKAGES:
                self.assertTrue((workspace / "build" / package / "keep.txt").is_file())
                self.assertTrue((workspace / "install" / package / "keep.txt").is_file())

    def test_non_ros2_environment_is_rejected_before_cleaning(self) -> None:
        with tempfile.TemporaryDirectory(
            prefix="lw-package-clean-test-"
        ) as temporary:
            workspace = Path(temporary) / "workspace"
            environment = self.create_clean_fixture(workspace)
            environment["ROS_DISTRO"] = "noetic"

            result = self.run_clean(
                workspace, ["--clean", "rl_sar"], environment
            )

            self.assertNotEqual(result.returncode, 0)
            self.assertIn("supports ROS 2 Foxy and Humble only", result.stdout)
            for package in self.CLEAN_PACKAGES:
                self.assertTrue((workspace / "build" / package / "keep.txt").is_file())
                self.assertTrue((workspace / "install" / package / "keep.txt").is_file())

    def test_non_ros2_build_is_rejected_before_dependency_setup(self) -> None:
        with tempfile.TemporaryDirectory(
            prefix="lw-package-clean-test-"
        ) as temporary:
            workspace = Path(temporary) / "workspace"
            environment = self.create_clean_fixture(workspace)
            environment["ROS_DISTRO"] = "noetic"

            result = self.run_clean(workspace, [], environment)

            self.assertNotEqual(result.returncode, 0)
            self.assertIn("supports ROS 2 Foxy and Humble only", result.stdout)
            self.assertNotIn("Dependency installer not found", result.stdout)
            for package in self.CLEAN_PACKAGES:
                self.assertTrue((workspace / "build" / package / "keep.txt").is_file())
                self.assertTrue((workspace / "install" / package / "keep.txt").is_file())

    def test_full_clean_keeps_existing_workspace_wide_behavior(self) -> None:
        with tempfile.TemporaryDirectory(
            prefix="lw-package-clean-test-"
        ) as temporary:
            workspace = Path(temporary) / "workspace"
            environment = self.create_clean_fixture(workspace, "merged")

            result = self.run_clean(
                workspace, ["--clean"], environment, "y\n"
            )

            self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
            for artifact in ("build", "install", "log", "logs", "cmake_build"):
                self.assertFalse((workspace / artifact).exists())
            for package in self.CLEAN_PACKAGES:
                package_dir = workspace / "src" / package
                manifest = package_dir / "package.xml"
                self.assertTrue(manifest.is_file())
                self.assertFalse(manifest.is_symlink())

    def test_ros2_packages_use_standard_manifests(self) -> None:
        repository_root = BUILD_SCRIPT.parent
        for relative_package in (
            "src/rl_sar",
            "src/robot_joint_controller",
            "src/robot_msgs",
        ):
            package_dir = repository_root / relative_package
            manifest = package_dir / "package.xml"
            self.assertTrue(manifest.is_file())
            self.assertFalse(manifest.is_symlink())
            self.assertFalse((package_dir / "package.ros1.xml").exists())
            self.assertFalse((package_dir / "package.ros2.xml").exists())

    def test_documentation_uses_the_single_entry_point(self) -> None:
        for document in (README, DEPLOYMENT_GUIDE, QUICK_START_GUIDE):
            content = document.read_text(encoding="utf-8")
            self.assertNotIn("build_LW.sh", content)
            self.assertIn("./build.sh", content)

    def test_quick_start_preserves_deployment_safety_gates(self) -> None:
        quick_start = QUICK_START_GUIDE.read_text(encoding="utf-8")
        deployment_guide = DEPLOYMENT_GUIDE.read_text(encoding="utf-8")

        shared_commands = (
            "src/rl_sar/scripts/build_lw_deployment.sh",
            "--verify-deployment-only",
            "collect-host",
            "collect-hardware",
            "I_CONFIRM_LW_IS_SUSPENDED_AND_MOTORS_MUST_REMAIN_DISABLED",
            "PYTHONDONTWRITEBYTECODE=1 ros2 launch rl_sar rl_real_LW.launch.py",
        )
        for command in shared_commands:
            self.assertIn(command, quick_start)
            self.assertIn(command, deployment_guide)

        for safety_gate in (
            "Sim2Sim",
            "可靠吊装",
            "运动范围隔离",
            "物理急停",
            "不会使能电机",
            "motors_disable=true",
            "candidate-review.json",
            "不得把它直接覆盖到当前部署包",
        ):
            self.assertIn(safety_gate, quick_start)

        self.assertIn("LW_BUILD_DEPLOYMENT_CN.md", quick_start)

    def test_build_runs_dependency_setup_before_runtime_setup(self) -> None:
        content = BUILD_SCRIPT.read_text(encoding="utf-8")
        main = content[content.index("main()") :]
        self.assertIn("setup_system_dependencies", main)
        self.assertLess(
            main.index("setup_system_dependencies"),
            main.index("setup_inference_runtime"),
        )

    def test_all_builds_prepare_only_onnx(self) -> None:
        build_content = BUILD_SCRIPT.read_text(encoding="utf-8")
        setup = build_content[build_content.index("setup_inference_runtime()") :]
        setup = setup[: setup.index("setup_system_dependencies()")]
        self.assertIn("runtime_target=onnx", setup)
        self.assertNotIn("runtime_target=all", setup)
        self.assertNotIn('if [ "${IS_JETSON}" = true ]', setup)
        self.assertIn('bash "$DOWNLOAD_SCRIPT" "$runtime_target"', setup)

        downloader = RUNTIME_DOWNLOADER.read_text(encoding="utf-8")
        self.assertIn('DOWNLOAD_TARGET="onnx"', downloader)
        self.assertIn('ensure_runtime onnx "$ONNXRUNTIME_VERSION"', downloader)
        self.assertNotIn("LIBTORCH_VERSION", downloader)
        self.assertNotIn("ensure_runtime libtorch", downloader)
        self.assertNotIn("install_pytorch_jetson.sh", downloader)

    def test_runtime_download_is_pinned_and_transactional(self) -> None:
        downloader = RUNTIME_DOWNLOADER.read_text(encoding="utf-8")
        self.assertIn("inference_runtime_archives.json", downloader)
        self.assertIn("manage_inference_runtime.py", downloader)
        self.assertIn("Expected SHA-256", downloader)
        self.assertNotIn('rm -rf "$LIBTORCH_DIR"', downloader)
        self.assertNotIn('rm -rf "$ONNXRUNTIME_DIR"', downloader)
        self.assertNotIn("Darwin)", downloader)
        self.assertNotIn("MINGW", downloader)

    def test_removed_libtorch_target_is_rejected(self) -> None:
        environment = os.environ.copy()
        environment.update(
            {
                "IS_JETSON": "true",
                "RL_SAR_PLATFORM_OS": "Linux",
                "RL_SAR_PLATFORM_ARCH": "aarch64",
            }
        )
        result = subprocess.run(
            ["bash", str(RUNTIME_DOWNLOADER), "libtorch"],
            check=False,
            text=True,
            capture_output=True,
            env=environment,
        )
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("Download target must be onnx", result.stdout)
        self.assertNotIn("Downloading", result.stdout)

    def test_production_cmake_and_deployment_are_onnx_only(self) -> None:
        cmake = CMAKE_FILE.read_text(encoding="utf-8")
        self.assertIn("if(IS_JETSON OR LW_PRODUCTION_DEPLOYMENT)", cmake)
        self.assertNotIn("TORCH_BACKEND_ALLOWED", cmake)
        self.assertNotIn("USE_TORCH", cmake)
        self.assertNotIn("find_package(Torch", cmake)
        self.assertIn("ONNX Runtime validation failed", cmake)
        self.assertIn('INSTALL_RPATH "$ORIGIN/onnxruntime"', cmake)
        self.assertIn('RENAME "libonnxruntime.so.1"', cmake)
        self.assertIn("LW_ONNX_RUNTIME_PROVIDER_LIBRARY", cmake)

        deployment = DEPLOYMENT_BUILDER.read_text(encoding="utf-8")
        self.assertIn("validate_inference_runtime.sh", deployment)
        self.assertIn('grep -Eq "libtorch|libc10"', deployment)
        self.assertIn("$ORIGIN/onnxruntime", deployment)
        self.assertIn("retains build-tree ONNX RPATH", deployment)
        self.assertIn("resolved ONNX Runtime outside the deployment", deployment)
        self.assertIn('verify_deployment_prefix "$relocated_prefix"', deployment)
        self.assertIn(
            "PYTHONDONTWRITEBYTECODE=1", deployment
        )
        self.assertIn(
            "ros2 launch rl_sar rl_real_LW.launch.py --show-args", deployment
        )
        self.assertIn('"$output_prefix/share/rl_sar/launch"', deployment)

    def test_production_install_contains_only_real_launch(self) -> None:
        cmake = CMAKE_FILE.read_text(encoding="utf-8")
        install_block = cmake[cmake.index("# executable for actuator net") :]
        production_block = install_block[
            install_block.index("if(LW_PRODUCTION_DEPLOYMENT)") :
        ]
        production_block, development_block = production_block.split(
            "else()", 1
        )

        self.assertIn("FILES launch/rl_real_LW.launch.py", production_block)
        self.assertNotIn("DIRECTORY launch worlds", production_block)
        self.assertIn("DIRECTORY launch worlds", development_block)

    def test_offline_actuator_training_assets_remain_installed(self) -> None:
        cmake = CMAKE_FILE.read_text(encoding="utf-8")
        install_block = cmake[cmake.index("# executable for actuator net") :]
        self.assertIn("scripts/actuator_net.py", install_block)

        repository_root = CMAKE_FILE.parents[2]
        self.assertTrue(
            (repository_root / "policy/LW/robot_lab/motors/leg_actuator_net.pt")
            .is_file()
        )
        self.assertTrue(
            (repository_root / "policy/LW/robot_lab/motors/foot_actuator_net.pt")
            .is_file()
        )

    def test_dependency_installer_lists_base_and_ros_packages(self) -> None:
        environment = os.environ.copy()
        environment["ROS_DISTRO"] = "humble"
        result = subprocess.run(
            ["bash", str(DEPENDENCY_INSTALLER), "--print-packages"],
            check=True,
            text=True,
            capture_output=True,
            env=environment,
        )
        packages = set(result.stdout.splitlines())
        self.assertIn("python3-pip", packages)
        self.assertIn("libtbb-dev", packages)
        self.assertIn("ros-humble-ros2-control", packages)

    def test_strict_warning_gate_is_reproducible_and_vendor_scoped(self) -> None:
        validator = STRICT_BUILD_VALIDATOR.read_text(encoding="utf-8")
        self.assertIn("-DLW_STRICT_WARNINGS=ON", validator)
        self.assertIn('cmake --build "${strict_build_dir}"', validator)
        self.assertIn('ctest --test-dir "${strict_build_dir}"', validator)

        cmake = CMAKE_FILE.read_text(encoding="utf-8")
        self.assertIn("option(\n    LW_STRICT_WARNINGS", cmake)
        for warning in ("-Wall", "-Wextra", "-Wpedantic", "-Werror"):
            self.assertIn(warning, cmake)
        self.assertIn("function(lw_silence_vendor_warnings", cmake)
        self.assertIn("lw_joystick_vendor", cmake)
        self.assertIn("lw_mujoco_simulate_vendor", cmake)
        self.assertIn("target_include_directories(LW_sdk SYSTEM INTERFACE", cmake)

        sim_target = cmake[cmake.index("add_executable(rl_sim_LW") :]
        sim_target = sim_target[: sim_target.index("target_link_libraries(rl_sim_LW")]
        self.assertNotIn("MUJOCO_SIMULATE_SRC", sim_target)
        self.assertNotIn("JOYSTICK_SRC", sim_target)

    def test_missing_dependencies_invoke_package_manager(self) -> None:
        with tempfile.TemporaryDirectory(
            prefix="lw-build-dependency-test-"
        ) as temporary:
            temporary_path = Path(temporary)
            log = temporary_path / "apt.log"
            for command in ("apt-get", "sudo"):
                executable = temporary_path / command
                executable.write_text(
                    "#!/bin/sh\n"
                    "printf '%s\\n' \"$*\" >> \"$RL_SAR_APT_LOG\"\n",
                    encoding="utf-8",
                )
                executable.chmod(0o755)
            dpkg_query = temporary_path / "dpkg-query"
            dpkg_query.write_text("#!/bin/sh\nexit 1\n", encoding="utf-8")
            dpkg_query.chmod(0o755)

            environment = os.environ.copy()
            environment.update(
                {
                    "PATH": f"{temporary_path}:{environment['PATH']}",
                    "ROS_DISTRO": "humble",
                    "RL_SAR_APT_LOG": str(log),
                }
            )
            result = subprocess.run(
                ["bash", str(DEPENDENCY_INSTALLER)],
                check=False,
                text=True,
                capture_output=True,
                env=environment,
            )
            self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
            calls = log.read_text(encoding="utf-8")
            self.assertIn("update", calls)
            self.assertIn("install -y", calls)
            self.assertIn("python3-pip", calls)
            self.assertIn("ros-humble-ros2-control", calls)

    def test_installed_dependencies_skip_package_manager(self) -> None:
        with tempfile.TemporaryDirectory(
            prefix="lw-build-dependency-test-"
        ) as temporary:
            temporary_path = Path(temporary)
            log = temporary_path / "apt.log"
            apt_get = temporary_path / "apt-get"
            apt_get.write_text(
                "#!/bin/sh\n"
                "printf '%s\\n' \"$*\" >> \"$RL_SAR_APT_LOG\"\n",
                encoding="utf-8",
            )
            apt_get.chmod(0o755)
            dpkg_query = temporary_path / "dpkg-query"
            dpkg_query.write_text(
                "#!/bin/sh\nprintf '%s\\n' 'install ok installed'\n",
                encoding="utf-8",
            )
            dpkg_query.chmod(0o755)

            environment = os.environ.copy()
            environment.update(
                {
                    "PATH": f"{temporary_path}:{environment['PATH']}",
                    "ROS_DISTRO": "humble",
                    "RL_SAR_APT_LOG": str(log),
                }
            )
            result = subprocess.run(
                ["bash", str(DEPENDENCY_INSTALLER)],
                check=False,
                text=True,
                capture_output=True,
                env=environment,
            )
            self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
            self.assertIn("already installed", result.stdout)
            self.assertFalse(log.exists())


if __name__ == "__main__":
    unittest.main()
