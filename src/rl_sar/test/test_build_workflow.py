#!/usr/bin/env python3

from pathlib import Path
import os
import subprocess
import sys
import tempfile
import unittest


BUILD_SCRIPT = Path(sys.argv.pop(1)).resolve()
REMOVED_LW_SCRIPT = Path(sys.argv.pop(1)).resolve()
README = Path(sys.argv.pop(1)).resolve()
DEPLOYMENT_GUIDE = Path(sys.argv.pop(1)).resolve()
DEPENDENCY_INSTALLER = Path(sys.argv.pop(1)).resolve()
RUNTIME_DOWNLOADER = Path(sys.argv.pop(1)).resolve()
REMOVED_JETSON_INSTALLER = Path(sys.argv.pop(1)).resolve()
CMAKE_FILE = Path(sys.argv.pop(1)).resolve()
DEPLOYMENT_BUILDER = Path(sys.argv.pop(1)).resolve()
STRICT_BUILD_VALIDATOR = Path(sys.argv.pop(1)).resolve()


class BuildWorkflowTests(unittest.TestCase):
    def test_lw_wrapper_was_removed(self) -> None:
        self.assertFalse(REMOVED_LW_SCRIPT.exists())

    def test_jetson_pytorch_installer_was_removed(self) -> None:
        self.assertFalse(REMOVED_JETSON_INSTALLER.exists())

    def test_selected_builds_include_dependency_closure(self) -> None:
        content = BUILD_SCRIPT.read_text(encoding="utf-8")
        self.assertNotIn("--packages-select", content)
        self.assertIn('--packages-up-to "${packages[@]}"', content)

        build_function = content[content.index("run_ros_build()") :]
        build_function = build_function[: build_function.index("# Clean Functions")]
        self.assertLess(
            build_function.index("create_symlinks_for_all_packages"),
            build_function.index("--packages-up-to"),
        )

    def test_documentation_uses_the_single_entry_point(self) -> None:
        for document in (README, DEPLOYMENT_GUIDE):
            content = document.read_text(encoding="utf-8")
            self.assertNotIn("build_LW.sh", content)
            self.assertIn("./build.sh", content)

    def test_build_runs_dependency_setup_before_runtime_setup(self) -> None:
        content = BUILD_SCRIPT.read_text(encoding="utf-8")
        main = content[content.index("main()") :]
        self.assertIn("setup_system_dependencies", main)
        self.assertLess(
            main.index("setup_system_dependencies"),
            main.index("setup_inference_runtime"),
        )

    def test_jetson_build_prepares_only_onnx(self) -> None:
        build_content = BUILD_SCRIPT.read_text(encoding="utf-8")
        setup = build_content[build_content.index("setup_inference_runtime()") :]
        setup = setup[: setup.index("setup_system_dependencies()")]
        self.assertIn("runtime_target=all", setup)
        self.assertIn('if [ "${IS_JETSON}" = true ]', setup)
        self.assertIn("runtime_target=onnx", setup)
        self.assertIn('bash "$DOWNLOAD_SCRIPT" "$runtime_target"', setup)

        downloader = RUNTIME_DOWNLOADER.read_text(encoding="utf-8")
        self.assertIn("Jetson production path is ONNX-only", downloader)
        self.assertIn("DOWNLOAD_TARGET=onnx", downloader)
        self.assertNotIn("install_pytorch_jetson.sh", downloader)

    def test_non_jetson_development_keeps_automatic_libtorch(self) -> None:
        build_content = BUILD_SCRIPT.read_text(encoding="utf-8")
        setup = build_content[build_content.index("setup_inference_runtime()") :]
        setup = setup[: setup.index("setup_system_dependencies()")]
        self.assertIn("runtime_target=all", setup)

        downloader = RUNTIME_DOWNLOADER.read_text(encoding="utf-8")
        all_target = downloader[downloader.index('    all)') :]
        self.assertIn('ensure_runtime libtorch "$LIBTORCH_VERSION"', all_target)
        self.assertIn('ensure_runtime onnx "$ONNXRUNTIME_VERSION"', all_target)

    def test_runtime_download_is_pinned_and_transactional(self) -> None:
        downloader = RUNTIME_DOWNLOADER.read_text(encoding="utf-8")
        self.assertIn("inference_runtime_archives.json", downloader)
        self.assertIn("manage_inference_runtime.py", downloader)
        self.assertIn("Expected SHA-256", downloader)
        self.assertNotIn('rm -rf "$LIBTORCH_DIR"', downloader)
        self.assertNotIn('rm -rf "$ONNXRUNTIME_DIR"', downloader)
        self.assertNotIn("Darwin)", downloader)
        self.assertNotIn("MINGW", downloader)

    def test_explicit_jetson_libtorch_request_is_rejected(self) -> None:
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
        self.assertIn("not supported", result.stdout)
        self.assertNotIn("Downloading", result.stdout)

    def test_production_cmake_and_deployment_are_onnx_only(self) -> None:
        cmake = CMAKE_FILE.read_text(encoding="utf-8")
        self.assertIn("if(IS_JETSON OR LW_PRODUCTION_DEPLOYMENT)", cmake)
        self.assertIn("set(TORCH_BACKEND_ALLOWED FALSE)", cmake)
        self.assertIn("LibTorch disabled", cmake)
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
        install_block = cmake[cmake.rindex("if(NOT USE_CMAKE)") :]
        production_block = install_block[
            install_block.index("if(LW_PRODUCTION_DEPLOYMENT)") :
        ]
        production_block, development_block = production_block.split(
            "else()", 1
        )

        self.assertIn("FILES launch/rl_real_LW.launch.py", production_block)
        self.assertNotIn("DIRECTORY launch worlds", production_block)
        self.assertIn("DIRECTORY launch worlds", development_block)

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
