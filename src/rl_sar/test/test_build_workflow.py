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


class BuildWorkflowTests(unittest.TestCase):
    def test_lw_wrapper_was_removed(self) -> None:
        self.assertFalse(REMOVED_LW_SCRIPT.exists())

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
