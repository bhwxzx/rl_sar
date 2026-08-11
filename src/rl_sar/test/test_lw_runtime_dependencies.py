#!/usr/bin/env python3

from pathlib import Path
import sys
import unittest
import xml.etree.ElementTree as ElementTree


RL_SAR_PACKAGE = Path(sys.argv.pop(1)).resolve()
FDILINK_PACKAGE = Path(sys.argv.pop(1)).resolve()


def declared_dependencies(package_path: Path) -> dict[str, set[str]]:
    root = ElementTree.parse(package_path).getroot()
    dependencies: dict[str, set[str]] = {}
    for element in root:
        if element.tag.endswith("depend") and element.text:
            dependencies.setdefault(element.tag, set()).add(element.text.strip())
    return dependencies


class RuntimeDependencyTests(unittest.TestCase):
    def test_rl_sar_declares_imu_runtime_dependency(self) -> None:
        dependencies = declared_dependencies(RL_SAR_PACKAGE)
        self.assertIn("fdilink_ahrs", dependencies.get("exec_depend", set()))

    def test_fdilink_declares_all_cmake_dependencies(self) -> None:
        dependencies = declared_dependencies(FDILINK_PACKAGE)
        declared = set().union(*dependencies.values())
        self.assertTrue(
            {"serial", "tf2_geometry_msgs", "eigen"}.issubset(declared),
            f"missing fdilink dependencies: "
            f"{sorted({'serial', 'tf2_geometry_msgs', 'eigen'} - declared)}",
        )


if __name__ == "__main__":
    unittest.main()
