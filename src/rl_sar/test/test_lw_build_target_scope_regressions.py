#!/usr/bin/env python3
"""Positive and negative cases for the LW compiler-command scope checker."""

from pathlib import Path
import shlex
import unittest

from test_lw_build_target_scope import verify_compile_command


class CompileScopeTests(unittest.TestCase):
    directory = Path("/lw/build")
    onnx = Path("/lw/onnxruntime")
    warnings = ["-Wall", "-Wextra", "-Wpedantic", "-Werror"]

    def check(self, tokens, *, target="inference_runtime", strict=False,
              target_options=(), onnx=None):
        verify_compile_command(
            target, tokens, self.directory, onnx or self.onnx,
            strict, list(target_options),
        )

    def test_equivalent_include_and_definition_spellings(self):
        for include in (
            ["-I/lw/onnxruntime/include"],
            ["-I", "/lw/onnxruntime/include"],
            ["-isystem/lw/onnxruntime/include"],
            ["-isystem", "/lw/onnxruntime/include"],
            ["-I../onnxruntime/include"],
        ):
            for definition in (["-DUSE_ONNX"], ["-D", "USE_ONNX=1"]):
                with self.subTest(include=include, definition=definition):
                    self.check(include + definition)
        self.check(
            shlex.split('-I"/lw/runtime with spaces/include" -D USE_ONNX'),
            onnx=Path("/lw/runtime with spaces"),
        )

    def test_user_warnings_are_allowed_without_project_strict_policy(self):
        for target in ("test_observation_buffer", "lw_joystick_vendor"):
            with self.subTest(target=target):
                self.check(self.warnings, target=target)
        self.check(self.warnings, target="lw_joystick_vendor", strict=True)

    def test_strict_policy_must_exist_in_properties_and_final_command(self):
        self.check(self.warnings, target="test_observation_buffer",
                   strict=True, target_options=self.warnings)
        with self.assertRaisesRegex(RuntimeError, "project warning policy"):
            self.check(self.warnings, target="test_observation_buffer",
                       strict=True, target_options=self.warnings[:-1])
        with self.assertRaisesRegex(RuntimeError, "missing from command"):
            self.check(self.warnings[:-1], target="test_observation_buffer",
                       strict=True, target_options=self.warnings)
        with self.assertRaisesRegex(RuntimeError, "project warning policy"):
            self.check(self.warnings, target="lw_joystick_vendor",
                       strict=True, target_options=self.warnings)

    def test_missing_or_undefined_onnx_macro_is_rejected(self):
        for definitions in ([], ["-DUSE_ONNX", "-U", "USE_ONNX"]):
            with self.subTest(definitions=definitions):
                with self.assertRaisesRegex(RuntimeError, "header/layout"):
                    self.check(["-I/lw/onnxruntime/include"] + definitions)
        with self.assertRaisesRegex(RuntimeError, "header/layout"):
            self.check(["-DUSE_ONNX"])

    def test_unrelated_dependency_is_rejected(self):
        with self.assertRaisesRegex(RuntimeError, "unrelated SDK"):
            self.check(["-I/lw/onnxruntime/include", "-DUSE_ONNX"],
                       target="test_observation_buffer")
        with self.assertRaisesRegex(RuntimeError, "unrelated compile dependency"):
            self.check(["-I/lw/library/mujoco/include"],
                       target="test_observation_buffer")


if __name__ == "__main__":
    unittest.main()
