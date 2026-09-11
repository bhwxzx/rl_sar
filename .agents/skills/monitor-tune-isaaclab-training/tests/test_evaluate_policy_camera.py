#!/usr/bin/env python3
"""Regression tests for evaluator camera setup and renderer warmup ordering."""

from __future__ import annotations

import ast
import unittest
from pathlib import Path

import numpy as np


EVALUATOR = (
    Path(__file__).resolve().parents[4]
    / "scripts"
    / "reinforcement_learning"
    / "rsl_rl"
    / "evaluate_policy.py"
)


def _evaluator_tree() -> ast.Module:
    return ast.parse(EVALUATOR.read_text(encoding="utf-8"), filename=str(EVALUATOR))


def _function_node(name: str) -> ast.FunctionDef:
    for node in _evaluator_tree().body:
        if isinstance(node, ast.FunctionDef) and node.name == name:
            return node
    raise AssertionError(f"missing function: {name}")


def _load_prime_video_renderer():
    function = _function_node("_prime_video_renderer")
    module = ast.Module(
        body=[
            ast.ImportFrom(
                module="__future__",
                names=[ast.alias(name="annotations")],
                level=0,
            ),
            function,
        ],
        type_ignores=[],
    )
    namespace = {"np": np}
    exec(compile(ast.fix_missing_locations(module), str(EVALUATOR), "exec"), namespace)
    return namespace["_prime_video_renderer"]


class _FakeUnwrappedEnv:
    def __init__(self, frame):
        self.frame = frame
        self.render_calls = 0
        self.step_calls = 0

    def render(self):
        self.render_calls += 1
        return self.frame

    def step(self, _action):
        self.step_calls += 1
        raise AssertionError("renderer warmup must not step physics")


class _FakeEnv:
    def __init__(self, frame):
        self.unwrapped = _FakeUnwrappedEnv(frame)


class PrimeVideoRendererTests(unittest.TestCase):
    def test_primes_exactly_once_without_stepping(self):
        env = _FakeEnv(np.zeros((720, 1280, 3), dtype=np.uint8))

        _load_prime_video_renderer()(env)

        self.assertEqual(env.unwrapped.render_calls, 1)
        self.assertEqual(env.unwrapped.step_calls, 0)

    def test_rejects_invalid_render_results(self):
        invalid_frames = (
            None,
            np.zeros((720, 1280), dtype=np.uint8),
            np.zeros((720, 1280, 2), dtype=np.uint8),
        )
        for frame in invalid_frames:
            with self.subTest(frame_shape=getattr(frame, "shape", None)):
                env = _FakeEnv(frame)
                with self.assertRaisesRegex(ValueError, "did not return an RGB frame"):
                    _load_prime_video_renderer()(env)
                self.assertEqual(env.unwrapped.render_calls, 1)
                self.assertEqual(env.unwrapped.step_calls, 0)

    def test_helper_contains_no_physics_step(self):
        helper = _function_node("_prime_video_renderer")
        called_attributes = {
            node.func.attr
            for node in ast.walk(helper)
            if isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute)
        }
        self.assertNotIn("step", called_attributes)

    def test_initial_camera_update_then_prime_then_evaluation_loop(self):
        evaluate = _function_node("_evaluate")
        camera_lines = [
            node.lineno
            for node in ast.walk(evaluate)
            if isinstance(node, ast.Call)
            and isinstance(node.func, ast.Name)
            and node.func.id == "_update_follow_camera"
        ]
        prime_lines = [
            node.lineno
            for node in ast.walk(evaluate)
            if isinstance(node, ast.Call)
            and isinstance(node.func, ast.Name)
            and node.func.id == "_prime_video_renderer"
        ]
        loop_lines = [
            node.lineno
            for node in ast.walk(evaluate)
            if isinstance(node, ast.For)
            and isinstance(node.iter, ast.Call)
            and isinstance(node.iter.func, ast.Name)
            and node.iter.func.id == "range"
            and any(
                isinstance(argument, ast.Attribute)
                and isinstance(argument.value, ast.Name)
                and argument.value.id == "args_cli"
                and argument.attr == "duration_steps"
                for argument in node.iter.args
            )
        ]

        self.assertGreaterEqual(len(camera_lines), 1)
        self.assertEqual(len(prime_lines), 1)
        self.assertEqual(len(loop_lines), 1)
        self.assertLess(camera_lines[0], prime_lines[0])
        self.assertLess(prime_lines[0], loop_lines[0])


if __name__ == "__main__":
    unittest.main()
