#!/usr/bin/env python3
"""Check generated compiler commands and ELF paths, not just CMake source text."""

import argparse
import json
from pathlib import Path
import re
import shlex

from test_lw_runtime_linkage import (
    dynamic_search_paths,
    dynamic_section,
    verify_binary,
)


def require(condition: bool, message: str) -> None:
    if not condition:
        raise RuntimeError(message)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--build-dir", required=True, type=Path)
    parser.add_argument("--onnx-dir", required=True, type=Path)
    parser.add_argument("--mujoco-dir", required=True, type=Path)
    parser.add_argument("--strict", choices=("ON", "OFF"), required=True)
    parser.add_argument("--readelf", required=True)
    args = parser.parse_args()
    commands = json.loads(
        (args.build_dir / "compile_commands.json").read_text(encoding="utf-8")
    )
    targets = {}
    for entry in commands:
        tokens = entry.get("arguments") or shlex.split(entry["command"])
        match = re.search(r"CMakeFiles/([^/]+)\.dir/", " ".join(tokens))
        require(match is not None, f"Missing target in command: {entry['file']}")
        target = match.group(1)
        targets[target] = tokens
        has_onnx_header = str(args.onnx_dir / "include") in tokens
        require(("-DUSE_ONNX" in tokens) == has_onnx_header,
                f"{target}: ONNX public header/layout contract differs")
        require(not any("BOOST_BIND_GLOBAL_PLACEHOLDERS" in t for t in tokens),
                f"{target}: obsolete global Boost definition")
        source_macro = any(t.startswith("-DCMAKE_CURRENT_SOURCE_DIR=") for t in tokens)
        require(source_macro == (target == "rl_sim_LW"),
                f"{target}: simulator source path leaked or is missing")
        vendor = target in ("lw_joystick_vendor", "lw_mujoco_simulate_vendor")
        for warning in ("-Wall", "-Wextra", "-Wpedantic", "-Werror"):
            require((warning in tokens) == (args.strict == "ON" and not vendor),
                    f"{target}: incorrect maintained/vendor warning policy: {warning}")

    # These small targets must not inherit the SDK, ONNX, YAML or simulator.
    for target in ("observation_buffer", "test_observation_buffer",
                   "test_lw_safety_policy", "lw_joystick_vendor"):
        tokens = targets[target]
        for forbidden in ("USE_ONNX", "POLICY_DIR", "onnxruntime", "yaml-cpp",
                          "core/rl_sdk", "fsm_robot", "library/mujoco"):
            require(not any(forbidden in token for token in tokens),
                    f"{target}: unrelated compile dependency {forbidden}")

    onnx_enabled = "test_inference_runtime" in targets
    for target, needs_onnx, needs_mujoco in (
        ("test_observation_buffer", False, False),
        ("test_lw_safety_policy", False, False),
        ("test_lw_deployment_bundle", False, False),
        ("test_inference_runtime", True, False),
        ("test_lw_fsm_transitions", onnx_enabled, False),
        ("rl_real_LW", onnx_enabled, False),
        ("lw_config_profiler", onnx_enabled, False),
        ("rl_sim_LW", onnx_enabled, True),
        ("test_lw_mujoco_control_adapter", False, True),
    ):
        if target not in targets:
            continue
        binary = args.build_dir / target
        verify_binary(args.readelf, binary)
        section = dynamic_section(args.readelf, binary)
        paths = dynamic_search_paths(section)
        for runtime_dir, required in (
            (args.onnx_dir, needs_onnx), (args.mujoco_dir, needs_mujoco),
        ):
            require((str(runtime_dir / "lib") in paths) == required,
                    f"{target}: incorrect runtime search paths: {paths}")
        require(not any(path in ("/usr/local/lib", "/opt/homebrew/lib") for path in paths),
                f"{target}: global link search path leaked")
        if needs_onnx or needs_mujoco:
            require("(RPATH)" in section and "(RUNPATH)" not in section,
                    f"{target}: prebuilt dependency lost transitive RPATH semantics")
    print(f"LW target scope verified for {len(targets)} compiled targets")


if __name__ == "__main__":
    main()
