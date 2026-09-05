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

ISOLATED_TARGETS = (
    "observation_buffer", "test_observation_buffer",
    "test_lw_safety_policy", "lw_joystick_vendor",
)


def require(condition: bool, message: str) -> None:
    if not condition:
        raise RuntimeError(message)


def preprocessor_options(
    tokens: list[str], directory: Path,
) -> tuple[set[Path], set[str]]:
    """Parse joined/separate GCC/Clang options after shell tokenization."""
    includes: set[Path] = set()
    definitions: set[str] = set()
    index = 0
    while index < len(tokens):
        token = tokens[index]
        for option in ("-isystem", "-iquote", "-idirafter", "-I", "-D", "-U"):
            if not token.startswith(option):
                continue
            value = token[len(option):]
            if not value:
                index += 1
                require(index < len(tokens), f"Missing argument for {option}")
                value = tokens[index]
            if option == "-D":
                definitions.add(value.split("=", 1)[0])
            elif option == "-U":
                definitions.discard(value)
            else:
                includes.add((directory / value).resolve())
            break
        index += 1
    return includes, definitions


def verify_compile_command(
    target: str, tokens: list[str], directory: Path,
    onnx_dir: Path, strict: bool, target_options: list[str],
) -> None:
    includes, definitions = preprocessor_options(tokens, directory)
    has_onnx_header = (onnx_dir / "include").resolve() in includes
    require(("USE_ONNX" in definitions) == has_onnx_header,
            f"{target}: ONNX public header/layout contract differs")
    require("BOOST_BIND_GLOBAL_PLACEHOLDERS" not in definitions,
            f"{target}: obsolete global Boost definition")
    require(("CMAKE_CURRENT_SOURCE_DIR" in definitions) == (target == "rl_sim_LW"),
            f"{target}: simulator source path leaked or is missing")

    vendor = target in ("lw_joystick_vendor", "lw_mujoco_simulate_vendor")
    for warning in ("-Wall", "-Wextra", "-Wpedantic", "-Werror"):
        require((warning in target_options) == (strict and not vendor),
                f"{target}: incorrect project warning policy: {warning}")
        if strict and not vendor:
            require(warning in tokens,
                    f"{target}: strict warning missing from command: {warning}")
    # Extra user/toolchain warnings in the final command are allowed, including
    # for vendor units. They are not evidence of a project target-option leak.

    if target in ISOLATED_TARGETS:
        require(not definitions.intersection({"USE_ONNX", "POLICY_DIR"}),
                f"{target}: unrelated SDK compile definition")
        for forbidden in ("onnxruntime", "yaml-cpp", "core/rl_sdk",
                          "fsm_robot", "library/mujoco"):
            require(not any(forbidden in str(path) for path in includes),
                    f"{target}: unrelated compile dependency {forbidden}")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--build-dir", required=True, type=Path)
    parser.add_argument("--onnx-dir", required=True, type=Path)
    parser.add_argument("--mujoco-dir", required=True, type=Path)
    parser.add_argument("--strict", choices=("ON", "OFF"), required=True)
    parser.add_argument("--target-options-dir", required=True, type=Path)
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
        target_options = (args.target_options_dir / f"{target}.txt").read_text(
            encoding="utf-8",
        ).splitlines()
        verify_compile_command(
            target, tokens, Path(entry["directory"]), args.onnx_dir,
            args.strict == "ON", target_options,
        )

    require(set(ISOLATED_TARGETS).issubset(targets),
            "Required isolated targets are missing from compile_commands.json")
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
