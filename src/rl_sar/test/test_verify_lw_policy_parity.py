#!/usr/bin/env python3

import hashlib
from pathlib import Path
import subprocess
import sys
import tempfile


POLICY_FILES = (
    "policy/LW/base.yaml",
    "policy/LW/robot_lab/leg_loco/config.yaml",
    "policy/LW/robot_lab/leg_loco/policy.onnx",
    "policy/LW/robot_lab/leg_to_wheel/config.yaml",
    "policy/LW/robot_lab/leg_to_wheel/leg_to_wheel_transform_60hz.csv",
    "policy/LW/robot_lab/leg_to_wheel/policy.onnx",
    "policy/LW/robot_lab/wheel_loco/config.yaml",
    "policy/LW/robot_lab/wheel_loco/policy.onnx",
    "policy/LW/robot_lab/wheel_to_leg/config.yaml",
    "policy/LW/robot_lab/wheel_to_leg/policy.onnx",
    "policy/LW/robot_lab/wheel_to_leg/wheel_to_leg_transform_60hz.csv",
)


def digest(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def run(script: Path, policy_root: Path, manifest: Path) -> subprocess.CompletedProcess:
    return subprocess.run(
        [
            sys.executable,
            str(script),
            "--policy-root",
            str(policy_root),
            "--manifest",
            str(manifest),
        ],
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        check=False,
    )


def main() -> int:
    if len(sys.argv) != 2:
        raise RuntimeError("expected parity verifier path")
    script = Path(sys.argv[1]).resolve()
    with tempfile.TemporaryDirectory(prefix="lw-policy-parity-") as temporary:
        root = Path(temporary)
        policy_root = root / "policy"
        manifest = root / "manifest.yaml"
        for relative_string in POLICY_FILES:
            relative = Path(relative_string).relative_to("policy")
            asset = policy_root / relative
            asset.parent.mkdir(parents=True, exist_ok=True)
            asset.write_bytes(f"asset:{relative_string}\n".encode())

        lines = [
            "schema_version: 3",
            f'source_commit: "{"a" * 40}"',
            'build_type: "Release"',
            "files:",
        ]
        for relative_string in POLICY_FILES:
            asset = policy_root / Path(relative_string).relative_to("policy")
            lines.extend(
                [
                    f'  - path: "{relative_string}"',
                    f'    sha256: "{digest(asset)}"',
                ]
            )
        lines.append("runtime_files:")
        manifest.write_text("\n".join(lines) + "\n", encoding="utf-8")

        valid = run(script, policy_root, manifest)
        if valid.returncode != 0:
            raise RuntimeError(f"valid policy parity failed: {valid.stderr}")

        tampered = policy_root / "LW" / "base.yaml"
        tampered.write_bytes(tampered.read_bytes() + b"tampered\n")
        invalid = run(script, policy_root, manifest)
        if invalid.returncode == 0 or "SHA-256 mismatch" not in invalid.stderr:
            raise RuntimeError(
                "tampered policy was not rejected: "
                f"stdout={invalid.stdout!r}, stderr={invalid.stderr!r}"
            )
    print("LW policy parity verifier tests passed")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
