#!/usr/bin/env python3

import argparse
import os
from pathlib import Path
import re
import subprocess


def parse_arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Verify maintained LW binaries do not link Python"
    )
    parser.add_argument("--readelf", required=True)
    parser.add_argument("--binary", action="append", required=True, type=Path)
    return parser.parse_args()


def dynamic_section(readelf: str, binary: Path) -> str:
    result = subprocess.run(
        [readelf, "-d", str(binary)],
        check=False,
        text=True,
        capture_output=True,
    )
    if result.returncode != 0:
        raise RuntimeError(
            f"readelf failed for {binary}: {result.stderr.strip()}"
        )
    return result.stdout


def dynamic_search_paths(section: str) -> list[str]:
    paths: list[str] = []
    for match in re.finditer(
        r"\((?:RPATH|RUNPATH)\).*?\[(.*?)\]", section
    ):
        paths.extend(path for path in match.group(1).split(":") if path)
    return paths


def is_within(path: Path, root: Path) -> bool:
    try:
        path.relative_to(root)
        return True
    except ValueError:
        return False


def verify_binary(readelf: str, binary: Path) -> None:
    binary = binary.resolve(strict=True)
    section = dynamic_section(readelf, binary)
    if re.search(r"\bNEEDED\b.*\blibpython", section, re.IGNORECASE):
        raise RuntimeError(f"{binary} unexpectedly depends on libpython")

    conda_prefix_value = os.environ.get("CONDA_PREFIX")
    if not conda_prefix_value:
        return
    conda_prefix = Path(conda_prefix_value).resolve(strict=False)
    for search_path in dynamic_search_paths(section):
        expanded = search_path.replace("$ORIGIN", str(binary.parent))
        resolved = Path(expanded).resolve(strict=False)
        if is_within(resolved, conda_prefix):
            raise RuntimeError(
                f"{binary} has a Conda runtime search path: {search_path}"
            )


def main() -> int:
    arguments = parse_arguments()
    for binary in arguments.binary:
        verify_binary(arguments.readelf, binary)
    print(
        "LW runtime linkage verified for "
        f"{len(arguments.binary)} binaries"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
