#!/usr/bin/env python3
"""Validate the algorithm profile registry used by the training skill."""

from __future__ import annotations

import argparse
import sys

from algorithm_profiles import DEFAULT_REGISTRY_PATH, ProfileError, load_registry


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("registry", nargs="?", default=str(DEFAULT_REGISTRY_PATH))
    args = parser.parse_args()
    try:
        registry = load_registry(args.registry)
    except ProfileError as exc:
        print(f"INVALID: {exc}", file=sys.stderr)
        return 2
    print(f"VALID: {len(registry['profiles'])} profiles")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
