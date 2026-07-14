#!/usr/bin/env python3
"""Run clang-tidy over project-owned translation units."""

from __future__ import annotations

import argparse
import shutil
import subprocess
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_DIRS = ("src", "tests")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--build-dir",
        required=True,
        help="CMake build directory containing compile_commands.json",
    )
    return parser.parse_args()


def collect_sources() -> list[Path]:
    files: list[Path] = []
    for directory in DEFAULT_DIRS:
        root = REPO_ROOT / directory
        if not root.exists():
            continue
        files.extend(sorted(root.rglob("*.cpp")))
    return files


def main() -> int:
    args = parse_args()
    clang_tidy = shutil.which("clang-tidy")
    if clang_tidy is None:
        print("FAIL: clang-tidy was not found on PATH.")
        return 1

    build_dir = Path(args.build_dir).resolve()
    compile_commands = build_dir / "compile_commands.json"
    if not compile_commands.exists():
        print(f"FAIL: compile_commands.json not found in {build_dir}")
        return 1

    files = collect_sources()
    if not files:
        print("PASS: no C++ translation units found for clang-tidy.")
        return 0

    command = [
        clang_tidy,
        "-p",
        str(build_dir),
        *[str(path) for path in files],
    ]
    completed = subprocess.run(command, cwd=REPO_ROOT)
    return completed.returncode


if __name__ == "__main__":
    raise SystemExit(main())
