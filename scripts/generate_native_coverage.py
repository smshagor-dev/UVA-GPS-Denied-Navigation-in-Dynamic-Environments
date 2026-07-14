#!/usr/bin/env python3
"""Generate GCC native coverage reports with gcovr."""

from __future__ import annotations

import argparse
import shutil
import subprocess
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[1]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--build-dir", required=True, help="CMake build directory")
    parser.add_argument(
        "--output-dir", required=True, help="Coverage artifact directory"
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    gcovr = shutil.which("gcovr")
    if gcovr is None:
        print("FAIL: gcovr was not found on PATH.")
        return 1

    build_dir = Path(args.build_dir).resolve()
    output_dir = Path(args.output_dir).resolve()
    output_dir.mkdir(parents=True, exist_ok=True)

    common = [
        gcovr,
        "--root",
        str(REPO_ROOT),
        "--filter",
        str(REPO_ROOT / "include"),
        "--filter",
        str(REPO_ROOT / "src"),
        "--exclude",
        str(REPO_ROOT / "third_party"),
        "--exclude",
        str(REPO_ROOT / "tests"),
        "--exclude",
        str(REPO_ROOT / "build"),
        "--exclude",
        str(REPO_ROOT / "build-.*"),
        "--object-directory",
        str(build_dir),
        "--gcov-ignore-errors=no_working_dir_found",
    ]

    commands = [
        common + ["--xml", str(output_dir / "coverage.xml")],
        common + ["--json-summary", str(output_dir / "coverage-summary.json")],
        common + ["--html-details", str(output_dir / "index.html")],
        common + ["--txt", str(output_dir / "coverage.txt")],
    ]

    for command in commands:
        completed = subprocess.run(command, cwd=REPO_ROOT)
        if completed.returncode != 0:
            return completed.returncode

    print(f"PASS: native coverage reports written to {output_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
