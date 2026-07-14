#!/usr/bin/env python3
"""Verify clang-format compliance for project-owned C/C++ sources."""

from __future__ import annotations

import shutil
import subprocess
import sys
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[1]
FORMAT_EXTENSIONS = {".c", ".h", ".cpp", ".hpp"}
EXCLUDE_PARTS = {"build", "third_party", ".git", "__pycache__"}
SCAN_DIRS = ("include", "src", "tests")


def iter_source_files() -> list[Path]:
    files: list[Path] = []
    for scan_dir in SCAN_DIRS:
        root = REPO_ROOT / scan_dir
        if not root.exists():
            continue
        for path in root.rglob("*"):
            if path.suffix not in FORMAT_EXTENSIONS:
                continue
            if any(part in EXCLUDE_PARTS for part in path.parts):
                continue
            files.append(path)
    return sorted(files)


def main() -> int:
    clang_format = shutil.which("clang-format")
    if clang_format is None:
        print("FAIL: clang-format was not found on PATH.")
        return 1

    failures: list[str] = []
    for path in iter_source_files():
        completed = subprocess.run(
            [clang_format, "--dry-run", "--Werror", str(path)],
            cwd=REPO_ROOT,
            text=True,
            encoding="utf-8",
            errors="replace",
            capture_output=True,
        )
        if completed.returncode != 0:
            failures.append(str(path.relative_to(REPO_ROOT)))
            if completed.stdout:
                sys.stdout.write(completed.stdout)
            if completed.stderr:
                sys.stderr.write(completed.stderr)

    if failures:
        print("FAIL: clang-format check failed for:")
        for failure in failures:
            print(f"  - {failure}")
        return 1

    print(f"PASS: clang-format verified {len(iter_source_files())} files.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
