#!/usr/bin/env python3
"""Fail if tracked Go files are not gofmt-clean."""

from __future__ import annotations

import shutil
import subprocess
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[1]


def main() -> int:
    gofmt = shutil.which("gofmt")
    if gofmt is None:
        print("FAIL: gofmt was not found on PATH.")
        return 1

    listed = subprocess.run(
        ["git", "ls-files", "*.go"],
        cwd=REPO_ROOT,
        text=True,
        capture_output=True,
        check=True,
    )
    files = [line for line in listed.stdout.splitlines() if line]
    if not files:
        print("PASS: no Go files were found.")
        return 0

    completed = subprocess.run(
        [gofmt, "-d", *files],
        cwd=REPO_ROOT,
        text=True,
        capture_output=True,
    )
    if completed.stdout.strip():
        print(completed.stdout)
        print("FAIL: gofmt check failed.")
        return 1

    if completed.returncode != 0:
        if completed.stderr:
            print(completed.stderr)
        return completed.returncode

    print(f"PASS: gofmt verified {len(files)} files.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
