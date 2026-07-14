#!/usr/bin/env python3
"""Run clang-tidy over project-owned translation units."""

from __future__ import annotations

import argparse
import json
import os
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
    parser.add_argument(
        "--changed-only",
        action="store_true",
        help="Lint only changed compiled translation units for the current git range.",
    )
    return parser.parse_args()


def collect_compiled_sources(build_dir: Path) -> list[Path]:
    compile_commands = build_dir / "compile_commands.json"
    commands = json.loads(compile_commands.read_text(encoding="utf-8"))

    allowed_roots = tuple(
        (REPO_ROOT / directory).resolve() for directory in DEFAULT_DIRS
    )
    files: list[Path] = []
    seen: set[Path] = set()

    for entry in commands:
        candidate = Path(entry["file"])
        if not candidate.is_absolute():
            candidate = (Path(entry["directory"]) / candidate).resolve()
        else:
            candidate = candidate.resolve()

        if candidate in seen:
            continue

        if not any(root in candidate.parents for root in allowed_roots):
            continue

        seen.add(candidate)
        files.append(candidate)

    return sorted(files)


def git_stdout(*args: str) -> str:
    completed = subprocess.run(
        ["git", *args],
        cwd=REPO_ROOT,
        check=True,
        capture_output=True,
        text=True,
        encoding="utf-8",
    )
    return completed.stdout.strip()


def resolve_diff_range() -> str | None:
    event_name = os.environ.get("GITHUB_EVENT_NAME", "")
    base_ref = os.environ.get("GITHUB_BASE_REF", "")

    try:
        if event_name == "pull_request" and base_ref:
            remote_base = f"origin/{base_ref}"
            merge_base = git_stdout("merge-base", "HEAD", remote_base)
            return f"{merge_base}..HEAD"

        git_stdout("rev-parse", "HEAD^")
        return "HEAD^..HEAD"
    except subprocess.CalledProcessError:
        return None


def collect_changed_sources(compiled_sources: list[Path]) -> list[Path]:
    diff_range = resolve_diff_range()
    if diff_range is None:
        return compiled_sources

    try:
        changed_output = git_stdout(
            "diff",
            "--name-only",
            "--diff-filter=ACMR",
            diff_range,
            "--",
            *DEFAULT_DIRS,
        )
    except subprocess.CalledProcessError:
        return compiled_sources

    if not changed_output:
        return []

    changed_paths = {
        (REPO_ROOT / line).resolve()
        for line in changed_output.splitlines()
        if line.endswith((".c", ".cc", ".cpp", ".cxx"))
    }
    return [path for path in compiled_sources if path in changed_paths]


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

    files = collect_compiled_sources(build_dir)
    if args.changed_only:
        files = collect_changed_sources(files)

    if not files:
        if args.changed_only:
            print("PASS: no changed compiled C++ translation units require clang-tidy.")
        else:
            print("PASS: no compiled C++ translation units found for clang-tidy.")
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
