#!/usr/bin/env python3
"""Run repeatable local validation for Python, Go, and native C++ paths."""

from __future__ import annotations

import argparse
import json
import os
import platform
import shutil
import subprocess
import sys
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[1]
BUILD_DIR = "build-local-validate"
PY_COMPILE_FILES = [
    "tests/__init__.py",
    "tests/test_dashboard_backend_status.py",
    "main.py",
    "scripts/generate_firmware_manifest.py",
    "scripts/drone_setup.py",
    "scripts/bench_check.py",
    "scripts/local_validate.py",
    "scripts/generate_tls_certs.py",
    "scripts/pre_arm_check.py",
    "scripts/production_telemetry_smoke_test.py",
    "scripts/telemetry_smoke_test.py",
    "gui/dashboard.py",
    "gui/backend_status.py",
]


def safe_write(text: str, *, stream_name: str = "stdout") -> None:
    stream = getattr(sys, stream_name)
    encoding = getattr(stream, "encoding", None) or "utf-8"
    sanitized = text.encode(encoding, errors="replace").decode(encoding, errors="replace")
    stream.write(sanitized)
    stream.flush()


def print_missing_tool(tool: str, hint: str) -> None:
    print(f"FAIL: `{tool}` was not found on PATH.")
    print(hint)


def existing_vcpkg_root() -> Path | None:
    value = os.environ.get("VCPKG_ROOT")
    if not value:
        vcpkg_executable = shutil.which("vcpkg") or shutil.which("vcpkg.exe")
        if vcpkg_executable:
            path = Path(vcpkg_executable).resolve().parent
            if (path / "scripts" / "buildsystems" / "vcpkg.cmake").exists():
                return path

        if platform.system() == "Windows":
            default_root = Path.home() / "vcpkg"
            if (default_root / "scripts" / "buildsystems" / "vcpkg.cmake").exists():
                return default_root
        return None
    path = Path(value).expanduser()
    return path if path.exists() else None


def default_native_preset() -> str | None:
    system = platform.system()
    if system == "Windows":
        return "validation-msvc"
    if system == "Linux":
        return "validation-linux-gcc"
    return None


def run_step(
    title: str,
    command: list[str],
    *,
    env: dict[str, str] | None = None,
    cwd: Path = REPO_ROOT,
) -> subprocess.CompletedProcess[str]:
    print(f"\n==> {title}")
    print(" ".join(command))
    completed = subprocess.run(
        command,
        cwd=cwd,
        env=env,
        text=True,
        encoding="utf-8",
        errors="replace",
        capture_output=True,
    )
    if completed.stdout:
        safe_write(completed.stdout if completed.stdout.endswith("\n") else completed.stdout + "\n")
    if completed.stderr:
        safe_write(
            completed.stderr if completed.stderr.endswith("\n") else completed.stderr + "\n",
            stream_name="stderr",
        )
    return completed


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--preset", help="CMake preset to use for native validation")
    parser.add_argument("--config", default="Release", help="Build configuration for manual native mode")
    parser.add_argument("--toolchain", help="Explicit CMake toolchain file path for manual native mode")
    parser.add_argument("--report", help="Write a JSON validation summary to this path")
    parser.add_argument("--skip-python", action="store_true", help="Skip Python syntax and unit-test steps")
    parser.add_argument("--skip-go", action="store_true", help="Skip Go tests")
    parser.add_argument("--skip-native", action="store_true", help="Skip native CMake build and CTest")
    return parser.parse_args()


def ensure_required_tools(skip_go: bool, skip_native: bool) -> None:
    if shutil.which("python") is None:
        print_missing_tool("python", "Install Python 3.11+ and rerun validation.")
        raise SystemExit(1)
    if not skip_go and shutil.which("go") is None:
        print_missing_tool("go", "Install Go from https://go.dev/dl/ and rerun validation.")
        raise SystemExit(1)
    if not skip_native:
        if shutil.which("cmake") is None:
            print_missing_tool("cmake", "Install CMake and ensure its bin directory is on PATH.")
            raise SystemExit(1)
        if shutil.which("ctest") is None:
            print_missing_tool("ctest", "Install CMake and ensure its bin directory is on PATH.")
            raise SystemExit(1)


def configure_manual_native(
    *,
    config: str,
    toolchain: str | None,
    env: dict[str, str],
) -> tuple[list[str], list[str], list[str]]:
    configure_command = ["cmake", "-S", ".", "-B", BUILD_DIR, "-DDRONE_BUILD_TESTS=ON"]
    if not any(arg.startswith("-DCMAKE_BUILD_TYPE=") for arg in configure_command) and platform.system() != "Windows":
        configure_command.append(f"-DCMAKE_BUILD_TYPE={config}")
    if toolchain:
        configure_command.append(f"-DCMAKE_TOOLCHAIN_FILE={toolchain}")

    build_command = ["cmake", "--build", BUILD_DIR]
    test_command = ["ctest", "--test-dir", BUILD_DIR, "--output-on-failure"]
    if config:
        build_command.extend(["--config", config])
        test_command.extend(["-C", config])
    return configure_command, build_command, test_command


def run_native_validation(
    *,
    preset: str | None,
    config: str,
    toolchain: str | None,
    env: dict[str, str],
) -> list[dict[str, object]]:
    steps: list[dict[str, object]] = []
    if preset:
        commands = [
            ("CMake configure", ["cmake", "--preset", preset]),
            ("CMake build", ["cmake", "--build", "--preset", preset]),
            ("CTest", ["ctest", "--preset", preset]),
        ]
    else:
        commands = [
            ("CMake configure", configure_manual_native(config=config, toolchain=toolchain, env=env)[0]),
            ("CMake build", configure_manual_native(config=config, toolchain=toolchain, env=env)[1]),
            ("CTest", configure_manual_native(config=config, toolchain=toolchain, env=env)[2]),
        ]

    for title, command in commands:
        completed = run_step(title, command, env=env)
        step = {
            "title": title,
            "command": command,
            "returncode": completed.returncode,
        }
        steps.append(step)
        if completed.returncode != 0:
            print(f"FAIL: {title} failed.")
            raise SystemExit(completed.returncode)
    return steps


def record_tool_versions(env: dict[str, str], skip_go: bool, skip_native: bool) -> list[dict[str, object]]:
    steps: list[dict[str, object]] = []
    commands = [("Python version", [sys.executable, "--version"])]
    if not skip_go:
        commands.append(("Go version", ["go", "version"]))
    if not skip_native:
        commands.extend([
            ("CMake version", ["cmake", "--version"]),
            ("CTest version", ["ctest", "--version"]),
        ])

    for title, command in commands:
        completed = run_step(title, command, env=env)
        steps.append({
            "title": title,
            "command": command,
            "returncode": completed.returncode,
        })
        if completed.returncode != 0:
            print(f"FAIL: {title} failed.")
            raise SystemExit(completed.returncode)
    return steps


def maybe_write_report(path_text: str | None, summary: dict[str, object]) -> None:
    if not path_text:
        return
    path = Path(path_text)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(summary, indent=2), encoding="utf-8")
    print(f"Validation report written to: {path}")


def main() -> int:
    args = parse_args()
    ensure_required_tools(args.skip_go, args.skip_native)

    env = dict(os.environ)
    preset = args.preset or (None if args.toolchain else default_native_preset())
    vcpkg_root = existing_vcpkg_root()
    if vcpkg_root:
        env["VCPKG_ROOT"] = str(vcpkg_root)
        print(f"Active VCPKG_ROOT: {vcpkg_root}")
    else:
        print("Active VCPKG_ROOT: <unset>")

    if args.toolchain:
        toolchain_path = Path(args.toolchain).expanduser().resolve()
        if not toolchain_path.exists():
            print(f"FAIL: requested toolchain file does not exist: {toolchain_path}")
            return 1
        toolchain = str(toolchain_path)
        print(f"Active CMAKE_TOOLCHAIN_FILE: {toolchain}")
    else:
        toolchain = None

    if preset:
        print(f"Active native preset: {preset}")

    summary: dict[str, object] = {
        "platform": platform.platform(),
        "preset": preset,
        "config": args.config,
        "steps": [],
    }

    summary["steps"].extend(record_tool_versions(env, args.skip_go, args.skip_native))

    if not args.skip_python:
        completed = run_step(
            "Python syntax check",
            [sys.executable, "-m", "py_compile", *PY_COMPILE_FILES],
            env=env,
        )
        summary["steps"].append({"title": "Python syntax check", "command": ["python", "-m", "py_compile", *PY_COMPILE_FILES], "returncode": completed.returncode})
        if completed.returncode != 0:
            maybe_write_report(args.report, summary)
            return completed.returncode

        completed = run_step(
            "Python unit tests",
            [sys.executable, "-m", "unittest", "tests.test_dashboard_backend_status"],
            env=env,
        )
        summary["steps"].append({"title": "Python unit tests", "command": ["python", "-m", "unittest", "tests.test_dashboard_backend_status"], "returncode": completed.returncode})
        if completed.returncode != 0:
            maybe_write_report(args.report, summary)
            return completed.returncode

    if not args.skip_go:
        completed = run_step("Go test suite", ["go", "test", "./..."], env=env)
        summary["steps"].append({"title": "Go test suite", "command": ["go", "test", "./..."], "returncode": completed.returncode})
        if completed.returncode != 0:
            maybe_write_report(args.report, summary)
            return completed.returncode

    if not args.skip_native:
        try:
            summary["steps"].extend(
                run_native_validation(
                    preset=preset,
                    config=args.config,
                    toolchain=toolchain,
                    env=env,
                )
            )
        except SystemExit as exc:
            maybe_write_report(args.report, summary)
            return int(exc.code)

    summary["status"] = "PASS"
    maybe_write_report(args.report, summary)
    print("\nPASS: local validation completed successfully.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
