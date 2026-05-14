#!/usr/bin/env python3
"""Run repeatable local validation for Python, Go, and C++ paths."""

from __future__ import annotations

import argparse
import os
import platform
import re
import shutil
import subprocess
import sys
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[1]
BUILD_DIR = "build-local-validate"
WINDOWS_VCPKG_CANDIDATES = (
    Path(os.environ.get("VCPKG_ROOT", "")) if os.environ.get("VCPKG_ROOT") else None,
    Path(r"D:\tools\vcpkg-full"),
    Path(r"C:\tools\vcpkg-full"),
    Path(r"C:\vcpkg"),
)


def existing_vcpkg_root() -> Path | None:
    for candidate in WINDOWS_VCPKG_CANDIDATES:
        if candidate and candidate.exists():
            return candidate
    return None


def detect_toolchain_file() -> Path | None:
    vcpkg_root = existing_vcpkg_root()
    if not vcpkg_root:
        return None
    toolchain = vcpkg_root / "scripts" / "buildsystems" / "vcpkg.cmake"
    if toolchain.exists():
        return toolchain
    return None


def print_eigen_install_instructions(toolchain: Path | None) -> None:
    print("FAIL: Eigen3 headers are missing or not discoverable by CMake.")
    print("Install Eigen3, then rerun `python scripts/local_validate.py`.")
    print("Windows with vcpkg:")
    if existing_vcpkg_root():
        vcpkg_root = existing_vcpkg_root()
        print(f"  set VCPKG_ROOT={vcpkg_root}")
        print(f"  {vcpkg_root}\\vcpkg.exe install eigen3")
        if toolchain:
            print(f"  python scripts/local_validate.py --toolchain \"{toolchain}\"")
    else:
        print("  git clone https://github.com/microsoft/vcpkg D:\\tools\\vcpkg-full")
        print("  D:\\tools\\vcpkg-full\\bootstrap-vcpkg.bat")
        print("  set VCPKG_ROOT=D:\\tools\\vcpkg-full")
        print("  %VCPKG_ROOT%\\vcpkg.exe install eigen3")
    print("Ubuntu/Debian:")
    print("  sudo apt-get update && sudo apt-get install -y libeigen3-dev")
    print("Manual Eigen install:")
    print("  Download Eigen, then point EIGEN3_ROOT or EIGEN3_INCLUDE_DIR at the folder containing Eigen/Core")


def safe_write(text: str, *, stream_name: str = "stdout") -> None:
    stream = getattr(sys, stream_name)
    encoding = getattr(stream, "encoding", None) or "utf-8"
    sanitized = text.encode(encoding, errors="replace").decode(encoding, errors="replace")
    stream.write(sanitized)
    stream.flush()


def print_missing_cmake_instructions() -> None:
    print("FAIL: `cmake` was not found on PATH.")
    print("Install CMake, then rerun `python scripts/local_validate.py`.")
    print("Windows:")
    print("  winget install Kitware.CMake")
    print("  or choco install cmake --installargs 'ADD_CMAKE_TO_PATH=System'")
    print("Linux:")
    print("  Ubuntu/Debian: sudo apt-get update && sudo apt-get install -y cmake")
    print("  Fedora: sudo dnf install -y cmake")
    print("  Arch: sudo pacman -S cmake")


def run_step(title: str, command: list[str], env: dict[str, str] | None = None) -> subprocess.CompletedProcess[str]:
    print(f"\n==> {title}")
    print(" ".join(command))
    completed = subprocess.run(
        command,
        cwd=REPO_ROOT,
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
    parser.add_argument("--toolchain", help="Explicit CMake toolchain file path")
    parser.add_argument("--skip-cpp", action="store_true", help="Skip C++ configure/build/test steps")
    return parser.parse_args()


def cmake_configure_command(toolchain: str | None) -> list[str]:
    command = ["cmake", "-S", ".", "-B", BUILD_DIR, "-DBUILD_TESTS=ON"]
    if toolchain:
        command.append(f"-DCMAKE_TOOLCHAIN_FILE={toolchain}")
    return command


def with_runtime_path(env: dict[str, str], toolchain: Path | None) -> dict[str, str]:
    updated = dict(env)
    path_entries: list[str] = []
    if toolchain:
        vcpkg_root = toolchain.parents[2]
        for candidate in (
            vcpkg_root / "installed" / "x64-windows" / "bin",
            vcpkg_root / "installed" / "x64-windows" / "debug" / "bin",
        ):
            if candidate.exists():
                path_entries.append(str(candidate))
    if path_entries:
        updated["PATH"] = os.pathsep.join(path_entries + [updated.get("PATH", "")])
    return updated


def report_cmake_failure(completed: subprocess.CompletedProcess[str], toolchain: Path | None) -> int:
    combined = "\n".join(part for part in (completed.stdout, completed.stderr) if part)
    if re.search(r"Could not find a package configuration file provided by \"Eigen3\"|Eigen3 headers were not found", combined):
        print_eigen_install_instructions(toolchain)
        return completed.returncode or 1
    print("FAIL: CMake configure failed.")
    print("Next command to retry after fixing dependencies:")
    command = " ".join(cmake_configure_command(str(toolchain) if toolchain else None))
    print(f"  {command}")
    return completed.returncode or 1


def main() -> int:
    args = parse_args()
    base_env = dict(os.environ)
    if shutil.which("cmake") is None:
        print_missing_cmake_instructions()
        return 1
    if shutil.which("ctest") is None:
        print("FAIL: `ctest` was not found on PATH. It is normally installed with CMake.")
        print("Reinstall CMake and ensure its bin directory is on PATH, then rerun validation.")
        return 1
    if shutil.which("go") is None:
        print("FAIL: `go` was not found on PATH. Install Go from https://go.dev/dl/ and rerun validation.")
        return 1

    toolchain_arg = Path(args.toolchain).resolve() if args.toolchain else detect_toolchain_file()
    if args.toolchain and not toolchain_arg.exists():
        print(f"FAIL: requested toolchain file does not exist: {toolchain_arg}")
        return 1
    if toolchain_arg:
        print(f"Using CMake toolchain: {toolchain_arg}")
    elif platform.system() == "Windows":
        print("No vcpkg toolchain was auto-detected. CMake will rely on default package search paths.")

    python = sys.executable
    completed = run_step(
        "Python syntax check",
        [python, "-m", "py_compile", "gui/dashboard.py", "gui/backend_status.py", "main.py"],
        env=base_env,
    )
    if completed.returncode != 0:
        return completed.returncode
    completed = run_step(
        "Python unit tests",
        [python, "-m", "unittest", "tests.test_dashboard_backend_status"],
        env=base_env,
    )
    if completed.returncode != 0:
        return completed.returncode
    completed = run_step(
        "Go test suite",
        ["go", "test", "./..."],
        env=base_env,
    )
    if completed.returncode != 0:
        return completed.returncode

    if args.skip_cpp:
        print("WARNING: --skip-cpp was requested. C++ configure/build/ctest steps were not run.")
        print("PASS: Python and Go validation completed with C++ checks skipped explicitly.")
        return 0

    cpp_env = with_runtime_path(base_env, toolchain_arg)
    completed = run_step(
        "CMake configure",
        cmake_configure_command(str(toolchain_arg) if toolchain_arg else None),
        env=cpp_env,
    )
    if completed.returncode != 0:
        return report_cmake_failure(completed, toolchain_arg)

    completed = run_step(
        "CMake build",
        ["cmake", "--build", BUILD_DIR, "--config", "Release"],
        env=cpp_env,
    )
    if completed.returncode != 0:
        print("FAIL: CMake build failed.")
        print(f"Next command to retry after fixing dependencies:")
        print(f"  cmake --build {BUILD_DIR} --config Release")
        return completed.returncode

    completed = run_step(
        "CTest",
        ["ctest", "--test-dir", BUILD_DIR, "--output-on-failure", "-C", "Release"],
        env=cpp_env,
    )
    if completed.returncode != 0:
        return completed.returncode
    print("\nPASS: local validation completed successfully.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
