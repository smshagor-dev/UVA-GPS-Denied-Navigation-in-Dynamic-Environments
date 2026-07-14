#!/usr/bin/env python3
"""Validate installed package contents and downstream CMake consumption."""

from __future__ import annotations

import argparse
import json
import shutil
import subprocess
import tempfile
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[1]


CONSUMER_CMAKELISTS = """\
cmake_minimum_required(VERSION 3.24)
project(DroneSwarmConsumer LANGUAGES CXX)
find_package(DroneSwarmSensorFusion CONFIG REQUIRED)
add_executable(consumer main.cpp)
target_link_libraries(consumer PRIVATE DroneSwarm::sensor_fusion_core)
"""

CONSUMER_MAIN = """\
#include <autonomy/DecisionEngine.hpp>
#include <safety/SafetyManager.hpp>

int main() {
    drone::autonomy::DecisionEngine engine;
    drone::autonomy::DecisionContext decision_context{};
    auto command = engine.update(decision_context);

    drone::safety::SafetyManager safety_manager;
    drone::safety::SafetyContext safety_context{};
    auto safety = safety_manager.evaluate(safety_context);
    safety_manager.enforce(safety, command, decision_context.pose);
    return 0;
}
"""


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--install-prefix", required=True, help="Installation prefix to validate"
    )
    parser.add_argument("--manifest-out", help="Optional JSON manifest output path")
    return parser.parse_args()


def collect_manifest(prefix: Path) -> dict[str, object]:
    files = sorted(
        str(path.relative_to(prefix)).replace("\\", "/")
        for path in prefix.rglob("*")
        if path.is_file()
    )
    return {
        "install_prefix": str(prefix),
        "files": files,
    }


def main() -> int:
    args = parse_args()
    prefix = Path(args.install_prefix).resolve()
    if not prefix.exists():
        print(f"FAIL: install prefix does not exist: {prefix}")
        return 1

    required_paths = [
        prefix / "include" / "autonomy" / "DecisionEngine.hpp",
        prefix
        / "lib"
        / "cmake"
        / "DroneSwarmSensorFusion"
        / "DroneSwarmSensorFusionConfig.cmake",
    ]
    for required in required_paths:
        if not required.exists():
            print(f"FAIL: required installed path missing: {required}")
            return 1

    with tempfile.TemporaryDirectory(prefix="drone-swarm-consumer-") as temp_dir_text:
        temp_dir = Path(temp_dir_text)
        (temp_dir / "CMakeLists.txt").write_text(CONSUMER_CMAKELISTS, encoding="utf-8")
        (temp_dir / "main.cpp").write_text(CONSUMER_MAIN, encoding="utf-8")

        configure = subprocess.run(
            [
                "cmake",
                "-S",
                str(temp_dir),
                "-B",
                str(temp_dir / "build"),
                "-G",
                "Ninja",
                f"-DCMAKE_PREFIX_PATH={prefix}",
            ],
            cwd=REPO_ROOT,
        )
        if configure.returncode != 0:
            return configure.returncode

        build = subprocess.run(
            ["cmake", "--build", str(temp_dir / "build")], cwd=REPO_ROOT
        )
        if build.returncode != 0:
            return build.returncode

    if args.manifest_out:
        manifest_path = Path(args.manifest_out)
        manifest_path.parent.mkdir(parents=True, exist_ok=True)
        manifest_path.write_text(
            json.dumps(collect_manifest(prefix), indent=2), encoding="utf-8"
        )

    print(f"PASS: install tree and downstream consumer validated for {prefix}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
