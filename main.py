#!/usr/bin/env python3
# System Designer and Developer: Md Shahanur Islam Shagor
# Project: UVA GPS Denied Navigation in Dynamic Environments
# Technology: C++, Python, Go, CMake

"""Project launcher for the C++ drone node, Go control plane, and PySide6 dashboard."""

from __future__ import annotations

import argparse
import logging
import os
import shutil
import signal
import subprocess
import sys
import time
import traceback
from logging.handlers import RotatingFileHandler
from dataclasses import dataclass
from pathlib import Path


ROOT = Path(__file__).resolve().parent
LOG_DIR = ROOT / "logs" / "launcher"
logger = logging.getLogger(__name__)


@dataclass
class ManagedProcess:
    name: str
    command: list[str]
    log_path: Path
    process: subprocess.Popen[str] | None = None
    required: bool = True
    log_handle: object | None = None


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Launch the drone swarm stack")
    parser.add_argument("--skip-go", action="store_true", help="skip Go control-plane startup")
    parser.add_argument("--skip-cpp", action="store_true", help="skip C++ drone node startup")
    parser.add_argument("--skip-gui", action="store_true", help="skip PySide6 dashboard startup")
    parser.add_argument("--dry-run", action="store_true", help="print commands without executing them")
    parser.add_argument("--drone-id", type=int, default=1, help="local drone id for drone_node")
    parser.add_argument("--esp32", default="192.168.4.1", help="ESP32 camera IP for drone_node")
    parser.add_argument("--lidar", default="192.168.1.201:2368", help="LiDAR endpoint for drone_node")
    parser.add_argument("--go-port", type=int, default=8080, help="Go control-plane port")
    parser.add_argument("--dashboard-poll-hz", type=int, default=20, help="dashboard polling rate")
    parser.add_argument("--dashboard-ids", default="1,2,3,4,5", help="comma-separated drone ids for dashboard")
    return parser.parse_args()


def find_drone_node() -> Path | None:
    candidates = [
        ROOT / "build-dashboard" / "drone_node.exe",
        ROOT / "build" / "drone_node.exe",
        ROOT / "build-full" / "drone_node.exe",
    ]
    for candidate in candidates:
        if candidate.exists():
            return candidate
    return None


def find_go_launcher() -> list[str] | None:
    go = shutil.which("go")
    if go:
        return [go, "run", "./cmd/control-plane"]

    exe_candidates = [
        ROOT / "build-go" / "control-plane.exe",
        ROOT / "bin" / "control-plane.exe",
        ROOT / "control-plane.exe",
    ]
    for candidate in exe_candidates:
        if candidate.exists():
            return [str(candidate)]
    return None


def open_log(path: Path):
    path.parent.mkdir(parents=True, exist_ok=True)
    return path.open("w", encoding="utf-8")


def start_process(spec: ManagedProcess, env: dict[str, str] | None = None) -> None:
    logger.info("starting process name=%s command=%s", spec.name, spec.command)
    log_handle = open_log(spec.log_path)
    spec.log_handle = log_handle
    creationflags = 0
    if os.name == "nt":
        creationflags = subprocess.CREATE_NEW_PROCESS_GROUP

    try:
        spec.process = subprocess.Popen(
            spec.command,
            cwd=str(ROOT),
            stdout=log_handle,
            stderr=subprocess.STDOUT,
            text=True,
            env=env,
            creationflags=creationflags,
        )
    except Exception:
        try:
            log_handle.write(traceback.format_exc())
            log_handle.flush()
        except Exception:
            pass
        try:
            log_handle.close()
        except Exception:
            pass
        spec.log_handle = None
        raise


def terminate_process(spec: ManagedProcess) -> None:
    logger.info("terminating process name=%s", spec.name)
    if spec.process is not None and spec.process.poll() is None:
        try:
            if os.name == "nt":
                spec.process.send_signal(signal.CTRL_BREAK_EVENT)
                try:
                    spec.process.wait(timeout=3)
                except subprocess.TimeoutExpired:
                    spec.process.kill()
            else:
                spec.process.terminate()
                try:
                    spec.process.wait(timeout=3)
                except subprocess.TimeoutExpired:
                    spec.process.kill()
        except Exception:
            pass

    if spec.log_handle is not None:
        try:
            spec.log_handle.close()
        except Exception:
            pass
        spec.log_handle = None


def print_plan(processes: list[ManagedProcess]) -> None:
    logger.info("printing startup plan count=%s", len(processes))
    print("Startup plan:")
    for spec in processes:
        print(f"  - {spec.name}: {' '.join(spec.command)}")
        print(f"    log: {spec.log_path}")


def main() -> int:
    try:
        LOG_DIR.mkdir(parents=True, exist_ok=True)
        root = logging.getLogger()
        root.setLevel(logging.DEBUG)
        root.handlers.clear()
        formatter = logging.Formatter("%(asctime)s %(levelname)s %(name)s: %(message)s")
        console = logging.StreamHandler(sys.stdout)
        console.setLevel(logging.INFO)
        console.setFormatter(formatter)
        file_handler = RotatingFileHandler(
            LOG_DIR / "launcher.log",
            maxBytes=2 * 1024 * 1024,
            backupCount=5,
            encoding="utf-8",
        )
        file_handler.setLevel(logging.DEBUG)
        file_handler.setFormatter(formatter)
        root.addHandler(console)
        root.addHandler(file_handler)
        logger.info("launcher logging initialized file=%s", LOG_DIR / "launcher.log")

        args = parse_args()
        logger.info("launcher args=%s", args)

        processes: list[ManagedProcess] = []
        go_url = f"http://127.0.0.1:{args.go_port}"

        if not args.skip_go:
            go_cmd = find_go_launcher()
            if go_cmd is None:
                print("Go control-plane not found. Install Go or provide a built control-plane.exe.", file=sys.stderr)
            else:
                processes.append(
                    ManagedProcess(
                        name="go-control-plane",
                        command=go_cmd,
                        log_path=LOG_DIR / "go-control-plane.log",
                        required=False,
                    )
                )

        if not args.skip_cpp:
            drone_node = find_drone_node()
            if drone_node is None:
                print("C++ drone_node.exe not found. Build target `drone_node` first.", file=sys.stderr)
            else:
                processes.append(
                    ManagedProcess(
                        name="cpp-drone-node",
                        command=[
                            str(drone_node),
                            f"--id={args.drone_id}",
                            f"--esp32={args.esp32}",
                            f"--lidar={args.lidar}",
                        ],
                        log_path=LOG_DIR / "cpp-drone-node.log",
                    )
                )

        if not args.skip_gui:
            dashboard_cmd = [
                sys.executable,
                str(ROOT / "gui" / "dashboard.py"),
                "--ids",
                args.dashboard_ids,
                "--poll-hz",
                str(args.dashboard_poll_hz),
            ]
            if not args.skip_go and any(p.name == "go-control-plane" for p in processes):
                dashboard_cmd.extend(["--backend-url", go_url])
            processes.append(
                ManagedProcess(
                    name="python-dashboard",
                    command=dashboard_cmd,
                    log_path=LOG_DIR / "python-dashboard.log",
                )
            )

        if not processes:
            print("Nothing to launch. All components are disabled or unavailable.", file=sys.stderr)
            return 1

        print_plan(processes)
        if args.dry_run:
            return 0

        env = os.environ.copy()
        env.setdefault("PYTHONUNBUFFERED", "1")

        if any(p.name == "go-control-plane" for p in processes):
            env["DRONE_SWARM_ADDR"] = f":{args.go_port}"

        started: list[ManagedProcess] = []
        try:
            for spec in processes:
                try:
                    start_process(spec, env=env)
                except Exception as exc:
                    print(f"Failed to start {spec.name}: {exc}", file=sys.stderr)
                    return 1 if spec.required else 0
                started.append(spec)
                print(f"Started {spec.name} (pid={spec.process.pid if spec.process else '?'})")
                if spec.name == "go-control-plane":
                    time.sleep(1.2)

            print("Project stack is running. Press Ctrl+C to stop all processes.")
            while True:
                for spec in started:
                    if spec.process is None:
                        continue
                    code = spec.process.poll()
                    if code is not None:
                        print(f"{spec.name} exited with code {code}. Shutting down the remaining stack.")
                        return code if spec.required else 0
                time.sleep(0.5)
        except KeyboardInterrupt:
            print("Shutdown requested. Stopping all processes...")
            return 0
        finally:
            for spec in reversed(started):
                terminate_process(spec)
    except Exception as exc:
        print(f"launcher failed: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
