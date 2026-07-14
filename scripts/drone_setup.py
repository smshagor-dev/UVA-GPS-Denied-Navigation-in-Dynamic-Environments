#!/usr/bin/env python3
# System Designer and Developer: Md Shahanur Islam Shagor
# Project: UVA GPS Denied Navigation in Dynamic Environments
# Technology: C++, Python, Go, CMake

# 
# drone_setup.py    Automated Build / Flash / Environment Setup Script
# Drone Swarm Sensor Fusion  |  Phase 1  Build System
#
# Usage:
#   python3 scripts/drone_setup.py --help
#   python3 scripts/drone_setup.py setup          # install all deps
#   python3 scripts/drone_setup.py build          # cmake + make
#   python3 scripts/drone_setup.py build --clean  # clean rebuild
#   python3 scripts/drone_setup.py flash          # flash ESP32-CAM
#   python3 scripts/drone_setup.py run --id=1     # launch drone_node
#   python3 scripts/drone_setup.py gui            # launch PySide6 dashboard
#   python3 scripts/drone_setup.py all            # full pipeline
# 
from __future__ import annotations

import argparse
import os
import platform
import shutil
import subprocess
import sys
import time
from pathlib import Path
from typing import Optional

#  ANSI colors â”€
class C:
    RED    = "\033[91m"
    GREEN  = "\033[92m"
    YELLOW = "\033[93m"
    CYAN   = "\033[96m"
    BOLD   = "\033[1m"
    RESET  = "\033[0m"

def ok(msg: str)   -> None: print(f"{C.GREEN}  âœ”  {C.RESET}{msg}")
def err(msg: str)  -> None: print(f"{C.RED}  âœ˜  {C.RESET}{msg}")
def info(msg: str) -> None: print(f"{C.CYAN}  â–¶  {C.RESET}{msg}")
def warn(msg: str) -> None: print(f"{C.YELLOW}  âš   {C.RESET}{msg}")
def hdr(msg: str)  -> None:
    w = 60
    print(f"\n{C.BOLD}{C.CYAN}{'â”€'*w}")
    print(f"  {msg}")
    print(f"{'â”€'*w}{C.RESET}")

#  Paths â”€
ROOT       = Path(__file__).resolve().parent.parent
BUILD_DIR  = ROOT / "build"
FIRMWARE   = ROOT / "firmware" / "esp32_cam"
GUI_SCRIPT = ROOT / "gui" / "dashboard.py"

#  Detect platform 
def detect_platform() -> str:
    machine = platform.machine().lower()
    if "aarch64" in machine:
        # Check for Jetson (presence of tegra files)
        if Path("/etc/nv_tegra_release").exists():
            return "jetson"
        return "rpi"
    return "x86"

PLAT = detect_platform()

# 
def run(cmd: str | list, cwd: Optional[Path] = None,
        check: bool = True, capture: bool = False) -> subprocess.CompletedProcess:
    """Run a shell command with live output or captured output."""
    if isinstance(cmd, str):
        cmd_list = cmd.split()
    else:
        cmd_list = cmd

    info(f"$ {' '.join(cmd_list)}")
    try:
        result = subprocess.run(
            cmd_list,
            cwd=str(cwd or ROOT),
            check=check,
            capture_output=capture,
            text=True,
        )
        return result
    except subprocess.CalledProcessError as e:
        err(f"Command failed (exit {e.returncode}): {' '.join(cmd_list)}")
        if e.stderr:
            print(e.stderr)
        if check:
            sys.exit(1)
        return e

# 
# STEP 1: Environment Setup
# 
def cmd_setup(args: argparse.Namespace) -> None:
    hdr("STEP 1  Environment & Dependency Setup")

    # Python packages
    python_pkgs = [
        "pyside6>=6.6",
        "pyqtgraph>=0.13",
        "PyOpenGL>=3.1",
        "numpy>=1.24",
        "opencv-python>=4.8",
        "pyserial>=3.5",
        "esptool>=4.6",
        "cryptography>=45.0",
        "spdlog",
    ]

    info("Installing Python dependenciesâ€¦")
    run([sys.executable, "-m", "pip", "install", "--upgrade"] + python_pkgs)
    ok("Python packages installed")

    if sys.platform.startswith("linux"):
        _setup_linux(args)

    # Create log directory
    (ROOT / "logs").mkdir(exist_ok=True)
    (ROOT / "models").mkdir(exist_ok=True)
    ok("Directory structure verified")

    _check_dependencies()


def _setup_linux(args: argparse.Namespace) -> None:
    APT_PKGS = [
        "cmake", "ninja-build", "build-essential", "git",
        "libeigen3-dev", "libopencv-dev", "libpcl-dev",
        "libspdlog-dev", "pybind11-dev",
        "python3-pyside6",
        "libssl-dev", "libasio-dev",
    ]

    # Jetson extras
    if PLAT == "jetson":
        APT_PKGS += ["nvidia-jetpack", "tensorrt"]
        warn("TensorRT: ensure JetPack SDK is installed via NVIDIA SDK Manager")

    info("Installing system packages (requires sudo)â€¦")
    run(["sudo", "apt-get", "update", "-qq"])
    run(["sudo", "apt-get", "install", "-y"] + APT_PKGS)
    ok("System packages installed")

    # Fast-DDS
    info("Building Fast-DDS from source (this may take a few minutes)â€¦")
    _install_fastdds()


def _install_fastdds() -> None:
    fastdds_dir = ROOT / ".deps" / "Fast-DDS"
    if fastdds_dir.exists():
        ok("Fast-DDS source already present  skipping clone")
    else:
        fastdds_dir.parent.mkdir(parents=True, exist_ok=True)
        run(["git", "clone", "--depth=1", "--branch", "v2.13.0",
             "https://github.com/eProsima/Fast-DDS.git",
             str(fastdds_dir)])

    build = fastdds_dir / "build"
    build.mkdir(exist_ok=True)
    run(["cmake", "..", "-GNinja",
         "-DCMAKE_BUILD_TYPE=Release",
         "-DCOMPILE_EXAMPLES=OFF"], cwd=build)
    run(["ninja", "-j4"], cwd=build)
    run(["sudo", "ninja", "install"], cwd=build)
    ok("Fast-DDS installed")


def _check_dependencies() -> None:
    hdr("Dependency Check")
    checks = {
        "cmake"       : "cmake --version",
        "g++"         : "g++ --version",
        "python3"     : f"{sys.executable} --version",
        "git"         : "git --version",
        "esptool.py"  : "esptool.py version",
    }
    all_ok = True
    for name, cmd in checks.items():
        r = run(cmd, check=False, capture=True)
        if r.returncode == 0:
            ver = r.stdout.split("\n")[0].strip()
            ok(f"{name:15s}  {ver}")
        else:
            warn(f"{name:15s}  NOT FOUND")
            all_ok = False

    if not all_ok:
        warn("Some dependencies missing  build may fail")

# 
# STEP 2: Build
# 
def cmd_build(args: argparse.Namespace) -> None:
    hdr("STEP 2  CMake Build")

    if args.clean and BUILD_DIR.exists():
        info("Cleaning build directoryâ€¦")
        shutil.rmtree(BUILD_DIR)
        ok("Build directory cleaned")

    BUILD_DIR.mkdir(exist_ok=True)

    build_type = "Debug" if args.debug else "Release"
    cmake_args = [
        "cmake", "..",
        f"-DCMAKE_BUILD_TYPE={build_type}",
        "-GNinja",
        f"-DBUILD_TESTS={'ON' if args.tests else 'OFF'}",
    ]

    if PLAT == "jetson":
        cmake_args += [
            "-DCMAKE_TOOLCHAIN_FILE=cmake/jetson_toolchain.cmake",
        ]

    info(f"Configuring CMake ({build_type})â€¦")
    run(cmake_args, cwd=BUILD_DIR)

    jobs = args.jobs or os.cpu_count() or 4
    info(f"Building with {jobs} parallel jobsâ€¦")
    t0 = time.time()
    run(["ninja", f"-j{jobs}"], cwd=BUILD_DIR)
    elapsed = time.time() - t0
    ok(f"Build complete in {elapsed:.1f}s")

    binary = BUILD_DIR / "drone_node"
    if binary.exists():
        ok(f"Binary: {binary}")
    else:
        err("drone_node binary not found after build!")
        sys.exit(1)

# 
# STEP 3: Flash ESP32-CAM
# 
def cmd_flash(args: argparse.Namespace) -> None:
    hdr("STEP 3  ESP32-CAM Flash")

    port = args.port or _detect_serial_port()
    if not port:
        err("No serial port found. Connect ESP32-CAM and try --port=/dev/ttyUSB0")
        sys.exit(1)

    info(f"Using port: {port}")

    # Check for arduino-cli or esptool
    if shutil.which("arduino-cli"):
        _flash_arduino_cli(port)
    else:
        warn("arduino-cli not found  attempting esptool.py with pre-built binary")
        _flash_esptool(port)


def _detect_serial_port() -> Optional[str]:
    candidates = ["/dev/ttyUSB0", "/dev/ttyUSB1", "/dev/ttyACM0"]
    for p in candidates:
        if Path(p).exists():
            return p
    return None


def _flash_arduino_cli(port: str) -> None:
    info("Installing ESP32 board packageâ€¦")
    run(["arduino-cli", "core", "install", "esp32:esp32@2.0.14"])

    libs = [
        "ArduinoJson@6.21.3",
        "ArduinoOTA",
    ]
    for lib in libs:
        run(["arduino-cli", "lib", "install", lib])

    info("Compiling firmwareâ€¦")
    run(["arduino-cli", "compile",
         "--fqbn", "esp32:esp32:ai_thinker",
         str(FIRMWARE)])

    info(f"Uploading to {port}â€¦")
    run(["arduino-cli", "upload",
         "--fqbn", "esp32:esp32:ai_thinker",
         "--port", port,
         str(FIRMWARE)])
    ok("Firmware flashed successfully")


def _flash_esptool(port: str) -> None:
    firmware_bin = FIRMWARE / "build" / "esp32_cam_firmware.ino.bin"
    if not firmware_bin.exists():
        err(f"Pre-built binary not found at {firmware_bin}")
        err("Please build with Arduino IDE or arduino-cli first")
        sys.exit(1)

    run(["esptool.py",
         "--chip", "esp32",
         "--port", port,
         "--baud", "921600",
         "--before", "default_reset",
         "--after", "hard_reset",
         "write_flash", "-z",
         "--flash_mode", "dio",
         "--flash_freq", "80m",
         "--flash_size", "detect",
         "0x1000", str(firmware_bin)])
    ok("Firmware flashed via esptool.py")

# 
# STEP 4: Run drone_node
# 
def cmd_run(args: argparse.Namespace) -> None:
    hdr("STEP 4  Launch Drone Node")

    binary = BUILD_DIR / "drone_node"
    if not binary.exists():
        err("drone_node not built. Run: python3 scripts/drone_setup.py build")
        sys.exit(1)

    cmd = [str(binary),
           f"--id={args.id}",
           f"--esp32={args.esp32}",
           f"--lidar={args.lidar}",
    ]
    if args.yolo:
        cmd.append(f"--yolo={args.yolo}")

    info(f"Launching: {' '.join(cmd)}")
    os.execv(str(binary), cmd)  # replace process (handles Ctrl-C cleanly)

# 
# STEP 5: Launch GUI
# 
def cmd_gui(args: argparse.Namespace) -> None:
    hdr("STEP 5  PySide6 Dashboard")

    if not GUI_SCRIPT.exists():
        err(f"GUI script not found: {GUI_SCRIPT}")
        sys.exit(1)

    os.execv(sys.executable, [sys.executable, str(GUI_SCRIPT)])

# 
# CLI
# 
def main() -> None:
    parser = argparse.ArgumentParser(
        description="Drone Swarm Sensor Fusion  Automation Script",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  python3 scripts/drone_setup.py setup
  python3 scripts/drone_setup.py build --clean --debug
  python3 scripts/drone_setup.py flash --port=/dev/ttyUSB0
  python3 scripts/drone_setup.py run --id=2 --esp32=192.168.4.2
  python3 scripts/drone_setup.py all
"""
    )

    sub = parser.add_subparsers(dest="command")

    # setup
    sub.add_parser("setup", help="Install dependencies and configure environment")

    # build
    bp = sub.add_parser("build", help="Compile C++ project with CMake")
    bp.add_argument("--clean",  action="store_true", help="Clean before build")
    bp.add_argument("--debug",  action="store_true", help="Debug build")
    bp.add_argument("--tests",  action="store_true", help="Build unit tests")
    bp.add_argument("--jobs",   type=int,            help="Parallel jobs (default: nproc)")

    # flash
    fp = sub.add_parser("flash", help="Flash ESP32-CAM firmware")
    fp.add_argument("--port", default=None, help="Serial port (e.g. /dev/ttyUSB0)")

    # run
    rp = sub.add_parser("run", help="Launch drone_node")
    rp.add_argument("--id",    default="1",             help="Drone ID")
    rp.add_argument("--esp32", default="192.168.4.1",   help="ESP32-CAM IP")
    rp.add_argument("--lidar", default="192.168.1.201:2368", help="LiDAR endpoint")
    rp.add_argument("--yolo",  default="models/yolov8n.engine", help="TRT engine path")

    # gui
    sub.add_parser("gui", help="Launch PySide6 dashboard")

    # all
    ap = sub.add_parser("all", help="setup  build  flash  run")
    ap.add_argument("--port",  default=None)
    ap.add_argument("--id",    default="1")
    ap.add_argument("--esp32", default="192.168.4.1")
    ap.add_argument("--lidar", default="192.168.1.201:2368")

    args = parser.parse_args()

    print(f"\n{C.BOLD}{'â•'*60}")
    print(f"   GPS-Denied Drone Swarm  Automation Script")
    print(f"   Platform: {PLAT.upper()}  |  Python {sys.version.split()[0]}")
    print(f"{'â•'*60}{C.RESET}\n")

    dispatch = {
        "setup" : cmd_setup,
        "build" : cmd_build,
        "flash" : cmd_flash,
        "run"   : cmd_run,
        "gui"   : cmd_gui,
    }

    if args.command == "all":
        # synthesise sub-args
        args.clean = False; args.debug = False; args.tests = False; args.jobs = None
        cmd_setup(args)
        cmd_build(args)
        cmd_flash(args)
        # don't auto-run  user should review first
        ok("Pipeline complete. Run manually: python3 scripts/drone_setup.py run")
    elif args.command in dispatch:
        dispatch[args.command](args)
    else:
        parser.print_help()


if __name__ == "__main__":
    main()
# System Designer and Developer: Md Shahanur Islam Shagor
# Project: UVA GPS Denied Navigation in Dynamic Environments
# Technology: C++, Python, Go, CMake
# System Designer and Developer: Md Shahanur Islam Shagor
# Project: UVA GPS Denied Navigation in Dynamic Environments
# Technology: C++, Python, Go, CMake
