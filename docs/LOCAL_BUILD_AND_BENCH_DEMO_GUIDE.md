# Local Build And Bench Demo Guide

## Purpose

Use this guide to:

- install the local C++ build toolchain on Windows or Linux
- build `drone_node` and the C++ tests
- run the combined Python, Go, and C++ validation flow
- run repeatable bench-demo telemetry smoke tests against the Go control-plane

Current status on this repository:

- Python validation path exists and is locally scriptable with `scripts/local_validate.py`
- Go backend validation exists and is locally scriptable with `scripts/local_validate.py`
- C++ validation is now wired into the same script and no longer silently skips when `cmake` is missing
- Bench-demo telemetry validation is scriptable with `scripts/telemetry_smoke_test.py` and `scripts/production_telemetry_smoke_test.py`

## 1. Install CMake

### Windows

Preferred options:

```powershell
winget install Kitware.CMake
```

or:

```powershell
choco install cmake --installargs 'ADD_CMAKE_TO_PATH=System'
```

Verify:

```powershell
cmake --version
ctest --version
```

### Linux

Ubuntu/Debian:

```bash
sudo apt-get update
sudo apt-get install -y cmake
```

Fedora:

```bash
sudo dnf install -y cmake
```

Arch:

```bash
sudo pacman -S cmake
```

Verify:

```bash
cmake --version
ctest --version
```

## 2. Install Compiler / Toolchain

### Windows

Install Visual Studio 2022 Build Tools or full Visual Studio with:

- `Desktop development with C++`
- MSVC v143 toolset
- Windows 10/11 SDK

Recommended package managers:

```powershell
winget install Microsoft.VisualStudio.2022.BuildTools
```

If you use Ninja instead of Visual Studio generators:

```powershell
winget install Ninja-build.Ninja
```

### Linux

Ubuntu/Debian:

```bash
sudo apt-get update
sudo apt-get install -y build-essential g++ gcc pkg-config ninja-build
```

Fedora:

```bash
sudo dnf groupinstall -y "Development Tools"
sudo dnf install -y gcc-c++ pkgconf-pkg-config ninja-build
```

Arch:

```bash
sudo pacman -S base-devel gcc pkgconf ninja
```

## 3. Install Eigen / OpenCV / PCL / spdlog

This project requires:

- Eigen3
- OpenCV
- PCL
- spdlog

### Windows

The top-level [CMakeLists.txt](/d:/Final%20Project/drone_swarm/CMakeLists.txt) is already prepared to use `vcpkg` on Windows when `VCPKG_ROOT` is set.

1. Install `vcpkg`

```powershell
git clone https://github.com/microsoft/vcpkg D:\tools\vcpkg-full
D:\tools\vcpkg-full\bootstrap-vcpkg.bat
```

2. Set the environment variable:

```powershell
$env:VCPKG_ROOT="D:\tools\vcpkg-full"
```

3. Install packages:

```powershell
& "$env:VCPKG_ROOT\vcpkg.exe" install eigen3 opencv4 pcl spdlog
```

4. Configure with explicit toolchain if needed:

```powershell
cmake -S . -B build-local -DBUILD_TESTS=ON -DCMAKE_TOOLCHAIN_FILE="$env:VCPKG_ROOT\scripts\buildsystems\vcpkg.cmake"
```

5. If you use the validator, it can auto-detect the toolchain from `VCPKG_ROOT` or you can pass it directly:

```powershell
python scripts/local_validate.py --toolchain "$env:VCPKG_ROOT\scripts\buildsystems\vcpkg.cmake"
```

Optional:

- `pybind11` is auto-fetched by CMake if it is not already installed
- Fast-DDS is optional
- TensorRT is optional

### Linux

Ubuntu/Debian:

```bash
sudo apt-get update
sudo apt-get install -y \
  libeigen3-dev \
  libopencv-dev \
  libpcl-dev \
  libspdlog-dev
```

Configure afterward with:

```bash
cmake -S . -B build-local -G Ninja -DBUILD_TESTS=ON -DCMAKE_BUILD_TYPE=Release
```

Fedora:

```bash
sudo dnf install -y \
  eigen3-devel \
  opencv-devel \
  pcl-devel \
  spdlog-devel
```

Arch:

```bash
sudo pacman -S \
  eigen \
  opencv \
  pcl \
  spdlog
```

### Manual Eigen install

If you already unpacked Eigen manually instead of using a package manager, point CMake at the directory containing `Eigen/Core`.

Typical layout:

```text
C:\deps\eigen-3.4.0\include\eigen3\Eigen\Core
```

Windows PowerShell:

```powershell
$env:EIGEN3_INCLUDE_DIR="C:\deps\eigen-3.4.0\include\eigen3"
cmake -S . -B build-local -DBUILD_TESTS=ON
```

Linux:

```bash
export EIGEN3_INCLUDE_DIR=/opt/eigen-3.4.0/include/eigen3
cmake -S . -B build-local -G Ninja -DBUILD_TESTS=ON -DCMAKE_BUILD_TYPE=Release
```

Alternative environment variable:

```text
EIGEN3_ROOT
```

The project now supports an include-only Eigen fallback, so a full `Eigen3Config.cmake` package is not strictly required as long as the headers are reachable.

## 4. Configure The Build

### Windows

Visual Studio generator:

```powershell
cmake -S . -B build-local -DBUILD_TESTS=ON
```

Ninja generator:

```powershell
cmake -S . -B build-local -G Ninja -DBUILD_TESTS=ON
```

### Linux

Ninja is recommended:

```bash
cmake -S . -B build-local -G Ninja -DBUILD_TESTS=ON -DCMAKE_BUILD_TYPE=Release
```

## 5. Build `drone_node`

Windows:

```powershell
cmake --build build-local --config Release --target drone_node
```

Linux:

```bash
cmake --build build-local --target drone_node
```

## 6. Build The Tests

Windows:

```powershell
cmake --build build-local --config Release
```

Linux:

```bash
cmake --build build-local
```

The C++ test executables are declared in [tests/CMakeLists.txt](/d:/Final%20Project/drone_swarm/tests/CMakeLists.txt).

## 7. Run `ctest`

Windows:

```powershell
ctest --test-dir build-local --output-on-failure -C Release
```

Linux:

```bash
ctest --test-dir build-local --output-on-failure
```

## 8. One-Command Local Validation

Run:

```powershell
python scripts/local_validate.py
```

The script runs this exact sequence:

1. `python -m py_compile gui/dashboard.py gui/backend_status.py main.py`
2. `python -m unittest tests.test_dashboard_backend_status`
3. `go test ./...`
4. `cmake -S . -B build-local-validate -DBUILD_TESTS=ON`
5. `cmake --build build-local-validate --config Release`
6. `ctest --test-dir build-local-validate --output-on-failure`

Behavior notes:

- if `cmake` is missing, the script prints install instructions and exits non-zero
- C++ validation is not skipped
- `ctest` is also required and hard-fails if missing
- if `VCPKG_ROOT` is set and contains `scripts/buildsystems/vcpkg.cmake`, the script auto-adds that toolchain
- you can override toolchain detection with `--toolchain <path>`
- `--skip-cpp` exists, but only for explicit temporary bypasses and prints a warning
- if Eigen is missing, the script prints exact Windows `vcpkg` and Ubuntu `apt` install commands

## 9. Bench Demo Command Flow

Start the backend in production mode so no fake simulation fleet is seeded:

### Windows PowerShell

```powershell
$env:DRONE_BACKEND_MODE="production"
$env:DRONE_BACKEND_SIMULATION_ENABLED="false"
go run ./cmd/control-plane
```

### Linux

```bash
DRONE_BACKEND_MODE=production DRONE_BACKEND_SIMULATION_ENABLED=false go run ./cmd/control-plane
```

In another terminal:

```powershell
python scripts/telemetry_smoke_test.py --backend-url http://127.0.0.1:8080
python scripts/production_telemetry_smoke_test.py --backend-url http://127.0.0.1:8080
```

Expected result:

- simulation-tagged telemetry appears in `GET /api/v1/fleet` and stays marked `source=simulation`
- LiDAR points are truncated to the backend cap of `256`
- production backend reports `backend_mode=production`
- production backend reports `simulation_enabled=false`
- no seeded fake simulation fleet is present in production mode
- unavailable-source telemetry does not increase `real_drone_count`

## 10. Troubleshooting Common CMake Errors

### `cmake` or `ctest` is not recognized

Cause:

- CMake is not installed or not on `PATH`

Fix:

- reinstall CMake
- reopen the terminal
- verify with `cmake --version` and `ctest --version`

### `Could not find Eigen3`

Cause:

- Eigen is not installed where CMake can discover it

Fix:

- Windows: install `eigen3` through `vcpkg` and set `VCPKG_ROOT`
- Linux: install `libeigen3-dev` or distro equivalent
- manual install: set `EIGEN3_INCLUDE_DIR` or `EIGEN3_ROOT` to the directory containing `Eigen/Core`

### `Could not find OpenCV`

Cause:

- OpenCV development headers/libraries are missing

Fix:

- Windows: install `opencv4` through `vcpkg`
- Linux: install `libopencv-dev` or distro equivalent

### `Could not find PCL`

Cause:

- PCL development package is missing

Fix:

- Windows: install `pcl` through `vcpkg`
- Linux: install `libpcl-dev` or distro equivalent

### Fixing stale vcpkg root / PCL link mismatch

Symptom:

- CMake is invoked with `D:\tools\vcpkg-full`, but the build tries to link libraries from `C:\tools\vcpkg-full`, for example `C:\tools\vcpkg-full\installed\x64-windows\lib\pcl_io.lib`

Cause:

- an existing `CMakeCache.txt` was created with a different vcpkg root and still contains stale package paths

Fix from the repository root:

```powershell
Remove-Item -Recurse -Force .\build-local-validate -ErrorAction SilentlyContinue
Remove-Item -Recurse -Force .\build-edge -ErrorAction SilentlyContinue
$env:VCPKG_ROOT="D:\tools\vcpkg-full"
```

Optional persistent user environment:

```powershell
setx VCPKG_ROOT "D:\tools\vcpkg-full"
```

Verify the expected files exist:

```powershell
Test-Path "D:\tools\vcpkg-full\installed\x64-windows\lib\pcl_io.lib"
Test-Path "D:\tools\vcpkg-full\installed\x64-windows\include\eigen3\Eigen\Core"
```

If dependencies are missing:

```powershell
D:\tools\vcpkg-full\vcpkg.exe install pcl:x64-windows opencv:x64-windows eigen3:x64-windows spdlog:x64-windows fmt:x64-windows
```

Rerun with an explicit toolchain:

```powershell
cmake -S . -B build-local-validate -DBUILD_TESTS=ON -DCMAKE_TOOLCHAIN_FILE=D:/tools/vcpkg-full/scripts/buildsystems/vcpkg.cmake
cmake --build build-local-validate --config Release
ctest --test-dir build-local-validate --output-on-failure -C Release
```

The validator also checks for this mismatch:

```powershell
python scripts/local_validate.py --toolchain "D:\tools\vcpkg-full\scripts\buildsystems\vcpkg.cmake"
```

### `Could not find spdlog`

Cause:

- `spdlog` development package is missing

Fix:

- Windows: install `spdlog` through `vcpkg`
- Linux: install `libspdlog-dev` or distro equivalent

### Windows generator mismatch or missing MSVC environment

Cause:

- CMake found the wrong generator or no valid C++ compiler

Fix:

- open the `x64 Native Tools Command Prompt for VS 2022`
- or use a Visual Studio developer PowerShell
- rerun `cmake -S . -B build-local -DBUILD_TESTS=ON`

### `FetchContent` cannot download `pybind11` or `googletest`

Cause:

- network access is blocked during configure

Fix:

- allow outbound network access during configure
- or preinstall those dependencies in your package manager environment

## 11. Readiness Level

Current readiness level:

- local software validation: ready
- backend telemetry demo: ready
- honest production-mode no-hardware smoke test: ready
- real hardware bench acceptance: still blocked on physical sensor and replay evidence
