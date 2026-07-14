# Validation Report

Date: 2026-07-14  
Commit SHA: `cb94b22008e217b3f016888d3e98a2ca5aa5af7a`  
Repository state: `UNCOMMITTED WORKTREE`  
Operating system: Microsoft Windows 11 Pro 10.0.26200 64-bit

## Toolchain Summary
- CMake: 4.4.0 portable at `C:\Users\smsha\AppData\Local\Programs\CMakePortable\4.4.0\bin\cmake.exe`
- CTest: 4.4.0 portable at `C:\Users\smsha\AppData\Local\Programs\CMakePortable\4.4.0\bin\ctest.exe`
- Compiler: MSVC 19.44.35228.0 (`cl.exe`)
- MSVC tools root: `C:\BuildTools\VC\Tools\MSVC\14.44.35207`
- Generator: `Visual Studio 17 2022`
- Platform: `x64`
- Windows SDK: `10.0.22621.0`
- vcpkg: `2026-05-27-d5b6777d666efc1a7f491babfcdab37794c1ae3e`
- vcpkg root: `C:\Users\smsha\vcpkg`
- vcpkg triplet: `x64-windows`

## Dependency Versions
- `eigen3:x64-windows` `5.0.1`
- `opencv4:x64-windows` `4.12.0#5`
- `pcl:x64-windows` `1.15.1#1`
- `spdlog:x64-windows` `1.17.0#1`
- `nlohmann-json:x64-windows` `3.12.0#2`

## Toolchain Installation and Discovery
- Existing discovery before installation:
  - `vswhere.exe` was present at `C:\Program Files (x86)\Microsoft Visual Studio\Installer\vswhere.exe`
  - `vswhere -all -products *` returned no installed Visual Studio instances
  - `cl.exe`, `nmake.exe`, `gcc.exe`, `g++.exe`, `clang.exe`, and `clang++.exe` were not discoverable on `PATH`
- Installed toolchain:
  - `winget install --id Microsoft.VisualStudio.2022.BuildTools --exact --accept-source-agreements --accept-package-agreements --override "--quiet --wait --norestart --nocache --installPath C:\BuildTools --add Microsoft.VisualStudio.Workload.VCTools --add Microsoft.VisualStudio.Component.VC.Tools.x86.x64 --add Microsoft.VisualStudio.Component.VC.CMake.Project --add Microsoft.VisualStudio.Component.Windows11SDK.22621 --add Microsoft.VisualStudio.Component.VC.CoreBuildTools --add Microsoft.VisualStudio.Component.Ninja"`
- Verified after installation:
  - `vswhere` reported `C:\BuildTools`
  - `vcvars64.bat` found at `C:\BuildTools\VC\Auxiliary\Build\vcvars64.bat`
  - `cl` reported `19.44.35228 for x64`
  - `nmake /?` succeeded
  - `ninja.exe` discovered at `C:\BuildTools\Common7\IDE\CommonExtensions\Microsoft\CMake\Ninja\ninja.exe`

## Repository Build-System Adjustment
- Added `/bigobj` to the MSVC compile options in [CMakeLists.txt](../../../CMakeLists.txt) so the official validator could compile `src/vio/EKFEstimator.cpp` under the MSVC-backed validation path without changing runtime behavior or algorithms.

## Commands Executed and Results
| Command | Result | Notes |
|---|---|---|
| `cmake.exe --version` | PASS | CMake 4.4.0 confirmed |
| `ctest.exe --version` | PASS | CTest 4.4.0 confirmed |
| `cl` after `vcvars64.bat` | PASS | MSVC 19.44.35228.0 |
| `nmake /?` after `vcvars64.bat` | PASS | NMake 14.44.35228.0 |
| `git clone https://github.com/microsoft/vcpkg %USERPROFILE%\vcpkg` | PASS | local user-level vcpkg clone |
| `%USERPROFILE%\vcpkg\bootstrap-vcpkg.bat -disableMetrics` | PASS | vcpkg bootstrapped successfully |
| `%USERPROFILE%\vcpkg\vcpkg.exe install eigen3:x64-windows opencv4:x64-windows pcl:x64-windows spdlog:x64-windows nlohmann-json:x64-windows` | PASS | required native dependencies resolved |
| `cmake -S . -B build-phase1-clean -G "Visual Studio 17 2022" -A x64 -DBUILD_TESTS=ON -DCMAKE_TOOLCHAIN_FILE=%USERPROFILE%\vcpkg\scripts\buildsystems\vcpkg.cmake` | PASS | clean configure succeeded |
| `cmake --build build-phase1-clean --config Release --parallel` | PASS | Release build succeeded in `237.51` seconds |
| `ctest --test-dir build-phase1-clean -C Release --output-on-failure` | PASS | `112/112` native tests passed |
| `ctest --test-dir build-phase1-clean -C Release --output-junit docs/phase1/ctest-results.xml` | PASS | JUnit written under `build-phase1-clean/docs/phase1/ctest-results.xml` |
| `python scripts/local_validate.py` inside initialized MSVC environment | PASS | full official validation script passed |
| `python -m py_compile tests\__init__.py tests\test_dashboard_backend_status.py main.py scripts\generate_firmware_manifest.py scripts\drone_setup.py scripts\bench_check.py scripts\local_validate.py scripts\generate_tls_certs.py scripts\pre_arm_check.py scripts\production_telemetry_smoke_test.py scripts\telemetry_smoke_test.py gui\dashboard.py gui\backend_status.py` | PASS | maintained Python sources compiled |
| `python -m unittest tests.test_dashboard_backend_status` | PASS | `12` tests passed |
| `go test ./...` | PASS | both Go packages passed |

## Configure Outcome
- Status: PASS
- Clean tree: `build-phase1-clean`
- Generator: `Visual Studio 17 2022`
- Platform: `x64`
- Compiler: `MSVC 19.44.35228.0`
- Windows SDK: `10.0.22621.0`
- Resolved packages:
  - OpenCV `4.12.0`
  - PCL `1.15.1`
  - Eigen3 `5.0.1`
  - spdlog `1.17.0`
  - nlohmann-json `3.12.0`
- Configure warnings:
  - `CMP0144` warning from vcpkg/PCL `FLANN_ROOT`
  - pybind11 deprecation and `CMP0148` warnings under CMake 4.4.0

## Build Outcome
- Status: PASS
- Configuration: `Release`
- Build driver: `MSBuild 17.14.40+3e7442088 for .NET Framework`
- Primary native outputs:
  - `build-phase1-clean\Release\drone_node.exe`
  - `build-phase1-clean\Release\drone_bridge.cp314-win_amd64.pyd`
  - `build-phase1-clean\Release\sensor_fusion_core.lib`
- Warning count observed in the clean Release build: `3`
  - `src/sensors/IMUSensor.cpp`: `C4102` unreferenced label `simulate`
  - `src/telemetry/ControlPlaneTelemetryClient.cpp`: `C4996` on `getenv`
  - `src/hal/JetsonHAL.cpp`: `C4505` unreferenced internal helper
- Error count: `0`

## Native Test Outcome
- Status: PASS
- Command: `ctest --test-dir build-phase1-clean -C Release --output-on-failure`
- Total tests: `112`
- Passed: `112`
- Failed: `0`
- Skipped: `0`
- Not run: `0`
- Duration: `5.95 sec`
- JUnit artifact: `build-phase1-clean/docs/phase1/ctest-results.xml`

## Official Local Validation Outcome
- Status: PASS
- Command executed inside initialized MSVC environment with `VCPKG_ROOT` set
- Python syntax check: PASS
- Python unit tests: PASS (`12`)
- Go tests: PASS (`2` Go packages)
- CMake configure: PASS
- CMake Release build: PASS
- CTest: PASS (`112/112`)

## PASS / FAIL / SKIP / BLOCKED Counts
- PASS: 15
- FAIL: 0
- SKIP: 0
- BLOCKED: 0

## Remaining Limitations
- Compiler and CMake policy warnings remain and should be addressed in a future maintenance pass, but they did not block a successful native validation closure.
- The repository is still in an uncommitted worktree state; this report reflects validation of local changes, not a new commit.

## Evidence Paths
- `build-phase1-clean/CMakeCache.txt`
- `build-phase1-clean/CMakeFiles/CMakeConfigureLog.yaml`
- `build-phase1-clean/Testing/Temporary/LastTest.log`
- `build-phase1-clean/docs/phase1/ctest-results.xml`
- `build-local-validate/CMakeCache.txt`
- `build-local-validate/Testing/Temporary/LastTest.log`
