# Full Local Validation Report

Date: 2026-05-14

## Scope

This report validates only:

- local build readiness
- local C++ dependency readiness
- Python/Go validation readiness
- dashboard/backend readiness
- telemetry pipeline readiness
- bench-demo readiness

This report does **not** mark the platform flight-ready.

## Installed Dependencies

Installed on the validating Windows workstation with `vcpkg` rooted at the active `VCPKG_ROOT`:

- `eigen3:x64-windows`
- `opencv:x64-windows`
- `pcl:x64-windows`
- `spdlog:x64-windows`
- `fmt:x64-windows`

Notable transitive/runtime dependencies installed by `vcpkg` included:

- `opencv4:x64-windows`
- `boost-*`
- `flann:x64-windows`
- `protobuf:x64-windows`
- `qhull:x64-windows`
- `zlib:x64-windows`

## Toolchain

Detected toolchain used for validation:

```text
%VCPKG_ROOT%\scripts\buildsystems\vcpkg.cmake
```

Environment status:

- `VCPKG_ROOT` was set successfully for the current user and current session
- machine-level `VCPKG_ROOT` could not be written from this session because Windows registry elevation was not available
- validation succeeded anyway by using the explicit toolchain path above

## Validation Commands

Python and Go validation:

```powershell
python -m py_compile gui/dashboard.py gui/backend_status.py main.py
python -m unittest tests.test_dashboard_backend_status
go test ./...
```

CMake configure/build/test:

```powershell
cmake -S . -B build-local-validate -DBUILD_TESTS=ON -DCMAKE_TOOLCHAIN_FILE="$env:VCPKG_ROOT/scripts/buildsystems/vcpkg.cmake"
cmake --build build-local-validate --config Release
ctest --test-dir build-local-validate --output-on-failure -C Release
```

One-command validator:

```powershell
python scripts/local_validate.py --toolchain "$env:VCPKG_ROOT\scripts\buildsystems\vcpkg.cmake"
```

Backend production-mode smoke validation:

```powershell
$env:DRONE_BACKEND_MODE="production"
$env:DRONE_BACKEND_SIMULATION_ENABLED="false"
go run ./cmd/control-plane
```

Then:

```powershell
python scripts/telemetry_smoke_test.py --backend-url http://127.0.0.1:8080
python scripts/production_telemetry_smoke_test.py --backend-url http://127.0.0.1:8080
```

## Results

### Go / Python Validation Result

- `python -m py_compile ...`: passed
- `python -m unittest tests.test_dashboard_backend_status`: passed
- `go test ./...`: passed

### CMake Configure Result

- passed

Highlights:

- Eigen3 discovered through the installed `vcpkg` toolchain
- OpenCV discovered: `4.12.0`
- PCL discovered: `1.15.1`
- `spdlog` discovered

### Build Result

- passed

Built successfully:

- `drone_node.exe`
- `sensor_fusion_core.lib`
- `drone_bridge.cp314-win_amd64.pyd`
- all configured C++ test executables

### CTest Result

- passed

Summary:

- `84/84` tests passed

## Telemetry Smoke Test Result

### Simulation Telemetry Smoke Test

Command:

```powershell
python scripts/telemetry_smoke_test.py --backend-url http://127.0.0.1:8080
```

Result:

- passed
- simulation-tagged telemetry appeared in `GET /api/v1/fleet`
- payload stayed marked `source=simulation`
- LiDAR point array was capped to `256`

### Production Telemetry Smoke Test

Command:

```powershell
python scripts/production_telemetry_smoke_test.py --backend-url http://127.0.0.1:8080
```

Result:

- passed
- backend reported `backend_mode=production`
- backend reported `simulation_enabled=false`
- no seeded production fake demo fleet was present
- unavailable-source payload did not increase `real_drone_count`

## What Was Fixed

Dependency and validation setup:

- installed and bootstrapped `vcpkg` at the active `VCPKG_ROOT`
- added explicit Eigen3 discovery help and toolchain support to `scripts/local_validate.py`
- improved `CMakeLists.txt` Eigen discovery and fallback behavior
- made the validator inject runtime DLL paths for Windows `ctest`

Cross-platform build issues fixed during validation:

- Eigen geometry include exposure for anchor geometry cross-product logic
- Windows-safe subprocess output handling in `scripts/local_validate.py`
- missing brace / unmatched preprocessor region in `src/telemetry/ControlPlaneTelemetryClient.cpp`
- Windows build issues in `src/main.cpp`
- test target source/link completeness in `tests/CMakeLists.txt`
- callback return typing in `tests/test_telemetry.cpp`
- retry gating behavior in `ControlPlaneTelemetryClient`
- remote-command rejection wording alignment in `CommandPolicy`

## Remaining Blockers

No blocker remains for local software build validation or local bench-demo telemetry validation.

Remaining non-flight-readiness gaps:

- no real IMU/camera/LiDAR/UWB hardware validation is included in this report
- no replay-backed flight-sensor acceptance evidence is included in this report
- no tethered or free-flight validation is included in this report
- `pybind11` emits CMake deprecation warnings during configure, but this did not block validation
- machine-level `VCPKG_ROOT` was not writable from this session; user/session-level setup plus explicit toolchain path were sufficient

## Current Readiness Verdict

**Full local software validation passed. The platform is now locally buildable and bench-demo ready, pending real hardware validation.**

More specifically:

- local C++ build readiness: ready
- local Python/Go validation readiness: ready
- dashboard/backend readiness: ready
- telemetry pipeline readiness: ready
- bench-demo readiness: ready
- flight readiness: **not validated and not claimed**
