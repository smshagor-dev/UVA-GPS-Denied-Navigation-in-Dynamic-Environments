# Dependency Audit Report

## C++ / CMake Dependencies

Observed native dependencies from `CMakeLists.txt` and `tests/CMakeLists.txt`:

- Eigen3
- OpenCV
- PCL
- spdlog
- Threads
- optional `pybind11` via `find_package()` or `FetchContent`
- optional `nlohmann_json`, `fastdds`, `fastrtps`, `TensorRT`

### Notes

- `pybind11` is fetched automatically when not installed, which is convenient but adds network/build variability
- Native dependency validation could not be fully re-run in this workspace because `cmake` and `ctest` are not available on PATH

## Go Dependencies

`go.mod` is minimal:

- module: `drone_swarm/controlplane`
- Go version: `1.22`
- no explicit third-party modules declared in `go.mod`

Go tests passed in this workspace.

## Python Dependencies

Current `requirements.txt` after cleanup:

- `pyside6>=6.6`
- `pyqtgraph>=0.13`
- `PyOpenGL>=3.1`
- `numpy>=1.24`
- `opencv-python>=4.8`
- `pyserial>=3.5`
- `esptool>=4.6`
- `cryptography>=45.0`

### Changes

- Removed `spdlog` from Python requirements because it is a C++ dependency and caused `pip install -r requirements.txt` to fail on Windows without MSVC build tools
- Removed unused `rich` from Python dependency declarations because it is not imported anywhere in the repository

## Dependency Risks and Recommendations

| ID | Severity | Status | Item | Recommendation |
|---|---|---|---|---|
| DEP-001 | Medium | Fixed | `spdlog` listed in Python requirements | Keep `spdlog` documented as a native dependency only |
| DEP-002 | Low | Fixed | `rich` declared but unused | Keep requirements aligned with imports |
| DEP-003 | Medium | Remaining | Native dependency setup is workstation-dependent | Add CI or containerized validation for CMake-based builds |
| DEP-004 | Low | Remaining | `pybind11` network fetch during configure | Consider pinning via package manager or vendored submodule policy for reproducibility |

## Verdict

Python dependency hygiene improved. Native dependency reproducibility is still limited by workstation tooling availability and the absence of a current reproducible CMake validation run in this environment.
