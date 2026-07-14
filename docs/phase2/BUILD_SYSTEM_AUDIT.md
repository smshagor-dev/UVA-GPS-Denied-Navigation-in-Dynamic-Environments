# Phase 2 Build System Audit

Date: 2026-07-14
Repository commit: `cb94b22008e217b3f016888d3e98a2ca5aa5af7a`

## Scope Audited

- `CMakeLists.txt`
- `tests/CMakeLists.txt`
- `cmake/CompilerWarnings.cmake`
- `cmake/Dependencies.cmake`
- `cmake/Sanitizers.cmake`
- `cmake/jetson_toolchain.cmake`
- `CMakePresets.json`
- `vcpkg.json`
- `scripts/local_validate.py`
- `README.md`
- `DEPLOYMENT.md`

## Target Inventory

Required production targets:

- `sensor_fusion_core` static library
- `drone_node` executable

Optional targets:

- `drone_bridge` pybind11 module

Test targets:

- `test_ekf`
- `test_sensors`
- `test_slam`
- `test_autonomy`
- `test_navigation_intelligence`
- `test_swarm_security`
- `test_boot_trust`
- `test_telemetry`
- `test_edge_swarm`
- `test_v2x` only when Fast-DDS is available

## Structural Changes Completed

- Converted the root build to target-based CMake with `target_sources`, `target_include_directories`, `target_link_libraries`, `target_compile_definitions`, and `target_compile_features`.
- Removed tracked hardcoded `vcpkg` paths from the root build.
- Centralized dependency discovery in `cmake/Dependencies.cmake`.
- Centralized warning policy in `cmake/CompilerWarnings.cmake`.
- Centralized sanitizer logic in `cmake/Sanitizers.cmake`.
- Standardized output directories for runtime, libraries, tests, and Python artifacts.
- Added install rules for binaries, libraries, headers, examples, docs, and exported CMake package files.

## Optional Feature Behavior

- Fast-DDS: optional transport, explicit fallback message when not found.
- TensorRT: optional, disabled by default, explicit OpenCV DNN fallback message.
- Python bindings: optional, package-config first, then FetchContent fallback.
- Tests: enabled by `DRONE_BUILD_TESTS`.

## Fragile Assumptions Found And Closed

- Default Windows preset previously required `pybind11` from vcpkg manifest mode and failed in dependency bootstrap.
- Public headers exposed OpenCV, PCL, and vendored crypto headers without consistent target interface coverage.
- `scripts/local_validate.py` depended on a pre-exported `VCPKG_ROOT`.
- `cmake/jetson_toolchain.cmake` used fixed tracked SDK paths.

## Remaining Build-System Risks

- Linux runtime validation is blocked because the available WSL Ubuntu environment does not currently have `cmake` installed.
- `pybind11` emits a deprecated upstream CMake warning under CMake 4.4.0 when fetched; this is upstream, not project-owned.
- Installed CMake package files were installed successfully, but a downstream consumer `find_package()` smoke project was not executed in this phase.
