# Phase 2 Dependency Management Report

Date: 2026-07-14

## Manifest

Tracked manifest:

- `vcpkg.json`

Baseline:

- `f87344cac03158cbf1467264565f1fd36b382a24`

## Classification

Required:

- `Eigen3`
- `OpenCV`
- `PCL`
- `spdlog`
- `Threads`

Optional:

- `pybind11`
- `Fast-DDS`
- `TensorRT`

Test-only:

- `GTest`

Vendored:

- `monocypher`
- `sha3`

## Resolution Rules

- `Eigen3`: package config first, then include-only fallback via `EIGEN3_ROOT` or `EIGEN3_INCLUDE_DIR`.
- `OpenCV`: required `find_package(OpenCV 4.5 REQUIRED ...)`.
- `PCL`: required `find_package(PCL 1.12 REQUIRED ...)`.
- `spdlog`: required `find_package(spdlog REQUIRED CONFIG)`.
- `Fast-DDS`: `find_package(fastdds QUIET CONFIG)` and `find_package(fastrtps QUIET CONFIG)`, else UDP fallback.
- `TensorRT`: explicit opt-in, environment or system hints only, else OpenCV DNN fallback.
- `pybind11`: package config first, then FetchContent fallback when enabled.
- `GTest`: package config first, then FetchContent fallback when enabled.

## Windows Evidence

Observed release configure summary:

- `Fast-DDS: NOT FOUND - UDP transport fallback enabled`
- `TensorRT: DISABLED - OpenCV DNN fallback enabled`
- `Python bindings: FETCHED - pybind11 FetchContent fallback`

## Phase 2 Outcome

- Required dependencies resolved reproducibly on Windows through the manifest-backed presets.
- Default Windows presets no longer require `pybind11` through the manifest, which removed the previous `libffi` and LLVM download failure path.
