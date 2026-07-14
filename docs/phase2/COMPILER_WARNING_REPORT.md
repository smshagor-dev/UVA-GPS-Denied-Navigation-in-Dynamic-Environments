# Compiler Warning Report

Date: 2026-07-14

## Project-Owned Warnings Fixed

`src/sensors/IMUSensor.cpp`

- Fixed MSVC `C4102` by removing the unused `goto` label path and keeping the fallback logic structured.

`src/telemetry/ControlPlaneTelemetryClient.cpp`

- Fixed MSVC `C4996` by using `_dupenv_s` on Windows instead of `std::getenv`.

`src/hal/JetsonHAL.cpp`

- Fixed MSVC `C4505` by limiting `read_sysfs_float` to Linux builds.
- Fixed follow-on MSVC `C4100` warnings in the Windows simulation path by explicitly marking unused parameters.

## Remaining Visible Warnings

Third-party or upstream:

- `pybind11` fetched source emits a deprecated CMake minimum warning under CMake 4.4.0.

Not suppressed:

- No broad `/wdXXXX` suppression flags were added.
- Warning behavior remains target-scoped through `cmake/CompilerWarnings.cmake`.
