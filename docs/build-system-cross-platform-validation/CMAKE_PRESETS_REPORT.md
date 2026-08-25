# CMake Presets Report

Date: 2026-07-14

## Added Presets

Windows:

- `windows-msvc-debug`
- `windows-msvc-release`
- `validation-msvc`
- `windows-msvc-release-werror`
- `windows-msvc-release-minimal`
- `windows-msvc-release-full`

Linux:

- `linux-gcc-debug`
- `linux-gcc-release`
- `validation-linux-gcc`
- `linux-clang-debug`
- `linux-clang-release`
- `linux-gcc-asan-ubsan`
- `linux-clang-asan-ubsan`
- `linux-gcc-release-werror`
- `linux-gcc-release-minimal`

## Design Notes

- No tracked personal paths were added.
- Windows presets use `VCPKG_ROOT` for the toolchain file.
- Windows default presets enable only the `tests` manifest feature.
- `windows-msvc-release-full` enables the `fastdds` manifest feature.
- `windows-msvc-release-minimal` disables Python bindings.
- Linux presets target Ninja plus GCC or Clang.

## Validation Evidence

Verified:

- `cmake --preset windows-msvc-release`
- `cmake --build --preset windows-msvc-release`
- `ctest --preset windows-msvc-release`
- `cmake --preset windows-msvc-debug`
- `cmake --build --preset windows-msvc-debug`
- `ctest --preset windows-msvc-debug`

Blocked:

- Linux preset execution in WSL Ubuntu because `cmake` was not installed there.
