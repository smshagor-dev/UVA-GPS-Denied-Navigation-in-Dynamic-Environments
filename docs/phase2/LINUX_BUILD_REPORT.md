# Linux Build Report

Date: 2026-07-14
Environment: WSL2 Ubuntu 26.04 LTS on Windows host

## Toolchain

- GCC `15.2.0`
- Clang `21.1.8`
- CMake `4.2.3`
- Ninja `1.13.2`
- Python `3.14.4`
- Go `1.26.0`
- OpenSSL `3.5.5`

## Presets Executed

- `linux-gcc-debug`: PASS, `112/112`
- `linux-gcc-release`: PASS, `112/112`
- `linux-clang-debug`: PASS, `112/112`
- `linux-clang-release`: PASS, `112/112`
- `linux-gcc-release-werror`: PASS, `112/112`
- `linux-gcc-release-minimal`: PASS, `112/112`
- `validation-linux-gcc`: PASS, `112/112`

## Commands

```bash
cmake --preset linux-gcc-debug
cmake --build --preset linux-gcc-debug
ctest --preset linux-gcc-debug --output-on-failure

cmake --preset linux-gcc-release
cmake --build --preset linux-gcc-release
ctest --preset linux-gcc-release --output-on-failure

cmake --preset linux-clang-debug
cmake --build --preset linux-clang-debug
ctest --preset linux-clang-debug --output-on-failure

cmake --preset linux-clang-release
cmake --build --preset linux-clang-release
ctest --preset linux-clang-release --output-on-failure

cmake --preset linux-gcc-release-werror
cmake --build --preset linux-gcc-release-werror
ctest --preset linux-gcc-release-werror --output-on-failure
```

## Linux-Specific Fixes Closed During Validation

- Reworked several nested config constructors that GCC rejected in headers using default arguments.
- Added Linux OpenSSL implementations for `SwarmSecurity` hashing, HMAC, PBKDF2, and AES paths.
- Marked external OpenCV and PCL include directories as `SYSTEM` so `-Werror` stays strict for project code without failing on third-party warnings.
- Fixed `test_navigation_intelligence` so standalone sanitizer builds include the concrete `IMUSensor.cpp` and `LidarSensor.cpp` units required by RTTI.
- Updated the exported package config and target usage requirements so Linux consumers receive MPI, OpenCV, and PCL requirements correctly.

## Notes

- GCC sanitizer and clean-source runs were most stable with `-j1` inside WSL.
- Clang builds emitted optional dependency noise about OpenMP-related PCL features being disabled; validation still passed.
- PCL printed an aligned-allocation warning under sanitizer builds, but no sanitizer or test failure occurred.
