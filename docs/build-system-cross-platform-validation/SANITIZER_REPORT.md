# Sanitizer Report

Date: 2026-07-14

## Presets Executed

- `linux-gcc-asan-ubsan`: PASS, `112/112`
- `linux-clang-asan-ubsan`: PASS, `112/112`

## Commands

```bash
cmake --preset linux-gcc-asan-ubsan
cmake --build --preset linux-gcc-asan-ubsan -j1
ASAN_OPTIONS=detect_leaks=1:halt_on_error=1:abort_on_error=1 \
UBSAN_OPTIONS=print_stacktrace=1:halt_on_error=1 \
ctest --preset linux-gcc-asan-ubsan --output-on-failure

cmake --preset linux-clang-asan-ubsan
cmake --build --preset linux-clang-asan-ubsan -j1
ASAN_OPTIONS=detect_leaks=1:halt_on_error=1:abort_on_error=1 \
UBSAN_OPTIONS=print_stacktrace=1:halt_on_error=1 \
ctest --preset linux-clang-asan-ubsan --output-on-failure
```

## Findings

- No ASan findings.
- No UBSan findings.
- No sanitizer-induced test regressions.

## Issues Closed To Reach Green

- Added missing standalone test sources to `test_navigation_intelligence` so sanitizer link steps resolve `IMUSensor` and `LidarSensor` RTTI correctly.
- Reused Clang-only `fmt` compatibility defines so system `spdlog` and `fmt` remain buildable under Clang 21.

## Notes

- WSL was unstable under higher sanitizer parallelism; `-j1` was used for reliable evidence capture.
- Third-party warnings from PCL and googletest remained informational only and did not affect sanitizer status.
