# Phase 4 Sanitizer Report

Date: 2026-07-16

Evidence:

- `docs/architecture-safety-validation-hardening/asan_lsan_build.log`
- `docs/architecture-safety-validation-hardening/asan_lsan_ctest.log`

## Command Path

Linux preset used:

- `cmake --build --preset linux-gcc-asan-ubsan -j 4`
- `ASAN_OPTIONS=detect_leaks=1:halt_on_error=1 ctest --preset linux-gcc-asan-ubsan --output-on-failure`

## Result

- build succeeded
- `112/112` tests passed
- no ASan findings
- no LSan findings
- no UBSan findings

Observed non-fatal noise:

- PCL emitted its aligned-malloc warning during compilation
- no runtime sanitizer error followed from that warning in this run

## Verdict

Sanitizer status: PASS

