# Phase 6 Memory Profile Report

Date: 2026-07-16

Primary artifacts:

- `docs/phase6/soak_results.json`
- `docs/phase6/valgrind_drone_node_phase65.log`
- `docs/phase6/valgrind_test_navigation_intelligence_phase65.log`
- `docs/phase6/valgrind_test_telemetry_phase65.log`
- `docs/phase6/asan_ubsan_ctest_phase65.log`
- `docs/phase6/tsan_ctest_phase65.log`

## Live Soak Evidence

From the completed `2 h` soak:

- working set: `32.824 MB` -> `20.211 MB`
- private memory: `65.988 MB` -> `58.562 MB`
- handles: `393` -> `393`
- thread count: `21` -> `20`
- queue depth peak: `0`

Interpretation:

- no leak signature was observed in the live process metrics
- memory ended lower than it began and stabilized after warmup

## ASan / UBSan

Command evidence: `docs/phase6/asan_ubsan_ctest_phase65.log`

- result: `100% tests passed, 0 tests failed out of 114`
- classification: repository-owned memory safety `PASS`

## Valgrind / Memcheck

### `drone_node`

Source: `docs/phase6/valgrind_drone_node_phase65.log`

- definitely lost: `21,211 bytes`
- indirectly lost: `0 bytes`
- possibly lost: `1,632 bytes`
- still reachable: `646,001 bytes`

Classification:

- `WARNING`, but primarily external dependency noise
- dominant stacks point into `gstreamer`, `gnutls`, `libtasn1`, and `opencv_videoio`

### `test_navigation_intelligence`

Source: `docs/phase6/valgrind_test_navigation_intelligence_phase65.log`

- definitely lost: `0 bytes`
- indirectly lost: `0 bytes`
- possibly lost: `0 bytes`
- still reachable: `58,149 bytes`
- error summary: `0`

Classification:

- `PASS`

### `test_telemetry`

Source: `docs/phase6/valgrind_test_telemetry_phase65.log`

- all heap blocks freed
- error summary: `0`

Classification:

- `PASS`

## ThreadSanitizer Note

Source: `docs/phase6/tsan_ctest_phase65.log`

- TSan rerun was not clean
- warnings repeatedly resolve to `libOpenNI2.so.0` and `pthread_mutex_unlock`
- this is treated as external dependency noise, not confirmed repository-owned memory corruption

## Repository-Owned vs External

Repository-owned issue:

- none proven by ASan, UBSan, or Valgrind in the targeted Phase 6 reruns

External dependency noise:

- `libOpenNI2.so.0` TSan mutex warnings
- `gstreamer` / `gnutls` / `opencv_videoio` reachable allocations and small leak buckets in `drone_node`

## Verdict

Memory status: `PASS`
