# Performance Report

Phase 6 performance validation remains at `95/100` on 2026-07-16 with an overall verdict of `PASS with documented external limitation`.

The evidence set is complete for benchmark, stress, memory, CPU, latency, and the validated `2 h` soak. An `8 h` soak has been started but has not yet completed, so Phase 6 is not officially closed. Raw results are stored in `docs/performance-engineering-stability-validation/benchmark_results.json`, `docs/performance-engineering-stability-validation/stress_results.json`, and `docs/performance-engineering-stability-validation/soak_results.json`.

The remaining limitation is regression sanitization noise from `libOpenNI2.so.0`, documented in `docs/performance-engineering-stability-validation/TSAN_ROOT_CAUSE_ANALYSIS.md`. All major repository-owned gates passed, including `ctest`, Python tests, Go tests, and Linux ASan/UBSan reruns.

Primary reports:

- `docs/performance-engineering-stability-validation/BENCHMARK_REPORT.md`
- `docs/performance-engineering-stability-validation/STRESS_TEST_REPORT.md`
- `docs/performance-engineering-stability-validation/SOAK_TEST_REPORT.md`
- `docs/performance-engineering-stability-validation/MEMORY_PROFILE_REPORT.md`
- `docs/performance-engineering-stability-validation/CPU_PROFILE_REPORT.md`
- `docs/performance-engineering-stability-validation/LATENCY_REPORT.md`
- `docs/performance-engineering-stability-validation/PERFORMANCE_REPORT.md`
- `docs/performance-engineering-stability-validation/VALIDATION_REPORT.md`
- `docs/performance-engineering-stability-validation/FINAL_REPORT.md`
