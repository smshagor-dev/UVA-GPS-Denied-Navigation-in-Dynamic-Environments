# Performance Report

Phase 6 performance validation remains at `95/100` on 2026-07-16 with an overall verdict of `PASS with documented external limitation`.

The evidence set is complete for benchmark, stress, memory, CPU, latency, and the validated `2 h` soak. An `8 h` soak has been started but has not yet completed, so Phase 6 is not officially closed. Raw results are stored in `docs/phase6/benchmark_results.json`, `docs/phase6/stress_results.json`, and `docs/phase6/soak_results.json`.

The remaining limitation is regression sanitization noise from `libOpenNI2.so.0`, documented in `docs/phase6/TSAN_ROOT_CAUSE_ANALYSIS.md`. All major repository-owned gates passed, including `ctest`, Python tests, Go tests, and Linux ASan/UBSan reruns.

Primary reports:

- `docs/phase6/BENCHMARK_REPORT.md`
- `docs/phase6/STRESS_TEST_REPORT.md`
- `docs/phase6/SOAK_TEST_REPORT.md`
- `docs/phase6/MEMORY_PROFILE_REPORT.md`
- `docs/phase6/CPU_PROFILE_REPORT.md`
- `docs/phase6/LATENCY_REPORT.md`
- `docs/phase6/PERFORMANCE_REPORT.md`
- `docs/phase6/PHASE6_VALIDATION_REPORT.md`
- `docs/phase6/PHASE6_FINAL_REPORT.md`
