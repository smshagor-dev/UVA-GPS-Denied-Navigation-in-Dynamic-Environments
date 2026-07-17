# Performance Baseline

## Scope

This document records software-only Phase 16 benchmark evidence gathered on Friday, July 17, 2026.
It does not claim hard real-time guarantees, hardware safety, or flight readiness.

## Benchmark tool

Target:

- `estimator_shadow_benchmark`

Scenarios:

- `active_only`
- `active_with_shadow`
- `shadow_overload`

Primary Phase 16D aggregate artifacts:

- `artifacts/phase16d_benchmark_aggregate.json`
- `artifacts/phase16d_benchmark_summary.md`

Earlier single-run cross-toolchain artifacts remain available for historical context:

- `artifacts/phase16c_gcc_benchmark.json`
- `artifacts/phase16c_clang_benchmark.json`
- `artifacts/phase16c_gcc_asan_benchmark.json`
- `artifacts/phase16c_clang_asan_benchmark.json`

## Repeated-run methodology

The current methodology uses deterministic randomized repeated runs with seed `16016`:

- `active_only`: 10 runs
- `active_with_shadow`: 10 runs
- `shadow_overload`: 5 runs

Per run:

- warm-up iterations: `250`
- measured iterations: `2000`

The `active_with_shadow` scenario now waits for shadow drain on each iteration so it measures a nominal synchronized shadow path instead of an accidental overload case.

## Aggregate environment

- Operating system: Linux
- Compiler: GCC 15.2.0
- Build type: Release
- CPU: 12th Gen Intel(R) Core(TM) i5-12400F
- Timer source: `std::chrono::steady_clock`

## Aggregate results

### Active only

- Mean of run means: `3.035 us`
- Median of run means: `3.004 us`
- Aggregate median: `2.804 us`
- Aggregate p95: `4.522 us`
- Aggregate p99: `5.370 us`
- Worst maximum latency: `215.220 us`
- Dropped events: `0`
- Final shadow healths: `disabled`

### Active with shadow

- Mean of run means: `13.522 us`
- Median of run means: `13.342 us`
- Aggregate median: `12.183 us`
- Aggregate p95: `19.610 us`
- Aggregate p99: `29.497 us`
- Worst maximum latency: `174.786 us`
- Dropped events: `0`
- Maximum queue high-water mark: `3`
- Final shadow healths: `synchronized`
- Active output matched baseline in all runs: `true`

### Shadow overload

- Mean of run means: `3.819 us`
- Median of run means: `3.897 us`
- Aggregate median: `2.924 us`
- Aggregate p95: `5.663 us`
- Aggregate p99: `11.674 us`
- Worst maximum latency: `121.168 us`
- Dropped events: `11951`
- Maximum queue high-water mark: `8`
- Final shadow healths: `stale`
- Active output matched baseline in all runs: `true`

## Interpretation

- The nominal shadow mode stayed synchronized without drops in the repeated aggregate.
- The overload scenario dropped events and became stale while keeping queue growth bounded.
- Active-output equivalence held across all repeated runs.
- The benchmark is useful for regression detection, not for universal timing guarantees.
