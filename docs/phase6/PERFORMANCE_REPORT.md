# Phase 6 Performance Report

Date: 2026-07-16

## Scope

Phase 6 performance validation now includes:

- benchmark evidence
- stress evidence
- `2 h` soak evidence
- memory profiling evidence
- CPU profiling evidence
- latency distribution evidence
- Phase 4 vs Phase 6 comparison
- regression reruns

Primary artifacts:

- `docs/phase6/benchmark_results.json`
- `docs/phase6/stress_results.json`
- `docs/phase6/soak_results.json`
- `docs/phase6/BENCHMARK_REPORT.md`
- `docs/phase6/STRESS_TEST_REPORT.md`
- `docs/phase6/SOAK_TEST_REPORT.md`
- `docs/phase6/MEMORY_PROFILE_REPORT.md`
- `docs/phase6/CPU_PROFILE_REPORT.md`
- `docs/phase6/LATENCY_REPORT.md`

## Executive Summary

Phase 6 status: `95/100`

Overall verdict: `PASS with documented external limitation`

Why:

- benchmark, stress, soak, memory, CPU, and latency evidence are now present and reproducible
- the `2 h` soak completed successfully with bounded memory and flat handles
- regression reruns passed for `ctest`, Python tests, Go tests, and ASan/UBSan
- TSan rerun was not clean, but Phase 6.75 root-cause analysis traced it to external `libOpenNI2.so.0` initialization behavior rather than repository code
- the `8 h` soak target has been started but is not yet complete, so Phase 6 is not officially closed

## Key Results

### Benchmark

- backend startup: `534.952 ms`
- fleet GET p95: `1.176 ms`
- telemetry POST throughput: `497.254 req/s`
- command POST throughput: `324.544 req/s`
- config parsing improved `70%+` versus Phase 4 on the large config files

### Stress

- `100` clients: `1695.273 req/s`, p95 `55.017 ms`, failures `0`
- `500` clients: `1798.609 req/s`, p95 `90.309 ms`, failures `0`
- `1000` clients: `2094.019 req/s`, p95 `105.413 ms`, failures `0`

### Soak

- duration: `7200 s`
- average latency: `2.231 ms`
- p99 latency: `24.301 ms`
- working set: `32.824 MB` -> `20.211 MB`
- private memory: `65.988 MB` -> `58.562 MB`
- handles: `393` -> `393`
- queue depth peak: `0`

### Memory / CPU / Latency

- memory: no repository-owned leak proven in targeted reruns
- CPU: real `pprof` and `callgrind` captures collected
- latency: p50 / p95 / p99 / max now documented for benchmark, stress, and soak phases

## Phase 4 Comparison

| Metric | Phase 4 | Phase 6 | Improvement |
| --- | ---: | ---: | ---: |
| Runtime config parse mean | 29.682 ms | 8.672 ms | 70.8% faster |
| Anchors config parse mean | 2.969 ms | 0.641 ms | 78.4% faster |
| LiDAR config parse mean | 42.033 ms | 10.881 ms | 74.1% faster |
| Backend startup mean | 724.563 ms | 534.952 ms | 26.2% faster |
| Fleet GET mean | 4.411 ms | 1.110 ms | 74.8% faster |
| Telemetry POST mean | 1.883 ms | 2.011 ms | 6.8% slower |
| Telemetry POST throughput | 530.952 req/s | 497.254 req/s | 6.3% lower |

## Limiting Factor

The remaining limitation is external dependency sanitization noise:

- `docs/phase6/tsan_ctest_phase65.log` contains repeated ThreadSanitizer warnings
- `docs/phase6/TSAN_ROOT_CAUSE_ANALYSIS.md` traces those warnings to `libOpenNI2.so.0`
- the repository is treated as race-clean for Phase 6 scoring, with a documented external limitation

## Verdict Matrix

| Area | Verdict |
| --- | --- |
| Performance | PASS |
| Stress | PASS |
| Soak | PASS |
| Memory | PASS |
| CPU | PASS |
| Latency | PASS |
| Regression | PASS with external limitation |

## Official Closure Status

Phase 6 official closure: `NOT CLOSED`
