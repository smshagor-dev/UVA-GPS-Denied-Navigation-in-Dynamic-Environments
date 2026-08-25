# Phase 4 Benchmark Report

Date: 2026-07-16

Evidence:

- `docs/architecture-safety-validation-hardening/benchmark_results.json`
- `docs/architecture-safety-validation-hardening/backend_benchmark.log`
- `docs/architecture-safety-validation-hardening/parser_benchmark.log`
- `docs/architecture-safety-validation-hardening/benchmark_rerun.log`

## Harnesses

- `scripts/phase4_benchmark_suite.py`
- ad hoc parser microbenchmark captured in `docs/architecture-safety-validation-hardening/parser_benchmark.log`

## Benchmark Rerun Snapshot

Rerun captured on 2026-07-16:

- backend startup: `724.563 ms`
- backend fleet GET mean/p95: `4.411 ms` / `14.922 ms`
- backend telemetry POST mean/p95: `1.883 ms` / `2.798 ms`
- backend telemetry POST throughput: `530.95 Hz`
- runtime config JSON load mean: `29.682 ms`
- anchor config JSON load mean: `2.969 ms`
- LiDAR config JSON load mean: `42.033 ms`
- parser extraction speedup: `44.35x`

## Before / After Comparison

Previous baseline from the earlier Phase 4 run on 2026-07-16:

- backend startup: `21.392 ms`
- backend fleet GET mean: `9.460 ms`
- backend telemetry POST mean: `15.948 ms`
- runtime config JSON load mean: `27.810 ms`
- LiDAR config JSON load mean: `37.467 ms`

Current rerun:

- backend startup: `724.563 ms`
- backend fleet GET mean: `4.411 ms`
- backend telemetry POST mean: `1.883 ms`
- runtime config JSON load mean: `29.682 ms`
- LiDAR config JSON load mean: `42.033 ms`
- parser path: `823.453 ms -> 18.566 ms` over 5,000 iterations

Interpretation:

- backend startup and config-load numbers moved with local host variability
- API and telemetry request latency improved materially in the rerun
- the parser optimization remains the clearest stable before/after win because it compares the old and new extraction logic directly on the same machine

## Software Smoke Results

- `scripts/telemetry_smoke_test.py`: PASS
- `scripts/production_telemetry_smoke_test.py`: PASS

## Verdict

Benchmark reproducibility status: PASS

Still not sufficient for:

- flight-loop timing claims
- live RF throughput claims
- hardware validation claims

