# Phase 14 Benchmark Framework

Date: July 17, 2026

## Purpose

This framework defines how the repository should be benchmarked for external comparison without overstating claims. It references measured Phase 6 evidence and keeps synthetic or workstation-local results clearly labeled.

## Benchmark Categories

| Category | Source | Measurement type |
|---|---|---|
| Backend startup | Phase 6 suite | measured |
| Fleet API latency | Phase 6 suite | measured |
| Telemetry throughput | Phase 6 suite | measured |
| Stress throughput | Phase 6 stress run | measured |
| Scenario recovery behavior | Phase 7 and Phase 8 scripts | measured |
| AI planning and coordination quality | Phase 9 and Phase 11 scripts | measured |

## Methodology Rules

- record command, environment, timestamp, and artifact path
- separate measured results from estimates and literature discussion
- report p50, p95, p99, and maximum where latency distributions exist
- treat workstation-local results as reproducibility evidence, not field superiority claims
- preserve machine-readable outputs under `docs/phase*/` or `results/`

## Current Baseline References

- Phase 6 performance suite: `docs/phase6/performance_results.json`
- Phase 6 benchmark report: `docs/phase6/BENCHMARK_REPORT.md`
- Phase 7 regression evidence: `docs/phase7/performance_regression_final.json`
- Phase 8 benchmark comparison package: `docs/phase8/FINAL_BENCHMARK_REPORT.md`

## External Comparison Guidance

Use like-for-like comparisons only:

- same workload class
- same hardware class when possible
- same concurrency profile
- same latency percentile definitions
- same software-only or hardware-backed validation category

## Conclusion

The repository has a documented and reproducible software benchmark methodology suitable for external engineering discussion.
