# Phase 14 Performance Baseline

Date: July 17, 2026
Status: PASS

## Baseline Source

This baseline references the validated Phase 6 evidence set and a fresh software rerun performed on July 17, 2026.

## Current Measured Values

| Metric | Value |
|---|---|
| Backend startup latency | 9626.005299971439 ms |
| Fleet API p95 latency | 2.716099959798157 ms |
| Telemetry throughput | 174.50382748822489 Hz |
| Stress throughput | 1978.039802308292 req/s |
| Phase 7 average mission latency | 6.338635987291733 ms |
| Phase 8 reliability | 1.0 |
| Phase 8 average detection time | 0.3 s |
| Phase 8 average recovery time | 1.0 s |

## Interpretation

- software performance evidence exists and is reproducible through repository scripts
- results are appropriate for workstation-local validation and regression tracking
- results must not be represented as aircraft, radio, or field benchmark proof

## Primary References

- `docs/performance-engineering-stability-validation/PERFORMANCE_REPORT.md`
- `docs/performance-engineering-stability-validation/FINAL_REPORT.md`
- `docs/research-validation-software-hil-scenarios/REGRESSION_REPORT.md`
- `docs/open-research-release-advanced-scenarios/FINAL_BENCHMARK_REPORT.md`

## Final Status

PASS: Baseline package is evidence-backed and suitable for external engineering review.

