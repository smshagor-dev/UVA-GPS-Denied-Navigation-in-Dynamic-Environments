# Phase 7 Regression Report

Date: July 16, 2026

Primary artifact: `docs/research-validation-software-hil-scenarios/performance_regression_final.json`

## Validation Commands

| Command | Result |
|---|---|---|
| `py -3.14 scripts\phase7_mission_scenarios.py` | PASS |
| `py -3.14 scripts\phase7_hil_runner.py` | PASS |
| `py -3.14 scripts\phase7_simulation_runner.py` | PASS |
| `py -3.14 scripts\phase7_failure_injection.py` | PASS |
| `python scripts\validate_config_schemas.py` | PASS |
| `python -m unittest tests.test_dashboard_backend_status` | PASS |
| `ctest --test-dir build\validation-msvc -C Release --output-on-failure` | PASS |
| `go test ./...` | PASS |
| `py -3.14 scripts\phase6_performance_suite.py` | PASS |
| `py -3.14 scripts\phase6_performance_benchmark.py` | PASS |
| `py -3.14 scripts\phase6_stress_test.py` | PASS |

## Isolation Confirmation

- The leftover Phase 6 soak monitor was stopped before the clean rerun.
- The isolated performance comparison was generated after that shutdown.

## Clean Regression Comparison

### API And Benchmark Highlights

| Metric | Phase 6 baseline | Isolated current | Outcome |
|---|---:|---:|---|
| Backend startup | 534.952 ms | 529.927 ms | Better |
| Fleet GET mean | 1.110 ms | 2.003 ms | Worse |
| Telemetry POST throughput | 497.254 req/s | 864.118 req/s | Better |
| Telemetry POST p95 | 12.149 ms | 1.693 ms | Better |
| Metrics GET mean | 1.691 ms | 3.609 ms | Worse |
| Command POST throughput | 324.544 req/s | 177.053 req/s | Worse |
| Command POST p95 | 15.465 ms | 16.001 ms | Slightly worse |

### Stress Highlights

| Metric | Phase 6 baseline | Isolated current | Outcome |
|---|---:|---:|---|
| 100 clients throughput | 1695.273 req/s | 2055.002 req/s | Better |
| 100 clients p95 | 55.017 ms | 46.323 ms | Better |
| 500 clients throughput | 1798.609 req/s | 2186.975 req/s | Better |
| 500 clients p95 | 90.309 ms | 71.823 ms | Better |
| 1000 clients throughput | 2094.019 req/s | 2184.108 req/s | Better |
| 1000 clients p95 | 105.413 ms | 93.919 ms | Better |

### Memory And Latency Stability

- queue depth peak: `0`
- stress failures total: `0`
- all stress handle deltas: `0`
- all stress thread deltas: `0`
- isolated soak working set end: `25.465 MB`
- isolated soak private memory end: `58.988 MB`

## Interpretation

- The previous contamination issue is closed.
- System-level stress behavior improved in the isolated rerun.
- Memory remained bounded and process-resource counts stayed flat.
- Focused endpoint benchmark samples on Windows still show per-run jitter, but the comparison is now clean and documented rather than blocked by concurrent workload.

## Verdict

Status: PASS

