# Phase 6 Stress Test Report

Date: 2026-07-16

Primary artifact: `docs/phase6/stress_results.json`

## Scenarios

- Scenario A: `100` concurrent clients, `1000` requests
- Scenario B: `500` concurrent clients, `2500` requests
- Scenario C: `1000` concurrent clients, `5000` requests
- Additional telemetry burst surrogate: `200` workers, `4000` requests

## Results

| Scenario | Requests/sec | Failure rate | Mean (ms) | p95 (ms) | p99 (ms) | Max (ms) | CPU delta (s) | WS delta (MB) | Private delta (MB) |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `api_stress_100` | 1695.273 | 0.0% | 30.176 | 55.017 | 63.974 | 94.066 | 0.000 | -0.438 | -0.438 |
| `api_stress_500` | 1798.609 | 0.0% | 47.843 | 90.309 | 107.150 | 132.778 | 0.000 | -0.438 | -0.438 |
| `api_stress_1000` | 2094.019 | 0.0% | 54.845 | 105.413 | 130.194 | 210.865 | 0.000 | 0.000 | 0.000 |
| `telemetry_burst_multi_node` | 2150.524 | 0.0% | 45.949 | 85.293 | 107.227 | 152.971 | 0.000 | -6.562 | -6.562 |

## Resource Stability

- thread-count delta stayed at `0` in every scenario
- handle-count delta stayed at `0` in every scenario
- no monotonic memory growth was observed
- no scenario recorded request failures, hangs, or backend restarts

## Latency Degradation

Relative to the `100`-client case:

- mean latency increased from `30.176 ms` to `47.843 ms` at `500` clients
- mean latency increased to `54.845 ms` at `1000` clients
- p95 latency increased from `55.017 ms` to `105.413 ms`
- degradation was progressive rather than cliff-like, and remained correctness-safe

## Interpretation

PASS:

- system remained stable under expected workload
- no crash, deadlock, or data corruption was observed

WARNING:

- latency tails widened under `500` and `1000` concurrent-client saturation
- the `1000`-client result should be treated as a workstation saturation test, not a production sizing claim

FAIL criteria observed:

- none

## Verdict

Stress status: `PASS`
