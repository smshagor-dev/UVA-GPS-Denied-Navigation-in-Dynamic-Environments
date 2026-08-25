# Phase 6 Latency Report

Date: 2026-07-16

Primary artifacts:

- `docs/performance-engineering-stability-validation/benchmark_results.json`
- `docs/performance-engineering-stability-validation/stress_results.json`
- `docs/performance-engineering-stability-validation/soak_results.json`

## End-to-End Path

Measured path model:

`Sensor input -> Fusion -> Decision -> Control output`

The repository does not expose one single end-to-end timestamp chain for every stage, so this report uses the closest validated harness metrics as stage proxies.

## Stage Breakdown

| Stage | Proxy metric | Average (ms) | p50 (ms) | p95 (ms) | p99 (ms) | Worst (ms) |
| --- | --- | ---: | ---: | ---: | ---: | ---: |
| Sensor processing | `lidar_processing_path` | 20.493 | 12.716 | 110.891 | 119.822 | 119.822 |
| Fusion | `sensor_fusion_path` | 34.655 | 19.090 | 121.180 | 122.300 | 122.300 |
| Communication | `backend_telemetry_post` | 2.011 | 0.939 | 12.149 | 22.059 | 22.059 |
| Control output | `backend_command_post` | 3.081 | 0.735 | 15.465 | 16.525 | 16.525 |

Approximate end-to-end total if summed as proxies:

- average: `60.240 ms`
- p50-style steady-state proxy: `33.480 ms`
- worst-case proxy sum: `280.706 ms`

Inference note:

- the summed values above are approximations derived from separate validated measurements, not a single traced request

## Backend Endpoint Latency

| Endpoint | Average (ms) | p50 (ms) | p95 (ms) | p99 (ms) | Worst (ms) |
| --- | ---: | ---: | ---: | ---: | ---: |
| `GET /api/v1/fleet` | 1.110 | 0.559 | 1.176 | 15.852 | 15.852 |
| `POST /api/v1/telemetry` | 2.011 | 0.939 | 12.149 | 22.059 | 22.059 |
| `GET /metrics` | 1.691 | 0.975 | 1.871 | 21.609 | 21.609 |
| `POST /api/v1/commands` | 3.081 | 0.735 | 15.465 | 16.525 | 16.525 |

## Soak Latency

From the completed `2 h` run:

| Metric | Value |
| --- | ---: |
| Average | 2.231 ms |
| p50 | 1.138 ms |
| p95 | 12.384 ms |
| p99 | 24.301 ms |
| Worst case | 26.063 ms |

## Stress Latency

| Scenario | Average (ms) | p95 (ms) | p99 (ms) | Worst (ms) |
| --- | ---: | ---: | ---: | ---: |
| `100` clients | 30.176 | 55.017 | 63.974 | 94.066 |
| `500` clients | 47.843 | 90.309 | 107.150 | 132.778 |
| `1000` clients | 54.845 | 105.413 | 130.194 | 210.865 |

## Verdict

Latency status: `PASS`

