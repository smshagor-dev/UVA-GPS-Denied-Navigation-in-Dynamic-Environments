# Phase 6 Benchmark Report

Date: 2026-07-16

Primary artifact: `docs/phase6/benchmark_results.json`

## Scope

This benchmark pass reran the Phase 6 suite and captured:

- backend startup latency
- API throughput and endpoint latency
- telemetry ingest rate
- control command latency
- metrics endpoint latency
- representative sensor-pipeline microbenchmarks
- swarm serialization and authentication overhead

## Backend Results

| Metric | Mean (ms) | p50 (ms) | p95 (ms) | p99 (ms) | Max (ms) | Throughput |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| Backend startup | 534.952 | 534.952 | 534.952 | 534.952 | 534.952 | n/a |
| `GET /api/v1/fleet` | 1.110 | 0.559 | 1.176 | 15.852 | 15.852 | n/a |
| `POST /api/v1/telemetry` | 2.011 | 0.939 | 12.149 | 22.059 | 22.059 | 497.254 req/s |
| `GET /metrics` | 1.691 | 0.975 | 1.871 | 21.609 | 21.609 | n/a |
| `POST /api/v1/commands` | 3.081 | 0.735 | 15.465 | 16.525 | 16.525 | 324.544 req/s |

Interpretation:

- control-plane startup is comfortably sub-second
- steady-state fleet and metrics reads are sub-2 ms at p95
- telemetry and command paths are low-latency at median, with visible tail spikes under local Windows scheduling noise

## Sensor Pipeline Results

These are representative wall-clock path timings from the automated harness, not direct hardware Hz captures.

| Metric | Mean (ms) | p50 (ms) | p95 (ms) | p99 (ms) | Max (ms) | Approx. rate |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| IMU / EKF update path | 17.390 | 13.276 | 23.970 | 123.001 | 123.001 | 57.5 Hz mean |
| LiDAR processing path | 20.493 | 12.716 | 110.891 | 119.822 | 119.822 | 48.8 Hz mean |
| Fusion path | 34.655 | 19.090 | 121.180 | 122.300 | 122.300 | 28.9 Hz mean |
| Telemetry serialization path | 15.740 | 9.985 | 22.272 | 121.210 | 121.210 | 63.5 Hz mean |
| Swarm auth path | 22.603 | 12.219 | 115.594 | 124.142 | 124.142 | 44.2 Hz mean |

Notes:

- the repository does not expose isolated GPS, VIO-only, or direct LiDAR hardware feed counters in this harness, so the nearest validated proxy metrics are reported
- medians are materially more representative than means for the native microbenchmarks because repeated process launch dominates the tail

## Config and Parsing Benchmarks

| Metric | Mean (ms) | p50 (ms) | p95 (ms) | p99 (ms) | Max (ms) |
| --- | ---: | ---: | ---: | ---: | ---: |
| `config/runtime.json` load | 8.672 | 9.096 | 10.308 | 12.613 | 12.613 |
| `config/anchors.json` load | 0.641 | 0.598 | 0.913 | 0.937 | 0.937 |
| `config/lidar.json` load | 10.881 | 10.664 | 13.858 | 17.115 | 17.115 |

## Swarm-Specific Costs

| Metric | Mean (ms) | p50 (ms) | p95 (ms) | p99 (ms) | Max (ms) |
| --- | ---: | ---: | ---: | ---: | ---: |
| Telemetry serialization | 15.740 | 9.985 | 22.272 | 121.210 | 121.210 |
| Authentication overhead | 22.603 | 12.219 | 115.594 | 124.142 | 124.142 |

## Phase 4 Comparison

Comparison source: `docs/phase4/benchmark_results.json`

| Metric | Phase 4 | Phase 6 | Improvement |
| --- | ---: | ---: | ---: |
| Runtime config parse mean | 29.682 ms | 8.672 ms | 70.8% faster |
| Anchors config parse mean | 2.969 ms | 0.641 ms | 78.4% faster |
| LiDAR config parse mean | 42.033 ms | 10.881 ms | 74.1% faster |
| Backend startup mean | 724.563 ms | 534.952 ms | 26.2% faster |
| Fleet GET mean | 4.411 ms | 1.110 ms | 74.8% faster |
| Telemetry POST mean | 1.883 ms | 2.011 ms | 6.8% slower |
| Telemetry POST throughput | 530.952 req/s | 497.254 req/s | 6.3% lower |

Interpretation:

- startup, config parsing, and fleet read paths improved materially versus the Phase 4 baseline
- telemetry POST stayed in the same performance band but regressed slightly in this local rerun
- native microbenchmark means remain too noisy to support strong improvement claims without an in-process profiler-backed harness

## Verdict

Benchmark status: `PASS`
