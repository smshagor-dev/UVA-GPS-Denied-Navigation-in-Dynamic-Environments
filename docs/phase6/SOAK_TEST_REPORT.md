# Phase 6 Soak Test Report

Date: 2026-07-16

Primary artifact: `docs/phase6/soak_results.json`

## Run Configuration

- duration: `7200 s` (`2 h`)
- sampling interval: `30 s`
- requests issued: `239`
- startup latency before soak: `551.403 ms`

## Latency Summary

| Metric | Value |
| --- | ---: |
| Average | 2.231 ms |
| p50 | 1.138 ms |
| p95 | 12.384 ms |
| p99 | 24.301 ms |
| Worst case | 26.063 ms |
| Latency drift | -14.887 ms |
| Queue depth peak | 0 |

## Resource Summary

| Metric | Start | End | Peak |
| --- | ---: | ---: | ---: |
| Working set | 32.824 MB | 20.211 MB | 32.824 MB |
| Private memory | 65.988 MB | 58.562 MB | 65.988 MB |
| CPU seconds | 0.781 | 1.125 | n/a |
| Thread count | 21 | 20 | 21 |
| Handle count | 393 | 393 | 393 |
| Open file descriptors proxy | 393 | 393 | 393 |

## Trend Snapshot

| Elapsed | Working set (MB) | Private (MB) | Latency (ms) |
| --- | ---: | ---: | ---: |
| 0.0 h | 32.82 | 65.99 | 16.02 |
| 1.0 h | 23.26 | 58.56 | 1.12 |
| 2.0 h | 20.21 | 58.56 | 1.14 |

Simple trend view:

```text
Working set MB: 32.8 -> 23.3 -> 20.2
Private MB:     66.0 -> 58.6 -> 58.6
Latency ms:     16.0 ->  1.1 ->  1.1
```

## Acceptance Review

PASS:

- memory growth remained bounded
- no crashes occurred
- no deadlocks occurred
- no increasing queue backlog was observed
- thread and handle counts remained flat after warmup

## Verdict

Soak status: `PASS`

## 8 Hour Extension Status

As of `2026-07-16 22:04` local time:

- an `8 h` soak continuation is running in the background
- process id: `28384`
- backend log continues to show successful 30-second telemetry ingestion
- `docs/phase6/soak_results.json` has not yet been replaced by an `8 h` completion artifact

Implication:

- the latest completed soak evidence remains the validated `2 h` run above
- the `8 h` closure target is not yet complete
