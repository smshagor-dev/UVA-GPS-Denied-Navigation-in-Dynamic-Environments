# Sanitizer Validation

## Date

Friday, July 17, 2026

## Prior sanitizer evidence

Before this closure pass:

- Linux GCC ASan+UBSan full test suite passed locally.
- Linux Clang ASan+UBSan full test suite passed locally.
- A manual local Clang TSan lane existed, but representative replay targets still hit noisy third-party startup warnings from OpenNI-linked paths.

## Phase 16D isolation change

To make the estimator validation lane attributable, the replay, shadow test, and benchmark targets were moved onto an estimator-only linkage path:

- `estimator_validation_core`
- `estimator_replay_support`
- `test_estimator_shadow`
- `estimator_shadow_benchmark`

This removed the need for the broader sensor-fusion linkage on the representative Phase 16 estimator targets used for TSan evidence.

## TSan targets executed

The following local targets were executed from `build/linux-clang-tsan`:

- `tests/Debug/test_estimator_replay`
- `tests/Debug/test_estimator_shadow`
- `tools/Debug/estimator_replay_runner`
- `tools/Debug/estimator_shadow_benchmark`

## Result classification

| Target | Result | Notes |
| --- | --- | --- |
| `test_estimator_replay` | PASS | No TSan finding observed |
| `test_estimator_shadow` | PASS | No TSan finding observed |
| `estimator_replay_runner` | PASS | Replay completed successfully on the Phase 15 stationary fixture |
| `estimator_shadow_benchmark` | PASS after fix | Initial TSan run found a tool-local race in benchmark latency sampling |

## Tool-local race fixed

TSan initially reported a data race in `tools/estimator_shadow_benchmark.cpp`:

- writer: shadow worker updated `last_processing_latency_us_`
- reader: main thread sampled `last_processing_latency_us()`

This was fixed by changing the latency field to `std::atomic<double>` and using relaxed load/store operations.

## Third-party result

For the isolated estimator-only Phase 16 targets executed in this session:

- no OpenNI startup warning reappeared
- no third-party TSan suppression file was used
- no third-party TSan issue is currently blocking this estimator-validation lane

## Remaining limitation

This document covers the representative Phase 16 estimator validation path, not every binary in the repository.
Other non-estimator binaries may still require separate sanitizer attribution if broader repository-wide TSan coverage is desired later.
