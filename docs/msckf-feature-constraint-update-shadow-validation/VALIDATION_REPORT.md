# Phase 22.1 Validation Report

Date: July 19, 2026

## Executed Lanes

- `msvc`: PASS, 19 Phase 22 unit tests passed
- `replay`: FAIL, nine scenarios executed and three scenarios failed required evidence predicates

## Blocked Lanes

The following lanes have explicit BLOCKED summaries because they were not rerun after the current Phase 22.1 code changes: `environment`, `msvc-werror`, `clang`, `clang-werror`, `gcc`, `gcc-werror`, `clang-format`, `clang-tidy`, `asan`, `ubsan`, `tsan`, `regressions`, `numerical-jacobians`, `nullspace`, `covariance`, `gating`, and `rollback`.

`clang-format` is blocked because `clang-format` is not available on PATH in this session.

## Unit Tests

`test_phase22_msckf_update` now contains 19 tests covering successful feature updates, residual convention, finite-difference Jacobians, null-space projection, DOF-aware chi-square diagnostics, covariance dimensions, non-finite input rejection, depth rejection, clone eviction layout, FEJ-enabled behavior, FEJ-disabled behavior, corrupt FEJ rejection, and deterministic ordering.

## Replay

`artifacts/phase22/ekf_phase22_replay_report.json` was refreshed by `ekf_phase22_replay`.

Scenarios executed:

- `straight_track_update`
- `turning_track_update`
- `long_track_update`
- `noisy_track_update`
- `rejected_track_update`
- `multi_feature_stack`
- `singular_geometry_rejection`
- `stale_fej_rejection`
- `update_disabled_control`

Passing scenarios: `straight_track_update`, `turning_track_update`, `long_track_update`, `noisy_track_update`, `rejected_track_update`, and `update_disabled_control`.

Failing scenarios: `multi_feature_stack`, `singular_geometry_rejection`, and `stale_fej_rejection`.

## Decision

- Phase 22 implementation status: `PARTIAL`
- Local validation status: `NOT READY`
- Active estimator authority preserved: `YES`
- Phase 23 work started: `NO`
