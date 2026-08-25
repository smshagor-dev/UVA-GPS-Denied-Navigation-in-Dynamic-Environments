# Phase 19 FEJ Design

Date: July 18, 2026

## Objective

Phase 19 adds First-Estimate Jacobian support to the Phase 17/18 shadow ESKF so that selected measurement Jacobians are evaluated at a frozen first estimate while nominal propagation continues to use the current estimate.

## Scope

Phase 19 is limited to the shadow estimator implementation in `Phase17ESKFEstimator`.

Preserved boundaries:

- active estimator authority remains unchanged
- propagation equations remain unchanged
- Joseph-form covariance update remains unchanged
- transactional update flow remains unchanged
- stationary detection and ZUPT remain unchanged
- no MSCKF, loop closure, or marginalization redesign was introduced

## Configuration

Phase 19 extends estimator validation configuration with a nested FEJ section:

- `fej.enabled`
- `fej.validation_checks`
- `fej.diagnostics_enabled`

Legacy `enable_fej` remains supported, and invalid mixed configuration where legacy FEJ is enabled but nested FEJ is disabled is rejected as invalid configuration.

## FEJ Snapshot Model

Each tracked feature can receive a deterministic FEJ snapshot containing:

- deterministic snapshot id
- quantized feature key derived from world position
- first position estimate
- first orientation estimate
- last observed epoch

Snapshots are stored in Phase 17 shadow-estimator state and are reset when the estimator resets or validation configuration changes.

## Snapshot Lifecycle

Lifecycle rules implemented in Phase 19:

- snapshot created on first observed feature use
- existing snapshot reused for repeated observations of the same quantized feature
- snapshot epoch updated on each observation
- snapshot released when not observed in the current vision update set
- snapshot storage cleared on estimator reset

This keeps the lifecycle deterministic and avoids stale references.

## Diagnostics

Phase 19 exposes FEJ diagnostics through estimator diagnostics and state snapshots:

- `fej_enabled`
- `fej_snapshots_created`
- `fej_snapshots_released`
- `fej_jacobian_evaluations`
- `fej_validation_failures`

Replay reporting also includes FEJ enabled state, snapshot count, Jacobian evaluation count, and validation-failure count.

## Validation Intent

The design is validated by:

- `tests/test_phase19_fej.cpp`
- `tests/ekf_phase19_replay.cpp`
- `artifacts/phase19/validation/`
- `artifacts/phase19/ekf_phase19_replay_report.json`
