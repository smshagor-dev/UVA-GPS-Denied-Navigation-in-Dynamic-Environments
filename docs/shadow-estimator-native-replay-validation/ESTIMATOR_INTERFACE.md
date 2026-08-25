# Estimator Interface

Date: July 17, 2026
Status: Implemented

## Implemented Contract

Phase 16 now exposes a concrete `StateEstimator` abstraction in `include/vio/StateEstimator.hpp`.

The interface currently supports:

- validation configuration
- initialize, reset, and stop lifecycle control
- ordered processing of typed `MeasurementEnvelope` values
- immutable `EstimatorStateSnapshot` publication
- estimator diagnostics, identity, and implementation version

## Current Adapter

`EKFStateEstimatorAdapter` wraps the existing `EKFEstimator` without duplicating estimator math.

The adapter preserves:

- Phase 15 IMU timestamp validation
- finite-value and covariance checks
- manual ZUPT handling
- disabled-by-default LiDAR depth correction behavior
- active-state authority
- no feedback path from shadow state into active state

## Supported Measurement Types

The current interface intentionally supports only measurement types that are already modeled in the project:

- IMU propagation
- visual pose correction
- manual ZUPT
- LiDAR depth correction
- disabled LiDAR observation notification

Unsupported or invalid measurements return explicit `EstimatorOperationResult` codes.

## Validation Notes

- `test_phase16_shadow` verifies direct EKF versus adapter equivalence for IMU propagation.
- `ekf_phase16_replay` verifies active-only versus shadow-enabled active equivalence across multiple scenarios.
- unsupported envelopes and invalid snapshots are contained without mutating active state.
