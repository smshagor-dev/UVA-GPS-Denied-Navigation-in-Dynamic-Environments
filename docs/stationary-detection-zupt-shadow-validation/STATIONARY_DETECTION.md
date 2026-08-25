# Phase 18 Stationary Detection

Date: July 18, 2026

## Objective

Phase 18 adds IMU-only stationary detection to the Phase 17 shadow estimator so that automatic ZUPT can run without changing the active estimator authority.

## Shadow-Only Scope

- Implemented only in `Phase17ESKFEstimator`
- Activated only through the shadow `Phase17StateEstimatorAdapter`
- Active `EKFStateEstimatorAdapter` remains unchanged
- `EstimatorCoordinator` still keeps Phase 16 active and Phase 17 shadow isolated

## Detector Inputs

The detector evaluates each accepted IMU sample using:

- accelerometer magnitude error: `abs(||a|| - 9.81)`
- gyroscope magnitude: `||w||`

## Configuration

Stationary detection uses `EstimatorValidationConfig::stationary_detector`:

- `enabled`
- `accel_threshold_mps2`
- `gyro_threshold_rads`
- `window_size`
- `enter_count`
- `exit_count`
- `minimum_stationary_time_s`
- `accel_exit_threshold_mps2`
- `gyro_exit_threshold_rads`

Default runtime wiring was added through `RuntimeModeFileConfig` and mapped in `src/main.cpp`.

## Enter Logic

- Each sample is marked as an enter candidate only if both enter thresholds pass
- A bounded deque stores the recent observation window
- Stationary entry requires at least `enter_count` enter-candidate samples inside the window
- Entry is not committed until the earliest enter-candidate timestamp has remained valid for at least `minimum_stationary_time_s`

## Exit Logic And Hysteresis

- Exit uses separate, looser thresholds: `accel_exit_threshold_mps2` and `gyro_exit_threshold_rads`
- While stationary, the detector stays latched until the window contains fewer than `exit_count` exit-candidate samples
- This hysteresis prevents rapid state flipping around the enter thresholds

## Diagnostics Exposed

Phase 18 extends diagnostics and snapshots with:

- `stationary_detected`
- `stationary_duration_s`
- `detector_state_change_count`
- `stationary_intervals`
- `zupt_accepted_count`
- `zupt_rejected_count`

## Unit Coverage

`tests/test_phase18_zupt.cpp` verifies:

- perfect stationary IMU entry
- noisy stationary IMU entry
- slow-motion rejection
- constant-rotation rejection
- hysteresis and exit transition behavior
- active-estimator authority preservation with shadow automatic ZUPT
