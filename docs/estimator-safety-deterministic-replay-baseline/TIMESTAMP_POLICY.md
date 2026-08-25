# Timestamp Policy

Date: July 17, 2026

## Estimator-Facing IMU Contract

Phase 15 adds `process_imu_measurement(accel, gyro, timestamp_s)` as the timestamp-validating IMU entry point used by `VIOPipeline`.

## Policy

- first accepted IMU sample establishes the last accepted timestamp and does not propagate
- forward finite timestamps with `dt` inside configured bounds are accepted
- duplicate timestamps are rejected with `rejected_duplicate_timestamp`
- backward timestamps are rejected with `rejected_backward_timestamp`
- non-finite timestamps are rejected with `rejected_invalid_timestamp`
- `dt < min_imu_dt_s` is rejected with `rejected_time_step_too_small`
- `dt > max_imu_dt_s` is rejected with `rejected_time_step_too_large`

## Phase 15 Defaults

- `min_imu_dt_s = 1e-6`
- `max_imu_dt_s = 0.1`

## Commit Policy

- rejected timestamps do not propagate state
- rejected timestamps do not advance the last accepted timestamp
- diagnostics record the last received timestamp and the last accepted timestamp separately
