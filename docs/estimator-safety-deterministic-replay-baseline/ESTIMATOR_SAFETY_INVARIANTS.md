# Estimator Safety Invariants

Date: July 17, 2026

## State Invariants

- accepted state must remain finite
- quaternion coefficients must be finite
- quaternion norm must stay above the configured minimum norm before normalization
- position, velocity, accelerometer bias, and gyroscope bias must remain finite
- rejected propagation and rejected updates must leave state unchanged

## Covariance Invariants

- covariance dimension remains `15 x 15`
- covariance entries must remain finite
- covariance is symmetrized after accepted propagation and accepted correction
- materially negative diagonal variances are rejected
- failed correction or failed propagation must not overwrite covariance with identity

## Timing Invariants

- timestamped IMU ingestion requires finite timestamps
- duplicate timestamps are rejected explicitly
- backward timestamps are rejected explicitly
- time steps below configured minimum are rejected explicitly
- time steps above configured maximum are rejected explicitly
- rejected timestamps do not advance the last accepted estimator timestamp

## Measurement Invariants

- accepted measurements must be finite
- measurement covariance must be finite and dimensionally consistent
- asymmetric or negative-variance measurement covariance is rejected
- disabled or unsupported measurement semantics are rejected explicitly
- rejected measurements do not partially mutate state or covariance
