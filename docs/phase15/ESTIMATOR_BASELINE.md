# Estimator Baseline

The Phase 15 baseline keeps the existing real-time ESKF active while adding deterministic validation hooks around it.

## Baseline protections

- IMU propagation rejects non-finite values, backward timestamps, and `dt` values above the configured bound.
- Measurement corrections use LDLT solves plus Joseph-form covariance updates.
- Covariance is symmetrized and bounded after propagation and correction.
- Diagnostics expose accepted/rejected updates, invalid input counts, timestamp violations, covariance metrics, and health state.
- The active LiDAR height correction is disabled until its frame semantics are observable and explicitly calibrated.

## Drift semantics

`drift_m` remains available for compatibility, but in the current implementation it represents estimated position uncertainty derived from covariance, not measured drift against ground truth.

## Visual orientation

The frontend currently produces relative orientation estimates, but Phase 15 does not feed them into the EKF update. That behavior is intentional until a bounded orientation-aware interface is introduced in a later phase.
