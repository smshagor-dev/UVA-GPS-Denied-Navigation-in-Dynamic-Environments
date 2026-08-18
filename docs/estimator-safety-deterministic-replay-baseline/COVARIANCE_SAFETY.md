# Covariance Safety

Date: July 17, 2026

## Existing Ordering Confirmed

The EKF uses a `15`-dimensional error-state covariance with block ordering:

- position error `[0:2]`
- velocity error `[3:5]`
- attitude error `[6:8]`
- accelerometer bias error `[9:11]`
- gyroscope bias error `[12:14]`

## Phase 15 Changes

- propagation validates candidate covariance before commit
- covariance is symmetrized with `0.5 * (P + P^T)` after accepted propagation and accepted correction
- materially negative diagonal variances are treated as failure
- covariance failures increment structured diagnostics
- failed propagation and failed update keep the previous valid covariance

## Correction Method

Active linear updates in the EKF now use Joseph-form covariance correction:

`P+ = (I - KH) P (I - KH)^T + K R K^T`

This replaced the unstable subtractive form on:

- visual pose correction
- manual ZUPT correction
- explicit depth correction when manually enabled
- per-feature vision correction path

## Remaining Limitation

The replay report currently summarizes covariance via trace and symmetry checks, not a full PSD proof.
