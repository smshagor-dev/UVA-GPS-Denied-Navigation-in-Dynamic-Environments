# Phase 21 Triangulation Design

Date: July 18, 2026

## 1. Scope

Phase 21 adds shadow-only feature observation history and geometric landmark initialization for the MSCKF sliding-window foundation introduced in Phase 20.

The implementation is intentionally limited to:

- feature-track history capture
- normalized bearing storage
- multi-view landmark initialization
- baseline, depth, and reprojection validation
- diagnostics and replay evidence

The implementation intentionally does not add:

- MSCKF feature-constraint updates
- null-space projection
- residual stacking
- marginalization redesign
- active-estimator authority changes

## 2. Shadow-Only Placement

Triangulation is integrated only into the Phase 17 shadow estimator path:

- `Phase17ESKFEstimator`
- shadow `MsckfConfig`
- `EstimatorCoordinator` shadow configuration flow
- `VIOPipeline` shadow configuration wiring

The active estimator remains unchanged and continues to own published state.

## 3. Observation History

Each tracked feature is stored in a `FeatureTrack` keyed by the same deterministic quantized feature key family already used by FEJ snapshot ownership.

Per observation, the shadow estimator stores:

- shadow camera-state id
- original pixel measurement
- normalized camera-frame bearing

Normalized bearings are computed from camera pixels using the active camera intrinsics:

- `bearing_c = normalize(K^-1 * [u v 1]^T)`

Observation capture occurs only after a vision update is accepted and only when shadow MSCKF plus triangulation are enabled.

## 4. Camera-State Dependency

Phase 21 reuses the deterministic Phase 20 shadow window:

- accepted vision updates capture or reuse a shadow camera state
- feature observations reference that camera-state id
- track observations are pruned when camera states are evicted
- MSCKF reset clears camera states, track history, diagnostics, and deterministic ids

This keeps feature initialization replay-safe and aligned with the existing window lifecycle.

## 5. Triangulation Algorithm

For a feature track with at least `minimum_observations` valid observations:

1. Collect only observations whose referenced shadow camera states are still present.
2. Compute the maximum pairwise camera-position separation across those valid observations.
3. Reject the track if the measured baseline is below `minimum_baseline`.
4. Convert each stored bearing into a world-frame direction using the stored camera-state orientation.
5. Accumulate the linear system:

   - `A += I - d d^T`
   - `b += (I - d d^T) p`

   where `d` is the world-frame ray direction and `p` is the camera position.

6. Solve the self-adjoint system using Eigen's `SelfAdjointEigenSolver`.
7. Reject degenerate systems when the minimum eigenvalue is below the configured tolerance.
8. Recover the landmark estimate in world coordinates from the solved linear system.
9. Validate positive depth and configured depth bounds in every observing camera.
10. Reproject the landmark into every observing camera and reject if any reprojection error exceeds `maximum_reprojection_error`.

Only fully validated landmarks are marked initialized.

## 6. Configuration

Phase 21 extends `MsckfConfig::TriangulationConfig` and runtime parsing with:

- `enabled`
- `minimum_observations`
- `minimum_baseline`
- `maximum_reprojection_error`
- `minimum_depth`
- `maximum_depth`

Safe defaults:

- `enabled = false`
- `minimum_observations = 2`
- `minimum_baseline = 0.05`
- `maximum_reprojection_error = 2.5`
- `minimum_depth = 0.1`
- `maximum_depth = 50.0`

Configuration validation rejects:

- fewer than two required observations
- negative baseline threshold
- negative reprojection threshold
- non-finite depth or reprojection limits
- `maximum_depth < minimum_depth`
- triangulation enabled while shadow MSCKF is disabled

## 7. Diagnostics

Phase 21 adds the following diagnostics and snapshot fields:

- `feature_tracks`
- `feature_tracks_created`
- `feature_tracks_removed`
- `triangulation_attempts`
- `triangulation_successes`
- `triangulation_failures`
- `rejected_insufficient_observations`
- `rejected_small_baseline`
- `rejected_negative_depth`
- `rejected_depth_range`
- `rejected_degenerate_geometry`
- `rejected_non_finite_input`
- `rejected_reprojection`
- `active_landmarks`

Diagnostics are published through the existing shadow-estimator diagnostics path and replay reports.

## 8. Determinism Guarantees

Determinism is preserved by:

- deterministic camera-state ids from Phase 20
- deterministic feature-track keys
- deterministic observation insertion order from replayed measurements
- replay-safe pruning on state eviction
- full reset of track state and ids on MSCKF reset

Phase 21 replay artifacts confirm deterministic checksums across repeated runs.

## 9. Accepted Limitations

Phase 21 initializes landmarks but does not yet consume them in MSCKF feature updates. Landmark creation is therefore a shadow-only geometric readiness layer for later phases rather than a new state-correction path.
