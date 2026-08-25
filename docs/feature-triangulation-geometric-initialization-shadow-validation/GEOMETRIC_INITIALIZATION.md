# Phase 21 Geometric Initialization

Date: July 18, 2026

## 1. Objective

Phase 21 establishes a robust geometric gate for shadow-only landmark initialization so the MSCKF sliding window can accumulate valid 3D features without affecting the active estimator.

## 2. Initialization Flow

For each shadow feature track:

1. Wait until the configured minimum observation count is reached.
2. Reuse only observations whose camera states still exist in the current shadow window.
3. Measure the maximum pairwise camera baseline.
4. Solve a multi-view linear triangulation estimate in world coordinates.
5. Validate geometry before accepting the landmark.

## 3. Baseline Validation

Baseline is computed as the maximum Euclidean distance between any two valid observing camera positions in the shadow window.

Rejection rule:

- reject when `baseline < minimum_baseline`

This prevents low-parallax tracks from being promoted to active shadow landmarks.

## 4. Positive Depth Validation

After solving the world landmark estimate, the point is transformed back into every observing camera frame.

Rejection rules:

- reject when camera-frame depth is not finite
- reject when depth is less than `minimum_depth`
- reject when depth is greater than `maximum_depth`

The `rejected_negative_depth` diagnostic covers both negative and out-of-range depth failures.

## 5. Reprojection Validation

Each accepted landmark candidate is reprojected back into every observing camera using the same camera intrinsics used to form the stored normalized bearings.

Rejection rule:

- reject when reprojection error exceeds `maximum_reprojection_error`

This keeps unstable or numerically inconsistent solutions out of the initialized landmark set.

## 6. Failure Modes

Phase 21 explicitly rejects:

- insufficient observations
- missing camera-state support after window eviction
- degenerate geometry
- insufficient baseline
- negative or invalid depth
- excessive reprojection error
- non-finite landmark coordinates

Rejected candidates are not inserted into the initialized landmark set.

## 7. Replay Evidence

Local replay artifacts confirm the intended behavior:

- `straight_motion`: successful triangulation with finite landmarks and active-output equivalence preserved
- `lateral_motion`: successful triangulation under cross-track motion
- `rotating_camera`: mixed success/failure behavior with explicit small-baseline rejections before later successful initialization
- `low_parallax`: repeated rejection with zero active landmarks, demonstrating baseline gating
- `feature_dropout`: successful initialization after partial observation loss

## 8. Limitations

Phase 21 is a geometric initialization phase only. It does not yet add:

- feature residual assembly
- MSCKF null-space projection
- feature-constraint update steps
- marginalization redesign

Those remain future work.
