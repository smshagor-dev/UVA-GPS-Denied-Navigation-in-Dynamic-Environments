# MSCKF Camera-State Marginalization & Feature-Track Retirement — Implementation Closure

The production ordering in `Phase17ESKFEstimator::update_vision` is intentionally:

1. capture/get the current clone,
2. record current feature observations,
3. initialize eligible landmarks,
4. apply MSCKF feature constraints,
5. only then retire clones while the window exceeds its bound.

This ordering is a hard invariant: no camera clone may be removed before the current frame's eligible feature constraints have had an opportunity to update the shadow estimator.

## Required production invariants

- oldest-clone retirement is deterministic,
- retained covariance is the exact retained principal submatrix,
- retained cross-covariances are preserved,
- stale observations referencing the retired clone are removed,
- empty feature tracks are retired,
- no dangling retired-clone reference remains,
- covariance remains finite, symmetric, PSD within tolerance, and dimensionally consistent,
- failed retirement does not publish partial metadata/covariance mutation,
- repeated rollover remains bounded,
- active-estimator authority remains unchanged.

## Current implementation state

The production shadow estimator already enforces the feature-update-before-retirement ordering and performs transactional clone/covariance/track retirement. Remaining closure work is focused on proving the retirement boundary under repeated rollover, feature-loss, rollback, FEJ cleanup, and cross-toolchain validation rather than adding a second estimator authority path.
