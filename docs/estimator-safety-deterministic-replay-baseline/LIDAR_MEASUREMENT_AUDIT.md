# LiDAR Measurement Audit

Date: July 17, 2026

## Existing Path Inspected

Files inspected:

- `src/vio/VIOPipeline.cpp`
- `include/vio/EKFEstimator.hpp`
- `src/vio/EKFEstimator.cpp`

## Existing Behavior Found

The LiDAR handler computed a median `ground_z` from cloud points, computed `height = pose.position.z() - ground_z`, and then called:

- `ekf_.update_depth(pose.position.z(), 0.05)`

This call path did not pass the derived height, a plane-fit altitude model, or a clearly defined world-frame altitude observation. It effectively reused the estimator’s current `z` position as the measurement.

## Phase 15 Conclusion

- the semantics are ambiguous for estimator correction
- the active measurement is not a safe, explicit altitude observation model
- Phase 15 therefore disables estimator depth correction by default

## Resulting Behavior

- raw LiDAR processing in the pipeline remains present
- estimator depth correction is guarded by `lidar_depth_correction_enabled`
- default configuration leaves that correction disabled
- disabled use increments diagnostics and does not mutate state

## Non-Goals

- no new ground-plane model
- no scan matching
- no world-altitude redesign
- no LiDAR-to-map correction model
