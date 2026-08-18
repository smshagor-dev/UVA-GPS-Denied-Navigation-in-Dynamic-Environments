# Phase 20 State Lifecycle

Date: July 18, 2026

## Creation

Camera-state creation happens only inside `Phase17ESKFEstimator` after accepted visual updates:

- `update_vision(...)`
- `update_visual_pose(...)`

The capture path calls `maybe_capture_msckf_camera_state_locked(...)` using the latest accepted IMU timestamp when available.

## Lookup

The lifecycle exposes deterministic test-only inspection helpers for:

- ordered state ids
- ordered timestamps
- lookup by state id
- lookup by timestamp

These helpers are used by `test_phase20_msckf` and `ekf_phase20_replay`.

## Eviction

When the configured maximum window size is exceeded:

1. the oldest state id is removed from `msckf_state_order_`
2. the corresponding timestamp lookup entry is erased
3. the camera-state record is erased
4. removal and deterministic-eviction diagnostics are incremented

No feature track owns camera-state lifetime in Phase 20.

## Reset

`reset_msckf_locked()` clears:

- ordered ids
- stored camera states
- timestamp lookup map
- next id counter
- MSCKF diagnostics

This guarantees replay-safe restart behavior and avoids stale references across resets.

## Replay Behavior

Phase 20 replay validation confirms:

- deterministic state ids across repeated runs
- deterministic window contents across repeated runs
- deterministic eviction ordering
- finite covariance preservation
- active-estimator equivalence
- shadow-only execution
