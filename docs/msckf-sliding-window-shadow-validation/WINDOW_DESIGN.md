# Phase 20 Window Design

Date: July 18, 2026

## Scope

Phase 20 adds the MSCKF sliding-window foundation only to the Phase 17 shadow estimator.

The implementation does not add:

- MSCKF feature-constraint updates
- triangulation
- marginalization updates
- loop closure
- pose graph optimization

## Configuration

Phase 20 keeps shared validation settings in `EstimatorValidationConfig` and applies MSCKF settings through a shadow-only `MsckfConfig`:

- `enabled`
- `max_camera_states`
- `eviction_policy`
- `diagnostics_enabled`

Safe defaults are present, and invalid configuration is rejected when:

- `enabled` is true and `max_camera_states == 0`
- `enabled` is true and `eviction_policy != "oldest_first"`

Runtime wiring is limited to the shadow-estimator validation path through:

- `RuntimeModeConfig`
- `src/main.cpp`
- `VIOPipeline::set_shadow_msckf_config`
- `EstimatorCoordinator::configure_shadow_msckf`
- `Phase17ESKFEstimator`

The active EKF implementation and Phase 16 adapter do not receive, store, validate, or execute MSCKF configuration.

## Camera State Contents

Each shadow-only camera state stores:

- deterministic state id
- quantized timestamp key
- timestamp in seconds
- position
- orientation
- velocity reference

## Deterministic Window Policy

The window uses deterministic insertion and deterministic eviction:

- accepted visual updates capture one camera state at the latest accepted IMU timestamp
- duplicate timestamp keys are ignored to prevent duplicate state creation
- insertion order is preserved with `msckf_state_order_`
- timestamp lookup is preserved with `msckf_timestamp_to_state_id_`
- oldest-first eviction is enforced whenever the configured maximum size is exceeded

The implementation resets the entire window on estimator reset and restarts state ids from `1`.

## Diagnostics

Phase 20 extends estimator diagnostics and state snapshots with:

- `msckf_window_size`
- `msckf_states_created`
- `msckf_states_removed`
- `msckf_oldest_state_age_s`
- `msckf_deterministic_evictions`

`diagnostics_enabled` gates diagnostic tracking and publication only. When diagnostics are disabled:

- the shadow MSCKF window still inserts, looks up, resets, and evicts deterministically
- published counters remain `0`
- `msckf_oldest_state_age_s` remains `0.0`
- reset clears any stale diagnostic values before the next publication

Replay summaries additionally report:

- maximum window size
- created states
- removed states
- final window size
