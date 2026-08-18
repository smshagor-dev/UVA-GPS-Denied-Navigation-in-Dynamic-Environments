# Phase 18 ZUPT Design

Date: July 18, 2026

## Objective

Phase 18 adds automatic zero-velocity updates to the Phase 17 shadow ESKF during detected stationary intervals.

## Measurement Model

When the stationary detector is active, the shadow estimator applies:

- measurement: `v = 0`
- innovation: `r = -v`
- Jacobian: velocity rows only
- measurement covariance: `sigma_v^2 I`

Implemented in `Phase17ESKFEstimator::update_zupt_locked(...)`.

## Safety Properties Preserved

The Phase 18 ZUPT path keeps the existing Phase 15 and Phase 17 safety guarantees:

- Joseph-form covariance update
- candidate-state validation before commit
- covariance symmetry enforcement
- non-finite rejection
- invalid-configuration rejection
- transactional no-commit on failure

## Configuration

ZUPT uses:

- `EstimatorValidationConfig::zupt.enabled`
- `EstimatorValidationConfig::zupt.velocity_noise_mps`
- `EstimatorValidationConfig::zupt.max_update_rate_hz`

Legacy `enable_automatic_zupt` and `zupt_sigma_velocity_mps` remain supported as compatibility inputs, but Phase 18 behavior is configured through the explicit nested `stationary_detector` and `zupt` sections.

## Automatic Triggering

- Automatic ZUPT runs only when the shadow detector reports stationary
- The active estimator never runs automatic ZUPT
- `max_update_rate_hz` limits repeated corrections during long stationary periods
- Accepted ZUPT updates are counted per stationary interval

## Replay Evidence

`tests/ekf_phase18_replay.cpp` executes:

- `stationary_long`
- `stop_and_go`
- `intermittent_motion`

Observed artifact: `artifacts/phase18/ekf_phase18_replay_report.json`

Results recorded in the artifact:

- all scenarios: `active_equivalent = true`
- all scenarios: `shadow_success = true`
- all scenarios: `deterministic = true`
- all scenarios: `covariance_finite = true`
- all scenarios: `zupt_only_when_stationary = true`

Scenario counts from the current artifact:

- `stationary_long`: `stationary_interval_count = 1`, `zupt_count = 45`
- `stop_and_go`: `stationary_interval_count = 1`, `zupt_count = 42`
- `intermittent_motion`: `stationary_interval_count = 2`, `zupt_count = 50`
