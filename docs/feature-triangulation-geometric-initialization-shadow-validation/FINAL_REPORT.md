# Phase 21 Final Report

Date: July 18, 2026

## 1. Objective

Phase 21 adds robust shadow-only feature triangulation and geometric initialization on top of the Phase 20 MSCKF sliding window so multi-view feature tracks can produce validated 3D landmarks without changing active estimator authority.

## 2. Implementation Summary

Phase 21 added:

- shadow-only feature-track history in `Phase17ESKFEstimator`
- normalized bearing storage derived from accepted pixel observations
- configurable multi-view triangulation with minimum observation, baseline, depth, and reprojection gates
- deterministic pruning of feature observations when shadow camera states are evicted or reset
- triangulation diagnostics surfaced through estimator diagnostics, state snapshots, and replay summaries
- dedicated Phase 21 unit and replay coverage

## 3. Authority Preservation

- Active estimator remains Phase 16
- Triangulation configuration is wired only into the shadow MSCKF path
- Active estimator and Phase 16 adapter do not receive triangulation state or landmark ownership
- `EstimatorCoordinator` continues to preserve active-output equivalence

## 4. Replay And Regression Evidence

Executed local evidence exists under `artifacts/phase21/validation/`.

Validated targets:

- `test_ekf`
- `test_phase16_shadow`
- `test_phase17_eskf`
- `test_phase18_zupt`
- `test_phase19_fej`
- `test_phase20_msckf`
- `test_phase21_triangulation`
- `ekf_phase15_replay`
- `ekf_phase16_replay`
- `ekf_phase17_replay`
- `ekf_phase18_replay`
- `ekf_phase19_replay`
- `ekf_phase20_replay`
- `ekf_phase21_replay`

Replay artifacts:

- `artifacts/phase15_estimator_replay_report.json`
- `artifacts/phase16/ekf_phase16_replay_report.json`
- `artifacts/phase17/ekf_phase17_replay_report.json`
- `artifacts/phase18/ekf_phase18_replay_report.json`
- `artifacts/phase19/ekf_phase19_replay_report.json`
- `artifacts/phase20/ekf_phase20_replay_report.json`
- `artifacts/phase21/ekf_phase21_replay_report.json`

Phase 21 replay scenarios executed:

- `straight_motion`
- `lateral_motion`
- `rotating_camera`
- `low_parallax`
- `feature_dropout`

Current Phase 21 replay results:

- all scenarios report `active_equivalent = true`
- all scenarios report `shadow_only_triangulation = true`
- all scenarios report `deterministic = true`
- all scenarios report `covariance_finite = true`
- all scenarios report `landmarks_finite = true`
- `low_parallax` correctly reports zero successful triangulations and zero active landmarks
- `feature_dropout` proves cleanup with `feature_tracks_removed = 1`, `stale_tracks_found = false`, `stale_landmarks_found = false`, and deterministic dropped-track removal at `cleanup_frame_index = 4`
- active-estimator independence is now scenario-derived through replay `active_equivalence_method` rather than a separate IMU-only control check
- shadow-only behavior is now scenario-derived through replay `shadow_only_evidence` plus zero active-path triangulation counters

## 5. Compiler And Analysis Results

- MSVC Release: PASS
- MSVC warnings-as-errors: PASS
- WSL Clang Release: PASS
- WSL Clang warnings-as-errors: PASS
- WSL GCC Release: PASS
- WSL GCC warnings-as-errors: PASS
- clang-format: PASS
- clang-tidy: PASS

Evidence:

- `artifacts/phase21/validation/msvc/summary.json`
- `artifacts/phase21/validation/msvc-werror/summary.json`
- `artifacts/phase21/validation/clang/summary.json`
- `artifacts/phase21/validation/clang-werror/summary.json`
- `artifacts/phase21/validation/gcc/summary.json`
- `artifacts/phase21/validation/gcc-werror/summary.json`
- `artifacts/phase21/validation/clang-format/summary.json`
- `artifacts/phase21/validation/clang-tidy/summary.json`

## 6. Sanitizer Results

- Clang ASan: PASS
- Clang UBSan: PASS
- TSan: PASS
- Project-owned race validation: PASS
- ThreadSanitizer warning count: 0
- Project-owned races found: NO

Evidence:

- `artifacts/phase21/validation/asan-ubsan/summary.json`
- `artifacts/phase21/validation/tsan/tsan-summary.json`

## 7. Final Status

- Defects fixed: missing Phase 21 rejection/cleanup tests, scenario-derived active-equivalence replay evidence, independent shadow-only replay evidence, feature-dropout cleanup evidence, and stale documentation/default-value mismatches
- Phase 21 implementation status: `COMPLETE`
- Local validation status: `READY`
- Active estimator authority preserved: `YES`
- Phase 22 work started: `NO`
