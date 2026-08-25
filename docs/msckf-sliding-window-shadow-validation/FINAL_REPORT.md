# Phase 20 Final Report

Date: July 18, 2026

## 1. Objective

Phase 20 adds the MSCKF sliding-window foundation to the Phase 17 shadow estimator so deterministic camera-state storage and eviction can be validated without changing active estimator authority.

## 2. Implementation Summary

Phase 20 added:

- deterministic shadow-only camera-state storage in `Phase17ESKFEstimator`
- lookup by state id and timestamp for replay-safe validation
- oldest-first deterministic eviction and reset handling
- shadow-only `MsckfConfig` runtime wiring through `VIOPipeline` and `EstimatorCoordinator`
- MSCKF diagnostics surfaced through estimator diagnostics and snapshots with explicit `diagnostics_enabled` gating
- dedicated Phase 20 unit and replay coverage

## 3. Authority Preservation

- Active estimator remains Phase 16
- MSCKF state-window logic and MSCKF configuration application run only in the Phase 17 shadow path
- Active estimator and Phase 16 adapter do not receive or store MSCKF configuration
- `EstimatorCoordinator` continues to preserve active-output equivalence

## 4. Replay And Regression Evidence

Executed local evidence exists under `artifacts/phase20/validation/`.

Validated targets:

- `test_ekf`
- `test_phase16_shadow`
- `test_phase17_eskf`
- `test_phase18_zupt`
- `test_phase19_fej`
- `test_phase20_msckf`
- `ekf_phase15_replay`
- `ekf_phase16_replay`
- `ekf_phase17_replay`
- `ekf_phase18_replay`
- `ekf_phase19_replay`
- `ekf_phase20_replay`

Replay artifacts:

- `artifacts/phase15_estimator_replay_report.json`
- `artifacts/phase16/ekf_phase16_replay_report.json`
- `artifacts/phase17/ekf_phase17_replay_report.json`
- `artifacts/phase18/ekf_phase18_replay_report.json`
- `artifacts/phase19/ekf_phase19_replay_report.json`
- `artifacts/phase20/ekf_phase20_replay_report.json`

Phase 20 replay scenarios executed:

- `continuous_camera_stream`
- `long_window`
- `repeated_reset`
- `eviction_stress`

Current Phase 20 replay results:

- all scenarios report `active_equivalent = true`
- all scenarios report `shadow_only_msckf = true`
- all scenarios report `deterministic = true`
- all scenarios report `covariance_finite = true`
- replay metrics demonstrate deterministic growth and oldest-first eviction up to the configured maximum window size

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

- `artifacts/phase20/validation/msvc/summary.json`
- `artifacts/phase20/validation/msvc-werror/summary.json`
- `artifacts/phase20/validation/clang/summary.json`
- `artifacts/phase20/validation/clang-werror/summary.json`
- `artifacts/phase20/validation/gcc/summary.json`
- `artifacts/phase20/validation/gcc-werror/summary.json`
- `artifacts/phase20/validation/clang-format/summary.json`
- `artifacts/phase20/validation/clang-tidy/summary.json`

## 6. Sanitizer Results

- Clang ASan: PASS
- Clang UBSan: PASS
- TSan: PASS
- Project-owned race validation: PASS
- ThreadSanitizer warning count: 0
- Project-owned races found: NO
- Diagnostics-disabled publication remains neutral while the shadow window continues operating

Evidence:

- `artifacts/phase20/validation/asan-ubsan/summary.json`
- `artifacts/phase20/validation/tsan/summary.json`
- `artifacts/phase20/validation/tsan/tsan-summary.json`

## 7. Final Status

- Defects fixed: diagnostics disable control enforcement and active-path MSCKF dependency removal
- Phase 20 implementation status: `COMPLETE`
- Local validation status: `READY`
- Active estimator authority preserved: `YES`
- Phase 21 work started: `NO`
