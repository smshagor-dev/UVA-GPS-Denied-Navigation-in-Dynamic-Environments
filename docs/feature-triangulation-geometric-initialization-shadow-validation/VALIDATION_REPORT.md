# Phase 21 Validation Report

Date: July 18, 2026

## Environment

- Host workspace: `E:\Final Project\drone_swarm`
- Windows lanes: `artifacts/phase21/validation/msvc/summary.json`, `artifacts/phase21/validation/msvc-werror/summary.json`
- WSL Linux lanes: `artifacts/phase21/validation/clang/summary.json`, `artifacts/phase21/validation/gcc/summary.json`, `artifacts/phase21/validation/clang-werror/summary.json`, `artifacts/phase21/validation/gcc-werror/summary.json`, `artifacts/phase21/validation/asan-ubsan/summary.json`, `artifacts/phase21/validation/clang-tidy/summary.json`
- Formatting lane: `artifacts/phase21/validation/clang-format/summary.json`
- TSan lane: `artifacts/phase21/validation/tsan/tsan-summary.json`

## Evidence Directories

- `artifacts/phase21/validation/msvc/`
- `artifacts/phase21/validation/msvc-werror/`
- `artifacts/phase21/validation/clang/`
- `artifacts/phase21/validation/gcc/`
- `artifacts/phase21/validation/clang-werror/`
- `artifacts/phase21/validation/gcc-werror/`
- `artifacts/phase21/validation/clang-format/`
- `artifacts/phase21/validation/clang-tidy/`
- `artifacts/phase21/validation/asan-ubsan/`
- `artifacts/phase21/validation/tsan/`
- `artifacts/phase21/validation/regressions/`
- `artifacts/phase21/validation/replay/`

## Replay Artifacts

- Phase 15 replay report: `artifacts/phase15_estimator_replay_report.json`
- Phase 16 replay report: `artifacts/phase16/ekf_phase16_replay_report.json`
- Phase 17 replay report: `artifacts/phase17/ekf_phase17_replay_report.json`
- Phase 18 replay report: `artifacts/phase18/ekf_phase18_replay_report.json`
- Phase 19 replay report: `artifacts/phase19/ekf_phase19_replay_report.json`
- Phase 20 replay report: `artifacts/phase20/ekf_phase20_replay_report.json`
- Phase 21 replay report: `artifacts/phase21/ekf_phase21_replay_report.json`

Phase 21 replay scenarios present in the local artifact:

- `straight_motion`
- `lateral_motion`
- `rotating_camera`
- `low_parallax`
- `feature_dropout`

## Acceptance Matrix

| Validation item | Result | Evidence |
|---|---|---|
| Shadow-only implementation | PASS | Triangulation ownership and runtime wiring are limited to `Phase17ESKFEstimator`, shadow `MsckfConfig`, `EstimatorCoordinator` shadow configuration, and `VIOPipeline` shadow wiring |
| Active estimator unchanged | PASS | `artifacts/phase21/ekf_phase21_replay_report.json` reports `active_equivalent = true` for all scenarios using the scenario-derived `active_equivalence_method` field based on coordinator traffic carrying the same visual-feature observations |
| Feature observation history implemented | PASS | `Phase17ESKFEstimator` stores per-track observations keyed to deterministic shadow camera-state ids and prunes them on state eviction |
| Normalized bearing storage implemented | PASS | `Phase17ESKFEstimator::pixel_to_bearing` converts accepted pixel observations to normalized camera-frame bearings before track insertion |
| Multi-view triangulation implemented | PASS | `Phase17ESKFEstimator::triangulate_track_locked` accumulates a multi-view linear system across all valid shadow observations and initializes only fully validated landmarks |
| Insufficient-observation rejection implemented | PASS | `test_phase21_triangulation` covers explicit insufficient-observation rejection, and replay summaries persist `rejected_insufficient_observations` |
| Baseline validation implemented | PASS | `triangulate_track_locked` computes maximum pairwise camera baseline and rejects tracks below `minimum_baseline`; `low_parallax` reports `triangulation_successes = 0` and `rejected_small_baseline = 12` |
| Reprojection validation implemented | PASS | `triangulate_track_locked` reprojects the solved landmark into every observing camera and rejects candidates above `maximum_reprojection_error` |
| Positive depth validation implemented | PASS | `triangulate_track_locked` rejects negative or too-shallow depth via `rejected_negative_depth` and over-range depth via `rejected_depth_range` |
| Config validation implemented | PASS | `validate_msckf_config` and runtime parsing reject invalid triangulation thresholds and disallow triangulation when shadow MSCKF is disabled |
| Triangulation diagnostics exposed | PASS | `EKFDiagnostics`, `EstimatorStateSnapshot`, and Phase 21 replay summaries publish track creation/removal counters, attempts, successes, failures, rejection counters, and active landmark counts |
| Deterministic replay preserved | PASS | `artifacts/phase21/ekf_phase21_replay_report.json` reports `deterministic = true` for `straight_motion`, `lateral_motion`, `rotating_camera`, `low_parallax`, and `feature_dropout` |
| Shadow-only triangulation independently verified | PASS | `artifacts/phase21/ekf_phase21_replay_report.json` reports `shadow_only_triangulation = true` for every scenario and persists the derivation in `shadow_only_evidence` |
| Feature-dropout cleanup verified | PASS | `feature_dropout` persists `feature_tracks_removed = 1`, `active_landmarks_before_cleanup = 3`, `active_landmarks_after_cleanup = 2`, `stale_tracks_found = false`, `stale_landmarks_found = false`, `dropped_feature_track_id = 1`, and `cleanup_frame_index = 4` |
| Finite landmark coordinates | PASS | `artifacts/phase21/ekf_phase21_replay_report.json` reports `landmarks_finite = true` for all scenarios, including rejected low-parallax tracks |
| Phase 15 regression | PASS | `artifacts/phase21/validation/regressions/summary.json`, `artifacts/phase21/validation/replay/summary.json`, and `artifacts/phase15_estimator_replay_report.json` |
| Phase 16 regression | PASS | `artifacts/phase21/validation/regressions/summary.json`, `artifacts/phase21/validation/replay/summary.json`, and `artifacts/phase16/ekf_phase16_replay_report.json` |
| Phase 17 regression | PASS | `artifacts/phase21/validation/regressions/summary.json`, `artifacts/phase21/validation/replay/summary.json`, and `artifacts/phase21/validation/tsan/tsan-summary.json` |
| Phase 18 regression | PASS | `artifacts/phase21/validation/regressions/summary.json`, `artifacts/phase21/validation/replay/summary.json`, and `artifacts/phase18/ekf_phase18_replay_report.json` |
| Phase 19 regression | PASS | `artifacts/phase21/validation/regressions/summary.json`, `artifacts/phase21/validation/replay/summary.json`, and `artifacts/phase19/ekf_phase19_replay_report.json` |
| Phase 20 regression | PASS | `artifacts/phase21/validation/regressions/summary.json`, `artifacts/phase21/validation/replay/summary.json`, and `artifacts/phase20/ekf_phase20_replay_report.json` |
| Phase 21 unit tests | PASS | `artifacts/phase21/validation/msvc/summary.json`, `artifacts/phase21/validation/clang/summary.json`, `artifacts/phase21/validation/gcc/summary.json`, and `artifacts/phase21/validation/asan-ubsan/summary.json` |
| Replay PASS | PASS | `artifacts/phase21/validation/replay/summary.json` and `artifacts/phase21/ekf_phase21_replay_report.json` |
| MSVC | PASS | `artifacts/phase21/validation/msvc/summary.json` |
| Clang | PASS | `artifacts/phase21/validation/clang/summary.json` |
| GCC | PASS | `artifacts/phase21/validation/gcc/summary.json` |
| warnings-as-errors | PASS | `artifacts/phase21/validation/msvc-werror/summary.json`, `artifacts/phase21/validation/clang-werror/summary.json`, and `artifacts/phase21/validation/gcc-werror/summary.json` |
| clang-format | PASS | `artifacts/phase21/validation/clang-format/summary.json` |
| clang-tidy | PASS | `artifacts/phase21/validation/clang-tidy/summary.json` |
| ASan | PASS | `artifacts/phase21/validation/asan-ubsan/summary.json` |
| UBSan | PASS | `artifacts/phase21/validation/asan-ubsan/summary.json` |
| TSan | PASS | `artifacts/phase21/validation/tsan/tsan-summary.json` reports `final_status = "PASS"` |
| Project-owned race validation | PASS | `artifacts/phase21/validation/tsan/tsan-summary.json` reports `project_owned_races_found = false`, `third_party_races_found = false`, and `warning_count = 0` |
| Documentation complete | PASS | `docs/feature-triangulation-geometric-initialization-shadow-validation/` contains triangulation design, geometric initialization, validation, final report, and remaining-risks documents |

## Final Validation Decision

- Phase 21 implementation status: `COMPLETE`
- Local validation status: `READY`
- Active estimator authority preserved: `YES`
- Phase 22 work started: `NO`
