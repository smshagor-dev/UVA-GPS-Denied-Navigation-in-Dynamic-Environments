# Phase 20 Validation Report

Date: July 18, 2026

## Environment

- Host workspace: `E:\Final Project\drone_swarm`
- Host evidence capture: `artifacts/phase20/validation/environment/summary.json`
- Windows lanes: `artifacts/phase20/validation/msvc/summary.json`, `artifacts/phase20/validation/msvc-werror/summary.json`
- WSL Linux lanes: `artifacts/phase20/validation/clang/summary.json`, `artifacts/phase20/validation/gcc/summary.json`, `artifacts/phase20/validation/clang-werror/summary.json`, `artifacts/phase20/validation/gcc-werror/summary.json`, `artifacts/phase20/validation/asan-ubsan/summary.json`, `artifacts/phase20/validation/clang-tidy/summary.json`
- TSan lane: `artifacts/phase20/validation/tsan/summary.json`, `artifacts/phase20/validation/tsan/tsan-summary.json`

## Evidence Directories

- `artifacts/phase20/validation/environment/`
- `artifacts/phase20/validation/msvc/`
- `artifacts/phase20/validation/msvc-werror/`
- `artifacts/phase20/validation/clang/`
- `artifacts/phase20/validation/gcc/`
- `artifacts/phase20/validation/clang-werror/`
- `artifacts/phase20/validation/gcc-werror/`
- `artifacts/phase20/validation/clang-format/`
- `artifacts/phase20/validation/clang-tidy/`
- `artifacts/phase20/validation/asan-ubsan/`
- `artifacts/phase20/validation/tsan/`
- `artifacts/phase20/validation/regressions/`
- `artifacts/phase20/validation/replay/`

## Replay Artifacts

- Phase 15 replay report: `artifacts/phase15_estimator_replay_report.json`
- Phase 16 replay report: `artifacts/phase16/ekf_phase16_replay_report.json`
- Phase 17 replay report: `artifacts/phase17/ekf_phase17_replay_report.json`
- Phase 18 replay report: `artifacts/phase18/ekf_phase18_replay_report.json`
- Phase 19 replay report: `artifacts/phase19/ekf_phase19_replay_report.json`
- Phase 20 replay report: `artifacts/phase20/ekf_phase20_replay_report.json`

Phase 20 replay scenarios present in the local artifact:

- `continuous_camera_stream`
- `long_window`
- `repeated_reset`
- `eviction_stress`

## Acceptance Matrix

| Validation item | Result | Evidence |
|---|---|---|
| Shadow-only implementation | PASS | MSCKF state ownership and configuration application exist only in `Phase17ESKFEstimator`, `EstimatorCoordinator::configure_shadow_msckf`, and `VIOPipeline::set_shadow_msckf_config`; active-path regressions passed in `artifacts/phase20/validation/regressions/summary.json` |
| Active-path MSCKF dependency removed | PASS | `Phase20MsckfIsolationTest.ActiveEstimatorDoesNotDependOnMsckfConfig` verifies the active estimator stays initialized and publishes zero MSCKF fields while shadow-only MSCKF runs |
| Active estimator unchanged | PASS | `artifacts/phase20/ekf_phase20_replay_report.json` reports `active_equivalent = true` and `shadow_only_msckf = true` for all Phase 20 replay scenarios |
| Sliding window implemented | PASS | Deterministic camera-state creation, storage, lookup, reset, and eviction are implemented in `Phase17ESKFEstimator` and exercised by `test_phase20_msckf` |
| Deterministic state IDs | PASS | `test_phase20_msckf` covers repeated-run id stability, and `artifacts/phase20/ekf_phase20_replay_report.json` reports `deterministic = true` for all scenarios |
| Deterministic insertion | PASS | Ordered state creation is covered by `test_phase20_msckf`, and replay artifacts preserve identical checksums across repeated runs |
| Deterministic eviction | PASS | `test_phase20_msckf` covers oldest-first eviction and diagnostics, and replay `eviction_stress` reports `states_removed = 17` with `maximum_window_size = 3` |
| Reset handling implemented | PASS | `test_phase20_msckf` covers reset clearing and id restart, and replay `repeated_reset` remains deterministic with finite covariance |
| Diagnostics disable control enforced | PASS | `DiagnosticsDisabledKeepsWindowFunctionalAndCountersNeutral`, `ResetWithDiagnosticsDisabledPreservesNeutralPublication`, and `DiagnosticsDisabledReplayRemainsDeterministic` verify window behavior remains active while published counters and ages stay neutral |
| Config validation implemented | PASS | `test_phase20_msckf` rejects zero-sized windows and unsupported eviction policies when shadow MSCKF is enabled |
| Finite covariance preserved | PASS | `artifacts/phase20/ekf_phase20_replay_report.json` reports `covariance_finite = true` for all scenarios, and sanitizer replay lanes passed in `artifacts/phase20/validation/asan-ubsan/summary.json` and `artifacts/phase20/validation/tsan/tsan-summary.json` |
| Joseph update preserved | PASS | Phase 17, Phase 18, Phase 19, and Phase 20 sanitizer and replay targets passed; see `artifacts/phase20/validation/regressions/summary.json`, `artifacts/phase20/validation/asan-ubsan/summary.json`, and `artifacts/phase20/validation/tsan/tsan-summary.json` |
| Replay determinism preserved | PASS | `artifacts/phase20/ekf_phase20_replay_report.json` reports `deterministic = true` for `continuous_camera_stream`, `long_window`, `repeated_reset`, and `eviction_stress` |
| Phase 15 regression | PASS | `artifacts/phase20/validation/regressions/summary.json`, `artifacts/phase20/validation/replay/summary.json`, and `artifacts/phase15_estimator_replay_report.json` |
| Phase 16 regression | PASS | `artifacts/phase20/validation/regressions/summary.json`, `artifacts/phase20/validation/replay/summary.json`, and `artifacts/phase16/ekf_phase16_replay_report.json` |
| Phase 17 regression | PASS | `artifacts/phase20/validation/regressions/summary.json`, `artifacts/phase20/validation/replay/summary.json`, and `artifacts/phase20/validation/tsan/tsan-summary.json` |
| Phase 18 regression | PASS | `artifacts/phase20/validation/regressions/summary.json`, `artifacts/phase20/validation/replay/summary.json`, and `artifacts/phase18/ekf_phase18_replay_report.json` |
| Phase 19 regression | PASS | `artifacts/phase20/validation/regressions/summary.json`, `artifacts/phase20/validation/replay/summary.json`, and `artifacts/phase19/ekf_phase19_replay_report.json` |
| Phase 20 unit tests | PASS | `artifacts/phase20/validation/msvc/summary.json`, `artifacts/phase20/validation/clang/summary.json`, `artifacts/phase20/validation/gcc/summary.json`, and `artifacts/phase20/validation/asan-ubsan/summary.json` |
| Replay PASS | PASS | `artifacts/phase20/validation/replay/summary.json` and `artifacts/phase20/ekf_phase20_replay_report.json` |
| MSVC | PASS | `artifacts/phase20/validation/msvc/summary.json` |
| Clang | PASS | `artifacts/phase20/validation/clang/summary.json` |
| GCC | PASS | `artifacts/phase20/validation/gcc/summary.json` |
| warnings-as-errors | PASS | `artifacts/phase20/validation/msvc-werror/summary.json`, `artifacts/phase20/validation/clang-werror/summary.json`, and `artifacts/phase20/validation/gcc-werror/summary.json` |
| clang-format | PASS | `artifacts/phase20/validation/clang-format/summary.json` |
| clang-tidy | PASS | `artifacts/phase20/validation/clang-tidy/summary.json` |
| ASan | PASS | `artifacts/phase20/validation/asan-ubsan/summary.json` |
| UBSan | PASS | `artifacts/phase20/validation/asan-ubsan/summary.json` |
| TSan | PASS | `artifacts/phase20/validation/tsan/summary.json` and `artifacts/phase20/validation/tsan/tsan-summary.json` |
| Project-owned race validation | PASS | `artifacts/phase20/validation/tsan/tsan-summary.json` reports `project_owned_races_found = false`, `third_party_races_found = false`, `warning_count = 0`, and `final_status = "PASS"` |
| Documentation complete | PASS | `docs/msckf-sliding-window-shadow-validation/` contains window design, state lifecycle, validation, final report, and remaining-risks documents |

## Final Validation Decision

- Phase 20 implementation status: `COMPLETE`
- Local validation status: `READY`
- Active estimator authority preserved: `YES`
- Phase 21 work started: `NO`
