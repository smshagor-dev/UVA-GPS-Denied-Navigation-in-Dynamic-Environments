# Phase 16 Validation Report

## Date

Friday, July 17, 2026

## Status summary

Phase 16 local software evidence is substantially complete.
Hosted GitHub Actions evidence was not executed from this session because no authenticated GitHub session was available.

## Prior evidence re-verified in this session

The following evidence existed before Phase 16D closure work and remains valid:

- Windows MSVC Release replay and shadow validation passed locally.
- Linux GCC Release CTest passed: `164/164`.
- Linux Clang Release CTest passed: `164/164`.
- Linux GCC ASan+UBSan CTest passed: `164/164`.
- Linux Clang ASan+UBSan CTest passed: `164/164`.
- Replay active-only and active-with-identical-shadow succeeded locally on MSVC, GCC, Clang, GCC ASan+UBSan, and Clang ASan+UBSan.
- `actionlint` passed locally.
- `python scripts/audit_workflows.py` passed locally.

## New evidence executed in this session

The following work was executed locally on Friday, July 17, 2026:

- Focused Linux GCC rebuild of estimator-only validation targets through `validation-linux-gcc`.
- Focused local rerun of `test_estimator_shadow`.
- Focused local rerun of `test_estimator_replay`.
- New repeated benchmark aggregation via `scripts/phase16_benchmark_aggregate.py`.
- Focused local ThreadSanitizer rerun on estimator-only targets in `build/linux-clang-tsan`.

## Current local validation matrix

| Area | Status | Evidence |
| --- | --- | --- |
| MSVC replay and shadow validation | PASS | Prior local evidence |
| GCC Release full test suite | PASS | Prior local evidence (`164/164`) |
| Clang Release full test suite | PASS | Prior local evidence (`164/164`) |
| GCC ASan+UBSan full test suite | PASS | Prior local evidence (`164/164`) |
| Clang ASan+UBSan full test suite | PASS | Prior local evidence (`164/164`) |
| Focused estimator replay test rerun | PASS | `test_estimator_replay` |
| Focused estimator shadow test rerun | PASS | `test_estimator_shadow` |
| Repeated benchmark aggregation | PASS | `artifacts/phase16d_benchmark_aggregate.json` |
| Isolated TSan estimator replay test | PASS | `test_estimator_replay` |
| Isolated TSan estimator shadow test | PASS | `test_estimator_shadow` |
| Isolated TSan replay runner | PASS | `artifacts/phase16d_tsan_replay_active.json` |
| Isolated TSan shadow benchmark | PASS | `artifacts/phase16d_tsan_shadow_benchmark.json` |
| GitHub-hosted CI run from this session | NOT RUN | No GitHub authentication |

## TSan outcome

Phase 16C TSan evidence was previously partial because representative replay targets still encountered third-party OpenNI startup warnings.

Phase 16D isolated the replay, shadow test, and benchmark targets onto an estimator-only linkage path so they no longer require the broader sensor-fusion stack for this validation lane.
After that isolation:

- `test_estimator_replay` passed under TSan.
- `test_estimator_shadow` passed under TSan.
- `estimator_replay_runner` completed successfully under TSan on `datasets/estimator/phase15_stationary_replay.json`.
- `estimator_shadow_benchmark` completed successfully under TSan after fixing one tool-local race in benchmark latency bookkeeping.

No third-party OpenNI startup warning reappeared on these isolated Phase 16 estimator targets.

## Repeated benchmark outcome

The benchmark methodology now uses deterministic randomized repeated runs with warm-up separation:

- `active_only`: 10 runs
- `active_with_shadow`: 10 runs
- `shadow_overload`: 5 runs

Current aggregate results are documented in `PERFORMANCE_BASELINE.md` and emitted to:

- `artifacts/phase16d_benchmark_aggregate.json`
- `artifacts/phase16d_benchmark_summary.md`

## Hosted CI status

Workflow files were linted locally, but no GitHub-hosted Actions run, branch push, pull request, or artifact download was completed from this session.
See `CI_EVIDENCE.md` for the explicit hosted-evidence status.

## Limits and honesty

- Deterministic replay evidence is software evidence only.
- Native replay and shadow validation do not imply flight readiness.
- Benchmark results are environment-specific regression evidence, not universal timing guarantees.
- Hosted CI evidence remains blocked on authentication rather than code correctness.
