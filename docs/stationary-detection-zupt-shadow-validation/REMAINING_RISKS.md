# Phase 18 Remaining Risks

Date: July 18, 2026

## Current Remaining Risks

- The stationary detector is IMU-only and magnitude-based, so unusual constant-acceleration profiles that mimic gravity magnitude can still require future sensor-fusion refinement.
- Phase 18 deliberately does not introduce FEJ, MSCKF, loop closure, or loop-level consistency improvements; those remain future work outside this phase.

## Non-Risks Confirmed By Validation

- Active estimator authority did not change.
- Phase 15, Phase 16, and Phase 17 regression coverage passed.
- Replay determinism remained intact.
- Covariance remained finite in replay artifacts and sanitizer lanes.
- Native Linux Docker TSan reported zero project-owned races.
- Targeted `clang-tidy` now passes with persisted evidence in `artifacts/phase18/validation/clang-tidy/summary.json`.
