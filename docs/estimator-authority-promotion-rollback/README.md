# Estimator Authority Promotion & Rollback

## Purpose

This work introduces a fail-closed authority-promotion controller for the existing active/shadow estimator topology.

The processing topology remains unchanged: the baseline estimator continues to receive the normal measurement stream and the advanced estimator continues to run through the existing shadow worker plus shadow-only visual-feature ingest. Promotion changes only which already-running estimator snapshot may eventually be exposed as authoritative output after an explicit readiness check.

## Safety invariants

- baseline authority is the default after construction, initialize, and reset
- promotion is never automatic
- promotion requires an explicit request and a passing `EstimatorPromotionReadiness` result
- a reset-generation mismatch rejects promotion
- rollback to baseline is always explicit and idempotent
- promotion does not swap estimator objects or stop the baseline estimator
- promotion does not re-route shadow-only feature measurements into the baseline estimator
- failed promotion leaves authority unchanged
- reset invalidates any promoted authority state and returns the controller to baseline

## Initial implementation

`EstimatorAuthorityController.hpp` provides a deterministic state machine with:

- baseline and advanced-shadow authority states
- fail-closed promotion using the existing readiness gate
- reset-generation binding
- explicit rollback
- idempotent repeated promotion/rollback requests
- promotion/rollback counters for future diagnostics

The controller is intentionally not yet wired into `EstimatorCoordinator::active_snapshot()` or `active_pose()`. That integration is the next gate and must prove atomic output selection, reset rollback, post-promotion degradation rollback behavior, and active-baseline continuity before authority can change in runtime output.

## Validation required before runtime integration

- promotion-ready evidence promotes exactly once
- insufficient readiness cannot change authority
- generation mismatch cannot change authority
- rollback restores baseline authority
- reset restores baseline authority
- repeated promotion/rollback calls are deterministic
- coordinator integration preserves the existing processing topology
- shadow-only `VisualFeatures` routing remains unchanged
- deterministic replay proves promotion and rollback output transitions
- compiler, warnings-as-errors, formatting, static-analysis, ASan/UBSan, TSan, and project-owned race checks pass

No HIL, flight activation, or automatic estimator switching is included in this work.
