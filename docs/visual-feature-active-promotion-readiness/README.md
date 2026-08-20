# Visual Feature Active-Promotion Readiness

## Purpose

This phase adds a fail-closed, read-only readiness gate for evaluating whether the shadow estimator has accumulated enough healthy evidence to be considered for a future authority-promotion phase.

This work does **not** switch estimator authority and does **not** route `VisualFeatures` measurements into the active estimator. The active baseline remains authoritative.

## Readiness evidence

`EstimatorPromotionReadiness.hpp` evaluates existing `CoordinatorDiagnostics` and blocks promotion unless all configured requirements are satisfied:

- shadow estimator enabled
- shadow worker running
- lifecycle state is `Running`
- active estimator healthy
- shadow estimator healthy
- minimum valid comparison count reached
- latest comparison valid
- queue drops remain within policy (zero by default)
- stale measurements remain within policy (zero by default)
- shadow processing failures remain within policy (zero by default)
- active/shadow position delta remains bounded
- active/shadow velocity delta remains bounded
- active/shadow orientation delta remains bounded
- absolute covariance-trace delta remains bounded

Default comparison thresholds are intentionally conservative and are configuration values, not flight-validation claims.

## Safety boundary

The evaluator is observational only. It returns `PromotionReadinessResult` with a single explicit blocking reason. No code in this phase can promote, swap, or mutate estimator authority.

Future authority promotion must remain a separate phase with its own explicit rollback path, deterministic replay evidence, sanitizer matrix, HIL acceptance criteria, and operator-visible state transition.

## Tests

Coverage is added to the existing shadow-only test target for:

- healthy bounded evidence becoming ready
- insufficient comparison evidence failing closed
- queue drops failing closed
- configured pose-delta violations failing closed
- absolute covariance-delta enforcement

## Status

Implementation phase: in progress on `visual-feature-active-promotion-readiness`.

Physical flight readiness remains unchanged: **not flight validated**.
