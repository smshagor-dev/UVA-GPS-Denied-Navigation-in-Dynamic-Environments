# Visual Estimator Soak Validation

This work item extends the existing point-in-time promotion-readiness check into sustained readiness evidence for the shadow visual estimator.

## Safety boundary

- The active estimator remains authoritative.
- The soak monitor is read-only and cannot switch estimators.
- Any blocked readiness sample fails closed.
- Readiness progress resets when configured to do so.
- Queue drops, stale measurements, processing failures, health regressions, invalid comparisons, and divergence remain blockers through the existing readiness evaluator.

## Goal

Require a configurable number of consecutive healthy readiness samples before reporting `sustained_ready=true`. This is evidence generation only; it is not authority promotion, flight activation, or estimator switching.
