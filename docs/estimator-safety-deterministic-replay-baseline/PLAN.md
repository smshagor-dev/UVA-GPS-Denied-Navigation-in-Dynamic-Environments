# Phase 15 Plan

Date: July 17, 2026

## Objective

Implement the estimator safety baseline on the existing EKF path before any shadow-estimator, FEJ, MSCKF, automatic ZUPT, visual-orientation, loop-closure, or estimator-switching work begins.

## Implementation Scope

- finite-value validation for state, covariance, IMU input, and measurement input
- timestamp and time-step validation on the estimator-facing IMU path
- fail-closed transactional propagation and correction behavior
- Joseph-form covariance correction on active linear updates
- explicit diagnostics and rejection accounting
- unsafe LiDAR depth correction disabled by default
- deterministic local replay with machine-readable JSON output
- focused unit and integration validation on the existing project

## Explicit Non-Goals

- no shadow estimator execution
- no automatic ZUPT or stationary detection
- no FEJ or MSCKF implementation
- no loop closure or pose graph correction
- no estimator switching or fallback orchestration
- no HIL, flight, or production-readiness claim
