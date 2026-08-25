# Phase 20 Remaining Risks

Date: July 18, 2026

## Accepted Limitations

- Phase 20 adds only the sliding-window state foundation. MSCKF feature-constraint updates, triangulation, and marginalization are intentionally not implemented yet.
- The supported eviction policy is intentionally limited to `oldest_first`. Alternative policies are rejected rather than partially supported.
- Validation is software-only. No HIL or flight hardware evidence exists for camera-state window scaling under physical sensor timing variance.

## Non-Blocking Observations

- Replay, regression, and sanitizer evidence confirm deterministic state lifecycle behavior for the current shadow-only design, including diagnostics-disabled neutral publication and zero active-path MSCKF dependency, but future constraint-update phases will need fresh covariance and numerical-stability review once camera states participate in measurement corrections.
- The current diagnostics focus on deterministic ownership and lifecycle metrics. Future phases may need deeper observability for feature-track associations and marginalization bookkeeping once those capabilities are introduced.

## Current Risk Decision

No open Phase 20 blocker remains for the project-owned requirements in this phase.
