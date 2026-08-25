# Phase 21 Remaining Risks

Date: July 18, 2026

## Accepted Limitations

- Phase 21 adds geometric landmark initialization only. MSCKF feature-constraint updates, null-space projection, residual stacking, and marginalization redesign are intentionally not implemented yet.
- Landmark ownership is shadow-only and does not yet participate in state correction, so downstream update-path numerical behavior remains future work.
- Validation is software-only. No HIL or flight hardware evidence exists for triangulation behavior under physical camera timing variance, calibration drift, or real-world feature quality.

## Non-Blocking Observations

- Replay evidence now confirms scenario-derived active equivalence, independent shadow-only triangulation evidence, deterministic feature-dropout cleanup, finite landmark coordinates, and stable rejection behavior for low-parallax tracks, but later phases will need fresh covariance and observability review once initialized landmarks are consumed by measurement updates.
- The current diagnostics cover track counts, triangulation attempts, successes, failures, and explicit rejection reasons. Future phases may need richer landmark lifecycle observability once marginalization and constraint updates are introduced.

## Current Risk Decision

No open Phase 21 blocker remains for the project-owned requirements in this phase.
