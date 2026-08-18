# Phase 22 Feature Update Design

Date: July 19, 2026

## State Layout

- Base error state dimension: 15.
- Camera clone error dimension: 6.
- Augmented covariance width: `15 + 6 * active_clone_count`.
- Clone ordering is deterministic and follows `msckf_state_order_`.
- Clone block offset is `15 + 6 * clone_index`; position is offset `0`, orientation is offset `3` inside each clone block.

## Covariance Lifecycle

New clones append one 6D block using the base position and orientation covariance cross terms. Eviction removes the matching 6D block and compacts the augmented covariance while preserving deterministic clone ordering.

## Correction

Feature updates use the full augmented covariance. The correction vector covers the base 15D error state and all active 6D clone blocks. Base attitude and clone attitudes use right-multiplicative small-angle injection. The reset Jacobian covers the base attitude block and every corrected clone-orientation block.

## Transactionality

The update path builds candidate nominal state, candidate clone states, and candidate augmented covariance before committing. Failed innovation, state, quaternion, or covariance validation leaves the committed state unchanged.
