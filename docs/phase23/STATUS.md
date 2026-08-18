# MSCKF Camera-State Marginalization & Feature-Track Retirement

Status: **IMPLEMENTATION IN PROGRESS**

The estimator-hardening work now focuses on explicit camera-state marginalization and feature-track retirement on top of the validated shadow MSCKF update path.

## Current Implementation

- deterministic marginalization planning primitives added
- affected feature tracks ordered by track id
- update-eligible retiring-clone tracks identified deterministically
- retained covariance principal-submatrix extraction implemented
- retained base/clone cross-covariance preservation covered by tests
- covariance finiteness, symmetry, and PSD-tolerance health checks implemented
- stale retired-state reference detection implemented
- fail-closed `MsckfRetirementTransaction` preparation primitive added
- retirement primitive requires oldest-first ordering and rejects invalid dimensions/configuration
- retirement result is commit-ready only after retained covariance passes finite/symmetry/PSD validation
- repeated retirement to the 15D base covariance is covered by focused tests and smoke validation
- standalone invariant executables cover both marginalization planning and retirement transactions
- GCC/Clang warnings-as-errors workflow covers both invariant executables

## Production Integration Boundary

Repository inspection confirms that `Phase17ESKFEstimator::evict_msckf_state_locked()` still performs direct oldest-clone removal: it pops the clone order, removes the covariance block, prunes feature observations, and erases the clone. The new transaction primitive is therefore not yet authoritative over that path.

The required production sequence remains:

1. select the oldest clone deterministically;
2. identify tracks referencing it in ascending track-id order;
3. attempt eligible MSCKF constraints while the retiring clone and its covariance block still exist;
4. keep rejected constraints as no-op corrections;
5. prepare the retained principal covariance from the post-constraint augmented covariance;
6. validate finite/symmetric/PSD health before any retirement commit;
7. commit clone-order, timestamp-map, feature-observation, feature/landmark lifecycle, and covariance retirement as one coherent boundary;
8. verify no retained observation references the retired state id;
9. synchronize the base 15x15 covariance with the retained augmented covariance;
10. publish diagnostics only after successful retirement.

## Guardrails

- active estimator authority remains unchanged
- deterministic replay must remain preserved
- no loop closure, bundle adjustment, pose graph, global mapping, or relocalization work is included
- covariance retirement must remain fail-closed and transactional
- retained cross-covariances must never be zeroed as a shortcut
- diagnostics-disabled mode must not disable retirement behavior
- completion requires production-path wiring, focused replay, and compiler/static-analysis/sanitizer/race/regression evidence

## Current Closure State

Implementation status: **PARTIAL**

Local validation status: **NOT READY**

Active estimator authority preserved: **YES**

Reason: the mathematical/transaction primitives are present, but the existing shadow estimator eviction function has not yet been replaced by the explicit pre-retirement feature-information lifecycle. No completion claim is permitted until that production boundary is wired and validated.
