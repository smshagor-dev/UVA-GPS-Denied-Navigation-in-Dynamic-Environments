# MSCKF Camera-State Marginalization & Feature-Track Retirement

Status: **IMPLEMENTATION IN PROGRESS**

## Objective

Replace eviction-only camera-clone removal with an explicit, auditable marginalization lifecycle that preserves usable feature information before a clone leaves the sliding window, then removes the clone covariance block without corrupting cross-covariances or estimator authority.

## Current Baseline

The estimator already maintains an augmented covariance over the 15-DoF base error state plus 6-DoF camera clones. Clone removal currently deletes the corresponding covariance rows/columns and prunes observations tied to the evicted clone. Feature-constraint updates already support clone-state Jacobians, FEJ-aware linearization, null-space projection, DOF-aware chi-square gating, transactional correction, and Joseph/reset covariance handling.

## Implementation Added

The working branch now contains a dedicated `MsckfMarginalization` primitive layer with:

- deterministic affected-track ordering by `track_id`;
- deterministic identification of update-eligible tracks that reference the retiring clone;
- duplicate retiring-state reference detection;
- retained principal-submatrix covariance extraction;
- preservation of retained base/clone cross-covariance entries;
- finite/symmetry/PSD-tolerance covariance health evaluation;
- retired-state reference checking after cleanup.

Focused tests cover deterministic planning, duplicate/stale references, exact principal-submatrix extraction, retained cross-covariance preservation, invalid dimensions, missing clone handling, covariance health, and stale-reference cleanup. A standalone invariant smoke executable validates the real 15D base + 6D-per-clone layout, and a dedicated GitHub Actions workflow compiles and executes that smoke path with both GCC and Clang under warnings-as-errors.

These primitives intentionally do not yet replace the estimator's eviction lifecycle. Wiring them into the production shadow-estimator retirement boundary is the next implementation step.

## Required Marginalization Lifecycle

For the oldest clone selected for retirement:

1. Identify feature tracks that reference the retiring clone.
2. Partition them deterministically by track id.
3. For tracks that have enough valid observations and an initialized landmark, attempt the normal MSCKF feature-constraint path before any observation or covariance block is removed.
4. Reject invalid, stale-FEJ, rank-deficient, non-finite, or statistically gated constraints without mutating estimator state.
5. Commit accepted constraints transactionally.
6. Retire the clone only after the constraint-processing boundary completes.
7. Remove observations that reference the retired clone.
8. Remove empty/stale feature tracks and landmark state according to the documented lifecycle policy.
9. Remove the clone covariance block while preserving the remaining base/clone covariance submatrix and symmetry.
10. Revalidate augmented covariance dimensions, finiteness, symmetry, and PSD tolerance.

## Mathematical Invariants

Let the augmented error state be

`delta_x_aug = [delta_x_I, delta_x_C1, ..., delta_x_CN]`.

Retiring clone `Ck` must produce the covariance of the retained variables by selecting the retained principal submatrix after all accepted measurement information involving `Ck` has been fused. No zeroing of retained cross-covariance terms is allowed.

Required invariants:

- retained covariance equals the corresponding principal submatrix of the post-update augmented covariance, within numerical tolerance;
- covariance remains finite and symmetric;
- minimum eigenvalue stays above the configured negativity tolerance;
- base covariance remains synchronized with the top-left 15x15 augmented block;
- clone order and covariance block order remain identical;
- no feature observation may reference a retired clone after cleanup;
- FEJ clone data for retained clones must remain unchanged by retirement;
- active estimator output remains authoritative and bit/tolerance equivalent to the control path.

## Determinism

Marginalization candidates and affected tracks must be processed in deterministic order. Use camera-state order for clone retirement and ascending `track_id` for feature processing. Repeated replay with identical measurements must produce identical retirement ids, accepted/rejected counts, covariance dimensions, and serialized evidence.

## Failure Semantics

A rejected feature constraint must not block safe clone retirement. A numerical failure during a candidate correction must roll back that correction completely. If covariance validation fails at the retirement boundary, fail closed: do not publish a partially mutated augmented state. Diagnostics must distinguish constraint rejection from retirement/marginalization failure.

## Diagnostics

Add or persist evidence for:

- marginalization attempts
- marginalizations completed
- marginalization failures
- retiring clone id
- affected feature-track count
- constraints attempted before retirement
- constraints accepted before retirement
- constraints rejected before retirement
- covariance dimension before/after retirement
- covariance symmetry error after retirement
- minimum covariance eigenvalue after retirement
- stale observations found after cleanup

Diagnostics-disabled mode must suppress publication/counters only; it must not disable the marginalization lifecycle.

## Tests Required

Focused unit coverage must include:

- oldest clone retirement order;
- retained principal-submatrix covariance correctness;
- retained cross-covariance preservation;
- base covariance synchronization;
- feature constraint consumed before clone removal;
- rejected constraint followed by safe retirement;
- stale-FEJ rejection followed by safe retirement;
- rank-deficient/null-space rejection followed by safe retirement;
- feature observation cleanup;
- empty-track/landmark cleanup;
- no stale state-id references;
- diagnostics-disabled behavior;
- deterministic reset/id restart behavior;
- repeated retirement covariance finiteness/symmetry/PSD;
- active-estimator preservation.

## Replay Scenarios

Create deterministic replay evidence for at least:

- `steady_window_retirement`
- `feature_constraint_before_retirement`
- `rejected_constraint_retirement`
- `stale_fej_retirement`
- `multi_feature_retirement`
- `repeated_window_rollover`
- `reset_after_retirement`

Each scenario must persist active equivalence, queue-drain evidence, retired clone ids, affected/accepted/rejected track counts, covariance dimensions, symmetry error, minimum eigenvalue, stale-reference count, determinism, and final status.

## Validation Gate

Do not call this work complete until the estimator retirement boundary uses the new lifecycle, all focused tests and replay scenarios pass, and the full compiler/warnings/static-analysis/sanitizer/regression matrix is rerun with persisted evidence. Required lanes: MSVC, MSVC warnings-as-errors, Clang, Clang warnings-as-errors, GCC, GCC warnings-as-errors, clang-format, clang-tidy, ASan, UBSan, TSan, project-owned race validation, prior estimator regressions, and deterministic replay validation.

The current smoke workflow is only a primitive-level gate. It is not completion evidence for estimator integration.

## Scope Boundary

This work is limited to camera-state marginalization/retirement and the feature-information boundary required before clone removal. Do not add loop closure, pose graph optimization, bundle adjustment, global mapping, relocalization, or change active-estimator authority.
