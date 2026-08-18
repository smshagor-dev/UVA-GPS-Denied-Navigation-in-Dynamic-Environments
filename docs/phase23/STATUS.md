# MSCKF Camera-State Marginalization & Feature-Track Retirement

Status: **IMPLEMENTATION IN PROGRESS**

The current estimator-hardening work now focuses on explicit camera-state marginalization and feature-track retirement on top of the validated shadow MSCKF update path.

## Current Implementation

- deterministic marginalization planning primitives added
- affected feature tracks ordered by track id
- update-eligible retiring-clone tracks identified deterministically
- retained covariance principal-submatrix extraction implemented
- retained base/clone cross-covariance preservation covered by tests
- covariance finiteness, symmetry, and PSD-tolerance health checks implemented
- stale retired-state reference detection implemented
- focused unit test source added
- standalone 15D-base + 6D-clone invariant smoke executable added
- GCC/Clang warnings-as-errors smoke workflow added

## Next Integration Boundary

The new primitives are intentionally not yet authoritative over clone eviction. The next code change must wire them into the shadow estimator so eligible constraints are processed before the oldest clone and its observations are retired.

## Guardrails

- active estimator authority remains unchanged
- deterministic replay must remain preserved
- no loop closure, bundle adjustment, pose graph, global mapping, or relocalization work is included
- covariance retirement must remain fail-closed and transactional
- retained cross-covariances must never be zeroed as a shortcut
- completion requires focused replay plus compiler/static-analysis/sanitizer/race/regression evidence
