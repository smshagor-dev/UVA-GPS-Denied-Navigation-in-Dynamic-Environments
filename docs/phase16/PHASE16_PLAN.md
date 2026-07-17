# Phase 16 Plan

## Scope implemented in Phase 16

- Stable `StateEstimator` interface
- `MinimalEskfAdapter` around the existing `EKFEstimator`
- Standardized typed measurement envelope and validation status model
- Bounded shadow-estimator coordinator and comparison telemetry
- Fail-closed runtime configuration for unsupported active hybrid mode
- Native C++ replay runner
- Local cross-toolchain replay validation on MSVC, GCC, and Clang
- Local ASan+UBSan validation on GCC and Clang
- Repeated benchmark aggregation for nominal and overload shadow scenarios
- Isolated representative TSan validation on estimator-only targets

## Explicitly not implemented here

- FEJ
- MSCKF
- Active visual-orientation fusion
- Automatic ZUPT
- Loop-closure correction into estimator state
- Hosted GitHub Actions evidence from this session

## Safety invariants preserved

1. The minimal ESKF remains the only authoritative estimator.
2. Shadow processing is best-effort and non-blocking for the active path.
3. Shadow queueing is bounded and exposes drop telemetry.
4. Unsupported estimator modes fail closed.
5. Visual orientation metadata is transported but not fused.

## Closure note

Phase 16 local software evidence is now materially stronger than the original implementation checkpoint.
Remaining closure work, if required, is operational rather than architectural: authenticated hosted CI execution and artifact verification.
