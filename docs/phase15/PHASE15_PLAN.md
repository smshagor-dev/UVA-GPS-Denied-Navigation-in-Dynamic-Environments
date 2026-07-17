# Phase 15 Plan

Phase 15 establishes a deterministic correctness baseline for the current minimal ESKF without enabling FEJ, MSCKF, automatic ZUPT, or loop-closure corrections.

## Scope

- Keep the active 16-state nominal and 15-state error formulation unchanged.
- Harden estimator inputs, covariance updates, and timestamp handling.
- Expose lightweight diagnostics and deterministic replay tooling.
- Preserve runtime mode behavior and fail closed on unsupported experimental estimator settings.

## Confirmed code issues

- The LiDAR path in `src/vio/VIOPipeline.cpp` computed a ground-relative height and then fed `pose.position.z()` back into `update_depth()`, creating covariance reduction without independent information.
- `update_depth()` and `update_zupt()` used subtractive covariance updates instead of the Joseph form.
- The visual frontend computes `relative_orientation`, but the EKF correction path only consumes position and velocity.
- `drift_m` semantics reflected estimated position uncertainty, not measured trajectory drift against ground truth.

## Phase boundary

- Experimental hybrid estimator components remain disabled.
- The current minimal ESKF remains the only active estimator output.
- Phase 15 software validation does not imply hardware or flight safety.
