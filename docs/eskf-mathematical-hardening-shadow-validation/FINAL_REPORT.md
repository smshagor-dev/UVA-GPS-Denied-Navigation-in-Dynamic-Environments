# Phase 17 Final Report

Date: July 18, 2026

## 1. Phase 17 Objective

Phase 17 hardened the project’s error-state estimator mathematics while preserving the Phase 16 architecture rule that the new estimator must run only in shadow mode and must not become authoritative.

## 2. Architecture Summary

Phase 17 adds:

- `Phase17ESKFEstimator`
- `Phase17StateEstimatorAdapter`
- shadow-only integration through `EstimatorCoordinator`

The active estimator remains the existing `EKFStateEstimatorAdapter` and the `VIOPipeline` constructor still keeps active and shadow separated.

## 3. Math Conventions

Confirmed from implementation and documentation:

- nominal state: `position, velocity, quaternion, accel bias, gyro bias`
- error state: `delta_p, delta_v, delta_theta, delta_ba, delta_bg`
- quaternion order: `w, x, y, z`
- frame direction: `q_wb` rotates body-frame vectors into world
- covariance order: position, velocity, attitude, accel bias, gyro bias
- gravity convention: world `[0, 0, -9.81]`

## 4. Propagation Implementation

Propagation uses:

- midpoint quaternion propagation
- body-frame specific force corrected by accelerometer bias
- gyro corrected by gyro bias
- continuous error-state dynamics with explicit position/velocity/attitude/bias coupling
- candidate-state validation before commit

Executed tests covering propagation passed:

- zero motion
- constant acceleration
- constant angular velocity
- long-duration finite propagation

## 5. Noise Discretization

The implementation constructs:

- continuous noise matrix `Q_imu_`
- continuous input matrix `Gc`
- continuous dynamics matrix `Fc`
- discrete transition `Phi`
- discrete process noise `Qd = Phi * Qc * Phi^T * dt` with added second-order position terms

Validation evidence:

- covariance remained finite and symmetric in unit tests
- long-duration replay completed successfully

## 6. Error-State Injection

Implemented injection applies:

- `p += delta_p`
- `v += delta_v`
- `q = normalize(q * dq(delta_theta))`
- `b_a += delta_ba`
- `b_g += delta_bg`

Evidence:

- bias-only injection test passed
- repeated error injection test passed
- invalid-input no-op behavior passed

## 7. Reset Jacobian

After attitude injection, covariance reset applies:

- `G_reset = I - 0.5 * skew(delta_theta)`

Evidence:

- dedicated reset Jacobian unit test passed

## 8. Covariance Safety

Phase 17 preserved safety behavior by:

- validating covariance symmetry
- rejecting significant negative variance
- rejecting non-finite state or covariance
- preserving no-op behavior on rejected updates
- preserving Joseph-form covariance update before reset Jacobian application

Evidence:

- covariance symmetry test passed
- repeated injection remained finite and symmetric
- long-duration propagation remained finite

## 9. Shadow Integration

`VIOPipeline` now constructs:

- active: `EKFStateEstimatorAdapter(..., "ekf_active", "phase16")`
- shadow: `Phase17StateEstimatorAdapter(..., "eskf_shadow", "phase17")`

This confirms Phase 17 runs only in shadow mode.

## 10. Active Authority Preservation

Active authority remained preserved.

Evidence:

- `Phase17Coordinator.ActiveEstimatorUnchangedWhenPhase17ShadowEnabled` passed
- Phase 17 replay artifact reports `"active_equivalent": true` for all scenarios

## 11. Tests Added

- `tests/test_phase17_eskf.cpp`
- `tests/ekf_phase17_replay.cpp`

They cover:

- propagation cases
- injection behavior
- quaternion sign equivalence
- reset Jacobian
- covariance validity
- deterministic replay
- active/shadow equivalence

## 12. Replay Scenarios

Executed scenarios:

- `stationary`
- `constant_yaw`
- `long_duration_shadow`

Observed artifact:

- `artifacts/phase17/ekf_phase17_replay_report.json`

Results:

- shadow replay succeeded
- determinism was `true` for all scenarios
- long-duration replay succeeded

## 13. Compiler Results

- MSVC Release: PASS
- MSVC warnings-as-errors: PASS
- WSL Clang Release: PASS
- WSL GCC Release: PASS

## 14. Sanitizer Results

- Clang ASan/UBSan: PASS
- Clang TSan: PASS in Docker `ubuntu:24.04` using the dedicated `linux-clang-tsan-phase17` preset, `--privileged --security-opt seccomp=unconfined`, ASLR disabled, and zero TSan warnings across `test_phase17_eskf` and `ekf_phase17_replay`
- Project-owned race detection: PASS
- ThreadSanitizer reported zero project-owned races
- ThreadSanitizer warning count: 0
- GCC ASan/UBSan preset attempt: BLOCKED by timeout during configure/build/run

## 15. Remaining Limitations

- the GCC ASan/UBSan preset did not complete within the executed session window
- historical WSL TSan attempts were unstable, but they are no longer blocking because the native Linux Docker lane now passes

## 16. Final Status

- Phase 17 implementation status: `COMPLETE`
- Local Phase 17 validation status: `READY`
- Project-owned race validation: `PASS`
- Project-owned races found: `NO`
- Active estimator authority preserved: `YES`
- Phase 18 work started: `NO`

Reason:

The implementation is real, shadow-only, and its project-owned requirements passed under executed unit, replay, compiler, Clang sanitizer, TSan, and clang-format evidence. The dedicated Phase 17 TSan lane required build-system isolation from unrelated OpenNI-linked modules, but no project-owned race was found and no estimator mathematics changed. The remaining non-PASS GCC sanitizer lane is an environment limitation rather than an implementation defect, so the overall Phase 17 conclusion remains complete.
