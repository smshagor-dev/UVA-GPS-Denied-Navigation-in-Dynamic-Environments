# Phase 22 Residual Formulation

Date: July 19, 2026

## Convention

The feature update uses `r = z - h(x, p_f)` in normalized camera coordinates. A world feature is transformed into the camera frame as `p_c = R_wc^T * (p_f - p_w)`, with camera orientation stored as a world-from-camera quaternion.

## Projection

The normalized projection is `h = [x / z, y / z]`. The projection Jacobian is:

```text
[ 1/z   0   -x/z^2 ]
[ 0     1/z -y/z^2 ]
```

For `r = z - h`, the implemented feature Jacobian is `-J_proj * R_cw`, the clone-position Jacobian is `J_proj * R_cw`, and the clone-orientation Jacobian is `-J_proj * skew(p_c)`.

## Evidence

`Phase22MsckfUpdateTest.ResidualConventionAndJacobiansMatchFiniteDifferences` checks camera-clone position, camera-clone orientation, and feature-position Jacobians with perturbation size `1.0e-6` and tolerance `1.0e-4`.
