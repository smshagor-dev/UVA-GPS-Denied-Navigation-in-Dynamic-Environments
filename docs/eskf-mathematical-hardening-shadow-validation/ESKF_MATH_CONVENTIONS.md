# Phase 17 ESKF Math Conventions

Date: July 17, 2026
Status: Implemented locally

## Nominal State

The hardened Phase 17 shadow estimator uses the nominal state:

- position `p_w` in world meters
- velocity `v_w` in world meters per second
- orientation quaternion `q_wb`
- accelerometer bias `b_a` in body-frame meters per second squared
- gyro bias `b_g` in body-frame radians per second

Quaternion storage order remains `w, x, y, z`.

`q_wb` maps body vectors into the world frame through `R_wb = q_wb.toRotationMatrix()`.

## Error State

The 15-state error vector is ordered as:

- `delta_p`
- `delta_v`
- `delta_theta`
- `delta_ba`
- `delta_bg`

The covariance block order matches that exact layout:

- indices `0:2` position
- indices `3:5` velocity
- indices `6:8` attitude
- indices `9:11` accelerometer bias
- indices `12:14` gyro bias

## Frames And Gravity

- world frame is the navigation frame used by the existing estimator
- body-frame IMU specific force is rotated into world with `R_wb`
- gravity is applied in world as `[0, 0, -9.81]`

This preserves the existing project convention that a level stationary IMU reports approximately
`[0, 0, +9.81]` in body acceleration.

## Noise Units

The implementation keeps the existing configuration units:

- accelerometer white noise `sigma_na` in `m/s^2`
- gyro white noise `sigma_ng` in `rad/s`
- accelerometer bias random walk `sigma_nba` in `m/s^2`
- gyro bias random walk `sigma_nbg` in `rad/s`

The continuous-time noise vector remains ordered as:

- accelerometer noise
- gyro noise
- accelerometer bias drive
- gyro bias drive

## Propagation Model

The hardened shadow estimator uses:

- midpoint quaternion propagation for the nominal state
- explicit attitude-rate coupling `-skew(omega)`
- position, velocity, attitude, accelerometer-bias, and gyro-bias coupling in the error model
- continuous-to-discrete propagation through `Phi = I + Fc * dt` with second-order
  position coupling terms for attitude and accelerometer bias
- discrete process noise from the continuous model and IMU noise matrix, followed by symmetry
  enforcement and covariance validation

## Injection And Reset

Measurement updates remain Joseph-form and transactional.

Error-state injection applies:

- `p += delta_p`
- `v += delta_v`
- `q = normalize(q * dq(delta_theta))`
- `b_a += delta_ba`
- `b_g += delta_bg`

After attitude injection, the covariance reset uses the small-angle reset Jacobian:

- `G_reset = I - 0.5 * skew(delta_theta)`

This Jacobian is applied only in the Phase 17 hardened shadow estimator.

## Authority

- the active estimator remains the existing Phase 16 EKF adapter
- the hardened Phase 17 estimator is wired only into the shadow slot through
  `EstimatorCoordinator`
