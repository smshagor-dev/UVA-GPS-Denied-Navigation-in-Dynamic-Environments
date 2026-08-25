# Mathematical Formulation and Implementation Mapping

This document maps the principal mathematical operations in the repository to the source code that uses them. It is intended as an engineering and academic reference: each equation below is either directly implemented in the current codebase or is explicitly marked as a design/planning model.

The purpose is not to claim physical validation from software equations. Numerical thresholds, weights, noise values, confidence functions, and policy gains are software configuration choices unless a section explicitly says otherwise. They require calibration and experimental validation on the target camera, IMU, LiDAR, UWB hardware, radio, compute platform, airframe, and operating environment.

## 1. Notation and coordinate conventions

The estimator uses a nominal navigation state containing position, velocity, orientation, accelerometer bias, and gyroscope bias. The baseline implementation stores a quaternion in the nominal state and propagates a 15-dimensional error state.

$$
\mathbf{x} = \left(\mathbf{p},\mathbf{v},\mathbf{q},\mathbf{b}_a,\mathbf{b}_g\right)
$$

with error state

$$
\delta\mathbf{x}=
\begin{bmatrix}
\delta\mathbf{p}^T &
\delta\mathbf{v}^T &
\delta\boldsymbol{\theta}^T &
\delta\mathbf{b}_a^T &
\delta\mathbf{b}_g^T
\end{bmatrix}^T \in \mathbb{R}^{15}.
$$

The main symbols used below are:

| Symbol | Meaning |
|---|---|
| $\mathbf{p}$ | world-frame position |
| $\mathbf{v}$ | world-frame velocity |
| $\mathbf{q}$ | body-to-world unit quaternion |
| $\mathbf{R}(\mathbf{q})$ | rotation matrix generated from $\mathbf{q}$ |
| $\mathbf{b}_a,\mathbf{b}_g$ | accelerometer and gyroscope bias |
| $\mathbf{a}_m,\boldsymbol\omega_m$ | measured acceleration and angular rate |
| $\mathbf{P}$ | estimator covariance |
| $\mathbf{Q}$ | process-noise covariance |
| $\mathbf{R}_m$ | measurement-noise covariance |
| $\mathbf{H}$ | measurement Jacobian |
| $\mathbf{K}$ | Kalman gain |
| $\mathbf{r}$ | innovation/residual |
| $[\mathbf{u}]_\times$ | skew-symmetric matrix of vector $\mathbf{u}$ |
| $\Delta t$ | integration interval |

**Primary code:** `include/vio/EKFEstimator.hpp`, `src/vio/EKFEstimator.cpp`, `src/vio/Phase17ESKFEstimator.cpp` (legacy filename for the secondary ESKF implementation).

---

# Part I — Sensor-side calculations

## 2. IMU raw-unit conversion

For the MPU-6050 path configured at ±2 g and ±250 deg/s, raw signed 16-bit readings are converted using fixed scale factors.

Accelerometer scale:

$$
s_a=\frac{9.81}{16384}\;\text{m/s}^2\text{/LSB}
$$

$$
\mathbf{a}_m=s_a\,\mathbf{a}_{raw}.
$$

Gyroscope scale:

$$
s_g=\frac{250}{32768}\frac{\pi}{180}\;\text{rad/s/LSB}
$$

$$
\boldsymbol\omega_m=s_g\,\boldsymbol\omega_{raw}.
$$

The MPU-6050 temperature conversion in the current driver is

$$
T[^{\circ}C]=\frac{T_{raw}}{340}+36.53.
$$

**Used in:** hardware IMU decoding before calibration and estimator ingestion.

**Code:** `src/sensors/IMUSensor.cpp`.

## 3. Static IMU calibration

Given $N$ stationary samples, the gyroscope bias is the sample mean:

$$
\hat{\mathbf b}_g=\frac{1}{N}\sum_{i=1}^{N}\boldsymbol\omega_i.
$$

The accelerometer bias assumes +Z gravity during calibration:

$$
\hat{\mathbf b}_a=\frac{1}{N}\sum_{i=1}^{N}\mathbf a_i-
\begin{bmatrix}0&0&9.81\end{bmatrix}^T.
$$

Runtime calibration is then applied as

$$
\mathbf a_c=\mathbf S_a(\mathbf a_m-\hat{\mathbf b}_a),
\qquad
\boldsymbol\omega_c=\boldsymbol\omega_m-\hat{\mathbf b}_g,
$$

where `accel_scale_` is the implemented accelerometer scale matrix.

The observed sample rate is estimated from adjacent timestamps:

$$
f_s=\frac{1}{t_k-t_{k-1}}.
$$

**Code:** `src/sensors/IMUSensor.cpp`.

## 4. LiDAR Cartesian point geometry

For each Cartesian LiDAR point $(x,y,z)$, the driver computes range

$$
r=\sqrt{x^2+y^2+z^2},
$$

azimuth

$$
\psi=\operatorname{atan2}(y,x)\frac{180}{\pi},
$$

and elevation

$$
\theta=\operatorname{atan2}\left(z,\sqrt{x^2+y^2}\right)\frac{180}{\pi}.
$$

Only finite points whose range lies inside the configured `[min_range_m, max_range_m]` interval are copied into the PCL cloud.

**Code:** `src/sensors/LidarSensor.cpp`.

## 5. Camera preprocessing and detector coordinate conversion

The OpenCV DNN fallback constructs the network tensor with an intensity scale of

$$
I_{net}=\frac{I}{255},
$$

and a 640×640 network input.

A detector output $(c_x,c_y,w,h)$ is mapped back to image coordinates using

$$
x=\left(c_x-\frac{w}{2}\right)\frac{W}{640},
\qquad
y=\left(c_y-\frac{h}{2}\right)\frac{H}{640},
$$

$$
w_{img}=w\frac{W}{640},
\qquad
h_{img}=h\frac{H}{640}.
$$

The stored bounding box is normalized by image width and height and clamped to $[0,1]$. Candidate detections first pass the configured class-confidence threshold and then OpenCV NMS with the configured NMS threshold.

Camera undistortion uses the calibrated intrinsic matrix

$$
\mathbf K=
\begin{bmatrix}
f_x&0&c_x\\
0&f_y&c_y\\
0&0&1
\end{bmatrix}
$$

plus five configured distortion coefficients through OpenCV's rectify/remap implementation.

**Code:** `src/sensors/CameraSensor.cpp`.

## 6. Motor-health heuristic

The current motor sensor is a software/simulation-oriented health source. Per-motor penalties are

$$
p_T=\operatorname{clamp}\left(\frac{T-55}{35},0,0.5\right),
$$

$$
p_V=\operatorname{clamp}\left(1.5(V-0.25),0,0.4\right),
$$

$$
p_I=\operatorname{clamp}\left(0.15(I-6.0),0,0.2\right).
$$

Health is

$$
h_i=\operatorname{clamp}(1-p_T-p_V-p_I,0,1),
$$

and fleet-facing average motor health is

$$
\bar h=\frac{1}{M}\sum_{i=1}^{M}h_i.
$$

A critical motor-health flag is set when

$$
\bar h<0.45.
$$

These equations are policy heuristics, not experimentally identified motor-failure probabilities.

**Code:** `src/sensors/MotorSensor.cpp`.

## 7. Placeholder sensor equations

The current barometer placeholder uses

$$
P=101325-12h,
$$

with a fixed simulated altitude $h=8$ m. The current optical-flow and rangefinder implementations also emit fixed software placeholder measurements rather than deriving them from hardware data.

These values must not be interpreted as calibrated atmospheric, optical-flow, or ranging models.

**Code:** `src/sensors/BarometerSensor.cpp`, `src/sensors/OpticalFlowSensor.cpp`, `src/sensors/RangefinderSensor.cpp`.

---

# Part II — Baseline EKF

## 8. IMU noise model

The continuous IMU noise covariance is assembled as

$$
\mathbf Q_{imu}=\operatorname{diag}
\left(
\sigma_{na}^2\mathbf I_3,
\sigma_{ng}^2\mathbf I_3,
\sigma_{nba}^2\mathbf I_3,
\sigma_{nbg}^2\mathbf I_3
\right).
$$

Current software defaults are:

| Parameter | Default |
|---|---:|
| accelerometer white-noise $\sigma_{na}$ | `0.02` |
| gyroscope white-noise $\sigma_{ng}$ | `0.005` |
| accelerometer bias walk $\sigma_{nba}$ | `1e-4` |
| gyroscope bias walk $\sigma_{nbg}$ | `1e-5` |

**Code:** `include/vio/EKFEstimator.hpp`, `src/vio/EKFEstimator.cpp`.

## 9. Initial covariance

Initial covariance is diagonal by state group:

$$
\mathbf P_0=
\operatorname{diag}
\left(
\sigma_p^2\mathbf I_3,
\sigma_v^2\mathbf I_3,
\sigma_\theta^2\mathbf I_3,
\sigma_{ba}^2\mathbf I_3,
\sigma_{bg}^2\mathbf I_3
\right).
$$

Current default standard deviations are 0.1 m position, 0.05 m/s velocity, 0.05 rad attitude-error representation, 0.01 accelerometer bias, and 0.001 gyroscope bias.

## 10. Bias-corrected inertial input

The estimator removes the current bias estimates before propagation:

$$
\mathbf a=\mathbf a_m-\mathbf b_a,
\qquad
\boldsymbol\omega=\boldsymbol\omega_m-\mathbf b_g.
$$

Gravity is represented as

$$
\mathbf g=\begin{bmatrix}0&0&-9.81\end{bmatrix}^T.
$$

## 11. Midpoint attitude and world acceleration

A half-step quaternion is used for acceleration rotation:

$$
\mathbf q_{1/2}=\mathbf q_k\otimes\operatorname{Exp}
\left(\frac{1}{2}\boldsymbol\omega\Delta t\right),
$$

$$
\mathbf a_w=\mathbf R(\mathbf q_{1/2})\mathbf a+\mathbf g.
$$

## 12. Nominal-state propagation

Position:

$$
\mathbf p_{k+1}=\mathbf p_k+\mathbf v_k\Delta t+
\frac{1}{2}\mathbf a_w\Delta t^2.
$$

Velocity:

$$
\mathbf v_{k+1}=\mathbf v_k+\mathbf a_w\Delta t.
$$

Orientation:

$$
\mathbf q_{k+1}=\operatorname{normalize}
\left(\mathbf q_k\otimes\operatorname{Exp}(\boldsymbol\omega\Delta t)\right).
$$

**Code:** `src/vio/EKFEstimator.cpp`.

## 13. Error-state transition matrix

The implemented first-order discrete transition contains the principal blocks

$$
\mathbf F_{pv}=\mathbf I\Delta t,
$$

$$
\mathbf F_{v\theta}=-\mathbf R[\mathbf a]_\times\Delta t,
$$

$$
\mathbf F_{v b_a}=-\mathbf R\Delta t,
$$

$$
\mathbf F_{\theta b_g}=-\mathbf I\Delta t.
$$

The IMU-noise mapping contains

$$
\mathbf G_{v n_a}=-\mathbf R,
\quad
\mathbf G_{\theta n_g}=-\mathbf I,
\quad
\mathbf G_{b_a n_{ba}}=\mathbf I,
\quad
\mathbf G_{b_g n_{bg}}=\mathbf I.
$$

## 14. Discrete process covariance and covariance propagation

The baseline discretization is

$$
\mathbf Q_d=(\mathbf G\mathbf Q_{imu}\mathbf G^T)\Delta t.
$$

Covariance prediction is

$$
\mathbf P^- = \mathbf F\mathbf P\mathbf F^T+\mathbf Q_d.
$$

After numerical operations, covariance is explicitly symmetrized:

$$
\mathbf P\leftarrow\frac{1}{2}(\mathbf P+\mathbf P^T).
$$

## 15. Perspective camera projection

For world landmark $\mathbf p_f$, camera-frame coordinates are

$$
\mathbf p_c=\mathbf R^T(\mathbf p_f-\mathbf p)
=\begin{bmatrix}X&Y&Z\end{bmatrix}^T.
$$

Predicted pixel position is

$$
\hat u=f_x\frac{X}{Z}+c_x,
\qquad
\hat v=f_y\frac{Y}{Z}+c_y.
$$

The projection Jacobian is

$$
\mathbf J_\pi=
\begin{bmatrix}
\frac{f_x}{Z}&0&-\frac{f_xX}{Z^2}\\
0&\frac{f_y}{Z}&-\frac{f_yY}{Z^2}
\end{bmatrix}.
$$

Position and attitude blocks used by the visual update are formed from

$$
\mathbf H_p=\mathbf J_\pi(-\mathbf R^T),
$$

$$
\mathbf H_\theta=\mathbf J_\pi\mathbf R^T[\mathbf p_f-\mathbf p]_\times.
$$

The image innovation is

$$
\mathbf r=\mathbf z-\hat{\mathbf z}.
$$

## 16. Innovation covariance and Mahalanobis gate

For any linearized measurement,

$$
\mathbf S=\mathbf H\mathbf P\mathbf H^T+\mathbf R_m.
$$

The visual path computes

$$
d_M^2=\mathbf r^T\mathbf S^{-1}\mathbf r
$$

and rejects the update when the value exceeds the configured `mahal_gate`. The current default scalar threshold is `7.815`. This document reports the threshold exactly as configured; statistical interpretation must use the actual residual dimension of the measurement being evaluated.

## 17. Generic Kalman correction

The Kalman gain is

$$
\mathbf K=\mathbf P\mathbf H^T\mathbf S^{-1}.
$$

The error correction is

$$
\delta\mathbf x=\mathbf K\mathbf r.
$$

Position, velocity, accelerometer bias, and gyroscope bias are injected additively. Attitude error is injected through a rotation-vector/quaternion update.

## 18. Joseph-form covariance correction

The corrected covariance is

$$
\mathbf P^+=(\mathbf I-\mathbf K\mathbf H)\mathbf P
(\mathbf I-\mathbf K\mathbf H)^T+
\mathbf K\mathbf R_m\mathbf K^T.
$$

The matrix is symmetrized after correction.

This Joseph form is used to reduce loss of symmetry/positive-semidefinite behavior from finite-precision arithmetic.

## 19. Visual-pose correction

The visual-pose residual stacks position and velocity:

$$
\mathbf r=
\begin{bmatrix}
\mathbf p_{obs}-\mathbf p\\
\mathbf v_{obs}-\mathbf v
\end{bmatrix}.
$$

Measurement covariance is block diagonal:

$$
\mathbf R_m=
\operatorname{diag}(\sigma_p^2\mathbf I_3,\sigma_v^2\mathbf I_3).
$$

## 20. LiDAR depth correction

The scalar depth/height residual is

$$
r=z_{depth}-p_z,
$$

with

$$
R_m=\sigma_z^2.
$$

The typed measurement wrapper also records `covariance_hint = sigma_m^2`.

## 21. Zero-velocity update

For a stationary vehicle, the measurement model enforces zero velocity:

$$
\mathbf r=-\mathbf v,
$$

$$
\mathbf H_v=\begin{bmatrix}\mathbf 0&\mathbf I_3&\mathbf 0&\mathbf 0&\mathbf 0\end{bmatrix},
$$

$$
\mathbf R_{zupt}=\sigma_v^2\mathbf I_3.
$$

After an accepted update, the baseline path explicitly sets velocity to zero.

## 22. Position uncertainty, drift proxy, and localization confidence

The position standard-deviation vector is taken from the covariance diagonal:

$$
\boldsymbol\sigma_p=
\begin{bmatrix}
\sqrt{P_{xx}}&\sqrt{P_{yy}}&\sqrt{P_{zz}}
\end{bmatrix}^T.
$$

The software drift/uncertainty proxy is

$$
d_u=\|\boldsymbol\sigma_p\|_2.
$$

Baseline confidence begins as

$$
c=\operatorname{clamp}\left(1-\frac{d_u}{2.5},0,1\right).
$$

Age-based adjustments in the current code are:

- if visual age > 0.8 s: $c\leftarrow0.78c$;
- if visual age > 1.6 s: $c\leftarrow0.62c$ again;
- if depth age < 0.6 s: $c\leftarrow\min(1,c+0.08)$.

The software marks localization degraded when confidence is below 0.58, uncertainty exceeds 0.85, or visual age exceeds 1.2 s. It marks localization lost when confidence is below 0.22, uncertainty exceeds 1.8, or visual age exceeds 3.5 s.

These thresholds are runtime policy defaults, not universal estimator-consistency criteria.

**Code for Sections 8–22:** `include/vio/EKFEstimator.hpp`, `src/vio/EKFEstimator.cpp`.

---

# Part III — Secondary error-state estimator and MSCKF calculations

## 23. Continuous ESKF linearization

The secondary estimator builds a continuous-time linearized error system. Principal blocks are

$$
\mathbf F_{pv}=\mathbf I,
$$

$$
\mathbf F_{v\theta}=-\mathbf R[\mathbf a]_\times,
$$

$$
\mathbf F_{v b_a}=-\mathbf R,
$$

$$
\mathbf F_{\theta\theta}=-[\boldsymbol\omega]_\times,
$$

$$
\mathbf F_{\theta b_g}=-\mathbf I.
$$

Noise mapping follows the same physical grouping as the baseline filter.

## 24. ESKF transition discretization

The implementation starts from

$$
\boldsymbol\Phi\approx\mathbf I+\mathbf F_c\Delta t
$$

and explicitly includes second-order position terms:

$$
\Phi_{p\theta}=-\frac{1}{2}\mathbf R[\mathbf a]_\times\Delta t^2,
$$

$$
\Phi_{p b_a}=-\frac{1}{2}\mathbf R\Delta t^2,
$$

$$
\Phi_{v\theta}=-\mathbf R[\mathbf a]_\times\Delta t,
$$

$$
\Phi_{v b_a}=-\mathbf R\Delta t,
$$

$$
\Phi_{\theta\theta}=\mathbf I-[\boldsymbol\omega]_\times\Delta t,
$$

$$
\Phi_{\theta b_g}=-\mathbf I\Delta t.
$$

## 25. ESKF process-noise discretization

Continuous mapped covariance is

$$
\mathbf Q_c=\mathbf G_c\mathbf Q_{imu}\mathbf G_c^T.
$$

The implementation forms

$$
\mathbf Q_d=\boldsymbol\Phi\mathbf Q_c\boldsymbol\Phi^T\Delta t
$$

and adds integrated accelerometer-noise terms

$$
\mathbf Q_{pp}\leftarrow\mathbf Q_{pp}+
\frac{1}{4}\sigma_{na}^2\Delta t^4\mathbf I,
$$

$$
\mathbf Q_{pv}\leftarrow\mathbf Q_{pv}+
\frac{1}{2}\sigma_{na}^2\Delta t^3\mathbf I,
\qquad
\mathbf Q_{vp}=\mathbf Q_{pv}^T.
$$

For the augmented MSCKF state,

$$
\mathbf P^-_{aug}=\boldsymbol\Phi_{aug}\mathbf P_{aug}\boldsymbol\Phi_{aug}^T+
\mathbf Q_{aug}.
$$

## 26. Quaternion error injection and reset Jacobian

A small attitude error $\delta\boldsymbol\theta$ is injected as

$$
\mathbf q^+=\operatorname{normalize}\left(
\mathbf q\otimes\operatorname{Exp}(\delta\boldsymbol\theta)
\right).
$$

The error-state reset Jacobian for attitude is

$$
\mathbf G_{reset}=\mathbf I-rac{1}{2}[\delta\boldsymbol\theta]_\times.
$$

Covariance is transformed by

$$
\mathbf P\leftarrow\mathbf J_{reset}\mathbf P\mathbf J_{reset}^T.
$$

The augmented path applies corresponding reset blocks to cloned camera attitudes.

## 27. Innovation-conditioning check

Before solving some secondary-estimator updates, the innovation covariance is symmetrized:

$$
\mathbf S_s=\frac{1}{2}(\mathbf S+\mathbf S^T).
$$

With eigenvalues $\lambda_{min}$ and $\lambda_{max}$, the condition number used by the guard is

$$
\kappa(\mathbf S)=\frac{\lambda_{max}}{\lambda_{min}}.
$$

The implementation requires a minimum eigenvalue above approximately $10^{-12}$ and condition number no larger than $10^{12}$ before accepting the solve.

## 28. Stationary detector

Accelerometer gravity mismatch is

$$
e_a=\left|\|\mathbf a_m\|_2-g\right|,
$$

and gyroscope magnitude is

$$
e_g=\|\boldsymbol\omega_m\|_2.
$$

An entry candidate requires both quantities below the configured entry thresholds. The default entry thresholds are 0.15 m/s² and 0.02 rad/s. A sliding window of 20 samples requires at least 16 stationary candidates and at least 0.15 s candidate duration before stationary state is entered.

The exit thresholds are wider: 0.25 m/s² and 0.04 rad/s; fewer than 10 retained stationary samples can drive exit. Automatic ZUPT is rate-limited by

$$
\Delta t_{zupt,min}=\frac{1}{f_{zupt,max}},
$$

with default $f_{zupt,max}=10$ Hz.

## 29. First-Estimate Jacobian (FEJ) linearization

For selected visual/MSCKF Jacobians, the residual can be evaluated at the current nominal state while the Jacobian is formed using stored first-estimate pose values.

The FEJ camera point is

$$
\mathbf p_c^{FEJ}=\mathbf R(\mathbf q_{first})^T
(\mathbf p_f-\mathbf p_{first}).
$$

The same perspective or normalized projection Jacobian structure is then evaluated using this stored linearization point.

This is an implementation choice intended to reduce repeated relinearization of selected geometric constraints; it is not a claim of full observability-consistency proof.

## 30. Camera-state covariance augmentation

With base/previous covariance $\mathbf P$ and a clone Jacobian $\mathbf J_c$ selecting position and attitude error,

$$
\mathbf P_{xc}=\mathbf P\mathbf J_c^T,
$$

$$
\mathbf P_{cc}=\mathbf J_c\mathbf P\mathbf J_c^T.
$$

The augmented covariance is

$$
\mathbf P_{aug}=
\begin{bmatrix}
\mathbf P & \mathbf P_{xc}\\
\mathbf P_{xc}^T & \mathbf P_{cc}
\end{bmatrix}.
$$

The implementation maintains a bounded camera-state window and retires the oldest clone first.

## 31. Multi-view feature triangulation

For camera observation $i$, a normalized body/camera bearing is rotated into the world frame:

$$
\mathbf d_i=\operatorname{normalize}(\mathbf R_i\mathbf b_i).
$$

The projector orthogonal to the ray is

$$
\mathbf M_i=\mathbf I-\mathbf d_i\mathbf d_i^T.
$$

The implementation accumulates

$$
\mathbf A=\sum_i\mathbf M_i,
\qquad
\mathbf b=\sum_i\mathbf M_i\mathbf p_i,
$$

and solves the least-squares intersection

$$
\mathbf p_f=\mathbf A^{-1}\mathbf b
$$

through an eigen-based inverse construction. The minimum eigenvalue must remain above the configured numerical tolerance (approximately $10^{-9}$ in the implementation).

Geometry is rejected if the maximum camera baseline is below the configured 0.05 m default, depth falls outside 0.1–50 m, or reprojection error exceeds the configured 2.5 px default.

## 32. Reprojection validation

For each observation,

$$
\hat{\mathbf z}_i=
\begin{bmatrix}
f_xX/Z+c_x\\f_yY/Z+c_y\end{bmatrix},
$$

and pixel reprojection error is

$$
e_{rep,i}=\|\mathbf z_i-\hat{\mathbf z}_i\|_2.
$$

A feature is rejected when its geometry or reprojection violates the configured bounds.

## 33. MSCKF normalized residual

The observed normalized image coordinate is

$$
\mathbf z_i=
\begin{bmatrix}
b_x/b_z\\b_y/b_z\end{bmatrix},
$$

while predicted normalized coordinate is

$$
\hat{\mathbf z}_i=
\begin{bmatrix}X/Z\\Y/Z\end{bmatrix}.
$$

Residual:

$$
\mathbf r_i=\mathbf z_i-\hat{\mathbf z}_i.
$$

Normalized projection Jacobian:

$$
\mathbf J_n=
\begin{bmatrix}
1/Z&0&-X/Z^2\\
0&1/Z&-Y/Z^2
\end{bmatrix}.
$$

The implementation forms feature and camera-clone blocks from this Jacobian, including

$$
\mathbf H_f=-\mathbf J_n\mathbf R_{cw}.
$$

The raw track residual norm is

$$
e_{track}=\sqrt{\sum_i\|\mathbf r_i\|_2^2}.
$$

## 34. Feature null-space projection

Stacked feature equations have the form

$$
\mathbf r=\mathbf H_x\delta\mathbf x+\mathbf H_f\delta\mathbf p_f+\mathbf n.
$$

The implementation computes an SVD of $\mathbf H_f$ and selects a left-nullspace basis $\mathbf A$ satisfying approximately

$$
\mathbf A^T\mathbf H_f\approx\mathbf 0.
$$

The landmark perturbation is eliminated by

$$
\mathbf r_o=\mathbf A^T\mathbf r,
$$

$$
\mathbf H_o=\mathbf A^T\mathbf H_x,
$$

$$
\mathbf R_o=\mathbf A^T\mathbf R\mathbf A.
$$

The implementation checks both null-space annihilation and orthogonality,

$$
\|\mathbf A^T\mathbf H_f\|,
\qquad
\|\mathbf A^T\mathbf A-\mathbf I\|,
$$

against small numerical tolerances around $10^{-8}$.

## 35. MSCKF residual and chi-square gating

A projected feature constraint is first bounded by the configured residual norm:

$$
\|\mathbf r_o\|_2\le r_{max}.
$$

The innovation is

$$
\mathbf S=\mathbf H_o\mathbf P_{aug}\mathbf H_o^T+\mathbf R_o.
$$

The normalized innovation statistic is

$$
\gamma=\mathbf r_o^T\mathbf S^{-1}\mathbf r_o.
$$

The threshold is generated from a chi-square quantile:

$$
\tau=F_{\chi^2_\nu}^{-1}(p),
$$

where $\nu=\dim(\mathbf r_o)$ and current default probability is $p=0.95$. The feature passes when

$$
\gamma\le\tau.
$$

Accepted feature constraints are vertically stacked, while their noise matrices are assembled block-diagonally, before the augmented Kalman/Joseph update.

## 36. Augmented-state Kalman correction

For the MSCKF-augmented state,

$$
\mathbf K=\mathbf P_{aug}\mathbf H_{aug}^T
(\mathbf H_{aug}\mathbf P_{aug}\mathbf H_{aug}^T+\mathbf R)^{-1}.
$$

The correction is

$$
\delta\mathbf x_{aug}=\mathbf K\mathbf r.
$$

The first 15 elements correct the base navigation error state; each 6-element clone block corrects clone position and clone orientation. The Joseph form is then applied to the entire augmented covariance followed by the attitude reset transformation.

## 37. Clone marginalization

When the oldest camera clone is retired, its six covariance rows and columns are removed. If $\mathcal I$ is the retained index set,

$$
\mathbf P_{ret}=\mathbf P_{aug}[\mathcal I,\mathcal I].
$$

The retained covariance is symmetrized:

$$
\mathbf P_{ret}\leftarrow\frac{1}{2}
(\mathbf P_{ret}+\mathbf P_{ret}^T).
$$

Covariance-health diagnostics include symmetry error

$$
e_{sym}=\max_{ij}|P_{ij}-P_{ji}|
$$

and minimum eigenvalue

$$
\lambda_{min}=\min\operatorname{eig}(\mathbf P_{ret}).
$$

The retirement transaction commits only when the retained covariance is finite, symmetric within tolerance, and positive semidefinite within the configured negative-eigenvalue tolerance.

**Code for Sections 23–37:** `src/vio/Phase17ESKFEstimator.cpp`, `include/vio/MsckfMarginalization.hpp`, `include/vio/MsckfRetirementTransaction.hpp`.

---

# Part IV — Visual front end and persistent feature geometry

## 38. Optical-flow quality aggregation

The visual front end averages valid per-feature optical-flow errors:

$$
\bar e=\frac{1}{N_v}\sum_{i\in\mathcal V}e_i.
$$

Tracked-feature count and RANSAC inlier count produce

$$
r_{inlier}=\frac{N_{inlier}}{N_{tracked}}.
$$

## 39. Visual update confidence

The implementation builds three bounded scores:

$$
s_f=\operatorname{clamp}\left(\frac{N_{tracked}}{140},0,1\right),
$$

$$
s_i=\operatorname{clamp}(r_{inlier},0,1),
$$

$$
s_r=\operatorname{clamp}\left(1-\frac{e_{reproj}}{8},0,1\right).
$$

Combined visual confidence is

$$
c_v=0.35s_f+0.45s_i+0.20s_r.
$$

If the frontend update is rejected, confidence is multiplied by 0.45. Placeholder visual data is capped at 0.42. A zero-feature failure path returns a low confidence value of 0.18.

These weights are software-quality heuristics, not calibrated probabilities.

## 40. Essential-matrix motion direction and metric scale

The visual front end estimates an essential matrix with RANSAC and recovers relative rotation/translation direction through OpenCV. The recovered translation is direction-only, so metric scale is borrowed from the current predicted motion:

$$
\Delta\mathbf p_{pred}=\mathbf p_{pred,k}-\mathbf p_{k-1},
$$

$$
s=\max\left(\|\Delta\mathbf p_{pred}\|,
\|\mathbf v_{pred}\|\Delta t\right).
$$

With normalized recovered translation direction $\hat{\mathbf t}$,

$$
\Delta\mathbf p_w=\mathbf R_{prev}\hat{\mathbf t}\,s.
$$

Observed position and velocity are then

$$
\mathbf p_{obs}=\mathbf p_{prev}+\Delta\mathbf p_w,
$$

$$
\mathbf v_{obs}=\frac{\Delta\mathbf p_w}{\Delta t}.
$$

Current acceptance requires at least 24 tracked features, inlier ratio at least 0.55, and reprojection/error metric no greater than 3.5 px.

**Code:** `src/vio/VIOPipeline.cpp`, `include/vio/VIOPipeline.hpp`.

## 41. Persistent track association

A previous observation is associated with an existing track using squared pixel distance

$$
d_i^2=\|\mathbf u_i-\mathbf u_{track}\|_2^2.
$$

The default association radius is 3 px, so a candidate must satisfy

$$
d_i^2<9.
$$

Tie-breaking is deterministic by lower track ID.

## 42. Two-view triangulation used by the track manager

The camera baseline is

$$
\mathbf b=\mathbf p_2-\mathbf p_1,
$$

with default minimum

$$
\|\mathbf b\|\ge0.05\;\text{m}.
$$

Pixel rays are back-projected as

$$
\mathbf r_1=\operatorname{normalize}
\left(\mathbf R_1\mathbf K^{-1}[u_1,v_1,1]^T\right),
$$

$$
\mathbf r_2=\operatorname{normalize}
\left(\mathbf R_2\mathbf K^{-1}[u_2,v_2,1]^T\right).
$$

Parallax is

$$
\theta=\cos^{-1}(\operatorname{clamp}(\mathbf r_1^T\mathbf r_2,-1,1))
\frac{180}{\pi},
$$

with default minimum 1 degree.

Ray depths solve

$$
\begin{bmatrix}\mathbf r_1&-\mathbf r_2\end{bmatrix}
\begin{bmatrix}\lambda_1\\\lambda_2\end{bmatrix}
=\mathbf p_2-\mathbf p_1
$$

by QR least squares.

The two ray points are

$$
\mathbf x_1=\mathbf p_1+\lambda_1\mathbf r_1,
\qquad
\mathbf x_2=\mathbf p_2+\lambda_2\mathbf r_2.
$$

Their gap must satisfy

$$
\|\mathbf x_1-\mathbf x_2\|\le0.25\;\text{m}
$$

with current defaults, and the landmark is initialized at

$$
\mathbf p_f=\frac{1}{2}(\mathbf x_1+\mathbf x_2).
$$

Track-manager depth defaults are 0.10–80 m and the track table is bounded to 300 tracks.

**Code:** `include/vio/VisualFeatureTrackManager.hpp`.

---

# Part V — Time synchronization, UWB/TDOA, and localization fusion

## 43. Clock-offset observations

IMU-camera offset:

$$
\Delta t_{ic}=1000(t_{cam}-t_{imu})\;\text{ms}.
$$

Anchor/reference offset:

$$
\Delta t_a=1000(t_{arrival}-t_{reference})\;\text{ms}.
$$

Peer-clock offset:

$$
\Delta t_p=1000(t_{local\_receive}-t_{remote})\;\text{ms}.
$$

## 44. Mean offset and jitter

For a rolling window $\{x_i\}_{i=1}^{N}$,

$$
\mu=\frac{1}{N}\sum_i x_i,
$$

and the implementation uses mean absolute deviation

$$
MAD=\frac{1}{N}\sum_i|x_i-\mu|.
$$

The reported jitter is the maximum MAD across IMU-camera, anchor, and peer windows.

The default maximum history is 128 samples.

## 45. Synchronization confidence

Let

$$
o_{worst}=\max(|\mu_{ic}|,|\mu_a|,|\mu_p|).
$$

The implemented confidence is

$$
c_{sync}=\operatorname{clamp}\left(
1-\frac{o_{worst}}{\max(T_{degraded},1)},0,1\right).
$$

Current $T_{degraded}=20$ ms. Synchronization is considered nominal only when both worst mean offset and maximum jitter are no greater than the 8 ms synchronized threshold.

**Code:** `include/localization/TimeSyncTracker.hpp`, `src/localization/TimeSyncTracker.cpp`.

## 46. TDOA range-difference observation

The earliest timestamp in a measurement batch is selected as the reference anchor 0. For candidate position $\mathbf x$,

$$
\rho_i=\|\mathbf x-\mathbf a_i\|,
\qquad
\rho_0=\|\mathbf x-\mathbf a_0\|.
$$

Predicted range difference:

$$
h_i(\mathbf x)=\rho_i-\rho_0.
$$

Observed range difference:

$$
y_i=c(t_i-t_0),
$$

where the current configured propagation speed default is

$$
c=299702547.0\;\text{m/s}.
$$

Residual:

$$
r_i=y_i-h_i(\mathbf x).
$$

## 47. TDOA Jacobian

Each Jacobian row is

$$
\mathbf J_i=
\frac{\mathbf x-\mathbf a_i}{\rho_i}-
\frac{\mathbf x-\mathbf a_0}{\rho_0}.
$$

Distances are lower-bounded by $10^{-3}$ m for numerical protection.

## 48. Damped TDOA least-squares solve

The update solves

$$
(\mathbf J^T\mathbf J+\lambda\mathbf I)\Delta\mathbf x
=\mathbf J^T\mathbf r
$$

with Eigen LDLT, then

$$
\mathbf x\leftarrow\mathbf x+\Delta\mathbf x.
$$

Default damping is $10^{-3}$, convergence requires

$$
\|\Delta\mathbf x\|\le10^{-4}\;\text{m},
$$

and maximum iteration count is 10.

At least four anchors and four arrival-time measurements are required. Without an external initial guess, the initial point is the anchor centroid:

$$
\mathbf x_0=\frac{1}{M}\sum_{j=1}^{M}\mathbf a_j.
$$

## 49. TDOA residual and confidence

RMS range-difference residual is

$$
e_{rms}=\sqrt{\frac{\mathbf r^T\mathbf r}{\max(n,1)}}.
$$

If the solver does not converge, the implementation forces the reported RMS to at least 5 m. TDOA confidence is then

$$
c_t=\operatorname{clamp}\left(1-\frac{e_{rms}}{6},0,1\right).
$$

This is a software confidence mapping, not a statistically calibrated posterior probability.

**Code:** `include/localization/TDOALocalizer.hpp`, `src/localization/TDOALocalizer.cpp`.

## 50. Anchor visibility ratio

The ingestor tracks anchors seen in the current source and reports

$$
r_a=\frac{N_{visible}}{N_{configured}}.
$$

A TDOA batch is not emitted until at least four valid measurements have been collected.

**Code:** `src/localization/TDOAIngestor.cpp`.

## 51. VIO/TDOA fusion weight

Let $c_t$ be TDOA confidence and $d_{vio}$ the VIO drift proxy. The implementation computes

$$
b_d=\operatorname{clamp}\left(\frac{d_{vio}}{2},0,1\right),
$$

$$
w_t=\operatorname{clamp}(0.55c_t+0.45b_d,0,0.85).
$$

Fused position is

$$
\mathbf p_f=(1-w_t)\mathbf p_{vio}+w_t\mathbf p_{tdoa}.
$$

## 52. Localization fusion confidence

The initial VIO confidence is clamped to $[0,1]$. When TDOA is valid,

$$
c\leftarrow\max\left(c,
(1-w_t)c_{vio}+w_tc_t\right).
$$

Then the current availability/timing factors are applied:

- no camera: $c\leftarrow0.72c$;
- no LiDAR and no rangefinder: $c\leftarrow0.92c$;
- unsynchronized timing: $c\leftarrow c\,\operatorname{clamp}(c_{sync},0.35,1)$;
- anchor visibility: $c\leftarrow c\,\operatorname{clamp}(0.65+0.35r_a,0.65,1)$.

When TDOA exists, $c_{sync}\ge0.8$, and $r_a\ge0.5$, a TDOA-supported floor is

$$
c_{floor}=0.55c_t+0.25\operatorname{clamp}(r_a,0,1)
+0.20\operatorname{clamp}(c_{sync},0,1),
$$

$$
c\leftarrow\max(c,c_{floor}).
$$

Confidence trend is

$$
\Delta c_k=c_k-c_{k-1}.
$$

The fusion state is lost below 0.22 and degraded below 0.58.

**Code:** `src/localization/LocalizationFusion.cpp`.

---

# Part VI — SLAM and map calculations

## 53. Occupancy-grid coordinate conversion

LiDAR point $(x,y)$ translated by drone world position is mapped to grid coordinates with

$$
g_x=\left\lfloor\frac{x_w}{r}\right\rfloor+\frac{W}{2},
$$

$$
g_y=\left\lfloor\frac{y_w}{r}\right\rfloor+\frac{H}{2},
$$

where $r$ is map resolution, and $W,H$ are cell dimensions.

Linear storage index is

$$
i=g_yW+g_x.
$$

## 54. Occupancy ratio

The implementation counts any non-zero cell as occupied/marked:

$$
r_{occ}=\frac{N_{nonzero}}{N_{cells}}.
$$

Anchor cells use a different stored marker value, so this ratio is a map-usage/marked-cell ratio rather than a pure obstacle probability.

**Code:** `src/slam/OccupancyGridMap.cpp`.

## 55. Map-planner segmentation

Planar distance is

$$
d=\|(\mathbf p_g-\mathbf p_s)_{xy}\|_2.
$$

The number of straight-line interpolation segments is

$$
N=\max\left(2,
\left\lceil\frac{d}{\max(1,4r)}\right\rceil\right).
$$

Waypoint interpolation uses

$$
t_i=\frac{i}{N},
\qquad
\mathbf p_i=\mathbf p_s+t_i(\mathbf p_g-\mathbf p_s).
$$

## 56. Occupancy-aware altitude and path cost

Occupancy penalty is

$$
p_{occ}=\operatorname{clamp}(r_{occ},0,1).
$$

Waypoint altitude is

$$
z_i=\max(z_s,z_g)+2p_{occ}+0.5\,\mathbb 1[N_{visible\ anchors}>0].
$$

Per-segment cost is

$$
c_i=\frac{d}{N}(1+p_{occ}),
$$

and total cost is cumulative

$$
C=\sum_{i=1}^{N}c_i.
$$

This planner is a simple occupancy-weighted interpolation heuristic, not A*, D*, RRT*, or an optimal trajectory solver.

**Code:** `src/slam/MapPlanner.cpp`.

## 57. Keyframe orientation difference

For normalized quaternions $q_a,q_b$,

$$
q_\Delta=q_a^{-1}\otimes q_b,
$$

$$
\Delta\theta=2\cos^{-1}(|q_{\Delta,w}|)\frac{180}{\pi}.
$$

A new keyframe is considered after the minimum time interval when translation exceeds 0.4 m or rotation exceeds 10 degrees, subject to at least 20 tracked features with current defaults.

## 58. Keyframe overlap heuristic

For previous map-point count $N_p$ and current tracked-feature count $N_t$,

$$
o=\frac{\min(N_p,N_t)}{\max(N_p,N_t)}.
$$

A keyframe may also be created when overlap is below the configured maximum overlap (default 0.9).

## 59. ORB descriptor matching

The loop/relocalization matcher uses Hamming-distance KNN. For a two-neighbor match, the current good-match rule includes a Lowe-style ratio test

$$
d_1<0.75d_2
$$

and an absolute Hamming bound $d_1\le50$. A single-neighbor match is accepted when its distance is no greater than 40.

Loop candidates require at least 15 accepted descriptor matches.

## 60. Approximate map-point projection and fusion

A keypoint bearing is transformed into world coordinates and projected to a fixed 5 m nominal depth:

$$
\mathbf p_{proj}=\mathbf p_{kf}+5\hat{\mathbf b}_w.
$$

For a matched local map point, position is updated by

$$
\mathbf p_{mp}\leftarrow0.7\mathbf p_{mp}+0.3\mathbf p_{proj},
$$

and confidence increases by 0.05 up to 1.0.

For a matched remote observation, the position blend is

$$
\mathbf p_{mp}\leftarrow0.5\mathbf p_{mp}+0.5\mathbf p_{proj},
$$

and confidence increases by 0.08 up to 1.0.

These are map-maintenance heuristics, not probabilistic triangulation updates.

## 61. Relocalization confidence and pose blending

The best descriptor-match score $s$ must be at least 18. Confidence is

$$
c_{reloc}=\operatorname{clamp}\left(\frac{s}{80},0,1\right).
$$

Corrected position is

$$
\mathbf p_c=0.7\mathbf p_{matched}+0.3\mathbf p_{guess}.
$$

Orientation is spherical-linear interpolation:

$$
\mathbf q_c=\operatorname{slerp}(\mathbf q_{matched},\mathbf q_{guess},0.3).
$$

A relocalization event counter is incremented when confidence reaches at least 0.45.

**Code for Sections 57–61:** `include/slam/KeyframeManager.hpp`, `src/slam/KeyframeManager.cpp`.

---

# Part VII — Autonomy and experience-memory calculations

## 62. Perception focus geometry

For normalized detection box $(x,y,w,h)$,

$$
c_x=x+\frac{w}{2},
\qquad
c_y=y+\frac{h}{2},
$$

$$
\mathbf o=
\begin{bmatrix}c_x-0.5\\c_y-0.5\end{bmatrix}.
$$

Normalized area is

$$
A=\operatorname{clamp}(wh,0,1).
$$

Centering score is

$$
s_c=\operatorname{clamp}(1-1.4\|\mathbf o\|,0,1).
$$

The rough distance proxy used only by the behavior layer is

$$
d_{proxy}=\frac{1}{\sqrt{\max(A,10^{-3})}}.
$$

This is not a calibrated monocular range estimate.

## 63. Detection priority score

With detector confidence $c_d$ and class multiplier $b_c$,

$$
s_{det}=c_d(0.45+0.35s_c+0.20A)b_c.
$$

Current class multipliers are 1.25 for configured hazard labels, 1.15 for unknown labels, 1.0 for configured mission-target labels, and 0.65 otherwise.

## 64. Caution and localization speed scales

Experience-memory caution scaling is

$$
s_{caution}=\operatorname{clamp}(1-0.18r_{risk},0.55,0.90)
$$

when the memory prior requests caution; otherwise it is 1.

Localization speed scaling is

$$
s_{loc}=\operatorname{clamp}(c_{loc},0.35,1).
$$

Many behavior speed limits use the product

$$
s_v=s_{caution}s_{loc}.
$$

## 65. Speed-vector saturation

For requested velocity $\mathbf v$ and limit $v_{max}$,

$$
\mathbf v_{cmd}=
\begin{cases}
\mathbf v\frac{v_{max}}{\|\mathbf v\|},&\|\mathbf v\|>v_{max},\\
\mathbf v,&\text{otherwise}.
\end{cases}
$$

This norm-preserving saturation is used throughout autonomy and safety.

## 66. Return-home position blending

If TDOA confidence is at least 0.45, the position used for return-home direction is

$$
\mathbf p_{rh}=0.45\mathbf p_{vio}+0.55\mathbf p_{tdoa}.
$$

The home error is

$$
\mathbf e_h=\mathbf p_{home}-\mathbf p_{rh}.
$$

Vertical error is separately clamped before speed saturation.

## 67. Search command

With body-forward unit vector $\hat{\mathbf f}$, body-up vector $\hat{\mathbf u}$, nominal-altitude error $e_z$, and scale $s_v$,

$$
\mathbf v_{search}=
\hat{\mathbf f}(v_{search,max}s_v)+
\hat{\mathbf u}\operatorname{clamp}(0.18e_z,-0.5,0.5),
$$

followed by norm saturation to $v_{search,max}s_v$.

The scan yaw alternates between ±0.18 rad/s based on the target-miss counter parity.

## 68. Target-tracking command

The desired normalized target area is 0.12:

$$
e_A=0.12-A.
$$

The unconstrained velocity is a sum of forward range-control and image-plane centering terms:

$$
\mathbf v=
\hat{\mathbf f}\operatorname{clamp}(8e_A,-0.3,v_{track,max}s_v)
+\hat{\mathbf r}\operatorname{clamp}(2.2o_x,-1,1)
+\hat{\mathbf u}\operatorname{clamp}(-1.6o_y,-0.8,0.8).
$$

Yaw rate is

$$
\dot\psi=\operatorname{clamp}(1.8o_x,-0.9,0.9).
$$

## 69. Camera-obstacle avoidance command

The avoidance velocity includes reverse motion proportional to apparent object area, lateral image error, and positive altitude recovery:

$$
\mathbf v=
-\hat{\mathbf f}(0.9+2A)
-\hat{\mathbf r}\operatorname{clamp}(3o_x,-1.2,1.2)
+\hat{\mathbf u}\operatorname{clamp}(0.15e_z,0,0.9).
$$

The final vector is speed-limited using caution and localization scaling.

## 70. LiDAR-obstacle escape gain

For nearest obstacle distance $d$,

$$
d_s=\max(0.1,d),
$$

$$
k_{escape}=\operatorname{clamp}\left(\frac{2.5}{d_s},0.8,2.4\right).
$$

Reverse avoidance is proportional to $k_{escape}$ plus a bounded upward recovery component.

## 71. Hold and localization-recovery damping

Several recovery modes apply proportional velocity damping. Examples implemented in the current behavior logic include

$$
\mathbf v_{hold}=-0.35\mathbf v,
$$

$$
\mathbf v_{hover}\supset-0.55\mathbf v,
$$

$$
\mathbf v_{degraded}\supset-0.45\mathbf v,
$$

$$
\mathbf v_{lost}\supset-0.60\mathbf v.
$$

These are policy gains and must be tuned against real vehicle dynamics before physical use.

**Code for Sections 62–71:** `src/autonomy/DecisionEngine.cpp`.

## 72. Experience-memory trend estimation

For observations $(t_0,y_0)$ and $(t_1,y_1)$, the stored per-minute slope is

$$
s_y=\frac{y_1-y_0}{\max(t_1-t_0,10^{-6})}\times60.
$$

Drift trend uses this directly. Battery-burn rate negates the battery slope so discharge becomes positive.

## 73. Experience frequencies and means

For history length $N$,

$$
f_{hazard}=\frac{N_{hazards}}{N},
\qquad
f_{target}=\frac{N_{targets}}{N},
$$

$$
\bar c_{loc}=\frac{1}{N}\sum_i c_{loc,i},
$$

$$
f_{dropout}=\frac{N_{lost\ or\ }c<0.35}{N},
$$

$$
f_{lowfeature}=\frac{N_{observations\ with\ zero\ detections}}{N}.
$$

## 74. Experience-memory risk score

The current heuristic risk function is

$$
r=\operatorname{clamp}\Big(
0.32f_{hazard}
+0.85\max(0,s_{drift})
+0.06\max(0,s_{battery})
+0.70f_{dropout}
+0.22f_{lowfeature}
+0.55\max(0,0.70-\bar c_{loc}),
0,1.5\Big).
$$

Caution is recommended when this score exceeds the configured caution threshold.

This is a deterministic experience heuristic, not a learned probabilistic risk model.

**Code:** `src/autonomy/ExperienceMemory.cpp`.

---

# Part VIII — Local safety calculations

## 75. Safety speed saturation

The safety manager uses the same Euclidean norm clamp:

$$
\mathbf v_s=\mathbf v\min\left(1,\frac{v_{max}}{\max(\|\mathbf v\|,\epsilon)}\right).
$$

The configured safety state can only reduce an autonomy command's acceleration limit:

$$
a_{cmd,max}\leftarrow\min(a_{cmd,max},a_{safety,max}).
$$

## 76. Emergency descent direction

Body-up in world coordinates is

$$
\hat{\mathbf u}=\mathbf R_{wb}
\begin{bmatrix}0&1&0\end{bmatrix}^T.
$$

Emergency descent is

$$
\mathbf v_{emergency}=-\hat{\mathbf u}\,v_{emergency},
$$

with current default $v_{emergency}=0.85$ m/s.

## 77. Link/sensor fault hold

For link or required-sensor faults, the existing velocity is damped before the speed constraint:

$$
\mathbf v_{hold}=\operatorname{sat}(-0.55\mathbf v,v_{max}).
$$

## 78. Localization-lost safety command

The localization-loss safety command combines damping and controlled descent:

$$
\mathbf v_{lost}=\operatorname{sat}
\left(-0.60\mathbf v-\hat{\mathbf u}v_{lost\_descent},v_{max}\right),
$$

with current default $v_{lost\_descent}=0.18$ m/s.

Selected safety defaults are:

| Parameter | Default |
|---|---:|
| indoor max speed | 0.75 m/s |
| indoor max acceleration | 0.60 m/s² |
| low-VIO confidence threshold | 0.55 |
| low-VIO max speed | 0.35 m/s |
| low-VIO max acceleration | 0.40 m/s² |
| localization-lost descent | 0.18 m/s |
| emergency descent | 0.85 m/s |

**Code:** `include/safety/SafetyManager.hpp`, `src/safety/SafetyManager.cpp`.

---

# Part IX — Swarm coordination and formation calculations

## 79. Leadership score

An emergency-fault node receives zero leadership score. Otherwise battery is normalized as

$$
b=\operatorname{clamp}\left(\frac{battery\_%}{100},0,1\right).
$$

The implemented score is

$$
L=\operatorname{clamp}\left(
0.32b+0.28h_m+0.18q_l+0.12h_{cpu}+0.10h_{thermal},0,1
\right),
$$

where motor health, link quality, CPU headroom, and thermal headroom are individually clamped to $[0,1]$.

Leader election chooses the reachable candidate with the greatest score; near-equal scores use battery and then node ID as deterministic tie-breakers.

Re-election can be forced by emergency fault or current policy thresholds including motor health <0.35, link quality <0.25, battery <18%, or leadership score <0.40.

**Code:** `src/swarm/V2XMeshNetwork.cpp`.

## 80. Formation geometry

For spacing $s$ and leader position $\mathbf p_L$, LINE formation follower $i$ uses

$$
\mathbf o_i=
\begin{bmatrix}0&-(i+1)s&0\end{bmatrix}^T.
$$

VEE formation uses fixed $30^\circ$ arms. With row

$$
r=\left\lfloor\frac{i}{2}\right\rfloor+1
$$

and side $\eta\in\{-1,+1\}$,

$$
\mathbf o_i=
\begin{bmatrix}
\eta rs\sin30^\circ\\
-rs\cos30^\circ\\
0
\end{bmatrix}.
$$

Diamond offsets are explicitly stored as $(s,0)$, $(-s,0)$, $(0.7s,-s)$, and $(-0.7s,-s)$.

Target position is

$$
\mathbf p_i^*=\mathbf p_L+\mathbf o_i.
$$

## 81. Formation P-controller

Tracking error is

$$
\mathbf e=\mathbf p^*-\mathbf p.
$$

Preferred velocity is

$$
\mathbf v_p=k_p\mathbf e,
$$

with default $k_p=1.5$ for the public controller interface.

Avoidance is added before final speed saturation:

$$
\mathbf v=\mathbf v_p+\mathbf v_{avoid}.
$$

When the tracking command is substantial but the combined vector nearly cancels, the implementation injects a deterministic lateral escape velocity of 0.8 m/s to avoid a simple local minimum.

## 82. Relative closing speed

For relative position

$$
\mathbf r=\mathbf p_{self}-\mathbf p_{obj}
$$

and relative velocity

$$
\mathbf v_r=\mathbf v_{obj}-\mathbf v_{self},
$$

closing speed along the line of sight is

$$
v_c=-\mathbf v_r^T\frac{\mathbf r}{\max(\|\mathbf r\|,\epsilon)}.
$$

Positive $v_c$ means separation is closing under the implemented sign convention.

## 83. Separation-field weight

Let $R$ be influence radius and $d_{min}$ minimum separation. The normalized gap is

$$
g=\operatorname{clamp}\left(
\frac{R-d}{\max(R-d_{min},\epsilon)},0,1\right).
$$

For a peer, base repulsion weight is

$$
w=g^2.
$$

Inside minimum separation,

$$
w\leftarrow w+1+\frac{d_{min}-d}{\max(d_{min},\epsilon)}.
$$

For a static/dynamic LiDAR-derived obstacle, base weight starts as

$$
w=0.8g^2,
$$

and inside minimum separation adds

$$
1.1+\frac{d_{min}-d}{\max(d_{min},\epsilon)}.
$$

## 84. Time-to-collision augmentation

For $v_c>0$,

$$
t_{tc}=\frac{d}{\max(v_c,\epsilon)}.
$$

If $t_{tc}<T_p$, peer weight receives

$$
\Delta w=\frac{T_p-t_{tc}}{\max(T_p,0.1)}.
$$

For a dynamic external obstacle, the same term is multiplied by 0.75.

Default public parameters are minimum separation 2 m, influence radius 6 m, maximum avoidance speed 3 m/s, and prediction horizon 2 s.

## 85. Final avoidance vector

Weighted unit separation directions are summed:

$$
\mathbf r_{sum}=\sum_jw_j\hat{\mathbf d}_j.
$$

After normalization, magnitude is controlled by the strongest hazard weight:

$$
\mathbf v_{avoid}=\frac{\mathbf r_{sum}}{\|\mathbf r_{sum}\|}
\min(v_{avoid,max},v_{avoid,max}w_{max}).
$$

Vertical components are intentionally attenuated relative to horizontal separation.

**Code for Sections 80–85:** `include/swarm/V2XMeshNetwork.hpp`, `src/swarm/V2XMeshNetwork.cpp`.

## 86. Consensus quorum

Supporters are stored as unique peer IDs. The effective quorum is

$$
Q_{eff}=\max(Q_{proposal},Q_{default}).
$$

Quorum is met when

$$
|\mathcal S|\ge Q_{eff}.
$$

Collective action is allowed only if a non-empty proposal has quorum and local safety override is false.

Stale supporters are removed before quorum is recomputed.

**Code:** `src/swarm/EdgeConsensusManager.cpp`.

## 87. Peer age, stale state, and expiry

Peer age is

$$
a_{peer}=t_{now}-t_{last}.
$$

Current cache defaults are:

- stale when $a_{peer}>900$ ms;
- remove entry when $a_{peer}>2500$ ms.

A peer is safety-eligible only when it is not stale, is not reporting disconnected operation, is not in fault edge-health state, and has a positive trust epoch.

The swarm cache reports disconnected operation when it is empty or contains no safety-eligible peers.

**Code:** `include/swarm/SwarmStateCache.hpp`, `src/swarm/SwarmStateCache.cpp`.

---

# Part X — Peer transport calculations

## 88. Swarm-message fixed-point timestamp

The older binary swarm envelope stores seconds as integer nanoseconds:

$$
t_{ns}=\lfloor t_s\times10^9\rfloor,
$$

and decodes with

$$
t_s=t_{ns}\times10^{-9}.
$$

**Code:** `src/swarm/V2XMeshNetwork.cpp`.

## 89. Edge packet lifetime

An edge packet is invalid when its age exceeds its TTL. The implementation also rejects zero TTL and non-monotonic sequence numbers.

At the authentication layer, absolute clock difference is

$$
a_t=|t_{now}-t_{packet}|.
$$

A packet is rejected by the auth clock-skew guard when

$$
a_t>T_{skew}+TTL.
$$

## 90. Serialization compression ratio

For encoded packet size $B_e$ and JSON-equivalent size $B_j$,

$$
r_c=\frac{B_e}{B_j}.
$$

Encode and decode latency are measured using the monotonic clock around the serializer/parser call and reported in microseconds.

The packet validator defaults to a 1400-byte application-size limit. The CBOR reader also limits strings to 512 bytes and arrays to 64 elements.

## 91. Current mesh-bandwidth indicator

The current `mesh_bandwidth_kbps()` function returns

$$
B_{indicator}=B_{local}+\frac{B_{tx,total}+B_{rx,total}}{128}.
$$

Despite the field name, the second term is based on cumulative byte counters and is not divided by a time window. Therefore it must be treated as a runtime indicator, not a physically measured kbps throughput value. A future implementation should compute a windowed rate

$$
B_{rate}=\frac{8\Delta B}{1000\Delta t}
$$

before presenting it as measured kbps.

**Code:** `src/swarm/EdgePeerProtocol.cpp`, `src/swarm/V2XMeshNetwork.cpp`, `src/security/PeerPacketAuth.cpp`.

---

# Part XI — Security and integrity constructions

## 92. Canonical peer-packet hash

For edge peer packet $P$, authentication first clears the mutable `auth_hook`, serializes the remaining packet to canonical CBOR, and computes

$$
h=SHA256(CBOR(P_{canonical})).
$$

The HMAC mode computes

$$
MAC=HMAC\text{-}SHA256(K,C(P)).
$$

The transmitted hook stores the hexadecimal MAC. Verification recomputes the value and uses a constant-time byte comparison.

**Code:** `src/security/PeerPacketAuth.cpp`.

## 93. Replay and trust-epoch gates

A packet must satisfy exact trust-epoch equality

$$
E_{packet}=E_{configured}
$$

and sequence monotonicity

$$
s_k>s_{last}.
$$

These are discrete security policy checks rather than statistical tests.

## 94. Per-node key derivation for the secured swarm envelope

For node ID $i$, salt is the UTF-8 string

```text
swarm-node-<i>
```

and the implementation derives 128 bytes using

$$
D=PBKDF2\text{-}HMAC\text{-}SHA256(secret,salt,iterations,128).
$$

The default iteration count is 120000. The 128 derived bytes are split into four 32-byte segments:

$$
D=K_{enc}\|K_{mac}\|K_{eddsa-seed}\|K_{future}.
$$

The EdDSA seed is expanded by Monocypher into the signing key pair.

## 95. Encrypted swarm frame

Plain message bytes are encrypted with AES-256-CBC and a fresh 16-byte IV:

$$
C=AES\text{-}256\text{-}CBC_{K_{enc},IV}(M).
$$

The secure header includes source ID, sequence, issue time, IV, digest fields, previous ledger hash, and ciphertext length.

## 96. Present, past, and future digests

Present digest:

$$
h_{present}=SHA256(M).
$$

If a previous frame exists, the past field is

$$
h_{past}=SHA1(h_{frame,k-1}).
$$

The future-bound field is

$$
h_{future}=SHA3\text{-}256(M\|K_{future}\|t_{issued}).
$$

These fields are verified after decryption. Their presence is an implementation-specific integrity/history construction; it should not be described as a formal blockchain or consensus proof.

## 97. Frame MAC and signature

MAC input is header concatenated with ciphertext:

$$
MAC=HMAC\text{-}SHA256(K_{mac},H\|C).
$$

The signature covers

$$
S=EdDSA_{sk}(H\|MAC\|C).
$$

The receiver verifies the MAC, EdDSA signature, decrypted present/future/past digests, and message/header identity before advancing peer state.

## 98. Ledger-style previous-frame linkage

For selected message types such as formation, leader election, mission sync, and emergency stop, the header carries the previous accepted ledger frame hash:

$$
h_{prev}=SHA256(F_{ledger,k-1}).
$$

The receiver requires exact equality with its retained previous ledger hash before accepting the new linked frame.

**Code for Sections 94–98:** `include/swarm/SwarmSecurity.hpp`, `src/swarm/SwarmSecurity.cpp`.

## 99. Onboard security health score

The runtime security monitor computes a heuristic tamper score

$$
t=\operatorname{clamp}\Big(
 t_{input}
+0.20\,\mathbb 1[N_{replay}>0]
+0.25\,\mathbb 1[N_{spoof}>0]
+0.12\,\mathbb 1[N_{control}>0]
+0.10\,\mathbb 1[N_{auth}>0],0,1\Big).
$$

Link integrity is

$$
L=\operatorname{clamp}\Big(
0.35q_{link}+0.25c_{sync}+0.15c_{loc}+0.15t_{issuer}
+0.10p,0,1\Big),
$$

where

$$
p=\begin{cases}1,&N_{peer}>0\\0.65,&N_{peer}=0.\end{cases}
$$

Policy thresholds then map the current signals to states such as `DEGRADED_LINK`, `COMMAND_REPLAY_SUSPECT`, `PEER_SPOOF_SUSPECT`, `SAFE_RETURN`, or `LAND_IMMEDIATELY`. Security-state transitions increment the trust epoch.

These scores are rule-based software heuristics, not calibrated probabilities of attack.

**Code:** `include/security/DroneSecurity.hpp`.

## 100. Security-event decay windows

The runtime monitor clears stale counters after fixed windows: authorization failures after 90 s and replay, spoof, and control-plane failures after 120 s.

This creates finite-memory state for security policy rather than permanently accumulating historical faults.

---

# Part XII — Control-plane command and audit integrity

## 101. Signed command canonical string

The Go control plane signs a newline-joined canonical sequence containing

```text
action
payload_json
operator_id
normalized_role
issued_at
expires_at
nonce
```

with

$$
S_{cmd}=HMAC\text{-}SHA256(K_{operator},canonical\_command).
$$

The result is hexadecimal encoded and verified with Go's constant-time HMAC comparison.

## 102. Command validity interval

The command must satisfy

$$
t_{expire}>t_{issue},
$$

$$
t_{expire}-t_{issue}\le TTL_{max},
$$

with default maximum TTL 90 s.

Future issue time may not exceed

$$
t_{issue}\le t_{now}+T_{skew}
$$

and expiry may not be older than

$$
t_{expire}\ge t_{now}-T_{skew},
$$

with default command skew 30 s.

A nonce is retained until the later of its expiry or current time plus an additional default 5-minute retention interval, preventing replay during that window.

**Code:** `internal/controlplane/security.go`.

## 103. Audit hash chain

Commands and events are appended to a SHA-256 audit chain. For record fields $R_k$ and previous hash $h_{k-1}$,

$$
h_k=SHA256(kind\|subject\|message\|actor\|payload\|h_{k-1}\|timestamp),
$$

where the implementation joins canonical string fields with newline separators and JSON-encodes the payload.

**Code:** `internal/controlplane/state.go`.

## 104. Fleet aggregation

For $N$ drones, arithmetic means are used for metrics such as CPU temperature, GPU load, peer latency, mesh-bandwidth field, and battery health where applicable:

$$
\bar x=\frac{1}{N}\sum_{i=1}^{N}x_i.
$$

Cluster average battery is similarly accumulated and divided by cluster size.

A drone contributes to the critical-alert count when any implemented critical predicate is true, including battery <15%, CPU temperature >82°C, unreachable/stale state, lost localization, sync confidence <0.35, or selected non-nominal security states.

The current snapshot's `PacketLossPct` field is populated with a fixed placeholder value rather than a measured estimator. It must not be cited as experimental packet-loss data.

**Code:** `internal/controlplane/state.go`.

---

# Part XIII — Estimator comparison and sustained-readiness calculations

## 105. Active-versus-secondary timing difference

Comparable snapshots use

$$
\Delta t_{ms}=1000|t_a-t_s|.
$$

The comparison is rejected as stale when the age exceeds the configured maximum comparison lag.

## 106. State-difference norms

Position difference:

$$
d_p=\|\mathbf p_a-\mathbf p_s\|_2.
$$

Velocity difference:

$$
d_v=\|\mathbf v_a-\mathbf v_s\|_2.
$$

Accelerometer-bias difference:

$$
d_{ba}=\|\mathbf b_{a,a}-\mathbf b_{a,s}\|_2.
$$

Gyroscope-bias difference:

$$
d_{bg}=\|\mathbf b_{g,a}-\mathbf b_{g,s}\|_2.
$$

Covariance-trace difference:

$$
d_P=|\operatorname{tr}(P_a)-\operatorname{tr}(P_s)|.
$$

## 107. Quaternion angular difference

After normalizing both quaternions,

$$
d_q=\operatorname{clamp}(|q_a^Tq_s|,0,1),
$$

$$
\Delta\theta=2\cos^{-1}(d_q)\frac{180}{\pi}.
$$

The absolute quaternion dot handles the sign equivalence $q$ and $-q$.

**Code:** `src/vio/EstimatorCoordinator.cpp`.

## 108. Readiness gate for estimator comparison

The current comparison-readiness helper is a strict conjunction, not a weighted score. Defaults require:

- at least 100 valid comparisons;
- $d_p\le0.25$ m;
- $d_v\le0.35$ m/s;
- $\Delta\theta\le5^\circ$;
- $d_P\le1.0$;
- zero queue drops;
- zero stale queued measurements;
- zero shadow-processing failures;
- both estimator health states healthy and worker lifecycle running.

This function does not automatically promote the secondary estimator to flight authority.

**Code:** `include/vio/EstimatorPromotionReadiness.hpp`.

## 109. Sustained-readiness monitor

Let $R_k\in\{0,1\}$ be the readiness result of sample $k$. The monitor tracks consecutive ready and blocked samples. With default maximum consecutive blocked samples equal to zero and minimum ready samples 500,

$$
ready_{sustained}=
(N_{blocked,consecutive}\le0)\land
(N_{ready,consecutive}\ge500).
$$

With default reset-on-block enabled, any blocked sample resets the consecutive-ready counter.

This is a soak-test policy primitive, not an automatic estimator promotion mechanism.

**Code:** `include/vio/EstimatorPromotionSoakMonitor.hpp`.

---

# Part XIV — Design/planning models that are not direct runtime equations

The README also uses several analytical equations to reason about scaling and future experiments. They are retained here separately so they are not confused with implemented runtime calculations.

## 110. Peer-information confidence decay model

A proposed analytical model for shared information is

$$
C_{final}=C_{initial}
\exp(-\lambda_t\Delta t)
\exp(-\lambda_hh)
T(E)D(local\ consistency).
$$

This exponential confidence function is a design/research model; the current peer cache primarily implements TTL, stale-state, trust-epoch, and safety-eligibility gates rather than this exact continuous decay equation.

## 111. Swarm-bandwidth planning model

For $N$ senders, packet size $B$, and update rate $f$,

$$
B_{total}\approx NBf.
$$

A simple full-mesh approximation is

$$
B_{mesh}\approx N(N-1)Bf.
$$

These equations are planning models. Actual radio throughput must be measured over a time window on target hardware.

## 112. Latency decomposition

A useful analytical decomposition is

$$
T_{total}=T_{sensor}+T_{fusion}+T_{coordination}+T_{network}+T_{actuation}.
$$

The local safety path is intentionally modeled without backend round-trip latency:

$$
T_{local\ safety}=T_{sensor}+T_{fusion}+T_{safety}+T_{actuation}.
$$

## 113. Independent-reference drift measurement

When physical ground truth is available,

$$
drift(t)=\|\mathbf p_{estimated}(t)-\mathbf p_{reference}(t)\|_2.
$$

A finite-window drift-rate estimate is

$$
\dot d\approx\frac{drift(t_2)-drift(t_1)}{t_2-t_1}.
$$

The current covariance-derived `drift_m`/uncertainty values are not a substitute for this independent-reference error measurement.

---

# Part XV — Source-to-equation index

| Subsystem | Main calculations | Source |
|---|---|---|
| IMU acquisition | raw scaling, temperature, static bias mean, sample-rate estimate | `src/sensors/IMUSensor.cpp` |
| Camera | normalization, detector box scaling, confidence/NMS gate, intrinsics/undistortion | `src/sensors/CameraSensor.cpp` |
| LiDAR | Cartesian range, azimuth, elevation, range filtering | `src/sensors/LidarSensor.cpp` |
| Motor health | temperature/vibration/current penalties, average health | `src/sensors/MotorSensor.cpp` |
| Baseline EKF | inertial propagation, covariance, projection, Mahalanobis, Kalman/Joseph, confidence | `src/vio/EKFEstimator.cpp` |
| Secondary ESKF | error-state dynamics, second-order discretization, reset, FEJ, ZUPT | `src/vio/Phase17ESKFEstimator.cpp` |
| MSCKF | clone augmentation, triangulation, nullspace, chi-square gate, marginalization | `src/vio/Phase17ESKFEstimator.cpp`, `include/vio/MsckfMarginalization.hpp` |
| Visual frontend | flow/inlier metrics, confidence, essential-matrix direction, predicted metric scale | `src/vio/VIOPipeline.cpp` |
| Track manager | pixel association, parallax, two-ray QR triangulation, midpoint | `include/vio/VisualFeatureTrackManager.hpp` |
| Estimator comparison | time, state/bias norms, quaternion angle, covariance trace | `src/vio/EstimatorCoordinator.cpp` |
| Estimator readiness | strict delta/health gates and sustained-ready counters | `include/vio/EstimatorPromotionReadiness.hpp`, `include/vio/EstimatorPromotionSoakMonitor.hpp` |
| Time synchronization | mean offsets, MAD jitter, confidence | `src/localization/TimeSyncTracker.cpp` |
| TDOA | range differences, Jacobian, damped least squares, RMS/confidence | `src/localization/TDOALocalizer.cpp` |
| Localization fusion | adaptive TDOA weight, confidence factors/trend | `src/localization/LocalizationFusion.cpp` |
| Occupancy map | grid projection and occupancy ratio | `src/slam/OccupancyGridMap.cpp` |
| Map planner | interpolation segments, altitude heuristic, cost | `src/slam/MapPlanner.cpp` |
| Keyframe SLAM | quaternion delta, overlap, descriptor ratio, map blending, relocalization | `src/slam/KeyframeManager.cpp` |
| Autonomy | detection score, motion scales, search/track/avoid vectors | `src/autonomy/DecisionEngine.cpp` |
| Experience memory | slopes, frequencies, mean confidence, risk score | `src/autonomy/ExperienceMemory.cpp` |
| Safety | velocity/acceleration limiting, damping, descent commands | `src/safety/SafetyManager.cpp` |
| Leader election | weighted health score and deterministic tie break | `src/swarm/V2XMeshNetwork.cpp` |
| Formation | geometric offsets, P control, speed saturation | `src/swarm/V2XMeshNetwork.cpp` |
| Collision avoidance | inverse separation field, closing speed, TTC term | `src/swarm/V2XMeshNetwork.cpp` |
| Consensus/cache | quorum, age/staleness, safety-eligible count | `src/swarm/EdgeConsensusManager.cpp`, `src/swarm/SwarmStateCache.cpp` |
| Edge protocol | packet age, size guard, CBOR/JSON size and latency ratio | `src/swarm/EdgePeerProtocol.cpp` |
| Peer auth | canonical SHA-256, HMAC-SHA256, epoch/sequence/clock gates | `src/security/PeerPacketAuth.cpp` |
| Secure swarm envelope | PBKDF2, AES-256-CBC, HMAC, EdDSA, digest/history linkage | `src/swarm/SwarmSecurity.cpp` |
| Onboard security policy | link-integrity and tamper heuristics, time-decayed fault counters | `include/security/DroneSecurity.hpp` |
| Go command security | HMAC command signature, TTL/skew, nonce retention | `internal/controlplane/security.go` |
| Control-plane audit | SHA-256 hash chain and fleet arithmetic means | `internal/controlplane/state.go` |

---

# Part XVI — Reproducibility rules for mathematical claims

When publishing or benchmarking this repository, report each result with its source category:

1. **Analytical/design model** — equation used for reasoning but not directly evaluated in the runtime.
2. **Runtime-derived software metric** — value calculated by code from current inputs or internal state.
3. **Simulation/replay result** — output from synthetic or recorded inputs.
4. **Bench/HIL measurement** — measurement from hardware-in-the-loop or controlled hardware bench setup.
5. **Physical flight measurement** — result from an independently instrumented real-flight experiment.

Do not merge these categories in a table or graph without labeling them. In particular:

- covariance-derived uncertainty is not independent ground-truth position error;
- TDOA confidence is a residual-based software mapping, not a calibrated probability;
- visual confidence and memory risk are deterministic software heuristics;
- leader/security scores are policy functions, not learned probabilities;
- simulated sensor constants are not hardware measurements;
- the current mesh-bandwidth indicator is not yet a window-normalized RF throughput estimator;
- a repeatable replay is evidence of software repeatability under the replay conditions, not deterministic physical behavior.

For an academic experiment, record at minimum the repository commit SHA, compiler and dependency versions, configuration files, sensor calibration, anchor coordinates, clock source, input dataset/log identifier, runtime mode, radio/hardware setup, and the independent reference used for error calculation.
