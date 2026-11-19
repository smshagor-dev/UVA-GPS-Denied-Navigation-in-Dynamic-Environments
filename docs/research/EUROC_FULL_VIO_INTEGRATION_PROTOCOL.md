# EuRoC Phase 81: stereo-derived visual odometry + native inertial fusion

**Status:** implementation and synthetic CI validation; no authentic EuRoC performance established.

## Architecture

1. **Original sensor intake** (#76) validates raw imu0, cam0, cam1 metadata/images/CSV. Original input files are never modified and publisher ground-truth files are not read.
2. **Calibration gate** (#78) validates `T_BS` sensor-to-body transforms, right-handed rotations, camera intrinsics, radial-tangential distortion and noise parameters.
3. **Exact stereo/IMU timestamp synchronization** (#80) validates camera pairing, sensor gaps, missing observations and source hashes; reject records remain reported.
4. **Calibrated stereo frontend** (#79) detects and ratio-matches ORB landmarks in undistorted/rectified stereo pairs and triangulates positive-depth metric 3D points.
5. **New temporal VO** (`euroc_stereo_visual_odometry.py`) uses rectified left-camera optical flow and RANSAC PnP against the **previous frame's stereo-derived 3D points**. Its `T_Ccurrent_Cprevious` measurements are integrated into an IMU-anchored world coordinate frame using calibrated `T_I_Crect`. Source images and calibration are hashed; no ground truth is involved.
6. **New native EKF/ESKF replay** (`euroc_stereo_imu_native_fusion.cpp`) reads genuine imu0/data.csv and the VO-generated visual.csv, enforces chronological and positive-quality input, and invokes the repository's existing `EKFStateEstimatorAdapter` and `Phase17StateEstimatorAdapter`. Visual pose and attitude observations update **position, velocity and a separately gated quaternion orientation**. Each native estimator emits `timestamp_ns,x_m,y_m,z_m,qw,qx,qy,qz`. The runner additionally exports immutable-position columns as `active_position_only.csv` and `shadow_position_only.csv` for the legacy independent ATE/RPE evaluator. If any active/shadow event is rejected, replay fails rather than silently producing a benchmark row.

**Important method limitation:** The visual frontend now supplies right-handed `q_W_I` through a distinct `VisualAttitudeMeasurementPayload`; both EKF and ESKF perform a body-right quaternion error-state correction with a conservative outlier gate. This is **experimental 6-DoF software fusion, not validated robust 6-DoF VIO**: initial attitude defaults to identity (gravity unverified); the optional stationary-gravity mode estimates **tilt only** using a strict pre-motion IMU interval, rejects moving/short starts, and cannot estimate yaw; sequential PnP orientation/position updates are temporally correlated; the 0.25 rad uncertainty is provisional and uncalibrated; absolute heading remains unobservable; and no authentic EuRoC ATE/RPE or orientation error evidence is reported. Do not claim genuine robustness, flight performance or accuracy on this basis.

## Reproducibility (Linux)

Prepare an original, independently downloaded EuRoC MAV sequence with the exact `mav0/imu0/data.csv`, `mav0/{cam0,cam1}/data.csv`, camera PNG images and `sensor.yaml` files. Independently verify publisher archive provenance and licensing.

```bash
python3 -m pip install "numpy>=2.3,<3" "PyYAML>=6.0.3,<7" "opencv-python-headless>=4.10,<6"
python3 -m unittest tests.test_euroc_stereo_visual_odometry -v

# Independently verify timestamp pairing/drop accounting first.
python3 -m research.experiments.euroc_stereo_imu_synchronization \
  --sequence-root /data/EuRoC/MH_01_easy \
  --events /data/exports/MH_01_easy_sync.jsonl \
  --manifest /data/exports/MH_01_easy_sync.json \
  --hash-images

# Extract visual odometry observations without ground truth.
python3 -m research.experiments.euroc_stereo_visual_odometry \
  --sequence-root /data/EuRoC/MH_01_easy \
  --output-visual-csv /data/exports/MH_01_easy_visual.csv \
  --manifest /data/exports/MH_01_easy_visual_manifest.json \
  --max-frames 0

# Build and run the native fused estimator on a machine with C++ dependencies.
cmake --preset linux-gcc-release
cmake --build --preset linux-gcc-release --target euroc_stereo_imu_native_fusion -- -j2
ctest --preset linux-gcc-release -R '^euroc_stereo_imu_native_fusion_selftest$' --output-on-failure
build/linux-gcc-release/tests/Release/euroc_stereo_imu_native_fusion \
  --imu-csv /data/EuRoC/MH_01_easy/mav0/imu0/data.csv \
  --visual-csv /data/exports/MH_01_easy_visual.csv \
  --active-output /data/exports/MH_01_easy_active_estimated.csv \
  --shadow-output /data/exports/MH_01_easy_shadow_estimated.csv \
  --assume-initial-imu-identity
```

Adjust the build output executable path for the selected CMake multi-config preset and system. All outputs must be new paths outside the original dataset. The Python visual CSV contains fixed non-benchmark uncertainty weights `sigma_position_m=0.5`, `sigma_velocity_mps=0.75`, `sigma_orientation_rad=0.25`, `qw,qx,qy,qz`, PnP inlier count and reprojection RMSE. It is NOT legitimate to reinterpret these constants as measured uncertainties.

## One-command end-to-end evidence collection

After building the native C++ binary, use the orchestration wrapper to
validate synchronization, extract visual odometry, run both native estimators
and verify accepted measurement timestamps and SHA-256 digests:

```bash
python3 -m research.experiments.euroc_vio_run \
  --sequence-root /data/EuRoC/MH_01_easy \
  --native-binary build/linux-gcc-release/tests/euroc_stereo_imu_native_fusion \
  --output-dir /data/exports/MH_01_easy_trial01 \
  --max-frames 0 \
  --min-inliers 12
```

The output directory must **not already exist**, may not be inside the
source sequence, and retains a `run_manifest.json` with explicit
`status: failed` for any interrupted or invalid run. Successful runs
are labeled `completed_unverified_software_replay`, never
`publishable_accuracy_evidence`. The runner hashes all generated
synchronization/visual/native files, checks native counter totals,
and rejects native trajectory missing/duplicate/reordered timestamps.
The actual native file location varies by platform/CMake preset.

The visual manifest contains an ordered `frame_audit` ledger with one
timestamped entry for each examined frame. It records accepted anchor/visual
updates, rejected synchronization windows and the first terminal tracking
failure. Accepted visual entries include PnP inlier counts, reprojection RMSE,
and optical-flow track counts; the ledger has its own SHA-256 digest. The
end-to-end runner verifies its length and timestamps against the camera stream
and against the exact visual rows that the native estimators will consume.
This ensures dropped frames and accepted updates are independently inspectable
instead of appearing only as aggregate counts.

The integration test for the wrapper uses **mocked sensor data and an
in-memory fake native process** to verify the evidence contract; the
separate native CTest uses a compiled real C++ executable. These are
not substitute validations for independently obtained original EuRoC
images or benchmark comparisons.

The `active_position_only.csv` and `shadow_position_only.csv` files contain
the same checked nanosecond timestamps and meter-valued positions as the
corresponding full 6-DoF files. Position-only ATE/RPE cannot validate the
quaternion attitude estimates. Visual quaternions use scalar-first
`qw,qx,qy,qz` orientation from the initial IMU-anchored world to the IMU body
(`q_W_I`). Correctness of gravity alignment and uncertainty still requires
a separate physically meaningful experiment.

## Explicit temporal interpolation, not timestamp rounding

The native bridge consumes original IMU timestamps. When a camera-derived visual update falls *between* consecutive original IMU timestamps, it inserts a linearly interpolated accelerometer/gyro sample at the precise camera nanosecond time (and counts these events separately). This is an approximation that must be tested in sensitivity analysis. Unsupported sensor gaps over 25 ms, time inversions, insufficient visual correspondences, large PnP residual, nonfinite values, source schema mismatches and native estimator rejections are fail-closed.

Visual tracking intentionally **stops after the first tracking failure**. By default, the export refuses to create a partial visual trajectory after this failure. An explicit `--allow-partial` diagnostic run may write the accepted prefix, with `partial_failure_export=true`, `failed_closed_on_tracking_loss=true`, and its covered frame count recorded in the manifest. It neither silently reinitializes a new coordinate frame nor injects any publisher ground-truth pose. The exported `world` frame is anchored at the first accepted IMU frame, not the EuRoC ground-truth world origin. Pose alignment for ATE must therefore be *evaluation-only* and performed downstream from genuine ground truth.

## Evidence checklist before publication

- [ ] CI checks and native self-tests pass on GCC/Clang/Windows, with no sanitizer failures.
- [ ] Real EuRoC source archives independently verified (not generated or fabricated).
- [ ] Measure actual accepted, rejected, interpolated and dropped events across prespecified authentic sequences.
- [ ] Validate gravity alignment and attitude observability, propagation noise, clock offsets, PnP degeneracy and long-run tracking resets.
- [ ] Validate the new visual quaternion attitude update on authentic EuRoC sequences; run independent orientation error analysis and IMU-gravity alignment ablations.
- [ ] Compare genuine estimated trajectories against EuRoC ground truth using independent #75 ATE/RPE evaluator; run paired credible SOTA and ablations. Use GT only for post-run evaluation.
- [ ] Distinguish single-UAV MAV benchmark evidence from physical multi-UAV swarm/fault testing.

**Do not call this phase a completed trustworthy VIO benchmark, physical GPS-denied UAV performance experiment or paper-ready scientific contribution.** The CI exercises synthetic source fixtures and synthetic camera geometry only.

## Optional tilt-only stationary gravity initialization (synthetic-tested)

The default remains `--assume-initial-imu-identity`, which is not an
observed initial orientation. For a source sequence with a *genuinely stationary*
200 Hz IMU segment extending from the very first IMU measurement through the
first accepted visual pose, use the alternate prior:

```bash
python3 -m research.experiments.euroc_vio_run \\
  --sequence-root /data/EuRoC/MH_01_easy \\
  --native-binary build/linux-gcc-release/tests/euroc_stereo_imu_native_fusion \\
  --output-dir /data/exports/MH_01_easy_gravity_trial01 \\
  --initial-attitude stationary-gravity
```

This mode computes an accelerometer mean *without ground truth*, enforces
at least 200 ms of preceding samples, verifies continuous IMU timestamps,
angular rate <=0.15 rad/s, acceleration magnitude near 9.81 m/s² and small
sample variability. If any check fails, the native replay **exits nonzero**
before writing trajectories. A quaternion rotates body-measured specific
force to world +Z (the estimator integrates gravity as world -Z). The exact
same rotation is applied to the filter initial attitude and the visual VO
position, velocity and orientation observations. It cannot estimate global
yaw and cannot cure long-term PnP correlations, drift, outliers or
uncalibrated covariance. An IMU segment that is dynamically accelerated but
happens to satisfy these thresholds can still produce an incorrect prior;
therefore this is an experimental diagnostic ablation, **not** proof of
physically correct orientation, stationarity or robust GPS-denied flight.

The runner's manifest records the commanded `initial_attitude_mode`, and
requires the native executable's corresponding initial-attitude attestation.
Both identity and gravity-aligned variants remain unverified software replays
until evaluated using independently obtained genuine EuRoC data.

## Predeclared paired initialization ablation (same sensor stream)

Generate a single immutable VO CSV with `euroc_stereo_visual_odometry.py`
and replay that exact file twice, once with an identity prior and once with
the opt-in stationary-gravity tilt prior. The paired driver enforces input
SHA-256 equality, exact active/shadow timestamps and identical native IMU/
visual counters. **A gravity-mode stationarity rejection fails the paired
study** rather than silently comparing unequal subsets.

```bash
python3 -m research.experiments.euroc_initialization_ablation \\
  --sequence-root /data/EuRoC/MH_01_easy \\
  --visual-csv /data/exports/MH_01_easy_visual.csv \\
  --native-binary build/linux-gcc-release/tests/euroc_stereo_imu_native_fusion \\
  --output-dir /data/exports/MH_01_easy_initialization_ablation
```

`ablation_manifest.json` contains two native trajectory pairs and
paired **differences** in world-anchored position (meters) and quaternion
geodesic orientation (radians), not ATE/RPE or correctness metrics. It has
`ground_truth_used=false` and `publishable_accuracy_evidence=false`.
Raw world-frame position and quaternion differences can primarily represent
arbitrary initial coordinate-frame orientation and translation, rather than
estimator drift. The ablation therefore ALSO reports **first-pose aligned**
paired SE(3) residual differences. The single relative rigid transform is
fixed using only the first pair of estimated poses, without ground truth,
scale fitting or any subsequent trajectory optimization. Synthetic tests
verify that a constant 90-degree yaw + translation disappears after alignment
while subsequent relative motion disagreement remains. Neither raw nor
aligned differences are GT accuracy metrics: publication-grade ATE/RPE needs
the separate independent evaluator in #82 and prespecified time association
in #83 on independently verified original EuRoC sequences. No SOTA validation
is inferred.

## Draft follow-up: experimental 3D PnP scene conditioning (#85)

The 2D image covariance rank, RANSAC consensus and low reprojection error
alone cannot certify that a set of stereo-derived 3D points is well conditioned
for generic EPNP. The follow-up branch checks the absolute scene spread and
normalized smallest/largest centered 3D singular values **both before** and
**after** RANSAC, refuses negative depths and near-coplanar/collinear inliers,
and stores the inlier conditioning ratio in accepted frame audits. The
predeclared 0.005 ratio is a conservative *experimental* rejection threshold,
not an optimized cutoff. A planar-specialized PnP solver is not implemented.
This may reject legitimate data; authentic EuRoC dropout coverage, ablation
and independent accuracy are required before asserting research benefit.

### Audit integrity gate

An accepted `visual_pose` ledger row **must contain** a finite,
unit-interval RANSAC inlier 3D singular-value ratio that exceeds the
predeclared conditioning threshold. A missing, malformed or below-threshold
ratio now terminates tracking fail-closed instead of silently omitting
conditioning evidence. Synthetic regression covers missing, nonfinite,
out-of-range and valid ratios. This confirms the *audit contract*, not
real-world 6-DoF accuracy or the optimality of the experimental threshold.
