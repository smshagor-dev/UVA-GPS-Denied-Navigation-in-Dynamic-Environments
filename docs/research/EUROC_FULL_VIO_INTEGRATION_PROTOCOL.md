# EuRoC Phase 81: stereo-derived visual odometry + native inertial fusion

**Status:** implementation and synthetic CI validation; no authentic EuRoC performance established.

## Architecture

1. **Original sensor intake** (#76) validates raw imu0, cam0, cam1 metadata/images/CSV. Original input files are never modified and publisher ground-truth files are not read.
2. **Calibration gate** (#78) validates `T_BS` sensor-to-body transforms, right-handed rotations, camera intrinsics, radial-tangential distortion and noise parameters.
3. **Exact stereo/IMU timestamp synchronization** (#80) validates camera pairing, sensor gaps, missing observations and source hashes; reject records remain reported.
4. **Calibrated stereo frontend** (#79) detects and ratio-matches ORB landmarks in undistorted/rectified stereo pairs and triangulates positive-depth metric 3D points.
5. **New temporal VO** (`euroc_stereo_visual_odometry.py`) uses rectified left-camera optical flow and RANSAC PnP against the **previous frame's stereo-derived 3D points**. Its `T_Ccurrent_Cprevious` measurements are integrated into an IMU-anchored world coordinate frame using calibrated `T_I_Crect`. Source images and calibration are hashed; no ground truth is involved.
6. **New native EKF/ESKF replay** (`euroc_stereo_imu_native_fusion.cpp`) reads genuine imu0/data.csv and the VO-generated visual.csv, enforces chronological and positive-quality input, and invokes the repository's existing `EKFStateEstimatorAdapter` and `Phase17StateEstimatorAdapter`. Visual measurements update **position and velocity only**. Both native filter outputs are separate CSV trajectories with `timestamp_ns,x_m,y_m,z_m`. If any active/shadow event is rejected, replay fails rather than silently producing a benchmark row.

**Important method limitation:** The visual frontend estimates orientation changes through PnP for camera-frame geometry, but `VisualPoseMeasurementPayload` does not fuse attitude. The EKF's initial attitude is **identity, not a gravity-aligned estimated state**, and extrinsic rotation is not dynamically updated. The resulting estimated trajectories are *experimental software outputs*, not validated robust 6-DoF VIO. There is no absolute heading observability or learned IMU bias calibration. These limitations must be resolved or explicitly treated in benchmark interpretation.

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

Adjust the build output executable path for the selected CMake multi-config preset and system. All outputs must be new paths outside the original dataset. The Python visual CSV contains fixed non-benchmark uncertainty weights `sigma_position_m=0.5` and `sigma_velocity_mps=0.75`, PnP inlier count and reprojection RMSE. It is NOT legitimate to reinterpret these constants as measured uncertainties.

## Explicit temporal interpolation, not timestamp rounding

The native bridge consumes original IMU timestamps. When a camera-derived visual update falls *between* consecutive original IMU timestamps, it inserts a linearly interpolated accelerometer/gyro sample at the precise camera nanosecond time (and counts these events separately). This is an approximation that must be tested in sensitivity analysis. Unsupported sensor gaps over 25 ms, time inversions, insufficient visual correspondences, large PnP residual, nonfinite values, source schema mismatches and native estimator rejections are fail-closed.

Visual tracking intentionally **stops after the first tracking failure**. By default, the export refuses to create a partial visual trajectory after this failure. An explicit `--allow-partial` diagnostic run may write the accepted prefix, with `partial_failure_export=true`, `failed_closed_on_tracking_loss=true`, and its covered frame count recorded in the manifest. It neither silently reinitializes a new coordinate frame nor injects any publisher ground-truth pose. The exported `world` frame is anchored at the first accepted IMU frame, not the EuRoC ground-truth world origin. Pose alignment for ATE must therefore be *evaluation-only* and performed downstream from genuine ground truth.

## Evidence checklist before publication

- [ ] CI checks and native self-tests pass on GCC/Clang/Windows, with no sanitizer failures.
- [ ] Real EuRoC source archives independently verified (not generated or fabricated).
- [ ] Measure actual accepted, rejected, interpolated and dropped events across prespecified authentic sequences.
- [ ] Validate gravity alignment and attitude observability, propagation noise, clock offsets, PnP degeneracy and long-run tracking resets.
- [ ] Check full 6-DoF orientation integration or mark positional-only visual fusion as a restricted baseline.
- [ ] Compare genuine estimated trajectories against EuRoC ground truth using independent #75 ATE/RPE evaluator; run paired credible SOTA and ablations. Use GT only for post-run evaluation.
- [ ] Distinguish single-UAV MAV benchmark evidence from physical multi-UAV swarm/fault testing.

**Do not call this phase a completed trustworthy VIO benchmark, physical GPS-denied UAV performance experiment or paper-ready scientific contribution.** The CI exercises synthetic source fixtures and synthetic camera geometry only.
