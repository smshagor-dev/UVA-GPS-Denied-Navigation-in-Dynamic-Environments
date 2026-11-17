# Native EuRoC IMU-only estimator replay (partial integration)

**Status:** implementation and synthetic in-memory self-test only. No genuine EuRoC sequence was executed in this PR.

This experiment connects the published EuRoC `mav0/imu0/data.csv` column format directly to this repository's C++ **active EKF** (`EKFStateEstimatorAdapter`) and independent **shadow ESKF** (`Phase17StateEstimatorAdapter`). Unlike fixture-only trajectory metrics, these use the real estimator implementations. There is no camera frontend or visual correction in this step.

## Run

Build the project normally with `DRONE_BUILD_TESTS=ON`, then locate `euroc_imu_native_replay` in the configured test binary directory.

```bash
ctest --test-dir build/linux-gcc-release -R euroc_imu_native_replay_selftest --output-on-failure

build/linux-gcc-release/tests/Release/euroc_imu_native_replay \
  --imu-csv /data/EuRoC/MH_01_easy/mav0/imu0/data.csv \
  --output /data/exports/MH_01_easy_imu_only_baseline.csv \
  --assume-imu-frame-identity
```

CMake generator/preset-specific executable paths may differ.

- Strict monotonic integer nanosecond timestamps, finite gyro/accelerometer values, expected EuRoC column order.
- Converts timestamps to seconds **relative to the first sample** to avoid loss of timing precision in epoch-nanosecond conversion.
- Uses the repository's native `MeasurementEnvelope` API and records both active and shadow estimated positions for every accepted IMU sample.
- Fails on estimator rejection; no silent missing rows; output file must not preexist.
- Initializes the estimators with **zero pose, identity attitude and zero velocity**, not ground truth.
- Does not read any file under `state_groundtruth_estimate0` or inject GT into the estimator.
- The `--assume-imu-frame-identity` flag is an explicit experimental simplifying assumption, **not** a verified sensor-to-body extrinsics calibration.
- Unit tests use synthetic IMU data. Real sensor streams require source-archive SHA-256, calibration inspection, manifest and raw failure accounting.

## Important limitations

**Do not claim this is visual-inertial odometry (VIO).** The output is uncalibrated, IMU-only identity-initialized dead reckoning. Drift and gravity-frame initialization errors make its positions unsuitable for performance or safety claims. It is not a valid camera/IMU benchmark, not a matched published method, and not evidence of multi-UAV operation.

Output columns have explicit `active_` and `shadow_` labels, rather than the `timestamp_ns,x_m,y_m,z_m` schema consumed by the independent EuRoC trajectory evaluator, to avoid accidental promotion of these baseline estimates into a validated real-data result.

## Next genuine evidence gate

1. Confirm publisher archive provenance and sequence checksums.
2. Parse/verify EuRoC `sensor.yaml` calibration, IMU-to-body and camera-to-IMU extrinsics and gravity/initial attitude conventions.
3. Decode camera PNGs and synchronize with native IMU playback; use the project's real visual frontend/feature tracker rather than synthetic features.
4. Produce separately timestamped active/shadow pose streams with documented IMU-body origin and no GT leakage.
5. Run multiple prespecified sequences, published-baseline comparisons, tracking-loss statistics and the `euroc_trajectory_evaluation.py` evaluation pipeline.

Until then, the only verified property of this PR is the reproducible **software replay interface and its synthetic self-test**, not accuracy.
