# EuRoC stereo/IMU intake protocol (sensor-only; no estimator evidence)

**Status:** implementation of a fail-closed ingest/manifest stage; no authentic EuRoC benchmark has been executed in this PR.

## Why this exists

The repository has an EKF / shadow-estimator interface but lacks a validated, frame-aware EuRoC sensor replay adapter and working camera calibration bridge. This tool validates the raw stereo/IMU input and writes a deterministic, timestamp-ordered JSONL event contract for future integration. It never reads ground-truth trajectories or creates estimated positions.

## Source and layout

Acquire a genuine EuRoC MAV sequence independently from https://projects.asl.ethz.ch/datasets/euroc-mav/ and document its original archive URL, publisher metadata, license and SHA-256. Supply the unpacked directory that contains `mav0/` with:

- `imu0/data.csv`, `imu0/sensor.yaml`
- `cam0/data.csv`, `cam0/sensor.yaml`, `cam0/data/*.png`
- `cam1/data.csv`, `cam1/sensor.yaml`, `cam1/data/*.png`

Each CSV must retain the original order, nanosecond timestamps and sensor field ordering. Camera filenames must be basenames; symlinks and paths outside the sequence root are rejected. The tool requires all referenced frames to exist but does not decode PNGs. YAML is hashed without interpreting calibration. There is no hidden use of `state_groundtruth_estimate0`.

## Invocation

```bash
python3 -m unittest tests.test_euroc_sensor_intake -v
python3 -m research.experiments.euroc_sensor_intake \
  --sequence-root /data/EuRoC/MH_01_easy \
  --events /data/exports/MH_01_easy_events.jsonl \
  --manifest /data/exports/MH_01_easy_intake.json \
  --hash-images
```

The output destination must be outside the original sequence root and must not preexist. Without `--hash-images`, the input CSV and calibration YAML files are SHA-256 hashed, and camera image paths are existence-checked but **not** byte-hashed. With the flag each individual PNG is also SHA-256 hashed. Neither mode proves source authenticity; verify archive integrity independently.

## Event contract

- Every line is strict JSON containing `timestamp_ns` (integer) and `sensor`.
- IMU records contain `gyro_rad_s` and `accel_m_s2`, each three finite sensor-frame values.
- Camera records contain `image_relative_path` and, when enabled, `image_sha256`.
- Events are sorted by nanosecond timestamp; tie order is IMU, camera 0, camera 1.
- Dataset ground truth is **not** read. The output is **not** `timestamp_ns,x_m,y_m,z_m`; do not pass it to the trajectory evaluator.
- Manifest records original sensor CSV / YAML hashes, camera exact-timestamp pair count, unpaired stereo counts, event count and output JSONL hash.

## Required next integration

1. Establish EuRoC sensor-to-IMU frame transforms from `sensor.yaml`; document calibrated camera intrinsics and camera/IMU extrinsics.
2. Add a concrete C++ replay bridge into `VIOPipeline` or the `EstimatorCoordinator`, using the original IMU and decoded camera samples, with explicit timestamp conversion and bounded queue/backpressure. Avoid treating camera filenames or synthetic optical flow as visual observations.
3. Capture actual `PoseEstimate` / `EstimatorStateSnapshot` output from the active and separately enabled shadow estimators. Export pose at accepted event stamps in the documented IMU-body origin.
4. Preserve per-event rejects, resets, tracking failures and initialization status. No ground-truth injection to the estimator. Benchmark multiple prespecified sequences and compare against matched published VIO methods.
5. Only after those conditions use `euroc_trajectory_evaluation.py` to produce authentic-sensor translation ATE with the intended measurement boundaries. This remains single-UAV recorded-sensor validation, **not** physical multi-UAV swarm evidence.

CI uses synthetic fake CSV and image bytes exclusively. It does not download, modify, or evaluate EuRoC records.
