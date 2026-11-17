# EuRoC calibration and coordinate-frame audit (Phase: independent geometry gate)

**Status:** strict validation tool and synthetic tests implemented; actual EuRoC calibration files have NOT been examined here.

## Goal and frame definitions

A genuine EuRoC `mav0/<sensor>/sensor.yaml` contains `T_BS`, the homogeneous 4x4 transform taking a vector expressed in **sensor S** coordinates to the **body B** frame. The transform is validated for finite elements, a canonical bottom row, right-handed rotation and rotation orthonormality.

For the IMU sensor I and cameras C0/C1, this tool constructs:

- `T_body_imu = T_B_I` from `imu0/sensor.yaml`
- `T_imu_cam0 = inverse(T_B_I) × T_B_C0` and likewise `T_imu_cam1`
- `T_cam1_cam0 = inverse(T_B_C1) × T_B_C0`
- The Euclidean stereo baseline in meters from both camera origins expressed in body B

It validates camera pinhole intrinsics, resolution, radial-tangential coefficients and the sensor sample-rate fields, and reads positive IMU noise density/random-walk values. The tool **rejects** unsupported distortion models instead of converting them implicitly.

The cross-camera transform is **cam0 coordinates into cam1**, not the reverse. Do not interchange camera-frame and body/IMU-frame values in subsequent estimator code.

## Source inputs

Acquire genuine sensor metadata from the [ETH/ASL EuRoC MAV source](https://projects.asl.ethz.ch/datasets/euroc-mav/). Verify original archive and file SHA-256 against publisher material separately.

Pass the unpacked EuRoC sequence root containing:

```
mav0/imu0/sensor.yaml
mav0/cam0/sensor.yaml
mav0/cam1/sensor.yaml
```

No ground-truth CSV is opened. No trajectory accuracy is computed.

## Reproduce

```bash
python3 -m pip install "numpy>=2.3,<3" "PyYAML>=6.0.3,<7"
python3 -m unittest tests.test_euroc_calibration_validation -v
python3 -m research.experiments.euroc_calibration_validation \
  --sequence-root /data/EuRoC/MH_01_easy \
  --output /data/exports/MH_01_easy_calibration_audit.json
```

Outputs go outside the source tree, cannot overwrite existing files or symlinks and include each original `sensor.yaml` SHA-256 and dependency versions for reproducibility. YAML uses `safe_load` and a 1 MB file cap.

## Evidence boundaries

Passing the audit proves only that the supplied calibration files are internally well-formed according to the checks. It does **not** prove publisher provenance, calibrated sensors, correct runtime camera images, correct alignment between dataset timestamps, or accurate visual-inertial navigation.

The native IMU-only runner in #77 does not incorporate these transforms or camera images yet. The stereo/IMU ingestion in #76 currently hashes calibration files only. Until those two paths are connected, none of these modules justify an authentic EuRoC VIO accuracy claim.

## Next required integration

1. Consume these validated `T_imu_camN` and camera intrinsics in the native camera frontend; apply radial-tangential undistortion and verify image decoding.
2. Match stereo pairs and timestamp them alongside IMU measurements, retaining gaps and dropped events.
3. Replay to active EKF / shadow ESKF with explicit coordinate conversions, no injected ground truth and an independently captured trajectory exporter.
4. Evaluate several prespecified authentic EuRoC sequences, comparisons to published baselines, failure/missing-data rates and any offline alignment caveats.
