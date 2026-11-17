# Calibrated stereo landmark frontend: EuRoC measurement integration stage

**Status:** calibrated stereo image reader, feature matcher and one-frame metric triangulation implemented. Synthetic tests only; no authentic EuRoC images were evaluated in this PR.

## Scientific scope

This standalone frontend extends the independent calibration-validation foundation in #78. It accepts an unpacked EuRoC MAV sequence containing **original** `mav0/imu0/sensor.yaml`, `mav0/cam0/sensor.yaml`, `mav0/cam1/sensor.yaml`, camera CSV indexes and stereo PNG images. The required pair must have the same exact nanosecond timestamp in both unmodified camera indexes. The source camera files must be present and remain inside the original directory.

It audits EuRoC `T_BS` right-handed homogeneous transforms and camera pinhole/radial-tangential calibration through `euroc_calibration_validation.audit`. The supplied `T_cam1_cam0` is the transformation from C0 into C1. OpenCV `stereoRectify` and `initUndistortRectifyMap` use that transform and the original intrinsics/distortion for calibrated stereo rectification.

A local ORB detector, binary matcher, Lowe-style ratio test, vertical epipolar constraint and positive metric-depth test triangulate source-image matches. Each triangulated **rectified C0** point is mapped back into the original camera C0 frame, then into the original IMU frame by the audited `T_imu_cam0` transform. Accepted points have finite 3D coordinates and a configurable depth window.

**This is one-frame stereo geometry and not odometry.** There is no temporal optical flow, world frame, recovered camera pose, inertial fusion, state estimation, tracking-loss result, loop closure, published VIO baseline or multi-UAV measurement. Even a geometrically valid 3D match can be a visual outlier. Do not use its point count or depth distribution as a benchmark accuracy result.

## Requirements

```bash
python3 -m pip install "numpy>=2.3,<3" "PyYAML>=6.0.3,<7" "opencv-python-headless>=4.10,<6"
python3 -m unittest tests.test_euroc_calibration_validation -v
python3 -m unittest tests.test_euroc_stereo_frontend -v
```

Obtain the genuine sequence directly from ETH/ASL, preserve its metadata and verify its archive hash independently. No data archive is bundled or downloaded by this experiment.

## Run on your verified sequence

```bash
python3 -m research.experiments.euroc_stereo_frontend \
  --sequence-root /data/EuRoC/MH_01_easy \
  --timestamp-ns 1403636579763555584 \
  --output /data/exports/MH_01_easy_stereo_landmarks.json \
  --max-features 2200 \
  --include-landmarks
```

Replace the example timestamp with a timestamp actually present in **both** original `mav0/cam0/data.csv` and `mav0/cam1/data.csv`. The tool fails on absent/unreadable images, invalid or mismatched camera dimensions, unsupported calibration, unsafe camera filenames, insufficient texture, too few reliable matches, non-finite geometry or an output path inside the original dataset.

The output includes per-image SHA-256, per-calibration YAML SHA-256, match counts, positive metric depth distribution, optional rectified image feature coordinates and IMU-frame 3D landmarks. `world_pose_estimated=false`, `ground_truth_used=false`, `dataset_provenance_verified=false` are explicit limitations, not scores. Outputs never overwrite existing files.

## Integration dependency and next milestone

This branch is based on **#78 calibration frame audit**; its PR is intentionally stacked on that branch. After #78 merges to `main`, rebase this change and retarget to `main` before integration.

The source-index intake in #76 and the native active EKF / shadow ESKF IMU-only runner in #77 remain separate. Next, link their trusted timestamped event streams with stereo landmarks, add **multi-frame tracking and camera/IMU synchronized measurement envelopes**, and export poses from real accepted estimator events with explicit initialization/extrinsics. Only then benchmark against independent ground truth across prespecified original EuRoC sequences.

**Never** label synthetic image regression results as real EuRoC evidence, or single-UAV EuRoC evidence as physical multi-UAV swarm validation.
