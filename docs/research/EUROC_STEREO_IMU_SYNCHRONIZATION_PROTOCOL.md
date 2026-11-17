# EuRoC stereo + IMU synchronization: timestamp-preserving replay contract

**Implementation:** raw sensor synchronization and explicit missing-data accounting. **Not** a VIO estimator, sensor-ground-truth comparison, or flight test.

This research stage connects the strict original camera/IMU ingestion in #76 and the calibrated geometry audit in #78. It requires a real unpacked EuRoC sequence with `mav0/{imu0,cam0,cam1}` `data.csv`, camera PNG images, and original `sensor.yaml` calibration files. It invokes both validators and does not read any ground-truth CSV or estimate pose.

## Reproduce

```bash
python3 -m pip install "numpy>=2.3,<3" "PyYAML>=6.0.3,<7"
python3 -m unittest tests.test_euroc_stereo_imu_synchronization -v

python3 -m research.experiments.euroc_stereo_imu_synchronization \
  --sequence-root /data/EuRoC/MH_01_easy \
  --events /data/exports/MH_01_easy_sync.jsonl \
  --manifest /data/exports/MH_01_easy_sync_manifest.json \
  --max-bracket-ms 25 \
  --max-imu-gap-ms 25 \
  --hash-images
```

The example sequence directory is illustrative. Authentic EuRoC archives must be acquired independently from ETH/ASL and their publisher archive provenance, license and SHA-256 checked independently. Neither the code nor its CI downloads a sequence.

## Exact temporal association

1. Every cam0 and cam1 timestamp is preserved, and the union of timestamp IDs is examined in increasing order. A frame is paired **only** if both cameras have exactly the same original nanosecond timestamp; no nearest-neighbor cam0/cam1 substitution.
2. Each timestamp must have original IMU readings on or before, and on or after, the image acquisition time. Missing or too-distant IMU samples are rejected. The maximum sample gap crossing the image timestamp must also pass a separate check.
3. The first fully supported stereo frame is an `anchor` only. It has no synthetic IMU propagation or estimated pose.
4. A subsequent valid frame is a `ready` replay window with every IMU measurement in **(last accepted stereo timestamp, current stereo timestamp]**, in its original input order, without duplicate interval use or interpolated samples.
5. Rejected frames remain in the JSONL with an explicit status and empty IMU segment. The next valid window starts from the last accepted timestamp, preserving intervening real IMU measurements rather than skipping them.
6. Native IMU measurements retain original gyroscope `rad/s` and accelerometer `m/s²` readings. No body-frame rotation or image undistortion is performed by this synchronization layer.

Reported statuses include `anchor`, `ready`, `missing_cam0`, `missing_cam1`, `no_imu_before`, `no_imu_after`, `imu_bracket_gap`, `no_imu_between_frames` and `imu_interframe_gap`. All status counts, raw source input hashes, calibration hashes, optional per-image hashes and the JSONL output SHA-256 are emitted to an adjacent manifest.

**Fail-closed boundary:** `ready` means temporally associated and gap-checked, *not* a successful visual-inertial pose. Even geometrically matching time streams can have invalid clock offsets or physically inconsistent sensor calibration. User-selectable tolerance must be prespecified and justified. The input root is read-only and neither output path may be inside the original dataset or overwrite an existing file.

## What remains before a research-paper localization result

- Connect these ordered IMU+stereo windows to the actual C++ `VIOPipeline` / `EstimatorCoordinator` with authenticated camera decoding and the #79 calibrated stereo measurements, calibrated extrinsic transformations and explicit timestamp origin.
- Preserve every estimator rejection, frontend tracking failure, initialization/reset, and camera/IMU gap in logs. Prohibit ground-truth injection.
- Export actual active/shadow estimator trajectories with one documented IMU-body origin and timestamps.
- Compare on multiple prespecified genuine EuRoC sequences against matched traceable VIO baselines using #75's fail-closed independent trajectory evaluator and report uncertainty/failure counts.

Passing the included offline regression tests demonstrates only correctness on invented sensor fixture records. It is **not** genuine EuRoC accuracy, physical GPS-denied UAV performance, or multi-UAV collision safety.
