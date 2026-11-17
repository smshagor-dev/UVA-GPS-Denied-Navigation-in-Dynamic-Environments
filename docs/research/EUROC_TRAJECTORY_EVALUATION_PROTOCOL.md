# EuRoC ground-truth trajectory evaluation: evidence gate

**Status: metric evaluator implemented; real EuRoC/estimator study NOT yet executed.**

This protocol introduces a verifiable measurement stage for the GPS-denied navigation
research program without pretending that synthetic C++ replay is real sensor evidence.
The `unittest` fixtures use invented trajectories; they are not EuRoC results.

## Independent dataset

Official EuRoC MAV dataset and bibliographic details:
https://projects.asl.ethz.ch/datasets/euroc-mav/

EuRoC provides stereo camera sequences, IMU measurements and separately
recorded ground-truth trajectories. Each sequence normally includes
`mav0/state_groundtruth_estimate0/data.csv`, whose first eight columns are
nanosecond timestamps, position in meters and quaternion `w,x,y,z`.

The dataset is **single-UAV visual-inertial research data**, not a multi-UAV
swarm flight or RF/FANET validation record. Record the exact ETH archive
version, sequence ID, acquisition URL, acquisition date, terms of use and
original archive SHA-256 before claiming an independent real-data evaluation.

## Required genuine experimental inputs

1. Original, unmodified EuRoC `mav0/state_groundtruth_estimate0/data.csv`.
2. Real execution of this repository's actual estimator on the corresponding
   sensor sequence, producing a strictly time-ordered estimator export:
   `timestamp_ns,x_m,y_m,z_m`.
3. A documented, source-bound IMU/body frame convention, camera–IMU extrinsics
   and estimator timestamp source. **Do not assert** `--frame-agreement same-imu-body`
   if estimator positions refer to a different frame or origin.
4. Exact estimator executable, commit SHA, compiler/toolchain, calibration,
   configuration, initialization and any actual ground-truth access during
   estimator initialization (otherwise leakage may invalidate comparison).
5. Archive all logs and independent input hashes; preserve failures, tracking
   loss and excluded timestamps. Do not silently curate away failures.

**Important unresolved work:** the repository does **not yet expose a validated
EuRoC camera+IMU replay-to-estimator trajectory exporter**. This PR adds only
the independent evaluator and tests, not a successful end-to-end real-data
study. A future exporter must be integrated with the actual active and shadow
estimators and their frame/time interfaces.

## Evaluate actual outputs

From the repository root, after independently acquiring genuine data:

```bash
python3 -m pip install numpy
python3 -m unittest tests.test_euroc_trajectory_evaluation -v

python3 -m research.experiments.euroc_trajectory_evaluation \
  --ground-truth /data/EuRoC/MH_01_easy/mav0/state_groundtruth_estimate0/data.csv \
  --estimated /data/actual_estimator_run/estimated_pose_imu.csv \
  --sequence MH_01_easy \
  --frame-agreement same-imu-body \
  --tolerance-ms 20 \
  --min-pairs 10 \
  --min-overlap-s 5 \
  --relative-lag-s 1 \
  --output /data/results/MH_01_easy_trajectory_metrics.json
```

**No bundled input file or numeric outcome is provided here.** Dataset paths
above are examples only and must be replaced with actual files.

## Exact metrics and caveats

- One-to-one nearest-neighbor time association with strict nanosecond
  timestamps; default +/- 20 ms tolerance; ground-truth points never reused.
- A single global **rigid SE(3) position alignment**, with no scale fit.
  This is offline, full-trajectory alignment (not online accuracy). The
  alignment rejects degenerate collinear position trajectories.
- 3-D **translation ATE** RMSE, mean, median, 95th percentile and maximum,
  in meters.
- Fixed-lag **aligned world-frame displacement error** (default 1 s);
  it is **not** full SE(3) relative pose error and must not be labeled
  `evo_rpe` without a separate orientation-aware computation.
- Output records associated sample count, coverage, overlap duration,
  declared timestamp offset, rigid alignment and both raw file SHA-256s.
- Covariance calibration/NEES, orientation error, failure rates, cross-sequence
  confidence intervals, real-time runtime and published-baseline comparison
  are **not** calculated by this evaluator.
- The metric JSON sets `provenance_verified=false`; a source SHA-256 proves
  input immutability *after* capture, not independent source authenticity.

## Research acceptance criteria

- Validate on multiple prespecified EuRoC sequences, not one chosen example.
- Ensure the active and shadow estimator outputs are genuinely produced from
  the same original measurements, with identical initialization and no
  ground-truth leakage.
- Provide matched OpenVINS or another traceable published VIO baseline where
  the output coordinate frame/calibration/evaluation protocol is equivalent.
- Keep failure counts, time-overlap fractions, deterministic replay records,
  model configurations and raw per-sequence outputs.
- Only after executing on authentic sensor data may the manuscript report
  *real recorded sensor dataset* accuracy. It still cannot claim actual
  multi-UAV flight or safety.

The original approved Scientific Reports firmware-policy work is a separate
research topic and not evidence of EuRoC localization accuracy.
