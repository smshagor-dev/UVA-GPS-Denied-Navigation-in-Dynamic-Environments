# EuRoC Fixed Evaluation Timestamp-Association Sensitivity — Phase 83

**Evidence scope:** software implementation, synthetic fixtures only, no original
EuRoC accuracy result. This is a stacked change on Phase 82; Phase 81 contains
the still-experimental producer of the native full-pose trajectories.

## Motivation and scientific boundary

Estimated-to-publisher-GT timestamp association offsets can change apparent
trajectory accuracy. **This does not perturb camera-to-IMU sensor timestamps,
rerun the native estimator, or measure physical sensor-clock calibration.** Reporting only the best-fitting offset
from ground truth would leak test information into selection and bias the
comparison. This tool requires the analyst to specify an **ordered evaluation timestamp-offset
grid before looking at the independent publisher ground truth**, then evaluates
every point and records failed points without filtering them. Grid predeclared
means predeclared for this run, not independently preregistered with a third
party. No offset is selected or optimized.

It compares native active EKF and shadow ESKF on *identical original estimated
timestamps*; any timestamp mismatch fails. Each point reports full-window
rigid/no-scale aligned ATE and absolute quaternion attitude error, plus
1-second relative-pose translation (first-body frame) and orientation error
from the independent 6-DoF evaluator. Paired active-minus-shadow differences
are descriptive only, NOT confidence intervals, a superiority test or proof
of trustworthy flight.

All offsets use the same declared minimum number of matched samples, minimum
overlap, relative lag and temporal tolerance. Actual matched counts can differ
between offsets, so downstream comparisons must inspect coverage and failure
status. **An incomplete offset grid exits nonzero and still writes an auditable
failure artifact.**

## Reproducible invocation (after original EuRoC playback)

Run Phase 81 native replay first on original camera/IMU data. The independent
Phase 82 evaluator must be available. Choose the evaluation-association offset grid before
opening the original publisher ground-truth file.

~~~bash
python3 -m pip install "numpy>=2.3,<3"

python3 -m research.experiments.euroc_pose_sensitivity \
  --sequence-root /data/EuRoC/MH_01_easy \
  --sequence MH_01_easy \
  --active-estimated /data/exports/MH_01_easy_active_estimated.csv \
  --shadow-estimated /data/exports/MH_01_easy_shadow_estimated.csv \
  --frame-agreement same-imu-body \
  --offsets-ms=-20,-10,0,10,20 \
  --tolerance-ms 25 \
  --min-pairs 10 \
  --min-overlap-s 5 \
  --relative-lag-s 1 \
  --output /data/exports/MH_01_easy_clock_offset_sensitivity.json
~~~

The original sequence is read-only; the output must live outside that source,
and existing evidence artifacts cannot be overwritten. Source digest fields
identify the exact publisher GT input and both externally estimated streams,
but do not verify publisher archive authenticity. Report the exact offsets,
matching tolerances, raw per-offset metrics, dropped/failed entries, the
identity/attitude limitations from Phase 81, and source provenance.

## Still needed for scientifically publishable evidence

- [ ] Independently verify an original publisher EuRoC archive and source SHA.
- [ ] Run original sensor streams with complete accepted/rejected frame audit.
- [ ] Compare both native estimators at all prespecified offsets on multiple
      sequences with original publisher ground truth held out.
- [ ] Investigate gravity alignment, unobservable yaw, PnP degeneracy,
      uncertainty consistency, camera/IMU interpolation and tracking resets.
- [ ] Perform proper baseline replication and sensitivity/ablation analysis.
- [ ] Keep single-MAV benchmark distinct from multi-UAV flight experiments.

This source and CI do not constitute a real EuRoC experiment, new localization
accuracy, hardware evidence, or a validated swarm safety guarantee.
