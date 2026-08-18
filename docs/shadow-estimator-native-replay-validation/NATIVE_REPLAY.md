# Native Replay

Date: July 17, 2026
Status: Implemented

## Implemented Path

Phase 16 adds a native replay executable at `tests/ekf_phase16_replay.cpp`.

The executable exercises:

- `EstimatorCoordinator`
- the active `EKFStateEstimatorAdapter`
- the bounded shadow queue and worker
- active-versus-shadow comparison publication
- deterministic artifact generation in `artifacts/phase16/ekf_phase16_replay_report.json`

## Current Scenarios

- stationary IMU
- constant yaw
- repeated manual ZUPT
- invalid timestamp sequence
- disabled LiDAR correction
- bounded long-duration shadow replay with 4000 IMU samples, periodic manual ZUPT, one rejected NaN IMU input, and one disabled depth event

## Current Guarantees

- active-only and shadow-enabled active outputs are compared per scenario
- replay waits for explicit shadow completion with `flush_shadow()`
- queue capacity, peak depth, drop count, stale count, final snapshots, and checksums are recorded
- disagreement is reported as state delta, not ground-truth error
- same-build replay determinism is checked by rerunning each scenario and comparing final checksums

## Limits

- this is software replay only
- it is not HIL, flight, or production evidence
- dedicated overload and lifecycle coverage lives in `tests/test_phase16_shadow.cpp`, not in the replay executable itself
