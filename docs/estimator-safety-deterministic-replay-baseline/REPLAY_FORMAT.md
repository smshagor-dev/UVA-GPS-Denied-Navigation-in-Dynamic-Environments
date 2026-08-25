# Replay Format

Date: July 17, 2026

## Executable

- `build/validation-msvc/tests/Release/ekf_phase15_replay.exe`

## Output

Default output path:

- `artifacts/phase15/ekf_replay_report.json`

## JSON Fields

- `schema_version`
- `scenario_count`
- `scenarios[]`

Per-scenario fields:

- `scenario_name`
- `success`
- `processed_record_count`
- `accepted_count`
- `rejected_count`
- `final_position`
- `final_velocity`
- `final_orientation`
- `covariance_trace`
- `finite_state`
- `covariance_symmetric`
- `diagnostics`
- `checksum`

## Scenario Deck Implemented

- stationary IMU
- constant yaw rate
- repeated manual ZUPT
- invalid timestamp sequence
- non-finite measurement
- oversized time step
- disabled unsafe LiDAR correction
- long-duration stationary replay

## Determinism Contract

- same build
- same executable
- same scenario deck
- same checksum per scenario across repeated execution inside the replay runner
