# Native Replay

## Status

Implemented in Phase 16B and re-validated during Phase 16D on the estimator-only validation path:

`EstimatorMeasurement -> MinimalEskfAdapter -> optional EstimatorCoordinator shadow path -> EKFEstimator`

## Supported modes

- `active_only`
- `active_with_identical_shadow`

The identical-shadow mode runs a separate minimal ESKF instance and never promotes shadow output to the authoritative pose.

## Input schema

Replay input currently requires:

- `schema_version = 1`
- `coordinate_frame`
- optional `initial_state`
- ordered `records`

Supported record types:

- `imu`
- `visual_pose`
- `depth`
- `altitude`
- `zupt`
- `tdoa_candidate`

`ground_truth` may appear as metadata, but it is not fused.
Visual orientation may be present on `visual_pose`, is transported through the envelope, and remains unfused by `MinimalEskfAdapter`.

## Safety limits

Replay input is treated as untrusted and validated for:

- maximum file size
- maximum record count
- maximum string length
- supported schema version
- finite numeric values
- strictly monotonic timestamps
- supported coordinate frame
- positive covariance or sigma values
- normalizable quaternion input
- bounded vector dimensions
- non-zero exit status on failure

## Output report

`estimator_replay_runner` emits machine-readable JSON including:

- input and report schema versions
- replay mode
- estimator names
- processed, accepted, rejected, invalid and unsupported counts
- final state and uncertainty
- active and shadow health
- active versus shadow deltas
- queue and synchronization telemetry
- deterministic result hash
- average and maximum propagation/update latencies
- success flag and failure reason

## Deterministic hash

The replay hash is derived from a canonical text payload built from meaningful report fields only.
It does not include raw memory, struct padding, or transient timing-only values.
Floating-point values are normalized with fixed decimal formatting to 12 digits after the decimal point.

Determinism is only claimed for the same input, build, executable, platform, and relevant runtime configuration.

Across local MSVC, GCC, and Clang validation, the final replay state matched on the Phase 15 stationary dataset while deterministic hashes remained mode-specific artifacts rather than a cross-compiler equality guarantee.

## Example commands

```powershell
.\build\validation-msvc\tools\Release\estimator_replay_runner.exe `
  --input datasets\estimator\phase15_stationary_replay.json `
  --output artifacts\phase16_replay_active.json `
  --mode active_only
```

```bash
./build/validation-linux-gcc/tools/Release/estimator_replay_runner \
  --input datasets/estimator/phase15_stationary_replay.json \
  --output artifacts/phase16_replay_shadow.json \
  --mode active_with_identical_shadow
```

## Current validation note

The replay path was re-executed during Phase 16D under the isolated local ThreadSanitizer lane using the same stationary replay fixture.

## Limitations

- Native replay currently covers only the safe measurement subset listed above.
- Unsupported experimental estimator features remain fail-closed.
- Software replay evidence is not flight evidence.
