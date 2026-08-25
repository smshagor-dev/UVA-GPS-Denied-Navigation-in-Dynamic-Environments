# Phase 7 Research Reproducibility

Date: July 16, 2026

## Environment Requirements

- Windows 11 host used for the validated run
- Python 3.14.5
- Go toolchain available in PATH
- Existing native validation build at `build/validation-msvc`
- Python dependencies required by the repository, including `jsonschema`

## Repository Structure Added For Reproducibility

- `datasets/`
- `experiments/`
- `results/`
- `experiments/phase7_metadata_schema.json`

## Reproducible Commands

### Mission Validation

```powershell
py -3.14 scripts\phase7_mission_scenarios.py
```

Expected outputs:

- `docs/research-validation-software-hil-scenarios/mission_results.json`
- `experiments/phase7-*.json`
- `results/phase7/phase7_backend.log`
- `results/phase7/phase7_mission_scenarios.log`

### Configuration Validation

```powershell
python scripts\validate_config_schemas.py
```

Expected output:

- `PASS all tracked configuration files validated against Phase 5 schemas`

### Python Validation

```powershell
python -m unittest tests.test_dashboard_backend_status
```

Expected output:

- `Ran 12 tests`
- `OK`

### Native Validation

```powershell
ctest --test-dir build\validation-msvc -C Release --output-on-failure
```

Expected output:

- `100% tests passed out of 114`

### Go Validation

```powershell
go test ./...
```

Expected output:

- `ok   drone_swarm/controlplane/cmd/control-plane`
- `ok   drone_swarm/controlplane/internal/controlplane`

### Performance Regression Checks

```powershell
py -3.14 scripts\phase6_performance_benchmark.py
py -3.14 scripts\phase6_stress_test.py
```

Expected outputs:

- `docs/performance-engineering-stability-validation/benchmark_results.json`
- `docs/performance-engineering-stability-validation/stress_results.json`

## Configuration Snapshot Provenance

The validated mission run recorded SHA-256 hashes for:

- `config/runtime.json`
- `config/anchors.json`
- `config/lidar.json`
- `config/swarm_edge_protocol.json`

## Dataset Requirements

No repository-managed hardware dataset was required for the July 16, 2026 run.

- Synthetic telemetry was used for scenario execution.
- `datasets/` remains reserved for future replay or HIL datasets.

## Metadata Validation

Experiment metadata schema validation passed for `experiments/phase7-20260716T191134Z.json` on July 16, 2026.

## Reproducibility Limitations

- Results depend on the local workstation load.
- The July 16 benchmark rerun overlapped an active long-duration Phase 6 soak workload, so microbenchmark deltas should be interpreted carefully.
- No hardware replay dataset or external simulator package is bundled in this repository snapshot.

