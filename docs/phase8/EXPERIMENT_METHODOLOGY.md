# Phase 8 Experiment Methodology

Date: July 16, 2026

## Experiment Protocol

1. define scenario and objective
2. run deterministic software scenario driver
3. capture machine-readable output
4. validate supporting tests
5. compare against prior-phase evidence where relevant
6. document limitations and scope

## Variables

- runtime mode
- scenario type
- injected fault type
- detection time
- recovery time
- latency
- throughput
- safety state
- localization confidence

## Datasets

Current repository state:

- no bundled benchmark or replay dataset payload
- synthetic, deterministic software data is used for Phase 8 execution
- dataset expectations are standardized in `docs/phase8/DATASET_STANDARD.md`

## Evaluation Criteria

- scenario completes without harness failure
- expected safety-state transition occurs when applicable
- recovery returns to nominal or expected terminal state
- no regression in tests or config validation
- performance regression comparison is generated from isolated runs

## Reproducibility Steps

```powershell
py -3.14 scripts\phase7_mission_scenarios.py
py -3.14 scripts\phase7_hil_runner.py
py -3.14 scripts\phase7_simulation_runner.py
py -3.14 scripts\phase7_failure_injection.py
py -3.14 scripts\phase8_advanced_scenarios.py
python scripts\validate_config_schemas.py
python -m unittest tests.test_dashboard_backend_status
ctest --test-dir build\validation-msvc -C Release --output-on-failure
go test ./...
py -3.14 scripts\phase6_performance_suite.py
py -3.14 scripts\phase6_stress_test.py
```

## Notes

- all results are workstation-local evidence unless otherwise stated
- physical hardware validation remains outside the Phase 8 software release scope
