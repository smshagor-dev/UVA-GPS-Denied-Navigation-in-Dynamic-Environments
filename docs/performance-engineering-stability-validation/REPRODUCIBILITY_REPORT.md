# Phase 6 Reproducibility Report

Date: 2026-07-16

## Scope

This pass verified fast reproducibility signals on the current workspace without restarting Phase 6 or redoing multi-hour validation.

## Verified In This Pass

Commands rerun successfully on 2026-07-16:

- `ctest --test-dir build/validation-msvc -C Release --output-on-failure`
- `python -m unittest tests.test_dashboard_backend_status`
- `go test ./...`
- `go test ./internal/controlplane/...`
- `py -3.14 scripts/phase6_performance_benchmark.py`
- `py -3.14 scripts/phase6_stress_test.py`

Results:

- native tests: `114/114` PASS
- Python tests: `12/12` PASS
- Go tests: PASS
- benchmark script: PASS
- stress script: PASS

## Configuration / Build State

Validated from the existing repository workspace:

- configured native build exists at `build/validation-msvc`
- benchmark and stress scripts still execute against the current source tree
- control-plane Go package still builds and tests successfully

## What Was Not Reproduced In This Pass

- fresh clone
- fresh configure from an empty build directory
- fresh full sanitizer rebuild
- fresh `8 h` soak completion

## Conclusion

Reproducibility status: `PARTIAL PASS`

Reason:

- quick rerun signals are clean and consistent
- full fresh-clone reproducibility was not re-executed in this pass
