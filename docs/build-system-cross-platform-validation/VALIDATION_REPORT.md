# Phase 2 Validation Report

Date: 2026-07-14

## Windows

- `windows-msvc-debug`: PASS, `112/112`
- `windows-msvc-release`: PASS, `112/112`
- `validation-msvc`: PASS
- `python -m unittest tests.test_dashboard_backend_status`: PASS, `12/12`
- `go test ./...`: PASS, `2/2`
- Windows install validation: PASS
- Windows clean-clone-style validation: PASS

## Linux Native Presets

- `linux-gcc-debug`: PASS, `112/112`
- `linux-gcc-release`: PASS, `112/112`
- `linux-clang-debug`: PASS, `112/112`
- `linux-clang-release`: PASS, `112/112`
- `linux-gcc-release-werror`: PASS, `112/112`
- `linux-gcc-release-minimal`: PASS, `112/112`
- `validation-linux-gcc`: PASS, `112/112`

## Sanitizers

- `linux-gcc-asan-ubsan`: PASS, `112/112`
- `linux-clang-asan-ubsan`: PASS, `112/112`

## Validator

Commands:

```bash
python3 scripts/local_validate.py
python3 scripts/local_validate.py --preset validation-linux-gcc --report docs/phase2/local_validate_linux_report.json
```

Result:

- PASS

## Installation And Consumer Validation

- Linux install from `build/linux-gcc-release`: PASS
- Linux downstream `find_package(DroneSwarmSensorFusion)` consumer: PASS

## Reproducibility

- Windows clean-clone-style validation: PASS
- Linux fresh-source snapshot validation with `linux-gcc-release-minimal`: PASS

## Git Hygiene

`git diff --check` had already passed for patch-shape and whitespace validation in the earlier Phase 2 sweep.
