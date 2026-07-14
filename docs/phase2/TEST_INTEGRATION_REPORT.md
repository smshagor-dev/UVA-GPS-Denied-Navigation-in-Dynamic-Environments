# Test Integration Report

Date: 2026-07-14

## CTest Registration

All maintained native tests are registered through CTest in `tests/CMakeLists.txt`.

Labels in use:

- `unit`
- `navigation`
- `sensor`
- `simulation`
- `swarm`
- `security`
- `telemetry`
- `integration`

## Native Totals

Verified totals:

- Windows Release: `112/112` PASS
- Windows Debug: `112/112` PASS
- Validation preset: `112/112` PASS

## Notes

- `test_v2x` stays conditional on Fast-DDS availability and is not faked into the build when the dependency is absent.
- Tests run from the build tree and deploy required runtime DLLs into the target output directories.
