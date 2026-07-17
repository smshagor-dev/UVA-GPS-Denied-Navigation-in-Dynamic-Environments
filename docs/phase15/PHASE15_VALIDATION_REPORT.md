# Phase 15 Validation Report

## Validation status

Phase 15 validation was executed on July 17, 2026 in the local development environment. This report covers software-only evidence and does not claim hardware or flight validation.

## Commands executed

- `cmake --preset validation-msvc -DDRONE_ENABLE_PYTHON_BINDINGS=OFF`
- `cmake --build --preset validation-msvc --target test_ekf test_navigation_intelligence --parallel`
- `ctest --test-dir build/validation-msvc -C Release --output-on-failure -R "EKFTest|LocalizationFusion|RuntimeMode"`
- `python -m py_compile scripts/phase15_estimator_replay.py`
- `python -m black scripts/phase15_estimator_replay.py`
- `python scripts/phase15_estimator_replay.py --input datasets/estimator/phase15_stationary_replay.json --output artifacts/phase15_estimator_replay_report.json`
- `build\validation-msvc\tests\Release\test_ekf.exe --gtest_filter=EKFTest.DepthUpdateCorrection:EKFTest.ZUPTClearsVelocity:EKFTest.ZuptPreservesPositionAttitudeAndBiases`
- `build\validation-msvc\tests\Release\test_navigation_intelligence.exe --gtest_filter=LocalizationFusion.RejectsNonFiniteTdoaInput`

## Summary

- The estimator baseline now rejects non-finite inputs and timestamp violations.
- The ambiguous LiDAR depth update is disabled instead of silently shrinking covariance.
- Diagnostics and replay tooling are present for deterministic software validation.
- The focused native validation slice passed in full: 36 of 36 `EKFTest`, `LocalizationFusion`, and `RuntimeMode` cases passed on July 17, 2026.
- The deterministic replay artifact was generated at `artifacts/phase15_estimator_replay_report.json` with replay hash `ac6205282a71044d6434eced9ebe6c6c8710846204f47b43d4dbd62757a694a0`.
