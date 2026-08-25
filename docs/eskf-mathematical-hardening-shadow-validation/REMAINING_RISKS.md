# Remaining Risks

Date: July 18, 2026

## 1. Incomplete GCC ASan/UBSan Evidence For Phase 17

- Classification: ENVIRONMENT LIMITATION
- Description: The Linux GCC ASan/UBSan preset exists, but the Phase 17 configure-build-run attempt did not complete in the executed session.
- Severity: Medium
- Evidence: `wsl.exe bash -lc "cd '/mnt/e/Final Project/drone_swarm' && cmake --preset linux-gcc-asan-ubsan && cmake --build --preset linux-gcc-asan-ubsan --target test_phase17_eskf ekf_phase17_replay && build/linux-gcc-asan-ubsan/tests/Debug/test_phase17_eskf && build/linux-gcc-asan-ubsan/tests/Debug/ekf_phase17_replay"` timed out after `304115 milliseconds`.
- Impact: Sanitizer evidence exists for Clang and TSan, but not yet for the GCC sanitizer preset in the Phase 17 report set.
- Mitigation: Reuse the partially prepared build directory or split configure, build, and run into separate longer-lived steps and rerun the Phase 17 targets.
- Blocks Phase 18: No
