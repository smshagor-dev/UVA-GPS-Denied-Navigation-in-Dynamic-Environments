# Phase 4 Thread Safety Report

Date: 2026-07-16

Evidence:

- `docs/architecture-safety-validation-hardening/tsan_ctest.log`
- `docs/architecture-safety-validation-hardening/tsan_build.log`
- `docs/architecture-safety-validation-hardening/LOCK_ORDER_POLICY.md`

## Results

ThreadSanitizer run:

- toolchain: Clang + `-fsanitize=thread`
- suite result after rebuilding current sources: `114/114` tests passed
- no remaining ThreadSanitizer warnings in the final log

Important nuance:

- an initial unsuppressed run reported external `libOpenNI2.so.0` mutex misuse during GoogleTest discovery
- that external noise was isolated with a suppression file
- a second issue came from OpenCV/TBB worker-pool teardown in visual/keyframe tests
- this recovery pass added a ThreadSanitizer-only `cv::setNumThreads(1)` guard in `VIOPipeline` and `KeyframeManager`, which allowed TSan to validate project code cleanly without changing production behavior

## Policy Hardening

- lock-order expectations are now documented in `docs/architecture-safety-validation-hardening/LOCK_ORDER_POLICY.md`

## Verdict

Thread safety status: PASS for software race detection

Remaining caveat:

- this is strong software evidence, not proof of hardware scheduling behavior under live sensor load

