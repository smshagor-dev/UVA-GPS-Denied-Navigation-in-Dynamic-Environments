# Phase 6.75 ThreadSanitizer Root Cause Analysis

Date: 2026-07-16

Primary evidence:

- `docs/phase6/tsan_ctest_phase65.log`
- `docs/phase4/tsan_ctest.log`

## Goal

Determine whether the remaining ThreadSanitizer warnings come from:

1. repository-owned code
2. third-party libraries
3. false positives

## Result

Final classification: `THIRD-PARTY RACE / EXTERNAL LIMITATION`

Status: `PASS with documented external limitation`

## Complete Warning Pattern

Every failing TSan report in `docs/phase6/tsan_ctest_phase65.log` has the same warning class:

- `ThreadSanitizer: unlock of an unlocked mutex (or by a wrong thread)`

Representative stack:

```text
#0 pthread_mutex_unlock
#1 libOpenNI2.so.0+0x15d88
#2 ld-linux-x86-64.so.2:call_init

allocation:
#0 calloc
#1 libOpenNI2.so.0+0x158dc

mutex creation:
#0 pthread_mutex_init
#1 libOpenNI2.so.0+0x15396
```

Observed variant:

- the third frame is either loader `call_init` or `libOpenNI2.so.0+0x7d2f`

No warning stack in the captured log resolves to a repository-owned source file.

## Origin Inventory

The Phase 6.5 TSan log contains `249` warning blocks grouped into `7` unique binary-plus-library origin sets:

| Test binary | Warning count | Third-party origin | Classification |
| --- | ---: | --- | --- |
| `test_ekf` | 27 | `libOpenNI2.so.0` | Third-party dependency |
| `test_sensors` | 48 | `libOpenNI2.so.0` | Third-party dependency |
| `test_slam` | 12 | `libOpenNI2.so.0` | Third-party dependency |
| `test_autonomy` | 51 | `libOpenNI2.so.0` | Third-party dependency |
| `test_navigation_intelligence` | 69 | `libOpenNI2.so.0` | Third-party dependency |
| `test_swarm_security` | 24 | `libOpenNI2.so.0` | Third-party dependency |
| `test_telemetry` | 18 | `libOpenNI2.so.0` | Third-party dependency |

Important point:

- `test_telemetry` and `test_swarm_security` are not OpenNI driver tests, yet they still reproduce the same warning pattern
- this indicates the warning is tied to transitive library loading and initialization, not to test-specific repository logic

## Why The Repository Is Not The Origin

### 1. The stacks terminate inside `libOpenNI2.so.0`

The warning site, allocation site, and mutex creation site all resolve to `libOpenNI2.so.0` offsets:

- unlock: `libOpenNI2.so.0+0x15d88`
- allocation: `libOpenNI2.so.0+0x158dc`
- init: `libOpenNI2.so.0+0x15396`

No repository source path appears in those frames.

### 2. The warning reproduces across unrelated tests

The same warning occurs in:

- EKF tests
- sensor tests
- SLAM tests
- autonomy tests
- navigation tests
- swarm security tests
- telemetry serialization tests

That breadth is inconsistent with a single repository race localized to one subsystem.

### 3. Telemetry-only tests do not use OpenNI APIs directly

`tests/test_telemetry.cpp` exercises `ControlPlaneTelemetryClient` serialization and retry logic. It does not contain OpenNI calls, camera driver calls, or repository thread logic around OpenNI initialization.

Yet `test_telemetry` still emits the same `libOpenNI2.so.0` warning.

### 4. The warning matches dynamic library initialization behavior

The stack includes:

- `ld-linux-x86-64.so.2:call_init`

This indicates the warning is triggered during shared-library constructor or loader initialization flow, not from a repository call site that explicitly unlocks a mutex.

### 5. Phase 4 already isolated the same external issue

`docs/phase4/RACE_DETECTION_REPORT.md` records that the earlier TSan obstacle was narrowed to external `libOpenNI2.so.0` mutex misuse and that the race-clean run required handling that external issue separately from repository code.

The new Phase 6.5 unsuppressed rerun reproduces the same external pattern.

## Dependency Chain

WSL package versions on 2026-07-16:

| Library | Installed package | Version |
| --- | --- | --- |
| OpenNI2 | `libopenni2-0` | `2.2.0.33+dfsg-19` |
| OpenCV core | `libopencv-core410` | `4.10.0+dfsg-7ubuntu5` |
| TBB | `libtbb12` | `2022.3.0-2` |

`ldd` on `build/linux-clang-tsan/tests/Debug/test_telemetry` and `test_swarm_security` shows both binaries transitively load:

- `libOpenNI2.so.0`
- multiple `libopencv_*.so.410` libraries
- `libtbb.so.12`

This explains why unrelated tests can reproduce the same OpenNI2 initialization warning.

## Known Issue Status

Known issue classification:

- upstream-style third-party library mutex misuse during initialization is strongly indicated
- an exact public upstream issue ID for `libopenni2-0 2.2.0.33+dfsg-19` was not confirmed in this pass

What is known conclusively from local evidence:

- the warning originates in `libOpenNI2.so.0`
- it reproduces during dependency initialization
- it is not localized to repository logic

## False Positive Assessment

Classification: `NOT TREATED AS A PURE FALSE POSITIVE`

Reason:

- TSan is reporting a real misuse pattern shape: unlock of an unlocked or wrong-thread-owned mutex
- however, the misuse appears to belong to the external library, not to repository-owned code

So the correct classification is:

- `third-party dependency issue`

not:

- `repository race`
- `pure sanitizer false positive`

## Phase 6.75 Conclusion

Repository-owned race:

- not evidenced

Third-party dependency race:

- evidenced in `libOpenNI2.so.0`

Required code fix in this repository:

- none justified by the captured traces

Safe suppression policy:

- suppression may be acceptable only if it is documented as a third-party initialization issue
- no suppression was added in this Phase 6.75 closure pass

## Final Verdict

ThreadSanitizer classification: `PASS with documented external limitation`
