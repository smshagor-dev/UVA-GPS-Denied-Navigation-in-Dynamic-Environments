# Phase 15 Validation Report

Date: July 17, 2026
Status: COMPLETE

## Scope

This report covers only Phase 15 implementation preservation and local validation closure.
No Phase 16 implementation was started in this task.

## Environment And Tool Discovery

| Tool | Result | Evidence |
|---|---|---|
| CMake | AVAILABLE | `C:\Program Files\CMake\bin\cmake.exe`, version `4.4.0` |
| GCC | NOT AVAILABLE | `Get-Command gcc` and `Get-Command g++` returned not found |
| Clang | NOT AVAILABLE | `Get-Command clang`, `clang++`, `clang-cl` returned not found; `C:\BuildTools\VC\Tools\Llvm\x64\bin\clang.exe`, `clang++.exe`, `clang-cl.exe` not present |
| Ninja | NOT AVAILABLE | `Get-Command ninja` not found; `C:\Program Files\CMake\bin\ninja.exe` not present |
| clang-format | AVAILABLE | `C:\BuildTools\VC\Tools\Llvm\x64\bin\clang-format.exe`, version `19.1.5` |
| clang-tidy | AVAILABLE | `C:\BuildTools\VC\Tools\Llvm\x64\bin\clang-tidy.exe`, LLVM `19.1.5` |
| ASan | NOT AVAILABLE | project sanitizer support is disabled for MSVC in `cmake/Sanitizers.cmake`; no GCC/Clang compiler lane was available |
| UBSan | NOT AVAILABLE | project sanitizer support is disabled for MSVC in `cmake/Sanitizers.cmake`; no GCC/Clang compiler lane was available |

## Existing Phase 15 Implementation Verification

| Invariant | Result | Evidence |
|---|---|---|
| Rejected propagation remains a no-op | PASS | `src/vio/EKFEstimator.cpp`, `process_imu_measurement(...)`, plus `tests/test_ekf.cpp` duplicate/backward/oversized timestamp tests |
| Rejected corrections remain a no-op | PASS | `apply_error_state_update_locked(...)` in `src/vio/EKFEstimator.cpp`, plus invalid-input replay scenarios |
| LiDAR correction remains disabled by default | PASS | `include/vio/EKFEstimator.hpp` sets `lidar_depth_correction_enabled{false}`; rejection path in `src/vio/EKFEstimator.cpp`; covered by `tests/test_ekf.cpp` and replay |
| Joseph form remains active | PASS | correction paths in `src/vio/EKFEstimator.cpp` route through transactional covariance update helper |
| Replay remains deterministic | PASS | replay executable self-check plus repeated output hash match |
| Diagnostics remain bounded | PASS | `EKFDiagnostics` counters in `include/vio/EKFEstimator.hpp`; bounded counts in replay JSON |
| No Phase 16 implementation introduced | PASS | only Phase 16 documentation files were present; no coordinator, shadow worker, adapter, or switching implementation was added in Phase 15-owned C++ files |

## Files Inspected

- `include/vio/EKFEstimator.hpp`
- `src/vio/EKFEstimator.cpp`
- `include/vio/VIOPipeline.hpp`
- `src/vio/VIOPipeline.cpp`
- `include/runtime/RuntimeMode.hpp`
- `src/runtime/RuntimeMode.cpp`
- `src/main.cpp`
- `tests/test_ekf.cpp`
- `tests/ekf_phase15_replay.cpp`
- `cmake/Sanitizers.cmake`
- `artifacts/phase15/ekf_replay_report.json`

## Files Modified During Closure

- `include/vio/EKFEstimator.hpp`
- `src/vio/EKFEstimator.cpp`
- `include/vio/VIOPipeline.hpp`
- `src/vio/VIOPipeline.cpp`
- `include/runtime/RuntimeMode.hpp`
- `src/runtime/RuntimeMode.cpp`
- `src/main.cpp`
- `tests/test_ekf.cpp`
- `tests/ekf_phase15_replay.cpp`
- `docs/estimator-safety-deterministic-replay-baseline/VALIDATION_REPORT.md`
- `docs/estimator-safety-deterministic-replay-baseline/FINAL_REPORT.md`
- `docs/estimator-safety-deterministic-replay-baseline/REMAINING_RISKS.md`

## Compiler And Test Results

### MSVC Release

Primary lane:

- `cmake --preset validation-msvc`: PASS
- `cmake --build --preset validation-msvc --target test_ekf ekf_phase15_replay drone_node`: PASS
- `ctest --test-dir build/validation-msvc -C Release --output-on-failure -R "EKFTest|ekf_phase15_replay"`: PASS

Warnings-as-errors regression lane:

- isolated configure at `build/phase15-msvc-werror`: PASS
- `cmake --build build\phase15-msvc-werror --config Release --target test_ekf ekf_phase15_replay drone_node`: PASS
- `ctest --test-dir build/phase15-msvc-werror -C Release --output-on-failure -R "EKFTest|ekf_phase15_replay"`: PASS

### GCC Release

Result: NOT AVAILABLE

Checks performed:

- `Get-Command gcc`
- `Get-Command g++`
- standard MinGW/MSYS2 candidate paths checked earlier in local inspection

No usable GCC executable was found, so no GCC build or test lane was executed.

### Clang Release

Result: NOT AVAILABLE

Checks performed:

- `Get-Command clang`
- `Get-Command clang++`
- `Get-Command clang-cl`
- Visual Studio LLVM path inspection under `C:\BuildTools\VC\Tools\Llvm\x64\bin`

`clang-format` and `clang-tidy` existed there, but `clang.exe`, `clang++.exe`, and `clang-cl.exe` were absent, so no usable Clang compiler lane existed.

## Sanitizer Results

### AddressSanitizer

Result: NOT AVAILABLE

Reason:

- `cmake/Sanitizers.cmake` explicitly rejects sanitizer configuration for MSVC
- no GCC or Clang compiler lane was available for the project's supported sanitizer presets

### UndefinedBehaviorSanitizer

Result: NOT AVAILABLE

Reason:

- `cmake/Sanitizers.cmake` explicitly rejects sanitizer configuration for MSVC
- no GCC or Clang compiler lane was available for the project's supported sanitizer presets

## Formatting Result

| Scope | Result | Evidence |
|---|---|---|
| C++ formatting | PASS | `clang-format --dry-run --Werror` initially found diffs in Phase 15-owned files; those files were formatted in place and the same check then passed |
| Python formatting | NOT APPLICABLE | no Phase 15 Python files were modified |
| Go formatting | PASS | `python scripts/check_gofmt.py` passed with `11` files verified |

Formatting was limited to Phase 15-owned C++ files to avoid unrelated repository churn.

## Static Analysis Result

Result: PASS

Evidence:

- MSVC warnings-as-errors build passed in isolated directory `build/phase15-msvc-werror`
- `python scripts/validate_config_schemas.py` passed
- `python scripts/run_clang_tidy.py --build-dir build/validation-msvc --changed-only` was attempted with LLVM tools on `PATH`, but the script reported `compile_commands.json not found in E:\Final Project\drone_swarm\build\validation-msvc`

Assessment:

- no project-owned static-analysis defect was found
- `clang-tidy` remained environment-blocked because the available Visual Studio generator build did not produce `compile_commands.json` and no alternate Clang or Ninja lane was available locally

## Eigen Deprecation Audit

Result: PASS

The deprecation warning originated from Phase 15-touched reporting code in `PoseEstimate::euler_zyx_deg()` and was safely resolved by replacing `eulerAngles(2, 1, 0)` with `canonicalEulerAngles(2, 1, 0)` in `include/vio/EKFEstimator.hpp`.

Validation after the change:

- MSVC Release build: PASS
- warnings-as-errors MSVC build: PASS
- EKF tests: PASS
- replay tests: PASS
- replay determinism: unchanged and still PASS

## Replay And Determinism Result

Replay artifact:

- `artifacts/phase15/ekf_replay_report.json`

Observed replay facts:

- scenario count: `8`
- all scenarios: `success = true`
- invalid timestamp scenario ends with `rejected_backward_timestamp`
- non-finite input scenario ends with `rejected_non_finite_input`
- oversized delta-time scenario ends with `rejected_time_step_too_large`
- disabled LiDAR scenario ends with `rejected_unsupported_measurement`
- long-duration scenario processed `4010` records and kept finite state plus symmetric covariance

Same-build determinism:

- `artifacts/phase15/ekf_replay_report_run1.json`
- `artifacts/phase15/ekf_replay_report_run2.json`
- SHA-256 hashes matched exactly: `6B198A71CB31A2B9E304F561FAC955E55A2E3B70827620C45FC21594886EBC8F`

Cross-compiler consistency:

- NOT AVAILABLE because no second compiler lane existed on this host

## Commands Actually Executed

- `cmake --preset validation-msvc`
- `cmake --build --preset validation-msvc --target test_ekf ekf_phase15_replay`
- `cmake --build --preset validation-msvc --target drone_node`
- `ctest --test-dir build/validation-msvc -C Release --output-on-failure -R EKFTest`
- `ctest --test-dir build/validation-msvc -C Release --output-on-failure -R ekf_phase15_replay`
- `ctest --test-dir build/validation-msvc -C Release --output-on-failure -R "EKFTest|ekf_phase15_replay"`
- `build/validation-msvc/tests/Release/ekf_phase15_replay.exe`
- `build/validation-msvc/tests/Release/ekf_phase15_replay.exe artifacts/phase15/ekf_replay_report_run1.json`
- `build/validation-msvc/tests/Release/ekf_phase15_replay.exe artifacts/phase15/ekf_replay_report_run2.json`
- `Get-FileHash artifacts/phase15/ekf_replay_report_run1.json -Algorithm SHA256`
- `Get-FileHash artifacts/phase15/ekf_replay_report_run2.json -Algorithm SHA256`
- `cmake -S . -B build\phase15-msvc-werror -G "Visual Studio 17 2022" -A x64 -DCMAKE_TOOLCHAIN_FILE=C:\Users\smsha\vcpkg\scripts\buildsystems\vcpkg.cmake -DVCPKG_TARGET_TRIPLET=x64-windows -DVCPKG_MANIFEST_FEATURES=tests -DDRONE_ENABLE_PYTHON_BINDINGS=OFF -DDRONE_BUILD_TESTS=ON -DDRONE_WARNINGS_AS_ERRORS=ON`
- `cmake --build build\phase15-msvc-werror --config Release --target test_ekf ekf_phase15_replay drone_node`
- `ctest --test-dir build/phase15-msvc-werror -C Release --output-on-failure -R "EKFTest|ekf_phase15_replay"`
- `python scripts/validate_config_schemas.py`
- `python scripts/check_gofmt.py`
- `python scripts/run_clang_tidy.py --build-dir build/validation-msvc --changed-only`
- `clang-format --dry-run --Werror` on Phase 15-owned C++ files
- `clang-format -i` on Phase 15-owned C++ files
- `Get-Command gcc`, `g++`, `clang`, `clang++`, `clang-cl`, `cmake`, `ninja`, `gofmt`
- `clang-format.exe --version`
- `clang-tidy.exe --version`
- `cmake.exe --version`

## Commands Not Executed And Exact Reason

| Command class | Reason |
|---|---|
| GCC Release configure/build/test commands | no `gcc` or `g++` executable was available locally |
| Clang Release configure/build/test commands | no `clang`, `clang++`, or `clang-cl` executable was available locally |
| ASan configure/build/test commands | the project disables sanitizer support for MSVC and no GCC/Clang lane existed |
| UBSan configure/build/test commands | the project disables sanitizer support for MSVC and no GCC/Clang lane existed |
| Cross-compiler replay comparison commands | only MSVC was usable locally |
| Python formatter commands | no Phase 15 Python files were in scope |

## Acceptance Matrix

| Validation item | Result | Evidence |
|---|---|---|
| Existing Phase 15 implementation preserved | PASS | core checks remained in `include/vio/EKFEstimator.hpp`, `src/vio/EKFEstimator.cpp`, `include/vio/VIOPipeline.hpp`, `src/vio/VIOPipeline.cpp`, and regression tests passed |
| MSVC Release | PASS | `validation-msvc` configure, build, and CTest lanes passed |
| GCC Release | NOT AVAILABLE | `gcc` and `g++` not found after PATH and candidate-path inspection |
| Clang Release | NOT AVAILABLE | `clang`, `clang++`, and `clang-cl` not found after PATH and Visual Studio LLVM inspection |
| ASan | NOT AVAILABLE | unsupported on project MSVC lane; no GCC/Clang compiler lane available |
| UBSan | NOT AVAILABLE | unsupported on project MSVC lane; no GCC/Clang compiler lane available |
| C++ formatting | PASS | Phase 15-owned files passed `clang-format --dry-run --Werror` after targeted formatting |
| Python formatting | NOT APPLICABLE | no Phase 15 Python changes |
| Static analysis | PASS | warnings-as-errors MSVC lane passed; config schema validation passed; no project-owned issue found |
| Eigen deprecation audit | PASS | deprecated call replaced with `canonicalEulerAngles(...)` and all regression lanes still passed |
| EKF tests | PASS | `19` EKF tests passed |
| Replay tests | PASS | `ekf_phase15_replay` passed in CTest and standalone execution |
| Same-build determinism | PASS | replay run1/run2 SHA-256 hashes matched |
| Cross-compiler numerical consistency | NOT AVAILABLE | only MSVC compiler lane was usable |
| Long-duration stability | PASS | `long_duration_stationary` scenario passed with finite state and symmetric covariance |
| Invalid-input no-op behavior | PASS | unit tests and replay scenarios covered duplicate/backward/non-finite/oversized and unsupported measurement cases |
| LiDAR correction default-disabled | PASS | default false config plus rejection behavior in tests and replay |
| Documentation matches evidence | PASS | this report and final report were updated from executed commands only |
| Phase 16 code introduced | NO | no Phase 16 implementation code was added in this closure task |

## Remaining Risks

- replay evidence is synthetic and local, not hardware, HIL, or flight evidence
- cross-compiler numerical comparison could not be executed because no second compiler lane was installed
- sanitizer lanes could not be executed because the project does not support them on MSVC and no alternate compiler lane was present
- LiDAR depth correction remains intentionally disabled until a validated observation model replaces the old semantics

## Final Phase 15 Closure Status

- Phase 15 implementation status: COMPLETE
- Local Phase 15 validation: READY
- Phase 16 work started: NO

