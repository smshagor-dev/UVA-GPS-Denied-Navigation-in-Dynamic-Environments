# Build Artifact Report

Date: 2026-07-14  
Commit SHA: `cb94b22008e217b3f016888d3e98a2ca5aa5af7a`  
Repository state: `UNCOMMITTED WORKTREE`  
Operating system: Microsoft Windows 11 Pro 10.0.26200 64-bit

## Tool Versions
- Git 2.55.0.windows.2
- CMake 4.4.0 portable
- MSVC 19.44.35228.0
- Windows SDK 10.0.22621.0

## Commands Executed
- `git status --short`
- `git diff --cached --name-status`
- `git ls-files`
- clean validation runs that created:
  - `build-phase1-clean/`
  - `build-local-validate/`

## Findings
Artifacts scheduled for removal from versioned content:
- `.env`
- `build/CMakeFiles/3.28.1/CMakeSystem.cmake`
- `build/CMakeFiles/3.28.1/CompilerIdCXX/CMakeCXXCompilerId.cpp`
- `build/CMakeFiles/CMakeConfigureLog.yaml`
- `build/CMakeFiles/cmake.check_cache`
- `data/dashboard/dashboard.sqlite3`
- `final_review.md`

Validation-only artifacts created locally and intentionally left untracked:
- `build-phase1-clean/`
- `build-local-validate/`
- deployed dependency DLLs beneath each build tree
- `build-phase1-clean/docs/foundation-repository-hygiene/ctest-results.xml`

Tracked executable status:
- No generated validation executable from `build-phase1-clean/` or `build-local-validate/` is staged.
- The clean validation build produced local-only outputs such as:
  - `build-phase1-clean\Release\drone_node.exe`
  - `build-phase1-clean\tests\Release\test_*.exe`
  - `build-local-validate\Release\drone_node.exe`

## PASS / FAIL / SKIP / BLOCKED Counts
- PASS: 5
- FAIL: 0
- SKIP: 0
- BLOCKED: 0

## Remaining Limitations
- Staged deletions still appear in `git ls-files` because they are present in `HEAD` until a commit records their removal.
- Local validation directories are intentionally preserved as evidence for this Phase 1 closure pass.

## Evidence Paths
- `build-phase1-clean/CMakeCache.txt`
- `build-phase1-clean/Release/drone_node.exe`
- `build-phase1-clean/tests/Release/test_ekf.exe`
- `build-local-validate/CMakeCache.txt`
- `docs/foundation-repository-hygiene/VALIDATION_REPORT.md`

