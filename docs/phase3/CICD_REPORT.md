# Phase 3 CI/CD Report

Date: July 17, 2026

## Objective

This report documents the CI/CD coverage that supports the Phase 3 control-plane and command-orchestration layer already present in the repository. It closes a documentation gap only. No workflow logic, application logic, or runtime behavior was changed to produce this file.

## Phase 3 Scope

Phase 3 is the point where orchestration responsibilities moved into the Go control plane. The architecture notes describe this phase as:

- add `command-dispatcher`
- move formation, election, and mission dispatch into the Go control plane

That means the CI/CD evidence relevant to Phase 3 must cover:

- Go backend build and test validation
- command and orchestration path regression checks
- cross-platform native integration safety
- packaging and installability for downstream consumers
- recurring deep validation and security scanning

## Documented Workflow Sources

This report is based on the workflows already tracked in the repository:

- [.github/workflows/ci.yml](../../.github/workflows/ci.yml)
- [.github/workflows/nightly.yml](../../.github/workflows/nightly.yml)
- [.github/workflows/release.yml](../../.github/workflows/release.yml)
- [.github/workflows/security.yml](../../.github/workflows/security.yml)

## Pipeline Overview

The repository CI/CD layout is organized into four layers:

1. `ci.yml`
   Standard push and pull-request validation for formatting, static analysis, Python, Go, native builds, Windows validation, and full local validation.

2. `nightly.yml`
   Deep validation for warnings-as-errors, sanitizer runs, coverage generation, package validation, SBOM generation, and extended artifact capture.

3. `release.yml`
   Tag-driven and manual release packaging for Linux and Windows, with checksums, install validation, and release publication support.

4. `security.yml`
   Security-specific validation covering dependency review, secret scanning, dependency vulnerability scanning, and CodeQL analysis.

## Trigger Model

### Standard CI

`.github/workflows/ci.yml` runs on:

- push to `main`
- pull request targeting `main`
- manual dispatch

This makes it the primary regression gate for Phase 3 code changes.

### Nightly Deep Validation

`.github/workflows/nightly.yml` runs on:

- scheduled nightly execution
- manual dispatch

This provides recurring evidence for deeper quality gates that are too heavy for every pull request.

### Release Validation

`.github/workflows/release.yml` runs on:

- version tags matching `v*`
- manual dispatch

This gives a release-oriented validation path for packaged outputs.

### Security Validation

`.github/workflows/security.yml` runs on:

- push to `main`
- pull request targeting `main`
- scheduled execution
- manual dispatch

This ensures security review is continuous rather than one-time.

## CI Workflow Coverage

The main CI workflow contains the following job families.

### Workflow linting

The pipeline validates workflow definitions using:

- `actionlint`
- `python3 scripts/audit_workflows.py`

This reduces the risk of broken automation definitions blocking Phase 3 backend changes.

### Formatting

The formatting stage checks:

- C++ formatting through `scripts/check_clang_format.py`
- Python formatting with `black --check`
- Go formatting through `scripts/check_gofmt.py`

### Static analysis

The static-analysis stage configures compile commands and runs:

- `clang-tidy`

This provides an additional guard against integration-level defects in the mixed-language codebase that surrounds the Phase 3 backend.

### Python CI

The Python stage performs:

- syntax validation with `python3 -m py_compile`
- unit testing with coverage
- artifact upload for coverage reports

This is relevant to dashboard and scripting layers that interact with the control plane and operator workflows introduced around earlier orchestration phases.

### Go CI

The Go stage performs:

- `go vet ./...`
- `go test -race ./...`
- Go coverage generation
- artifact upload for coverage results

This is the most directly relevant CI evidence for Phase 3 because the control-plane orchestration logic lives in the Go backend.

### Native validation

The CI workflow also validates the native stack with:

- Linux GCC validation
- Linux GCC debug
- Linux GCC release
- Linux Clang release
- Linux Clang debug
- Windows MSVC validation
- Windows MSVC debug
- Windows MSVC release

Each of these configure/build/test paths produces test artifacts through `ctest` JUnit output uploads.

### Full local validator

The `full-validator` job runs:

- `python3 scripts/local_validate.py`

This is especially useful as an integration checkpoint because it exercises a repo-defined validation flow instead of only isolated language-specific test commands.

### Install and consumer validation

The `install-consumer` job validates:

- install step correctness
- downstream consumer package behavior
- exported package usability

This matters because Phase 3 backend work still depends on the wider project remaining consumable as a validated software package.

### Required gate aggregation

The `ci-required` job enforces that the critical jobs all succeed before the workflow is considered successful. That creates a single decision gate for standard CI.

## Nightly Workflow Coverage

The nightly workflow expands validation depth beyond the default CI path.

### Compiler strictness and profile diversity

It includes:

- GCC warnings-as-errors validation
- GCC minimal-profile validation
- Clang debug validation

This helps catch issues that may not surface under only one preset.

### Sanitizer validation

The nightly pipeline runs:

- GCC ASan/UBSan
- Clang ASan/UBSan

with explicit runtime options such as:

- `ASAN_OPTIONS=detect_leaks=1:halt_on_error=1:abort_on_error=1`
- `UBSAN_OPTIONS=print_stacktrace=1:halt_on_error=1`

This strengthens confidence that Phase 3 backend integration does not introduce hidden memory or undefined-behavior regressions elsewhere in the stack.

### Coverage generation

Nightly coverage includes:

- native coverage generation
- Python coverage
- Go coverage

and uploads the coverage artifacts for later inspection.

### Package validation

The nightly workflow also validates:

- package-preset configure/build/test
- installability
- CPack packaging
- downstream consumer checks
- SBOM generation with `syft`
- checksum generation

## Release Workflow Coverage

The release workflow documents how validated software artifacts are prepared for release use.

### Linux release packaging

The Linux path performs:

- configure/build/test
- install step
- CPack archive generation
- downstream consumer validation
- SBOM generation
- SHA256 checksum generation

### Windows release packaging

The Windows path performs:

- configure/build/test
- install step
- CPack archive generation
- SHA256 checksum generation

### Release publication

The workflow contains a publication stage using `gh release create` for tagged releases after packaging jobs complete.

## Security Workflow Coverage

The security workflow adds a security-focused CI layer that complements functional validation.

### Dependency review

For pull requests, the workflow runs GitHub dependency review.

### Secret scanning

The workflow installs and runs `gitleaks` against:

- the working tree
- git history on trusted refs

### Dependency vulnerability scanning

The workflow runs:

- `pip-audit`
- `govulncheck`

### CodeQL analysis

CodeQL is configured for:

- `c-cpp`
- `python`
- `go`

This is important because the Phase 3 backend is not isolated; it participates in a multi-language system boundary.

### Required security gate

The workflow contains a final gate that fails the security pipeline if the required security jobs do not succeed.

## Artifact Evidence

The current CI/CD setup uploads machine-readable artifacts from multiple stages, including:

- Python coverage reports
- Go coverage reports
- native `ctest` JUnit reports
- local validator reports
- install manifests
- sanitizer outputs
- coverage bundles
- package bundles
- SBOM files
- checksum manifests
- security scan reports

This is useful Phase 3 evidence because it shows the repo is not relying on narrative-only validation.

## Phase 3 Relevance Mapping

The following table maps pipeline coverage back to the Phase 3 backend/orchestration scope.

| Phase 3 concern | Existing CI/CD evidence |
|---|---|
| Go control-plane correctness | `go vet`, `go test -race`, Go coverage |
| Mixed-language integration safety | Linux and Windows native configure/build/test jobs |
| Operator and scripting compatibility | Python syntax checks and unit-test coverage |
| Repo-wide orchestration regression checks | `scripts/local_validate.py` in `full-validator` |
| Long-running validation depth | nightly deep validation workflow |
| Release readiness | release packaging workflow |
| Security posture | secret scanning, dependency audit, CodeQL |

## What This Report Does Not Claim

This report documents software automation coverage only. It does not claim:

- hardware-in-the-loop execution
- physical drone testing
- free-flight or tethered-flight validation
- PX4, Gazebo, Ignition, or SITL execution
- cloud deployment certification
- airworthiness or operational production approval

## Conclusion

The repository already contains a substantial CI/CD framework that covers the backend, native runtime, Python tooling, packaging, release flow, and security scanning relevant to Phase 3. The missing part was the documentation artifact in `docs/phase3/`, not the workflow layer itself.

## Verdict

Status: COMPLETE
