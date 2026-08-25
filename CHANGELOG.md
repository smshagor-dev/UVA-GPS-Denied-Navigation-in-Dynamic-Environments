# Changelog

All notable repository-level changes should be documented in this file.

## Unreleased

No unreleased changes are currently queued. New development after the v2.0.0 release should be documented here.

## [2.0.0] - 2026-08-20

### Added

- A complete 22-stage research-engineering lifecycle covering repository hardening, cross-platform builds, CI/CD, deterministic validation, software HIL, AI autonomy, deployment packaging, publication artifacts, estimator hardening, and shadow-estimator validation.
- Deterministic estimator replay and fail-closed EKF validation paths with transactional state/covariance update behavior.
- Active/shadow estimator architecture that preserves active-estimator authority while allowing independent shadow validation.
- Hardened shadow ESKF propagation, error-state injection/reset handling, and deterministic replay coverage.
- IMU-only stationary detection with hysteresis and automatic shadow-only zero-velocity updates (ZUPT).
- First-Estimate Jacobian (FEJ) snapshot support for shadow-estimator measurement Jacobian evaluation.
- Deterministic MSCKF camera-state sliding-window support with bounded oldest-first lifecycle management.
- Shadow-only feature-track history, multi-view triangulation, geometric landmark initialization, positive-depth checks, and reprojection gating.
- Shadow-only MSCKF feature constraint updates with residual/Jacobian construction, null-space projection, deterministic stacking, statistical gating, and Joseph-form correction.
- Sustained estimator-promotion readiness soak monitoring that requires configurable consecutive healthy samples and resets fail-closed on readiness regressions.
- Focused soak-validation tests for consecutive readiness, blocker-induced reset, degraded shadow health, and explicit monitor reset behavior.
- Release packaging for Linux and Windows, downstream install-consumer validation, SBOM generation, SHA-256 checksums, and tag-driven GitHub Release publication.
- Repository security and quality automation including formatting, static analysis, sanitizer, workflow-audit, dependency, and secret-scanning support.

### Changed

- Hardened repository ignore rules for environment files, build outputs, runtime databases, IDE state, logs, and platform-specific cache files.
- Removed insecure swarm-secret fallbacks from runtime and dashboard sidecar paths.
- Stopped echoing firmware-signing secrets from manifest-generation helpers.
- Corrected runtime configuration references to tracked configuration paths.
- Expanded CMake preset coverage for native validation, warnings-as-errors, sanitizers, coverage, packaging, and package-consumer checks.
- Updated repository documentation to distinguish software/research readiness from hardware, radio, HIL, tethered, and free-flight validation.
- Preserved active-estimator authority throughout Stages 16-22 and the sustained readiness soak work; no automatic estimator promotion or flight activation was introduced.

### Validation Contract

The v2.0.0 tag is intended to be published only after the release candidate passes the repository's applicable hosted validation gates, including:

- GCC, Clang, and MSVC native builds/tests
- warnings-as-errors validation
- clang-format and clang-tidy quality gates
- sanitizer and regression coverage
- Go and Python validation
- install/package consumer checks
- release-readiness container/configuration smoke validation
- Linux and Windows release packaging
- SBOM and checksum generation

### Safety and Scope

- v2.0.0 is a software/research release, not a flight-certification claim.
- No real free-flight validation is claimed.
- No production-radio qualification is claimed.
- Physical HIL and multi-drone/tethered validation remain future validation work.
- Simulation, replay, software HIL, and bench evidence must not be represented as operational airworthiness evidence.
