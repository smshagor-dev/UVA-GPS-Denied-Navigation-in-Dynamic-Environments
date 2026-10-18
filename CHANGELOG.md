# Changelog

All notable repository-level changes should be documented in this file.

## Unreleased

Post-v3.0.0 release metadata and packaging-workflow corrections are tracked on `main`. These corrections are not part of the immutable v3.0.0 source tag.

## [3.0.0] - 2026-10-18

### Added

- Expanded firmware-admission regression coverage with 17 reviewer-focused cases for independent version/counter rollback, signer and digest rejection, boot claims, measurement policy, maintenance-token presence, and state persistence.
- Added fail-closed localization metadata regression tests and additional MSCKF, covariance, visual-feature, and deterministic replay validation.

### Fixed

- Rejected unsafe field/production trust settings with missing signing secret or trusted signer allowlist, and denied admission when accepted-state persistence fails.
- Protected localization confidence/state from non-finite input metadata.
- Repaired cross-platform CI, linker, formatting, static analysis, CodeQL, Nightly GCC strict builds, CPack artifact handling, CycloneDX SBOM generation, and checksum verification.

### Changed

- Set CMake/CPack project version to 3.0.0 and refreshed the README landing section.
- Preserved software-only evidence boundaries: no hardware-secure-boot, protected rollback state, authenticated maintenance authority, free-flight qualification, or operational airworthiness claims.

### Validation and merged PRs

- Exact release source: `d5c30268ceceec62f169e6a6fa20ef4351e1d722`.
- GitHub Actions CI, Security, and Nightly Deep Validation PASS; Linux native CTest 273/273 PASS; release-package and SBOM checks PASS in Nightly validation.
- Merged PRs: [#26](https://github.com/smshagor-dev/UVA-GPS-Denied-Navigation-in-Dynamic-Environments/pull/26), [#27](https://github.com/smshagor-dev/UVA-GPS-Denied-Navigation-in-Dynamic-Environments/pull/27), [#28](https://github.com/smshagor-dev/UVA-GPS-Denied-Navigation-in-Dynamic-Environments/pull/28), [#29](https://github.com/smshagor-dev/UVA-GPS-Denied-Navigation-in-Dynamic-Environments/pull/29), [#30](https://github.com/smshagor-dev/UVA-GPS-Denied-Navigation-in-Dynamic-Environments/pull/30), [#31](https://github.com/smshagor-dev/UVA-GPS-Denied-Navigation-in-Dynamic-Environments/pull/31), [#32](https://github.com/smshagor-dev/UVA-GPS-Denied-Navigation-in-Dynamic-Environments/pull/32), [#33](https://github.com/smshagor-dev/UVA-GPS-Denied-Navigation-in-Dynamic-Environments/pull/33), [#34](https://github.com/smshagor-dev/UVA-GPS-Denied-Navigation-in-Dynamic-Environments/pull/34), [#35](https://github.com/smshagor-dev/UVA-GPS-Denied-Navigation-in-Dynamic-Environments/pull/35), [#36](https://github.com/smshagor-dev/UVA-GPS-Denied-Navigation-in-Dynamic-Environments/pull/36), [#38](https://github.com/smshagor-dev/UVA-GPS-Denied-Navigation-in-Dynamic-Environments/pull/38), [#39](https://github.com/smshagor-dev/UVA-GPS-Denied-Navigation-in-Dynamic-Environments/pull/39), [#40](https://github.com/smshagor-dev/UVA-GPS-Denied-Navigation-in-Dynamic-Environments/pull/40), [#41](https://github.com/smshagor-dev/UVA-GPS-Denied-Navigation-in-Dynamic-Environments/pull/41), [#47](https://github.com/smshagor-dev/UVA-GPS-Denied-Navigation-in-Dynamic-Environments/pull/47), and [#48](https://github.com/smshagor-dev/UVA-GPS-Denied-Navigation-in-Dynamic-Environments/pull/48).

See [the versioned release notes](docs/releases/v3.0.0.md) for per-PR details and scientific scope.

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
