# Changelog

All notable repository-level changes should be documented in this file.

## Unreleased

### Added

- Public-release repository metadata: `CONTRIBUTING.md`, `SECURITY.md`, `.github/` templates, `CODEOWNERS`, and Phase 1 audit reports.
- Phase 3 CI/CD automation: GitHub Actions workflows for CI, security scanning, nightly deep validation, and tag-driven release packaging.
- Repository quality-gate configuration: `.editorconfig`, `.clang-format`, `.clang-tidy`, `.gitleaks.toml`, and Dependabot automation.
- CI helper scripts for clang-format verification, gofmt verification, clang-tidy execution, native coverage generation, workflow auditing, and downstream package-consumer validation.

### Changed

- Hardened repository ignore rules for environment files, build outputs, runtime databases, IDE state, logs, and platform-specific cache files.
- Removed insecure swarm-secret fallbacks from the runtime and dashboard sidecar path.
- Stopped echoing firmware signing secrets from the manifest-generation helper.
- Corrected runtime configuration references to the tracked config file paths that actually exist in this repository.
- Expanded CMake preset coverage with native coverage and package-validation profiles that align CI with the Phase 2 build contract.
- Updated README badges and validation notes to reflect real CI workflows rather than static status claims.

### Removed

- Tracked generated artifacts and sensitive local-runtime files from version control tracking where safe to preserve local copies.
