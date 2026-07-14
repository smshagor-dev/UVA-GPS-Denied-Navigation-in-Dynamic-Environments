# Changelog

All notable repository-level changes should be documented in this file.

## Unreleased

### Added

- Public-release repository metadata: `CONTRIBUTING.md`, `SECURITY.md`, `.github/` templates, `CODEOWNERS`, and Phase 1 audit reports.

### Changed

- Hardened repository ignore rules for environment files, build outputs, runtime databases, IDE state, logs, and platform-specific cache files.
- Removed insecure swarm-secret fallbacks from the runtime and dashboard sidecar path.
- Stopped echoing firmware signing secrets from the manifest-generation helper.
- Corrected runtime configuration references to the tracked config file paths that actually exist in this repository.

### Removed

- Tracked generated artifacts and sensitive local-runtime files from version control tracking where safe to preserve local copies.
