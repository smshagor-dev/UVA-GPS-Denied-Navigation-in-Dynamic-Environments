# Contributing

## Goal

This project mixes C++, Python, Go, firmware, and deployment concerns, so contributions are easiest when they stay scoped and reproducible.

Use this guide when adding features, fixing bugs, or updating docs.

## Before You Change Anything

1. Work from the repository root.
2. Check for unrelated local changes before editing.
3. Prefer a fresh build directory instead of reusing older checked-in build folders.

Helpful commands:

```powershell
git status --short
rg --files
```

## Development Areas

Use the right language/runtime for the job:

- `src/` and `include/`: onboard C++ logic
- `gui/`: Python dashboard
- `cmd/` and `internal/`: Go control plane
- `firmware/`: ESP32-CAM firmware
- `docs/`: architecture and operational documentation

## Local Checks

### Python

```powershell
python -m py_compile main.py gui/dashboard.py scripts/drone_setup.py
```

### Go

```powershell
go test ./...
```

### C++

Use a clean configure first:

```powershell
cmake -S . -B build-dev -DBUILD_TESTS=ON
cmake --build build-dev --config Release
ctest --test-dir build-dev --output-on-failure -C Release
```

If configure fails, capture the missing dependency rather than silently skipping the check.

## Contribution Rules

- Keep changes focused; avoid mixing refactors with behavior changes unless needed.
- Update docs when CLI flags, APIs, env vars, or deployment steps change.
- Do not trust old checked-in build folders for validation.
- Preserve optional behavior for `pybind11`, `Fast-DDS`, and `TensorRT` instead of assuming they exist everywhere.
- Prefer simulation/degraded fallback behavior over hard failure when that matches the current codebase pattern.

## Code Style

### C++

- Follow the existing CMake target structure.
- Keep headers in `include/` and implementations in `src/`.
- Avoid adding platform-specific logic without guards.

### Python

- Keep the dashboard responsive; avoid blocking the UI thread.
- Reuse the existing logging setup and command/telemetry worker patterns.

### Go

- Keep handlers small and state mutations centralized in `internal/controlplane/state.go`.
- Run formatting before finalizing:

```powershell
gofmt -w cmd\control-plane\main.go internal\controlplane\*.go
```

## Documentation Expectations

Update the relevant document when you change:

- startup flow
- environment variables
- API routes
- build requirements
- deployment assumptions

At minimum, reflect those updates in [README.md](README.md).

## Pull Request Checklist

Before opening a PR or handing off changes:

1. Confirm `git status` only shows intended files.
2. Run the language-specific checks that apply to your change.
3. Note any checks you could not run.
4. Mention platform assumptions, especially for C++ dependencies or hardware-only features.
5. Include screenshots for dashboard UI changes when relevant.
