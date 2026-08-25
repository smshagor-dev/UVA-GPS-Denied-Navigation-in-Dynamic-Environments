# Reproducibility Report

Date: 2026-07-14
Commit SHA reference: `cb94b22008e217b3f016888d3e98a2ca5aa5af7a`

## Windows Clean-Clone-Style Validation

Result:

- PASS
- `112/112` tests passed
- install validation passed

## Linux Fresh-Source Validation

Method:

- Created a fresh source snapshot tarball from the working tree with `.git`, `build/`, logs, caches, and runtime databases excluded.
- Extracted into `/tmp/drone_swarm_source_snapshot`.
- Validated from that extracted source tree, not from the in-place workspace build directories.

Preset used:

- `linux-gcc-release-minimal`

Commands:

```bash
cmake --preset linux-gcc-release-minimal
cmake --build --preset linux-gcc-release-minimal -j1
ctest --preset linux-gcc-release-minimal --output-on-failure
cmake --install build/linux-gcc-release-minimal --prefix /tmp/drone_swarm_clone_install
cmake -S /tmp/drone_swarm_clone_consumer -B /tmp/drone_swarm_clone_consumer/build -G Ninja \
  -DCMAKE_PREFIX_PATH=/tmp/drone_swarm_clone_install
cmake --build /tmp/drone_swarm_clone_consumer/build
```

Result:

- configure: PASS
- build: PASS
- CTest: `112/112` PASS
- install: PASS
- downstream consumer: PASS

## Notes

- Full in-worktree Linux release validation also passed earlier.
- WSL proved unstable during one heavier clean-source full-release attempt with Python bindings enabled, so the reproducibility proof was captured with the repo's minimal Linux preset for deterministic clean-source evidence.
