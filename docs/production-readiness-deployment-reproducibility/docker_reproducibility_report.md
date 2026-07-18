# Docker Reproducibility Report

Date: 2026-07-16

## Result

Status: PASS

## Method

Because the current Phase 5.5 state in the primary workspace includes uncommitted files and edits, reproducibility was validated from a temporary snapshot repository created from the current working tree, then cloned via `git clone`.

This preserves the actual current state instead of silently falling back to the older `HEAD` commit.

## Reproducibility path

1. Create snapshot copy of the current workspace
2. Initialize snapshot Git repository
3. Commit snapshot state
4. `git clone` snapshot into a fresh workspace
5. From the fresh clone:
   - build production image
   - expand production compose config
   - expand simulation compose config
   - bring up simulation compose stack
   - verify `/api/v1/ready`

## Observed outcome

- snapshot commit created successfully:
  - `12c99ab1245f67bf30f5bd61ccb80330de7169ce`
- fresh clone created successfully
- fresh-clone Docker build succeeded
- fresh-clone compose config succeeded for both compose files
- fresh-clone simulation runtime started successfully
- fresh-clone readiness endpoint returned HTTP `200`

## Raw evidence

- [clone_head.txt](./repro_artifacts/clone_head.txt)
- [clone_exit_codes.txt](./repro_artifacts/clone_exit_codes.txt)
- [clone_docker_build.out](./repro_artifacts/clone_docker_build.out)
- [clone_docker_build.err](./repro_artifacts/clone_docker_build.err)
- [clone_compose_prod.out](./repro_artifacts/clone_compose_prod.out)
- [clone_compose_sim.out](./repro_artifacts/clone_compose_sim.out)
- [clone_compose_up.out](./repro_artifacts/clone_compose_up.out)
- [clone_ready.json](./repro_artifacts/clone_ready.json)
- [clone_logs.txt](./repro_artifacts/clone_logs.txt)

