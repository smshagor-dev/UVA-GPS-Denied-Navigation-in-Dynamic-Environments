# Container Security Report

Date: 2026-07-16

## Result

Status: PASS

## Checks

### Runtime user

- production image runs as non-root user `drone`
- verified uid/gid: `999/999`

### Secrets handling

- no secret values are baked into the production image environment
- image environment contains only non-secret runtime defaults:
  - `DRONE_SWARM_ADDR`
  - `DRONE_BACKEND_MODE`
  - `DRONE_BACKEND_SIMULATION_ENABLED`

### Build-context hygiene

- `.dockerignore` excludes:
  - `.git`
  - `.env`
  - `.env.local`
  - `certs`
  - build outputs
  - logs
  - transient artifacts

### Exposed surface

- exposed port: `8080/tcp`
- container healthcheck uses local loopback HTTP probe

### Production image contents

- runtime image contains `curl` for health checks
- production image does not include `gcc` or `g++`
- build toolchain remains out of the runtime image

## Raw evidence

- [docker_image_inspect_prod.json](./evidence/docker_image_inspect_prod.json)
- [docker_history_prod.txt](./evidence/docker_history_prod.txt)
- [docker_prod_runtime_user.txt](./evidence/docker_prod_runtime_user.txt)
- [.dockerignore](../../.dockerignore)

