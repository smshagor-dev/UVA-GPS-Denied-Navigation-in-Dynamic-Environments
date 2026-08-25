# Reproducibility Final Report

Date: 2026-07-16

## Objective

Validate the documented fresh-environment path:

1. fresh checkout
2. docker build
3. docker compose config
4. docker compose up

## Local evidence collected

Validated locally:

- `go test ./...`
- `py -3.14 scripts/validate_config_schemas.py`
- `docker compose -f docker-compose.yml config`
- `docker compose -f docker-compose.simulation.yml config`

Not validated locally:

- `git clone` into a fresh temp workspace
- `docker build`
- `docker compose up`

## Blocker

Fresh-environment Docker reproducibility could not be completed because the Docker daemon was unavailable:

- `failed to connect to the docker API at npipe:////./pipe/dockerDesktopLinuxEngine`
- `The system cannot find the file specified.`

## Reproducibility status

Status: PARTIAL

The source-level and compose-definition reproducibility path is in place, but end-to-end container reproducibility remains unproven on this machine until Docker runtime access is restored.

