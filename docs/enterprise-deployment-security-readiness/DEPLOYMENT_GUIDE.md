# Phase 10 Deployment Guide

Date: July 17, 2026

## Scope

This guide covers the software deployment artifacts delivered in Phase 10. It does not claim live cloud deployment or production traffic cutover.

## Available Artifacts

- `deployment/docker/Dockerfile.production`
- `docker-compose.production.yml`
- `deployment/compose/.env.production.example`
- `deployment/compose/control-plane.secrets.env.template`
- `deployment/kubernetes/`

## Recommended Local Validation

```powershell
docker compose config
docker compose -f docker-compose.production.yml config
python deployment/scripts/phase10_validate_deployment.py
python deployment/scripts/phase10_validate_monitoring.py
```

## Deployment Notes

- production container runs as non-root
- read-only filesystem and `no-new-privileges` are configured in the production Compose stack
- health and readiness endpoints are available at `/api/v1/health` and `/api/v1/ready`
- TLS and operator/device security inputs must be provided through environment variables and mounted files

## Kubernetes Notes

Use `kubectl kustomize deployment/kubernetes` to render the manifest set locally before any cluster-specific rollout work.

