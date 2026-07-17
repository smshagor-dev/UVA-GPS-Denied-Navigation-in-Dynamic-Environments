# Production Runtime Smoke Report

Date: 2026-07-16

## Commands

```powershell
docker compose -f docker-compose.yml config
docker compose -f docker-compose.yml up -d --build
GET /api/v1/health
GET /api/v1/ready
POST /api/v1/telemetry
GET /api/v1/ready
GET /metrics
docker compose -f docker-compose.yml down
```

## Result

Status: PASS

## Observed behavior

- compose configuration expansion succeeded
- container build and startup succeeded
- health checks passed
- container logs showed normal control-plane startup and request handling
- production readiness behaved correctly:
  - before telemetry ingest: HTTP `503`, `waiting_for_telemetry`
  - after telemetry ingest: HTTP `200`, `ready=true`
- metrics endpoint responded successfully
- compose shutdown completed successfully

## Host-port note

The host machine already had ports `8080` and `18080` occupied by unrelated local services, so the validated smoke run used an automatically selected free host port:

- validated host port: see [production_host_port.txt](./runtime_artifacts/production_host_port.txt)

This was a validation-environment adaptation only; the compose file itself was not redesigned.

## Raw evidence

- [production_compose_config.txt](./runtime_artifacts/production_compose_config.txt)
- [production_compose_up.out](./runtime_artifacts/production_compose_up.out)
- [production_compose_up.err](./runtime_artifacts/production_compose_up.err)
- [production_compose_ps.txt](./runtime_artifacts/production_compose_ps.txt)
- [production_container_logs_before.txt](./runtime_artifacts/production_container_logs_before.txt)
- [production_container_logs_after.txt](./runtime_artifacts/production_container_logs_after.txt)
- [production_health_before.json](./runtime_artifacts/production_health_before.json)
- [production_ready_before.json](./runtime_artifacts/production_ready_before.json)
- [production_telemetry_post.json](./runtime_artifacts/production_telemetry_post.json)
- [production_ready_after.json](./runtime_artifacts/production_ready_after.json)
- [production_metrics.json](./runtime_artifacts/production_metrics.json)
- [production_compose_down.out](./runtime_artifacts/production_compose_down.out)

