# Simulation Runtime Validation

Date: 2026-07-16

## Commands

```powershell
docker compose -f docker-compose.simulation.yml config
docker compose -f docker-compose.simulation.yml up -d --build
GET /api/v1/ready
GET /api/v1/fleet
GET /metrics
docker restart <container>
GET /api/v1/ready
GET /api/v1/fleet
docker compose -f docker-compose.simulation.yml down
```

## Result

Status: PASS

## Observed behavior

- simulation compose configuration expansion succeeded
- simulation container started successfully
- control plane reported ready immediately in simulation mode
- seeded simulation fleet was available through `/api/v1/fleet`
- metrics endpoint responded successfully
- restart/recovery test passed:
  - container restarted successfully
  - readiness returned to HTTP `200`
  - seeded fleet remained available after restart

## Raw evidence

- [simulation_compose_config.txt](./runtime_artifacts/simulation_compose_config.txt)
- [simulation_compose_up.out](./runtime_artifacts/simulation_compose_up.out)
- [simulation_compose_up.err](./runtime_artifacts/simulation_compose_up.err)
- [simulation_compose_ps.txt](./runtime_artifacts/simulation_compose_ps.txt)
- [simulation_ready_before_restart.json](./runtime_artifacts/simulation_ready_before_restart.json)
- [simulation_fleet_before_restart.json](./runtime_artifacts/simulation_fleet_before_restart.json)
- [simulation_metrics_before_restart.json](./runtime_artifacts/simulation_metrics_before_restart.json)
- [simulation_restart.txt](./runtime_artifacts/simulation_restart.txt)
- [simulation_ready_after_restart.json](./runtime_artifacts/simulation_ready_after_restart.json)
- [simulation_fleet_after_restart.json](./runtime_artifacts/simulation_fleet_after_restart.json)
- [simulation_container_logs_after.txt](./runtime_artifacts/simulation_container_logs_after.txt)

