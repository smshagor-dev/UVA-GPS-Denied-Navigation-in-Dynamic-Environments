# Observability Runbook

## Endpoints

### Health

- route: `/api/v1/health`
- purpose: liveness-style fleet health summary
- expected use: smoke tests, dashboards, service checks

### Readiness

- route: `/api/v1/ready`
- purpose: determine whether the control plane is operational for its configured mode
- behavior:
  - simulation mode reports ready after startup
  - non-simulation mode reports not ready until telemetry is ingested
  - all-stale telemetry reports not ready

### Metrics

- route: `/metrics`
- purpose: lightweight scrapeable operational counters
- exposed counters/gauges include:
  - readiness
  - online drones
  - total drones
  - real drones
  - stale drones
  - critical alerts
  - pending approvals
  - event log entries
  - command log entries

## Logs

Current logging is plain-text and emitted by the Go standard logger and the Python launcher logger.

Operational expectations:

- persist `logs/control-plane/` and `logs/launcher/`
- treat request logs as deployment evidence, not as a complete audit platform
- use Phase 4 and Phase 5 validation artifacts for release evidence capture

## Suggested checks

```powershell
curl http://127.0.0.1:8080/api/v1/health
curl http://127.0.0.1:8080/api/v1/ready
curl http://127.0.0.1:8080/metrics
```

