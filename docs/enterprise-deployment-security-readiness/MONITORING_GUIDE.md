# Phase 10 Monitoring Guide

Date: July 17, 2026

## Delivered Monitoring Assets

- `deployment/monitoring/prometheus.yml`
- `deployment/monitoring/grafana-control-plane-dashboard.json`

## Verified Runtime Surfaces

- `/api/v1/health`
- `/api/v1/ready`
- `/metrics`

## Exposed Metrics

- `drone_swarm_controlplane_ready`
- `drone_swarm_online_drones`
- `drone_swarm_total_drones`
- `drone_swarm_real_drones`
- `drone_swarm_stale_drones`
- `drone_swarm_critical_alerts`
- `drone_swarm_pending_approvals`
- `drone_swarm_event_log_entries`
- `drone_swarm_command_log_entries`

