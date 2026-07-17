# Deployment Architecture Review

## Purpose

This review captures the production-facing deployment shape of the current repository as of July 16, 2026 and defines the intended Phase 5 packaging boundary.

## Current deployable components

- `control-plane`:
  - implemented in Go
  - serves HTTP APIs for telemetry, fleet state, health, approvals, events, discovery, readiness, and metrics
- `drone_node`:
  - implemented in C++
  - hardware-facing runtime for onboard sensing, autonomy, and telemetry uplink
- `dashboard`:
  - implemented in Python / PySide6
  - operator UI for bench and control-room workflows
- `main.py`:
  - local multi-process launcher for non-container orchestration

## Production packaging decision

Phase 5 containerization packages the Go control plane as the primary containerized runtime unit.

Reasons:

- It has the cleanest dependency boundary.
- It already exposes a network API suitable for health, readiness, and metrics checks.
- The C++ drone node remains hardware-coupled and should not be misrepresented as universally container-ready.
- The PySide6 dashboard is operator-workstation software, not a headless deployment service.

## Supported deployment topologies

### Topology A: Local simulation service

- Compose-managed control plane
- simulation mode enabled
- demo fleet seeding enabled
- intended for demos, API validation, and research workflows

### Topology B: Bench / lab control-plane service

- Compose-managed or system-managed control plane
- simulation disabled
- telemetry expected from native bench nodes
- readiness remains false until live or replay telemetry arrives

### Topology C: Edge or field control-plane host

- native or containerized control plane
- TLS and client-certificate settings injected by environment
- logs persisted outside the container
- certificates mounted read-only

## Secret and configuration model

- Runtime configuration remains environment-driven for the control plane.
- TLS assets are expected as mounted files.
- Shared secrets and operator credentials stay out of tracked config JSON.
- JSON config schemas under `config/schema/` document tracked artifact shape, but do not replace secret management.

## Persistence and observability boundaries

- container logs and `logs/control-plane/` are the supported local persistence surface
- `/api/v1/health` is a liveness-style operational summary
- `/api/v1/ready` is a deployment-readiness signal
- `/metrics` exposes lightweight text metrics for scraping or smoke tests

## Known limitations

- The container image does not package the dashboard.
- The container image does not package hardware-specific native drone-node execution.
- The current metrics surface is intentionally lightweight and not a full Prometheus integration layer.
- Jetson deployment is still documented primarily through native build/deployment guidance in [DEPLOYMENT.md](../../DEPLOYMENT.md).

