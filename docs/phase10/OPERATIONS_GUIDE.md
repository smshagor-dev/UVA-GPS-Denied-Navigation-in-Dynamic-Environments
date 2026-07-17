# Phase 10 Operations Guide

Date: July 17, 2026

## Core Endpoints

- `GET /api/v1/health`
- `GET /api/v1/ready`
- `GET /metrics`
- `GET /api/v1/fleet`
- `POST /api/v1/telemetry`

## Routine Checks

1. Confirm health endpoint returns `200`.
2. Confirm readiness reflects expected telemetry state.
3. Confirm metrics are scrapeable.
4. Confirm logs are being written under `logs/control-plane/`.
5. Confirm command submission policy matches the intended security profile.

## Restart Procedure

The Go control-plane handles `SIGINT` and `SIGTERM` with graceful shutdown. Compose production configuration also sets `restart: unless-stopped` and `stop_grace_period: 20s`.

## Known Boundary

This guide covers the backend control-plane service only. It does not certify field deployment, flight readiness, or hardware operations.

