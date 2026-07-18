# Phase 7 Mission Scenario Report

Date: July 16, 2026
Primary artifact: [`docs/research-validation-software-hil-scenarios/mission_results.json`](./mission_results.json)

## Run Metadata

- Experiment ID: `phase7-20260716T191134Z`
- Git revision: `58daa36decd2f6a9e846acc0c60a09e863c88b4d`
- Backend mode: `production`
- Simulation enabled: `false`
- Stale timeout: `2` seconds
- Scenario count: `5`
- Pass count: `5`
- Fail count: `0`

## Aggregate Mission Timing

| Metric | Value |
|---|---:|
| Backend startup | 538.300 ms |
| Average | 2.022 ms |
| p50 | 1.181 ms |
| p95 | 5.665 ms |
| p99 | 5.665 ms |
| Max | 5.665 ms |

## Scenario Results

### 1. Formation Maintenance

- Result: PASS
- Cluster: `phase7-formation`
- Leader: `7101`
- Drones preserved in cluster: `3`
- Average latency: `1.103 ms`

### 2. GPS-Denied Navigation

- Result: PASS
- Drone: `7201`
- Localization source: `vio-tdoa-fused`
- Localization confidence: `0.94`
- Visible anchors: `4`
- Average latency: `0.729 ms`

### 3. Communication Loss Recovery

- Result: PASS
- Drone: `7301`
- Stale telemetry detected after timeout: yes
- Recovery readiness status: `ready`
- Recovery reason: `control-plane operational`
- Average latency: `5.665 ms`

### 4. Obstacle Avoidance Degradation

- Result: PASS
- Drone: `7401`
- Safety state: `DEGRADED_LOCALIZATION`
- Localization confidence: `0.41`
- Local obstacle count: `8`
- Shared obstacle count: `11`
- Average latency: `1.433 ms`

### 5. Emergency Landing Behavior

- Result: PASS
- Drone: `7501`
- Safety state: `EMERGENCY_LAND`
- Security state: `LAND_IMMEDIATELY`
- Readiness status after event: `ready`
- Average latency: `1.181 ms`

## Notes

- The emergency scenario intentionally accepts `remote_command_allowed=null` in the backend echo because the Go response omits `false` booleans from JSON snapshots.
- These scenarios validate software-state propagation and observability, not actual vehicle movement.

