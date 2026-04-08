# System Designer and Developer: Md Shahanur Islam Shagor
# Project: UVA GPS Denied Navigation in Dynamic Environments
# Technology: C++, Python, Go, CMake

# Go Architecture for 1000+ Drone Swarms

This document defines a practical scale-out architecture for the current project when moving from a single-drone lab system to a 1000+ drone fleet.

## 1. Design Principle

Keep hard real-time and sensor-heavy logic on the drone in C++. Move fleet-scale coordination, routing, aggregation, and operator-facing orchestration to Go services.

Use this split:

- `C++ onboard`: EKF, VIO, sensor fusion, local autonomy, collision avoidance, emergency safety.
- `Go backend`: telemetry ingest, mission control, swarm grouping, health monitoring, command dispatch, dashboard API.
- `PySide6 dashboard`: operator console for monitoring and control.

## 2. Recommended System Split

### Onboard per drone

Keep these modules in C++:

- `EKFEstimator`
- `VIOPipeline`
- `DecisionEngine`
- `V2XMeshNetwork`
- sensor drivers
- local failsafes

Reason:

- deterministic timing
- low-latency math
- direct hardware access
- safer behavior when cloud/backend is unavailable

### Ground / edge / cloud

Build these in Go:

- `telemetry-gateway`
- `command-dispatcher`
- `swarm-coordinator`
- `fleet-state-service`
- `mission-service`
- `alert-service`
- `dashboard-api`
- `auth/audit-service`

Reason:

- massive concurrent I/O
- simple scaling with goroutines
- strong support for gRPC, WebSocket, NATS, Redis, Kafka
- lower ops complexity for distributed services

## 3. Service Map

### 3.1 telemetry-gateway

Responsibility:

- receive telemetry from all drones
- validate and normalize messages
- fan out to message bus
- apply rate-limits and backpressure

Protocols:

- Drone to gateway: `gRPC streaming` or `QUIC`
- Gateway to internal services: `NATS JetStream` or `Kafka`

Why Go:

- 1000+ persistent streams
- efficient connection handling
- easy batching and routing

### 3.2 fleet-state-service

Responsibility:

- maintain latest state of every drone
- track online/offline/leader/follower role
- expose fast query API for dashboard
- keep short trajectory history

Storage:

- hot state: `Redis`
- historical telemetry: `ClickHouse` or `TimescaleDB`

Why Go:

- fast in-memory aggregation
- strong fit for cache + API services

### 3.3 command-dispatcher

Responsibility:

- accept operator/system commands
- send targeted or broadcast mission commands
- retry, ack tracking, timeout handling
- command audit logging

Commands:

- election
- formation change
- waypoint push
- return home
- emergency land

Why Go:

- high fan-out command delivery
- concurrency control
- delivery guarantees

### 3.4 swarm-coordinator

Responsibility:

- split 1000+ drones into sectors/groups
- assign leader sets
- formation templates per cluster
- deconflict missions between groups

Important:

- do not run per-frame autonomy here
- do run fleet-level decisions here

Examples:

- make 1000 drones into 50 swarms of 20
- assign 1 leader + 2 backups per swarm
- move groups to different map regions

### 3.5 mission-service

Responsibility:

- mission planning
- mission state machine
- objective assignment
- geofence and no-fly zone policies

Why Go:

- business logic and orchestration fit Go well

### 3.6 alert-service

Responsibility:

- low battery alerts
- missing heartbeat alerts
- thermal overload alerts
- collision-risk or drift-risk alerts

Rules examples:

- battery below 15%
- no telemetry for 2 seconds
- CPU temp above 82 C
- drift above mission threshold

### 3.7 dashboard-api

Responsibility:

- single backend for the PySide6 dashboard
- aggregate fleet summaries
- stream updates by WebSocket or gRPC
- avoid UI polling every drone individually

This is critical:

- dashboard must never connect to 1000 drones directly
- dashboard should connect only to the Go backend

## 4. Transport Recommendation

### Best practical stack

- Drone to edge: `gRPC streaming`
- Internal bus: `NATS JetStream`
- Dashboard live stream: `WebSocket` or `gRPC`
- Command path: `gRPC + NATS`

### If you need very high telemetry firehose

Use:

- Drone to edge: `QUIC` or `gRPC`
- Internal analytics: `Kafka`

### If you need simpler operations first

Use:

- `gRPC`
- `Redis`
- `NATS`
- `PostgreSQL`

This is usually the best first production setup.

## 5. 1000+ Drone Scaling Pattern

Do not treat 1000 drones as one flat swarm.

Instead:

- partition fleet into regions
- create swarm clusters
- assign one leader per cluster
- assign one backup leader
- aggregate telemetry by cluster
- push mission updates at cluster level when possible

Recommended scale unit:

- `1 cluster = 10 to 30 drones`

For 1000 drones:

- about `35 to 80` clusters depending on mission type

This reduces:

- command storms
- election storms
- dashboard rendering overload
- network congestion

## 6. Dashboard Changes Needed

For 1000+ drones, the current Python dashboard should not pull directly from the pybind bridge for fleet-wide data.

Recommended dashboard model:

- local single-drone debug mode: current pybind11 path
- fleet mode: connect to `dashboard-api` in Go

UI behavior:

- top-level fleet overview
- cluster overview
- selected-cluster detail
- selected-drone deep inspection

Do not render 1000 detailed cards at once.

Render like this:

- fleet KPIs
- cluster map
- selected cluster table
- selected drone telemetry panel

## 7. Data Model Recommendation

### Drone telemetry message

Fields:

- drone_id
- cluster_id
- timestamp
- position_xyz
- velocity_xyz
- attitude
- battery_pct
- cpu_temp_c
- gpu_load_pct
- drift_m
- role
- connectivity
- health_flags
- mission_state

### Command envelope

Fields:

- command_id
- target_scope
- target_ids
- cluster_id
- command_type
- payload
- priority
- issued_by
- issued_at
- deadline

### Fleet state aggregate

Fields:

- total_drones
- online_drones
- active_clusters
- leaders_online
- avg_battery
- avg_drift
- max_temp
- critical_alert_count

## 8. Suggested Repo Expansion

Add a Go workspace beside the current C++ and Python code.

Recommended structure:

```text
drone_swarm/
â”œâ”€â”€ cmd/
â”‚   â”œâ”€â”€ telemetry-gateway/
â”‚   â”œâ”€â”€ command-dispatcher/
â”‚   â”œâ”€â”€ swarm-coordinator/
â”‚   â”œâ”€â”€ dashboard-api/
â”‚   â””â”€â”€ mission-service/
â”œâ”€â”€ internal/
â”‚   â”œâ”€â”€ bus/
â”‚   â”œâ”€â”€ fleetstate/
â”‚   â”œâ”€â”€ telemetry/
â”‚   â”œâ”€â”€ commands/
â”‚   â”œâ”€â”€ missions/
â”‚   â”œâ”€â”€ auth/
â”‚   â””â”€â”€ alerts/
â”œâ”€â”€ proto/
â”‚   â”œâ”€â”€ telemetry.proto
â”‚   â”œâ”€â”€ commands.proto
â”‚   â”œâ”€â”€ fleet.proto
â”‚   â””â”€â”€ mission.proto
â””â”€â”€ deployments/
    â”œâ”€â”€ docker-compose/
    â””â”€â”€ k8s/
```

Keep current code as:

```text
include/  -> onboard C++
src/      -> onboard C++
gui/      -> operator desktop UI
```

## 9. Migration Plan

### Phase 1

- keep current C++ onboard path
- add `dashboard-api` in Go
- dashboard reads fleet data from Go instead of direct drone polling

### Phase 2

- add `telemetry-gateway`
- add `fleet-state-service`
- send drone telemetry to Go backend

### Phase 3

- add `command-dispatcher`
- move formation/election/mission dispatch to Go control plane

### Phase 4

- add clustering and coordinator logic
- support 1000+ drones by region and cluster

### Phase 5

- add alerting, audit, auth, replay, mission history

## 10. Where Go Should Not Replace C++

Do not move these to Go:

- EKF propagation
- VIO estimation
- LiDAR point cloud processing
- per-frame YOLO execution
- local collision avoidance
- emergency stop safety loop
- direct actuator or flight-control timing logic

## 11. Final Recommendation

For this project, the strongest architecture is:

- `C++` on the drone
- `Go` in the control plane
- `PySide6` as the operator console

If you want to scale beyond 1000 drones, this split is much safer and easier to operate than trying to make the desktop app or onboard C++ process own the entire fleet lifecycle.
# System Designer and Developer: Md Shahanur Islam Shagor
# Project: UVA GPS Denied Navigation in Dynamic Environments
# Technology: C++, Python, Go, CMake
# System Designer and Developer: Md Shahanur Islam Shagor
# Project: UVA GPS Denied Navigation in Dynamic Environments
# Technology: C++, Python, Go, CMake
