# Phase 4 Lock Order Policy

Date: 2026-07-16

## Goal

Prevent future deadlocks by documenting a single lock acquisition order for the project hotspots reviewed in Phase 4.

## Global Rules

- Prefer one mutex at a time.
- Never call external code, network I/O, or callbacks while holding more than one project mutex.
- If a snapshot can be copied under lock and processed later, copy first and release the lock.
- If a new path needs more than one lock, follow the order below exactly.

## Canonical Order

For native runtime code, acquire in this order when multiple locks are unavoidable:

1. `src/main.cpp` local command/telemetry coordination mutexes
2. `VIOPipeline::queue_mutex_`
3. `VIOPipeline::runtime_mutex_`
4. `VIOPipeline::visual_metrics_mutex_`
5. `V2XMeshNetwork::peers_mutex_`
6. `V2XMeshNetwork::local_edge_state_mutex_`
7. `V2XMeshNetwork::edge_serialization_metrics_mutex_`
8. `V2XMeshNetwork::health_mutex_`
9. `KeyframeManager::map_mutex_`
10. sensor instance `data_mutex_`
11. sensor instance `status_mutex_`
12. sensor instance `cb_mutex_`
13. telemetry client `state_mutex_`

## Go Backend Rule

- In `internal/controlplane/state.go`, take `sync.RWMutex` before any narrower per-feature mutex if a future nested path is introduced.
- Avoid holding Go state locks across HTTP writes.

## Current Assessment

- The inspected code usually acquires only one mutex at a time.
- The main risk was lack of documentation, not a reproduced deadlock.

## Enforcement Guidance

- Keep callback invocation outside locks.
- Keep broadcast/send paths outside map/state locks where possible.
- If a code review introduces nested locking, cite this file and document the reason in the change.
