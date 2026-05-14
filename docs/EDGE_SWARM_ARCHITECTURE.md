# Edge Computing in Swarm Intelligence for GPS-Denied Autonomous UAV Systems

## Scope

This document defines a production-oriented edge-computing architecture for the current GPS-denied UAV swarm platform. It is an architecture and implementation-design artifact, not evidence of real-flight validation.

Current repository status remains:

- locally buildable
- telemetry-pipeline validated
- dashboard-ready
- bench-demo ready
- not yet flight-validated on real hardware

## Centralized vs Edge Swarm Comparison

| Area | Centralized backend-heavy swarm | Edge swarm architecture |
|---|---|---|
| Decision latency | bounded by round-trip to control plane | bounded by onboard compute plus peer hop |
| Backend dependence | high | reduced to coordination, audit, and supervisory tasks |
| GPS-denied resilience | lower during degraded uplink | higher because local perception, fusion, and action remain onboard |
| Swarm continuity | backend outage can stall collective response | peer mesh can maintain local mission logic |
| Bandwidth profile | raw/high-rate uploads trend upward with swarm size | compressed local summaries and event-driven sharing |
| Failure blast radius | central node failure affects many drones | failure is partitioned to local peers or sub-swarms |

## Distributed Autonomy Benefits

- lower obstacle-reaction latency because LiDAR, VIO, TDOA/UWB, and mission arbitration stay onboard
- better resilience when backend connectivity is intermittent or absent
- improved containment of single-node or single-link failures
- more scalable swarm growth because awareness is shared as compact digests instead of full raw streams
- safer GPS-denied operation because navigation does not require centralized position arbitration

## Edge Processing Topology

Recommended topology:

- per-drone edge autonomy loop in C++20
- per-drone peer telemetry exchange over secure mesh
- local cluster leader only for coordination hints, not hard real-time control
- Go control plane retained for operator visibility, audit, mission injection, and post-mission analytics

Operational tiers:

1. Onboard edge tier: perception, fusion, safety, local autonomy, collision avoidance, peer cache.
2. Swarm mesh tier: peer summaries, shared obstacles, consensus voting, degraded-health propagation.
3. Supervisory tier: dashboard, fleet snapshots, command approval, telemetry archiving.

## Drone-to-Drone Decision Sharing

Each node shares compact decision-support messages rather than raw sensor streams:

- pose and velocity summary
- localization confidence
- obstacle digest
- consensus state and epoch
- edge health and autonomy mode
- disconnected-operation flag

Peers consume these messages as advisory context. Final actuation remains local.

## Local Sensor Fusion at the Edge

Fusion remains onboard and authoritative:

- VIO provides short-horizon motion estimate
- EKF fuses IMU, VIO, barometer, LiDAR-derived constraints, and TDOA/UWB
- peer state is fused as bounded-confidence context, not a replacement for local state estimation

## Distributed Obstacle Awareness

Each node maintains:

- local obstacle map from LiDAR and range sensors
- obstacle digest for peer sharing
- decayed shared obstacle cache tagged by timestamp and confidence

Peers exchange obstacle cells or obstacle tracks, not full point clouds.

## Edge AI Inference

Edge AI should prioritize:

- local obstacle classification
- local target detection/tracking
- threat cue extraction
- confidence-aware publication to peers

Inference outputs are shared as compact semantic objects with confidence, class, velocity estimate, and age.

## Mesh Communication Architecture

Recommended mesh properties:

- adaptive multi-hop peer mesh
- rate-limited broadcast for health and consensus
- event-driven high-priority packets for threat, obstacle, and emergency actions
- bounded caches with stale-peer rejection
- authenticated peer identities and anti-replay protection

## Edge Safety and Failsafe Concepts

- safety loop remains onboard and higher priority than mesh coordination
- local autonomy continues if backend is unavailable
- stale peer data is excluded from safety-critical consensus
- emergency landing can be locally initiated without backend approval
- split-swarm operation is allowed only in degraded mode with explicit operator visibility

## Fault Tolerance Strategy

- local mission logic tolerates backend loss
- peer cache tolerates partial mesh loss
- cluster-level consensus tolerates one or more peer drops depending on swarm size
- rejoin logic restores shared awareness after partition recovery

## Bandwidth Optimization

- share summaries, not raw camera or LiDAR payloads
- publish obstacle digests and semantic detections
- adaptive rate control based on mission phase and link integrity
- burst only on hazard, consensus, or health transitions

## Latency Reduction Analysis

Expected gains come from removing repeated backend round-trips from:

- collision avoidance
- obstacle classification
- peer deconfliction
- local emergency action

Backend remains supervisory, not on the critical path for fast reaction.

## GPS-Denied Operational Advantages

- no dependence on GPS or backend-centric navigation truth
- better tolerance of indoor, subterranean, urban-canyon, and RF-challenged spaces
- local UWB/TDOA and VIO remain usable when remote links degrade

## Security Considerations

- mTLS or equivalent authenticated peer identity
- signed peer packets with anti-replay nonce/epoch
- role-bounded collective actions
- stale-peer rejection before consensus participation
- audit of backend commands kept separate from local safety overrides

## Future Scalability

Scale by clustering:

- 5 to 20 drones per edge-consensus group
- inter-cluster summaries only
- backend supervises cluster aggregates rather than every raw stream

## Proposed `edge_swarm` Runtime Role

`edge_swarm` should be treated as:

- stricter than `bench`
- backend-supervised but locally autonomous
- live-sensor required
- no playback-only TDOA source
- no hidden simulation fallback for safety-critical decisions
