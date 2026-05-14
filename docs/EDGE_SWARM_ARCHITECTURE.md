# Edge Computing in Swarm Intelligence for GPS-Denied Autonomous UAV Systems

## Abstract

This document presents an architecture-level formulation for `edge_swarm`, a GPS-denied UAV swarm mode based on distributed autonomy, bounded-latency coordination, confidence-aware fusion, and resilient mesh topology. The design shifts time-critical perception, safety, and peer awareness from backend-supervised control toward onboard edge-supervised autonomy. All claims are architectural unless explicitly supported by bench or HIL validation.

## Scope

This document defines a production-oriented edge-computing architecture for the current GPS-denied UAV swarm platform. It is an architecture and implementation-design artifact, not evidence of real-flight validation.

Current repository status remains:

- locally buildable
- telemetry-pipeline validated
- dashboard-ready
- bench-demo ready
- not yet flight-validated on real hardware

## Motivation

GPS-denied robotics requires autonomy that survives degraded links, uncertain localization, and partial peer visibility. A backend-heavy swarm can simplify orchestration, but it introduces round-trip latency and a central dependency. The proposed architecture is motivated by European robotics research priorities around dependable autonomy, degraded-link continuity, transparent safety boundaries, and scalable multi-agent coordination.

## Proposed Method

The proposed method separates the swarm into three coupled layers:

```text
local edge loop -> peer mesh loop -> supervisory backend loop
```

Local edge loops perform perception, state estimation, obstacle avoidance, and emergency decisions. Peer mesh loops exchange compact digests for distributed awareness and consensus hints. The backend remains supervisory for operator visibility, audit, and mission injection rather than a hard real-time control dependency.

```text
[Sensors] -> [VIO/EKF/Fusion] -> [Local Safety] -> [Actuation]
                 |                    ^
                 v                    |
          [Peer Digest Cache] <-> [Mesh Consensus Hints]
                 |
                 v
          [Backend Telemetry/Audit]
```

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

## Partition Rejoin and Conflict Resolution

This is a proposed resilience architecture, not a validated BFT implementation. The core split-swarm problem is that two disconnected partitions may each elect a different leader and continue local mission logic under different consensus epochs. Rejoining requires deterministic reconciliation rather than accepting whichever leader packet arrives first.

On reconnect, partitions should exchange and compare:

- latest consensus epoch
- current trust epoch and revocation state
- quorum health and safety-eligible peer count
- mission continuity state
- leader uptime and recent fault score

Deterministic leader preference:

1. highest valid trust epoch
2. highest quorum health
3. longest stable uptime
4. lowest fault score
5. deterministic node ID fallback

Byzantine fault-tolerant concepts:

- reject malicious, unsigned, stale, or safety-ineligible peers from merge votes
- require quorum validation before accepting a remote partition leader
- isolate epoch mismatches until trust state converges
- use consensus timeouts to fall back to local-only degraded operation
- keep emergency corridor and emergency descent packets independent from consensus

```text
procedure PartitionMerge(local, remote):
    local_peers = FilterFreshEligible(local.peers)
    remote_peers = FilterFreshEligible(remote.peers)

    if TrustEpochMismatch(local, remote):
        return isolate_partition("epoch mismatch")
    if not HasValidQuorum(remote_peers):
        return local_only("remote quorum invalid")

    leader = DeterministicLeaderChoice(
        candidates = [local.leader, remote.leader],
        priority = [
            trust_epoch,
            quorum_health,
            stable_uptime,
            inverse_fault_score,
            node_id_fallback
        ])

    consensus_epoch = max(local.consensus_epoch, remote.consensus_epoch) + 1
    mission_state = ReconcileMissionContinuity(local.mission_state, remote.mission_state)
    return merged_partition(leader, consensus_epoch, mission_state)
```

Complexity: stale-peer filtering is `O(n)` for `n` visible peers. Leader reconciliation is `O(n)` with a single-pass priority comparison, or `O(n log n)` if candidates are sorted. Consensus merge is `O(e)` for bounded epoch/vote records. The peer cache remains bounded, so merge work should be predictable under HIL timing tests.

Safety rule: local collision avoidance and emergency descent MUST bypass consensus. Partition recovery may coordinate leader continuity and mission resumption, but it must not block local safety loops.

## Bandwidth Optimization

- share summaries, not raw camera or LiDAR payloads
- publish obstacle digests and semantic detections
- adaptive rate control based on mission phase and link integrity
- burst only on hazard, consensus, or health transitions

## Adaptive Edge Serialization Strategy

Current implementation status:

- JSON is implemented for development and bench/debug readability.
- CBOR is implemented as the first production-oriented binary serialization prototype for peer packets.
- JSON is not the preferred long-term runtime transport for production `edge_swarm`.
- No production protobuf or FlatBuffers peer-packet implementation is claimed by this document.

Preferred serialization roadmap:

1. CBOR for the current binary runtime prototype because it is compact, deterministic, and close to the JSON data model.
2. Protocol Buffers for future production peer packets once schemas are explicit, compact, versionable, and compatibility-tested.
3. FlatBuffers as optional future exploration where zero-copy reads or memory-mapped packet inspection become useful.

JSON is valuable while packet models are still changing, but it is inefficient for real-time mesh operation. Field names are repeated in every packet, numeric values are carried as text, parsers allocate and scan more heavily, and packet size rises quickly as peer count grows. That wastes edge bandwidth and CPU time, increasing mesh congestion and packet jitter exactly where GPS-denied swarm coordination needs predictable timing.

Estimated serialization comparison:

| Format | Packet size | CPU overhead | Parsing latency | Edge suitability |
|---|---:|---:|---:|---|
| JSON | highest | highest | highest and allocation-prone | best for bench/debug, weak for production loops |
| CBOR | low to medium | low to medium | low | implemented prototype for constrained binary digests |
| protobuf | low | low | low and schema-driven | reserved production roadmap item |

Current local benchmark samples from unit tests:

| Packet | JSON bytes | CBOR bytes | CBOR / JSON |
|---|---:|---:|---:|
| `heartbeat` | 370 | 97 | 0.26 |
| `obstacle_digest` | 278 | 58 | 0.21 |
| `consensus_state` | 315 | 57 | 0.18 |

These values are controlled software observations, not production RF mesh throughput measurements.

Swarm bandwidth can be approximated as:

```text
Bandwidth_total ~= N * packet_size * update_rate
```

where `N` is the number of transmitting peers, `packet_size` is the serialized packet size, and `update_rate` is packets per second. In all-to-all or multi-hop forwarding patterns, the effective load can grow faster because relays retransmit selected packets. Compact digests matter because reducing `packet_size` directly improves scalability, lowers channel occupancy, and leaves room for emergency bursts.

Adaptive-rate policy:

- emergency corridor, peer_goodbye, threat, and collective halt packets are high priority and may burst immediately
- heartbeat, pose, and health packets run at bounded periodic rates
- background health, backend summaries, and noncritical AI digests are reduced first under degraded mesh quality
- hazard windows temporarily allow bursts, followed by cooldown to prevent mesh collapse

Architecture note: real-time safety-critical swarm loops should prefer binary serialization over human-readable transport. JSON remains appropriate for bench inspection, dashboards, logging, and debug replay; CBOR is now the first runtime binary prototype. Protobuf remains a future migration path after HIL timing validation and schema compatibility testing.

## Latency Reduction Analysis

Expected gains come from removing repeated backend round-trips from:

- collision avoidance
- obstacle classification
- peer deconfliction
- local emergency action

Backend remains supervisory, not on the critical path for fast reaction.

The expected end-to-end reaction latency can be modeled as:

```text
L_edge = L_sensor + L_inference + L_fusion + L_local_policy
L_centralized = L_edge + L_uplink + L_backend + L_downlink
```

The proposed architecture reduces dependence on `L_uplink`, `L_backend`, and `L_downlink` for safety-adjacent loops. This is a design estimate and must be verified through HIL timing.

## Complexity Analysis

Let `n` be cached peers, `m` be digests per peer, `k` be local perception observations, and `e` be bounded consensus records.

- local inference: `O(k)` for one frame or scan batch, excluding model-specific neural inference cost
- peer cache merge: `O(n * m)` for bounded digest ingestion
- obstacle digest reconciliation: `O(n * m)` with direct merge, or `O(n * m * log k)` with indexed map checks
- consensus update: `O(e)` for bounded epoch/vote history
- mesh synchronization: `O(n)` for one-hop peer state exchange, with relay overhead increasing under multi-hop forwarding

Bounded cache sizes and rate-limited packet classes are required to keep this complexity predictable.

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

## Post-Quantum Peer Authentication

Current implementation status:

- edge peer packets carry an `auth_hook` placeholder rather than a production cryptographic signature
- implemented validation covers packet structure, maximum size, TTL expiry, sequence freshness, source normalization, stale-peer exclusion, and safety gating
- cryptographic signing and post-quantum verification are proposed future work, not flight-validated or production-implemented behavior

Future `edge_swarm` security should support crypto-agile peer authentication with a hybrid classical and post-quantum path. The intended design is:

- use ML-KEM, derived from CRYSTALS-Kyber, for post-quantum peer key establishment
- use ML-DSA, derived from CRYSTALS-Dilithium, for post-quantum packet signatures
- retain classical signatures or mTLS during transition so the swarm can operate in mixed deployments
- bind `sender_id`, canonical packet hash, sequence number, nonce, and `trust_epoch` into every signed packet

The intended verification flow is:

1. validate packet envelope and payload shape
2. reject expired TTLs and stale sequence numbers
3. verify `(sender_id, key_id)` against the swarm keyring
4. recompute the canonical packet hash
5. verify hybrid classical plus PQC signature when configured
6. validate trust epoch freshness and revocation state
7. reject replayed nonces
8. update peer cache only after authentication and safety gating

```text
procedure PeerPacketVerification(packet, keyring, replay_cache):
    if not ValidatePacketShape(packet):
        return reject
    if IsExpired(packet) or IsStaleSequence(packet):
        return reject
    if replay_cache.contains(packet.sender_id, packet.trust_epoch, packet.nonce):
        return reject

    key = keyring.lookup(packet.sender_id, packet.key_id)
    if key is missing or key.revoked:
        return quarantine

    digest = Hash(CanonicalizeWithoutSignature(packet))
    if not VerifyConfiguredSignatureSuite(key, packet.signature, digest):
        return quarantine
    if not TrustEpochAccepted(packet.trust_epoch):
        return quarantine

    replay_cache.insert(packet.sender_id, packet.trust_epoch, packet.nonce)
    return accept
```

Security model implications:

- swarm node compromise is contained by key revocation, trust epoch rotation, and stale-peer quarantine
- stale peers remain visible to operators but are excluded from safety-critical consensus
- trust revocation should advance the local epoch and invalidate old signed packets
- epoch rotation should be explicit after replay suspicion, command-auth failures, or backend trust transitions

Performance implications:

- PQC signatures and keys are larger than common classical signatures, so mesh bandwidth and MTU pressure must be measured
- hybrid signing is preferred at first because it preserves classical interoperability while adding PQC migration evidence
- high-rate heartbeat traffic may need lightweight authenticated session channels, while emergency, consensus, and trust-transition packets receive full signatures
- HIL tests must measure verification latency before enabling PQC signatures on safety-adjacent peer flows

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

## Limitations

- no real flight validation is claimed
- no full hardware swarm validation is claimed
- no production radio validation is claimed
- CBOR peer packets are implemented as a software prototype, but not yet validated on production radios
- PQC signing and full BFT recovery are proposed architecture, not implemented operational guarantees

## Future Work

- HIL timing validation with representative radios, Jetson-class compute, LiDAR, and camera loads
- HIL validation of CBOR peer transport and future protobuf schema migration
- bounded BFT-inspired consensus implementation for small UAV swarms
- flight-adjacent tethered validation after bench HIL success
