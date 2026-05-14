# Edge AI Workflow

## Abstract

This document formalizes the proposed edge AI workflow for `edge_swarm`: local inference, confidence-aware fusion, compact semantic digest sharing, and advisory peer integration. The workflow targets bounded-latency coordination under degraded-link continuity and remains architecture-level until HIL and flight-adjacent validation.

## Goal

Define how AI inference should run onboard and how its outputs should be shared across the swarm with bounded latency and bounded bandwidth.

## Motivation

Naive sharing of raw detections can increase bandwidth, amplify stale observations, and make peer claims appear more certain than they are. Edge AI in this architecture therefore uses confidence decay, trust epochs, stale-peer rejection, and local-sensor consistency to keep remote perception advisory rather than blindly authoritative.

## Proposed Method

The method is:

```text
local inference -> semantic digest -> confidence decay -> peer cache -> local consistency check -> advisory fusion
```

Only compact semantic digests are exchanged. Local safety remains the final authority for collision avoidance and emergency descent.

## Onboard Inference Pipeline

1. capture local frame or local fused perception slice
2. run lightweight onboard detector/tracker
3. classify obstacle or target
4. assign confidence and freshness
5. publish compact semantic digest locally and to peers

## Local Priorities

Highest priority inference tasks:

- obstacle detection near flight corridor
- obstacle classification for dynamic/static separation
- local target tracking
- threat cue generation

Lower priority tasks:

- rich scene labeling
- archive-quality metadata

## Edge Obstacle Classification

Recommended labels:

- static_structure
- movable_ground_object
- dynamic_air_risk
- teammate_risk
- unknown_obstacle

## Swarm-Level Shared Awareness

Share only:

- object id or digest id
- class
- relative location
- velocity estimate
- confidence
- age

Peers fuse this with local maps and discard stale or low-confidence entries.

## Local Target Tracking

Tracking remains locally authoritative.

Peer-shared target tracks are:

- advisory
- confidence-gated
- age-limited

## Dynamic Threat Sharing

Urgent threat messages should be event-driven and preempt lower-priority telemetry when:

- dynamic obstacle enters collision corridor
- hostile or unknown air object appears
- peer declares emergency route reservation

## AI Confidence Propagation

Confidence should decay with:

- time
- hop count
- disagreement with local sensing
- inconsistent peer confirmation

## Mathematical Confidence Propagation Model

This section is a proposed architecture-level formulation for `edge_swarm`; it is not flight-validated. Its purpose is to make peer-shared AI confidence explicit, bounded, and auditable before HIL and real multi-drone trials.

Let a peer observation or semantic digest be represented as:

```text
o_i = (class_i, pose_i, velocity_i, C_initial, timestamp_i, hop_count_i, trust_epoch_i, source_i)
```

The propagated confidence used by a receiving drone is:

```text
C_final =
    C_initial
    * exp(-lambda_t * Delta t)
    * exp(-lambda_h * hop_count)
    * T(epoch)
    * D(local_consistency)
    * S(stale_peer)
```

where:

```text
Delta t = t_now - timestamp_i
hop_count = number of peer-radio relay hops
lambda_t >= 0 = time decay constant
lambda_h >= 0 = hop-count decay constant
T(epoch) in [0, 1] = trust epoch factor
D(local_consistency) in [0, 1] = local-sensor agreement factor
S(stale_peer) in [0, 1] = stale-peer penalty
```

The time decay term models freshness loss:

```text
C_time = C_initial * exp(-lambda_t * Delta t)
```

The hop-count decay term models uncertainty introduced by relay depth:

```text
C_hop = C_time * exp(-lambda_h * hop_count)
```

The trust epoch factor is proposed as:

```text
T(epoch) =
    1.0,                         if trust_epoch_i == trust_epoch_local
    exp(-lambda_e * |Delta e|),  if 0 < |Delta e| <= e_max
    0.0,                         if |Delta e| > e_max or epoch is stale
```

where:

```text
Delta e = trust_epoch_i - trust_epoch_local
lambda_e >= 0 = trust epoch mismatch decay constant
e_max = maximum tolerated epoch mismatch
```

The local agreement factor penalizes disagreement between peer-shared perception and local sensing:

```text
D(local_consistency) = exp(-lambda_d * d_i)
```

where `d_i` is a normalized disagreement score. For obstacle or threat digests, `d_i` may combine class mismatch, position residual, velocity residual, and occupancy-grid disagreement:

```text
d_i = w_p * ||p_peer - p_local||_2
    + w_v * ||v_peer - v_local||_2
    + w_c * I[class_peer != class_local]
    + w_m * map_residual
```

with non-negative weights `w_p`, `w_v`, `w_c`, and `w_m`.

The stale-peer penalty is:

```text
S(stale_peer) =
    1.0, if peer is fresh and safety-eligible
    0.0, if peer is stale or safety-ineligible
```

For non-safety advisory displays, a softer stale penalty may be used:

```text
S_advisory(stale_peer) = exp(-lambda_s * stale_age)
```

Safety-critical decisions should use the hard exclusion rule.

### Trust Epoch Behavior

`trust_epoch` is an architecture-level security freshness marker. The intended behavior is:

- peers advance `trust_epoch` when security state changes, replay suspicion is detected, or authenticated command trust changes
- receivers reject observations from stale epochs outside the tolerated mismatch window
- receivers degrade confidence for small epoch mismatches instead of treating them as equally trustworthy
- consensus and shared perception should only use safety-eligible peers with current or near-current trust epochs

This prevents an old but syntactically valid digest from retaining full influence after a peer security transition.

### Why Confidence-Aware Propagation Is Safer

Naive shared awareness treats a peer detection as a fact once it appears on the mesh. In GPS-denied and degraded-radio operation, that is risky because observations can be old, relayed through several hops, issued under a stale trust epoch, or inconsistent with local sensing.

The proposed confidence-aware model makes uncertainty explicit. A peer digest can still improve situational awareness, but its influence decays as freshness, relay depth, trust alignment, and sensor agreement degrade. This preserves the core safety rule: local collision avoidance and emergency landing remain locally authoritative, while peer intelligence remains advisory unless it is fresh, trusted, and locally consistent.

### Confidence Fusion Procedure

```text
procedure FusePeerConfidence(local_state, peer_cache, local_observations):
    fused_digests = empty set

    for each peer in peer_cache:
        if peer is stale or safety-ineligible:
            continue

        for each digest in peer.active_digests:
            Delta t = now_ms - digest.timestamp_ms
            if Delta t > digest.ttl_ms:
                continue

            epoch_factor = TrustEpochFactor(digest.trust_epoch, local_state.trust_epoch)
            if epoch_factor == 0:
                continue

            disagreement = LocalDisagreement(digest, local_observations)
            confidence =
                digest.C_initial
                * exp(-lambda_t * Delta t)
                * exp(-lambda_h * digest.hop_count)
                * epoch_factor
                * exp(-lambda_d * disagreement)

            if confidence >= confidence_min:
                fused_digests.add(UpdateDigestConfidence(digest, confidence))

    return MergeWithLocalMap(fused_digests, local_observations)
```

### Computational Complexity

Let `n` be the number of bounded cached peers, `m` be the maximum number of active digests per peer, and `k` be the number of local observations used for agreement checks.

Local fusion against local observations is `O(k)` for a direct scan, or `O(log k)` per lookup if observations are indexed spatially. Peer confidence merge is `O(n * m * k)` with a direct local agreement scan, or `O(n * m * log k)` with a spatial index. The bounded peer cache keeps `n <= N_max`, so memory is `O(N_max * m)` and update cost remains bounded for HIL timing analysis.

## Safety Constraint

Low-confidence AI outputs must not directly trigger unsafe collective maneuvering without local safety confirmation.

## Limitations

- confidence equations are proposed design models, not flight-validated performance claims
- detector accuracy under thermal throttling, blur, darkness, or frame drop is not established here
- peer AI digests are advisory and cannot replace local sensing
- production binary semantic-digest transport is proposed but not yet implemented

## Future Work

- HIL evaluation of confidence decay under WiFi congestion and packet loss
- learned obstacle digest compression
- calibration of `lambda_t`, `lambda_h`, `lambda_d`, and epoch mismatch penalties
- comparison of JSON, CBOR, and protobuf semantic digest latency
