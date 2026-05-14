# Edge Failsafe Strategy

## Abstract

This document presents a proposed failsafe strategy for `edge_swarm`, emphasizing local safety authority, degraded-link continuity, stale-peer rejection, and partition recovery. It is suitable as architecture-level research documentation and must not be read as real-flight validation evidence.

## Safety Position

This strategy describes intended behavior for edge autonomy. It must not be treated as real-flight validation evidence.

## Motivation

In distributed UAV autonomy, failures may arise from backend loss, peer isolation, localization degradation, propulsion faults, or split-swarm partitions. The failsafe strategy therefore prioritizes local collision avoidance and emergency descent over consensus-dependent coordination.

## Proposed Method

The proposed failsafe hierarchy is:

```text
local immediate safety
  > local degraded autonomy
  > fresh peer consensus hints
  > backend mission intent
```

Consensus may coordinate recovery, but it must not gate emergency descent or immediate obstacle avoidance.

## Backend Loss

If backend is lost:

- continue local autonomy
- keep peer mesh alive
- set `disconnected_operation=true`
- restrict mission behavior to safe local objectives

## Peer Isolation Handling

If a node loses peer quorum:

- fall back to local-only autonomy
- widen separation assumptions
- refuse consensus-based maneuver updates

## Stale-Peer Rejection

Reject peer data from safety-critical decisions when:

- heartbeat timeout exceeded
- trust epoch mismatched
- timestamp freshness exceeded

## Emergency Landing Coordination

If one node enters emergency landing:

- publish emergency corridor reservation
- nearby peers widen exclusion zone
- backend is informed when available but is not required for the landing trigger

## Local-Only Degraded Operation

Enter local-only degraded mode when:

- backend unavailable
- peer quorum lost
- localization confidence drops below cooperative threshold

## Split-Swarm Recovery

When swarm partitions heal:

- exchange latest consensus epoch
- refresh trust state
- reconcile obstacle digest caches
- rejoin via safe loose formation before tight formation

## Partition Rejoin and Conflict Resolution

This section describes a proposed resilience architecture for split-swarm healing. It is not a validated Byzantine fault-tolerant implementation.

Problem statement: during disconnection, two or more swarm partitions may independently elect different local leaders, advance different consensus epochs, and maintain different obstacle or mission-continuity states. When connectivity returns, accepting the first visible leader would be unsafe because the peer may be stale, malicious, operating under an old trust epoch, or representing a weak quorum.

When partitions reconnect, nodes should compare:

- consensus epoch
- trust epoch
- quorum health
- mission continuity state
- stale-peer and fault status

Deterministic leader preference should be evaluated in this order:

1. highest valid trust epoch
2. highest quorum health
3. longest stable uptime in the current epoch
4. lowest fault score
5. deterministic node ID fallback

Byzantine fault-tolerant concepts for this design:

- malicious or stale peers are rejected from safety-critical rejoin voting
- quorum validation requires fresh packets from safety-eligible peers
- epoch mismatch causes temporary isolation until the mismatch is reconciled or explicitly revoked
- consensus timeout handling falls back to local-only degraded autonomy instead of blocking safety action

Safety note: local collision avoidance and emergency descent MUST bypass consensus. Partition merge logic is advisory for formation, leader continuity, and shared mission state; it must never delay immediate local safety.

### Partition Merge Procedure

```text
procedure PartitionMerge(local_partition, remote_partition):
    local_candidates = FilterFreshSafetyEligible(local_partition.peers)
    remote_candidates = FilterFreshSafetyEligible(remote_partition.peers)

    if remote_candidates is empty:
        return keep_local_degraded("remote partition has no eligible peers")

    if TrustEpochConflict(local_partition, remote_partition):
        return isolate_until_epoch_reconciled()

    if not ValidateQuorum(remote_candidates):
        return keep_local_degraded("remote quorum invalid")

    leader_set = {local_partition.leader, remote_partition.leader}
    preferred = SelectLeader(
        leader_set,
        order = [
            highest_trust_epoch,
            highest_quorum_health,
            longest_stable_uptime,
            lowest_fault_score,
            deterministic_node_id
        ])

    merged_epoch = max(local_partition.consensus_epoch, remote_partition.consensus_epoch) + 1
    merged_mission = ReconcileMissionContinuity(local_partition, remote_partition)
    merged_obstacles = MergeFreshObstacleDigests(local_partition, remote_partition)

    return enter_loose_rejoin_formation(preferred, merged_epoch, merged_mission, merged_obstacles)
```

### Complexity Notes

Let `n` be visible peers across both partitions and `e` be exchanged epoch or vote records. Stale-peer filtering is `O(n)`. Peer reconciliation is `O(n log n)` if candidates are sorted by deterministic leader preference, or `O(n)` if the best candidate is selected by one scan. Consensus merge overhead is `O(e)` for bounded history comparison. Cache-bounded obstacle digest merge remains `O(n * m)` where `m` is the per-peer digest cap.

## Limitations

- no real flight validation is claimed
- no full hardware swarm validation is claimed
- no production radio validation is claimed
- BFT behavior is proposed resilience architecture, not a validated implementation

## Future Work

- HIL partition recovery timing under controlled packet loss
- tethered tests for emergency corridor reservation and local descent
- formal verification of safety invariants around consensus bypass
