# Edge Failsafe Strategy

## Safety Position

This strategy describes intended behavior for edge autonomy. It must not be treated as real-flight validation evidence.

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
