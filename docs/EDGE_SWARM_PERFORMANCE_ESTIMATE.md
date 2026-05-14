# Edge Swarm Performance Estimate

## Statement

These are architecture-based estimates only. They are not flight-validated metrics.

## Estimated Improvements

- latency reduction: 2x to 4x on critical local reaction loops
- bandwidth savings: 35% to 65% by sharing digests instead of raw streams
- swarm reaction improvement: materially faster obstacle and peer-separation response
- backend load reduction: fewer high-rate dependencies on centralized processing
- scalability improvement: better small-cluster autonomy with less control-plane fan-out pressure
- resilience improvement: continued operation during uplink degradation
- GPS-denied benefit: stronger continuity when external positioning and uplink are both constrained

## Estimated Limits

- benefit depends on onboard compute headroom
- larger swarms still need cluster partitioning
- consensus overhead grows if every node talks to every node

## Readiness View

Software architecture readiness for edge phase: moderate to good.

Remaining blocker for true performance claims: hardware validation with representative radios, compute, and sensing loads.
