# Edge Swarm Research Notes

## Research Novelty

- GPS-denied swarm autonomy with explicit edge runtime separation
- peer-shared obstacle and confidence digests instead of backend-heavy coordination
- safety-first split between local fast loops and backend supervisory loops

## Academic Contribution

- practical architecture for edge swarm autonomy in GPS-denied settings
- telemetry schema for confidence-aware peer coordination
- operator-facing truthfulness between simulation, playback, bench, and edge states

## Experimental Design

Study modes separately:

1. simulation
2. playback-assisted bench
3. live bench edge-swarm
4. hardware-in-the-loop mesh degradation

## Proposed Evaluation Metrics

- obstacle reaction latency
- peer deconfliction latency
- shared-awareness freshness
- stale-peer rejection accuracy
- backend load per drone
- bandwidth per peer
- mission continuity under uplink loss

## Benchmark Ideas

- centralized-only control path
- edge_swarm path with healthy mesh
- edge_swarm path with partial peer loss
- backend-disconnected edge_swarm path

## Comparison with Centralized UAV Systems

Centralized systems may simplify orchestration but are weaker under uplink degradation. Edge-swarm systems trade implementation complexity for lower latency and higher resilience.

## Future Work

- real UWB/TDOA edge synchronization tuning
- learned shared-obstacle compression
- cluster-level consensus optimization
- HIL and tethered validation
