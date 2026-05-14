# Edge AI Workflow

## Goal

Define how AI inference should run onboard and how its outputs should be shared across the swarm with bounded latency and bounded bandwidth.

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

## Safety Constraint

Low-confidence AI outputs must not directly trigger unsafe collective maneuvering without local safety confirmation.
