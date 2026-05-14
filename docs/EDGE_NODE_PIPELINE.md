# Edge Node Pipeline

## Purpose

This document defines the onboard edge-processing pipeline for each UAV node in `edge_swarm` runtime mode. It is design intent, not validated flight behavior.

## Pipeline Overview

1. Sensor ingest
2. Local perception
3. Local fusion and state estimation
4. Local obstacle-map update
5. Local mission logic
6. Peer telemetry exchange
7. Distributed consensus assist
8. Local safety override and emergency decision

## Local Perception Node

Inputs:

- camera
- LiDAR
- IMU
- optical flow
- rangefinder

Outputs:

- local detections
- local obstacle cells
- target tracks
- inference confidence

## Local Obstacle Map

Recommended behavior:

- maintain short-horizon occupancy grid onboard
- timestamp every cell update
- merge peer obstacle digests with confidence decay
- reject peer cells older than stale-peer timeout

## Local Mission Logic

Responsibilities:

- execute mission state machine without backend round-trip
- consume local safety constraints first
- consume peer awareness second
- consume backend mission updates as supervisory overrides

## Local Swarm State Cache

Store bounded per-peer entries:

- last pose
- last health
- last consensus state
- last obstacle digest
- last trust epoch

Cache must be size-limited and age-limited.

## Peer-to-Peer Telemetry Exchange

Exchange only compact state:

- heartbeat
- pose/velocity/confidence
- edge-health summary
- consensus state
- obstacle digest
- threat digest

## Edge Collision Avoidance

Priority order:

1. local immediate obstacle avoidance
2. local peer separation enforcement
3. mission objective preservation
4. backend intent adherence

## Distributed Consensus

Use consensus for:

- leader continuity hints
- sector ownership
- collective halt or reroute requests
- emergency coordination intents

Do not require consensus for immediate collision avoidance or emergency descent.

## Local Emergency Decision System

Triggers:

- localization confidence collapse
- collision imminent
- propulsion fault
- lost peer quorum during tightly coupled formation
- backend unavailable plus local safety margin violation

Outputs:

- hold
- local reroute
- safe return by anchor reference
- emergency land

## Recommended Module Breakdown

- `EdgePerceptionNode`
- `EdgeObstacleMap`
- `LocalMissionLogic`
- `SwarmStateCache`
- `PeerTelemetryExchange`
- `EdgeConsensusManager`
- `EmergencyDecisionSystem`

## Runtime Separation

- `simulation`: synthetic/demo allowed
- `bench`: live-sensor oriented, replay may still support validation workflows
- `production`: live-sensor only
- `edge_swarm`: live-sensor plus local distributed autonomy; no playback-only positioning path
