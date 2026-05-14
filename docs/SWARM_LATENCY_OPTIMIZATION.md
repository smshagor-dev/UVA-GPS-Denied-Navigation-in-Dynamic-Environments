# Swarm Latency Optimization

## Objective

Reduce swarm reaction latency by removing backend dependence from time-critical loops while preserving supervisory backend visibility.

## Strategy

- keep obstacle detection and avoidance onboard
- keep local mission arbitration onboard
- share only compressed peer state
- use backend for audit, mission injection, and coarse coordination

## Degraded Network Behavior

When backend latency rises or backend is lost:

- continue local autonomy
- continue peer-awareness exchange
- reduce packet budgets to critical summaries
- mark disconnected operation explicitly in telemetry and dashboard

## Disconnected Swarm Operation

Allowed in `edge_swarm` mode when:

- local sensing remains healthy
- peer trust remains above threshold
- mission logic supports partition-safe behavior

## Local Fallback Decisions

- hold formation loosely instead of tightly
- widen separation margins
- prefer safe corridor over mission optimality
- land locally if confidence or propulsion health collapses

## Expected Timing Comparison

These are engineering estimates, not validated measurements.

| Path | Centralized estimate | Edge estimate |
|---|---:|---:|
| obstacle reaction | 120 to 220 ms | 30 to 70 ms |
| peer deconfliction | 90 to 180 ms | 25 to 60 ms |
| emergency collective halt propagation | 140 to 260 ms | 40 to 90 ms |

## Expected Swarm Reaction Improvement

- roughly 2x to 4x faster local reaction in backend-degraded conditions
- stronger continuity under intermittent uplink loss
- reduced coordination jitter for small tactical swarms

## Constraint

Real latency values still require hardware-in-the-loop and flight-adjacent validation.
