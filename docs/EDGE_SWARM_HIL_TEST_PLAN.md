# Edge Swarm HIL Test Plan

This plan validates `edge_swarm` peer packet exchange before real multi-drone flight. Run with bench props removed, current-limited power, synchronized logs, and unique `DRONE_ID` values.

## 1. Two-Node Bench Packet Exchange

Start two nodes on the same V2X multicast group. Confirm both publish and receive `heartbeat`, `pose_state`, `edge_health`, `obstacle_digest`, and `consensus_state` packets. Expected result: each node reports `peer_count >= 1`, `stale_peer_count == 0`, normalized `source`, increasing sequence numbers, and non-zero mesh bandwidth.

## 2. Three-Node Stale-Peer Test

Run three nodes, then stop one node without shutting down the others. Expected result: remaining nodes mark that peer stale after the configured timeout, exclude it from `safety_eligible_peer_count`, expose split/isolation state if all known peers are stale, and remove the entry after expiry.

## 3. Backend Disconnected Test

Run nodes in `edge_swarm` with the control-plane backend unavailable. Expected result: telemetry shows `disconnected_operation=true`, autonomy remains local/distributed, and peer packet exchange continues without backend dependency.

## 4. Emergency Corridor Reservation Test

Inject or trigger a local emergency fault on one node. Expected result: the node broadcasts `emergency_corridor`; peers accept it even during degraded/disconnected state; local collision avoidance and emergency landing remain active without waiting for consensus.

## 5. Degraded Mesh Bandwidth Test

Throttle or impair the mesh link while keeping at least two nodes alive. Expected result: `mesh_bandwidth_kbps` drops, edge health moves toward degraded where appropriate, packets over the size limit are rejected, and stale peers are not used for safety-critical decisions.

## 6. Consensus Recovery Test

Create a quorum proposal such as `collective_halt` or `emergency_reroute`, then isolate and restore one peer. Expected result: quorum is lost when the peer is stale, collective action is not applied under local safety override, and consensus state recovers after fresh packets arrive in a newer epoch.

## Pass Criteria

All packet validation failures must be logged with reasons, dashboard/backend fields must reflect real peer state, and no safety action may depend on consensus when immediate collision avoidance or emergency landing is required.
