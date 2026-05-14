# Edge Peer Packet Protocol

The `edge_swarm` peer protocol carries compact drone-to-drone state over the existing V2X mesh as `SwarmMessage::EDGE_PACKET`. The first implementation uses JSON for readable bench validation, with the C++ API isolated behind `EdgePeerProtocol` so the wire body can move to CBOR or a fixed binary frame later.

## Envelope

Every packet has the same envelope:

| Field | Purpose |
| --- | --- |
| `packet_type` | One of the supported edge packet types. Unknown values are rejected. |
| `sender_id` | Non-zero drone/node id. |
| `timestamp_ms` | Sender monotonic timestamp in milliseconds. |
| `sequence_number` | Per-sender monotonic packet sequence. Stale or duplicate sequence numbers are rejected. |
| `trust_epoch` | Security/runtime trust epoch associated with the sender state. |
| `source` | Normalized to `real`, `playback`, `simulation`, or `unavailable`. |
| `ttl_ms` | Packet time-to-live. Expired or zero-TTL packets are rejected. |
| `auth_hook` | Signature/authentication placeholder. Current values are `swarm-security-hook` or `unsigned`. |
| `payload` | Type-specific packet data. |

The default packet size limit is 1400 bytes so JSON packets fit a conservative UDP/mesh MTU. `V2XMeshNetwork` still enforces the larger transport payload limit before send.

## Packet Types

`heartbeat` publishes peer liveness, peer counts, battery, motor health, link quality, emergency fault, edge health status, and autonomy state.

`pose_state` publishes position, velocity, and localization confidence.

`edge_health` publishes edge health status, autonomy state, consensus state, mesh bandwidth estimate, and disconnected-operation state.

`obstacle_digest` publishes local/shared obstacle counts, digest freshness, and a digest id.

`threat_digest` publishes threat level, summary, and confidence.

`consensus_state` publishes proposal type, consensus epoch, quorum count, consensus state, and whether local safety is overriding collective action.

`emergency_corridor` publishes an immediate reserved corridor center, radius, hold TTL, and summary. This packet is accepted independently of consensus state.

`peer_goodbye` marks planned shutdown or isolation and makes split-swarm state visible.

## Validation Rules

Malformed JSON-like packets, missing required envelope fields, missing required payloads, expired packets, unknown packet types, zero sender ids, stale sequence numbers, oversized packets, non-finite pose vectors, and invalid emergency corridor radii are rejected.

Stale peers are excluded from safety-critical peer eligibility and consensus support. Consensus never gates local collision avoidance or emergency landing; local safety and emergency handling always win.
