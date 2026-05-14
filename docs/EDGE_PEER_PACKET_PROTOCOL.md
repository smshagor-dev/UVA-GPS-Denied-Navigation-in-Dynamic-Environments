# Edge Peer Packet Protocol

The `edge_swarm` peer protocol carries compact drone-to-drone state over the existing V2X mesh as `SwarmMessage::EDGE_PACKET`. The implementation now supports a JSON debug path and a first production-oriented CBOR binary path through the same `EdgePeerProtocol` validation surface. JSON remains useful for bench visibility and replay inspection; CBOR is the preferred low-latency edge runtime prototype.

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
| `auth_hook` | Signature/authentication placeholder for the current implementation. Current values are `swarm-security-hook` or `unsigned`. |
| `payload` | Type-specific packet data. |

The default packet size limit is 1400 bytes so peer packets fit a conservative UDP/mesh MTU. Validation can use the actual encoded wire size, so CBOR packets are bounded by their binary size rather than by their JSON-equivalent debug rendering. `V2XMeshNetwork` still enforces the larger transport payload limit before send.

## Serialization Modes

Supported modes:

| Mode | Current status | Intended use |
| --- | --- | --- |
| `json` | Implemented | Default debug/development transport, readable logs, bench inspection. |
| `cbor` | Implemented prototype | Preferred edge runtime binary transport for compact peer packets. |
| `protobuf_placeholder` | Reserved | Future schema-driven transport; requests currently fall back to JSON compatibility until implemented. |

The runtime mode can be selected through `DRONE_EDGE_SERIALIZATION_MODE` or `--edge-serialization=<mode>`. The example protocol configuration exposes:

```json
{
  "serialization": {
    "serialization_mode": "cbor"
  }
}
```

The CBOR frame is a deterministic fixed-shape array:

```text
[
  version,
  packet_type,
  sender_id,
  timestamp_ms,
  sequence_number,
  trust_epoch,
  source,
  ttl_ms,
  auth_hook,
  payload_array
]
```

Payload arrays are fixed by packet type. Unknown packet types, unsupported CBOR versions, truncated CBOR data, indefinite-length fields, oversized strings, trailing bytes, and payload shape mismatches are rejected. This keeps binary decoding deterministic and avoids schema ambiguity during safety gating.

Current bench sample sizes from the unit-test benchmark are illustrative local software measurements, not RF throughput claims:

| Packet | JSON bytes | CBOR bytes | CBOR / JSON |
| --- | ---: | ---: | ---: |
| `heartbeat` | 370 | 97 | 0.26 |
| `obstacle_digest` | 278 | 58 | 0.21 |
| `consensus_state` | 315 | 57 | 0.18 |

Serialization metrics are captured in runtime telemetry:

- `edge_serialization_mode`
- `edge_average_packet_size_bytes`
- `edge_bandwidth_savings_estimate_pct`
- `edge_packet_encode_latency_us`

The bandwidth-savings field is an estimate against the JSON-equivalent packet size; it is not a measured production radio bandwidth result.

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

Malformed JSON or CBOR packets, missing required envelope fields, missing required payloads, expired packets, unknown packet types, zero sender ids, stale sequence numbers, oversized packets, non-finite pose vectors, and invalid emergency corridor radii are rejected.

Stale peers are excluded from safety-critical peer eligibility and consensus support. Consensus never gates local collision avoidance or emergency landing; local safety and emergency handling always win.

## Post-Quantum Peer Authentication

This section is a proposed security roadmap for `edge_swarm`; it is not an operational claim. The current `auth_hook` field is a placeholder. Today, packet validation focuses on structure, source normalization, TTL expiry, sequence freshness, stale-peer exclusion, and safety gating. Cryptographic signing of edge peer packets is not yet production-implemented in this protocol layer.

Future peer authentication should move from `auth_hook` to explicit signed metadata:

```text
packet_hash
signature_algorithm
signature
key_id
nonce
trust_epoch
```

The proposed direction is a hybrid classical and post-quantum transition path:

- classical signatures or mTLS remain available for near-term interoperability
- ML-KEM, the NIST-standardized successor name for CRYSTALS-Kyber, is used for post-quantum key establishment between peers
- ML-DSA, derived from CRYSTALS-Dilithium, is used for post-quantum packet signatures when library maturity and embedded performance allow it
- hybrid signatures bind both a classical signature and a post-quantum signature during migration

### Intended Packet Security Flow

1. Packet creation: construct the edge packet envelope and payload without the signature field.
2. Hash generation: compute `packet_hash = H(canonical_packet_bytes)`.
3. Signature generation: sign `packet_hash || sender_id || sequence_number || nonce || trust_epoch`.
4. Peer verification: verify sender identity, key id, signature algorithm, and signature bytes.
5. Trust epoch validation: reject stale epochs, quarantine revoked epochs, and degrade near-mismatch epochs according to the confidence model.
6. Replay nonce validation: reject duplicate `(sender_id, trust_epoch, nonce)` or stale `sequence_number`.
7. Safety gating: only then update peer cache, consensus, and dashboard-visible telemetry.

### Peer Packet Verification

```text
procedure VerifyPeerPacket(packet, peer_keyring, replay_cache, local_trust_epoch):
    if not ValidateEnvelope(packet):
        return reject("invalid envelope")

    if IsExpired(packet.timestamp_ms, packet.ttl_ms):
        return reject("expired packet")

    if replay_cache.contains(packet.sender_id, packet.trust_epoch, packet.nonce):
        return reject("replay nonce")

    key = peer_keyring.lookup(packet.sender_id, packet.key_id)
    if key is missing or key.revoked:
        return quarantine("unknown or revoked peer key")

    canonical = CanonicalizeWithoutSignature(packet)
    packet_hash = Hash(canonical)

    if not VerifySignature(key, packet.signature_algorithm, packet.signature, packet_hash):
        return quarantine("signature verification failed")

    if IsStaleTrustEpoch(packet.trust_epoch, local_trust_epoch):
        return quarantine("stale trust epoch")

    replay_cache.insert(packet.sender_id, packet.trust_epoch, packet.nonce)
    return accept(packet)
```

### Security Model Notes

Node compromise should be isolated by peer identity and trust epoch. A compromised or stale peer is removed from safety-critical consensus, its observations are excluded from local safety decisions, and its key id can be revoked without halting local emergency behavior. Epoch rotation gives the swarm a lightweight way to invalidate old packets after security transitions, backend command trust changes, or replay suspicion.

### Performance Notes

Post-quantum primitives generally increase signature size, verification cost, and packet overhead compared with classical elliptic-curve signatures. For low-bandwidth mesh radios, the initial deployment path should prefer hybrid signing for high-priority packets and rate-limited digests, then benchmark ML-DSA verification latency on the target onboard CPU. ML-KEM can be used for session establishment while per-packet signatures remain selective until HIL timing confirms the budget.
