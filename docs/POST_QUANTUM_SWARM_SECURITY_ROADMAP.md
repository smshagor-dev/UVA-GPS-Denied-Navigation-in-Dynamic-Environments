# Post-Quantum Swarm Security Roadmap

Current status: packet authentication is implemented for bench/HIL with `hmac_sha256`. This is not post-quantum cryptography. The `pqc_hybrid_placeholder` mode is intentionally unsupported and returns a clear roadmap-only result.

## Implemented Now

- `hmac_sha256` signs canonical edge peer packets with a shared secret from `DRONE_SWARM_SECRET` or `config/swarm_edge_protocol.json`.
- The canonical packet excludes the signature field before hashing/signing.
- Verification binds `sender_id`, `sequence_number`, `timestamp_ms`, `trust_epoch`, `packet_type`, source, TTL, and payload bytes.
- Missing or invalid signatures reject peer packets before `SwarmStateCache`, consensus, or peer-driven action.
- Unsigned packets are allowed only in explicit simulation/debug configuration.

HMAC is useful for bench and HIL because it gives real tamper detection, replay resistance through sequence checks, and operational telemetry without requiring PQC libraries or large signature payloads during early mesh timing work.

## PQC Roadmap

- Signatures: migrate to ML-DSA, derived from Dilithium, for long-lifecycle packet authenticity.
- Key establishment: use ML-KEM, derived from Kyber, for swarm session/key provisioning.
- Hybrid transition: run classical authentication and PQC authentication together while nodes, tooling, and MTU budgets are upgraded.
- Trust epochs: rotate trust epochs after revocation, replay suspicion, command-auth failures, or key refresh events.
- Replay binding: bind nonce or monotonic sequence, sender id, trust epoch, packet type, timestamp, and canonical payload hash into every authenticated packet.
- Revocation: distribute signed node revocation records and refuse packets from revoked sender ids after epoch rotation.

## Verification Pseudocode

```text
function verify_peer_packet(packet, peer_state, auth_config):
    if packet.timestamp is stale:
        reject STALE_TIMESTAMP
    if packet.sequence <= peer_state.last_sequence:
        reject REPLAY_DETECTED
    if packet.trust_epoch != peer_state.trust_epoch:
        reject STALE_EPOCH
    canonical = canonicalize(packet without signature)
    if auth_config.mode == hmac_sha256:
        expected = HMAC_SHA256(secret, canonical)
        if !constant_time_equal(expected, packet.signature):
            reject INVALID_SIGNATURE
    if auth_config.mode == pqc_hybrid_placeholder:
        reject UNSUPPORTED_ROADMAP_MODE
    accept
```

## Threat Model

The auth layer targets spoofed peer packets, payload tampering, stale sequence replay, trust-epoch rollback, and unauthenticated emergency or consensus influence. It does not yet solve compromised node keys, RF jamming, side-channel leakage, or PQC signature latency on embedded targets.

## Limitations

`hmac_sha256` uses a shared secret, so compromise of one node can expose the swarm packet-auth secret. PQC signatures are not implemented in this build; the placeholder mode is present only to make future configuration and telemetry explicit without creating false security claims.
