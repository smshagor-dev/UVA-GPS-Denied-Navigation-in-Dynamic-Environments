# Peer-packet security: reviewer-facing implementation evidence

This document records what can be reproduced in the current software and what still requires a separate experiment or protocol implementation. It is not a flight-test or hardware-security report.

## Receiver acceptance

For the legacy group-HMAC or opt-in per-node HMAC edge-swarm profile, a packet must satisfy all of the following:

- The packet parses as a complete, bounded CBOR message with representable integer fields and a positive TTL no greater than the receiver-configured `max_packet_ttl_ms` (default 5000 ms).
- Its sender timestamp is no more than `max_clock_skew_ms` (default 2000 ms) **ahead** of receiver time. The TTL does not increase this future bound.
- Its sender timestamp is no more than `ttl_ms + max_clock_skew_ms` **behind** receiver time. Subtraction-based comparisons avoid unsigned timestamp addition overflow.
- Its trust epoch exactly matches the locally configured trust epoch.
- Its canonical CBOR payload and packet metadata verify against a full HMAC-SHA256 tag using the configured group secret **or** the exact claimed sender's provisioned node key, depending on the selected mode; there is no cross-mode fallback.
- Its sequence number is not a duplicate and falls within a 64-sequence sliding replay window for the claimed peer (or advances the highest accepted sequence number).

In symbols, with receiver time `T`, sender time `t`, skew budget `delta`, packet lifetime `ttl`, and configured maximum lifetime `ttl_max`:

```text
0 < ttl <= ttl_max
-delta <= T - t <= ttl + delta
epoch == receiver_epoch
valid HMAC(configured group key OR claimed sender's node key,
           canonical_cbor(packet with cleared auth_hook))
sequence is new and within the 64-sequence replay window, or advances its watermark
```

All gates must pass. HMAC verification now precedes reporting a valid-looking packet as a replay. An unauthenticated packet with an old sequence should not be treated as verified replay evidence. The receiver reports distinct `invalid_ttl`, `future_timestamp`, `stale_timestamp`, `invalid_signature`, `stale_epoch`, and `replay_detected` outcomes. A packet that fails authentication does not reach the state cache or peer actions.

## Timestamp and clock assumptions

On-wire sender timestamps and the receiver's timestamp comparison now use `std::chrono::system_clock`, not separate nodes' `steady_clock` epochs. **This makes timestamps comparable in principle; it does not synchronize clocks.** Correct acceptance depends on the participating UAV operating-system clocks being synchronized within the configured skew bound. UTC clock synchronization, its source of trust, deliberate clock tampering, loss of synchronization, and time adjustments during flight have not been experimentally validated. The local peer-liveness timer continues to use `steady_clock`. A production deployment needs an authenticated or otherwise trusted time-synchronization strategy with drift monitoring and defined fail-closed behavior when the skew assumption fails.

## Experimental coverage from software tests

Relevant cases in `tests/test_edge_swarm.cpp` cover:

- HMAC-SHA256 known-answer digest and successful signing/verification;
- authenticated payload modification and sender-identifier tampering;
- acceptance and rejection at the *future* skew boundary, regardless of sender TTL;
- rejection of zero and oversized TTL, stale timestamp and zero-skew boundary;
- acceptance at the cache's TTL-plus-skew boundary, rejection beyond it, and persistence of the in-memory high-watermark across short peer expiry;
- verified duplicate/too-old rejection and acceptance of previously unseen reordered packets within the 64-sequence sliding window;
- verifying authenticity before classifying a packet as a replay;
- deterministic packet-delivery traces with adjacent reordering, sparse reordering, and synthetic packet loss; outputs include delivered count, sliding-window false rejection count, and observed out-of-order arrival rate; these **are not** radio-channel measurements;
- CBOR 64-bit header values that would otherwise narrow to 32 bits;
- non-wrapping timestamp expiry arithmetic;
- a **counterexample** to per-node attribution: a group-key holder can sign another sender ID;
- missing signatures, simulation profile exception and unsupported post-quantum modes.

`PeerPacketAuth.InProcessComponentLatencySamples` collects 512 in-process samples of CBOR serialization, the **HMAC primitive**, full packet signing, and full packet verification separately. It reports nanosecond p50/p95/p99 and sample count but enforces no machine-dependent speed threshold. Its results must be reported with CPU, OS, compiler, compiler flags, power state and toolchain, and treated as **software latency** until representative airborne hardware is measured. Benchmark logging does not constitute a demonstrated flight-control deadline or guaranteed real-time performance.

Reproduce the regression tests and collect the in-process measurements using a native CMake test build (the exact executable path is build-system dependent):

```sh
ctest --test-dir build --output-on-failure -R test_edge_swarm
./build/tests/test_edge_swarm --gtest_filter=PeerPacketAuth.InProcessComponentLatencySamples
```

The second command is an example and may require using the executable produced by your build preset. Archive raw stdout and machine metadata, not only summary tables.

## Claims not established by this change

1. **Group secret and node capture.** Legacy `hmac_sha256` remains supported and uses one shared group secret. The opt-in `hmac_sha256_per_node` mode accepts only sender-specific MACs from provisioned peer IDs and signs only with the matching local ID. A peer holding just its own signing secret cannot impersonate another peer whose key remains secret. **However, each verifying receiver holds other peers' symmetric keys**; compromising that receiver or its key database can therefore enable impersonation of those peers. This is not asymmetric physical-node identity, distributed enrollment, authenticated key provisioning, hardware-backed key isolation or revocation. A trust epoch integer does not distribute new secrets.
2. **Receiver restart and cache eviction.** Replay-window state can now be written to an opt-in append-only journal before a packet is admitted and recovered on process restart. If the journal is configured, missing, malformed or unwriteable state fails closed; cache clearing or expiry does not remove journal history. The default remains memory-only, and therefore restart/eviction replay risks remain when durable storage is not explicitly configured. The journal is **not** tamper-resistant: deletion, offline truncation, rollback, replacement, concurrent writers, or a compromised storage account are outside its security guarantees. No authenticated reboot handshake or trusted hardware counter is established.
3. **Reordering.** The receiver now uses a 64-sequence in-memory sliding bitmap and admits authenticated, unseen out-of-order packets within that bound. Duplicates and packets 64 or more positions behind the watermark remain rejected. An accepted reordered packet is delivered to downstream consumers, but does not overwrite the latest cached peer state. Out-of-window legitimate reordering can still be rejected, and the in-memory window is not durable across restart or eviction.
4. **Key and epoch changes.** No synchronized distributed group-rekey handshake, quorum transition, rollback protection, or seamless 32-bit rollover protocol is implemented.
5. **Radio and flight experiments.** No physical FANET, RF packet-loss/reordering distribution, multi-UAV flight, GNSS-denied clock-sync characterization, or hardware side-channel measurement is established by repository software tests. The benchmark does not establish timing or power side-channel resistance.
6. **Authentication scope.** The changed admission gate applies to the edge peer packet path; it should not be generalized to every message or control channel without separately checking their call sites.

## How to use this evidence in the revised manuscript

- Update the timestamp predicate in Methods to the asymmetric condition above and disclose the trusted-clock/skew assumption.
- Describe CBOR field-length encoding, integer-overflow checks, authenticated bytes and validation order explicitly.
- Separate `INVALID_SIGNATURE` from the policy-based `REPLAY_DETECTED` classification in telemetry and threat discussion.
- Include a concrete packet-reordering example: if authenticated sequence 11 arrives before sequence 10, the later distinct sequence 10 is admitted when it remains in the 64-sequence window; a repeated sequence 10 is rejected. Avoid labeling out-of-window rejection proof of hostile intent.
- Describe legacy group HMAC as group-key possession. Distinguish opt-in sender-specific HMAC from asymmetric or hardware-rooted physical-node identity; receivers with all symmetric peer keys still have signing capability.
- Present the in-process component benchmark only after running it on documented hardware; do not reuse process-launch timing as primitive HMAC latency.
- Distinguish an explicitly configured durable journal from the default memory-only receiver, and preserve journal tampering, clock synchronization, per-node keys, rekey and hardware testing as limitations.
- Pin all reported results to one exact Git commit and archive corresponding source, inputs, configurations, test outputs, and reproducibility instructions in a DOI-assigning repository.

This evidence separates implemented mechanisms, software-observed behavior, assumptions, and proposed extensions. It does not replace the author's point-by-point response to all editor and reviewer comments.

## Optional durable replay journal

Set `PacketAuthConfig::replay_ledger_path` to a private, per-UAV local file path **before packet reception**. The parent directory must already exist, must be protected against access by untrusted local users, and must have only one receiving process writing to the journal. Set `initialize_new_replay_ledger = true` only during explicit first-time provisioning when that path does not exist; disable it for all subsequent starts. A missing file after provisioning must fail closed rather than start an empty replay history.

Each accepted peer-packet admission appends its sender ID, trust epoch, highest sequence and 64-bit replay bitmap, then flushes and synchronizes the file before the receiver updates state or dispatches the packet. Startup rejects malformed records and unterminated writes. Recovery reconstructs the last replay bitmap for each sender. Epochs are still externally configured and do **not** imply real key distribution.

This journal assumes a trusted local filesystem. It has no authenticated rollback proof against privileged disk modification, deletion followed by reprovisioning, or a copied older but syntactically valid journal. It does not implement file locking across processes or journal compaction. The file grows for every admitted packet; disk synchronization can add significant latency and flash wear. It is not yet tested for system power loss, filesystem durability, storage exhaustion, malicious file edits, or actual UAV-hardware timing. Do not state certified restart security in a paper on the basis of these software tests.

## Opt-in sender-specific HMAC

Select `AuthMode::PER_NODE_HMAC_SHA256` (text mode `hmac_sha256_per_node`). Each sending UAV must configure `PacketAuthConfig::local_node_id` equal to its actual V2X local ID and `local_signing_secret` with at least 32 bytes of independently generated secret material. Each receiver must provision `trusted_node_secrets` entries keyed by trusted sender ID, containing the corresponding 32-byte-or-longer secrets. **Do not use one secret for multiple UAVs.** Provision keys out of band and protect the receiver's complete key database.

The on-wire tag is `hmac_sha256_node:<64 hex chars>`, distinct from the legacy `hmac_sha256:` tag. Packet `sender_id`, epoch, sequence, timestamps, type and payload remain authenticated as part of canonical CBOR. Unknown IDs, insufficient keys, mismatched signatures, missing tags, and legacy group-HMAC tags in this mode fail closed; no shared-secret/environment fallback is permitted. An HMAC from sender A with its own key cannot validate as sender B unless B's secret is compromised or both were incorrectly provisioned with the same key.

This is **symmetric-key sender separation**, not Ed25519 signatures or a guarantee against receiver compromise. Any recipient with sender B's verification secret can also generate B's HMAC; the implementation lacks key attestation, secure provisioning, authenticated rotation, per-recipient key derivation, physical-UAV identity verification and compromise-resistant storage. Keep the original legacy group-key spoofing counterexample and report which mode each experiment used.
