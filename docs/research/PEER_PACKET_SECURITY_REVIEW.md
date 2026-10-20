# Peer-packet security: reviewer-facing implementation evidence

This document records what can be reproduced in the current software and what still requires a separate experiment or protocol implementation. It is not a flight-test or hardware-security report.

## Receiver acceptance

For the HMAC edge-swarm profile, a packet must satisfy all of the following:

- The packet parses as a complete, bounded CBOR message with representable integer fields and a positive TTL no greater than the receiver-configured `max_packet_ttl_ms` (default 5000 ms).
- Its sender timestamp is no more than `max_clock_skew_ms` (default 2000 ms) **ahead** of receiver time. The TTL does not increase this future bound.
- Its sender timestamp is no more than `ttl_ms + max_clock_skew_ms` **behind** receiver time. Subtraction-based comparisons avoid unsigned timestamp addition overflow.
- Its trust epoch exactly matches the locally configured trust epoch.
- Its canonical CBOR payload and packet metadata verify against a full HMAC-SHA256 tag under the configured group secret.
- Its sequence number exceeds the last accepted sequence number for the claimed peer.

In symbols, with receiver time `T`, sender time `t`, skew budget `delta`, packet lifetime `ttl`, and configured maximum lifetime `ttl_max`:

```text
0 < ttl <= ttl_max
-delta <= T - t <= ttl + delta
epoch == receiver_epoch
valid HMAC(group_key, canonical_cbor(packet with cleared auth_hook))
sequence > last_accepted_sequence_for_sender
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
- verified replay rejection and rejection of legitimate packets arriving out of sequence;
- verifying authenticity before classifying a packet as a replay;
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

1. **Group secret and node capture.** All members still share a signing secret. Anyone holding that secret can generate a valid HMAC with another sender identifier. There is no implemented per-node identity, key-provisioning or distributed revocation protocol. A trust epoch is an integer gate; incrementing it does not itself distribute a new secret.
2. **Receiver restart and cache eviction.** Last accepted sequences live in `SwarmStateCache` memory. No crash-atomic replay watermark checkpoint, persistent session binding, or authenticated reboot handshake exists in this change. Reusing the same secret and epoch after restart or removal of a stale cache entry may admit an old still-fresh signed packet.
3. **Reordering.** The current receiver deliberately rejects any authenticated sequence less than or equal to the highest accepted one. A sliding anti-replay bitmap in the manuscript is a *proposed* protocol, not implemented here. A benign out-of-order rejection must not be labeled proof of hostile replay.
4. **Key and epoch changes.** No synchronized distributed group-rekey handshake, quorum transition, rollback protection, or seamless 32-bit rollover protocol is implemented.
5. **Radio and flight experiments.** No physical FANET, RF packet-loss/reordering distribution, multi-UAV flight, GNSS-denied clock-sync characterization, or hardware side-channel measurement is established by repository software tests. The benchmark does not establish timing or power side-channel resistance.
6. **Authentication scope.** The changed admission gate applies to the edge peer packet path; it should not be generalized to every message or control channel without separately checking their call sites.

## How to use this evidence in the revised manuscript

- Update the timestamp predicate in Methods to the asymmetric condition above and disclose the trusted-clock/skew assumption.
- Describe CBOR field-length encoding, integer-overflow checks, authenticated bytes and validation order explicitly.
- Separate `INVALID_SIGNATURE` from the policy-based `REPLAY_DETECTED` classification in telemetry and threat discussion.
- Include a concrete packet-reordering example: if authenticated sequence 11 arrives before sequence 10, the later arrival of 10 is rejected without proving an attack.
- Describe group HMAC as group-key possession, not physical-node attribution.
- Present the in-process component benchmark only after running it on documented hardware; do not reuse process-launch timing as primitive HMAC latency.
- Preserve restart-state, reordering, clock synchronization, per-node keys, rekey and hardware testing as explicit limitations unless additional experiments and code are delivered.
- Pin all reported results to one exact Git commit and archive corresponding source, inputs, configurations, test outputs, and reproducibility instructions in a DOI-assigning repository.

This evidence separates implemented mechanisms, software-observed behavior, assumptions, and proposed extensions. It does not replace the author's point-by-point response to all editor and reviewer comments.
