# Peer-packet security review: formal claims and evidence requirements

This document is an implementation-grounded revision aid for the manuscript
*Sequence-, Epoch-, and Integrity-Aware Detection of Replay and Peer-Spoofing
Attacks in Decentralized UAV Mesh Networks: A Software-Validated Security
Framework*. It addresses Reviewer 2's novelty/formal-analysis criticism and
the related requests by Reviewers 1 and 3. It **does not** constitute an
independently peer-reviewed cryptographic proof or a real-UAV measurement.

## Research contribution and appropriate scope

HMAC-SHA256, authenticated signatures, timestamp windows, epoch tags and replay
bitmaps are established mechanisms. Do not describe their combination as a new
cryptographic primitive, a new unforgeable MAC, or a formal security theorem.
The defensible contribution is a reproducible **receiver-side composition and
validation analysis**, with an implementation-to-acceptance-predicate map,
explicit attacker models, negative tests and clearly separated operational
failure modes. A claim of practical advantage over baselines requires matched,
measured comparator results, not merely the existence of a software test.

The manuscript originally analyzes a group-HMAC receiver with strict sequence
monotonicity. **Do not silently substitute the changed implementation**:
the current development line has a 64-position sliding replay bitmap, optional
disk journal, an opt-in symmetric per-sender mode, and an opt-in Monocypher
EdDSA/BLAKE2b public-key mode. For each reported result, specify the immutable
Git commit, the selected authentication mode, the replay storage configuration,
and the implementation used. The merged PR #54 targeted another feature branch;
its revocation controls must not be described as available in `main` until
the integration PR has actually merged.

## Acceptance predicate and threat models

For an input frame `p`, define the receiver decision (in the configured
security profile) as

```text
Accept(p, state, cfg, T)
  = ParseOK(p)
  AND TTLBound(p)
  AND TimestampFresh(p, T, cfg.skew)
  AND EpochMatches(p, cfg.epoch)
  AND SenderPolicyOK(p, cfg)
  AND VerifyAuthenticator(p, cfg)
  AND ReplayWindowAdmits(p, state)
```

The checks are not mutually statistically independent. Parsing has to precede
field evaluation; protocol-level authentication must precede authoritative
replay classification; replay-state mutation must follow successful admission.
Profile exceptions must be stated explicitly. The effective replay-window
policy uses a per-sender watermark and a 64-bit bitmap. A unique reordered
packet within the window can be admitted without rolling back the cached
latest peer-state fields; duplicates and packets outside the window are rejected.

The timestamp predicate uses sender time `t`, receiver time `T`, claimed
lifetime `ttl` and allowed skew `delta`:

```text
0 < ttl <= ttl_max
t > T  =>  t - T <= delta
t <= T =>  T - t <= ttl + delta
```

This is a receiver policy **conditional on trusted, synchronized clocks**,
not a synchronization protocol for GPS-denied flight. Avoid unsigned overflow
in both implementation and algebra. The epoch equality check does not itself
rotate or distribute any cryptographic key.

Distinguish the following adversaries:

- **Outsider** can record, replay, delay, drop, reorder, inject and modify wire
  frames, but does not learn the honest sender's secret signing material.
- **Compromised member with a group HMAC key** can sign as any sender in
  group-HMAC mode. No per-node attribution advantage should be claimed.
- **Compromised sender with only its own per-node HMAC secret** can sign under
  its own identity, not another properly keyed sender, provided the other
  sender's secret and receiver trust mapping remain uncompromised.
- **Compromised HMAC receiver** holding every sender's verification secret can
  impersonate those senders. Per-node symmetric HMAC does not defend against
  this case.
- **Compromised public-key verifier** holding public keys only cannot derive
  honest private signing seeds *from those public keys under the assumed
  signature security*, but trust-list substitution, enrollment fraud and
  compromised private seeds are separate attacks. The embedded Monocypher
  EdDSA+BLAKE2b mode is **not** RFC 8032 Ed25519.
- **Storage or clock adversary** able to roll back the replay journal, replace
  trust config or manipulate receiver time violates the trusted-state/time
  assumptions. The current software tests do not establish resistance.

## What can and cannot be derived from cryptography

A well-defined outsider *fresh-forgery* game grants the adversary signing
oracle access to honest, uncompromised senders and a network injection channel.
The adversary wins only if the receiver admits an altered/new canonical packet
that honest senders did not sign for that claimed sender and context, excluding
plain delivery of an already signed frame. Assume unambiguous canonical CBOR,
correct key provisioning, no secret exposure, correct MAC/signature
implementation and fixed trust configuration.

Then admission implies a valid authenticator, so **under those assumptions**
a fresh-forgery win can be reduced to a successful underlying MAC or signature
forgery (plus any separately modeled implementation/encoding failure).
Write the bound schematically rather than inventing a concrete number:

```text
Pr[FreshForgeryWin]
   <= Adv_authenticity(sign/verify queries, security parameter)
      + Pr[assumption or implementation failure]
```

The extra failure term is not measured or necessarily independently
distributed. For group HMAC, this statement only applies to an outsider without
the group key; it fails for an insider who possesses it. For the per-node
symmetric mode, it also excludes receivers that store the target's secret.
For the public-key mode, cite a security analysis appropriate to the actual
Monocypher EdDSA+BLAKE2b algorithm; do not reuse Ed25519/SHA-512 claims.

The previously printed `q / 2^256` is only an illustrative ideal
uniform-tag guessing term, **not** a complete HMAC security proof or
hardware-side-channel bound. Claims about `Adv_HMAC(q)` require a specified
security notion, adversary queries, key-distribution model and reference.
A constant-time byte comparison alone does not establish timing or power
side-channel resistance for a deployed UAV processor.

The manuscript's Equation (18),

```text
P_success,out ≈ P(F_time) * P(F_epoch) * P(F_seq) * Adv_HMAC(q)
```

must **not** be reported as a measured success probability or an established
bound: these events share inputs, policy state and adversarial choice. An exact
probability chain, if a sampling experiment is fully defined, uses conditional
probabilities:

```text
Pr[A and B and C and D]
  = Pr[A] * Pr[B|A] * Pr[C|A,B] * Pr[D|A,B,C]
```

That identity alone does not provide numerical security assurance. For
outsider *fresh forgeries* all acceptance gates only decrease the set of
forgeries that get through; the MAC/signature reduction above is the
defensible high-level argument. For replay of valid signed packets there is
**no MAC forgery**; replay rejection is a separate state invariant, not a
consequence of `Adv_HMAC`.

## Replay, reordering and state invariants

The reproducible property for one sender, trusted epoch, uninterrupted
receiver state and one receiver-local serial admission order is:

1. The first authenticated, not previously admitted sequence in the active
   64-position window can be marked in the bitmap.
2. A duplicate of a marked sequence is rejected.
3. A sequence at least 64 below the current high watermark is rejected,
   whether malicious or benign.
4. A unique reordered sequence within 63 below the highest can be admitted
   without rolling back the latest presentation state.
5. An epoch mismatch is rejected, while deliberate epoch changes require
   provisioning and coordinated trust policy; equality alone does not rotate
   keys.

When an explicitly configured journal successfully saves the bitmap before
admission, a process restart can reconstruct receiver-local replay history.
This guarantee assumes the stored journal is intact, trusted and not rolled
back. Journal deletion, privileged file replacement, crash durability,
concurrent writers, flash wear and exhaustion require separate tests or
mitigations. In the default memory-only mode, no restart persistence claim
is supported. Sequence rollover within one epoch remains unsupported.

The invariant is not a radio availability result: rejected old packets may
be legitimate, and missing frames change the observed arrival ordering.
A real-world `p_R` cannot be inferred from deterministic unit-test traces.

## Receiver rejection evidence and interpretation

Report counters or traces separately for: malformed encoding, invalid TTL,
clock-skew/stale timestamp, epoch mismatch, untrusted/revoked sender, invalid
authenticator, duplicate sequence, out-of-window sequence, storage failure
and policy-disabled unsigned packets. A failed authenticator is evidence of
cryptographic invalidity in the configured verification context, **not**
conclusive attribution of a malicious UAV. Duplicate/out-of-window packets
are replay-policy rejections, not proof of hostile intent. If telemetry
collapses cache rejection into another code, disclose that gap and fix before
claiming separate operational classification.

A defensible worked example:

```text
Receiver expects epoch 1; sender 7 signed sequence 11 at T=1,000 ms.
Signature is verified, time/TTL pass, epoch matches; receiver marks seq 11.
Sender 7's previously signed seq 10 arrives next and is still within the
64-position bitmap: verify signature, admit once, mark bit 1.
The same signed seq 10 arrives again: signature and freshness may still pass,
but bit 1 is already marked; reject duplicate without changing peer state.
```

## Evidence required for reviewer response

| Reviewer issue | Existing code support | Required additional evidence |
| --- | --- | --- |
| R2.1 novelty | Runnable receiver-side composition, canonical encoding and negative tests | State novelty as compositional validation; add matched baseline comparison, not a new-crypto claim |
| R2.2 weak formal proof / Eq. 18 | Explicit admission predicate and software assertions | Replace independence product; define experiments and assumptions; have cryptographic reduction reviewed |
| R2.3 and R3.3 insider impersonation | Opt-in per-node HMAC and Monocypher public-key authentication | Measure key compromise cases; secure provisioning/revocation mechanism not yet established |
| R2.4, R3.1, R3.2, R3.4 practical performance | In-process component benchmark and synthetic reordering tests | Run measured hardware benchmark with p50/p95/p99, raw logs, compiler/hardware metadata; obtain real RF or reproducible network-emulation traces |
| R2.5 and R3.14 replay | Bounded bitmap; opt-in trusted-storage journal | Add power-loss, journal rollback/tampering, realistic reordering and continuous-operation testing |
| R2.6 and R3.9 timestamps | Corrected future-skew predicate and boundary tests | Document trusted clock source and clock-drift/outage experiment for GPS-denied deployment |
| R3.5 classification | Distinct packet auth results and cache reject reasons | End-to-end telemetry trace with benign reordering, spoofing and parser failures separately counted |
| R3.6, R3.7, R3.12 transitions | Equality check and local test scenarios | Distributed epoch/key rollover, 32-bit sequence rollover and network partition handling or explicit limitations |
| R3.8, R3.10, R3.13, R3.15 | Authenticated canonical packet and profile config | Annotated parser length proof/negative tests; side-channel limits; profile hardening audit; explain validation order |
| R3.11 physical identity | Discussion only | Verify and cite suggested PUF article; disclose lack of device-attested identity |
| R1 and R3.16–18 writing | Existing software behavior | Update abstract, figures, notation, PQC wording, and bandwidth calculation using an actual serialized packet example |

## Conditions for a publication-ready evidence package

Record immutable source commit, all active modes/keys as redacted configuration,
test data and seeds, compiler flags, OS, CPU, optional UAV/flight controller,
radio model, loss/delay/reordering profiles, raw traces, measurement units and
an auditable calculation script. Compare strict watermark and 64-position
bitmap on the **same input trace** and report true duplicate detection and
benign false rejection separately. Run baseline + implementation under
comparable resource constraints; archive a reproducible package and DOI.

Do not claim hardware-in-the-loop, physical flight, observed `p_R`,
cryptographic latency, a security proof, or distributed synchronized key
rotation until the corresponding actual evidence exists.
