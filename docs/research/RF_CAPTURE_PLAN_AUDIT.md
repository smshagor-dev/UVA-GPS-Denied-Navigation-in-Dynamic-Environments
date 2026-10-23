# Auditing RF captures against a declared TX packet plan

This audit addresses a specific reviewer-risk: a sender log can omit records
without being noticed when a receiver-side CSV is used to infer packet-loss
rates. A planned transmission schedule, recorded **separately from both raw
logs**, lets investigators test one narrower claim: whether the supplied TX
CSV includes every packet identity specified by the plan.

The tool DOES NOT prove that the plan existed before the flight, that a radio
actually transmitted those packets, or that the supplied logs came from an
authentic field experiment. It never upgrades a synthetic experiment to a
measured FANET result.

## Source acquisition and provenance

The dataset sources considered so far are:

- AERPAW 2025 LoRaWAN gateway/helikite dataset:
  https://doi.org/10.5061/dryad.w0vt4b939
- AERPAW AADM real-flight reception logs:
  https://doi.org/10.5061/dryad.7d7wm3898
- Point-to-point LoRa CAN telemetry, published 2026:
  https://doi.org/10.17632/jm58sggtj7.2

The latter dataset reports packet-level measurements over configured links,
but those experiments concern vehicle CAN telemetry rather than a multi-UAV
FANET. The AERPAW files have not been successfully downloaded and examined
in this work. None of these references establishes that we possess a
complete, independent sender TX inventory. Their published loss indicators
must not be substituted for missing independent sender capture records.

## Trial packet plan

A plan should be created and archived **before** data collection, with a
dated protocol version, expected sender IDs, epoch and sequence ranges,
experiment duration, topology and channel conditions. Keep independently
collected radio/gateway provenance and TX logger configuration.

For each run, prepare independently recorded raw TX and RX CSV files with:

- TX: `sender_id,epoch,sequence,tx_ms`
- RX: `sender_id,epoch,sequence,rx_ms`

Then calculate SHA-256 hashes from the **original** files. Do not reuse
normalized exports or receiver rows as your independent TX source.

A one-run plan JSON can have this structure (all values are placeholders):

```json
{
  "flight_id": "REPLACE_WITH_REAL_FLIGHT",
  "source_doi": "doi:REPLACE_WITH_VERIFIED_SOURCE",
  "protocol_reference": "REPLACE_WITH_PRETRIAL_ARCHIVED_PROTOCOL",
  "epoch": 1,
  "tx": {
    "file": "sender-raw.csv",
    "sha256": "REPLACE_WITH_ACTUAL_64_HEX_SHA256"
  },
  "rx": {
    "file": "gateway-raw.csv",
    "sha256": "REPLACE_WITH_ACTUAL_64_HEX_SHA256"
  },
  "planned_senders": [
    {"sender_id": 7, "sequence_first": 1, "sequence_last": 240},
    {"sender_id": 8, "sequence_first": 1, "sequence_last": 240}
  ]
}
```

Use:

```sh
python3 scripts/rf_capture_plan_audit.py \
  --plan /data/flight-a-plan.json \
  --output /data/flight-a-plan-audit.json

python3 -m unittest tests.test_rf_capture_plan_audit
```

The audit checks raw SHA-256 equality, distinct TX/RX files, nonoverlapping
planned ranges, monotonic TX sequences, **exact match** between the plan's
identities and the TX log, and every RX event's membership in the TX
inventory. It then performs the established strict-versus-64-position
replay-policy comparison on the exact RX ordering. Missing planned sender
packets, unexpected sender packets, duplicate TX identities and unknown
receptions all fail closed. At most one million planned packet identities
are expanded for a single audit.

Pass means **internal coverage of this declared plan**. The output has
`capture_provenance_independently_verified: false`,
`protocol_was_preregistered_verified: false` and
`physical_rf_origin_verified: false` by design. An unmeasured missing
reception may be caused by instrumentation, channel conditions, filtering
or receiver processing rather than an RF propagation dropout.

## Reviewer 3 completion criteria

Before publishing empirical false-rejection or packet-loss claims:

1. Verify field trial identity, sender-side instrumentation, immutable
   protocol registration and original dataset hashes independently.
2. Confirm recorded packets span the experiment's start/end observation
   windows and all participating UAV devices; investigate resets and
   retransmissions explicitly.
3. Repeat under measured network load, mobility, topology, interference
   and drift conditions, and report each trial's per-policy results and
   paired differences with independent-trial confidence intervals.
4. Cross-check packets against the production C++ replay implementation,
   state clearly that sequence-only parity is not RF-layer authentication.
5. Revise the manuscript's methods/results/reviewer response to reflect
   **only verified data**; do not translate simulated or proxy-network
   findings into generalized flight-safety claims.

This PR contains software auditing and synthetic unit tests. It does not
contain external RF traces, measured outcomes, or certified pretrial plans.
