# Reviewer 3 RF evidence dossier

This tool connects the separately declared sender packet plans, the original
TX and RX captures, and the multi-flight replay-policy statistics into **one
reproducible descriptive draft**. It has no network access and cannot verify
physical collection, radio-layer loss, independent flight origins, immutable
pretrial protocols or complete transmission instrumentation by itself.

## Inputs

1. A paired-flight JSON list for
   `scripts/rf_paired_flight_study.py` with at least two trials. Each
   entry identifies `flight_id`, `scenario`, `source_doi`, `epoch`,
   `tx` and `rx` files. Refer to
   [PAIRED_RF_CAPTURE_VALIDATION.md](PAIRED_RF_CAPTURE_VALIDATION.md).
2. A packet-plan index with exactly the same flight IDs. Each entry names
   a separate plan JSON file, checked against the same **raw** capture
   bytes. A plan is as documented in
   [RF_CAPTURE_PLAN_AUDIT.md](RF_CAPTURE_PLAN_AUDIT.md).

Example index, where entries and filenames are illustrative **only**:

```json
[
  {"flight_id": "outdoor-001", "plan": "outdoor-001/plan.json"},
  {"flight_id": "outdoor-002", "plan": "outdoor-002/plan.json"}
]
```

Run locally after collecting properly licensed raw field captures:

```sh
python3 scripts/reviewer_rf_evidence_dossier.py \
  --manifest /data/paired-flights.json \
  --plans /data/plan-index.json \
  --output-json /data/reviewer-evidence.json \
  --output-md /data/reviewer-evidence-draft.md
```

An optional `--native /path/to/peer_replay_cache_cli` uses the compiled
C++ implementation to check sequence-admission parity, with the same
restrictions as the paired-flight study.

## Checks and report structure

The program fails on inconsistent flight IDs, repeated plan sources,
missing or unexpected packet identities, changed raw SHA-256 hashes,
epoch/claimed-source mismatch and mismatches between audit counts and
paired-flight statistics. The JSON contains all per-flight file checksums,
descriptive paired statistics and their bootstrap intervals. The Markdown
draft contains a compact summary table and remaining evidence gaps.

The *same original TX/RX files* must be referenced by each flight's packet
plan and the paired-flight manifest. Reprocessed files, receiver-only
`sent_timestamp` fields and synthetic sample data cannot be substituted
for independent transmitter captures.

## Meaning of report status

`software_evidence_consistent: true` means only that the **supplied
documents and traces agree internally**. The following remain false:

- `safe_to_claim_measured_rf_loss`
- `safe_to_claim_fanet_operational_reliability`
- `physical_capture_provenance_verified`
- `independent_sender_instrumentation_verified`
- `preregistered_protocol_verified`
- `physical_trial_independence_verified`

The tool deliberately does not provide a switch that marks these claims
true: independent checks require source archives, field acquisition
metadata, evidence of complete TX instrumentation, authentic pretrial
protocol records and human scientific review.

**No public AERPAW raw measurements were acquired or analyzed in this PR.**
The unit tests use invented, synthetic CSVs with explicit synthetic labels.
Reported sample values from tests must never be copied into a paper as
real-world UAV network behavior.

Dataset candidates to evaluate separately:
[AERPAW AADM 33-flight archive](https://doi.org/10.5061/dryad.7d7wm3898)
and [AERPAW LoRaWAN moving-link archive](https://doi.org/10.5061/dryad.w0vt4b939).
The first has published RF reception records but independent complete
sender-side packet inventory is not established. The second is a helikite
LoRaWAN experiment, not a multi-node UAV FANET, and its raw fields must be
inspected before making completeness claims.

## Reviewer-facing language

Until original traces and independent verification have been obtained,
the appropriate description is:

> We added a reproducible software-based sensitivity analysis and
> sequence-admission parity test using the production C++ implementation.
> We also implemented provenance-aware tools for evaluating independently
> recorded radio TX/RX traces. However, because a complete, independently
> verified real-world sender/receiver capture pair is not yet available,
> these results do not establish measured FANET packet-loss or benign
> replay-rejection rates, nor demonstrate flight-level availability.

A real empirical results section should only be written after measured
data and provenance have been independently confirmed. Distinguish
end-to-end missing first receptions from RF-layer packet loss and
avoid treating a collection of simulated trials as physical replicates.
