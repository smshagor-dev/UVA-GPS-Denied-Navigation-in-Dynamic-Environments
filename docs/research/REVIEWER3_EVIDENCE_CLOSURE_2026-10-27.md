# Reviewer 3 empirical evidence closure checklist (27 October 2026)

## Evidence verdict
**OPEN — physical multi-UAV FANET validation is not established.** Passing CI and merging the Dryad SHA-256 intake PR only validate software and published archive integrity. The 2025 AERPAW LoRaWAN gateway/helikite dataset is not a multi-UAV mesh. The AADM reception-only logs cannot establish attempted transmissions or packet-loss denominators. NS-3 traces are simulated.

## Acquisition acceptance gate
For at least two independently acquired physical trials per scenario, collect original immutable sender TX and receiver RX logs with sender ID, authenticated epoch, application sequence, monotonic timestamp, receiver identity, packet digest, and independent instrumentation evidence. Preserve original SHA-256, source DOI/permissions, signed collection metadata, start/end capture coverage, counter resets/retransmissions, radio configuration, mobility, topology, interference, clock methods, firmware revision, and declared pretrial packet plan. Do not infer TX events from RX counter gaps. Separate receivers and epochs.

## Reproducible commands (after verified data exists)
```bash
python3 scripts/rf_capture_plan_audit.py --plan /data/flight-01/plan.json --output /data/flight-01/audit.json
python3 scripts/rf_paired_flight_study.py --manifest /data/paired-flights.json --output /data/paired-report.json
python3 scripts/reviewer_rf_evidence_dossier.py --manifest /data/paired-flights.json --plans /data/plan-index.json --output-json /data/evidence.json --output-md /data/evidence.md
cmake --preset validation-linux-gcc
cmake --build --preset validation-linux-gcc --target peer_replay_cache_cli
ctest --preset validation-linux-gcc -R peer_replay_native_parity --output-on-failure
```

## Reportable quantities and limits
Report end-to-end missing-first-reception fraction only if independent complete TX inventories are verified. Do not call it RF propagation loss without radio-layer attribution. Report arrival-order inversions and benign rejection on distinct first receptions per receiver/epoch. Estimate paired policy differences using independent whole-flight trials, not packet-level pseudo-replicates. Include per-trial sample sizes, confidence intervals, source hashes, and native C++ decision parity. Never represent test fixtures, simulated traces, or receiver-only logs as real FANET trials.

## Manuscript-ready limitation statement
We implemented deterministic sensitivity analysis, production C++ sequence-admission parity tests, and provenance-aware TX/RX capture audit tools. However, independently verified physical multi-UAV sender/receiver trace pairs with complete transmission inventories are not yet available. Consequently, this study does not claim empirically measured FANET packet-loss rates, real-flight benign-replay rejection rates, or flight-level availability. The software results are conditional on explicitly modeled assumptions.

## Reviewer 3 response draft
Thank you for requesting real/calibrated FANET traces, multiple loss/reordering scenarios, and comparison against production C++ policy. We added replicated software sensitivity studies and a native C++ decision-parity harness, together with a reproducible protocol for validating independently captured paired TX/RX traces. We explicitly distinguish synthetic simulations, reception-only RF telemetry, and complete physical FANET experiments. We cannot claim the requested physical validation without independently verified raw captures; accordingly, we have limited the empirical claims and documented this limitation. We will not substitute LoRaWAN helikite or ns-3 data for physical multi-UAV measurements.

## Release gate
Do not mark Reviewer 3 physical evidence complete or paper resubmission-ready until original source files and independent provenance checks pass. This checklist is a documentation improvement, not new field measurements.
