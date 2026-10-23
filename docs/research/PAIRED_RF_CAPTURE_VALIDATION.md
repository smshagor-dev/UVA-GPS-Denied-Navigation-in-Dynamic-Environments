# Paired sender/receiver capture validation for Reviewer 3

The paired field study accepts **complete purported sender TX logs plus matched
receiver RX logs**. Unlike reception-only LoRa counters, such inputs permit
calculating the fraction of transmitted packet identities unseen at a receiver.
The result is still a *trace-specific end-to-end missing observation*, not an
identified RF-layer dropout. Running this script cannot certify source
authenticity, physical flight conditions or completeness of sender logging.

## Suitable external sources

1. AERPAW AADM outdoor UAV dataset (33 flights):
   https://doi.org/10.5061/dryad.7d7wm3898. LoRa `LORAlog.csv` provides
   reception-side frame counters; a full independent application TX inventory
   has not been confirmed. Do **not** invent it from frame-counter gaps.
2. AERPAW LoRaWAN gateway / helikite dataset (1.33 MB):
   https://doi.org/10.5061/dryad.w0vt4b939. It documents `sent_timestamp`,
   packet IDs, received gateway fields and `failed_tx_packages.csv`. Inspect
   the actual archived records and independently verify whether all TX events
   were logged before treating them as a TX inventory. Its moving *helikite*
   and LoRaWAN links are **not equivalent** to a multi-UAV FANET mesh.
3. FANET NS-3.40 traces: https://doi.org/10.5281/zenodo.19373220.
   This is **simulation**, and cannot substantiate real RF observations.

The above sources were inspected as public documentation. No third-party raw
capture was downloaded or measured as part of this change. In particular, a
failed-download listing alone does not provide all attempted transmissions.

## Required normalized per-run files

TX CSV fields: `sender_id,epoch,sequence,tx_ms`.
RX CSV fields: `sender_id,epoch,sequence,rx_ms`. The timestamp columns
must be nonnegative integer milliseconds with reliable within-log ordering;
cross-device clock sync is not needed for the calculations because no delay
is inferred. All RX identities must occur in TX. TX identities must be unique
and monotonic per sender within each analyzed epoch. Use a **separate file
for each flight and radio scenario** and split on epoch/counter resets.

Example *manifest template* (paths and DOI must be replaced after actually
verifying capture provenance):

```json
[
  {
    "flight_id": "outdoor-run-01",
    "scenario": "moving-link",
    "source_doi": "doi:REPLACE-WITH-VERIFIED-DOI",
    "epoch": 1,
    "tx": "run-01/tx.csv",
    "rx": "run-01/rx.csv"
  },
  {
    "flight_id": "outdoor-run-02",
    "scenario": "moving-link",
    "source_doi": "doi:REPLACE-WITH-VERIFIED-DOI",
    "epoch": 1,
    "tx": "run-02/tx.csv",
    "rx": "run-02/rx.csv"
  }
]
```

```sh
python3 scripts/rf_paired_flight_study.py \
  --manifest /data/paired-flights.json \
  --output /data/paired-flights-report.json

cmake --preset validation-linux-gcc
cmake --build --preset validation-linux-gcc --target peer_replay_cache_cli
python3 scripts/rf_paired_flight_study.py \
  --manifest /data/paired-flights.json \
  --native build/validation-linux-gcc/tests/peer_replay_cache_cli \
  --output /data/paired-flights-native-checked.json
```

The native binary location depends on the chosen CMake configuration. The
native harness supports epoch 1 and constructs fresh **test-only signed**
heartbeats. It tests the production sequence admission path, not full
on-air authentication.

## Output and interpretation

For each declared flight, reports distinct TX, RX, duplicates, observed
missing fraction, observed reordered first receptions, strict and bitmap
rejections, paired policy difference, TX/RX hashes, and native parity status.

Per-scenario summaries report unweighted **whole-flight** means with 95%
percentile bootstrap intervals when at least two runs are supplied. With one
run, the confidence interval is null, not a fictitious population estimate.
Repeated flight IDs and identical or reused capture files are rejected.
This is a data-quality check, not proof that self-declared trials were
physically independent.

**Scientific gap still open:** Obtain verifiably complete sender logs,
matching receiver logs, independent runs under multiple measured conditions,
RF channel metadata, checksum/provenance record, and real trace results.
Only after these have been reviewed should the manuscript report actual
field-network packet-loss and false-rejection values. A helikite or LoRaWAN
comparison requires careful external-validity caveats and cannot establish
UAV swarm deployment reliability by itself.
