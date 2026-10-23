# Importing independent radio capture logs

The repository's paired RF study requires two *independent sources* per
experiment: application-level packet transmissions actually recorded at the
sender and receptions actually recorded at the gateway. This importer maps
investigator-provided CSV files into the existing `sender_id,epoch,sequence`
format. It **does not** establish physical provenance, prove the sender log
contains every attempted transmission, or infer RF loss from frame counters.

## What the AERPAW dataset descriptions establish

Vargas Villar et al. (2025), Dryad
https://doi.org/10.5061/dryad.w0vt4b939, publish a 1.33 MB LoRaWAN gateway
dataset collected with a moving helikite in August 2024. The Dryad page lists
`sent_timestamp`, a packet ID, gateway `rx_time`, RSSI and SNR; its data
archive also contains failed-transmission records and SigMF files. **The
published description does not prove a complete, independently instrumented
TX packet inventory exists in those files.** Archive download currently
requires manual retrieval; no raw records from it were inspected or run in
the repository as part of this implementation.

The larger AADM dataset
https://doi.org/10.5061/dryad.7d7wm3898 provides LoRa reception logs
from 33 physical flights. Its `f_cnt` values alone are not transmitter logs.

**Never use an RX table's embedded `sent_timestamp` field as TX evidence.**
That value only exists for successfully observed/reported packets and omits
unsuccessful/missing transmissions. Independently instrumented sender logs
are required for a trace-specific missed-packet denominator.

## Import a source pair

Create a private JSON specification alongside the independently acquired
`sender.csv` and `gateway.csv` source files. The following names are
illustrative field mappings, **not confirmed schema fields** in a downloaded
Dryad archive.

```json
{
  "flight_id": "REAL_FLIGHT_ID",
  "scenario": "moving-link",
  "source_doi": "doi:10.5061/dryad.w0vt4b939",
  "sender_id": 1,
  "epoch": 1,
  "tx": {
    "file": "sender.csv",
    "role": "independent_sender_device_log",
    "sequence_column": "packet_id",
    "time_column": "sent_timestamp",
    "time_format": "iso8601_utc"
  },
  "rx": {
    "file": "gateway.csv",
    "role": "gateway_reception_log",
    "sequence_column": "packet_id",
    "time_column": "rx_time",
    "time_format": "iso8601_utc",
    "filter_column": "rx_gatewayId",
    "filter_value": "REAL_GATEWAY_ID"
  }
}
```

```sh
python3 scripts/rf_capture_import.py \
  --spec /data/capture-1.json \
  --output-dir /data/normalized/run-1

python3 -m unittest tests.test_rf_capture_import
```

Inputs may use `integer_milliseconds` or timezone-qualified
`iso8601_utc` timestamps. The output includes normalized `matched-tx.csv`,
`matched-rx.csv`, a proposed manifest entry and `capture-audit.json` with
source/output SHA-256 hashes. The importer rejects identical input files,
counter duplicates/non-monotonic TX sequences, negative timestamps, unknown
RX packet identities and missing schema columns. Optionally filter one
gateway's received events; do not mix gateway delivery order.

Run each genuinely independent flight in a **different output directory**.
Combine the proposed manifest entries into a JSON list only after verifying
the captures are different flights and updating relative paths. Two or more
such captures can then be evaluated by
`python3 scripts/rf_paired_flight_study.py --manifest ... --output ...`.

## Before using numerical results in the paper

An investigator must verify and document:

- complete independent sender-side TX instrumentation, including attempted
  transmissions never observed by the receiver, and the exact capture
  observation window;
- source DOI, original archive hash, original filenames, device and gateway
  identity, mobility and interference conditions;
- reset/rejoin and counter semantics, with matching sender/epoch/sequence
  identity and no undocumented rollover or retransmission ambiguity;
- independent physical flight/trial collection, ethics and redistribution
  constraints, and whether LoRaWAN helikite results are externally relevant
  to the proposed UAV multi-node protocol.

The importer reports `input_origin_verified: false`,
`complete_transmission_inventory_verified: false` and
`physical_rf_provenance_verified: false`. These are deliberate: a JSON
declaration and SHA-256 values verify file identity and internal consistency,
not source truth or measurement completeness. Do not relabel software
checks as independent scientific validation.
