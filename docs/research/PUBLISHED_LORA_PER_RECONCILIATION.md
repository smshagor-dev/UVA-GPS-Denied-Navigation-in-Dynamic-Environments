# Reproducing published point-to-point LoRa packet error rates

This analysis cross-checks two **locally downloaded, checksum-matched**
files from the published *LoRa Field-Test P2P CAN Telemetry Dataset*,
version 2, DOI https://doi.org/10.17632/jm58sggtj7.2:

- `measurements.csv`: packet-level records with a publisher-defined loss indicator.
- `packet_loss_by_config.csv`: publisher's per-configuration packet
  counts, flagged loss counts and published packet error rates (PER).

Source dataset and acquisition:
[PUBLIC_LORA_FIELD_DATA_INTAKE.md](PUBLIC_LORA_FIELD_DATA_INTAKE.md).

**No raw third-party records were available to this implementation.**
All automated tests use synthetic CSVs. This tool is intended to run after
the investigator downloads and independently inspects the published files.

## Explicit schema review is mandatory

The exact CSV headers and missing-packet semantics must be inspected locally.
Do **not** invent mappings based on expected sensor names. The script has
no built-in assumption about the source's distance, spreading-factor, power,
bandwidth, packet ID or missing-packet field names.

A mapping JSON must specify every configuration key, packet-level loss
indicator, published packet count, published loss count, PER column and
whether PER is a `fraction` or `percent`.

Illustrative mapping **only**; replace names with the actual audited headers:

```json
{
  "groups": [
    {
      "label": "distance_m",
      "measurement": "ACTUAL_MEASUREMENT_DISTANCE_HEADER",
      "summary": "ACTUAL_SUMMARY_DISTANCE_HEADER"
    },
    {
      "label": "spreading_factor",
      "measurement": "ACTUAL_MEASUREMENT_SF_HEADER",
      "summary": "ACTUAL_SUMMARY_SF_HEADER"
    }
  ],
  "measurement_lost": "ACTUAL_BINARY_PACKET_LOSS_HEADER",
  "summary_count": "ACTUAL_TOTAL_ATTEMPTS_HEADER",
  "summary_lost": "ACTUAL_REPORTED_LOSS_COUNT_HEADER",
  "summary_per": "ACTUAL_REPORTED_PER_HEADER",
  "summary_per_unit": "fraction"
}
```

Include **all** configuration dimensions relevant to published aggregation,
for example distance, spreading factor, bandwidth, transmit power and test
direction if these are present in the actual source files. Both CSVs must
use the same configuration key values (numeric strings are not guessed or
normalized).

```sh
python3 scripts/public_lora_dataset_intake.py \
  --output-dir /data/field-lora-v2 --download

python3 scripts/reconcile_public_lora_per.py \
  --dataset-dir /data/field-lora-v2 \
  --mapping /data/reviewed-column-map.json \
  --output /data/published-per-comparison.json
```

The script requires the earlier downloader's
`public-dataset-integrity.json` and verifies original file
sizes and SHA-256 digests **before** processing either CSV. A changed local
file fails even if a modified statistics table would otherwise agree.

The tool recomputes packet-row counts, flagged losses and PER per configuration,
and compares all three against the published summary. Missing, extra or
duplicate configurations are explicitly reported/rejected; invalid count,
nonfinite PER, unexpected loss flag or missing mapped header fails closed.
A sample-count, loss-count or PER mismatch sets
`reproduction_matches_all` to false and exits with status 1
after writing the JSON report. It does not silently excuse disagreements.

The default tolerance is 0.000001 in **fraction units**; if the publisher
rounds displayed PER, provide a declared tolerance up to 0.01, explain the
publisher's rounding rule, and retain the tolerance in the report.

## Scientific interpretation

A successful comparison establishes only that the *published packet-level
indicator column* can reproduce the *published aggregate PER table* under
the reviewed mapping. It does **not** certify a complete independent sender
TX inventory, prove packet-level RF losses or authenticate the collection
equipment. The packet-row denominator is not assumed to equal every
attempted physical radio transmission.

These were point-to-point LoRa/vehicle-CAN experiments, not an independent
multi-UAV FANET validation. Never use these per-configuration results to claim
measured benign peer-replay false rejection, flight-safety availability or
production swarm performance. Reviewer 3's FANET empirical requirements
remain open until authentic paired UAV sender TX and receiver RX records
with source and protocol provenance are obtained.

The output deliberately sets `is_fanet_field_validation`,
`packet_tx_completeness_verified` and
`field_origin_independently_verified` to false.
