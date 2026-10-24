# Review locally downloaded public LoRa data without the API

The publisher API currently replies HTTP 401 to the unauthenticated
GitHub-hosted source probe. Investigators can try the browser download page
for the exact published dataset version:
https://data.mendeley.com/datasets/jm58sggtj7/2

This utility prepares an **honest, provisional local-file inventory** for
the two raw CSVs needed by later analysis, even without a publisher API token.
It does not claim a browser download was performed by the CI runner.

## Input files

Place original unmodified downloaded files into a local directory:

- `measurements.csv`: required packet-level record table
- `packet_loss_by_config.csv`: required published summary table
- `manifest.csv`, `experiment_constants.csv`,
  `airtime_by_sf_bw_payload.csv`, `README.md`: optional context

The actual filenames are those described on the publisher's Version 2
comparison page. If the publisher changes its distribution, investigate
the change rather than silently renaming arbitrary files.

## Commands

```sh
python3 scripts/review_local_public_lora_bundle.py \
  --input-dir /data/lora-v2-from-publisher \
  --output /data/lora-v2-local-inventory.json
```

The JSON lists every recognized input file's **locally computed** SHA-256
and size; for CSVs, it includes real column names and data row counts.
Files larger than 100 MiB, symlink sources, duplicate/empty headers,
invalid CSV structure or missing mandatory files are rejected.

After reviewing the actual column semantics from the publisher's README
and data dictionary, optionally check a manually prepared
`reviewed-column-map.json` without assuming any column names:

```sh
python3 scripts/review_local_public_lora_bundle.py \
  --input-dir /data/lora-v2-from-publisher \
  --mapping /data/reviewed-column-map.json \
  --output /data/lora-v2-local-inventory.json
```

A mapping passes this limited check only when all nominated columns
actually exist. The program **does not** infer loss flags, timestamps,
retransmission policies, packet denominators or physical radio truth.

## Limits and integrity distinctions

A SHA-256 computed from the supplied copy proves only the local bytes'
fingerprint. It is **not equivalent** to a SHA-256 value independently
obtained from Mendeley's authenticated dataset API. This report therefore
unconditionally sets:

- `publisher_sha256_independently_verified: false`
- `source_origin_independently_verified: false`
- `dataset_version_independently_verified: false`
- `is_multi_uav_fanet_evidence: false`
- `independent_sender_tx_inventory_verified: false`
- `can_use_as_measured_uav_replay_results: false`

Never rename this inventory to `public-dataset-integrity.json` to satisfy
downstream publisher verification. The existing
`reconcile_public_lora_per.py` intentionally requires verified publisher
hash metadata and will not treat this local inventory as an authenticated
source.

Mendeley's [current API documentation](https://data.mendeley.com/api/docs/)
confirms API calls use OIDC authorization; its open-access OAI-PMH endpoint
`https://data.mendeley.com/oai` is for **dataset metadata**, not a
replacement for verified binary downloads.

The real dataset is point-to-point LoRa carrying vehicle CAN telemetry,
not a multi-UAV FANET experiment. Original sender-side TX inventory
completeness and flight-level replay false-rejection remain unknown.

## What is still needed

Obtain publisher-provided file digests or properly authorized API access,
compare downloaded bytes against those independent hashes, then use the
already implemented header mapper and per-configuration PER reconciler.
Physical UAV peer TX/RX field trials remain a separate Reviewer 3
requirement.

Unit tests use only synthetic CSVs and never produce experimental results.
