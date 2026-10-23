# Live external LoRa dataset acquisition status

Use this informational probe to establish whether the official public
Mendeley Data API currently permits retrieval of the **real** archived
point-to-point LoRa telemetry files.

Source: [LoRa Field-Test P2P CAN Telemetry Dataset, v2](https://doi.org/10.17632/jm58sggtj7.2).

The corresponding GitHub Actions workflow is
`.github/workflows/public-radio-source.yml`. It runs on source-probe PRs
and can be manually dispatched from Actions. The job's checks report
**availability**, not a scientific PASS or measured field result.

## Run

```sh
python3 scripts/check_external_radio_source.py \
  --output-dir /tmp/public-radio-input
```

This accesses the real publisher, downloads whitelisted source files
subject to publisher SHA-256 and size checks, then inventories the actual
CSV columns and row counts. Network, authentication, expired signed-link,
missing-file and integrity failures are reported as
`source_unavailable_or_incomplete`. Successful source acquisition yields
`publisher_files_and_csv_schema_verified` but **does not** establish
physical experiment authenticity or complete independent TX logs.

For offline smoke testing of the metadata only:

```sh
python3 scripts/check_external_radio_source.py \
  --output-dir /tmp/public-radio-index --metadata-only
```

To make an unavailable source fail a local test command instead of
returning an informational report, add `--strict`.

## Source and evidence handling

The Actions job uploads **only** `external-source-status.json`, the
publisher metadata integrity index if available, and CSV header inventory
if downloaded. Raw measurements stay in the short-lived CI runner. All
reports explicitly say the source is **not** a multi-UAV FANET,
independently verified sender TX inventory or measured replay rejection
experiment. Reported error classes intentionally omit exception strings
because temporary signed publisher URLs can contain credentials.

An informational green Actions job means only that it recorded the
availability status correctly. To establish complete external data access
you must inspect the actual result field and check that it equals
`publisher_files_and_csv_schema_verified`. Unavailable data must **not**
be cited as reproduced.

Once real files are obtained and independently reviewed, use the actual
CSV headings to make a verified field mapping for
`scripts/reconcile_public_lora_per.py`. Do not infer a missing packet
denominator from reception-only records. Reproducing published point-to-point
LoRa packet-loss statistics is not a substitute for Reviewer 3's requested
UAV RF/TX/RX trial evidence.

The GitHub Actions test is deliberately network-dependent and therefore
*not a required merge gate*. Required Python CI only runs fully offline
synthetic regression tests against this probe's behavior.
