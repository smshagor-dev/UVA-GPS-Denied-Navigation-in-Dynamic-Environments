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

## Actual GitHub-hosted source probe result

The first PR #67 probe performed an unauthenticated request to the
publisher endpoint on GitHub-hosted Ubuntu and received **HTTP 401**. No
real dataset bytes were verified, and no CSV header inventory was made.
A completed informational workflow is **not evidence of dataset access**.

HTTP 401/403 is now reported as `publisher_access_denied` with an
`access_blocker` indicating authentication or permissions may be needed.
The probe intentionally records no signed URLs, tokens, or raw HTTP
exception strings.

For investigators authorized to use a publisher API access token, the
intake supports an **optional** `MENDELEY_DATA_ACCESS_TOKEN` environment
variable. The token is added only to requests sent to the exact
`api.data.mendeley.com` host, and removed from all redirects before
downloading any publisher storage content. Do not place the token in
repository files, test fixtures, shell history, PR descriptions, CI logs,
or a pull-request workflow environment.

Example local use after configuring the credential in the caller's
environment:

```sh
python3 scripts/check_external_radio_source.py \
  --output-dir /data/public-radio \
  --strict
```

The GitHub pull-request informational probe deliberately has **no
publisher API credential**. An authorized credential can be used in a
separately secured investigator environment. Even successful authenticated
download proves the retrieved file checksum matches the publisher
metadata; it does not establish a complete sender-side transmission
inventory, multi-UAV FANET applicability or RF loss attribution.
