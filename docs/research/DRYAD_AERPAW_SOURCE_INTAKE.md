# Public AERPAW LoRaWAN / Dryad source acquisition

This workflow addresses the failed unauthenticated Mendeley Data API access
(HTTP 401) by examining an independently published open-access candidate.

Dataset: **Vargas Villar et al. (2025), LoRaWAN gateway performance and
vehicle tracking data in AERPAW testbed**, Dryad DOI:
https://doi.org/10.5061/dryad.w0vt4b939

The public landing page lists a 1.33 MB 7z archive and README. The experiment
used LoRaWAN gateways and a moving helikite/vehicle transmitter, not a
self-organizing multi-UAV FANET. The archive may include received SigMF
records, transmitter information, and failed-transmission summaries, but
**no complete independent sender-side packet inventory has been verified**.

## Publisher-hash verification

The intake uses the official, read-only Dryad REST API:

1. GET the exact DOI-encoded public dataset record.
2. Follow the documented published-version resource URI.
3. List the version's files and validate the SHA-256 digests and declared sizes.
4. Optionally download both allowlisted files to an ephemeral runner directory.
5. Stream bytes to temporary files with a strict maximum size, and publish only
   after the SHA-256 digest and exact size match the publisher's catalog.

```sh
python3 scripts/check_dryad_aerpaw_source.py \
  --output-dir /tmp/dryad-aerpaw --download --strict
```

The script saves `dryad-source-status.json`. A result of
`all_publisher_sha256_matched` is necessary to claim that the local archive
bytes match the Dryad API's published file hashes. A result of
`publisher_file_manifest_available` establishes only that catalog metadata
was available, not that data files were obtained. If network access fails,
`source_unavailable_or_incomplete` is recorded without fabricating
measurements or making a green check imply source success.

The companion `.github/workflows/dryad-source.yml` runs an **informational**
live-source probe on PRs changing this code and via manual dispatch. It uploads
**only the JSON acquisition status**, not the 7z archive, README, inferred TX
logs, raw RF metadata or any actual research measurements.

## Evidence limits

The archive is a compressed `7z` file: this first stage deliberately does
**not** extract it, to avoid treating unaudited decompressed data as validated.
Any later archive inspection must use bounded extraction, inspect true schema
and provenance, and differentiate gateway RX observations from independent TX.

The status output hard-codes the following as false:

- `real_multi_uav_fanet_evidence`
- `complete_independent_sender_tx_inventory_established`
- `observed_rf_loss_causally_attributed`
- `replay_false_rejection_measured`

Successful download alone cannot establish these facts. Do not substitute
helikite/LoRaWAN reception statistics for an independent multi-UAV FANET
peer-replay measurement or for complete per-UAV sender captures.

The **mandatory Python CI tests are offline synthetic fixtures** exercising
metadata identity, file list, digest type, redirect origin, corruption,
existing file reuse and honest result classification. These tests do not
validate any measured RF outcome. Dryad's live service can still change;
the live-source probe is nonblocking and reports the actual result.

## Verified real publisher metadata, even when file downloads require a token

The first real GitHub-hosted Dryad API probe returned the **published
version identifier 353153** and publisher SHA-256 checksums for both files,
but the file download endpoint responded with **HTTP 401**. Actual raw
archive bytes have not been downloaded or analyzed.

The metadata-only `publisher_file_manifest_available` result is already
useful: an investigator can download the exact archive and README using
the Dryad browser/authorized channel, put the **original untouched files**
in a local directory and validate their bytes against the current public
Dryad metadata without guessing or fabricating checksums:

```sh
python3 scripts/check_dryad_aerpaw_source.py \
  --output-dir /data/independently-downloaded-dryad \
  --verify-local --strict
```

The `--verify-local` option **never downloads a missing local file**. It
requires both approved filenames already present, checks their original
length and streamed SHA-256 against the publisher version file listing, and
records `local_files_match_publisher_sha256` only if both match. It
distinguishes `verified_local_file_count` from
`verified_download_count` and describes the files as investigator-supplied.

A download HTTP 401/403 that occurs **after successfully fetching the
published file index** is recorded as
`publisher_manifest_available_download_denied` rather than pretending
the archive was never cataloged.

**Important:** cryptographic agreement with the publisher's file digest
authenticates the downloaded file's bytes against that catalog. It does
not prove complete independent sender TX logging, link-level RF loss,
physical flight independence, or multi-UAV peer-replay security.
