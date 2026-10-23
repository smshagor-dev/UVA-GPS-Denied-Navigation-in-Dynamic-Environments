# Acquire published point-to-point LoRa field measurements

The dataset by Croall and colleagues, *LoRa Field-Test P2P CAN Telemetry
Dataset*, version 2 (1 July 2026), DOI
https://doi.org/10.17632/jm58sggtj7.2, provides outdoor point-to-point LoRa
measurements across distances, spreading factors, bandwidths and powers.
The source is published on Mendeley Data under CC BY 4.0. It includes
packet-level measurements, per-configuration loss summaries and a manifest.

**This is not a multi-UAV FANET field experiment.** It uses stationary road
test positions and vehicle CAN telemetry with bidirectional LoRa links.
Its published "packet loss" values cannot be reinterpreted as authenticated
UAV peer replay false rejections. The source description says loss was
derived from packet sequences; an independently recorded, full TX inventory
still needs verification before use in the repository's matched trace tools.

## Pinned acquisition and checks

The script reads the *official public Mendeley Data dataset API* for
\`jm58sggtj7\` version \`2\`. It accepts only the six expected filenames:
\`measurements.csv\`, \`packet_loss_by_config.csv\`, \`manifest.csv\`,
\`experiment_constants.csv\`, \`airtime_by_sf_bw_payload.csv\` and
\`README.md\`. These names match the publisher's description; the actual
archive contents are only known after downloading.

Metadata-only mode:

\`\`\`sh
python3 scripts/public_lora_dataset_intake.py \
  --output-dir /data/field-lora-v2
\`\`\`

Download from the publisher's file URLs (requires HTTPS internet access):

\`\`\`sh
python3 scripts/public_lora_dataset_intake.py \
  --output-dir /data/field-lora-v2 --download
\`\`\`

The tool rejects unknown download hosts, non-HTTPS URLs, missing publisher
SHA-256 checksums, missing lengths, excessive individual file size,
unexpected version and duplicate filenames. Files are streamed into a
temporary file and become visible only after both publisher checksum and
exact file-size matches. Existing local files are not silently overwritten
if they differ from the publisher's metadata. It stores a non-experimental
\`public-dataset-integrity.json\` record. If the publisher API withholds
public file URLs, checksum or access, the script exits instead of inventing
download records.

The official API is documented at
https://data.mendeley.com/api/docs/; the current API host is
\`api.data.mendeley.com\`, not the old \`api.mendeley.com\` host.

## Research interpretation

1. Verify original dataset version, license and SHA-256 metadata. Cite
   Croall et al. and their dataset DOI before reuse.
2. Inspect actual packet-level CSV headers, physical measurement origin,
   missing-value labels, retransmission handling and any sequence resets.
3. Reproduce the publisher's packet-error-rate summary against the
   configuration manifest and published constants, reporting differences.
4. Treat outcomes as **external LoRa radio context only**, not as measured
   FANET \(p_R\), peer replay false-rejection rate or flight reliability.
5. For a true Reviewer 3 FANET claim, independently collect and certify
   the predeclared full sender TX inventory plus matched receiver RX capture
   across multiple UAV flights, then use the existing paired-flight study,
   capture-plan audit and reviewer evidence dossier.

No third-party raw records were downloaded or analyzed by this PR because
public API endpoints were inaccessible from the development runtime.
Automated tests mock the publisher's responses and use synthetic bytes.

This code should not be described as a completed experiment or substitute
for independent field collection.
