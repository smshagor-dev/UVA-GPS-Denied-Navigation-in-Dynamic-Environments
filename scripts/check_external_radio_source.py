#!/usr/bin/env python3
"""Attempt public LoRa acquisition in an isolated CI runner and record the outcome.

The probe is informational and never promotes third-party point-to-point radio
data to independently validated multi-UAV FANET evidence.
"""

from __future__ import annotations

import argparse
import json
import urllib.error
from pathlib import Path

if __package__:
    from .inspect_public_lora_schema import inspect_schema
    from .public_lora_dataset_intake import DATASET_ID, DOI, VERSION, acquire
else:
    from inspect_public_lora_schema import inspect_schema
    from public_lora_dataset_intake import DATASET_ID, DOI, VERSION, acquire

REPORT = "external-source-status.json"


def attempt(folder: Path, *, download: bool = True) -> dict:
    status = {
        "schema": "external-radio-availability-check-v1",
        "dataset_id": DATASET_ID,
        "dataset_version": VERSION,
        "source_doi": DOI,
        "attempted_real_publisher_access": True,
        "download_requested": download,
        "result": "source_unavailable",
        "verified_download_count": 0,
        "schema_inventory_available": False,
        "is_uav_fanet_evidence": False,
        "sender_tx_inventory_completeness_verified": False,
        "physical_origin_independently_verified": False,
        "replay_false_rejection_measured": False,
    }

    try:
        metadata = acquire(folder, download=download)
        status["publisher_index_file_count"] = len(metadata["files"])
        if not download:
            status["result"] = "publisher_metadata_available"
        else:
            verified = metadata.get("verified_downloads", [])
            status["verified_download_count"] = len(verified)
            index = inspect_schema(folder)
            status["result"] = "publisher_files_and_csv_schema_verified"
            status["schema_inventory_available"] = True
            status["csv_schema"] = index["contents"]
            (folder / "public-lora-schema.json").write_text(
                json.dumps(index, indent=2, sort_keys=True) + "\n",
                encoding="utf-8",
            )
    except (OSError, ValueError, TypeError, urllib.error.URLError) as error:
        status["result"] = "source_unavailable_or_incomplete"
        status["failure_class"] = type(error).__name__
        # Do not echo exception strings: HTTP endpoints can contain
        # temporary signed download credentials.
        if isinstance(error, urllib.error.HTTPError):
            status["http_status"] = error.code

    folder.mkdir(parents=True, exist_ok=True)
    (folder / REPORT).write_text(
        json.dumps(status, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    return status


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument(
        "--metadata-only",
        action="store_true",
        help="Read publisher metadata without downloading source files",
    )
    parser.add_argument(
        "--strict",
        action="store_true",
        help="Nonzero exit unless all source files and CSV schemas are verified",
    )
    args = parser.parse_args()
    outcome = attempt(args.output_dir, download=not args.metadata_only)
    print(json.dumps(outcome, indent=2, sort_keys=True))
    if args.strict and outcome["result"] not in (
        "publisher_files_and_csv_schema_verified",
        "publisher_metadata_available",
    ):
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
