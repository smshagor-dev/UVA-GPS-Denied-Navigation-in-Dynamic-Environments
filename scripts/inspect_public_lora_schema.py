#!/usr/bin/env python3
"""Inspect checksum-matched external LoRa CSV headings without guessing a schema.

Only publisher-index consistency is checked here. No physical provenance,
packet-error semantics or independent sender-side TX records are certified.
"""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path

if __package__:
    from .reconcile_public_lora_per import REQUIRED_FILES, check_files
else:
    from reconcile_public_lora_per import REQUIRED_FILES, check_files


def inspect_schema(folder: Path) -> dict:
    verified = check_files(folder)
    results = []
    for filename in REQUIRED_FILES:
        with (folder / filename).open(
            "r", encoding="utf-8-sig", newline=""
        ) as stream:
            reader = csv.reader(stream)
            try:
                headings = next(reader)
            except StopIteration as exc:
                raise ValueError(f"{filename}: empty CSV") from exc
            if (
                len(headings) > 128
                or not headings
                or any(not field.strip() for field in headings)
                or len(set(headings)) != len(headings)
            ):
                raise ValueError(f"{filename}: invalid or repeated CSV headings")
            count = 0
            for number, row in enumerate(reader, start=2):
                if len(row) != len(headings):
                    raise ValueError(
                        f"{filename}:{number}: column count mismatch"
                    )
                count += 1
        results.append(
            {
                "filename": filename,
                "publisher_index_sha256": verified[filename]["sha256"],
                "column_names": headings,
                "data_row_count": count,
            }
        )
    return {
        "schema": "external-lora-header-inventory-v1",
        "dataset_doi": "10.17632/jm58sggtj7.2",
        "files_match_local_publisher_metadata_index": True,
        "publisher_index_authenticity_independently_verified": False,
        "source_is_multi_uav_fanet": False,
        "schema_mapping_review_required": True,
        "packet_loss_denominator_verified": False,
        "contents": results,
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dataset-dir", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    try:
        protected = {
            (args.dataset_dir / filename).resolve()
            for filename in (*REQUIRED_FILES, "public-dataset-integrity.json")
        }
        if args.output.resolve() in protected:
            raise ValueError("schema report must not overwrite evidence inputs")
        report = inspect_schema(args.dataset_dir)
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(
            json.dumps(report, indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )
    except (OSError, ValueError, TypeError) as exc:
        parser.error(str(exc))
    print(json.dumps(report, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
