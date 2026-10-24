#!/usr/bin/env python3
"""Inventory locally supplied published LoRa CSVs when API access is blocked.

This tool computes LOCAL checksums and checks CSV structure. It does not
authenticate files against the publisher or claim a real UAV experiment.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
from pathlib import Path

if __package__:
    from .reconcile_public_lora_per import load_mapping
else:
    from reconcile_public_lora_per import load_mapping

REQUIRED = ("measurements.csv", "packet_loss_by_config.csv")
OPTIONAL = (
    "manifest.csv",
    "experiment_constants.csv",
    "airtime_by_sf_bw_payload.csv",
    "README.md",
)
MAX_FILE_BYTES = 100 * 1024 * 1024
MAX_ROWS = 5_000_000
MAX_COLUMNS = 128


def checksum(path: Path) -> str:
    sha = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            sha.update(block)
    return sha.hexdigest()


def csv_inventory(path: Path) -> dict:
    with path.open("r", newline="", encoding="utf-8-sig") as stream:
        reader = csv.reader(stream, strict=True)
        try:
            headings = next(reader)
        except StopIteration as exc:
            raise ValueError(f"{path.name}: empty CSV file") from exc
        if (
            not headings
            or len(headings) > MAX_COLUMNS
            or any(not name.strip() for name in headings)
            or len(set(headings)) != len(headings)
        ):
            raise ValueError(f"{path.name}: invalid or duplicate CSV headings")
        count = 0
        for row in reader:
            if len(row) != len(headings):
                raise ValueError(f"{path.name}: malformed row {count + 2}")
            count += 1
            if count > MAX_ROWS:
                raise ValueError(f"{path.name}: row limit exceeded")
    return {"column_names": headings, "data_row_count": count}


def review_mapping(mapping_path: Path, inventory: dict) -> dict:
    mapping = load_mapping(mapping_path)
    by_name = {item["filename"]: item for item in inventory["files"]}
    required = [
        (
            "measurements.csv",
            [entry["measurement"] for entry in mapping["groups"]]
            + [mapping["measurement_lost"]],
        ),
        (
            "packet_loss_by_config.csv",
            [entry["summary"] for entry in mapping["groups"]]
            + [
                mapping["summary_count"],
                mapping["summary_lost"],
                mapping["summary_per"],
            ],
        ),
    ]
    missing = {}
    for filename, selected in required:
        available = set(by_name[filename]["column_names"])
        absent = sorted(set(selected) - available)
        if absent:
            missing[filename] = absent
    return {
        "mapping_file_sha256": checksum(mapping_path),
        "mapped_csv_headings_present": not missing,
        "missing_mapped_headings": missing,
        "mapping_semantics_independently_reviewed": False,
    }


def inspect_local_bundle(folder: Path, mapping: Path | None = None) -> dict:
    if not folder.is_dir():
        raise ValueError("source directory does not exist")
    files = []
    for filename in REQUIRED + OPTIONAL:
        path = folder / filename
        if path.is_symlink():
            raise ValueError(f"{filename}: symlink is not an accepted source")
        if not path.exists():
            if filename in REQUIRED:
                raise ValueError(f"missing required local file: {filename}")
            continue
        if not path.is_file():
            raise ValueError(f"{filename}: expected a regular file")
        size = path.stat().st_size
        if size > MAX_FILE_BYTES:
            raise ValueError(f"{filename}: file too large for local review")
        item = {
            "filename": filename,
            "size_bytes": size,
            "local_sha256": checksum(path),
        }
        if path.suffix.lower() == ".csv":
            item.update(csv_inventory(path))
        files.append(item)
    result = {
        "schema": "local-public-lora-bundle-review-v1",
        "source_doi_claimed": "10.17632/jm58sggtj7.2",
        "capture_type_claimed": "point-to-point LoRa vehicle CAN telemetry",
        "acquisition": "investigator_supplied_local_files",
        "locally_computed_checksums": True,
        "publisher_sha256_independently_verified": False,
        "source_origin_independently_verified": False,
        "dataset_version_independently_verified": False,
        "is_multi_uav_fanet_evidence": False,
        "independent_sender_tx_inventory_verified": False,
        "loss_denominator_established": False,
        "can_use_as_measured_uav_replay_results": False,
        "files": files,
    }
    if mapping is not None:
        result["column_mapping_review"] = review_mapping(mapping, result)
    return result


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input-dir", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--mapping", type=Path)
    args = parser.parse_args()
    try:
        source_paths = {
            (args.input_dir / name).resolve() for name in REQUIRED + OPTIONAL
        }
        if args.mapping is not None:
            source_paths.add(args.mapping.resolve())
        if args.output.resolve() in source_paths:
            raise ValueError("review output must not overwrite a source file")
        report = inspect_local_bundle(args.input_dir, mapping=args.mapping)
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(
            json.dumps(report, indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )
    except (OSError, ValueError, TypeError, csv.Error) as exc:
        parser.error(str(exc))
    print(json.dumps(report, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
