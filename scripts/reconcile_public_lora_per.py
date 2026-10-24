#!/usr/bin/env python3
"""Reconcile LoRa packet loss indicators against a published aggregate CSV.

Publisher mode requires SHA-256 metadata. An explicit provisional local mode
permits exploratory table comparison without claiming publisher authenticity.
Column mapping and packet denominators must never be guessed.
This audit is not an independent radio experiment or a UAV FANET measurement.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
from collections import defaultdict
from decimal import Decimal, InvalidOperation
from pathlib import Path

DATASET_ID = "jm58sggtj7"
VERSION = 2
REQUIRED_FILES = ("measurements.csv", "packet_loss_by_config.csv")
PUBLIC_INDEX = "public-dataset-integrity.json"


def require_string(obj: dict, name: str) -> str:
    value = obj.get(name)
    if not isinstance(value, str) or not value.strip():
        raise ValueError(f"{name} must be a nonempty string")
    return value.strip()


def check_files(folder: Path) -> dict:
    record = json.loads((folder / PUBLIC_INDEX).read_text(encoding="utf-8"))
    if not isinstance(record, dict):
        raise ValueError("publisher integrity index must be a JSON object")
    if record.get("dataset_id") != DATASET_ID:
        raise ValueError("integrity index dataset ID mismatch")
    if type(record.get("dataset_version")) is not int:
        raise ValueError("integrity index version must be an integer")
    if record["dataset_version"] != VERSION:
        raise ValueError("integrity index version mismatch")
    files = record.get("files")
    if not isinstance(files, list):
        raise ValueError("integrity index files must be a list")
    metadata = {}
    for item in files:
        if not isinstance(item, dict):
            raise ValueError("invalid publisher file record")
        name = item.get("filename")
        if name not in REQUIRED_FILES:
            continue
        if name in metadata:
            raise ValueError("duplicate file checksum in publisher index")
        digest, size = item.get("sha256"), item.get("size")
        if not isinstance(digest, str) or len(digest) != 64:
            raise ValueError(f"{name}: missing publisher SHA-256")
        if any(character not in "0123456789abcdef" for character in digest):
            raise ValueError(f"{name}: invalid publisher SHA-256")
        if type(size) is not int or not 0 <= size <= 100 * 1024 * 1024:
            raise ValueError(f"{name}: invalid publisher file size")
        metadata[name] = {"sha256": digest, "size": size}
    if set(metadata) != set(REQUIRED_FILES):
        raise ValueError("publisher index lacks required packet and PER files")

    verified = {}
    for name in REQUIRED_FILES:
        path = folder / name
        sha = hashlib.sha256()
        with path.open("rb") as stream:
            for chunk in iter(lambda: stream.read(1024 * 1024), b""):
                sha.update(chunk)
        if path.stat().st_size != metadata[name]["size"]:
            raise ValueError(f"{name}: file size differs from publisher index")
        if sha.hexdigest() != metadata[name]["sha256"]:
            raise ValueError(f"{name}: SHA-256 differs from publisher index")
        verified[name] = metadata[name]
    return verified


def load_mapping(path: Path) -> dict:
    mapping = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(mapping, dict):
        raise ValueError("column mapping must be a JSON object")
    columns = mapping.get("groups")
    if not isinstance(columns, list) or not columns:
        raise ValueError("groups must be a nonempty list of column mappings")
    labels = set()
    measurement_columns = set()
    summary_columns = set()
    for entry in columns:
        if not isinstance(entry, dict):
            raise ValueError("each group must be a mapping")
        label = require_string(entry, "label")
        left = require_string(entry, "measurement")
        right = require_string(entry, "summary")
        if label in labels or left in measurement_columns or right in summary_columns:
            raise ValueError("duplicate group label or source column")
        labels.add(label)
        measurement_columns.add(left)
        summary_columns.add(right)

    for column in (
        "measurement_lost",
        "summary_count",
        "summary_lost",
        "summary_per",
    ):
        require_string(mapping, column)
    measurement_columns.add(mapping["measurement_lost"])
    summary_columns.update(
        [mapping["summary_count"], mapping["summary_lost"], mapping["summary_per"]]
    )
    if len(measurement_columns) != len(columns) + 1:
        raise ValueError("measurement loss column overlaps a grouping column")
    if len(summary_columns) != len(columns) + 3:
        raise ValueError("published statistics columns must be distinct")
    if mapping.get("summary_per_unit") not in ("fraction", "percent"):
        raise ValueError("summary_per_unit must be fraction or percent")
    return mapping


def rows(path: Path, required: set[str]):
    with path.open("r", encoding="utf-8-sig", newline="") as stream:
        reader = csv.DictReader(stream)
        if not reader.fieldnames or len(reader.fieldnames) != len(
            set(reader.fieldnames)
        ):
            raise ValueError(f"{path.name}: empty or duplicate CSV headings")
        if not required.issubset(reader.fieldnames):
            raise ValueError(f"{path.name}: missing mapped CSV headings")
        for index, row in enumerate(reader, start=2):
            if None in row or any(
                row.get(name) is None or not row[name].strip() for name in required
            ):
                raise ValueError(f"{path.name}:{index}: malformed or empty field")
            yield row


def nonnegative_integer(value: str, source: str) -> int:
    value = value.strip()
    if not value.isascii() or not value.isdecimal():
        raise ValueError(f"{source}: expected a nonnegative integer")
    return int(value)


def packet_rate(value: str, unit: str) -> Decimal:
    try:
        number = Decimal(value.strip())
    except InvalidOperation as exc:
        raise ValueError("published PER must be a decimal number") from exc
    if not number.is_finite():
        raise ValueError("published PER must be finite")
    if unit == "percent":
        number /= Decimal(100)
    if not Decimal(0) <= number <= Decimal(1):
        raise ValueError("published PER must be between zero and one")
    return number


def reconcile(
    folder: Path,
    mapping_path: Path,
    tolerance: Decimal,
    *,
    local_unverified: bool = False,
) -> dict:
    if not isinstance(tolerance, Decimal) or not tolerance.is_finite():
        raise ValueError("PER tolerance must be a finite Decimal")
    if not Decimal(0) <= tolerance <= Decimal("0.01"):
        raise ValueError("PER fraction tolerance must be between 0 and 0.01")
    if type(local_unverified) is not bool:
        raise ValueError("local_unverified must be a boolean")
    if local_unverified:
        # The local bundle reviewer checks source size, CSV shape and symlinks.
        # These are local fingerprints, NOT independent publisher hashes.
        if __package__:
            from .review_local_public_lora_bundle import inspect_local_bundle
        else:
            from review_local_public_lora_bundle import inspect_local_bundle

        local = inspect_local_bundle(folder)
        checksums = {
            item["filename"]: {
                "sha256": item["local_sha256"],
                "size": item["size_bytes"],
            }
            for item in local["files"]
            if item["filename"] in REQUIRED_FILES
        }
    else:
        checksums = check_files(folder)
    mapping = load_mapping(mapping_path)
    dimensions = mapping["groups"]
    observed = defaultdict(lambda: [0, 0])
    measurement_fields = {
        mapping["measurement_lost"],
        *(entry["measurement"] for entry in dimensions),
    }
    for row in rows(folder / REQUIRED_FILES[0], measurement_fields):
        key = tuple(row[entry["measurement"]].strip() for entry in dimensions)
        loss = row[mapping["measurement_lost"]].strip()
        if loss not in ("0", "1"):
            raise ValueError("packet loss flag must be exactly 0 or 1")
        observed[key][0] += 1
        observed[key][1] += int(loss)
    if not observed:
        raise ValueError("packet-level CSV contains no mapped records")

    summary_fields = {
        mapping["summary_count"],
        mapping["summary_lost"],
        mapping["summary_per"],
        *(entry["summary"] for entry in dimensions),
    }
    reported = {}
    for row in rows(folder / REQUIRED_FILES[1], summary_fields):
        key = tuple(row[entry["summary"]].strip() for entry in dimensions)
        if key in reported:
            raise ValueError("duplicate published configuration summary")
        count = nonnegative_integer(row[mapping["summary_count"]], "count")
        lost = nonnegative_integer(row[mapping["summary_lost"]], "lost")
        if count == 0 or lost > count:
            raise ValueError("published packet totals are inconsistent")
        per = packet_rate(row[mapping["summary_per"]], mapping["summary_per_unit"])
        reported[key] = (count, lost, per)
    if not reported:
        raise ValueError("published PER summary contains no mapped records")

    comparisons = []
    for key in sorted(set(observed) | set(reported)):
        calculated = observed.get(key)
        published = reported.get(key)
        found = calculated is not None and published is not None
        if found:
            count, lost = calculated
            published_count, published_lost, published_per = published
            rate = Decimal(lost) / Decimal(count)
            delta = abs(rate - published_per)
            matched = (
                count == published_count
                and lost == published_lost
                and delta <= tolerance
            )
        else:
            matched = False
            rate = None
            delta = None
        comparisons.append(
            {
                "configuration": {
                    entry["label"]: value for entry, value in zip(dimensions, key)
                },
                "packet_rows": calculated[0] if calculated else None,
                "flagged_lost": calculated[1] if calculated else None,
                "computed_per": float(rate) if rate is not None else None,
                "published_packets": published[0] if published else None,
                "published_lost": published[1] if published else None,
                "published_per": float(published[2]) if published else None,
                "fraction_delta": float(delta) if delta is not None else None,
                "matches_published_summary": matched,
            }
        )

    tables_agree = all(
        item["matches_published_summary"] for item in comparisons
    )
    return {
        "schema": "external-lora-packet-per-reconciliation-v1",
        "dataset_doi": "10.17632/jm58sggtj7.2",
        "evidence_class": (
            "unverified locally supplied LoRa CSV tables"
            if local_unverified
            else "external point-to-point LoRa CAN telemetry"
        ),
        "integrity_mode": (
            "local_checksums_only_not_publisher_authenticated"
            if local_unverified
            else "claimed_publisher_index_checksums_matched"
        ),
        "publisher_origin_independently_verified": False,
        "source_files_independently_authenticated": False,
        "safe_to_claim_measured_uav_replay_results": False,
        "is_fanet_field_validation": False,
        "packet_tx_completeness_verified": False,
        "field_origin_independently_verified": False,
        "publisher_checksum_matched_local_files": not local_unverified,
        "supplied_tables_agree": tables_agree,
        "reproduction_matches_all": tables_agree and not local_unverified,
        "loss_denominator": "packet-level CSV rows per configuration only",
        "tolerance_fraction": float(tolerance),
        "source_sha256": checksums,
        "configuration_comparisons": comparisons,
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dataset-dir", type=Path, required=True)
    parser.add_argument("--mapping", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--tolerance", default="0.000001")
    parser.add_argument(
        "--local-unverified",
        action="store_true",
        help="Compare local CSVs provisionally without publisher authentication",
    )
    args = parser.parse_args()
    try:
        tolerance = Decimal(args.tolerance)
        result = reconcile(
            args.dataset_dir,
            args.mapping,
            tolerance,
            local_unverified=args.local_unverified,
        )
        protected = {
            (args.dataset_dir / filename).resolve()
            for filename in (*REQUIRED_FILES, PUBLIC_INDEX)
        }
        protected.add(args.mapping.resolve())
        if args.output.resolve() in protected:
            raise ValueError("output would overwrite an evidence input")
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(
            json.dumps(result, indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )
    except (OSError, ValueError, TypeError, InvalidOperation) as exc:
        parser.error(str(exc))
    print(json.dumps(result, indent=2, sort_keys=True))
    if args.local_unverified:
        print(
            "PROVISIONAL ONLY: local source identity and publisher hashes "
            "have not been independently verified.",
        )
    return 0 if result["supplied_tables_agree"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
