#!/usr/bin/env python3
"""Fetch and integrity-check a fixed public LoRa field-test dataset.

This is point-to-point vehicle CAN telemetry, NOT an airborne FANET capture.
Downloaded files are not independent sender TX/receiver RX ground truth.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import re
import tempfile
import urllib.error
import urllib.parse
import urllib.request
from pathlib import Path

DATASET_ID = "jm58sggtj7"
VERSION = 2
DOI = "10.17632/jm58sggtj7.2"
API = (
    "https://api.data.mendeley.com/datasets/" + DATASET_ID + "?version=" + str(VERSION)
)
FILES = frozenset(
    {
        "measurements.csv",
        "packet_loss_by_config.csv",
        "manifest.csv",
        "experiment_constants.csv",
        "airtime_by_sf_bw_payload.csv",
        "README.md",
    }
)
MAX_BYTES = 100 * 1024 * 1024
SHA = re.compile(r"[0-9a-fA-F]{64}\Z")


def trusted_download_url(url: str) -> bool:
    """Reject untrusted origins and credential-bearing or non-HTTPS URLs."""
    if not isinstance(url, str):
        return False
    parsed = urllib.parse.urlsplit(url)
    if parsed.scheme != "https" or not parsed.hostname:
        return False
    if parsed.username or parsed.password or parsed.port not in (None, 443):
        return False
    host = parsed.hostname.lower()
    return (
        host == "api.data.mendeley.com"
        or host.endswith(".mendeley.com")
        or host == "storage.googleapis.com"
        or host.endswith(".amazonaws.com")
        or host.endswith(".blob.core.windows.net")
    )


class ApprovedRedirects(urllib.request.HTTPRedirectHandler):
    def redirect_request(self, req, fp, code, msg, headers, newurl):
        if not trusted_download_url(newurl):
            raise ValueError("redirect to untrusted download host")
        return super().redirect_request(req, fp, code, msg, headers, newurl)


def open_url(url: str):
    if not trusted_download_url(url):
        raise ValueError("download URL is not a permitted HTTPS location")
    opener = urllib.request.build_opener(ApprovedRedirects())
    request = urllib.request.Request(
        url,
        headers={
            "User-Agent": "Reviewer3-Radio-Evidence-Intake/1.0",
            "Accept": "application/json, application/octet-stream, */*",
        },
    )
    return opener.open(request, timeout=35)


def read_catalog() -> dict:
    with open_url(API) as stream:
        raw = stream.read(4 * 1024 * 1024 + 1)
    if len(raw) > 4 * 1024 * 1024:
        raise ValueError("metadata exceeds 4 MiB")
    metadata = json.loads(raw)
    if not isinstance(metadata, dict):
        raise ValueError("unexpected public dataset metadata")
    if metadata.get("id") != DATASET_ID:
        raise ValueError("dataset ID differs from pinned public dataset")
    if metadata.get("version") != VERSION:
        raise ValueError("published version differs from pinned version")
    return metadata


def catalog_files(metadata: dict) -> list[dict]:
    rows = metadata.get("files")
    if not isinstance(rows, list):
        raise ValueError("API did not provide file metadata")
    result = []
    names = set()
    for row in rows:
        if not isinstance(row, dict):
            raise ValueError("malformed dataset file metadata")
        filename = row.get("filename")
        if filename not in FILES:
            continue
        if filename in names:
            raise ValueError("duplicate dataset filename")
        names.add(filename)
        details = row.get("content_details")
        if not isinstance(details, dict):
            raise ValueError("missing immutable file content details")
        digest = details.get("sha256_hash")
        url = details.get("download_url")
        size = details.get("size", row.get("size"))
        if not isinstance(digest, str) or not SHA.fullmatch(digest):
            raise ValueError(f"{filename}: no verifiable SHA-256 from publisher")
        if not isinstance(size, int) or not 0 <= size <= MAX_BYTES:
            raise ValueError(f"{filename}: invalid file size")
        if not trusted_download_url(url):
            raise ValueError(f"{filename}: untrusted or absent publisher download URL")
        result.append(
            {
                "filename": filename,
                "sha256": digest.lower(),
                "size": size,
                "download_url": url,
            }
        )
    if not result:
        raise ValueError("no recognized public dataset files returned")
    return sorted(result, key=lambda entry: entry["filename"])


def stream_verified_file(item: dict, folder: Path) -> dict:
    """Bounded temp-file download; publish only after digest and size match."""
    folder.mkdir(parents=True, exist_ok=True)
    target = folder / item["filename"]
    if target.exists():
        digest = hashlib.sha256(target.read_bytes()).hexdigest()
        if digest != item["sha256"] or target.stat().st_size != item["size"]:
            raise ValueError(f"{target.name}: existing file differs from publisher")
        return {"filename": target.name, "sha256": digest, "reused": True}
    fd, temporary = tempfile.mkstemp(prefix=".rf-intake-", dir=folder)
    temp = Path(temporary)
    size = 0
    digest = hashlib.sha256()
    try:
        with os.fdopen(fd, "wb") as output:
            with open_url(item["download_url"]) as stream:
                while True:
                    chunk = stream.read(64 * 1024)
                    if not chunk:
                        break
                    size += len(chunk)
                    if size > MAX_BYTES or size > item["size"]:
                        raise ValueError("publisher file exceeds declared size")
                    digest.update(chunk)
                    output.write(chunk)
        if size != item["size"]:
            raise ValueError("downloaded size differs from publisher metadata")
        if digest.hexdigest() != item["sha256"]:
            raise ValueError("downloaded SHA-256 differs from publisher metadata")
        temp.replace(target)
        return {
            "filename": target.name,
            "sha256": digest.hexdigest(),
            "reused": False,
        }
    finally:
        temp.unlink(missing_ok=True)


def acquire(folder: Path, *, download: bool = False) -> dict:
    metadata = read_catalog()
    files = catalog_files(metadata)
    public_index = {
        "source": "Mendeley Data public dataset API",
        "source_doi": DOI,
        "dataset_id": DATASET_ID,
        "dataset_version": VERSION,
        "capture_type": "outdoor point-to-point LoRa vehicle CAN telemetry",
        "is_uav_fanet_capture": False,
        "physical_field_provenance_independently_verified": False,
        "full_sender_tx_inventory_verified": False,
        "radio_packet_loss_causal_attribution_established": False,
        "download_requested": download,
        "files": [
            {
                "filename": item["filename"],
                "sha256": item["sha256"],
                "size": item["size"],
            }
            for item in files
        ],
    }
    if download:
        public_index["verified_downloads"] = [
            stream_verified_file(item, folder) for item in files
        ]
    folder.mkdir(parents=True, exist_ok=True)
    (folder / "public-dataset-integrity.json").write_text(
        json.dumps(public_index, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    return public_index


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument(
        "--download",
        action="store_true",
        help="Download approved files after publisher SHA-256 verification",
    )
    args = parser.parse_args()
    try:
        result = acquire(args.output_dir, download=args.download)
    except (OSError, ValueError, urllib.error.URLError) as exc:
        parser.error(str(exc))
    print(json.dumps(result, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
