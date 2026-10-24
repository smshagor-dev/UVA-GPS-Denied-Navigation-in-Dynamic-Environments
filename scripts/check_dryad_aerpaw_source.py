#!/usr/bin/env python3
"""Acquire published Dryad LoRaWAN gateway files with source SHA-256 checks.

Not a multi-UAV FANET trial, not an independent complete sender TX log, and
not validation of actual RF packet-loss causes or replay false rejections.
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

DOI = "10.5061/dryad.w0vt4b939"
API = "https://datadryad.org/api/v2"
DATASET_URL = API + "/datasets/" + urllib.parse.quote("doi:" + DOI, safe="")
ARCHIVE = "LoRaWAN_Gateway_Performance_and_Vehicle_Tracking_Data.7z"
FILES = frozenset({ARCHIVE, "README.md"})
LIMIT = 20 * 1024 * 1024
METADATA_LIMIT = 2 * 1024 * 1024
HEX = re.compile(r"[a-fA-F0-9]{64}\Z")


def trusted(url: str) -> bool:
    if not isinstance(url, str):
        return False
    try:
        parts = urllib.parse.urlsplit(url)
        if parts.scheme != "https" or parts.username or parts.password:
            return False
        if parts.port not in (None, 443):
            return False
    except ValueError:
        return False
    host = (parts.hostname or "").lower()
    return (
        host == "datadryad.org"
        or host.endswith(".datadryad.org")
        or host == "storage.googleapis.com"
        or host == "s3.amazonaws.com"
        or host.endswith(".s3.amazonaws.com")
        or host.endswith(".cloudfront.net")
    )


class SafeRedirects(urllib.request.HTTPRedirectHandler):
    def redirect_request(self, req, fp, code, msg, headers, newurl):
        if not trusted(newurl):
            raise ValueError("publisher redirect points to an untrusted host")
        return super().redirect_request(req, fp, code, msg, headers, newurl)


def open_url(url: str):
    if not trusted(url):
        raise ValueError("untrusted public Dryad URL")
    opener = urllib.request.build_opener(SafeRedirects())
    request = urllib.request.Request(
        url,
        headers={
            "User-Agent": "Research-LoRaWANGateway-Audit/1.0",
            "Accept": "application/json, application/octet-stream, */*",
        },
    )
    return opener.open(request, timeout=35)


def read_json(url: str) -> dict:
    with open_url(url) as stream:
        raw = stream.read(METADATA_LIMIT + 1)
    if len(raw) > METADATA_LIMIT:
        raise ValueError("Dryad metadata exceeds size limit")
    obj = json.loads(raw)
    if not isinstance(obj, dict):
        raise ValueError("Dryad metadata is not a JSON object")
    return obj


def api_link(href: str, pattern: str) -> str:
    if not isinstance(href, str):
        raise ValueError("missing Dryad API resource link")
    parts = urllib.parse.urlsplit(href)
    if parts.scheme or parts.netloc or parts.query or parts.fragment:
        raise ValueError("unexpected absolute or parameterized API link")
    if not re.fullmatch(pattern, parts.path):
        raise ValueError("Dryad API link does not match expected resource")
    return "https://datadryad.org" + parts.path


def catalog() -> tuple[list[dict], dict]:
    dataset = read_json(DATASET_URL)
    identifier = dataset.get("identifier")
    if identifier not in (DOI, "doi:" + DOI):
        raise ValueError("Dryad returned a different dataset DOI")
    links = dataset.get("_links")
    if not isinstance(links, dict):
        raise ValueError("Dryad dataset links missing")
    current = links.get("stash:version")
    if not isinstance(current, dict):
        raise ValueError("Dryad published version link missing")
    version_url = api_link(
        current.get("href"), r"/api/v2/versions/[1-9][0-9]*"
    )
    version_id = version_url.rsplit("/", 1)[1]
    items = []
    seen = set()
    for page in range(1, 5):
        url = (
            API + "/versions/" + version_id
            + "/files?per_page=100&page=" + str(page)
        )
        response = read_json(url)
        embedded = response.get("_embedded")
        if not isinstance(embedded, dict):
            raise ValueError("Dryad file listing lacks embedded files")
        entries = embedded.get("stash:files")
        if not isinstance(entries, list):
            raise ValueError("Dryad file listing is not a list")
        for entry in entries:
            if not isinstance(entry, dict):
                raise ValueError("invalid Dryad file metadata")
            name = entry.get("path")
            if name not in FILES:
                continue
            if name in seen:
                raise ValueError("duplicate Dryad dataset filename")
            seen.add(name)
            size = entry.get("size")
            digest = entry.get("digest")
            if type(size) is not int or not 0 < size <= LIMIT:
                raise ValueError(f"{name}: invalid publisher file size")
            if not isinstance(digest, str) or not HEX.fullmatch(digest):
                raise ValueError(f"{name}: publisher SHA-256 missing")
            if entry.get("digestType", "").lower() not in ("sha-256", "sha256"):
                raise ValueError(f"{name}: publisher digest is not SHA-256")
            file_links = entry.get("_links")
            if not isinstance(file_links, dict):
                raise ValueError(f"{name}: no publisher download link")
            download = file_links.get("stash:download")
            if not isinstance(download, dict):
                raise ValueError(f"{name}: no publisher download link")
            download_url = api_link(
                download.get("href"), r"/api/v2/files/[1-9][0-9]*/download"
            )
            items.append(
                {
                    "filename": name,
                    "size": size,
                    "sha256": digest.lower(),
                    "download_url": download_url,
                }
            )
        if len(entries) < 100:
            break
    else:
        raise ValueError("Dryad file listing exceeds four pages")
    if seen != FILES:
        raise ValueError("Dryad did not list the expected archive and README")
    return sorted(items, key=lambda x: x["filename"]), {
        "dataset_doi": DOI,
        "dataset_api_url": DATASET_URL,
        "published_version_id": int(version_id),
    }


def download_verified(folder: Path, item: dict) -> dict:
    folder.mkdir(parents=True, exist_ok=True)
    name = item["filename"]
    if name not in FILES:
        raise ValueError("file not on Dryad allowlist")
    target = folder / name
    if target.is_symlink():
        raise ValueError("local source file cannot be a symlink")
    if target.exists():
        if not target.is_file() or target.stat().st_size != item["size"]:
            raise ValueError("existing Dryad file has unexpected size")
        sha = hashlib.sha256()
        with target.open("rb") as stream:
            for chunk in iter(lambda: stream.read(1024 * 1024), b""):
                sha.update(chunk)
        if sha.hexdigest() != item["sha256"]:
            raise ValueError("existing Dryad file has unexpected SHA-256")
        return {"filename": name, "sha256": sha.hexdigest(), "reused": True}
    fd, temp_name = tempfile.mkstemp(prefix=".dryad-", dir=folder)
    temporary = Path(temp_name)
    size, sha = 0, hashlib.sha256()
    try:
        with os.fdopen(fd, "wb") as destination:
            with open_url(item["download_url"]) as stream:
                while True:
                    chunk = stream.read(65536)
                    if not chunk:
                        break
                    size += len(chunk)
                    if size > LIMIT or size > item["size"]:
                        raise ValueError("publisher file exceeds declared size")
                    sha.update(chunk)
                    destination.write(chunk)
        if size != item["size"] or sha.hexdigest() != item["sha256"]:
            raise ValueError("Dryad downloaded file fails size or SHA-256")
        temporary.replace(target)
        return {"filename": name, "sha256": sha.hexdigest(), "reused": False}
    finally:
        temporary.unlink(missing_ok=True)


def verify_local_files(folder: Path, items: list[dict]) -> list[dict]:
    """Check investigator-supplied archive bytes against live Dryad digests.

    Never fall back to downloading a missing local file.
    """
    reviewed = []
    for item in items:
        path = folder / item["filename"]
        if path.is_symlink() or not path.is_file():
            raise ValueError(f"{item['filename']}: local original file is missing")
        if path.stat().st_size != item["size"] or path.stat().st_size > LIMIT:
            raise ValueError(f"{item['filename']}: local publisher size mismatch")
        sha = hashlib.sha256()
        with path.open("rb") as stream:
            for chunk in iter(lambda: stream.read(1024 * 1024), b""):
                sha.update(chunk)
        if sha.hexdigest() != item["sha256"]:
            raise ValueError(f"{item['filename']}: local publisher SHA-256 mismatch")
        reviewed.append({"filename": item["filename"], "sha256": sha.hexdigest()})
    return reviewed


def probe(
    folder: Path, *, download: bool = False, verify_local: bool = False
) -> dict:
    if download and verify_local:
        raise ValueError("download and verify_local are mutually exclusive")
    result = {
        "schema": "dryad-aerpaw-lorawan-source-probe-v1",
        "dataset_doi": DOI,
        "source_type": "LoRaWAN gateway and moving helikite/vehicle telemetry",
        "result": "source_unavailable",
        "verified_download_count": 0,
        "verified_local_file_count": 0,
        "publisher_file_hashes_matched": False,
        "input_origin": "not_obtained",
        "real_multi_uav_fanet_evidence": False,
        "complete_independent_sender_tx_inventory_established": False,
        "observed_rf_loss_causally_attributed": False,
        "replay_false_rejection_measured": False,
    }
    try:
        items, origin = catalog()
        result.update(origin)
        result["publisher_files"] = [
            {k: item[k] for k in ("filename", "size", "sha256")}
            for item in items
        ]
        result["result"] = "publisher_file_manifest_available"
        if download:
            verified = [download_verified(folder, item) for item in items]
            result["verified_download_count"] = len(verified)
            result["result"] = "all_publisher_sha256_matched"
            result["publisher_file_hashes_matched"] = True
            result["input_origin"] = "retrieved_from_dryad_api"
        elif verify_local:
            verified = verify_local_files(folder, items)
            result["verified_local_file_count"] = len(verified)
            result["result"] = "local_files_match_publisher_sha256"
            result["publisher_file_hashes_matched"] = True
            result["input_origin"] = "investigator_supplied_local_files"
    except (OSError, ValueError, TypeError, KeyError, urllib.error.URLError) as exc:
        result["result"] = "source_unavailable_or_incomplete"
        result["failure_class"] = type(exc).__name__
        if isinstance(exc, urllib.error.HTTPError):
            result["http_status"] = exc.code
            if exc.code in (401, 403) and result.get("publisher_files"):
                result["result"] = "publisher_manifest_available_download_denied"
    folder.mkdir(parents=True, exist_ok=True)
    (folder / "dryad-source-status.json").write_text(
        json.dumps(result, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    return result


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", type=Path, required=True)
    modes = parser.add_mutually_exclusive_group()
    modes.add_argument("--download", action="store_true")
    modes.add_argument(
        "--verify-local",
        action="store_true",
        help="Verify pre-existing files against the live publisher hash manifest",
    )
    parser.add_argument("--strict", action="store_true")
    args = parser.parse_args()
    result = probe(
        args.output_dir,
        download=args.download,
        verify_local=args.verify_local,
    )
    print(json.dumps(result, indent=2, sort_keys=True))
    if args.verify_local:
        expected = "local_files_match_publisher_sha256"
    elif args.download:
        expected = "all_publisher_sha256_matched"
    else:
        expected = "publisher_file_manifest_available"
    return 1 if args.strict and result["result"] != expected else 0


if __name__ == "__main__":
    raise SystemExit(main())
