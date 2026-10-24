"""Offline Dryad source tests: no downloaded real research observations."""

import hashlib
import io
import json
import tempfile
import unittest
import urllib.error
from pathlib import Path
from unittest.mock import patch

from scripts import check_dryad_aerpaw_source as source


def record(name: str, contents: bytes, number: int) -> dict:
    return {
        "path": name,
        "size": len(contents),
        "digest": hashlib.sha256(contents).hexdigest(),
        "digestType": "sha-256",
        "_links": {
            "stash:download": {
                "href": f"/api/v2/files/{number}/download"
            }
        },
    }


class DryadSourceTests(unittest.TestCase):
    def setUp(self):
        self.payloads = {
            source.ARCHIVE: b"synthetic-archive-placeholder",
            "README.md": b"synthetic-readme-placeholder",
        }
        self.dataset = {
            "identifier": source.DOI,
            "_links": {"stash:version": {"href": "/api/v2/versions/12345"}},
        }
        self.files = {
            "_embedded": {
                "stash:files": [
                    record(name, data, offset)
                    for offset, (name, data) in enumerate(
                        self.payloads.items(), start=101
                    )
                ]
            }
        }

    def test_public_dataset_catalog_and_sha256_manifest(self):
        with patch.object(
            source, "read_json", side_effect=[self.dataset, self.files]
        ):
            files, metadata = source.catalog()
        self.assertEqual(metadata["published_version_id"], 12345)
        self.assertEqual(len(files), 2)
        self.assertEqual({f["filename"] for f in files}, source.FILES)
        self.assertTrue(all(len(f["sha256"]) == 64 for f in files))

    def test_wrong_dataset_identity_fails_closed(self):
        dataset = dict(self.dataset)
        dataset["identifier"] = "10.5061/dryad.another"
        with patch.object(source, "read_json", return_value=dataset):
            with self.assertRaisesRegex(ValueError, "different dataset DOI"):
                source.catalog()

    def test_metadata_digest_not_sha256_is_rejected(self):
        listing = json.loads(json.dumps(self.files))
        listing["_embedded"]["stash:files"][0]["digestType"] = "md5"
        with patch.object(
            source, "read_json", side_effect=[self.dataset, listing]
        ):
            with self.assertRaisesRegex(ValueError, "not SHA-256"):
                source.catalog()

    def test_invalid_file_link_and_untrusted_host_are_rejected(self):
        self.assertFalse(source.trusted("http://datadryad.org/a"))
        self.assertFalse(source.trusted("https://127.0.0.1/"))
        self.assertFalse(source.trusted("https://datadryad.org.evil.org/a"))
        self.assertFalse(source.trusted("https://foo:bar@datadryad.org/a"))
        with self.assertRaisesRegex(ValueError, "parameterized API link"):
            source.api_link(
                "/api/v2/files/123/download?token=x",
                r"/api/v2/files/[1-9][0-9]*/download",
            )

    def test_bounded_download_and_existing_file_reuse(self):
        payload = self.payloads["README.md"]
        item = {
            "filename": "README.md",
            "size": len(payload),
            "sha256": hashlib.sha256(payload).hexdigest(),
            "download_url": source.API + "/files/105/download",
        }
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            with patch.object(
                source, "open_url", return_value=io.BytesIO(payload)
            ):
                first = source.download_verified(root, item)
            reused = source.download_verified(root, item)
            self.assertEqual((root / "README.md").read_bytes(), payload)
        self.assertFalse(first["reused"])
        self.assertTrue(reused["reused"])

    def test_download_rejects_tampering_and_deletes_temporary(self):
        payload = self.payloads["README.md"]
        item = {
            "filename": "README.md",
            "size": len(payload),
            "sha256": hashlib.sha256(payload).hexdigest(),
            "download_url": source.API + "/files/107/download",
        }
        changed = b"x" * len(payload)
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            with patch.object(
                source, "open_url", return_value=io.BytesIO(changed)
            ):
                with self.assertRaisesRegex(ValueError, "fails size or SHA-256"):
                    source.download_verified(root, item)
            self.assertFalse((root / "README.md").exists())
            self.assertEqual(list(root.glob(".dryad-*")), [])

    def test_availability_is_not_fanet_validation(self):
        entries = [
            {
                "filename": key,
                "size": len(payload),
                "sha256": hashlib.sha256(payload).hexdigest(),
                "download_url": source.API + "/files/101/download",
            }
            for key, payload in self.payloads.items()
        ]
        with tempfile.TemporaryDirectory() as directory:
            folder = Path(directory)
            with patch.object(
                source,
                "catalog",
                return_value=(entries, {"published_version_id": 12345}),
            ):
                with patch.object(
                    source,
                    "open_url",
                    side_effect=[
                        io.BytesIO(self.payloads[x["filename"]]) for x in entries
                    ],
                ):
                    report = source.probe(folder, download=True)
            stored = json.loads(
                (folder / "dryad-source-status.json").read_text(encoding="utf-8")
            )
        self.assertEqual(report["result"], "all_publisher_sha256_matched")
        self.assertTrue(report["publisher_file_hashes_matched"])
        self.assertFalse(stored["real_multi_uav_fanet_evidence"])
        self.assertFalse(stored["complete_independent_sender_tx_inventory_established"])
        self.assertFalse(stored["replay_false_rejection_measured"])

    def test_unavailable_source_never_claims_download_success(self):
        error = urllib.error.HTTPError(
            source.DATASET_URL, 403, "Forbidden", None, None
        )
        with tempfile.TemporaryDirectory() as directory:
            folder = Path(directory)
            with patch.object(source, "catalog", side_effect=error):
                result = source.probe(folder, download=True)
        self.assertEqual(result["result"], "source_unavailable_or_incomplete")
        self.assertEqual(result["http_status"], 403)
        self.assertEqual(result["verified_download_count"], 0)
        self.assertFalse(result["publisher_file_hashes_matched"])


if __name__ == "__main__":
    unittest.main()
