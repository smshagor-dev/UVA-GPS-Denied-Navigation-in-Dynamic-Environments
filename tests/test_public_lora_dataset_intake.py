"""Offline regression tests for public LoRa metadata and checksum intake."""

import hashlib
import io
import json
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

from scripts import public_lora_dataset_intake as intake


def fixture_file(payload=b"flight-demo-fixture-only"):
    return {
        "filename": "measurements.csv",
        "size": len(payload),
        "content_details": {
            "sha256_hash": hashlib.sha256(payload).hexdigest(),
            "size": len(payload),
            "download_url": "https://data.mendeley.com/content/fixture.csv",
        },
    }


class PublicRadioDatasetIntakeTests(unittest.TestCase):
    def metadata(self, files=None):
        return {
            "id": intake.DATASET_ID,
            "version": intake.VERSION,
            "files": files if files is not None else [fixture_file()],
        }

    def test_approved_metadata_and_manifest(self):
        data = self.metadata()
        with tempfile.TemporaryDirectory() as directory:
            with patch.object(intake, "read_catalog", return_value=data):
                report = intake.acquire(Path(directory))
            saved = json.loads(
                (Path(directory) / "public-dataset-integrity.json").read_text(
                    encoding="utf-8"
                )
            )
        self.assertFalse(report["is_uav_fanet_capture"])
        self.assertFalse(report["full_sender_tx_inventory_verified"])
        self.assertEqual(saved["dataset_version"], 2)
        self.assertEqual(report["files"][0]["filename"], "measurements.csv")

    def test_verified_download_then_reuse(self):
        payload = b"synthetic-radio-data-not-a-field-trial"
        item = intake.catalog_files(self.metadata([fixture_file(payload)]))[0]
        with tempfile.TemporaryDirectory() as directory:
            folder = Path(directory)
            with patch.object(intake, "open_url", return_value=io.BytesIO(payload)):
                first = intake.stream_verified_file(item, folder)
            second = intake.stream_verified_file(item, folder)
            self.assertEqual((folder / item["filename"]).read_bytes(), payload)
        self.assertFalse(first["reused"])
        self.assertTrue(second["reused"])

    def test_tampered_download_never_published(self):
        item = intake.catalog_files(self.metadata())[0]
        with tempfile.TemporaryDirectory() as directory:
            folder = Path(directory)
            wrong = b"x" * item["size"]
            with patch.object(intake, "open_url", return_value=io.BytesIO(wrong)):
                with self.assertRaisesRegex(ValueError, "SHA-256"):
                    intake.stream_verified_file(item, folder)
            self.assertFalse((folder / item["filename"]).exists())

    def test_existing_tampered_capture_is_not_overwritten(self):
        item = intake.catalog_files(self.metadata())[0]
        with tempfile.TemporaryDirectory() as directory:
            folder = Path(directory)
            target = folder / item["filename"]
            target.write_text("tampered", encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "existing file"):
                intake.stream_verified_file(item, folder)
            self.assertEqual(target.read_text(encoding="utf-8"), "tampered")

    def test_bad_origin_rejected(self):
        for url in (
            "http://data.mendeley.com/files/1",
            "https://127.0.0.1/file",
            "https://evil.example.com/payload",
            "https://user:password@api.data.mendeley.com/file",
            "file:///etc/passwd",
        ):
            self.assertFalse(intake.trusted_download_url(url), url)

    def test_unsigned_metadata_or_unknown_files_not_accepted(self):
        bad = fixture_file()
        bad["content_details"]["sha256_hash"] = "unverified"
        with self.assertRaisesRegex(ValueError, "SHA-256"):
            intake.catalog_files(self.metadata([bad]))
        missing = fixture_file()
        missing["filename"] = "../private.csv"
        with self.assertRaisesRegex(ValueError, "no recognized"):
            intake.catalog_files(self.metadata([missing]))

    def test_catalog_identity_and_version_are_pinned(self):
        with patch.object(
            intake,
            "open_url",
            return_value=io.BytesIO(
                json.dumps({"id": "another", "version": 2}).encode()
            ),
        ):
            with self.assertRaisesRegex(ValueError, "dataset ID"):
                intake.read_catalog()
        with patch.object(
            intake,
            "open_url",
            return_value=io.BytesIO(
                json.dumps({"id": intake.DATASET_ID, "version": 3}).encode()
            ),
        ):
            with self.assertRaisesRegex(ValueError, "published version"):
                intake.read_catalog()

    def test_metadata_cannot_replace_verifiable_raw_source(self):
        result = self.metadata([fixture_file()])
        with patch.object(intake, "read_catalog", return_value=result):
            with tempfile.TemporaryDirectory() as directory:
                report = intake.acquire(Path(directory), download=False)
        self.assertFalse(report["physical_field_provenance_independently_verified"])
        self.assertFalse(report["radio_packet_loss_causal_attribution_established"])


if __name__ == "__main__":
    unittest.main()
