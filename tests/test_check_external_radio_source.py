"""Offline checks for honest external-radio source acquisition status."""

import contextlib
import io
import json
import sys
import tempfile
import unittest
import urllib.error
from pathlib import Path
from unittest.mock import patch

from scripts import check_external_radio_source as probe


class PublicSourceProbeTests(unittest.TestCase):
    def test_source_access_failure_records_no_measured_evidence(self):
        with tempfile.TemporaryDirectory() as directory:
            target = Path(directory)
            with patch.object(probe, "acquire", side_effect=OSError("offline")):
                result = probe.attempt(target)
            report = json.loads((target / probe.REPORT).read_text(encoding="utf-8"))
        self.assertEqual(result["result"], "source_unavailable_or_incomplete")
        self.assertEqual(result["verified_download_count"], 0)
        self.assertFalse(report["is_uav_fanet_evidence"])
        self.assertFalse(report["replay_false_rejection_measured"])
        self.assertFalse(report["physical_origin_independently_verified"])

    def test_successful_mocked_download_still_not_fanet_evidence(self):
        meta = {
            "files": [{"filename": "measurements.csv"}],
            "verified_downloads": [
                {"filename": "measurements.csv"},
                {"filename": "packet_loss_by_config.csv"},
            ],
        }
        inventory = {
            "contents": [
                {
                    "filename": "measurements.csv",
                    "column_names": ["fixture_only"],
                    "data_row_count": 2,
                }
            ]
        }
        with tempfile.TemporaryDirectory() as directory:
            target = Path(directory)
            with patch.object(probe, "acquire", return_value=meta):
                with patch.object(probe, "inspect_schema", return_value=inventory):
                    result = probe.attempt(target)
            schema = json.loads(
                (target / "public-lora-schema.json").read_text(encoding="utf-8")
            )
        self.assertEqual(result["result"], "publisher_files_and_csv_schema_verified")
        self.assertEqual(result["verified_download_count"], 2)
        self.assertTrue(result["schema_inventory_available"])
        self.assertEqual(schema["contents"][0]["data_row_count"], 2)
        self.assertFalse(result["is_uav_fanet_evidence"])
        self.assertFalse(result["sender_tx_inventory_completeness_verified"])

    def test_metadata_only_keeps_no_verified_download_claim(self):
        with tempfile.TemporaryDirectory() as directory:
            folder = Path(directory)
            with patch.object(
                probe, "acquire", return_value={"files": [{"filename": "x"}]}
            ) as source:
                result = probe.attempt(folder, download=False)
        self.assertEqual(result["result"], "publisher_metadata_available")
        self.assertEqual(result["verified_download_count"], 0)
        self.assertFalse(result["schema_inventory_available"])
        source.assert_called_once_with(folder, download=False)

    def test_signed_url_is_not_leaked_to_report(self):
        secret = "SENSITIVE-SIGNED-CREDENTIAL"
        error = urllib.error.HTTPError(
            "https://storage.googleapis.com/x?" + secret,
            403,
            "Forbidden",
            None,
            None,
        )
        with tempfile.TemporaryDirectory() as directory:
            with patch.object(probe, "acquire", side_effect=error):
                result = probe.attempt(Path(directory))
                payload = (Path(directory) / probe.REPORT).read_text(encoding="utf-8")
        self.assertEqual(result["http_status"], 403)
        self.assertEqual(result["result"], "publisher_access_denied")
        self.assertEqual(
            result["access_blocker"],
            "publisher_authentication_or_permissions_required",
        )
        self.assertNotIn(secret, payload)

    def test_unauthorized_publisher_access_does_not_certify_source(self):
        error = urllib.error.HTTPError(
            "https://api.data.mendeley.com/datasets/jm58sggtj7",
            401,
            "Unauthorized",
            None,
            None,
        )
        with tempfile.TemporaryDirectory() as directory:
            with patch.object(probe, "acquire", side_effect=error):
                report = probe.attempt(Path(directory))
        self.assertEqual(report["http_status"], 401)
        self.assertEqual(report["result"], "publisher_access_denied")
        self.assertFalse(report["schema_inventory_available"])
        self.assertEqual(report["verified_download_count"], 0)

    def test_strict_mode_fails_when_external_source_unavailable(self):
        with tempfile.TemporaryDirectory() as directory:
            with patch.object(
                sys, "argv", ["probe", "--output-dir", directory, "--strict"]
            ):
                with patch.object(probe, "acquire", side_effect=OSError("offline")):
                    with contextlib.redirect_stdout(io.StringIO()):
                        code = probe.main()
        self.assertEqual(code, 1)


if __name__ == "__main__":
    unittest.main()
