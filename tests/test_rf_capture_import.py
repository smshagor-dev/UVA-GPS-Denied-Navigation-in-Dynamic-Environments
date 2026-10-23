"""Ensure RF capture import cannot invent sender-side transmission evidence."""

import csv
import json
import tempfile
import unittest
from pathlib import Path

from scripts.rf_capture_import import run


class CaptureImportTests(unittest.TestCase):
    def fixture(self, root: Path):
        tx = root / "sender.csv"
        rx = root / "gateway.csv"
        for path, header, rows in (
            (
                tx,
                ["packet_id", "sent_timestamp"],
                [
                    (1, "2026-07-01T12:00:00Z"),
                    (2, "2026-07-01T12:00:01Z"),
                    (3, "2026-07-01T12:00:02Z"),
                ],
            ),
            (
                rx,
                ["packet_id", "received_timestamp", "gateway"],
                [
                    (2, "2026-07-01T12:00:03Z", "LW1"),
                    (1, "2026-07-01T12:00:04Z", "LW1"),
                    (1, "2026-07-01T12:00:05Z", "LW2"),
                ],
            ),
        ):
            with path.open("w", encoding="utf-8", newline="") as handle:
                writer = csv.writer(handle)
                writer.writerow(header)
                writer.writerows(rows)

        spec = {
            "flight_id": "synthetic-test-not-a-real-flight",
            "scenario": "fixture",
            "source_doi": "doi:synthetic-test-fixture",
            "sender_id": 7,
            "epoch": 1,
            "tx": {
                "file": tx.name,
                "role": "independent_sender_device_log",
                "sequence_column": "packet_id",
                "time_column": "sent_timestamp",
                "time_format": "iso8601_utc",
            },
            "rx": {
                "file": rx.name,
                "role": "gateway_reception_log",
                "sequence_column": "packet_id",
                "time_column": "received_timestamp",
                "time_format": "iso8601_utc",
                "filter_column": "gateway",
                "filter_value": "LW1",
            },
        }
        spec_file = root / "capture.json"
        spec_file.write_text(json.dumps(spec), encoding="utf-8")
        return spec_file, spec

    def test_independent_sender_and_gateway_logs_normalize(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            spec, _ = self.fixture(root)
            report = run(spec, root / "out")
            tx = (root / "out" / "matched-tx.csv").read_text(encoding="utf-8")
            rx = (root / "out" / "matched-rx.csv").read_text(encoding="utf-8")
            entry = json.loads(
                (root / "out" / "paired-manifest-entry.json").read_text(
                    encoding="utf-8"
                )
            )
        self.assertEqual(report["source_files"]["tx"]["selected"], 3)
        self.assertEqual(report["source_files"]["rx"]["selected"], 2)
        self.assertFalse(report["complete_transmission_inventory_verified"])
        self.assertFalse(report["physical_rf_provenance_verified"])
        self.assertIn("7,1,1,", tx)
        self.assertIn("7,1,2,", rx)
        self.assertEqual(entry[0]["tx"], "matched-tx.csv")

    def test_receiver_side_embedded_send_time_not_independent_tx(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            spec_file, config = self.fixture(root)
            config["tx"] = {
                "file": "gateway.csv",
                "role": "gateway_reception_log",
                "sequence_column": "packet_id",
                "time_column": "received_timestamp",
                "time_format": "iso8601_utc",
            }
            spec_file.write_text(json.dumps(config), encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "independent sender-device"):
                run(spec_file, root / "out")

    def test_same_source_file_rejected_even_if_role_claimed(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            spec_file, config = self.fixture(root)
            config["rx"]["file"] = config["tx"]["file"]
            spec_file.write_text(json.dumps(config), encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "same source file"):
                run(spec_file, root / "out")

    def test_unmatched_gateway_packet_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            spec_file, _ = self.fixture(root)
            with (root / "gateway.csv").open("a", encoding="utf-8") as handle:
                handle.write("50,2026-07-01T12:00:06Z,LW1\n")
            with self.assertRaisesRegex(ValueError, "no matching sender-side"):
                run(spec_file, root / "out")

    def test_invalid_timezone_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            spec_file, _ = self.fixture(root)
            text = (root / "sender.csv").read_text(encoding="utf-8")
            (root / "sender.csv").write_text(
                text.replace("2026-07-01T12:00:01Z", "2026-07-01T12:00:01"),
                encoding="utf-8",
            )
            with self.assertRaisesRegex(ValueError, "explicit timezone offset"):
                run(spec_file, root / "out")


if __name__ == "__main__":
    unittest.main()
