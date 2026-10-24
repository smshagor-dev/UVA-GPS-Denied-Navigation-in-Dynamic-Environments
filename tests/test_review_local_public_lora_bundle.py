"""Tests for offline LoRa bundle review when the publisher API requires auth."""

import csv
import json
import tempfile
import unittest
from pathlib import Path

from scripts.review_local_public_lora_bundle import inspect_local_bundle


class LocalPublicLoRaBundleTests(unittest.TestCase):
    def make_bundle(self, folder: Path) -> Path:
        with (folder / "measurements.csv").open(
            "w", encoding="utf-8", newline=""
        ) as stream:
            writer = csv.writer(stream)
            writer.writerow(["configuration", "lost"])
            writer.writerows([("A", "0"), ("A", "1"), ("B", "0")])
        with (folder / "packet_loss_by_config.csv").open(
            "w", encoding="utf-8", newline=""
        ) as stream:
            writer = csv.writer(stream)
            writer.writerow(["configuration", "count", "lost", "per"])
            writer.writerows([("A", "2", "1", "0.5"), ("B", "1", "0", "0")])
        mapping = {
            "groups": [
                {
                    "label": "configuration",
                    "measurement": "configuration",
                    "summary": "configuration",
                }
            ],
            "measurement_lost": "lost",
            "summary_count": "count",
            "summary_lost": "lost",
            "summary_per": "per",
            "summary_per_unit": "fraction",
        }
        path = folder / "mapping.json"
        path.write_text(json.dumps(mapping), encoding="utf-8")
        return path

    def test_local_review_reports_only_local_integrity(self):
        with tempfile.TemporaryDirectory() as directory:
            folder = Path(directory)
            mapping = self.make_bundle(folder)
            report = inspect_local_bundle(folder, mapping=mapping)
        self.assertEqual(len(report["files"]), 2)
        self.assertEqual(report["files"][0]["data_row_count"], 3)
        self.assertTrue(report["locally_computed_checksums"])
        self.assertTrue(report["column_mapping_review"]["mapped_csv_headings_present"])
        self.assertFalse(report["publisher_sha256_independently_verified"])
        self.assertFalse(report["source_origin_independently_verified"])
        self.assertFalse(report["can_use_as_measured_uav_replay_results"])

    def test_missing_required_source_is_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            folder = Path(directory)
            self.make_bundle(folder)
            (folder / "measurements.csv").unlink()
            with self.assertRaisesRegex(ValueError, "missing required"):
                inspect_local_bundle(folder)

    def test_duplicate_headers_are_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            folder = Path(directory)
            self.make_bundle(folder)
            csv_path = folder / "measurements.csv"
            csv_path.write_text("configuration,configuration\nA,0\n", encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "duplicate CSV headings"):
                inspect_local_bundle(folder)

    def test_malformed_rows_are_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            folder = Path(directory)
            self.make_bundle(folder)
            csv_path = folder / "measurements.csv"
            with csv_path.open("a", encoding="utf-8") as stream:
                stream.write("A,1,unexpected\n")
            with self.assertRaisesRegex(ValueError, "malformed row"):
                inspect_local_bundle(folder)

    def test_unmatched_mapping_is_flagged_not_guessed(self):
        with tempfile.TemporaryDirectory() as directory:
            folder = Path(directory)
            mapping = self.make_bundle(folder)
            data = json.loads(mapping.read_text(encoding="utf-8"))
            data["measurement_lost"] = "different_column"
            mapping.write_text(json.dumps(data), encoding="utf-8")
            report = inspect_local_bundle(folder, mapping=mapping)
        review = report["column_mapping_review"]
        self.assertFalse(review["mapped_csv_headings_present"])
        self.assertEqual(
            review["missing_mapped_headings"]["measurements.csv"],
            ["different_column"],
        )

    def test_symlink_is_not_followed(self):
        with tempfile.TemporaryDirectory() as directory:
            folder = Path(directory)
            self.make_bundle(folder)
            target = folder / "measurements.csv"
            target.unlink()
            target.symlink_to(folder / "packet_loss_by_config.csv")
            with self.assertRaisesRegex(ValueError, "symlink"):
                inspect_local_bundle(folder)


if __name__ == "__main__":
    unittest.main()
