"""Validate packet-level LoRa PER reconciliation without invented field results."""

import csv
import hashlib
import json
import tempfile
import unittest
from decimal import Decimal
from pathlib import Path

from scripts.inspect_public_lora_schema import inspect_schema
from scripts.reconcile_public_lora_per import reconcile


class PublicLoRaPERReconciliationTests(unittest.TestCase):
    def prepare(self, root: Path) -> Path:
        samples = root / "measurements.csv"
        summary = root / "packet_loss_by_config.csv"
        with samples.open("w", encoding="utf-8", newline="") as stream:
            writer = csv.writer(stream)
            writer.writerow(["distance", "sf", "lost"])
            writer.writerows(
                [
                    ("25", "SF7", "0"),
                    ("25", "SF7", "1"),
                    ("25", "SF7", "0"),
                    ("25", "SF7", "0"),
                    ("50", "SF8", "1"),
                    ("50", "SF8", "1"),
                ]
            )
        with summary.open("w", encoding="utf-8", newline="") as stream:
            writer = csv.writer(stream)
            writer.writerow(["distance_m", "spreading", "count", "lost", "per"])
            writer.writerows(
                [
                    ("25", "SF7", "4", "1", "25"),
                    ("50", "SF8", "2", "2", "100"),
                ]
            )
        files = []
        for path in (samples, summary):
            files.append(
                {
                    "filename": path.name,
                    "sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
                    "size": path.stat().st_size,
                }
            )
        index = {
            "dataset_id": "jm58sggtj7",
            "dataset_version": 2,
            "files": files,
        }
        (root / "public-dataset-integrity.json").write_text(
            json.dumps(index), encoding="utf-8"
        )
        mapping = {
            "groups": [
                {
                    "label": "distance_m",
                    "measurement": "distance",
                    "summary": "distance_m",
                },
                {
                    "label": "spreading",
                    "measurement": "sf",
                    "summary": "spreading",
                },
            ],
            "measurement_lost": "lost",
            "summary_count": "count",
            "summary_lost": "lost",
            "summary_per": "per",
            "summary_per_unit": "percent",
        }
        target = root / "mapping.json"
        target.write_text(json.dumps(mapping), encoding="utf-8")
        return target

    def refresh_checksum(self, root: Path, name: str):
        index = root / "public-dataset-integrity.json"
        metadata = json.loads(index.read_text(encoding="utf-8"))
        for file in metadata["files"]:
            if file["filename"] == name:
                path = root / name
                file["sha256"] = hashlib.sha256(path.read_bytes()).hexdigest()
                file["size"] = path.stat().st_size
        index.write_text(json.dumps(metadata), encoding="utf-8")

    def test_checksum_matched_raw_header_inventory(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            self.prepare(root)
            report = inspect_schema(root)
        self.assertFalse(report["source_is_multi_uav_fanet"])
        self.assertFalse(report["packet_loss_denominator_verified"])
        self.assertEqual(len(report["contents"]), 2)
        self.assertEqual(report["contents"][0]["data_row_count"], 6)
        self.assertEqual(
            report["contents"][0]["column_names"], ["distance", "sf", "lost"]
        )

    def test_header_inventory_rejects_duplicate_headers(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            self.prepare(root)
            samples = root / "measurements.csv"
            samples.write_text(
                samples.read_text(encoding="utf-8").replace(
                    "distance,sf,lost", "distance,sf,sf"
                ),
                encoding="utf-8",
            )
            self.refresh_checksum(root, samples.name)
            with self.assertRaisesRegex(ValueError, "repeated CSV headings"):
                inspect_schema(root)

    def test_header_inventory_checks_original_checksums(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            self.prepare(root)
            samples = root / "measurements.csv"
            samples.write_bytes(samples.read_bytes().replace(b"SF7", b"SF8", 1))
            with self.assertRaisesRegex(ValueError, "SHA-256 differs"):
                inspect_schema(root)

    def test_reproduction_matches_publisher_summary(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            config = self.prepare(root)
            output = reconcile(root, config, Decimal("0.000001"))
        self.assertTrue(output["reproduction_matches_all"])
        self.assertFalse(output["is_fanet_field_validation"])
        self.assertFalse(output["packet_tx_completeness_verified"])
        self.assertEqual(len(output["configuration_comparisons"]), 2)
        first = output["configuration_comparisons"][0]
        self.assertEqual(first["packet_rows"], 4)
        self.assertEqual(first["flagged_lost"], 1)
        self.assertEqual(first["computed_per"], 0.25)

    def test_disagreement_is_reported_not_relabelled_success(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            config = self.prepare(root)
            summary = root / "packet_loss_by_config.csv"
            summary.write_text(
                summary.read_text(encoding="utf-8").replace(",4,1,25", ",4,1,50"),
                encoding="utf-8",
            )
            self.refresh_checksum(root, summary.name)
            result = reconcile(root, config, Decimal("0.000001"))
        self.assertFalse(result["reproduction_matches_all"])
        self.assertFalse(
            result["configuration_comparisons"][0]["matches_published_summary"]
        )

    def test_tampering_is_rejected_before_statistical_comparison(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            config = self.prepare(root)
            samples = root / "measurements.csv"
            original = samples.read_bytes()
            tampered = original.replace(b"25,SF7,0", b"25,SF7,1", 1)
            self.assertNotEqual(original, tampered)
            self.assertEqual(len(original), len(tampered))
            samples.write_bytes(tampered)
            with self.assertRaisesRegex(ValueError, "SHA-256 differs"):
                reconcile(root, config, Decimal("0.000001"))

    def test_appended_packet_is_rejected_by_file_size_check(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            config = self.prepare(root)
            with (root / "measurements.csv").open("a", encoding="utf-8") as stream:
                stream.write("25,SF7,0\n")
            with self.assertRaisesRegex(ValueError, "file size differs"):
                reconcile(root, config, Decimal("0.000001"))

    def test_missing_published_group_is_discrepancy(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            config = self.prepare(root)
            summary = root / "packet_loss_by_config.csv"
            summary.write_text(
                "distance_m,spreading,count,lost,per\n25,SF7,4,1,25\n",
                encoding="utf-8",
            )
            self.refresh_checksum(root, summary.name)
            result = reconcile(root, config, Decimal("0.000001"))
        self.assertFalse(result["reproduction_matches_all"])
        self.assertEqual(
            sum(
                item["matches_published_summary"]
                for item in result["configuration_comparisons"]
            ),
            1,
        )

    def test_loss_flag_must_be_a_binary_indicator(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            config = self.prepare(root)
            samples = root / "measurements.csv"
            samples.write_text(
                samples.read_text(encoding="utf-8").replace("25,SF7,1", "25,SF7,2"),
                encoding="utf-8",
            )
            self.refresh_checksum(root, samples.name)
            with self.assertRaisesRegex(ValueError, "exactly 0 or 1"):
                reconcile(root, config, Decimal("0.000001"))

    def test_mapped_fields_are_not_guessed(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            config = self.prepare(root)
            mapping = json.loads(config.read_text(encoding="utf-8"))
            mapping["measurement_lost"] = "unverified_column"
            config.write_text(json.dumps(mapping), encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "missing mapped CSV headings"):
                reconcile(root, config, Decimal("0.000001"))

    def test_nonfinite_per_and_large_tolerance_are_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            config = self.prepare(root)
            with self.assertRaisesRegex(ValueError, "tolerance"):
                reconcile(root, config, Decimal("0.5"))
            summary = root / "packet_loss_by_config.csv"
            summary.write_text(
                summary.read_text(encoding="utf-8").replace(",4,1,25", ",4,1,NaN"),
                encoding="utf-8",
            )
            self.refresh_checksum(root, summary.name)
            with self.assertRaisesRegex(ValueError, "must be finite"):
                reconcile(root, config, Decimal("0.000001"))


if __name__ == "__main__":
    unittest.main()
