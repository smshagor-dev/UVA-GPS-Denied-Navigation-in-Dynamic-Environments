"""Regression tests for raw RF captures checked against an explicit packet plan."""

import csv
import hashlib
import json
import tempfile
import unittest
from pathlib import Path

from scripts.rf_capture_plan_audit import audit


class CapturePlanAuditTests(unittest.TestCase):
    def make_plan(self, folder: Path) -> tuple[Path, dict]:
        tx = folder / "tx.csv"
        rx = folder / "rx.csv"
        for path, key, rows in (
            (
                tx,
                "tx_ms",
                [
                    (7, 1, 1, 10),
                    (7, 1, 2, 20),
                    (7, 1, 3, 30),
                    (8, 1, 10, 15),
                    (8, 1, 11, 25),
                ],
            ),
            (
                rx,
                "rx_ms",
                [
                    (7, 1, 2, 40),
                    (7, 1, 1, 50),
                    (7, 1, 3, 60),
                    (8, 1, 11, 70),
                    (7, 1, 2, 80),
                ],
            ),
        ):
            with path.open("w", encoding="utf-8", newline="") as handle:
                writer = csv.writer(handle)
                writer.writerow(["sender_id", "epoch", "sequence", key])
                writer.writerows(rows)
        spec = {
            "flight_id": "synthetic-unit-test-only",
            "source_doi": "doi:unit-test-fixture",
            "protocol_reference": "unit-test-protocol-not-preregistered",
            "epoch": 1,
            "tx": {
                "file": tx.name,
                "sha256": hashlib.sha256(tx.read_bytes()).hexdigest(),
            },
            "rx": {
                "file": rx.name,
                "sha256": hashlib.sha256(rx.read_bytes()).hexdigest(),
            },
            "planned_senders": [
                {"sender_id": 7, "sequence_first": 1, "sequence_last": 3},
                {"sender_id": 8, "sequence_first": 10, "sequence_last": 11},
            ],
        }
        path = folder / "plan.json"
        path.write_text(json.dumps(spec), encoding="utf-8")
        return path, spec

    def test_exact_plan_coverage_but_never_claims_origin(self):
        with tempfile.TemporaryDirectory() as directory:
            plan, _ = self.make_plan(Path(directory))
            result = audit(plan)
        self.assertTrue(result["plan_coverage_verified"])
        self.assertEqual(result["plan_expected_packet_count"], 5)
        self.assertEqual(result["policy_analysis"]["trace"]["received_distinct"], 4)
        self.assertEqual(
            result["policy_analysis"]["trace"]["observed_loss_fraction"], 0.2
        )
        self.assertFalse(result["capture_provenance_independently_verified"])
        self.assertFalse(result["physical_rf_origin_verified"])
        self.assertFalse(result["protocol_was_preregistered_verified"])

    def test_tampered_raw_file_fails_sha256(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            plan, _ = self.make_plan(root)
            with (root / "tx.csv").open("a", encoding="utf-8") as stream:
                stream.write("7,1,4,90\n")
            with self.assertRaisesRegex(ValueError, "SHA-256 mismatch"):
                audit(plan)

    def test_missing_sender_packet_fails_plan_coverage(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            plan, spec = self.make_plan(root)
            tx = root / "tx.csv"
            rows = tx.read_text(encoding="utf-8").splitlines()
            tx.write_text("\n".join(rows[:-1]) + "\n", encoding="utf-8")
            spec["tx"]["sha256"] = hashlib.sha256(tx.read_bytes()).hexdigest()
            plan.write_text(json.dumps(spec), encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "1 missing"):
                audit(plan)

    def test_overlapping_declared_plan_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            plan, spec = self.make_plan(Path(directory))
            spec["planned_senders"].append(
                {"sender_id": 7, "sequence_first": 2, "sequence_last": 3}
            )
            plan.write_text(json.dumps(spec), encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "overlapping"):
                audit(plan)

    def test_unknown_reception_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            plan, spec = self.make_plan(root)
            rx = root / "rx.csv"
            with rx.open("a", encoding="utf-8") as stream:
                stream.write("7,1,99,100\n")
            spec["rx"]["sha256"] = hashlib.sha256(rx.read_bytes()).hexdigest()
            plan.write_text(json.dumps(spec), encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "no matching transmitted"):
                audit(plan)

    def test_boolean_sequence_bound_is_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            plan, spec = self.make_plan(Path(directory))
            spec["planned_senders"][0]["sequence_first"] = True
            plan.write_text(json.dumps(spec), encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "must be integers"):
                audit(plan)


if __name__ == "__main__":
    unittest.main()
