"""Regression tests for audit-linked Reviewer 3 evidence reports."""

import csv
import hashlib
import json
import tempfile
import unittest
from pathlib import Path

from scripts.reviewer_rf_evidence_dossier import (
    markdown_report,
    verify_dossier,
)


class ReviewerEvidenceDossierTests(unittest.TestCase):
    def make_data(self, root: Path) -> tuple[Path, Path]:
        entries = []
        index = []
        for trial in range(2):
            folder = root / f"run-{trial}"
            folder.mkdir()
            tx = folder / "tx.csv"
            rx = folder / "rx.csv"
            for path, time_key, rows in (
                (
                    tx,
                    "tx_ms",
                    [
                        (7, 1, sequence, sequence * 20 + trial)
                        for sequence in (1, 2, 3, 4)
                    ],
                ),
                (
                    rx,
                    "rx_ms",
                    [
                        (7, 1, 2, 100 + trial),
                        (7, 1, 1, 110 + trial),
                        (7, 1, 3, 120 + trial),
                    ],
                ),
            ):
                with path.open("w", encoding="utf-8", newline="") as stream:
                    writer = csv.writer(stream)
                    writer.writerow(["sender_id", "epoch", "sequence", time_key])
                    writer.writerows(rows)
            flight = f"synthetic-{trial}"
            plan = {
                "flight_id": flight,
                "source_doi": "doi:test-fixture",
                "protocol_reference": "synthetic-fixture-only",
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
                    {"sender_id": 7, "sequence_first": 1, "sequence_last": 4}
                ],
            }
            plan_path = folder / "plan.json"
            plan_path.write_text(json.dumps(plan), encoding="utf-8")
            entries.append(
                {
                    "flight_id": flight,
                    "scenario": "synthetic-test",
                    "source_doi": "doi:test-fixture",
                    "epoch": 1,
                    "tx": str(tx.relative_to(root)),
                    "rx": str(rx.relative_to(root)),
                }
            )
            index.append(
                {"flight_id": flight, "plan": str(plan_path.relative_to(root))}
            )
        manifest = root / "paired.json"
        plans = root / "plans.json"
        manifest.write_text(json.dumps(entries), encoding="utf-8")
        plans.write_text(json.dumps(index), encoding="utf-8")
        return manifest, plans

    def test_report_is_explicitly_provisional(self):
        with tempfile.TemporaryDirectory() as directory:
            manifest, plans = self.make_data(Path(directory))
            result = verify_dossier(manifest, plans)
            report = markdown_report(result)
        self.assertTrue(result["software_evidence_consistent"])
        self.assertFalse(result["safe_to_claim_measured_rf_loss"])
        self.assertFalse(result["physical_capture_provenance_verified"])
        self.assertEqual(len(result["flight_audits"]), 2)
        self.assertEqual(
            result["paired_descriptive_statistics"]["scenarios"][
                "synthetic-test"
            ]["independent_flight_count_self_declared"],
            2,
        )
        self.assertIn("NOT VERIFIED AS EMPIRICAL RF EVIDENCE", report)
        self.assertIn("25.00%", report)

    def test_missing_plan_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            manifest, plans = self.make_data(Path(directory))
            entries = json.loads(plans.read_text(encoding="utf-8"))
            plans.write_text(json.dumps(entries[:1]), encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "match exactly"):
                verify_dossier(manifest, plans)

    def test_reused_packet_plan_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            manifest, plans = self.make_data(Path(directory))
            entries = json.loads(plans.read_text(encoding="utf-8"))
            entries[1]["plan"] = entries[0]["plan"]
            plans.write_text(json.dumps(entries), encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "same packet plan"):
                verify_dossier(manifest, plans)

    def test_capture_change_detected_from_original_hash(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            manifest, plans = self.make_data(root)
            with (root / "run-0" / "tx.csv").open(
                "a", encoding="utf-8"
            ) as handle:
                handle.write("7,1,5,200\n")
            with self.assertRaisesRegex(ValueError, "SHA-256 mismatch"):
                verify_dossier(manifest, plans)

    def test_plan_metadata_mismatch_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            manifest, plans = self.make_data(root)
            plan = root / "run-0" / "plan.json"
            content = json.loads(plan.read_text(encoding="utf-8"))
            content["source_doi"] = "doi:wrong-source"
            plan.write_text(json.dumps(content), encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "source DOI claims"):
                verify_dossier(manifest, plans)


if __name__ == "__main__":
    unittest.main()
