"""Matched RF capture analysis must not manufacture empirical provenance."""

import csv
import json
import tempfile
import unittest
from pathlib import Path

from scripts.rf_paired_flight_study import analyze_manifest


class MatchedFieldCapturesTests(unittest.TestCase):
    def prepare(self, folder: Path, duplicate=False, missing_sender=False):
        items = []
        for index in range(2):
            tx = folder / f"tx-{index}.csv"
            rx = folder / f"rx-{index}.csv"
            with tx.open("w", encoding="utf-8", newline="") as handle:
                writer = csv.writer(handle)
                writer.writerow(["sender_id", "epoch", "sequence", "tx_ms"])
                writer.writerows(
                    [(7, 1, seq, seq * 10 + index) for seq in range(1, 6)]
                )
            with rx.open("w", encoding="utf-8", newline="") as handle:
                writer = csv.writer(handle)
                writer.writerow(["sender_id", "epoch", "sequence", "rx_ms"])
                writer.writerows(
                    [
                        (7, 1, 2, 50 + index),
                        (7, 1, 1, 60 + index),
                        (7, 1, 3, 70 + index),
                        (7, 1, 3, 90 + index),
                        (7, 1, 6 if missing_sender else 5, 100 + index),
                    ]
                )
            items.append(
                {
                    "flight_id": f"flight-{index}",
                    "scenario": "outdoor-mobility",
                    "tx": tx.name,
                    "rx": rx.name,
                    "source_doi": "doi:10.5061/example",
                    "epoch": 1,
                }
            )
        if duplicate:
            items[1]["tx"] = items[0]["tx"]
            items[1]["rx"] = items[0]["rx"]
        manifest = folder / "captures.json"
        manifest.write_text(json.dumps(items), encoding="utf-8")
        return manifest

    def test_paired_trial_metrics_and_bootstrap(self):
        with tempfile.TemporaryDirectory() as directory:
            report = analyze_manifest(self.prepare(Path(directory)))
        self.assertFalse(report["physical_rf_origin_verified"])
        self.assertFalse(report["measured_rf_packet_loss_established"])
        self.assertFalse(report["tx_completeness_verified"])
        scenario = report["scenarios"]["outdoor-mobility"]
        self.assertEqual(scenario["independent_flight_count_self_declared"], 2)
        self.assertEqual(
            scenario["metrics"]["observed_missing_fraction"]["mean_of_flights"],
            0.2,
        )
        self.assertEqual(
            scenario["metrics"]["observed_reordering_fraction"]["mean_of_flights"],
            0.25,
        )
        self.assertEqual(
            scenario["metrics"]["strict_first_arrival_rejection_fraction"]["mean_of_flights"],
            0.25,
        )
        self.assertEqual(
            scenario["metrics"]["window_first_arrival_rejection_fraction"]["mean_of_flights"],
            0.0,
        )
        self.assertEqual(
            len(scenario["metrics"]["observed_missing_fraction"]["bootstrap_95pct"]),
            2,
        )

    def test_repeated_capture_not_two_independent_trials(self):
        with tempfile.TemporaryDirectory() as directory:
            manifest = self.prepare(Path(directory), duplicate=True)
            with self.assertRaisesRegex(ValueError, "duplicate TX/RX capture"):
                analyze_manifest(manifest)

    def test_unknown_reception_is_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            manifest = self.prepare(Path(directory), missing_sender=True)
            with self.assertRaisesRegex(ValueError, "no matching transmitted"):
                analyze_manifest(manifest)

    def test_single_trial_is_not_statistically_replicated(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            manifest = self.prepare(root)
            records = json.loads(manifest.read_text(encoding="utf-8"))
            records[1]["scenario"] = "different-link"
            manifest.write_text(json.dumps(records), encoding="utf-8")
            report = analyze_manifest(manifest)
            self.assertIsNone(
                report["scenarios"]["different-link"]["metrics"][
                    "observed_missing_fraction"
                ]["bootstrap_95pct"]
            )

    def test_epoch_mismatch_fails(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            manifest = self.prepare(root)
            records = json.loads(manifest.read_text(encoding="utf-8"))
            records[0]["epoch"] = 3
            manifest.write_text(json.dumps(records), encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "analyze each epoch"):
                analyze_manifest(manifest)


if __name__ == "__main__":
    unittest.main()
