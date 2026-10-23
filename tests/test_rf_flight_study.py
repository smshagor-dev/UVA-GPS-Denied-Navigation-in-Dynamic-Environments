"""Regression tests for reception-only flight summary boundaries."""

import csv
import json
import tempfile
import unittest
from pathlib import Path

from scripts.rf_flight_study import analyze_flights


class FlightStudyTests(unittest.TestCase):
    def make_manifest(self, root: Path, *, duplicate=False):
        items = []
        for number in (1, 2):
            path = root / f"flight{number}.csv"
            with path.open("w", encoding="utf-8", newline="") as stream:
                writer = csv.writer(stream)
                writer.writerow(
                    [
                        "dev_eui",
                        "rx_info_gatewayId",
                        "f_cnt",
                        "deduplication_id",
                        "rx_info_time",
                    ]
                )
                writer.writerows(
                    [
                        ("uav", "gw", 2, f"f{number}-2", "2026-08-01T12:00:00Z"),
                        ("uav", "gw", 1, f"f{number}-1", "2026-08-01T12:00:01Z"),
                    ]
                )
            items.append(
                {
                    "flight_id": "same" if duplicate else f"flight{number}",
                    "csv": path.name,
                    "device": "uav",
                    "gateway": "gw",
                }
            )
        manifest = root / "flights.json"
        manifest.write_text(json.dumps(items), encoding="utf-8")
        return manifest

    def test_two_flights_report_without_packet_loss(self):
        with tempfile.TemporaryDirectory() as directory:
            manifest = self.make_manifest(Path(directory))
            summary = analyze_flights(manifest)
        self.assertEqual(summary["sample_count"], 2)
        self.assertEqual(
            summary["metrics"]["observed_reception_order_p_r"][
                "unweighted_flight_mean"
            ],
            0.5,
        )
        self.assertIsNone(summary["packet_loss_fraction"])
        self.assertFalse(summary["rf_calibration_verified"])
        self.assertEqual(len(summary["flights"]), 2)
        self.assertTrue(all(item["input_sha256"] for item in summary["flights"]))

    def test_repeat_flight_id_fails_closed(self):
        with tempfile.TemporaryDirectory() as directory:
            manifest = self.make_manifest(Path(directory), duplicate=True)
            with self.assertRaisesRegex(ValueError, "duplicate flight ID"):
                analyze_flights(manifest)

    def test_insufficient_runs_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            manifest = Path(directory) / "empty.json"
            manifest.write_text("[]", encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "at least two"):
                analyze_flights(manifest)


if __name__ == "__main__":
    unittest.main()
