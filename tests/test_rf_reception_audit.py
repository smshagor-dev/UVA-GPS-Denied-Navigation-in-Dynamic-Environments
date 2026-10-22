"""Tests for LoRa receive-only sequence auditing and evidence boundaries."""

import csv
import tempfile
import unittest
from pathlib import Path

from scripts.rf_reception_audit import (
    load_receptions,
    parse_time,
    reception_report,
)

HEADER = [
    "dev_eui",
    "rx_info_gatewayId",
    "f_cnt",
    "deduplication_id",
    "rx_info_time",
    "rx_info_snr",
    "rx_info_rssi",
]


class LoRaReceptionAuditTests(unittest.TestCase):
    def read_rows(self, rows):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "LORAlog.csv"
            with path.open("w", encoding="utf-8", newline="") as stream:
                writer = csv.writer(stream)
                writer.writerow(HEADER)
                writer.writerows(rows)
            return load_receptions(path, device="uav7", gateway="gateway-a")

    def test_reordered_packets_and_repeated_delivery(self):
        rows = [
            ("uav7", "gateway-a", 11, "p11", "2026-08-01T12:00:01Z", 9, -63),
            ("uav7", "gateway-a", 10, "p10", "2026-08-01T12:00:02Z", 8, -70),
            ("uav7", "gateway-a", 10, "p10", "2026-08-01T12:00:03Z", 8, -70),
            ("uav7", "gateway-a", 12, "p12", "2026-08-01T12:00:04Z", 7, -73),
        ]
        events, meta = self.read_rows(rows)
        report = reception_report(
            events, flight="example-only", device="uav7", gateway="gateway-a"
        )
        self.assertEqual(report["received_deliveries"], 4)
        self.assertEqual(report["distinct_received_counters"], 3)
        self.assertEqual(report["reordered_distinct_counters"], 1)
        self.assertEqual(report["strict"]["first_arrival_rejections"], 1)
        self.assertEqual(report["window_64"]["first_arrival_rejections"], 0)
        self.assertEqual(report["duplicate_receptions"], 1)
        self.assertIsNone(report["observed_packet_loss_fraction"])
        self.assertFalse(report["capture_origin_verified"])
        self.assertEqual(meta["snr_samples"], 4)

    def test_per_gateway_and_per_device_isolation(self):
        rows = [
            ("uav7", "gateway-b", 1, "p1", "2026-08-01T12:00:00Z", "", ""),
            ("uav8", "gateway-a", 1, "p2", "2026-08-01T12:00:00Z", "", ""),
            ("uav7", "gateway-a", 3, "p3", "2026-08-01T12:00:01Z", "", ""),
            ("uav7", "gateway-a", 5, "p5", "2026-08-01T12:00:02Z", "", ""),
        ]
        events, _ = self.read_rows(rows)
        self.assertEqual([packet.sequence for packet in events], [3, 5])
        report = reception_report(
            events, flight="f1", device="uav7", gateway="gateway-a"
        )
        self.assertIsNone(report["observed_packet_loss_fraction"])

    def test_counter_reuse_with_distinct_identity_fails_closed(self):
        rows = [
            ("uav7", "gateway-a", 3, "old", "2026-08-01T12:00:00Z", "", ""),
            ("uav7", "gateway-a", 3, "new", "2026-08-01T12:00:01Z", "", ""),
        ]
        with self.assertRaisesRegex(ValueError, "counter reused"):
            self.read_rows(rows)

    def test_invalid_timestamp_and_missing_timezone_fail_closed(self):
        with self.assertRaisesRegex(ValueError, "UTC offset"):
            parse_time("2026-08-01T12:00:00")
        with self.assertRaisesRegex(ValueError, "invalid ISO"):
            parse_time("yesterday")

    def test_malformed_measurements_are_rejected(self):
        rows = [
            ("uav7", "gateway-a", 1, "p1", "2026-08-01T12:00:00Z", "NaN", -50),
        ]
        with self.assertRaisesRegex(ValueError, "nonfinite"):
            self.read_rows(rows)

    def test_empty_receiver_selection_is_error(self):
        rows = [
            ("uav9", "gateway-b", 1, "p1", "2026-08-01T12:00:00Z", "", ""),
        ]
        with self.assertRaisesRegex(ValueError, "no receptions match"):
            self.read_rows(rows)

    def test_reception_gap_is_not_counted_as_packet_loss(self):
        rows = [
            ("uav7", "gateway-a", 2, "p2", "2026-08-01T12:00:00Z", "", ""),
            ("uav7", "gateway-a", 100, "p100", "2026-08-01T12:00:01Z", "", ""),
        ]
        events, _ = self.read_rows(rows)
        report = reception_report(
            events, flight="f1", device="uav7", gateway="gateway-a"
        )
        self.assertFalse(report["empirical_packet_loss_supported"])
        self.assertIsNone(report["observed_packet_loss_fraction"])
        self.assertEqual(report["distinct_received_counters"], 2)


if __name__ == "__main__":
    unittest.main()
