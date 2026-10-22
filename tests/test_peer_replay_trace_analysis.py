"""Deterministic regression tests for trace-based replay model comparisons."""

import csv
import tempfile
import unittest
from pathlib import Path

from scripts.peer_replay_trace_analysis import analyze, read_events


class ReplayTraceAnalysisTests(unittest.TestCase):
    def analyze_rows(self, tx_rows, rx_rows):
        with tempfile.TemporaryDirectory() as directory:
            tx = Path(directory) / "tx.csv"
            rx = Path(directory) / "rx.csv"
            for path, time_key, rows in (
                (tx, "tx_ms", tx_rows),
                (rx, "rx_ms", rx_rows),
            ):
                with path.open("w", encoding="utf-8", newline="") as handle:
                    writer = csv.writer(handle)
                    writer.writerow(["sender_id", "epoch", "sequence", time_key])
                    writer.writerows(rows)
            return analyze(read_events(tx, "tx_ms", 1), read_events(rx, "rx_ms", 1))

    def test_reordering_duplicate_and_loss_are_separate(self):
        tx = [(7, 1, s, s * 10) for s in (10, 11, 12, 13, 14, 15, 16)]
        tx += [(8, 1, s, s * 10) for s in (1, 2)]
        rx = [(7, 1, s, 100 + i) for i, s in enumerate((11, 10, 10, 12, 14, 13, 16))]
        rx += [(8, 1, s, 200 + i) for i, s in enumerate((1, 2))]
        report = self.analyze_rows(tx, rx)
        self.assertEqual(report["trace"]["transmitted_distinct"], 9)
        self.assertEqual(report["trace"]["received_distinct"], 8)
        self.assertEqual(report["trace"]["observed_duplicate_deliveries"], 1)
        self.assertEqual(report["trace"]["unobserved_transmissions"], 1)
        self.assertEqual(report["trace"]["reordered_distinct"], 2)
        self.assertEqual(report["trace"]["observed_p_r"], 0.25)
        self.assertEqual(report["strict"]["accepted_deliveries"], 6)
        self.assertEqual(report["strict"]["benign_reordering_rejections"], 2)
        self.assertEqual(report["window_64"]["accepted_deliveries"], 8)
        self.assertEqual(report["window_64"]["duplicate_rejections"], 1)

    def test_window_still_rejects_benign_outside_window(self):
        tx = [(7, 1, s, s) for s in range(1, 71)]
        rx = [(7, 1, s, i * 10) for i, s in enumerate((1, 70, 2))]
        report = self.analyze_rows(tx, rx)
        self.assertEqual(report["window_64"]["benign_outside_window_rejections"], 1)
        self.assertEqual(report["window_64"]["accepted_deliveries"], 2)
        self.assertEqual(report["trace"]["unobserved_transmissions"], 67)

    def test_independent_senders(self):
        tx = [(7, 1, 10, 1), (8, 1, 1, 2), (7, 1, 11, 3)]
        rx = [(7, 1, 11, 10), (8, 1, 1, 11), (7, 1, 10, 12)]
        report = self.analyze_rows(tx, rx)
        self.assertEqual(report["window_64"]["accepted_deliveries"], 3)
        self.assertEqual(report["strict"]["benign_reordering_rejections"], 1)

    def test_unknown_reception_is_rejected(self):
        with self.assertRaisesRegex(ValueError, "no matching transmitted"):
            self.analyze_rows([(7, 1, 1, 0)], [(7, 1, 2, 10)])

    def test_sequence_wrap_is_rejected_not_silently_accepted(self):
        with self.assertRaisesRegex(ValueError, "nonmonotonic"):
            self.analyze_rows(
                [(7, 1, 4294967295, 0), (7, 1, 0, 1)],
                [(7, 1, 4294967295, 2)],
            )

    def test_invalid_epoch_is_rejected(self):
        with self.assertRaisesRegex(ValueError, "analyze each epoch"):
            self.analyze_rows([(7, 2, 1, 0)], [(7, 2, 1, 1)])


if __name__ == "__main__":
    unittest.main()
