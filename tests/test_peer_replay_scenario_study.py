"""Regression checks for the synthetic FANET sensitivity study."""

import unittest

from scripts.peer_replay_scenario_study import (
    SCENARIOS,
    confidence_interval,
    expected_window_decisions,
    generate,
    study,
)
from scripts.peer_replay_trace_analysis import Event, analyze


class FanetReplayScenarioTests(unittest.TestCase):
    def test_generation_is_reproducible_and_transmissions_are_valid(self):
        for scenario in SCENARIOS:
            first_tx, first_rx = generate(scenario, 3107, 75)
            next_tx, next_rx = generate(scenario, 3107, 75)
            self.assertEqual(first_tx, next_tx)
            self.assertEqual(first_rx, next_rx)
            result = analyze(first_tx, first_rx, source="synthetic")
            self.assertEqual(
                sum(expected_window_decisions(first_rx)),
                result["window_64"]["accepted_deliveries"],
            )
            self.assertGreater(result["trace"]["transmitted_distinct"], 0)

    def test_reference_window_rejects_duplicate_and_too_old(self):
        packet = lambda seq, at: Event(7, 1, seq, at, at + 2)
        decisions = expected_window_decisions(
            [packet(1, 1), packet(70, 2), packet(1, 3), packet(70, 4)]
        )
        self.assertEqual(decisions, [True, True, False, False])

    def test_window_allows_distinct_reordered_delivery(self):
        packet = lambda seq, at: Event(7, 1, seq, at, at + 2)
        self.assertEqual(
            expected_window_decisions(
                [packet(12, 1), packet(11, 2), packet(11, 3), packet(13, 4)]
            ),
            [True, True, False, True],
        )

    def test_bootstrap_is_repeatable_and_ordered(self):
        values = [0.01, 0.02, 0.04, 0.10]
        lower, upper = confidence_interval(values)
        self.assertEqual([lower, upper], confidence_interval(values))
        self.assertLessEqual(lower, upper)

    def test_scenario_results_are_explicitly_not_rf_evidence(self):
        report = study(seeds=2, packets_per_peer=80)
        self.assertTrue(report["not_rf_measurements"])
        self.assertFalse(report["native_cpp_per_arrival_verified"])
        self.assertEqual(set(report["scenarios"]), set(SCENARIOS))
        for result in report["scenarios"].values():
            self.assertEqual(len(result["runs"]), 2)
            for record in result["runs"]:
                self.assertGreater(record["transmitted"], 0)
                self.assertGreaterEqual(record["loss_fraction"], 0)
                self.assertLessEqual(record["loss_fraction"], 1)


if __name__ == "__main__":
    unittest.main()
