"""Integrity tests for IEEE baseline reference and paired simulation statistics."""

import unittest

from research.experiments.cbaa_reference_study import (
    auction,
    exact_oracle,
    outcome,
    scores_for,
    trial,
)
from research.experiments.swarm_stats import analyze, bootstrap_ci, exact_signflip


class SwarmAdvancedEvidenceTests(unittest.TestCase):
    def test_exact_assignment_oracle(self):
        value, assignment = exact_oracle([[10.0, 1.0], [5.0, 9.0]])
        self.assertEqual(value, 19.0)
        self.assertEqual(assignment, (0, 1))

    def test_healthy_connected_single_task_auction_converges(self):
        bids = scores_for(8, 11)
        assignments, overhead = auction(bids, "ring", 0.0, 11)
        self.assertNotIn(None, assignments)
        self.assertEqual(len(set(assignments)), 8)
        self.assertLessEqual(overhead["rounds"], 96)

    def test_conflict_adjusted_reward_never_exceeds_oracle(self):
        for topology in ("ring", "complete", "partition"):
            for seed in range(10):
                row = trial(8, topology, 0.1, seed)
                self.assertLessEqual(
                    row["cbaa_style"]["conflict_adjusted_reward"],
                    row["oracle_reward"] + 1e-4,
                )

    def test_duplicates_are_not_rewarded_as_distinct_tasks(self):
        scores = [[10.0, 1.0], [9.0, 2.0]]
        result = outcome(scores, [0, 0])
        self.assertEqual(result["unique_tasks"], 1)
        self.assertEqual(result["duplicate_claims"], 1)
        self.assertEqual(result["conflict_adjusted_reward"], 10.0)

    def test_exact_signflip_and_bootstrap_constant(self):
        self.assertEqual(exact_signflip([0.0, 0.0]), 1.0)
        self.assertEqual(exact_signflip([1.0] * 4), 0.125)
        self.assertEqual(bootstrap_ci([2.0, 2.0], 7), [2.0, 2.0])

    def test_unpaired_trial_is_rejected(self):
        with self.assertRaises(ValueError):
            analyze([{
                "uavs": 4,
                "scenario": "nominal",
                "seed": 0,
                "policy": "adaptive_freshness",
            }])


if __name__ == "__main__":
    unittest.main()
