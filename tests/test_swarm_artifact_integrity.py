"""Regression checks for the archived swarm experiment evidence validator."""

from __future__ import annotations

import copy
import unittest

from research.experiments.swarm_fault_study import experiment
from research.experiments.swarm_stats import analyze
from research.experiments.verify_swarm_artifacts import POLICY_PAIRS, validate


class SwarmArtifactIntegrityTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.records, cls.summary = experiment((4,), 2)
        cls.stats = {
            "policy_pairs": [list(pair) for pair in POLICY_PAIRS],
            "comparisons": analyze(cls.records),
        }

    def check(self, records=None, summary=None, stats=None):
        return validate(
            self.records if records is None else records,
            self.summary if summary is None else summary,
            self.stats if stats is None else stats,
            (4,), 2,
        )

    def test_valid_complete_paired_artifacts(self):
        result = self.check()
        self.assertTrue(result["validated"])
        self.assertEqual(result["trial_count"], 48)
        self.assertEqual(result["paired_comparisons"], 120)
        self.assertEqual(result["evidence_type"], "software_simulation_only")

    def test_missing_trial_is_rejected(self):
        with self.assertRaisesRegex(ValueError, "trial count"):
            self.check(records=self.records[:-1])

    def test_duplicate_pair_is_rejected(self):
        records = copy.deepcopy(self.records)
        records[-1] = copy.deepcopy(records[0])
        with self.assertRaisesRegex(ValueError, "duplicate trial"):
            self.check(records=records)

    def test_tampered_completion_rate_is_rejected(self):
        records = copy.deepcopy(self.records)
        records[0]["task_completion_rate"] = 0.123456
        with self.assertRaisesRegex(ValueError, "task completion denominator"):
            self.check(records=records)

    def test_tampered_group_mean_is_rejected(self):
        summary = copy.deepcopy(self.summary)
        summary["groups"][0]["completion_mean"] = -1.0
        with self.assertRaisesRegex(ValueError, "group mean"):
            self.check(summary=summary)

    def test_tampered_effect_size_is_rejected(self):
        stats = copy.deepcopy(self.stats)
        stats["comparisons"][0]["treatment_minus_control_mean"] = 999.0
        with self.assertRaisesRegex(ValueError, "invalid effect size"):
            self.check(stats=stats)

    def test_nonfinite_p_value_is_rejected(self):
        stats = copy.deepcopy(self.stats)
        stats["comparisons"][0]["exact_sign_test_p"] = float("nan")
        with self.assertRaisesRegex(ValueError, "non-finite"):
            self.check(stats=stats)

    def test_censored_recovery_cannot_be_relabelled(self):
        records = copy.deepcopy(self.records)
        row = next(row for row in records
                   if row["orphan_task_recovered"] is False)
        row["orphan_task_recovered"] = True
        with self.assertRaisesRegex(ValueError, "censored recovery"):
            self.check(records=records)


if __name__ == "__main__":
    unittest.main()
