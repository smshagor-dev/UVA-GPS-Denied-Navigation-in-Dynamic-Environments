"""Reproducibility and metric-integrity checks for the swarm fault study."""

from __future__ import annotations

import unittest

from research.experiments.swarm_fault_study import (
    POLICIES,
    SCENARIOS,
    PEER_TTL_S,
    Agent,
    Task,
    experiment,
    run_trial,
    select_task,
    safety_filter,
    swept_distance,
)


class SwarmFaultStudyTests(unittest.TestCase):
    def test_matrix_is_balanced(self) -> None:
        runs, report = experiment((4, 8), 2)
        self.assertEqual(len(runs), 2 * 4 * 2 * len(POLICIES))
        self.assertEqual(report["trial_count"], len(runs))
        self.assertEqual({row["policy"] for row in runs}, set(POLICIES))
        self.assertEqual({row["scenario"] for row in runs}, set(SCENARIOS))

    def test_deterministic_simulation_metrics(self) -> None:
        a = run_trial("adaptive_freshness", "combined", 4, 7)
        b = run_trial("adaptive_freshness", "combined", 4, 7)
        # Host-time latency is observational, not deterministic.
        a.pop("decision_latency_p95_wall_ms")
        b.pop("decision_latency_p95_wall_ms")
        self.assertEqual(a, b)

    def test_freshness_changes_eligibility_not_task_oracle(self) -> None:
        agents = [Agent(i, 0.0, 0.0, 90.0) for i in range(4)]
        tasks = [Task(0, 2.0, 0.0), Task(1, 10.0, 0.0)]
        agents[0].cache[1] = (1.0, 0)
        self.assertEqual(
            select_task(agents[0], agents, tasks, "adaptive_unfiltered", 9.0), 1
        )
        self.assertEqual(
            select_task(agents[0], agents, tasks, "adaptive_freshness", 9.0), 0
        )
        self.assertGreater(9.0 - 1.0, PEER_TTL_S)

    def test_completed_tasks_are_not_globally_known(self) -> None:
        agents = [Agent(i, 0.0, 0.0, 90.0) for i in range(4)]
        tasks = [Task(0, 2.0, 0.0, done_at_s=1.0), Task(1, 10.0, 0.0)]
        # World knows task 0 is complete, but agent 0 has not yet learned it.
        self.assertEqual(
            select_task(agents[0], agents, tasks, "adaptive_freshness", 9.0), 0
        )
        agents[0].known_done.add(0)
        self.assertEqual(
            select_task(agents[0], agents, tasks, "adaptive_freshness", 9.0), 1
        )

    def test_fault_outcomes_not_formula_derived(self) -> None:
        row = run_trial("static_roles", "node_failure", 4, 5)
        self.assertIsNotNone(row["fault_orphan_task"])
        self.assertIsNone(row["orphan_task_recovery_sim_s"])
        self.assertFalse(row["orphan_task_recovered"])
        self.assertGreater(row["packets_sent"], row["packets_delivered"])
        self.assertGreater(row["logical_unicast_tx_bytes"], 0)
        self.assertLessEqual(row["tasks_completed"], row["tasks_total"])

    def test_swept_distance_detects_crossing_between_endpoints(self) -> None:
        self.assertAlmostEqual(
            swept_distance((0.0, 0.0), (2.0, 0.0), (1.0, -1.0), (1.0, 1.0)),
            0.0,
        )
        self.assertAlmostEqual(
            swept_distance((0.0, 0.0), (2.0, 0.0), (0.0, 2.0), (2.0, 2.0)),
            2.0,
        )

    def test_idealized_safety_oracle_holds_conflicting_movements(self) -> None:
        agents = [Agent(0, 0.0, 0.0, 90.0), Agent(1, 2.0, 0.0, 90.0)]
        moves, rejected = safety_filter(agents, {0: (3.0, 0.0), 1: (-1.0, 0.0)})
        self.assertGreater(rejected, 0)
        self.assertEqual(moves[0], (0.0, 0.0))
        self.assertEqual(moves[1], (2.0, 0.0))

    def test_safety_reference_has_no_swept_violations_for_seed(self) -> None:
        row = run_trial("adaptive_freshness_safety", "combined", 4, 7)
        self.assertEqual(row["swept_separation_violations"], 0)
        self.assertGreater(row["safety_rejected_steps"], 0)

    def test_reject_invalid_inputs(self) -> None:
        with self.assertRaises(ValueError):
            run_trial("unknown", "nominal", 4, 1)
        with self.assertRaises(ValueError):
            run_trial("static_roles", "nominal", 2, 1)
        with self.assertRaises(ValueError):
            experiment((4,), 0)


if __name__ == "__main__":
    unittest.main()
