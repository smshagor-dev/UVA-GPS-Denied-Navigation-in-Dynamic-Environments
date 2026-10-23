#!/usr/bin/env python3
"""Build a Reviewer 3 evidence dossier from independently supplied RF captures.

This checks internal consistency and generates explicitly provisional tables.
No file or checksum can independently certify a physical flight or its
pre-registered protocol. The dossier is NOT a ready-to-publish RF result.
"""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path

if __package__:
    from .rf_capture_plan_audit import audit
    from .rf_paired_flight_study import analyze_manifest
else:
    from rf_capture_plan_audit import audit
    from rf_paired_flight_study import analyze_manifest


def read_list(path: Path) -> list:
    value = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(value, list) or not value:
        raise ValueError(f"{path.name}: expected a nonempty JSON list")
    return value


def plan_index(path: Path) -> dict[str, Path]:
    plans = {}
    used_paths = set()
    for entry in read_list(path):
        if not isinstance(entry, dict):
            raise ValueError("each plan entry must be an object")
        flight = entry.get("flight_id")
        file = entry.get("plan")
        if not all(isinstance(s, str) and s.strip() for s in (flight, file)):
            raise ValueError("plan index needs flight_id and plan")
        if flight in plans:
            raise ValueError("repeated flight ID in plan index")
        location = (path.parent / file).resolve()
        if location in used_paths:
            raise ValueError("the same packet plan is assigned to two flights")
        used_paths.add(location)
        plans[flight] = location
    return plans


def verify_dossier(
    capture_manifest: Path, index_file: Path, *, native: Path | None = None
) -> dict:
    entries = read_list(capture_manifest)
    flights = {}
    for entry in entries:
        if not isinstance(entry, dict) or not isinstance(entry.get("flight_id"), str):
            raise ValueError("paired manifest requires a flight ID per entry")
        flight = entry["flight_id"]
        if flight in flights:
            raise ValueError("duplicate paired flight ID")
        flights[flight] = entry

    plans = plan_index(index_file)
    if set(plans) != set(flights):
        raise ValueError("packet plans and paired flights must match exactly")

    verified = []
    for flight, entry in sorted(flights.items()):
        plan_path = plans[flight]
        result = audit(plan_path)
        if result["flight_id"] != flight:
            raise ValueError(f"{flight}: packet plan flight identity mismatch")
        if result["epoch"] != int(entry.get("epoch", 1)):
            raise ValueError(f"{flight}: packet plan epoch mismatch")
        if result["source_doi_claimed"] != entry.get("source_doi"):
            raise ValueError(f"{flight}: source DOI claims mismatch")

        for kind in ("tx", "rx"):
            raw = (capture_manifest.parent / entry[kind]).resolve()
            expected = (
                plan_path.parent
                / json.loads(plan_path.read_text(encoding="utf-8"))[kind]["file"]
            ).resolve()
            if raw != expected:
                raise ValueError(f"{flight}: {kind} does not point to plan source")
            hasher = hashlib.sha256()
            with raw.open("rb") as stream:
                for block in iter(lambda: stream.read(1024 * 1024), b""):
                    hasher.update(block)
            if hasher.hexdigest() != result["input_sha256"][kind]:
                raise ValueError(f"{flight}: {kind} trace digest mismatch")
        verified.append(
            {
                "flight_id": flight,
                "plan_sha256": hashlib.sha256(plan_path.read_bytes()).hexdigest(),
                "tx_sha256": result["input_sha256"]["tx"],
                "rx_sha256": result["input_sha256"]["rx"],
                "planned_packet_count": result["plan_expected_packet_count"],
                "reception_count": result["policy_analysis"]["trace"][
                    "received_distinct"
                ],
                "plan_coverage_checked": True,
                "source_provenance_independent": False,
            }
        )

    summary = analyze_manifest(capture_manifest, native=native)
    by_flight = {
        row["flight_id"]: row
        for scenario in summary["scenarios"].values()
        for row in scenario["flights"]
    }
    for item in verified:
        sample = by_flight[item["flight_id"]]
        if (
            sample["tx_sha256"] != item["tx_sha256"]
            or sample["rx_sha256"] != item["rx_sha256"]
            or sample["transmitted_distinct"] != item["planned_packet_count"]
            or sample["received_distinct"] != item["reception_count"]
        ):
            raise ValueError("paired statistics disagree with capture audit")

    return {
        "schema": "reviewer3-evidence-dossier-v1",
        "status": "internally_consistent_captures_not_certified_empirical",
        "software_evidence_consistent": True,
        "safe_to_claim_measured_rf_loss": False,
        "safe_to_claim_fanet_operational_reliability": False,
        "physical_capture_provenance_verified": False,
        "independent_sender_instrumentation_verified": False,
        "preregistered_protocol_verified": False,
        "physical_trial_independence_verified": False,
        "capture_manifest_sha256": hashlib.sha256(
            capture_manifest.read_bytes()
        ).hexdigest(),
        "plan_index_sha256": hashlib.sha256(index_file.read_bytes()).hexdigest(),
        "flight_audits": verified,
        "paired_descriptive_statistics": summary,
        "reviewer_gaps": [
            "Verify original dataset provenance and collection equipment.",
            "Verify complete sender instrumentation and protocol registration.",
            "Validate truly independent physical trials and RF conditions.",
            "Distinguish application-level missing deliveries from RF loss.",
            "Use verified field results before revising empirical claims.",
        ],
    }


def fmt(value: float | None) -> str:
    return "not estimable" if value is None else f"{value * 100:.2f}%"


def cell(value: str) -> str:
    return value.replace("|", "/").replace("\n", " ").replace("\r", " ")


def markdown_report(report: dict) -> str:
    """A descriptive draft with prominent unsupported-claim boundaries."""
    lines = [
        "# Reviewer 3 — RF capture evidence dossier",
        "",
        "**STATUS: NOT VERIFIED AS EMPIRICAL RF EVIDENCE.**",
        "",
        "This document reports software checks on investigator-supplied data.",
        "The physical origin, pretrial planning and sender completeness have",
        "**not** been independently established. The numbers below must not",
        "be presented as measured FANET performance or flight-safety evidence.",
        "",
        "## Plan-to-capture consistency",
        "",
        "| Declared trial | Planned TX | Distinct RX | Plan check |",
        "| --- | ---: | ---: | --- |",
    ]
    for item in report["flight_audits"]:
        lines.append(
            f"| {cell(item['flight_id'])} | "
            f"{item['planned_packet_count']} | {item['reception_count']} "
            "| internally consistent |"
        )
    lines.extend(
        [
            "",
            "## Descriptive comparison — source claims unverified",
            "",
            "| Scenario | Declared runs | Observed missing | "
            "Reordering | Strict rejection | Window rejection |",
            "| --- | ---: | ---: | ---: | ---: | ---: |",
        ]
    )
    for name, details in sorted(
        report["paired_descriptive_statistics"]["scenarios"].items()
    ):
        metrics = details["metrics"]

        def value(key: str) -> str:
            return fmt(metrics[key]["mean_of_flights"])

        lines.append(
            f"| {cell(name)} | "
            f"{details['independent_flight_count_self_declared']} | "
            f"{value('observed_missing_fraction')} | "
            f"{value('observed_reordering_fraction')} | "
            f"{value('strict_first_arrival_rejection_fraction')} | "
            f"{value('window_first_arrival_rejection_fraction')} |"
        )
    lines.extend(["", "## Still required before manuscript submission", ""])
    for gap in report["reviewer_gaps"]:
        lines.append(f"- {gap}")
    lines.extend(
        [
            "",
            "Missing reception is an end-to-end observation, not proven RF loss.",
            "All provisional estimates derive from the supplied records only.",
            "See the machine-readable JSON for per-flight hashes and bootstrap CIs.",
            "",
        ]
    )
    return "\n".join(lines)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--manifest", type=Path, required=True)
    parser.add_argument("--plans", type=Path, required=True)
    parser.add_argument("--output-json", type=Path, required=True)
    parser.add_argument("--output-md", type=Path, required=True)
    parser.add_argument("--native", type=Path)
    args = parser.parse_args()
    try:
        result = verify_dossier(args.manifest, args.plans, native=args.native)
        if args.output_json.resolve() == args.output_md.resolve():
            raise ValueError("JSON and Markdown outputs must be distinct")
        for output in (args.output_json, args.output_md):
            output.parent.mkdir(parents=True, exist_ok=True)
        args.output_json.write_text(
            json.dumps(result, indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )
        args.output_md.write_text(markdown_report(result), encoding="utf-8")
    except (OSError, ValueError, TypeError, KeyError) as exc:
        parser.error(str(exc))
    print("Draft evidence dossier written; independent RF provenance NOT verified.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
