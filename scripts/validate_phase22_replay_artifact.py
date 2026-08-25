#!/usr/bin/env python3

import json
import sys
from pathlib import Path


def expect(condition, message, errors):
    if not condition:
        errors.append(message)


def require_field(obj, field, expected_type, prefix, errors):
    if field not in obj:
        errors.append(f"{prefix}: missing field '{field}'")
        return None
    value = obj[field]
    if expected_type is not None and not isinstance(value, expected_type):
        errors.append(f"{prefix}: field '{field}' has wrong type")
        return None
    return value


def main() -> int:
    if len(sys.argv) != 2:
        print("usage: validate_phase22_replay_artifact.py <artifact.json>", file=sys.stderr)
        return 2

    artifact_path = Path(sys.argv[1])
    payload = json.loads(artifact_path.read_text(encoding="utf-8"))
    errors = []

    expect(payload.get("schema_version") == 2, "root: schema_version must equal 2", errors)
    expect(payload.get("phase") == 22, "root: phase must equal 22", errors)
    results = require_field(payload, "results", list, "root", errors) or []

    accepted = {
        "straight_track_update",
        "turning_track_update",
        "long_track_update",
        "noisy_track_update",
        "multi_feature_stack",
    }

    required_fields = {
        "scenario": str,
        "deterministic": bool,
        "deterministic_runs": int,
        "active_equivalent": bool,
        "shadow_queue_drained": bool,
        "state_finite": bool,
        "covariance_finite": bool,
        "covariance_symmetric": bool,
        "covariance_psd": bool,
        "attempted_updates": int,
        "applied_updates": int,
        "rejected_updates": int,
        "rejection_reason": str,
        "chi_square_dof": int,
        "chi_square_threshold_used": (int, float),
        "shadow_only_feature_update": bool,
        "active_phase22_counters_present": bool,
        "estimator_state_rollback_verified": bool,
        "feature_lifecycle_verified": bool,
        "diagnostics_update_verified": bool,
        "rollback_verified": bool,
        "final_status": str,
        "shadow_only_evidence": str,
        "active_equivalence_method": str,
    }

    for entry in results:
        scenario = entry.get("scenario", "<unknown>")
        prefix = f"scenario:{scenario}"
        for field, expected_type in required_fields.items():
            require_field(entry, field, expected_type, prefix, errors)

        expect(entry.get("deterministic") is True, f"{prefix}: deterministic must be true", errors)
        expect(entry.get("active_equivalent") is True, f"{prefix}: active_equivalent must be true", errors)
        expect(entry.get("shadow_queue_drained") is True, f"{prefix}: shadow_queue_drained must be true", errors)
        expect(entry.get("state_finite") is True, f"{prefix}: state_finite must be true", errors)
        expect(entry.get("covariance_finite") is True, f"{prefix}: covariance_finite must be true", errors)
        expect(entry.get("covariance_symmetric") is True, f"{prefix}: covariance_symmetric must be true", errors)
        expect(entry.get("covariance_psd") is True, f"{prefix}: covariance_psd must be true", errors)
        expect(bool(entry.get("shadow_only_evidence")), f"{prefix}: shadow_only_evidence missing", errors)
        expect(bool(entry.get("active_equivalence_method")), f"{prefix}: active_equivalence_method missing", errors)
        expect(entry.get("final_status") == "PASS", f"{prefix}: final_status must be PASS", errors)

        applied = entry.get("applied_updates", 0)
        rejected_updates = entry.get("rejected_updates", 0)
        rejection_reason = entry.get("rejection_reason", "")
        rollback_complete = (
            entry.get("estimator_state_rollback_verified") is True
            and entry.get("feature_lifecycle_verified") is True
            and entry.get("diagnostics_update_verified") is True
            and entry.get("rollback_verified") is True
        )

        if scenario in accepted:
            expect(entry.get("attempted_updates", 0) > 0,
                   f"{prefix}: accepted scenario has zero attempted updates", errors)
            expect(applied > 0, f"{prefix}: accepted scenario has zero applied updates", errors)
            expect(rejected_updates == 0,
                   f"{prefix}: accepted scenario has non-zero rejected updates", errors)
            expect(rejection_reason == "none",
                   f"{prefix}: accepted scenario should not report rejection", errors)
        elif scenario == "update_disabled_control":
            expect(applied == 0, f"{prefix}: update-disabled scenario has applied updates", errors)
            expect(rejected_updates == 0,
                   f"{prefix}: update-disabled scenario has rejected updates", errors)
            expect(rejection_reason == "update_disabled",
                   f"{prefix}: update-disabled scenario missing canonical reason", errors)
        elif scenario == "singular_geometry_rejection":
            expect(applied == 0, f"{prefix}: singular geometry scenario has applied updates", errors)
            expect(rejected_updates == 0,
                   f"{prefix}: singular geometry should reject before update attempt", errors)
            expect(rejection_reason == "degenerate_geometry",
                   f"{prefix}: singular geometry missing canonical reason", errors)
            expect(rollback_complete,
                   f"{prefix}: singular geometry rollback evidence incomplete", errors)
        else:
            expect(applied == 0, f"{prefix}: rejected scenario has non-zero applied updates", errors)
            expect(rejected_updates > 0,
                   f"{prefix}: rejected scenario has zero rejected updates", errors)
            expect(rejection_reason not in {"", "none"},
                   f"{prefix}: rejected scenario missing rejection reason", errors)
            expect(rollback_complete,
                   f"{prefix}: rejected scenario rollback evidence incomplete", errors)

        if scenario in {
            "straight_track_update",
            "turning_track_update",
            "long_track_update",
            "noisy_track_update",
            "rejected_track_update",
            "multi_feature_stack",
        }:
            expect(entry.get("chi_square_dof", 0) > 0, f"{prefix}: chi_square_dof missing", errors)
            expect(float(entry.get("chi_square_threshold_used", 0.0)) > 0.0,
                   f"{prefix}: chi_square_threshold_used missing", errors)

    if errors:
        print(json.dumps({"ok": False, "errors": errors}, indent=2))
        return 1

    print(json.dumps({"ok": True, "validated_results": len(results)}, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
