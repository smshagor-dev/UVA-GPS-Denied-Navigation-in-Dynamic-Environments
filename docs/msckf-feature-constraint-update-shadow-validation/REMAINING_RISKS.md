# Phase 22.1 Remaining Risks

Date: July 19, 2026

## Blocking Evidence Gaps

- Full compiler matrix was not rerun after the latest code changes.
- Static analysis and sanitizer lanes were not rerun after the latest code changes.
- Phase 15 through Phase 21 regression lanes were not rerun after the latest code changes.
- Replay scenarios `multi_feature_stack`, `singular_geometry_rejection`, and `stale_fej_rejection` do not yet provide required PASS evidence.
- TSan closure was not rerun in the required stable Linux or Docker environment after the latest code changes.

## Scope Risks

- Validation remains software-only.
- No HIL evidence exists for Phase 22.
- No flight-hardware evidence exists for Phase 22.
- Phase 23 marginalization redesign has not started.
- Global mapping, loop closure, pose graph optimization, relocalization, and bundle adjustment remain out of Phase 22 scope.
