# Phase 5 Implementation Plan

## Objective

Close the production-readiness gaps identified in [PHASE5_GAP_ANALYSIS.md](./PHASE5_GAP_ANALYSIS.md) without introducing new product features or reopening Phase 4.

## Workstreams

### 1. Deployment and containerization

Planned deliverables:

- `Dockerfile`
- `Dockerfile.dev`
- `docker-compose.yml`
- `docker-compose.simulation.yml`
- Phase 5 deployment architecture documentation under `docs/phase5/`

Implementation intent:

- Containerize the Go control plane and Python launcher/dashboard paths in a way that is explicit about simulation versus production-like usage.
- Keep native C++/hardware assumptions documented instead of pretending every hardware path is container-native.
- Reuse the existing dependency model from `.github/actions/setup-linux-deps/action.yml`, `requirements.txt`, and the current Go module.

### 2. Release validation engineering

Planned deliverables:

- `.github/workflows/release-validation.yml`
- supporting validation scripts or documentation only where needed

Implementation intent:

- Add a workflow dedicated to validating release-candidate build, packaging, and runtime expectations.
- Keep it complementary to existing `ci.yml`, `nightly.yml`, and `release.yml`.
- Favor artifact generation and evidence preservation over duplicating every existing CI job.

### 3. Configuration contract hardening

Planned deliverables:

- `config/schema/`
- Phase 5 config reference docs
- any minimal validation helper scripts needed for schema checks

Implementation intent:

- Define machine-readable schemas for tracked JSON configuration files.
- Document required fields, optional fields, environment overrides, and secret-related variables.
- Align schemas with the current runtime parsing logic instead of inventing a disconnected contract.

### 4. Observability and operations

Planned deliverables:

- targeted control-plane or launcher hardening if needed
- observability documentation under `docs/phase5/`

Implementation intent:

- Preserve the existing `/api/v1/health` endpoint and extend only where needed.
- Add readiness/operational diagnostics only if they materially improve deployment supportability.
- Prefer incremental logging improvements and documented log conventions over a large logging rewrite.

### 5. Research and community release documents

Planned deliverables:

- `RESEARCH_RELEASE.md`
- `CODE_OF_CONDUCT.md`
- updates to `CONTRIBUTING.md` if needed
- final Phase 5 validation and readiness reports under `docs/phase5/`

Implementation intent:

- Make research-release expectations explicit: reproducibility scope, safety disclaimers, and artifact boundaries.
- Fill genuine documentation gaps while preserving existing docs that already satisfy the brief.

## Sequencing

1. Establish Phase 5 baseline documentation.
2. Add containerization and deployment-architecture artifacts.
3. Add config schemas and config documentation.
4. Add release-validation workflow.
5. Add observability hardening only where the audit shows a concrete gap.
6. Add research/community release documents.
7. Run validation commands and capture Phase 5 evidence.

## Validation Strategy

Phase 5 validation should emphasize reproducibility and deployability:

- workflow linting or YAML validation where practical
- Docker build validation
- compose configuration validation
- Go tests
- Python syntax or targeted checks
- any native checks that remain feasible in the current environment

## Success Criteria

Phase 5 is complete when:

- deployment assets exist and are internally consistent
- release validation has a dedicated workflow
- config schemas and config docs exist
- missing release-facing documents are added
- observability/deployment gaps identified in the audit are addressed
- a final Phase 5 report records evidence, limits, and readiness scoring
