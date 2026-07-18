# Phase 5 Gap Analysis

## Scope and Guardrails

This document establishes the Phase 5 baseline for production readiness, deployment engineering, reproducibility, observability, and research release preparation.

Phase 4 evidence under `docs/architecture-safety-validation-hardening/` is treated as closed and unchanged.

This gap analysis was produced before any Phase 5 code or workflow changes.

## Repository Audit Summary

### Existing strengths

- Native, Python, and Go components are already split cleanly across `src/`, `gui/`, `cmd/`, and `internal/`.
- The repository already includes substantial deployment guidance in [DEPLOYMENT.md](../../DEPLOYMENT.md).
- CI is mature:
  - `.github/workflows/ci.yml` covers formatting, static analysis, Python, Go, Linux, Windows, install-consumer, and validation gates.
  - `.github/workflows/release.yml` already builds packages, generates checksums, and publishes release artifacts.
  - `.github/workflows/nightly.yml` already covers extended validation and package verification.
- Phase 4 already added validation evidence, benchmark artifacts, config audit output, and research-oriented reports under `docs/architecture-safety-validation-hardening/`.
- Runtime config loading and validation already exist in `src/main.cpp` and `src/runtime/RuntimeMode.cpp`.
- The Go control plane already exposes:
  - `/api/v1/health`
  - `/api/v1/fleet`
  - `/api/v1/events`
  - `/api/v1/approvals`
  - `/api/v1/discovery`

### Existing constraints

- The working tree already contains unrelated in-progress changes from earlier phases. Those must be preserved.
- The repo contains large runtime JSON data in `config/runtime.json`, which is usable for simulation/research workflows but is not ideal as a production configuration contract by itself.
- `cmake/jetson_toolchain.cmake` is closer to a target-host build preset than a complete cross-compilation/deployment story.

## Current Deployment Analysis

### What already exists

- [DEPLOYMENT.md](../../DEPLOYMENT.md) documents:
  - launcher flow
  - dashboard, control-plane, and drone node startup
  - TLS and mTLS settings
  - environment variables
  - Windows and Linux/Jetson deployment notes
  - verification steps
- [main.py](../../main.py) already acts as a multi-process launcher with:
  - `.env` and `.env.local` bootstrap
  - rotating launcher logs
  - hardened runtime checks for field/production TLS settings
  - optional component startup
- The Go control plane already includes basic HTTP server hardening:
  - timeouts
  - request-size limiting for JSON bodies
  - panic recovery
  - method restrictions on handlers
- Release packaging already exists in `.github/workflows/release.yml`.

### What is missing or incomplete for Phase 5

#### Containerization

- No `Dockerfile` exists for the control plane / launcher stack.
- No `Dockerfile.dev` exists for reproducible development environments.
- No `docker-compose.yml` exists for local production-like orchestration.
- No `docker-compose.simulation.yml` exists for simulation/demo stack startup.

#### Deployment architecture evidence

- The repo has deployment instructions, but not a Phase 5 deployment architecture review that explicitly states:
  - supported deployment topologies
  - production vs simulation boundaries
  - container responsibilities
  - secret injection model
  - persistence/logging boundaries
  - edge-host assumptions

#### Configuration contract hardening

- `config/schema/` does not exist.
- Config validation currently happens in code and scripts, but there is no schema-backed, documented configuration contract for release consumers.
- There is no single Phase 5 artifact that maps:
  - config file names
  - required fields
  - allowed environment overrides
  - secret-bearing variables
  - production-only constraints

#### Observability and operational diagnostics

- `/api/v1/health` already exists, but it is currently a basic JSON health summary rather than a fuller production-readiness surface.
- There is no dedicated readiness endpoint.
- There is no metrics endpoint or metrics export contract.
- Request logging currently uses `log.Printf`, which is useful but not yet a structured operational logging strategy.
- There is no documented log field standard for correlation, severity, component identity, or operator audit review.

#### Release validation workflow

- Existing `ci.yml`, `nightly.yml`, and `release.yml` are strong, but there is no dedicated `.github/workflows/release-validation.yml` focused on final release-candidate validation and evidence capture.

#### Research release packaging

- `RESEARCH_RELEASE.md` is missing.
- `CODE_OF_CONDUCT.md` is missing.
- `CONTRIBUTING.md` already exists, so this item is a refinement task rather than a missing-file task.
- There is no single research-release document that describes:
  - scope of included assets
  - reproducibility expectations
  - simulation-vs-field disclaimers
  - citation or attribution guidance

## Gap Matrix

| Area | Current State | Gap | Phase 5 Priority |
|---|---|---|---|
| CI / validation | Strong existing CI, nightly, and release workflows | Missing release-candidate validation workflow | High |
| Packaging | Native packaging exists | No container packaging/orchestration | Critical |
| Config management | Runtime checks and Phase 4 audit scripts exist | No `config/schema/` contract or schema docs | Critical |
| Observability | Health endpoint and plain logs exist | No readiness/metrics/structured operational conventions | High |
| Deployment docs | Good general deployment guide exists | Missing explicit architecture review and production topology docs | High |
| Research release docs | Phase 4 research evidence exists | Missing `RESEARCH_RELEASE.md` and release-facing packaging guidance | High |
| Community docs | `CONTRIBUTING.md` exists | `CODE_OF_CONDUCT.md` missing; contributing guide may need release-facing updates | Medium |

## Recommended Phase 5 Outcome

Phase 5 should focus on hardening and packaging what already works instead of re-architecting core runtime behavior:

1. Add reproducible container assets for local, simulation, and release validation use.
2. Add explicit config schemas and operational config documentation.
3. Add a release-validation workflow that complements, not replaces, the existing CI and release pipelines.
4. Improve observability surfaces enough for deployment troubleshooting and release evidence.
5. Add the missing research/community release documents and a Phase 5 evidence bundle.

## Explicit Non-Goals For This Phase

- Reopening Phase 4 architecture or benchmark findings.
- Introducing new autonomy, localization, or sensor features.
- Replacing the current release workflow wholesale.
- Reworking unrelated user changes already present in the working tree.

