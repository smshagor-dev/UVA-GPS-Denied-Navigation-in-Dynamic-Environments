# Security Policy

## Scope

This repository contains a GPS-denied UAV autonomy research stack with C++, Go, Python, firmware, packaging, and deployment assets. Security review scope includes:

- control-plane authentication and authorization
- dashboard trust boundaries and operator-command paths
- TLS, certificate, and secret-handling flaws
- swarm packet authentication and replay protection
- firmware manifest or update-trust weaknesses
- committed secrets, unsafe defaults, and release-hygiene gaps
- CI/CD workflow security, artifact exposure, and dependency-supply-chain risk

The CI security workflows currently cover:

- GitHub Actions dependency review for pull requests
- secret scanning with Gitleaks
- CodeQL analysis for C/C++, Python, and Go
- Python dependency auditing with `pip-audit`
- Go vulnerability scanning with `govulncheck`

## Supported Versions

Only the current `main` branch is considered supported for security review at this time.

Older commits, tags, and experimental local branches may contain known limitations that are no longer serviced.

## Reporting A Vulnerability

Do not open public GitHub issues for suspected vulnerabilities.

Use GitHub Security Advisories for this repository when available. If private advisories are not enabled, contact the maintainer through the repository owner account `@smshagor-dev` before disclosing technical details publicly.

Include:

- affected file paths or components
- impact summary
- reproduction steps
- proof-of-concept details only as needed for verification
- suggested remediation if known

Do not publicly disclose:

- live secrets
- private keys
- exploitable repro payloads that materially increase misuse risk
- sensitive infrastructure details

## Response Expectations

- Initial triage target: 7 calendar days
- Status update target after triage: 14 calendar days
- Coordinated disclosure is preferred after a fix or mitigation is available

## Responsible Disclosure Notes

- Please give the maintainer a reasonable opportunity to validate and address the report before public discussion.
- If you believe user safety or infrastructure safety is immediately at risk, say so clearly in the initial report.
- Reports that require hardware, RF, or flight context should state whether the issue was observed in simulation, bench, HIL, or physical testing.

## Research And Safety Boundary

This repository is not a flight-certification package and does not claim airworthiness, operational approval, or regulatory compliance. Security workflow success in CI means repository checks passed; it does not certify safe free-flight use.

## Hardening Notes

The implementation design document lives in [SECURITY_IMPLEMENTATION.md](SECURITY_IMPLEMENTATION.md). That document describes architecture and roadmap details; this policy file defines how vulnerabilities should be reported and how CI security validation is scoped.
