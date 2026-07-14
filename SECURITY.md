# Security Policy

## Scope

This repository contains a GPS-denied UAV autonomy research stack with C++, Go, Python, firmware, and deployment assets. Security reports should cover:

- control-plane authentication and authorization
- dashboard command signing or operator trust flows
- TLS, certificate, or secret-handling flaws
- swarm packet authentication and replay protection
- firmware manifest or update-trust weaknesses
- committed secrets, unsafe defaults, or release-hygiene gaps

## Reporting A Vulnerability

Do not open public GitHub issues for suspected vulnerabilities.

Use GitHub Security Advisories for this repository when available. If private advisories are not enabled, contact the maintainer directly through the repository owner account `@smshagor-dev` before disclosing technical details publicly.

Include:

- affected file paths or components
- impact summary
- reproduction steps
- proof-of-concept details only as needed for verification
- suggested remediation if known

## Response Expectations

- Initial triage target: 7 calendar days
- Status update target after triage: 14 calendar days
- Coordinated disclosure is preferred after a fix or mitigation is available

## Supported Versions

Only the current `main` branch is considered supported for security review during this Phase 1 cleanup window.

## Hardening Notes

The implementation design document lives in [SECURITY_IMPLEMENTATION.md](SECURITY_IMPLEMENTATION.md). That file describes the architecture and roadmap; this policy file defines how to report vulnerabilities against the repository.

