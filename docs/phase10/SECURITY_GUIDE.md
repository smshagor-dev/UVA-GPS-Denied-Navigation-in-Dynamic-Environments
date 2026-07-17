# Phase 10 Security Guide

Date: July 17, 2026

## Implemented Hardening

- TLS required outside lab mode
- mutual TLS required outside lab mode
- signed command enforcement
- response security headers
- least-privilege container settings
- secret templates kept out of tracked runtime config

## Security Validation

Run:

```powershell
python deployment/scripts/phase10_security_audit.py
go test ./...
```

## Runtime Expectations

- do not run production profile without valid cert/key/CA material
- do not keep placeholder shared secrets
- keep operator credentials out of tracked files
- review `docs/phase10/security_report.json` after each audit run

