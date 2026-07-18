# Phase 10 Backup Guide

Date: July 17, 2026

## Backup Command

```powershell
python deployment/scripts/phase10_backup_restore.py
```

## Covered Roots

- `config/`
- `datasets/autonomy/`
- `docs/ai-autonomy-intelligent-decision-layer/`
- `docs/enterprise-deployment-security-readiness/`
- `research/ai/`

## Validation

The Phase 10 backup script creates `results/phase10/phase10_backup.zip`, restores it into a temporary directory, and compares file hashes against the exported manifest.


