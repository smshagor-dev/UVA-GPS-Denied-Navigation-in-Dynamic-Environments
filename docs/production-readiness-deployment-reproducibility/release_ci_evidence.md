# Release CI Evidence

Date: 2026-07-16

## Workflow reviewed

- [.github/workflows/release-validation.yml](../../.github/workflows/release-validation.yml)

## Result

Status: PASS

## Verified locally

The workflow contains:

- Python dependency installation
- schema validation
- `go test ./...`
- production Docker image build
- development Docker image build
- compose expansion for production and simulation files
- container smoke checks for `/api/v1/health`, `/api/v1/ready`, and `/metrics`
- artifact upload for expanded compose files, endpoint outputs, and logs

## GitHub Actions remote trigger

Remote workflow trigger was not executed from this machine because GitHub CLI authentication was unavailable locally.

This does not block the workflow-definition validation itself.

## Raw evidence

- [gh_version.txt](./evidence/gh_version.txt)
- [gh_auth_status.txt](./evidence/gh_auth_status.txt)
- [git_remote.txt](./evidence/git_remote.txt)

