# Phase 10 Production Checklist

Date: July 17, 2026

- Validate `docker-compose.production.yml` with `docker compose -f docker-compose.production.yml config`
- Validate Kubernetes manifest rendering with `kubectl kustomize deployment/kubernetes`
- Confirm TLS files and operator secrets are supplied externally
- Confirm `DRONE_SECURITY_PROFILE` is set intentionally
- Confirm readiness and health endpoints respond as expected
- Confirm monitoring scrape configuration and dashboard template are present
- Confirm backup/restore validation succeeded
- Confirm Go, config, and native regression tests remain PASS
- Do not claim live cloud deployment unless a real environment is deployed and evidenced

