# Production Docker Build Validation

Date: 2026-07-16

## Command

```powershell
docker build -f Dockerfile -t drone-swarm:phase5-validation .
```

## Result

Status: PASS

## Observed outcome

- image tag: `drone-swarm:phase5-validation`
- image id: `sha256:feebce1413f9efd46b9b7529a93fb7aa7f7077b52b2d12a8ed21f7ba9a131d9c`
- image size: `36,368,205` bytes
- runtime user: `drone`
- exposed port: `8080/tcp`

## Notes

- The production image build succeeded.
- The final image contains the Go control-plane binary and runtime dependencies only.
- The image was rebuilt after hardening the container to run as a non-root user.
- No build failure warnings remained in the final production image path.

## Raw evidence

- [docker_build_validation.log](./docker_build_validation.log)
- [docker_image_inspect_prod.json](./evidence/docker_image_inspect_prod.json)
- [docker_history_prod.txt](./evidence/docker_history_prod.txt)

