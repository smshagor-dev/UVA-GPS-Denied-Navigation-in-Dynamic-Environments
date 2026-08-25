# Development Docker Validation

Date: 2026-07-16

## Command

```powershell
docker build -f Dockerfile.dev -t drone-swarm-dev:phase5-validation .
```

## Result

Status: PASS

## Observed outcome

- image tag: `drone-swarm-dev:phase5-validation`
- image id: `sha256:23c2aa822a1d1cd784e06f6ed2008bfdd85c91ca0481938edcbcabfb17ce721a`
- image size: `1,664,783,858` bytes
- tool availability verified:
  - `go`
  - `python3`
  - `cmake`
  - `clang`
- source mount compatibility verified with `/workspace` bind mount
- Python dependency probe passed:
  - `jsonschema`
  - `numpy`
  - `cv2`
  - `PySide6`

## Compatibility adjustment

The original container build failed against Debian Bookworm Python `3.11` because the tracked `requirements.txt` floors for `numpy>=2.5.1` and `opencv-python>=5.0.0.93` were not satisfiable in this container environment.

The final `Dockerfile.dev` uses a container-local compatibility rewrite for those two minimum versions while leaving the tracked repository requirements file unchanged.

## Raw evidence

- [docker_dev_validation.log](./docker_dev_validation.log)
- [docker_image_inspect_dev.json](./evidence/docker_image_inspect_dev.json)
- [docker_dev_python_deps.txt](./evidence/docker_dev_python_deps.txt)
- [docker_dev_mount_tools.txt](./evidence/docker_dev_mount_tools.txt)

