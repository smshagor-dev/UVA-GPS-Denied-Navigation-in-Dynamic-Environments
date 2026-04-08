# Deployment

## Scope

This document covers practical deployment paths for the current repository state:

- local development and demo use
- single-machine dashboard plus Go backend
- C++ drone node deployment prerequisites
- environment variables and operational notes

It does not assume the full future microservice architecture yet.

## Deployment Modes

### Mode 1: Dashboard + Go backend only

Best for:

- UI demos
- fleet API demos
- development without native C++ dependencies

Steps:

```powershell
go run ./cmd/control-plane
python gui/dashboard.py --backend-url http://127.0.0.1:8080
```

This works today because the Go service seeds an in-memory fleet snapshot on startup.
The Python dashboard also keeps a local SQLite datastore for restoring the previous UI/session context.

### Mode 2: Unified launcher

Best for:

- local orchestration
- checking which components are discoverable on one machine

```powershell
python main.py
```

Useful variants:

```powershell
python main.py --dry-run
python main.py --skip-go
python main.py --skip-cpp
python main.py --skip-gui
```

### Mode 3: C++ drone node + dashboard

Best for:

- local lab work
- sensor/runtime testing
- bridge-based integration when `drone_bridge` is available

Example:

```powershell
$env:DRONE_SWARM_SECRET="replace-with-a-strong-shared-secret"
build-runtime-check\Release\drone_node.exe --id=1 --esp32=192.168.4.1 --lidar=192.168.1.201:2368 --tdoa-serial=COM5
python gui/dashboard.py
```

Adjust the executable path to the build directory you actually generated.

## Required Environment Variables

You can now keep these in a repo-root `.env` or `.env.local` file for the Python launcher and dashboard.
The C++ `drone_node` also reads the drone and sensor connection variables directly from the process environment.

### `DRONE_SWARM_ADDR`

Used by the Go control plane.

Example:

```powershell
$env:DRONE_SWARM_ADDR=":8080"
go run ./cmd/control-plane
```

### `DRONE_SWARM_SECRET`

Used by the C++ node when swarm security is enabled.

Example:

```powershell
$env:DRONE_SWARM_SECRET="replace-with-a-strong-shared-secret"
```

If unset, the node currently falls back to a development secret and logs a warning. That is fine for demos and not acceptable for shared environments.

### Drone / Sensor Connection Variables

- `DRONE_NODE_ID`
- `DRONE_ENABLE_IMU`
- `DRONE_IMU_DEVICE`
- `DRONE_IMU_ADDR`
- `DRONE_ENABLE_CAMERA`
- `DRONE_ESP32_IP`
- `DRONE_CAMERA_STREAM_URL`
- `DRONE_ENABLE_LIDAR`
- `DRONE_LIDAR_ENDPOINT`
- `DRONE_ENABLE_BAROMETER`
- `DRONE_ENABLE_MOTOR`
- `DRONE_ENABLE_OPTICAL_FLOW`
- `DRONE_ENABLE_RANGEFINDER`
- `DRONE_ENABLE_TDOA_INGESTOR`
- `DRONE_ENABLE_UWB_SERIAL`
- `DRONE_SWARM_GROUP`
- `DRONE_SWARM_PORT`
- `DRONE_TDOA_SERIAL`
- `DRONE_TDOA_UDP_PORT`
- `DRONE_TDOA_CSV`
- `DRONE_BACKEND_URL`
- `DRONE_DASHBOARD_IDS`
- `DRONE_DASHBOARD_POLL_HZ`

Example `.env`:

```powershell
DRONE_NODE_ID=1
DRONE_ENABLE_IMU=true
DRONE_IMU_DEVICE=/dev/i2c-1
DRONE_IMU_ADDR=104
DRONE_ENABLE_CAMERA=true
DRONE_ESP32_IP=192.168.4.1
DRONE_CAMERA_STREAM_URL=
DRONE_ENABLE_LIDAR=true
DRONE_LIDAR_ENDPOINT=192.168.1.201:2368
DRONE_ENABLE_BAROMETER=true
DRONE_ENABLE_MOTOR=true
DRONE_ENABLE_OPTICAL_FLOW=true
DRONE_ENABLE_RANGEFINDER=true
DRONE_ENABLE_TDOA_INGESTOR=true
DRONE_ENABLE_UWB_SERIAL=true
DRONE_TDOA_SERIAL=COM5
DRONE_SWARM_GROUP=239.255.0.1
DRONE_SWARM_PORT=7400
DRONE_SWARM_ADDR=:8080
DRONE_BACKEND_URL=http://127.0.0.1:8080
DRONE_DASHBOARD_IDS=1,2,3,4,5
DRONE_DASHBOARD_POLL_HZ=20
DRONE_SWARM_SECRET=replace-with-a-strong-shared-secret
```

## Native Dependency Setup

### Windows

For the C++ path, make sure CMake can locate:

- `Eigen3`
- `OpenCV`
- `PCL`
- `spdlog`

Optional:

- `pybind11` can now be fetched automatically during CMake configure
- native `Fast-DDS` transport can be enabled from local package installation
- `TensorRT` requires an NVIDIA GPU/runtime and is not expected on AMD-only machines

If packages are installed in nonstandard locations, set `CMAKE_PREFIX_PATH` or package-specific `*_DIR` variables before configuring.

Current verified Windows path in this workspace:

- `vcpkg` packages under `C:\tools\vcpkg-full\installed\x64-windows`
- native `Fast-DDS` installed and detected
- `drone_bridge` built successfully through fetched `pybind11`

### Linux / Jetson

The repository includes `scripts/drone_setup.py`, which is oriented toward Linux/Jetson environments and can help with:

- Python package installation
- apt-based system dependency setup
- Fast-DDS installation
- CMake/Ninja builds
- ESP32-CAM flashing helpers

Example:

```bash
python3 scripts/drone_setup.py setup
python3 scripts/drone_setup.py build --tests
```

## Logs

Current log locations:

- `logs/launcher/`
- `logs/control-plane/`
- `logs/dashboard/`
- `logs/drone.log` from the C++ node
- `data/dashboard/dashboard.sqlite3` for persistent Python dashboard state

Before packaging or collecting artifacts, clear old logs if you need a clean run.

## Verification Before Deployment

Recommended checks:

```powershell
python -m py_compile main.py gui/dashboard.py scripts/drone_setup.py
go test ./...
cmake -S . -B build-deploy -DBUILD_TESTS=ON
cmake --build build-deploy --config Release
ctest --test-dir build-deploy --output-on-failure -C Release
```

Do not rely on older checked-in build directories such as `build-tests/` without regenerating them. They may contain machine-specific absolute paths.

## Operational Notes

- The Go backend currently stores state in memory. Restarting it resets missions, events, and seeded telemetry state.
- The dashboard can fall back to simulation mode if the backend or `drone_bridge` is unavailable.
- Swarm traffic can always use built-in UDP transport; native `Fast-DDS` is additive.
- The current TDOA flow in `src/main.cpp` supports synthetic, CSV, UDP text, and serial UWB-style inputs.
- On Windows, importing the built `drone_bridge` module may require adding the Python runtime DLL directory and `vcpkg` bin directory through `os.add_dll_directory(...)`.

## Suggested Near-Term Deployment Baseline

For the current repo, the lowest-friction baseline is:

1. deploy the Go control plane on one host
2. connect the dashboard to that host with `--backend-url`
3. deploy the C++ drone node only on machines where native dependencies are already validated

That gives a stable demo path while the full native stack is still being standardized.
