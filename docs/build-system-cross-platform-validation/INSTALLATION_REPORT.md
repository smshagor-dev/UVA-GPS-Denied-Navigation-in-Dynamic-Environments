# Installation Report

Date: 2026-07-14

## Windows Install

Command:

```powershell
cmake --install build\windows-msvc-release --config Release --prefix build\install-check
```

Result:

- PASS

## Linux Install

Command:

```bash
cmake --install build/linux-gcc-release --prefix /tmp/drone_swarm_install
```

Result:

- PASS

## Downstream Consumer Validation

Command sequence:

```bash
cmake -S /tmp/drone_swarm_consumer -B /tmp/drone_swarm_consumer/build -G Ninja \
  -DCMAKE_PREFIX_PATH=/tmp/drone_swarm_install
cmake --build /tmp/drone_swarm_consumer/build
```

Result:

- PASS

## Packaging Fixes Closed

- Added Linux package-config support for enabling `C` before resolving MPI-backed PCL and VTK dependencies.
- Exported OpenCV and PCL public include requirements through the installed target so consumer builds can compile public headers cleanly.

## Installed Structure Verified

- executable: `bin/drone_node` or `bin/drone_node.exe`
- core library: `lib/libsensor_fusion_core.a` or `lib/sensor_fusion_core.lib`
- optional Python bridge in non-minimal builds
- public headers under `include/`
- vendored crypto headers `sha3.h` and `monocypher.h`
- docs and notices under `share/DroneSwarmSensorFusion/`
- example config JSON files
- exported package files under `lib/cmake/DroneSwarmSensorFusion/`
