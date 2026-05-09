# QA GPS-Denied Flight Readiness Report

## 1. Executive summary

This repository is **not ready for real GPS-denied autonomous flight** today.

It contains meaningful building blocks:
- real Linux IMU access exists
- camera stream acquisition exists
- TDOA/UWB ingest paths exist for CSV, UDP, and serial text
- localization confidence, degraded/lost state handling, and security gating are implemented
- the Go control plane and dashboard can exchange structured fleet data

But the current end-to-end system still mixes real interfaces with demo/simulation behavior in several flight-critical places:
- VIO now includes a real feature-tracking frontend with RANSAC-gated visual correction, but it is not yet validated against replay or flight-hardware datasets
- LiDAR ingestion now has a real UDP receive path plus generic packet parser support, but it is not yet validated against a specific production LiDAR model or replay dataset
- the Go backend now separates `simulation` and `production` modes, but production telemetry/auth hardening still needs deployment validation
- onboard backend telemetry publishing now supports Linux plain-HTTP uplink, but transport hardening and backend-side telemetry authentication are not yet production-validated
- detector labels can now be mapped into semantic labels, but the deployed label map still needs validation against the actual detector model used in flight

This audit pass added a stricter runtime boundary:
- runtime modes now separate `simulation`, `bench`, and `production`
- synthetic TDOA fallback is restricted to `simulation`
- bench/production require anchor configuration
- production rejects playback-only TDOA sources
- anchors now load from validated JSON configuration rather than silently using hardcoded demo geometry outside simulation
- dashboard warning text now makes simulation localization, backend mode, no-real-drone, and stale-telemetry conditions visible to the operator
- bench, pre-arm, and tethered acceptance gates now exist in project docs/scripts, but they still require real hardware execution to count as readiness evidence

Result: the codebase is best described as **lab bench ready**, not indoor free-flight ready.

## 2. Current readiness verdict

**Verdict: Lab bench ready**

Why not higher:
- no proven real VIO position estimation loop from camera + IMU
- no vendor-validated LiDAR obstacle pipeline
- backend/dash production mode is now explicit, but still needs end-to-end validation with real onboard telemetry
- no evidence of hardware validation logs, flight logs, or replay-based acceptance criteria

## 3. Evidence table

| Feature | Code location | Current status | Real hardware dependency | Risk | Required fix |
|---|---|---|---|---|---|
| IMU ingest | `src/sensors/IMUSensor.cpp:45-88` | Real Linux I2C path exists | Linux I2C + MPU6050-style device | Medium | Validate on target hardware with timestamped logs, saturation/noise tests, and replay |
| IMU fallback | `src/sensors/IMUSensor.cpp:149-157` | Synthetic stationary-noise fallback exists | None | High | Mark simulation in runtime status and block flight modes when active |
| Camera stream ingest | `src/sensors/CameraSensor.cpp:23-31` | Real RTSP/OpenCV path exists | Camera stream availability | Medium | Validate frame timing, dropped-frame behavior, and calibration on target camera |
| VIO vision update | `src/vio/VIOPipeline.cpp`, `src/vio/EKFEstimator.cpp` | Real feature tracking / optical flow / RANSAC frontend implemented; simulation placeholder retained only in simulation mode | Real camera texture, calibration, and timing | High | Validate drift, inlier behavior, and confidence thresholds on replay and live hardware |
| Camera-to-autonomy semantic labels | `src/sensors/CameraSensor.cpp`, `src/autonomy/DecisionEngine.cpp`, `config/detector_labels.example.json` | Connected through configurable class-ID label mapping; unknown classes fall back to conservative obstacle handling | Trained detector label map | Medium | Validate deployed label map against the exact detector model and add replay coverage |
| LiDAR ingest | `src/sensors/LidarSensor.cpp`, `config/lidar.example.json`, `src/main.cpp` | Basic real UDP receive and generic packet parsing implemented; startup can now fail when required LiDAR is unavailable | Real LiDAR UDP packets and deployment config | High | Validate against a real sensor model, add vendor parser(s), and prove timing/filtering on replay/live hardware |
| Runtime mode separation | `src/main.cpp`, `src/runtime/RuntimeMode.cpp`, `config/runtime.example.json` | Implemented for simulation/bench/production startup policy | Runtime config and operator discipline | Medium | Extend with deployment docs and C++ test execution in CI |
| TDOA anchors | `src/main.cpp`, `src/runtime/RuntimeMode.cpp`, `config/anchors.example.json` | Configurable anchor loading is implemented; demo anchors remain available only in simulation mode | Real surveyed anchors | Medium | Replace example anchor file with calibrated deployment-specific geometry and validate on replay/live hardware |
| TDOA/UWB ingest | `src/localization/TDOAIngestor.cpp:202-290`, `src/localization/UWBSerialDriver.cpp:75-184` | Real interfaces exist | UDP/serial external measurements | Medium | Add schema validation, timing checks, anchor ID validation, and replay tests |
| TDOA fallback | `src/main.cpp:630-642`, `src/runtime/RuntimeMode.cpp` | Synthetic fallback now restricted to simulation mode | None | Medium | Validate bench/production startup rejection paths in C++ CI |
| Localization degradation detection | `src/localization/LocalizationFusion.cpp:42-63`, `src/vio/EKFEstimator.cpp:242-272` | Implemented in code | Reliable confidence inputs | Medium | Validate thresholds with logs and flight-like replay datasets |
| Autonomy degraded/lost handling | `src/autonomy/DecisionEngine.cpp:90-117`, `src/autonomy/DecisionEngine.cpp:278-356` | Implemented in code | Real localization confidence signal | Medium | Validate behavior in replay/HIL and tethered tests |
| Central safety manager | `include/safety/SafetyManager.hpp`, `src/safety/SafetyManager.cpp`, `src/main.cpp` | Implemented for safety-state evaluation, mission gating, emergency priority, and indoor motion limits | Accurate sensor/security/runtime inputs | Medium | Validate thresholds and actuation behavior against bench replay and tethered drills |
| Security-based command gating | `include/security/DroneSecurity.hpp:59-129`, `include/security/CommandPolicy.hpp:74-110` | Implemented in code | Accurate security state inputs | Medium | Add onboard integration tests with real swarm message flow |
| Backend telemetry uplink | `src/main.cpp`, `src/telemetry/ControlPlaneTelemetryClient.cpp`, `tests/test_telemetry.cpp` | Cross-platform Windows/Linux HTTP uplink implemented with timeout, auth headers, and retry backoff | Backend reachable from flight computer | Medium | Validate on target Linux flight computer and add HTTPS/TLS-capable hardened transport path |
| Go backend ingest | `internal/controlplane/server.go` | Real HTTP ingest exists and production telemetry defaults to `source=real` when not explicitly tagged | HTTP POST from node | Medium | Validate server-side auth enforcement and target deployment ingress behavior |
| Go backend fleet state | `internal/controlplane/server.go`, `internal/controlplane/state.go`, `cmd/control-plane/main.go` | Runtime-separated backend modes implemented; production no longer seeds fake drones or runs the simulation loop | Real onboard telemetry feed | Medium | Validate production deployment defaults, telemetry auth, and stale-threshold tuning on target backend |
| Dashboard fallback visibility | `gui/dashboard.py`, `gui/backend_status.py` | Backend mode, no-real-drone, stale telemetry, and localization source warnings are visibly labeled | Backend availability | Medium | Add UI integration test coverage for backend disconnect and stale-drone banners |
| Bench / pre-arm / tethered gates | `scripts/bench_check.py`, `scripts/pre_arm_check.py`, `docs/BENCH_ACCEPTANCE_TEST_PLAN.md`, `docs/TETHERED_FLIGHT_ACCEPTANCE_GATE.md` | Implemented as fail-closed operator gates for non-simulation setup and tethered pre-arm readiness | Real hardware, backend, and operator drill execution | Medium | Run these gates on real bench hardware and archive pass/fail evidence before tethered testing |

## 4. GPS-denied navigation checklist

- [x] IMU driver path exists
- [x] Camera ingest path exists
- [x] Real VIO frontend path implemented
- [ ] Real VIO frontend proven on camera + IMU data
- [ ] EKF validated against real motion logs
- [x] TDOA/UWB ingest interfaces exist
- [x] Non-simulation profiles now block synthetic TDOA fallback at startup
- [x] Anchor configuration loading and validation
- [ ] Real anchor survey/calibration validation
- [x] Real LiDAR UDP receive path and generic scan parsing
- [ ] Real vendor/model validation for LiDAR packets
- [x] Operator-visible separation of live vs fallback data across all components
- [ ] Hardware-in-the-loop or replay acceptance criteria
- [ ] Tethered indoor test evidence
- [ ] Multi-drone real mesh validation evidence

## 5. Sensor fusion QA checklist

- EKF propagates and exposes confidence/lost state in code
- Confidence model is heuristic, not hardware-validated
- Vision updates now use tracked features, optical flow, and RANSAC-gated motion estimates, but no replay dataset or drift-bounds evidence was found
- Fusion can prefer TDOA when VIO drift grows
- Lost/degraded transitions exist, but no replay dataset or tuning evidence is present

## 6. TDOA/UWB QA checklist

- CSV, UDP text, and serial text ingest paths exist
- Minimum 4 measurements are required in ingestors/localizer
- Synthetic demo TDOA is now restricted to explicit simulation mode
- Anchors now load from JSON config with duplicate/count/coordinate validation and geometry warnings
- No evidence of real clock-sync calibration or surveyed-anchor validation

## 7. VIO/SLAM QA checklist

- EKF core exists and has unit tests
- Camera frames can be acquired
- Production VIO path now uses a real visual frontend; the fixed-depth landmark shortcut is retained only for explicit simulation mode
- Loop-closure/relocalization scaffolding exists, but no real sensor validation evidence was found
- No bag/log playback tests proving drift bounds

## 8. Obstacle avoidance QA checklist

- Decision engine contains obstacle avoidance behavior
- LiDAR ingest now has a real UDP receive path and generic packet parser, so real obstacle clouds can reach the map
- Camera detector labels now map into semantic autonomy labels, and unknown classes fall back to `unknown_class_ID`
- LiDAR obstacle points now also feed the avoidance path, but real obstacle avoidance is still **not operationally proven** without hardware/replay validation

## 9. Swarm networking QA checklist

- UDP/Fast-DDS hybrid mesh scaffolding exists
- Secure swarm message assessment and onboard command policy exist
- No evidence of packet-loss/jitter/clock-skew validation in real multi-drone radio conditions
- Go backend and dashboard can simulate swarm behavior, which is useful for demos but weak as readiness evidence

## 10. Backend/dashboard QA checklist

- Go backend can receive telemetry via `/api/v1/telemetry`
- Go backend now supports explicit `simulation` and `production` modes
- Dashboard can consume Go backend snapshots
- Dashboard can fall back to simulation if backend is unreachable
- This audit added explicit backend-mode, stale-telemetry, no-real-drone, and simulation-localization warning text
- Onboard telemetry publisher now supports Linux plain-HTTP posting, but hardened HTTPS/TLS and backend-side auth enforcement remain open production tasks

## 11. Safety and failsafe QA checklist

- Localization degraded/lost state exists
- Autonomy has `HOVER_AND_SCAN`, `LOCALIZATION_LOST`, and `SAFE_RETURN_BY_ANCHOR` modes
- Central safety manager now defines `NORMAL`, `DEGRADED_LOCALIZATION`, `LOCALIZATION_LOST`, `LINK_LOST`, `SENSOR_FAULT`, `EMERGENCY_LAND`, and `MOTOR_LOCKED`
- Low-confidence visual localization now constrains commanded speed/acceleration in indoor-oriented bench mode
- Missing required LiDAR in production now blocks autonomous flight/arming through both startup validation and runtime safety state
- Security assessment can block remote commands and isolate autonomy
- Emergency land commands are allowed even when normal remote commands are blocked
- Bench/pre-arm/tethered scripts now enforce explicit no-fly gates in docs/tooling, but no real execution evidence was found in this audit
- No evidence of motor-controller integration or closed-loop flight-controller enforcement was found here
- No evidence of tethered failsafe drills, recovery timing limits, or controller saturation tests was found

## 12. Test commands that passed/failed

### Passed

- `python -m py_compile main.py gui/dashboard.py gui/backend_status.py scripts/drone_setup.py tests/test_dashboard_backend_status.py`
- `python -m py_compile main.py gui/dashboard.py gui/backend_status.py scripts/drone_setup.py scripts/bench_check.py scripts/pre_arm_check.py tests/test_dashboard_backend_status.py`
- `python -m unittest tests.test_dashboard_backend_status`
- `go test ./...`

### Failed / could not run in this environment

- `cmake -S . -B build-qa-check -DBUILD_TESTS=ON`
  - Failed because `cmake` is not installed in the current shell environment
- `cmake --build build-qa-check --config Release`
  - Failed because `cmake` is not installed in the current shell environment
- `ctest --test-dir build-qa-check --output-on-failure`
  - Failed because `ctest` is not installed in the current shell environment

## 13. Missing tests

- Real camera + IMU log replay proving bounded drift over time
- Real vendor-specific LiDAR packet parsing and map integration tests
- Surveyed-anchor TDOA replay with injected malformed/out-of-order packets
- Flight-profile test that forbids synthetic TDOA fallback
- C++ runtime-mode startup tests executed in CI or local CMake
- Linux onboard telemetry client integration tests to the Go backend
- Dashboard integration test proving backend-disconnect and stale-telemetry banner/state behavior
- Multi-drone swarm latency/jitter/packet-loss tests
- Hardware watchdog / estimator timeout / sensor dropout tests
- Bench motor/actuator interface safety tests

## 14. Upgrade roadmap before real flight

### Blockers before any untethered indoor flight

1. Validate the new visual frontend against replay logs and tune acceptance/confidence thresholds for real camera/IMU data.
2. Validate the generic LiDAR UDP integration against a real sensor model and add any required vendor-specific parser(s).
3. Validate the new non-simulation TDOA startup guard in compiled C++ test runs and deployment procedures.
4. Validate calibrated deployment anchor geometry and time-sync assumptions with replay/live measurements.
5. Validate the deployed detector label map against the exact model outputs used on target hardware.
6. Validate Linux telemetry uplink on target Jetson/RPi hardware and add hardened HTTPS/TLS transport plus server-side auth enforcement.
7. Validate Go backend production mode against real onboard telemetry, including stale-telemetry thresholds and no-real-drone operator behavior.
8. Add replay tests and tethered acceptance gates for degraded/lost localization behavior.

### Recommended staged test plan

Current go/no-go:
- **Proceed only through desktop simulation, log playback, and bench tests**
- **Do not proceed to indoor free-flight yet**

Proposed stages:

1. Desktop simulation
   - run Go backend and dashboard in explicit simulation mode only
   - verify fallback indicators are visible
2. Log playback
   - feed recorded camera/IMU/TDOA logs through VIO/fusion
   - require drift/confidence/lost-state acceptance thresholds
3. Bench sensor test
   - live IMU, live camera, live UWB/TDOA, live LiDAR on bench
   - verify timestamps, rates, dropped frames, anchor visibility, and backend telemetry
4. Tethered indoor hover
   - only after items 1-8 in blockers are addressed
   - require explicit no-simulation/no-demo status on node, backend, and dashboard
5. Small indoor waypoint test
   - single drone, guarded volume, operator with kill switch
6. Multi-drone controlled test
   - only after single-drone localization, avoidance, and telemetry are proven stable

## Answers to the main QA questions

1. **Can the drone estimate position without GPS using real sensor data?**  
   Partially scaffolded, but still not proven. IMU and camera ingest exist, TDOA ingest exists, and a real visual frontend path now exists, but it has not been validated on replay or flight-hardware datasets.

2. **Is VIO/EKF integration connected to real camera/IMU streams or mostly placeholder/demo?**  
   IMU and camera streams can connect, and the production path now uses tracked features plus RANSAC-gated visual correction. The remaining gap is validation and tuning rather than missing frontend code.

3. **Is TDOA/UWB localization using real external measurements, or falling back to synthetic demo data?**  
   Real ingest paths exist, and synthetic fallback is now restricted to explicit `simulation` mode. Bench/production no longer silently fall back, and bench/production now require external anchor configuration.

4. **Does the system detect localization degradation/lost state correctly?**  
   It has implemented heuristics and tests, but not real-hardware validation evidence.

5. **Does autonomy enter safe degraded mode when localization confidence drops?**  
   Yes in code; not validated in real flight conditions.

6. **Can obstacle avoidance work with real LiDAR/camera input?**  
   Partially, but not reliably enough for flight sign-off. LiDAR now has a real UDP ingest path and obstacle points feed avoidance, and camera labels now map into semantic autonomy labels, but the LiDAR path and detector-label configuration still need hardware/replay validation.

7. **Can swarm communication work in real multi-drone conditions?**  
   Networking/security scaffolding exists, but real multi-drone validation evidence is missing.

8. **Are secure swarm commands actually enforced onboard?**  
   The onboard policy exists and blocks/accepts commands based on security state. This is one of the stronger implemented areas.

9. **Can the Go backend receive real onboard telemetry?**  
   Yes for plain HTTP: the backend can receive telemetry and the onboard client now supports Linux uplink. The remaining gap is hardened transport/auth validation rather than basic connectivity.

10. **Can the dashboard distinguish real fleet data from simulation fallback?**  
   Yes, much more clearly now. It shows backend mode, drone `source`, simulation-enabled state, no-real-drone warnings, and stale telemetry warnings. The remaining gap is validating that behavior in a real deployment with onboard telemetry and backend disconnects.

11. **Is the system safe enough for indoor GPS-denied test flight?**  
   **Not for free-flight.** At best, after upgrades, it could move toward tethered indoor tests.

12. **What must be upgraded before outdoor/real mission flight?**  
   Replay-validated VIO tuning, vendor-validated LiDAR integration, hardened telemetry transport/auth, calibrated anchor survey validation, backend production-mode deployment validation, and hardware/replay validation.
