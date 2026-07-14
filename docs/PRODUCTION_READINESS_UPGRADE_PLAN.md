# Production Readiness Upgrade Plan

## Purpose

This plan converts the QA findings in `docs/QA_GPS_DENIED_FLIGHT_READINESS_REPORT.md` into a phased production-readiness program for a GPS-denied UAV stack.

Scope rules:
- no code changes are included in this document
- each phase has a clear gate before the next phase begins
- simulation/demo behavior must become explicit, controllable, and disallowed in flight phases
- no phase should be considered complete unless its acceptance criteria are met

---

## Phase 1. Simulation Lockdown

**Goal**

Make all simulation, demo, fallback, and digital-twin behaviors explicit, configurable, and impossible to confuse with production runtime behavior.

**Exact files to change**

- [src/main.cpp](../src/main.cpp)
- [gui/dashboard.py](../gui/dashboard.py)
- [gui/backend_status.py](../gui/backend_status.py)
- [main.py](../main.py)
- [internal/controlplane/server.go](../internal/controlplane/server.go)
- [internal/controlplane/state.go](../internal/controlplane/state.go)
- [README.md](../README.md)
- [DEPLOYMENT.md](../DEPLOYMENT.md)
- [.env.example](../.env.example)

**Implementation tasks**

- Introduce an explicit runtime mode model such as `simulation`, `bench`, `flight-test`, `production`.
- Make synthetic TDOA generation opt-in and disabled by default outside `simulation`.
- Add startup hard-fail behavior if a flight-capable profile still requests demo fallback paths.
- Disable Go control-plane digital-twin seeding and `simulateFlightLoop()` except in explicit simulation mode.
- Ensure dashboard and launcher present a strong operator-visible banner when backend, bridge, or localization data is simulated or degraded.
- Add structured status fields for:
  - sensor source type
  - localization source type
  - backend source type
  - simulation/fallback reason
- Document environment variables and startup policies for every mode.

**Test tasks**

- Add unit tests proving simulation mode is explicit and default flight profiles reject demo fallback.
- Add Go tests proving digital twin and simulation loop stay off in non-simulation mode.
- Add dashboard tests proving fallback state is visible and not labeled as live backend mode.
- Add startup smoke tests for all runtime modes via launcher or config parsing.

**Acceptance criteria**

- No production or flight-test profile can silently enter synthetic TDOA, simulated backend, or dashboard simulation mode.
- Go backend does not seed or simulate drones in `bench`, `flight-test`, or `production` mode.
- Dashboard always shows whether data is `live`, `fallback`, or `simulation`.
- Docs clearly define allowed behaviors per runtime mode.

**Risk level**

High

---

## Phase 2. Real Sensor Integration

**Goal**

Replace sensor stubs and loosely validated interfaces with real hardware ingestion paths and hardware-aware health reporting.

**Exact files to change**

- [src/sensors/IMUSensor.cpp](../src/sensors/IMUSensor.cpp)
- [include/sensors/IMUSensor.hpp](../include/sensors/IMUSensor.hpp)
- [src/sensors/CameraSensor.cpp](../src/sensors/CameraSensor.cpp)
- [include/sensors/CameraSensor.hpp](../include/sensors/CameraSensor.hpp)
- [src/sensors/LidarSensor.cpp](../src/sensors/LidarSensor.cpp)
- [include/sensors/LidarSensor.hpp](../include/sensors/LidarSensor.hpp)
- [src/hal/JetsonHAL.cpp](../src/hal/JetsonHAL.cpp)
- [include/hal/JetsonHAL.hpp](../include/hal/JetsonHAL.hpp)
- [tests/test_sensors.cpp](../tests/test_sensors.cpp)
- [tests/CMakeLists.txt](../tests/CMakeLists.txt)

**Implementation tasks**

- Keep Linux IMU path, but add:
  - startup self-test
  - sample-rate validation
  - stale-data detection
  - saturation and confidence reporting
- Harden camera ingest with:
  - stream reconnect policy
  - calibration validation
  - frame timestamp quality checks
  - inference-ready and model-class metadata loading
- Replace LiDAR `receive_udp_packet()` stub with real packet parsing and point cloud assembly.
- Add sensor health states that distinguish:
  - unavailable
  - stale
  - degraded
  - live
- Remove or strongly gate non-hardware simulation behavior for bench/flight modes.
- Ensure HAL battery and system stats report â€œunknown/unavailableâ€ instead of believable fake values in non-sim modes.

**Test tasks**

- Add IMU unit tests for invalid device, stale sample, and calibration edge cases.
- Add LiDAR parser tests with recorded packets, malformed packets, and empty-cloud cases.
- Add camera tests for stream-open failure, empty frame, dropped frame, and model metadata load.
- Add hardware bench scripts to capture rate, jitter, and error counters.

**Acceptance criteria**

- IMU, camera, and LiDAR each produce live data on target hardware with health state and timestamp telemetry.
- No bench or flight mode reports synthetic battery/sensor data as real.
- LiDAR point clouds are available to downstream mapping code.
- Sensor logs prove stable rate and acceptable drop/error counts over a continuous bench run.

**Risk level**

Critical

---

## Phase 3. GPS-Denied Localization Upgrade

**Goal**

Replace placeholder localization with a real GPS-denied fusion stack using validated camera/IMU/TDOA inputs and calibrated anchors.

**Exact files to change**

- [src/vio/VIOPipeline.cpp](../src/vio/VIOPipeline.cpp)
- [include/vio/VIOPipeline.hpp](../include/vio/VIOPipeline.hpp)
- [src/vio/EKFEstimator.cpp](../src/vio/EKFEstimator.cpp)
- [include/vio/EKFEstimator.hpp](../include/vio/EKFEstimator.hpp)
- [src/localization/LocalizationFusion.cpp](../src/localization/LocalizationFusion.cpp)
- [include/localization/LocalizationFusion.hpp](../include/localization/LocalizationFusion.hpp)
- [src/localization/TDOALocalizer.cpp](../src/localization/TDOALocalizer.cpp)
- [src/localization/TDOAIngestor.cpp](../src/localization/TDOAIngestor.cpp)
- [src/localization/UWBSerialDriver.cpp](../src/localization/UWBSerialDriver.cpp)
- [src/localization/TimeSyncTracker.cpp](../src/localization/TimeSyncTracker.cpp)
- [src/main.cpp](../src/main.cpp)
- [src/slam/KeyframeManager.cpp](../src/slam/KeyframeManager.cpp)
- [src/slam/OccupancyGridMap.cpp](../src/slam/OccupancyGridMap.cpp)
- [tests/test_ekf.cpp](../tests/test_ekf.cpp)
- [tests/test_navigation_intelligence.cpp](../tests/test_navigation_intelligence.cpp)
- `data/` replay fixtures to be added under versioned test assets

**Implementation tasks**

- Replace YOLO-box pseudo-landmark update with a real visual frontend:
  - feature detection/tracking
  - outlier rejection
  - geometric update into the estimator
- Define production confidence and lost-state logic from measurable signal quality, not placeholder assumptions alone.
- Validate calibrated anchor geometry loading from config or data files and prove it against replay/live measurements.
- Disable synthetic TDOA generation outside explicit simulation mode.
- Add stronger TDOA input validation:
  - anchor ID validation
  - duplicate handling
  - out-of-order measurement handling
  - time-window sanity checks
- Tune fusion weights against replay data.
- Add localization status outputs needed by autonomy and operator UI:
  - confidence
  - drift estimate
  - source selection
  - anchor visibility
  - sync quality

**Test tasks**

- Add replay tests using recorded camera/IMU sequences.
- Add TDOA replay tests with valid, missing, delayed, and malformed anchor measurements.
- Add tests proving flight profiles reject missing external TDOA if TDOA is required.
- Add acceptance tests for degraded/lost transitions under sensor dropout.
- Add drift-budget tests over fixed replay paths.

**Acceptance criteria**

- Position estimation works on real camera + IMU logs without placeholder vision logic.
- TDOA uses only external/calibrated anchor data in non-simulation modes.
- Replay tests demonstrate bounded drift and correct degraded/lost-state transitions.
- Localization source, confidence, and drift are trustworthy enough to gate autonomy.

**Risk level**

Critical

---

## Phase 4. Safety/Failsafe Hardening

**Goal**

Harden autonomy, security gating, and failure handling so the vehicle reacts conservatively when localization, sensing, or trust degrades.

**Exact files to change**

- [src/autonomy/DecisionEngine.cpp](../src/autonomy/DecisionEngine.cpp)
- [include/autonomy/DecisionEngine.hpp](../include/autonomy/DecisionEngine.hpp)
- [include/security/DroneSecurity.hpp](../include/security/DroneSecurity.hpp)
- [include/security/CommandPolicy.hpp](../include/security/CommandPolicy.hpp)
- [src/main.cpp](../src/main.cpp)
- [src/swarm/SwarmSecurity.cpp](../src/swarm/SwarmSecurity.cpp)
- [include/swarm/SwarmSecurity.hpp](../include/swarm/SwarmSecurity.hpp)
- [tests/test_autonomy.cpp](../tests/test_autonomy.cpp)
- [tests/test_swarm_security.cpp](../tests/test_swarm_security.cpp)

**Implementation tasks**

- Define explicit failsafe state machine for:
  - nominal
  - degraded localization
  - localization lost
  - sensor unavailable
  - backend disconnected
  - secure command blocked
- Require safety actions to be tied to measured confidence and health signals.
- Make `HOVER_AND_SCAN`, `SAFE_RETURN_BY_ANCHOR`, `RETURN_HOME`, and `EMERGENCY_LAND` thresholds configurable and documented.
- Ensure unsafe autonomy modes cannot persist when sensor health is stale or unknown.
- Confirm emergency commands remain available while non-critical remote commands are blocked.
- Add watchdog-style checks for stale localization and missing sensor feeds.

**Test tasks**

- Expand autonomy tests for stale localization, anchor loss, stale camera, and stale LiDAR conditions.
- Add security tests for command rejection/acceptance across trust states.
- Add integration tests that simulate chained failures:
  - localization lost + poor sync
  - backend disconnect + trusted onboard autonomy
  - command replay suspicion + operator override denial

**Acceptance criteria**

- Every major failure mode maps to a deterministic safe behavior.
- Remote command policy is deny-by-default outside explicitly trusted conditions.
- Bench and replay tests prove the vehicle enters conservative behavior quickly when confidence drops.
- Safety policy is documented and test-backed.

**Risk level**

High

---

## Phase 5. Backend/Dashboard Production Mode

**Goal**

Make backend telemetry, fleet state, and operator UI production-safe, Linux-compatible, and incapable of masking live-system failures with believable simulation data.

**Exact files to change**

- [src/telemetry/ControlPlaneTelemetryClient.cpp](../src/telemetry/ControlPlaneTelemetryClient.cpp)
- [include/telemetry/ControlPlaneTelemetryClient.hpp](../include/telemetry/ControlPlaneTelemetryClient.hpp)
- [src/main.cpp](../src/main.cpp)
- [cmd/control-plane/main.go](../cmd/control-plane/main.go)
- [internal/controlplane/server.go](../internal/controlplane/server.go)
- [internal/controlplane/state.go](../internal/controlplane/state.go)
- [internal/controlplane/types.go](../internal/controlplane/types.go)
- [internal/controlplane/server_test.go](../internal/controlplane/server_test.go)
- [internal/controlplane/server_api_test.go](../internal/controlplane/server_api_test.go)
- [gui/dashboard.py](../gui/dashboard.py)
- [gui/backend_status.py](../gui/backend_status.py)
- [tests/test_dashboard_backend_status.py](../tests/test_dashboard_backend_status.py)
- [SECURITY_IMPLEMENTATION.md](../SECURITY_IMPLEMENTATION.md)

**Implementation tasks**

- Implement Linux telemetry publish path for Jetson/RPi targets.
- Keep TLS and signed-command behavior aligned across node, backend, and dashboard.
- Remove simulated fleet motion and digital-twin state from production runtime.
- Add backend source metadata so every fleet snapshot declares whether data is:
  - live onboard telemetry
  - stale cached telemetry
  - simulation
- Add dashboard operator warnings for:
  - stale telemetry
  - missing drones
  - simulation/fallback
  - trust/security degraded
- Ensure health/events endpoints expose enough production diagnostics for test operations.

**Test tasks**

- Add Linux integration tests for telemetry POSTs into the Go backend.
- Add Go API tests for stale telemetry handling, fleet snapshot truthfulness, and health/event reporting.
- Add dashboard integration tests proving stale or fallback data cannot be rendered as live without warning.
- Run signed command tests under lab and production-style security profiles.

**Acceptance criteria**

- Linux-based drone node can publish real telemetry to Go backend.
- Go fleet state reflects only real telemetry in production mode.
- Dashboard clearly distinguishes live, stale, and fallback data.
- Backend health/events endpoints are sufficient for operational troubleshooting.

**Risk level**

Critical

---

## Phase 6. Bench Testing

**Goal**

Validate the fully integrated stack on physical hardware without flight, using repeatable bench procedures and measurable pass/fail thresholds.

**Exact files to change**

- [tests/test_sensors.cpp](../tests/test_sensors.cpp)
- [tests/test_ekf.cpp](../tests/test_ekf.cpp)
- [tests/test_navigation_intelligence.cpp](../tests/test_navigation_intelligence.cpp)
- [tests/test_autonomy.cpp](../tests/test_autonomy.cpp)
- [scripts/drone_setup.py](../scripts/drone_setup.py)
- [main.py](../main.py)
- New bench procedures document under `docs/`
- New replay/fixture assets under `data/`

**Implementation tasks**

- Define a standard bench validation checklist:
  - IMU live
  - camera live
  - LiDAR live
  - TDOA/UWB live
  - backend telemetry live
  - dashboard live/fallback truthfulness
- Add tooling to capture:
  - frame rate
  - sample rate
  - telemetry rate
  - sync offsets
  - localization confidence trend
  - packet loss
- Add repeatable recorded datasets for replay comparison.
- Define acceptable bench thresholds and standardized logs.

**Test tasks**

- Run replay tests from phase 3 on recorded hardware data.
- Run long-duration bench stability test.
- Run sensor unplug/replug and stale-feed tests.
- Run backend disconnect/reconnect test while node remains active.
- Run security command acceptance/rejection test with real processes.

**Acceptance criteria**

- Continuous bench session completes without hidden fallback transitions.
- All required sensors report live and healthy.
- Localization remains stable on replay and live bench movement tests.
- Telemetry, backend, and dashboard remain truthful and operational.

**Risk level**

Medium

---

## Phase 7. Tethered Indoor Flight

**Goal**

Prove basic safe flight behavior indoors under controlled tethered conditions with real GPS-denied sensing and conservative autonomy.

**Exact files to change**

- [src/main.cpp](../src/main.cpp)
- [src/autonomy/DecisionEngine.cpp](../src/autonomy/DecisionEngine.cpp)
- [src/localization/LocalizationFusion.cpp](../src/localization/LocalizationFusion.cpp)
- [gui/dashboard.py](../gui/dashboard.py)
- New flight-test procedures and signoff docs under `docs/`

**Implementation tasks**

- Add flight-test profile with stricter safety limits than general production.
- Limit autonomous velocity, altitude, and maneuver set for tethered testing.
- Add operator checklist and pre-arm style readiness report:
  - no simulation
  - all required sensors live
  - localization confidence above threshold
  - anchor visibility above threshold
  - backend/dashboard connected or explicitly not required
- Define abort criteria and test-card sequence for hover only.

**Test tasks**

- Tethered hover with live camera + IMU + localization stack.
- Induced degradation test:
  - partial anchor loss
  - camera occlusion
  - temporary backend disconnect
- Verify failsafe transitions and operator indications during tethered runs.

**Acceptance criteria**

- Stable tethered hover achieved with no synthetic/fallback localization.
- Vehicle enters correct degraded behavior on induced faults.
- Operators can identify state transitions in real time from dashboard/logs.
- No uncontrolled drift, unsafe oscillation, or hidden simulation state observed.

**Risk level**

High

---

## Phase 8. Indoor Free-Flight Acceptance

**Goal**

Reach a disciplined, evidence-based go/no-go decision for limited indoor free-flight in GPS-denied conditions.

**Exact files to change**

- [docs/PRODUCTION_READINESS_UPGRADE_PLAN.md](../docs/PRODUCTION_READINESS_UPGRADE_PLAN.md)
- New acceptance test procedure and readiness signoff docs under `docs/`
- Any remaining config or threshold files introduced by earlier phases

**Implementation tasks**

- Freeze production candidate configuration.
- Define final indoor acceptance envelope:
  - allowed speed
  - allowed altitude
  - allowed area
  - required anchor layout
  - required operator staffing
- Create final readiness checklist covering:
  - no demo/sim paths enabled
  - real VIO active
  - real LiDAR active
  - external TDOA/UWB active if required
  - failsafe transitions verified
  - backend/dashboard production truthfulness verified
- Establish exit criteria for expanding to waypoint tests or multi-drone tests.

**Test tasks**

- Controlled untethered hover
- Small waypoint hold
- Repeatability runs across multiple batteries/sessions
- Fault-free and fault-injection acceptance runs
- Post-flight log review against drift/confidence/health thresholds

**Acceptance criteria**

- Free-flight remains stable and bounded across repeated indoor runs.
- Localization confidence and drift remain within approved thresholds.
- Obstacle avoidance and degraded-mode handling behave as expected in real sensor conditions.
- Final signoff explicitly states indoor-only acceptance, not outdoor mission readiness.

**Risk level**

Critical

---

## Phase dependencies

1. Phase 1 must complete before any production-style testing.
2. Phase 2 and Phase 3 are the main technical blockers for flight readiness.
3. Phase 4 and Phase 5 must complete before tethered indoor flight.
4. Phase 6 must pass before Phase 7 begins.
5. Phase 7 must pass before Phase 8 begins.

---

## Recommended execution order inside the team

1. Runtime/production controls owner
   - Phase 1, Phase 5
2. Sensor and platform owner
   - Phase 2
3. Localization owner
   - Phase 3
4. Safety/autonomy owner
   - Phase 4
5. Integration and test owner
   - Phase 6, Phase 7, Phase 8

---

## Overall go/no-go rule

Do not authorize indoor free-flight until all of the following are true:

- no hidden simulation or fallback paths remain in flight mode
- real camera/IMU localization is validated on replay and bench data
- LiDAR obstacle input is real and test-backed
- TDOA/UWB behavior is calibrated and non-synthetic in flight mode
- telemetry/backend/dashboard paths are Linux-capable and truthful
- tethered indoor testing demonstrates stable behavior and correct failsafe response
