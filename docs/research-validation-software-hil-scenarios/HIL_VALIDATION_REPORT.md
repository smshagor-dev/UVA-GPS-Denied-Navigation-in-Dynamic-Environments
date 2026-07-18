# Phase 7 HIL Validation Report

Date: July 16, 2026
Primary artifact: `docs/research-validation-software-hil-scenarios/hil_results.json`

## Scope

Phase 7.5 closes the HIL gap with a deterministic software HIL harness implemented in [`scripts/phase7_hil_runner.py`](../../scripts/phase7_hil_runner.py). This is still software HIL, not physical HIL. It exercises the live Go control-plane backend with deterministic IMU, GPS, camera/VIO, LiDAR, telemetry-link, and command-channel surrogates.

## HIL Architecture

- Sensor source model: `simulation/sensor_models.py`
- Vehicle state model: `simulation/vehicle_state.py`
- Mission executor: `simulation/mission_executor.py`
- Fault injector: `simulation/fault_injector.py`
- Backend target: Go control plane in `production` mode with `simulation_enabled=false`
- Observation endpoints:
  - `/api/v1/fleet`
  - `/api/v1/health`
  - `/api/v1/ready`

## Simulator Communication Flow

1. Start the backend in isolated production mode.
2. Advance a deterministic vehicle state per time step.
3. Generate synthetic IMU, GPS, camera/VIO, LiDAR, telemetry-link, and command-channel inputs.
4. Apply scheduled faults and recovery actions.
5. Post telemetry to the backend.
6. Read fleet, health, and readiness state back from the backend.
7. Export timeline, response, detection time, recovery time, and safety-state transitions.

## Scenario Coverage

| Scenario | Result | Detection | Recovery | Avg latency | Max latency |
|---|---|---:|---:|---:|---:|
| `hil_gps_denied_navigation` | PASS | 0.0 s | 0.0 s | 21.221 ms | 31.479 ms |
| `hil_sensor_dropout` | PASS | 0.5 s | 1.5 s | 10.070 ms | 41.266 ms |
| `hil_telemetry_delay` | PASS | 0.5 s | 1.5 s | 18.704 ms | 32.004 ms |
| `hil_packet_loss` | PASS | 0.5 s | 1.5 s | 14.769 ms | 41.711 ms |
| `hil_estimator_degradation` | PASS | 0.5 s | 2.0 s | 7.817 ms | 24.753 ms |

## Sensor And Channel Simulation Coverage

- IMU stream: live throughout all scenarios
- GPS stream: denied in the GPS-denied scenario
- camera/VIO stream: degraded during sensor dropout
- LiDAR stream: present in the deterministic state model and available for dropout injection
- telemetry link: delayed and lossy scenarios executed
- command channel: tracked through state and backend echo

## Actuator Boundary Model

Validated:

- safety-state propagation
- degraded localization visibility
- link-loss visibility
- emergency fallback visibility
- recovery-state visibility

Not validated:

- motor/ESC control
- flight-controller coupling
- aerodynamic response
- physical emergency landing execution

## Validation Result

Status: PASS

Conclusion:

- Software HIL validation complete. Physical flight validation remains future work.

## Limitations

- No physical hardware interfaces were exercised on July 16, 2026.
- No PX4, Gazebo, Ignition, or SITL evidence is claimed here.

