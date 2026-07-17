# Phase 4 HIL Readiness Report

Date: 2026-07-16

## Scope

Hardware-In-The-Loop readiness audit only.

This report does not claim:

- flight validation
- tethered validation
- free-flight readiness

## Reviewed Interfaces

- hardware abstraction layer: `include/hal/JetsonHAL.hpp`, `src/hal/JetsonHAL.cpp`
- native runtime entrypoint: `src/main.cpp`
- sensor interfaces: `include/sensors/*`, `src/sensors/*`
- localization inputs: `include/localization/*`, `src/localization/*`
- telemetry interfaces: `include/telemetry/ControlPlaneTelemetryClient.hpp`, `src/telemetry/ControlPlaneTelemetryClient.cpp`
- simulator/runtime validation: `include/runtime/RuntimeMode.hpp`, `src/runtime/RuntimeMode.cpp`

## Supported Hardware Paths

- IMU path via `/dev/i2c-*`
- RTSP camera path via OpenCV video backends
- LiDAR UDP path with configurable parser model and timeout
- UWB/TDOA path through UDP text ingestion or serial driver
- telemetry uplink path through HTTP/HTTPS control-plane client
- V2X mesh runtime path for edge swarm packet exchange

## Simulator / Bench Readiness

- simulation mode is implemented and validated in software
- example configs exist for safe non-production startup
- synthetic fallback behavior is tested for missing live localization input
- production telemetry unavailable-source behavior is covered by smoke testing
- process restart smoke in simulation mode completed successfully on 2026-07-16

## Timing and Clock Observations

- sensor polling defaults are software-defined and heterogeneous:
  - most sensor threads: `100 Hz`
  - motor sensor: `20 Hz`
  - LiDAR acquisition thread: `10 Hz`
- telemetry interval is configurable and defaults to `1000 ms`
- time-sync tracking exists in software and has degraded-state tests
- no HIL trace of end-to-end sensor timestamp alignment was captured in this phase

## Actuator / Safety Boundary Observations

- command-policy and safety-manager logic can reject unsafe or unauthorized actions
- required-sensor gating exists for arming and production constraints
- no HIL evidence confirms actuator timing, PWM path integrity, or failsafe landing behavior on real hardware

## Failure Injection Readiness

Software-visible failure injection is available for:

- missing or malformed configuration
- missing live localization sources
- LiDAR unavailability
- localization degradation
- stale/disconnected peers
- telemetry retry / backend failure behavior

Missing HIL-grade failure injection evidence:

- real bus disconnect / reconnect timing
- live sensor brownout behavior
- clock skew injection with physical hardware
- actuator watchdog trip and recovery

## Missing Hardware Requirements

- accessible IMU device at the configured I2C path
- RTSP-capable camera source plus required multimedia plugins
- live LiDAR sender on the configured UDP path
- live UWB/TDOA source via UDP or serial
- controlled backend endpoint with TLS material for hardened-mode telemetry
- a repeatable bench harness for power, networking, and safe shutdown observation

## Readiness Assessment

HIL readiness status: PARTIAL PASS

What is ready:

- software interfaces are present
- non-production startup paths are usable
- failure behavior can be exercised in software

What is not yet ready enough to claim HIL completion:

- no recorded sensor timing capture on real hardware
- no actuator-loop verification evidence
- no clock-discipline evidence
- no physical fault-injection evidence

## Recommended Next Validation Plan

1. Stand up a bench harness with real IMU, camera, LiDAR, and TDOA sources.
2. Capture timestamp alignment and sensor update-rate traces under load.
3. Verify safe behavior for sensor disconnect, reconnect, and degraded-link events.
4. Record actuator-boundary behavior without entering free-flight testing.
