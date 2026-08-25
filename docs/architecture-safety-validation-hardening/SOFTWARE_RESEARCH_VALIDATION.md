# Phase 4 Software Research Validation

Date: 2026-07-16

Evidence:

- `docs/architecture-safety-validation-hardening/research_scenarios.log`
- `docs/architecture-safety-validation-hardening/research_telemetry_smoke.log`
- `docs/architecture-safety-validation-hardening/software_scenarios_expanded.log`
- `docs/architecture-safety-validation-hardening/process_restart_run.log`

## Scenario Results

### Navigation failures

1. Scenario: GPS unavailable
   Setup: `RuntimeMode.SimulationAllowsSyntheticFallbackWithoutAnchorConfig`
   Expected behavior: runtime allows synthetic fallback in simulation when no live external source is present
   Observed behavior: PASS, synthetic fallback remained allowed
   Logs: `docs/architecture-safety-validation-hardening/software_scenarios_expanded.log`
   Result: PASS

2. Scenario: delayed GPS recovery analogue
   Setup: `LocalizationFusion.PrefersTdoaWhenVioDriftIsHigh`
   Expected behavior: localization should shift weight toward alternate positioning input when the primary visual path degrades
   Observed behavior: PASS, fusion preferred TDOA when VIO drift was high
   Logs: `docs/architecture-safety-validation-hardening/software_scenarios_expanded.log`
   Result: PASS

3. Scenario: IMU dropout analogue
   Setup: `SafetyManager.MissingRequiredSensorBlocksArming`
   Expected behavior: required-sensor loss should block unsafe operation
   Observed behavior: PASS, the safety manager rejected arming with required sensor loss
   Logs: `docs/architecture-safety-validation-hardening/software_scenarios_expanded.log`
   Result: PASS

4. Scenario: LiDAR dropout
   Setup: `RuntimeMode.ProductionFailsIfRequiredLidarUnavailable`
   Expected behavior: production mode should reject required LiDAR absence
   Observed behavior: PASS
   Logs: `docs/architecture-safety-validation-hardening/software_scenarios_expanded.log`
   Result: PASS

5. Scenario: VIO degradation
   Setup: `VIOFrontend.LowFeatureCountLowersConfidence` and `LocalizationFusion.MarksLocalizationLostWhenCameraAndSyncCollapse`
   Expected behavior: visual degradation should lower confidence and eventually mark localization degraded/lost
   Observed behavior: PASS
   Logs: `docs/architecture-safety-validation-hardening/software_scenarios_expanded.log`
   Result: PASS

### Communication failures

6. Scenario: packet loss / stale peer analogue
   Setup: `SwarmStateCache.PeerBecomesStaleAfterTimeout`
   Expected behavior: stale peer should be marked stale and excluded from normal operation
   Observed behavior: PASS
   Logs: `docs/architecture-safety-validation-hardening/software_scenarios_expanded.log`
   Result: PASS

7. Scenario: delayed telemetry
   Setup: `ControlPlaneTelemetryClient.RetryLogicBacksOffAfterFailure`
   Expected behavior: telemetry path should back off rather than crash or tight-loop
   Observed behavior: PASS
   Logs: `docs/architecture-safety-validation-hardening/software_scenarios_expanded.log`
   Result: PASS

8. Scenario: disconnected node
   Setup: `EdgeConsensusManager.StalePeerExcludedFromConsensus`
   Expected behavior: disconnected or stale peer should be excluded from consensus
   Observed behavior: PASS
   Logs: `docs/architecture-safety-validation-hardening/software_scenarios_expanded.log`
   Result: PASS

9. Scenario: reconnect recovery
   Setup: no dedicated reconnect-recovery automated scenario was found in the current suite
   Expected behavior: rejoined node should recover safely and re-enter normal state
   Observed behavior: not directly validated by an automated reconnect harness on 2026-07-16
   Logs: none beyond general swarm/security tests
   Result: FAIL

### Runtime failures

10. Scenario: invalid configuration
    Setup: `RuntimeMode.DuplicateAnchorsFail`
    Expected behavior: invalid anchor config should be rejected
    Observed behavior: PASS
    Logs: `docs/architecture-safety-validation-hardening/software_scenarios_expanded.log`
    Result: PASS

11. Scenario: missing configuration / missing live source
    Setup: `RuntimeMode.ProductionRejectsMissingAnchorAndLiveTdoaSource`
    Expected behavior: production mode should reject missing anchor config and missing live TDOA source
    Observed behavior: PASS
    Logs: `docs/architecture-safety-validation-hardening/software_scenarios_expanded.log`
    Result: PASS

12. Scenario: sensor timeout
    Setup: `LidarSensor.UdpReceiveTimeoutReturnsNoPacket`
    Expected behavior: timeout should surface as no packet rather than a crash
    Observed behavior: PASS
    Logs: `docs/architecture-safety-validation-hardening/software_scenarios_expanded.log`
    Result: PASS

13. Scenario: process restart
    Setup: two consecutive bounded simulation launches of `drone_node` with `config/runtime.example.json`
    Expected behavior: clean startup and clean shutdown on repeated launches
    Observed behavior: PASS, both restarts completed with `Drone node 1 shutdown complete.`
    Logs: `docs/architecture-safety-validation-hardening/process_restart_run.log`
    Result: PASS

14. Scenario: watchdog recovery
    Setup: no explicit watchdog recovery harness or watchdog component evidence was found for automated execution
    Expected behavior: watchdog should detect hang/fault and recover or fail safe
    Observed behavior: not directly validated on 2026-07-16
    Logs: none
    Result: FAIL

15. Scenario: corrupted/incomplete TDOA input
    Setup: `TdoaIngestor.RejectsIncompleteCsvBatchAndTracksOnlyValidAnchors`
    Expected behavior: malformed lines should not produce a valid batch
    Observed behavior: PASS
    Logs: `docs/architecture-safety-validation-hardening/research_scenarios.log`
    Result: PASS

16. Scenario: remote command under isolated security state
    Setup: `CommandPolicy.BlocksRemoteCommandsWhenSecurityStateIsolated`
    Expected behavior: remote command should be blocked
    Observed behavior: PASS
    Logs: `docs/architecture-safety-validation-hardening/research_scenarios.log`
    Result: PASS

17. Scenario: production telemetry unavailable-source path
    Setup: `scripts/production_telemetry_smoke_test.py`
    Expected behavior: telemetry should report unavailable localization source cleanly
    Observed behavior: PASS
    Logs: `docs/architecture-safety-validation-hardening/research_telemetry_smoke.log`
    Result: PASS

## What This Does Validate

- software degraded-mode behavior across navigation, communication, and runtime failure classes
- safety/policy refusal paths
- repeatable process restart behavior in simulation mode
- telemetry handling when real localization data is unavailable

## What This Still Does Not Validate

- live GPS-denied navigation on hardware
- HIL timing behavior
- RF loss/recovery under real radios
- automated reconnect recovery of a real node
- watchdog recovery path
- free-flight safety claims

## Verdict

Software research validation status: PARTIAL PASS

Reason:

- most requested software failure scenarios now have direct automated evidence
- reconnect recovery and watchdog recovery remain uncovered by a dedicated executable harness

