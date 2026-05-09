# Tethered Flight Acceptance Gate

## Purpose

This gate defines the minimum no-fly barrier before any tethered indoor GPS-denied flight attempt.

It is stricter than the bench gate:

- all automatic pre-arm checks must pass
- required sensing must be real, not simulated
- backend and dashboard must show real fleet mode
- emergency recovery actions must be witnessed before takeoff

Passing this gate does **not** authorize free-flight or mission flight. It only defines the minimum conditions for a controlled tethered test.

## Mandatory Pass Conditions

All of the following must be true before tethered flight:

- runtime mode = `bench` or `production`
- no synthetic TDOA
- real anchors loaded
- real IMU active
- real camera active
- real LiDAR active
- backend receiving real telemetry
- dashboard shows real mode
- localization confidence stable
- safety manager active
- emergency land tested
- manual kill switch tested

If any item above is not satisfied, tethered flight is **not allowed**.

## Automatic Pre-Arm Gate

Run:

```powershell
python scripts/pre_arm_check.py --runtime-config config/runtime.json --backend-url http://127.0.0.1:8080
```

Expected result:

- `PRE-ARM VERDICT: PASS`

If the script prints `FAIL`, do not fly.

## Automatic Pre-Arm Fail Conditions

The pre-arm check must fail if any of these are true:

- simulation mode active
- fake drone data active
- missing LiDAR
- missing anchor config
- missing camera/IMU
- stale backend telemetry
- localization confidence below threshold

The script currently verifies:

- runtime config exists
- runtime mode is not `simulation`
- anchor config exists and defines at least 4 anchors
- LiDAR config exists and is populated
- camera stream opens
- IMU device opens
- backend is reachable
- backend mode is visible and non-simulation
- backend is receiving real drone telemetry
- no stale drone telemetry is present
- no non-real localization data source is present
- localization confidence meets the configured threshold

## Manual Witnessed Pass Conditions

The following items must be verified and signed off by the test operator even if the pre-arm script passes:

### Safety manager active

Verify:

- node logs show active `[ SAFETY ]` status output
- safety state is not `EMERGENCY_LAND`, `SENSOR_FAULT`, `LINK_LOST`, `LOCALIZATION_LOST`, or `MOTOR_LOCKED`
- indoor limits are in effect for the tethered test

### Emergency land tested

Verify:

- operator triggers emergency land while the vehicle is restrained
- commanded behavior changes immediately to emergency descent / land
- no competing mission or remote command overrides it

### Manual kill switch tested

Verify:

- the physical or operator-controlled kill switch path is armed
- test crew confirms immediate motor cutoff path
- stop procedure is rehearsed before lift attempt

## Localization Confidence Stability Rule

Automatic script threshold:

- `localization_confidence >= 0.65`

Manual stability expectation:

- confidence should remain stable during stationary tethered pre-arm observation
- no repeated nominal/degraded/lost oscillation
- no synthetic/playback localization source visible in backend or dashboard

If localization confidence is unstable, do not fly.

## Suggested Operator Sequence

1. Run the backend in `production` or controlled `bench` mode with simulation disabled.
2. Start the drone node with real runtime, anchor, and LiDAR config.
3. Confirm dashboard shows real backend mode and real localization source.
4. Run `python scripts/pre_arm_check.py ...`.
5. Verify emergency land and kill switch response on the ground.
6. Proceed only to restrained tethered hover if every item passes.

## Stop Conditions

Abort the tethered test immediately if any of the following appears:

- localization source changes to `simulation`, `playback`, or `unavailable`
- backend telemetry becomes stale
- localization confidence drops below the operating threshold
- LiDAR/camera/IMU path fails
- safety state escalates to `LINK_LOST`, `SENSOR_FAULT`, `LOCALIZATION_LOST`, `EMERGENCY_LAND`, or `MOTOR_LOCKED`

## Rule

Do not fly unless pre-arm check passes.
