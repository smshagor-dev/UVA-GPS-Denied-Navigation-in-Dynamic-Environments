# Shadow Estimator Architecture

## Authoritative path

The minimal ESKF remains the only authoritative estimator.

Runtime sensor events flow through:

`EstimatorMeasurement -> active MinimalEskfAdapter -> authoritative pose/diagnostics`

## Shadow path

When enabled, the coordinator creates a separate shadow estimator instance and routes copies of accepted measurements into a bounded asynchronous queue:

`EstimatorMeasurement -> bounded shadow queue -> shadow worker -> comparison telemetry only`

The shadow estimator:

- has independent mutable estimator state
- is reset independently from active construction
- is reinitialized from the last active reset state when configured
- cannot publish authoritative vehicle pose
- cannot switch or promote itself into the active role

## Synchronization rules

- Active processing completes before shadow work is awaited.
- Queue depth is bounded by configuration.
- Overflow drops the oldest shadow event and increments observable drop telemetry.
- Queue history gaps invalidate synchronized comparison semantics.
- Dropped events that later drain move the shadow status to `stale`.
- Worker failures move the shadow status to `failed` without stopping active estimation.

## Comparison telemetry

`EstimatorComparison` reports:

- active and shadow health
- comparable versus skipped comparison counts
- timestamp and sequence mismatch handling
- position, velocity and orientation deltas
- divergence state and last divergence reason

Divergence only changes telemetry. It does not alter active confidence, does not switch estimators, and does not change the authoritative output source.

## Validated behaviors

Local Phase 16 validation now covers:

- shadow-disabled active equivalence
- active non-blocking behavior while shadow is delayed
- queue overflow and stale transition
- shadow worker failure containment
- repeated configure and stop behavior
- telemetry reads during concurrent processing
- mismatched shadow history skipping comparison
- shadow pose isolation from authoritative pose publication

## Phase 16D validation-lane isolation

Representative replay, shadow, and benchmark validation now also build against an estimator-only validation core for the focused sanitizer lane.
That isolation keeps the authoritative and shadow estimator evidence attributable to Phase 16 code rather than unrelated sensor-fusion dependencies.
