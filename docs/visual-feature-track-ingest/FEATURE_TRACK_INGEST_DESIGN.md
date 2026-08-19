# Visual Feature Track Ingest Design

## Purpose

This work connects real optical-flow feature correspondences to the existing shadow-only MSCKF estimator path while preserving the authoritative baseline estimator.

## Data flow

1. `VIOPipeline` converts camera frames to grayscale.
2. The visual frontend detects and tracks image points with optical flow.
3. Essential-matrix and pose recovery gates remove inconsistent correspondences.
4. Inlier pixel pairs are exposed through `VisualFrontendResult`.
5. `VisualFeatureTrackManager` associates current correspondences with persistent deterministic track IDs.
6. Uninitialized tracks are geometrically initialized with two-view triangulation only when baseline, parallax, depth and ray-gap checks pass.
7. Initialized tracks are converted into `VisualFeatureMeasurementPayload` entries.
8. The payload is submitted through `EstimatorCoordinator::submit_shadow_measurement()`.
9. The active estimator does not consume the feature payload.

## Authority boundary

Visual pose measurements continue to use the normal coordinator path. Experimental visual-feature constraints use the dedicated shadow-only submission API. This keeps the baseline estimator authoritative and allows MSCKF feature processing to be evaluated without changing operator-facing state.

## Track identity

Track association is deterministic:

- existing track IDs are sorted before matching;
- a previous pixel can claim at most one existing track;
- nearest valid association inside the configured radius wins;
- equal-distance ties select the lower track ID;
- new track IDs are monotonically allocated;
- `reset()` clears tracks and restarts deterministic IDs from 1.

## Geometric initialization

Two-view triangulation is fail-closed. A candidate world point is rejected when any of the following applies:

- non-finite input;
- insufficient camera baseline;
- insufficient parallax;
- singular or invalid ray geometry;
- negative/too-small depth;
- depth above the configured maximum;
- excessive closest-ray separation;
- non-finite triangulated position.

An initialized world point remains attached to the persistent track and is reused for subsequent observations.

## Boundedness

The manager stores at most `maximum_tracks` active tracks. Each update constructs the next bounded track set from the current correspondence set, so disappeared features are retired deterministically and no unbounded observation queue is introduced in the frontend track manager.

## Failure behavior

Invalid feature batches do not reach the shadow queue. Invalid measurement envelopes are rejected by the coordinator before publication. Disabled or stopped shadow execution fails closed and cannot mutate the active estimator.

## Runtime telemetry

The pipeline exposes:

- current persistent track count;
- cumulative initialized track count;
- cumulative rejected-geometry count;
- submitted shadow feature-batch count;
- existing shadow queue, lag, drop, health and divergence telemetry.

## Scope limits

This work does not make the shadow estimator authoritative, does not add estimator switching, loop closure, bundle adjustment, global mapping or relocalization, and does not treat simulation placeholder detections as real camera feature tracks.
