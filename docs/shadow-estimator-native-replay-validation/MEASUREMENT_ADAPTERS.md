# Measurement Adapters

Date: July 17, 2026
Status: Implemented

## Implemented Adapters

Phase 16 uses a typed `MeasurementEnvelope` defined in `include/vio/MeasurementEnvelope.hpp`.

The live and replay paths currently build envelopes for:

- IMU samples
- visual pose corrections derived from the current visual frontend
- manual ZUPT commands
- LiDAR depth corrections
- disabled LiDAR observations when unsafe correction remains off

## Envelope Fields

Each published envelope carries:

- measurement type
- source identifier
- timestamp in seconds
- sequence identifier
- declared frame
- typed payload
- optional covariance hint
- sensor reference and metadata strings

## Validation Behavior

Envelope validation is fail-closed.

Current checks include:

- finite timestamp
- non-empty source identifier
- payload and type agreement
- finite vectors and positive sigma values
- non-negative covariance hint when present

Invalid envelopes are rejected before estimator mutation.

## Phase 16 Usage

- the active path processes envelopes synchronously on the caller thread
- the shadow path receives copied envelopes through a bounded queue
- reset increments a generation counter so queued stale envelopes cannot publish snapshots or comparisons
- replay and dedicated stress tests exercise valid, rejected, disabled, and malformed inputs without re-enabling unsafe fusion
