# Measurement Adapters

## Envelope

`EstimatorMeasurement` carries:

- timestamp
- sequence id
- sensor source
- coordinate frame
- measurement type
- typed payload

## Adapter status model

- `ACCEPTED`
- `REJECTED_INVALID`
- `REJECTED_STALE`
- `REJECTED_UNSUPPORTED`
- `REJECTED_FRAME_MISMATCH`
- `REJECTED_OUTLIER`
- `IGNORED_SHADOW_ONLY`

## Implemented adapters

- IMU
- Visual pose and velocity
- Visual orientation metadata transport
- Manual ZUPT
- Altitude/depth envelope
- TDOA position candidate tagging

## Phase 16 limitation

Visual orientation is preserved in the envelope but intentionally not consumed by the active minimal ESKF.
