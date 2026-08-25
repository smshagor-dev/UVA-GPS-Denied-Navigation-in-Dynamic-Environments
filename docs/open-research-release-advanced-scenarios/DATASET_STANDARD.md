# Phase 8 Dataset Standard

Date: July 16, 2026

## Benchmark Dataset Directory

Primary path: `datasets/benchmark/`

## Required Metadata

Datasets should include:

- dataset id
- version
- task
- format
- creation timestamp
- license
- evaluation metrics

Metadata schema:

- `datasets/benchmark/metadata_schema.json`

## Dataset Format Specification

Recommended fields:

- sample identifier
- timestamp
- sensor modality
- annotation or target payload
- split designation
- provenance notes

## Experiment Naming Convention

Recommended pattern:

`phase8_<task>_<dataset_id>_<run_id>`

Examples:

- `phase8_perception_benchsetA_run001`
- `phase8_localization_corridorset_run002`

## Evaluation Metrics

Metric selection should be task-specific, for example:

- latency
- throughput
- confidence calibration
- recovery time
- safety-state accuracy
- planning success rate

## Current Status

- standard defined
- benchmark dataset payload not bundled in source control
