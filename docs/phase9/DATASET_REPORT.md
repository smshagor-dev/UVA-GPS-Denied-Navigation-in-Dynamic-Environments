# Phase 9 Dataset Report

Date: July 16, 2026

## Dataset Framework

Phase 9 adds a software-only autonomous dataset standard under `datasets/autonomy/`.

Included files:

- `datasets/autonomy/README.md`
- `datasets/autonomy/metadata_schema.json`
- `datasets/autonomy/sample_dataset.json`

## Required Fields

- timestamp
- vehicle state
- IMU data
- GPS/localization
- telemetry
- mission state
- sensor confidence
- failure labels
- AI decision output

## Methodology

The sample dataset is synthetic and is provided to validate:

- field naming consistency
- downstream parser expectations
- AI experiment metadata structure
- reproducibility of software-only experiments

## Limitations

- no real-world sensor capture is bundled
- no sensitive field data is included
- dataset examples must not be interpreted as flight logs

