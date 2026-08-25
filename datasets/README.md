# Phase 7 Datasets

Phase 7 does not ship a flight or HIL sensor dataset in this repository.

This directory is reserved for reproducible research inputs such as:

- camera or VIO replay datasets
- LiDAR scan captures
- TDOA/UWB anchor measurement logs
- mission scenario playback traces

Dataset guidance:

- keep raw data outside source control when size or licensing requires it
- store dataset manifests, checksums, provenance, and licenses alongside each dataset
- reference dataset IDs from `experiments/*.json` and `results/phase7/*`

Current status:

- no repository-managed Phase 7 dataset artifact was available on July 16, 2026
- mission evidence in `docs/phase7/mission_results.json` is generated from live synthetic telemetry sent to the control-plane backend
