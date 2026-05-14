# Edge Swarm Benchmark Data

This directory stores visualization-ready benchmark data for `edge_swarm` documentation.

## Current Dataset

`edge_swarm_benchmark_mock_data.json` contains architecture-level estimates, simulation estimates, local software serialization samples, and mock visualization data. It is intended for research planning and reviewer-facing diagrams.

It is not:

- real flight data
- real RF mesh measurement data
- synchronized HIL timing data
- evidence of flight readiness

## Replacement Path

Future HIL campaigns should export data in the same general shape but replace the evidence labels with measured categories such as:

- `propeller_off_hil`
- `multi_node_rf_bench`
- `tethered_validation`
- `flight_adjacent_validation`

Every future dataset must preserve explicit validation labels so estimated, simulated, HIL, tethered, and flight-adjacent evidence cannot be mixed accidentally.
