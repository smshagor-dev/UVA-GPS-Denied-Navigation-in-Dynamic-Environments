# EuRoC Visual Tracking Coverage Audit (synthetic software validation)

Independent tool: `research/experiments/euroc_visual_coverage_audit.py`.

This audit consumes the saved `visual_manifest.json` frame ledger from the
EuRoC stereo VIO run. It **does not** read publisher ground truth. It
validates a canonical SHA-256 of the frame ledger, strictly increasing camera
timestamps, status counts, accepted samples, terminal failure disclosure,
and frame-limit truncation. A short experiment cannot be mislabeled as a
full-sequence run.

```bash
python3 -m unittest tests.test_euroc_visual_coverage_audit -v
python3 -m research.experiments.euroc_visual_coverage_audit \
  --visual-manifest /data/exports/MH_01_easy/visual_manifest.json \
  --output /data/exports/MH_01_easy/coverage_audit.json
```

Interpret `visual_update_fraction_of_all_camera_frames` as a **coverage
fraction only**, never ATE/RPE, reliability probability, sensor recall,
robustness or physical-flight evidence. Each run includes the fraction of
examined and total frames and distinguishes deliberately truncated frame
limits from fail-closed terminal tracking errors. Authentic EuRoC imagery,
source-provenance checks and independent full-pose evaluation are still needed.
