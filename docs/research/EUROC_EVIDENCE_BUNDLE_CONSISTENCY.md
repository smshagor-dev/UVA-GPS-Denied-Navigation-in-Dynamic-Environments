# EuRoC VIO Post-run Evidence Bundle Consistency — Phase 87

**Scope:** verifies internal consistency of a saved software replay bundle.
It does **not** verify original EuRoC archives, publisher ground truth,
hardware flight tests, calibrated uncertainty, localization accuracy or
research novelty. SHA-256 hashes are self-attested in a mutable manifest;
a party able to rewrite both files and manifest can forge consistent hashes.

The tool is separate from the VIO fusion runner and uses Python's standard
library plus the independent Phase 86 visual coverage auditor. It reads
**no original dataset files**.

## Validated evidence

- The run manifest must declare successful unverified software replay,
  zero native failure, `ground_truth_used=false`, unverified provenance
  and no publishable accuracy claim.
- The saved synchronization, visual CSV and visual manifest files must
  match their manifest SHA-256 digests; no artifact may be a symlink.
- The visual frame ledger, terminal failure and partial-run labels must
  pass the independent visual coverage audit.
- Accepted visual CSV timestamps must **exactly** match the frame ledger
  and native active/shadow 6-DoF trajectory timestamps, in order.
- Native full-pose CSV data must be finite and quaternion-normalized.
  Native trajectory metadata, source digests, first/last timestamps
  and update counts must be consistent.
- The active/shadow position-only evaluator projections must reproduce
  exactly the corresponding columns from their native pose CSV files.
- Intentional frame-limited runs require an explicit `--allow-partial`
  opt-in. A terminal tracking failure is never a completed bundle.

Run on an **immutable copy** of the completed export directory:

```bash
python3 -m unittest tests.test_euroc_evidence_bundle_verification -v
python3 -m research.experiments.euroc_evidence_bundle_verification \
  --bundle-dir /data/exports/MH_01_easy \
  --output /data/audits/MH_01_easy_consistency.json
```

For a deliberately truncated `--max-frames` run, add `--allow-partial`
and report the short coverage honestly. The verifier refuses to overwrite
an existing report and requires the report path outside the bundle.

## Required subsequent research

Run independently sourced and verified full EuRoC sequences, evaluate
failure/coverage fractions using Phase 86, and compute independent full
6-DoF ATE/RPE with Phase 82 and prespecified association sensitivity
from Phase 83. Compare against calibrated uncertainty and credible
reproduced SOTA baselines. Do not use the output of this verifier as an
accuracy score, original dataset provenance certificate or flight-safety
assurance.
