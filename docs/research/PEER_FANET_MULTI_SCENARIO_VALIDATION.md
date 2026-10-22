# Replay behavior under varying network conditions

## Evidence classification

The executable study in this repository is a **deterministic, uncalibrated,
software-only sensitivity analysis**, not measured radio performance, not a
validated channel simulator, and not an airborne safety demonstration.
Changes in modeled loss, delay, reordering, sender count and duplicates test
the replay policy's sensitivity, but those input probabilities must **not** be
described as measured or typical FANET parameters.

Reviewer's empirical RF/FANET requirement remains **open** until there are
independent packet-correlated transmission and reception traces and verified
capture provenance. Passing the software test does not resolve it.

## Four varied scenarios and replicated results

Run from the repository root:

```sh
python3 -m unittest tests.test_peer_replay_trace_analysis tests.test_peer_replay_scenario_study
python3 scripts/peer_replay_scenario_study.py --seeds 30 --output replay-sensitivity.json
```

The four scenarios are low traffic, higher mobility-like jitter, congested
traffic and intermittent links. They vary 3–8 senders, message intervals,
correlated loss, exponentially distributed delay and duplicate probability.
All are constructed assumptions. Each scenario uses deterministic independent
random seeds, identical transmitted/received events for both replay policies,
and at least two runs; 30 or more distinct seeds are recommended for research
tables. The report includes raw per-run fractions and 95% percentile
**cluster-bootstrap intervals over seeds**. Confidence intervals describe
variability under the chosen simulation mechanism, not the variability of real
flight networks. Receiver times induce arrival order; transmitter times induce
per-sender sequence order. No clock synchronization or latency estimate is
inferred from those times.

For each run, the denominators are those defined in
[PEER_REPLAY_TRACE_EVALUATION.md](PEER_REPLAY_TRACE_EVALUATION.md):
distinct attempted transmissions for loss, distinct received first deliveries
for reorder and benign-rejection fractions. Duplicates are counted separately.
The reported paired difference is
`window_benign_rejection_fraction - strict_benign_rejection_fraction` per
matched run. No per-packet pseudo-replication is used in the bootstrap.

## Check the production C++ implementation

The C++ tool `peer_replay_cache_cli` signs each synthetic heartbeat with a
**test-only HMAC secret**, runs production `verifyPacket` without passing a
prior sequence watermark, and then passes each arrival to production
`SwarmStateCache::observe_packet`. It returns a decision for every packet.

```sh
cmake --preset validation-linux-gcc
cmake --build --preset validation-linux-gcc --target peer_replay_cache_cli
ctest --preset validation-linux-gcc -R peer_replay_native_parity --output-on-failure
```

The CTest target runs six independent seeds per scenario through both models
and fails on **any** packet-order, identity or admission-decision mismatch.
The synthetic packets use constant valid TTL/time to isolate sequence policy;
this intentionally excludes radio processing, timestamp freshness failure,
cryptographic key rollover, journal durability and CPU latency. The command
should also be run on representative hardware for performance claims. The
production bitmap uses 64 sequence positions and is kept per sender/epoch.

## Independently published datasets and suitability

1. Moussaoui, A. and Lakhlef, H. (2026), *FANET Dataset: UAV Communication
   Scenarios in NS-3.40*, Version 2, Zenodo:
   https://doi.org/10.5281/zenodo.19373220.
   This dataset contains packet-level simulation traces across eight network
   configurations. It is **not physical RF data**, and its packets cannot be
   used as an authenticated peer-sequence study without validating that each
   transmission/receipt identifies the same application packet, receiver,
   sender and sender-specific **monotonic sequence**. A packet UID is not
   automatically a valid application-layer sequence counter.
2. Hossen et al. (2026), *UAV-based wireless multi-modal measurements from
   AERPAW autonomous data mule (AADM) challenge in digital twin and real-world
   environments*, Dryad: https://doi.org/10.5061/dryad.7d7wm3898.
   The outdoor campaign provides LoRa `f_cnt`, timestamps, receiver information
   and measured SNR/RSSI for 33 real testbed flights. Those are **real RF
   reception measurements**, but the advertised LoRa log alone does not
   establish a complete matching sender transmission inventory. Frame-counter
   gaps cannot be converted into measured packet-loss rates without checking
   retransmissions, counter resets, logging continuity and capture coverage.
   Multiple gateway receptions must be processed per distinct receiver.

These dataset catalogues were assessed for input suitability only; **no
external packet samples have been imported or analyzed in this change**. Their
published QoS metrics are not reported as our own measurements, and neither
dataset calibrates the synthetic parameters by default. Original dataset
licenses and publication citations must be retained on reuse.

## Real RF / calibrated emulator evidence required before manuscript claims

Capture or supply (for each fixed receiver, sender and epoch) both `tx.csv`
and `rx.csv` in the existing analyzer schema, with these fields:

- TX: `sender_id,epoch,sequence,tx_ms`
- RX: `sender_id,epoch,sequence,rx_ms`

Log every attempted packet on the sender before the radio transmit request,
and each accepted physical arrival on the receiver before replay filtering.
Match identity on packet body plus sender and epoch, **not** time proximity or
packet UID alone. Deduplicate MAC-layer retries separately from intentional
application sends. Capture RF channel/frequency, packet size, node positions,
mobility speed, interference conditions, sender/receiver clock method, receiver
gateway ID, firmware hashes, radio configuration, antenna configuration,
collection start/end, raw logs and network topology.

Use verified **independent** RF flights or explicitly defined ns-3 scenarios,
several independent trials per condition, plus checksums and capture provenance.
Do not combine observations across receivers into one receiver stream. Refuse
epoch resets/sequence wraps, missing TX provenance, or partial collection
windows where packet-loss denominators are unknown.

Then run:

```sh
python3 scripts/peer_replay_trace_analysis.py \
  --tx /data/session/tx.csv --rx /data/session/rx.csv \
  --epoch 1 --source measured_unverified --output /data/session/replay.json
```

To execute the **same externally supplied receiver sequence** in the compiled
production cache and check every admission decision, use:

```sh
python3 scripts/peer_replay_scenario_study.py \
  --native /path/to/peer_replay_cache_cli \
  --external-tx /data/session/tx.csv \
  --external-rx /data/session/rx.csv \
  --output /data/session/replay-native-parity.json
```

That command records SHA-256 hashes and verifies C++ parity, but does not
independently verify who acquired the trace or establish whether a capture is
real RF, simulated or incomplete. At present the C++ demonstration adapter
uses the selected fixed epoch 1; split and normalize each externally verified
fixed-epoch run before comparison.

The `measured_unverified` flag **does not certify source authenticity**.
Independent validation and explicit publication of network parameters,
measurement uncertainty, replication and raw source traces are required.
Neither field measurements nor multi-condition calibrated packet traces have
been completed by this repository change. Report that gap in Reviewer 3's
response and do not use synthetic fractions to claim empirical validation.
