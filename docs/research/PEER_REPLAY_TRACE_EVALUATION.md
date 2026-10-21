# Comparing strict sequencing and a 64-packet replay window

This software-only trace analysis supports the manuscript's response to Reviewer 3 (particularly the requests for the packet reordering rate, replay-policy false rejections, and network loss). It is **not** an RF experiment, an authentication verifier, a hardware performance benchmark, or a claim about typical flying-ad-hoc-network conditions.

## Method

The same receiver-arrival events are replayed into two independent policy models:

- **Strict monotonicity:** admit a sender's sequence only when it exceeds the highest previously admitted sequence.
- **64-sequence bitmap:** admit a higher sequence, or an unseen sequence within 63 of the highest; reject duplicate sequences and sequences 64 or more behind the watermark. Both models use independent sender states.

A complete sender transmission trace is required so missing packets and first deliveries can be distinguished from duplicate deliveries. Each input represents **one receiver and one fixed epoch**. The models assume correctly authenticated, timestamp-fresh packets and do not simulate CPU cost, state persistence or consensus. Therefore their output must not be labeled as a measured full-system admission rate.

Transmission CSV required fields: sender_id, epoch, sequence, tx_ms. Reception CSV required fields: sender_id, epoch, sequence, rx_ms. Additional columns are ignored. All time columns are nonnegative integer milliseconds, used **only** to establish ordering within their own trace. The two clocks do not have to be synchronized because end-to-end delay is not derived. Transmitted sequences must increase per sender within an epoch and must stay inside the unsigned 32-bit range. Every received (sender_id, epoch, sequence) must appear in the transmission log. If recorded provenance cannot be established, the analysis exits rather than estimating legitimate traffic.

A sender's first-arrival sequence is **reordered** if it is lower than the highest distinct sequence previously seen from that sender. First deliveries are distinct; later observations of exactly the same (sender, epoch, sequence) count as duplicate deliveries. Rates use these denominators:

- Observed p_R = reordered *distinct first deliveries* / all distinct first deliveries. This is a **trace-specific** quantity, not a population-wide FANET estimate.
- Observed missing-packet fraction = transmitted distinct identities not observed at the receiver / transmitted distinct identities. This is end-to-end observation loss; it does **not** identify the physical RF cause.
- Benign policy rejection fraction for each model = first deliveries of valid transmitted identities rejected by its sequence rule / all distinct first deliveries, assuming authenticated/fresh traffic.
- Duplicate rejection count = later observations of already-observed identities rejected by the model. It is not evidence of malicious replay intent.

Neither model checks a packet's payload or signature. Traces containing forged, manipulated or unauthenticated traffic are out of scope unless independently validated and separated beforehand. No trust-epoch rollover or journal reset is simulated.

## Reproduce synthetic baseline

    python3 scripts/peer_replay_trace_analysis.py \
      --tx docs/research/fixtures/peer_trace_tx_synthetic.csv \
      --rx docs/research/fixtures/peer_trace_rx_synthetic.csv \
      --epoch 1 --source synthetic \
      --output replay-baseline-synthetic.json
    python3 -m unittest tests.test_peer_replay_trace_analysis

The explicitly invented deterministic fixture has 9 unique transmissions, 8 distinct delivered packets, one duplicate delivery, two reordered first deliveries, and one unseen transmission. Its replay policy comparison is:

| Metric | Strict monotonic | 64-packet bitmap |
| --- | ---: | ---: |
| Unique accepted packets | 6 | 8 |
| Benign first-delivery rejections | 2 | 0 |
| Duplicate-delivery rejections | 1 | 1 |

Observed fixture p_R = 2/8 = 25% and missing-packet fraction 1/9 ≈ 11.1% are **synthetic demonstration values only**, not measurements, typical rates, or reviewer-ready empirical results. They must not be presented in a paper as measured UAV network statistics.

## Using an independently acquired trace

Export sender and receiver events with trustworthy monotonic event ordering, label their collection method, duration and source. Use --source measured_unverified only as a declaration of external origin, **not** as an automated provenance certification. The script does not validate trace origin. Collect several independent seeds/scenarios or measured runs with different link loads, topology changes and loss/delay profiles; include raw logs and capture metadata. Compare both policies on each exact same trace, and archive the JSON and the input CSVs with checksums.

For a reviewer claim about *typical* UAV mesh rates, obtain actual relevant wireless measurements or a clearly described/calibrated FANET radio/network emulator. Vary node mobility, interferers, packet sizes, rates and time sync drift; report uncertainty and the measurement method. Software fixtures cannot establish that the replay policy is safe for flight-control availability. Results for this standalone reference model should ultimately be checked against the production C++ cache using a shared replay-trace interface.
