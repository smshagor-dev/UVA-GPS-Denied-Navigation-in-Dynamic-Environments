# Independent real-RF reception evidence: what can and cannot be measured

This follow-up supports Reviewer 3 without inventing radio measurements or
treating public flight descriptions as evaluated traces. **No third-party raw
flight CSV was obtained, executed or validated in this change.** Software
tests use invented micro-fixtures only. Published dataset provenance is
independent of whether a particular user-supplied file is genuinely extracted
from that publication.

## External data references

- Hossen, M. S., Dickerson, C., Ozdemir, O., Sichitiu, M. and Guvenc, I.
  (2026). *UAV-based wireless multi-modal measurements from AERPAW autonomous
  data mule (AADM) challenge in digital twin and real-world environments*.
  Dryad: https://doi.org/10.5061/dryad.7d7wm3898. DOI's README describes
  33 outdoor testbed flights, LoRa `LORAlog.csv`, `f_cnt`,
  `deduplication_id`, `dev_eui`, `rx_info_gatewayId`, reception timestamp,
  SNR and RSSI. Full published archive is approximately **7.70 GB**; it
  contains additional RF sensing, radar, USRP and digital twin records.
  See the dataset paper DOI https://doi.org/10.1109/IEEEDATA.2026.3685296.
- Moussaoui, A. and Lakhlef, H. (2026), *FANET Dataset: UAV Communication
  Scenarios in NS-3.40*, version 2, Zenodo:
  https://doi.org/10.5281/zenodo.19373220. Contains **ns-3 simulation**
  packet traces from eight scenarios and explicit source code. The data are
  CC BY 4.0; cite the companion JJCIT paper
  https://jjcit.org/paper/289/FANET-DATASET-UAV-COMMUNICATION-SCENARIOS-IN-NS-340
  when using those data. **Simulation must not be labeled RF field capture.**

## Reception-only LoRa experiment

Place the genuine AERPAW `LoRa/.../LORAlog.csv` from a single flight locally.
Retain its original archive hash, source DOI, extraction path, selection of
gateway and transmitter, and a signed capture provenance sheet if available.
Don't publish flight telemetry or individual gateway locations without
checking redistribution/privacy requirements.

```sh
python3 scripts/rf_reception_audit.py \
  --rx-log /data/AADM/LoRa/flight-a/LORAlog.csv \
  --flight-id flight-a --device REAL_DEV_EUI --gateway REAL_GATEWAY_ID \
  --normalized-rx /tmp/flight-a-rx.csv \
  --output /tmp/flight-a-reception-report.json
```

The device and gateway are user-supplied values (the strings above are
**placeholders**, not identifiers we observed). Selected records are sorted
by the selected gateway's `rx_info_time` and stable file line order.
Each reception is mapped to a normalized `(sender_id=1, epoch=1, f_cnt)`
sequence. **This map is for the software comparison only**: LoRaWAN counters
are not the production swarm authenticated packet sequence or trust epochs.
Conflicting `deduplication_id` for a reused `f_cnt` fails closed; analyze
such counters separately after investigating reset, rejoin, collision and
radio-level retransmission.

Reportable from a suitable selected receiver: distinct received counter
count, observed duplicate receptions, observed arrival-order counter
inversions, receiver-only provisional strict vs 64-bit window decisions,
observed SNR/RSSI sample mean if present. The report contains `sha256` and
never emits inferred unobserved transmissions.

**Not reportable:** number of packets actually sent, RF packet-loss fraction,
true retransmission causes, receiver-versus-sender delay, robust radio
propagation calibration, or successful post-authentication acceptance.
A gap in `f_cnt` is not by itself proof of an RF drop. Unseen packets before
the first received counter and after the last received counter remain unknown.

For optional native parity, build `peer_replay_cache_cli` using the existing
CMake test target and pass `--native /path/to/peer_replay_cache_cli` to the
command above. This replays reception order with freshly constructed
**test-only HMAC heartbeats** into the production C++ cache; it is not an
on-radio authentication test.

## Across independent outdoor flights

Write a manifest of **different physical flights** (one chosen gateway/device
per flight) such as this template, then replace all placeholder paths and
IDs with the genuine extracted files:

```json
[
  {"flight_id": "flight-a", "csv": "flight-a/LORAlog.csv",
   "device": "REAL_DEVICE_1", "gateway": "REAL_GATEWAY_1"},
  {"flight_id": "flight-b", "csv": "flight-b/LORAlog.csv",
   "device": "REAL_DEVICE_2", "gateway": "REAL_GATEWAY_2"}
]
```

```sh
python3 scripts/rf_flight_study.py \
  --manifest /data/manifest.json \
  --output /data/reception-flights.json
```

The manifest paths resolve relative to its own directory. The analysis
rejects repeated flight identifiers and uses an unweighted 95% percentile
**bootstrap by whole flight**, not by individual packets. Report per-flight
counts, path-independent input hashes, paired strict-vs-window rejection
difference and confidence intervals. This is **descriptive uncertainty for
supplied flights** only; independent flight identity and RF provenance cannot
be proved from filenames or the self-declared manifest.

A receiver-only dataset can validate counter ordering under genuine RF
conditions, **not** the reviewer's full packet-loss and legitimate-replay
claim. It also cannot calibrate the earlier synthetic model until sender
transmission ground truth and conditions are established.

## Data required to close Reviewer 3's empirical claim

Acquire matching **sender-side application TX packets** and receiver-side RX
packets for each experimental run, with (sender identity, epoch, sequence,
transmitter timestamp, receiver timestamp, gateway identity), synchronized
or independently ordered logs, complete observation windows, zero counter
wraps, known retransmission semantics, authentic packet signatures and
documented mobility/topology/interference. Verify source hashes and research
metadata. Then feed normalized matched captures to
`peer_replay_scenario_study.py --external-tx ... --external-rx ...
--native ...`, perform paired trial-level analysis, and explicitly
distinguish end-to-end missed receptions from RF-layer failure.

**Status:** Source references inspected, receive-only adapter and statistical
study implemented, tests automated. Actual raw RF flight acquisitions,
measurement results, sender TX inventory, validated calibration and reviewer
manuscript updates remain pending. No missing packet rates or empirical
numerical results have been fabricated.
