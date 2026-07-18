# Phase 4 Valgrind Report

Date: 2026-07-16

Evidence:

- `docs/architecture-safety-validation-hardening/valgrind_version.log`
- `docs/architecture-safety-validation-hardening/valgrind_test_ekf.log`
- `docs/architecture-safety-validation-hardening/valgrind_test_edge_swarm.log`
- `docs/architecture-safety-validation-hardening/valgrind_drone_node.log`
- `docs/architecture-safety-validation-hardening/valgrind_drone_node_suppressed.log`

## Environment

- OS: `Ubuntu 26.04 LTS` on `WSL2`
- package versions:
  - `valgrind 1:3.26.0-0ubuntu1`
  - `libc6-dbg 2.43-2ubuntu2`
- `valgrind --version`: `valgrind-3.26.0`

## Commands Executed

Initial representative checks:

- `test_ekf --gtest_filter=EKFTest.ResetYieldsZeroState`
- `test_edge_swarm --gtest_filter=PeerPacketAuth.HmacSignedPacketVerifies`

Those earlier attempts were blocked before startup until `libc6-dbg` was installed.

Primary Phase 4.5 command path:

```bash
timeout --signal=INT 8s valgrind \
  --tool=memcheck \
  --leak-check=full \
  --show-leak-kinds=all \
  --track-origins=yes \
  --error-exitcode=99 \
  ./build/linux-gcc-release/bin/Release/drone_node \
  --runtime-config=config/runtime.example.json \
  --runtime-mode=simulation
```

Follow-up diagnostic command with narrow external suppressions:

```bash
timeout --signal=INT 8s valgrind \
  --tool=memcheck \
  --leak-check=full \
  --show-leak-kinds=all \
  --track-origins=yes \
  --error-exitcode=99 \
  --suppressions=/tmp/drone_valgrind.supp \
  ./build/linux-gcc-release/bin/Release/drone_node \
  --runtime-config=config/runtime.example.json \
  --runtime-mode=simulation
```

## Unsuppressed Result

Observed summary from `docs/architecture-safety-validation-hardening/valgrind_drone_node.log`:

- `definitely lost: 21,211 bytes in 5,290 blocks`
- `possibly lost: 1,632 bytes in 2 blocks`
- no invalid read report
- no invalid write report
- no use-after-free report
- no double-free report
- no uninitialized-value defect attributed to repository code

Root cause of the dominant leaks:

- OpenCV camera backend autodiscovery entered `libgphoto2` / `libusb`
- the main loss records terminate in `cv::VideoCapture::open(...)`
- this occurred during `drone::sensors::CameraSensor::initialize()`

## Suppressed Follow-Up Result

Observed summary from `docs/architecture-safety-validation-hardening/valgrind_drone_node_suppressed.log`:

- `definitely lost: 51 bytes in 2 blocks`
- `possibly lost: 1,632 bytes in 2 blocks`
- `suppressed: 21,160 bytes in 5,288 blocks`

Remaining unsuppressed contexts were still external:

- GStreamer missing-plugin description allocation
- GLib/GStreamer thread-pool TLS allocation during multimedia backend startup

No remaining repository-owned allocation path was proven by the final run.

## Findings

- Valgrind is now operational in the WSL environment.
- The remaining Memcheck issues are in third-party multimedia backend startup paths used by OpenCV camera probing.
- No repository-owned invalid read, invalid write, double-free, or use-after-free defect was proven in the bounded `drone_node` run.
- Because the final result is not perfectly clean without suppressions, this cannot be called a pristine zero-warning Valgrind pass.

## Verdict

Valgrind status: COMPLETED WITH ACCEPTED EXTERNAL-LIBRARY LIMITATION

Final status rationale:

- environment blocker is closed
- repository-owned memory defect not proven
- residual Memcheck noise remains in OpenCV/GStreamer/libgphoto camera backend probing

