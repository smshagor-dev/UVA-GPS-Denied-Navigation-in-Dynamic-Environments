# Remaining Risks

Date: July 17, 2026

## Open Technical Risks

- the estimator still uses the existing compact baseline EKF and is not yet upgraded to FEJ or MSCKF
- replay scenarios are synthetic software scenarios rather than hardware or field logs
- covariance validation enforces finiteness, symmetry, and diagonal sanity, but does not run a full eigenvalue decomposition on every step
- LiDAR estimator depth correction remains intentionally disabled until a validated geometric observation model replaces the old semantics

## Environment Limits

- GCC was not installed on this host after PATH and standard candidate-path inspection
- no usable Clang compiler executable was installed on this host, although `clang-format` and `clang-tidy` were present in the Visual Studio LLVM tools directory
- AddressSanitizer and UndefinedBehaviorSanitizer could not be executed because the project disables sanitizer support on MSVC and no GCC/Clang compiler lane was available
- `clang-tidy` was discoverable, but the available Visual Studio build directory did not contain `compile_commands.json`, and no alternate local Clang or Ninja build lane existed

## Explicit Non-Claims

- no HIL claim
- no flight claim
- no production-readiness claim
- no cross-compiler numerical consistency claim
- no Phase 16 implementation claim
