# Windows Build Report

Date: 2026-07-14
Host: Windows 11
Generator: Visual Studio 17 2022
Compiler: MSVC 19.44.35228.0
Triplet: `x64-windows`

## Release

Commands:

```powershell
cmake --preset windows-msvc-release
cmake --build --preset windows-msvc-release
ctest --preset windows-msvc-release
```

Result:

- Configure: PASS
- Build: PASS
- CTest: `112/112` PASS

Observed feature summary:

- `Fast-DDS: NOT FOUND - UDP transport fallback enabled`
- `TensorRT: DISABLED - OpenCV DNN fallback enabled`
- `Python bindings: FETCHED - pybind11 FetchContent fallback`

## Debug

Commands:

```powershell
cmake --preset windows-msvc-debug
cmake --build --preset windows-msvc-debug
ctest --preset windows-msvc-debug
```

Result:

- Configure: PASS
- Build: PASS
- CTest: `112/112` PASS

## Notable Warnings

- Project-owned MSVC warnings targeted in Phase 2 were removed.
- Remaining visible warning in configure output is the upstream `pybind11` deprecated CMake minimum warning when fetched.
