# ADR-0011: nanobind instead of pybind11

## Status
Accepted (2025-10-07)

## Context
The proposal mentioned `pybind11` for Python bindings. pybind11 is
still functional, but its successor **nanobind** (by the same author,
Wenzel Jakob) is now the de facto choice for new projects:

| Metric | pybind11 | nanobind |
|--------|----------|----------|
| Build time | baseline | 3-5× faster |
| Wheel size | baseline | ~30% smaller |
| Type stubs | manual | automatic (.pyi generation) |
| Thread-safety | OK | improved (explicit GIL state) |
| Maintenance | stable, slow | active, fast-moving |
| License | BSD-3 | BSD-3 |

## Decision
**nanobind** is the Python binding library for CAD_0. All bindings
live under `bindings/python/` and produce a single compiled extension
`_CAD_0.*.so` (Linux) / `_CAD_0.pyd` (Windows).

### Build integration
- `scikit-build-core` as the Python build backend (not setuptools).
- The CMake build invokes `nanobind_add_module()` (see
  `bindings/python/CMakeLists.txt`).
- `pip install CAD_0` builds the extension from source on the user's
  machine; cibuildwheel produces binary wheels for CI.

### Public Python API
The compiled extension exposes submodules: `CAD_0.math`,
`CAD_0.geometry`, `CAD_0.sdf`. The Python package
(`bindings/python/CAD_0/__init__.py`) re-exports these as
top-level symbols for convenience.

## Consequences
- Python 3.10+ required (nanobind dropped 3.7-3.9).
- The `stable-abi` flag is used so the wheel built on 3.10 also works
  on 3.11, 3.12, 3.13.
- Migration cost from pybind11 is minimal for new code (we have none
  to migrate).
