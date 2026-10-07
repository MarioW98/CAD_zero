# Contributing to cadforge

Thank you for your interest in contributing to cadforge! This document
describes how to set up your environment and the conventions we follow.

## Build prerequisites

- C++20 compiler: GCC ≥ 11, Clang ≥ 14, or MSVC ≥ 17.10
- CMake ≥ 3.22
- Python ≥ 3.10 (for bindings and tests)
- Ninja (recommended, not required on Windows)

## Quick setup

```bash
git clone https://github.com/your-org/cadforge.git
cd cadforge

# Configure (Linux)
cmake --preset linux-gcc-release

# Build
cmake --build --preset linux-gcc-release

# Test
ctest --preset linux-gcc-release
```

## Coding standards

- **C++**: C++20, no extensions. `.clang-format` and `.clang-tidy` are
  authoritative; CI rejects PRs that don't pass `clang-format --verify`.
- **Python**: ruff + black. Type-checked with mypy.
- **Headers**: every public header lives under
  `core/<module>/include/cadforge/<module>/`. Internal headers (no
  public API) live under `core/<module>/src/`.
- **Includes**: relative includes use `#include "cadforge/..."` (project
  root). Third-party includes use `#include <...>`.

## ADRs (Architecture Decision Records)

Any non-trivial design decision requires an ADR under `docs/adr/`.
Format: `NNNN-short-slug.md` (4-digit zero-padded, kebab-case).

## Testing

- Unit tests live under `tests/unit/` and use `doctest`.
- Regression tests (golden files) live under `tests/regression/`.
- Fuzz tests live under `tests/fuzz/` — we use libFuzzer.

A PR is not mergeable until `ctest` passes on Linux and Windows CI.

## Commit messages

We follow the Conventional Commits spec:

```
feat(sdf): add torus primitive with sharp feature preservation
fix(geometry): correct Shape::as_hybrid for moved-from state
docs(adr): add ADR-0015 for voxel memory layout
```

## Branches

- `main` — stable, always builds, always passes tests.
- `feature/<name>` — short-lived feature branches.
- `experimental/<name>` — long-lived research branches; no merge
  guarantees.

## Licensing

By contributing, you agree that your contributions are licensed under
the Apache-2.0 license. If your contribution includes third-party code,
declare it in the PR description and ensure compatibility with
ADR-0013.
