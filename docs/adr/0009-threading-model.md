# ADR-0009: Threading model — oneTBB + GIL guard

## Status
Accepted (2025-10-07)

## Context
SDF evaluation, mesh extraction, and B-Rep booleans are embarrassingly
parallel. The feature tree rebuild has dependency-ordered parallelism
(DAG). We need a unified concurrency model that:

- Plays well with the Python GIL when called from bindings.
- Is portable across Linux and Windows.
- Is mature enough that we do not spend time debugging scheduler bugs.

Options considered:
- Custom thread pool — high maintenance, edge cases, no DAG support
- Taskflow — header-only, nice API, less mature than TBB
- `std::execution` (C++26 P2300) — not yet shipped
- **oneTBB** — mature, Apache-2.0, supports both pools and DAGs

## Decision
**oneTBB** is the concurrency library for CAD_0.

### Module layout
```
core/concurrency/
├── include/CAD_0/concurrency/
│   ├── thread_pool.hpp      # wraps tbb::task_arena + global_control
│   ├── parallel_for.hpp     # tbb::parallel_for with grain size
│   ├── task_graph.hpp        # tbb::flow::graph wrapper for feature tree
│   └── gil_guard.hpp         # RAII for Python GIL (acquire/release)
└── src/
    └── thread_pool.cpp
```

### Python GIL
Every C++ function exposed via nanobind that does heavy work must:
1. Acquire no Python objects during the work.
2. Use `gil_release` RAII guard to drop the GIL before the work.
3. Use `gil_acquire` to re-acquire when calling back into Python.

This is enforced by code review; a static-analysis pass may be added
in Phase B.

### Scheduler limits
`ThreadPool` exposes a configurable number of threads (defaults to
`hardware_concurrency()`). The `default_pool()` accessor returns a
process-global pool initialized lazily.

### Task graph for rebuild
The feature tree rebuild uses `TaskGraph` (a wrapper around
`tbb::flow::graph`). Each feature registers as a node; edges declare
that one feature depends on another's output. The graph executes
topologically and parallelizes independent features.

## Consequences
- TBB is a build dependency. CPM downloads a pinned version (2021.13).
- TBB must be statically linked on Windows to avoid DLL shipping
  complications.
- When TBB is not available (`CAD_0_USE_TBB=OFF`), the system
  falls back to a single-threaded shim. This is for tests/CI only;
  production wheels will always ship with TBB.
