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

## Amendment (2026-11-01, Consegna B m2 — the register honest)

Three clauses above no longer say what the tree does; the record is
set straight here rather than by silently editing the decision:

- **The pin.** "2021.13" is the drafted number; the pin in
  `CMakeLists.txt` is oneTBB **v2022.0.0** — v2021.x fails the GCC 14
  column's `-Werror` (`atomic_base.h`, the stringop-overflow family),
  and the bump is commented at the option site. The full pin
  register, with the pin-vs-vendor decision for every dependency:
  `docs/dependencies.md`.
- **"Production wheels will always ship with TBB" is retired as
  never-true.** The wheel build (`pyproject.toml`) never sets
  `CAD_0_USE_TBB`, and no CI column does either: everything shipped
  and tested runs the single-threaded fallback (xsimd-vectorized,
  serial). oneTBB is an explicit opt-in (`-DCAD_0_USE_TBB=ON` → CPM
  fetches the pinned tag), with the known include-path caveat
  recorded at the option site in `CMakeLists.txt`.
- **TaskGraph is still the serial topological pass** — the "Phase B
  will dispatch to tbb::flow::graph" note in `thread_pool.cpp`
  stands unimplemented.

## Amendment 2 (2026-11-14, PERF 3 — the opt-in made real)

The first amendment retired the "production wheels ship with TBB"
clause; this one records what the PERF 3 audit found about the opt-in
itself and what the tree now does:

- **The first amendment's "(xsimd-vectorized, serial)" parenthetical
  was itself hollow** — the audit's primary finding (ADR-0008's
  amendment carries the evidence): the serial fallback evaluated one
  virtual `sample()` per point with zero SIMD instructions in the
  generated code. The clause is not edited above (the record stands);
  this amendment supersedes it: the serial road is now the xsimd
  batch road, bit-identical to the scalar form, the witness-counted
  truth pinned by the suite.
- **The opt-in had never been compiled by any column** — the
  register's own record: no CI column and no rite ever set
  `CAD_0_USE_TBB`. The first compile (the PERF 3 audit's own probe)
  hit the wall this ADR's context never named: oneTBB v2022.0.0's
  headers instantiate int → `depth_t` (unsigned char) conversions
  inside `tbb/parallel_for.h` that surface as `-Wconversion` errors
  under the kernel targets' `-Werror`, in OUR translation units. The
  cure is the xsimd precedent applied verbatim (the FASE B warning
  bonifica's own form): oneTBB's interface include directories get
  the SYSTEM treatment (core/sdf + core/concurrency), the compiled
  code unchanged — the "known include-path caveat" recorded at the
  option site in `CMakeLists.txt` is RESOLVED, and the comment now
  says so.
- **The parallel evaluate is real and deterministic**
  (`CpuEvalBackend::evaluate` under the flag): the span partitions
  into ARITHMETIC blocks — block b owns [b·chunk, min((b+1)·chunk,
  n)), chunk a pure function of n and the configured thread count —
  each block rides the same `sample_batch` inside a `task_arena` of
  the constructor's `num_threads` workers (the constructor's count is
  honored now, not stored and ignored). NO reductions exist anywhere
  in the batch road (each point writes its own slot), so the result
  is bit-identical at every thread count; the digest test
  (test_perf3_evaluator.cpp, {1, 2, 4, 8} threads against the serial
  per-point reference) runs on the TBB column and SKIPS DECLARED
  everywhere else — never a silence.
- **TaskGraph stands as amended above** — the serial topological
  pass, its seat unchanged by this morso.
