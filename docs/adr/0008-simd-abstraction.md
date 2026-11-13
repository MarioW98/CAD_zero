# ADR-0008: SIMD abstraction via xsimd

## Status
Accepted (2025-10-07); amended 2026-11-14 (PERF 3 — the fast path made real; see the amendment at the tail)

## Context
SDF evaluation processes millions of points per frame. Naive scalar
evaluation (one point per call) leaves 4-8x performance on the table
on modern CPUs with AVX2 / AVX-512 / NEON.

Options considered:
- `std::experimental::simd` — not yet standardized, varies by vendor
- `Vc` (deprecated, replaced by xsimd)
- **xsimd** — actively maintained, header-only-ish, works with Eigen
- Custom intrinsics — high maintenance, portability nightmare
- Highway (Google) — also mature, similar feature set to xsimd

## Decision
**xsimd** is the SIMD abstraction layer for CAD_0. We define the
evaluation backend as an abstract `EvalBackend` with two concrete
implementations:

- `CpuEvalBackend` — scalar fallback + xsimd-vectorized fast path
  for known primitive leaf types (sphere, box, cylinder, torus).
- `GpuEvalBackend` — stub interface (post-MVP, see "GPU path" below).

### Why xsimd over alternatives
- Mature: shipped for 7+ years, used by xtensor, Pythran, Boost.SIMD.
- Cross-platform: x86 (SSE/AVX2/AVX-512), ARM (NEON/SVE), WebAssembly.
- Single-include header layout, easy to vendor.
- Active maintenance (version 12+ as of 2025).

### Why GpuEvalBackend is stubbed in Phase A
The abstraction needs to exist *now* so that downstream code (mesh
extraction, ray marching, viewport picking) is written against the
stable interface. Adding a backend later should not require API
changes. The stub falls back to CPU so the system is usable
immediately.

## Consequences
- The SDF evaluation loop is not inlined at the Python boundary; it
  is a single `CpuEvalBackend::evaluate(body, points, out)` call.
- For massive grids (≥1M points), a Structure-of-Arrays variant will
  be added in Phase B (`evaluate_soa`) without changing the
  `EvalBackend` interface.
- xsimd headers add ~5 seconds to compile time. Acceptable.
- The GPU backend will be implemented in Phase G+ via Vulkan compute
  shaders. The `EvalBackend` interface must not be changed when this
  happens.

## Amendment (2026-11-14, PERF 3 — the fast path made real)

The Decision's "xsimd-vectorized fast path for known primitive leaf
types" was a DECLARATION without an implementation from Phase A to
PERF 2's tip: the evaluator looped one virtual `sample()` per point,
included `<xsimd/xsimd.hpp>` without referencing one xsimd symbol,
and the comment claimed the compiler's auto-vectorizer would handle
the common case — it cannot (the virtual call is the barrier). The
PERF 3 audit closed the gap with three instruments (source: the TU
compiles with the include removed; assembly: the loop is
`call *0x10(%rax)` + movss, zero packed instructions in the whole TU;
throughput: the scalar walk at ~56M pts/s against a bit-identical
hand-written kernel at ~711M pts/s on the declared sandbox machine).
The register's PERF 3 row carries the evidence; this amendment
records what the tree now does:

- **The batch road is the real path**: `SDFNode::sample_batch(points,
  values, gradients)` — the leaves with xsimd kernels are the seven
  primitives plus SlabZ (core/sdf/src/batch_kernels.cpp, xsimd PRIVATE
  to the TU); the interior (CSG operators, rigid transforms) composes
  its children's batches with the same per-point expressions its
  `sample()` uses — bit-identity by construction; the backend
  dispatches one `sample_batch` (serial) or arithmetic-block
  partitioned blocks (the oneTBB opt-in, no reductions).
- **Bit-identity is the kernel contract** (ADR-0014 extended): same
  expression tree per lane, division where the scalar divides,
  reciprocal where it multiplies, `select`-based min/max with
  std::min/std::max's exact lane semantics, -ffp-contract=off;
  per-class corpora pinned by tests/unit/sdf/test_perf3_evaluator.cpp,
  the six PERF 2 mesh digests held on every column.
- **The declared scalar seats**: TwistSDF (libm cos/sin is the
  bit-honest road — the math-library clause) and ExtrudeSDF (the 2D
  profile walk is its own arithmetic family). The dispatch witness
  (core/sdf/detail/batch_stats.hpp) counts leaf points by path; the
  suite pins the exact split per corpus — a declared-vectorized
  kernel that stops running is red, and so is a rogue vectorized
  twist.
- **The "Phase B SoA variant" consequence above stays registered**,
  now with the dispatch architecture in place: the interior's
  per-point combines and the twist's transcendentals are the seats a
  follow-up would pay, each with its entry criterion (a measured
  corpus that needs them).
