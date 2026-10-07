# ADR-0008: SIMD abstraction via xsimd

## Status
Accepted (2025-10-07)

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
