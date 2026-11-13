// core/sdf/src/evaluate.cpp
//
// The evaluation backend. PERF 3 rewrote the dispatch: the batch road
// (SDFNode::sample_batch) is the evaluator's real path — the scalar
// per-point walk it replaced was the audit's own finding (a virtual
// sample() per point, zero SIMD instructions in the generated code,
// the "xsimd-vectorized" declaration hollow; the register's PERF 3
// row carries the evidence and the cure).
//
// The two execution modes, one output:
//   * serial (the shipped default, no CAD_0_USE_TBB): ONE call into
//     root->sample_batch over the whole span;
//   * oneTBB (the explicit opt-in): the span is partitioned into
//     ARITHMETIC blocks — block b owns [b*chunk, min((b+1)*chunk, n))
//     with chunk a pure function of n and the configured thread count
//     — and each block rides the same sample_batch inside a task_arena
//     of num_threads_ workers. There are NO reductions (each point
//     writes its own slot), so the result is bit-identical at every
//     thread count: the map i -> block is index arithmetic, the
//     scheduler decides only when and where — the thread-varying
//     digest test (test_perf3_evaluator.cpp, on the TBB column) is
//     the pinned proof, ADR-0014's clause.
//
// GpuEvalBackend is unchanged: the stub whose name() announces the
// fallback (ADR-0008's no-silent-fallback law).
#include "CAD_0/sdf/evaluate.hpp"
#include "CAD_0/sdf/field.hpp"

#include <CAD_0/config.h>
#if defined(CAD_0_USE_TBB) && CAD_0_USE_TBB
#  include <tbb/parallel_for.h>
#  include <tbb/blocked_range.h>
#  include <tbb/task_arena.h>
#  define CAD_0_HAVE_TBB 1
#else
#  define CAD_0_HAVE_TBB 0
#endif

#include <algorithm>
#include <limits>
#include <thread>

namespace CAD_0::sdf {

// ----------------------------------------------------------------------------
// CpuEvalBackend
// ----------------------------------------------------------------------------

CpuEvalBackend::CpuEvalBackend(unsigned num_threads)
    : num_threads_(num_threads == 0
                       ? std::thread::hardware_concurrency()
                       : num_threads) {}

void CpuEvalBackend::evaluate(const SDFBody& body,
                               std::span<const math::Vec3f> points,
                               EvalResult& out) {
    const std::size_t n = points.size();
    out.values.resize(n);
    out.gradients.resize(n);

    if (!body.root()) {
        std::fill(out.values.begin(), out.values.end(), 0.0f);
        return;
    }

    const SDFNode* root = body.root();
    float* const values = out.values.data();
    math::Vec3f* const gradients = out.gradients.data();

#if CAD_0_HAVE_TBB
    // Small inputs run inline (the arena executes a single-block
    // parallel_for on the calling thread — no worker wakeup to pay).
    constexpr std::size_t kMinParallel = 1024;
    if (n < kMinParallel) {
        root->sample_batch(points, values, gradients);
        return;
    }
    // The deterministic partition map: chunk depends only on n and the
    // configured thread count; block b owns [b*chunk, min((b+1)*chunk,
    // n)) — pure index arithmetic. No reduction exists anywhere in the
    // batch road, so any scheduling of any thread count produces the
    // same bits in the same slots.
    const std::size_t nthreads =
        num_threads_ == 0 ? std::thread::hardware_concurrency() : num_threads_;
    const std::size_t lanes = nthreads > 0 ? nthreads : 1;
    constexpr std::size_t kGrain = 1024;
    const std::size_t chunk =
        std::max<std::size_t>(kGrain, (n + 4 * lanes - 1) / (4 * lanes));
    const std::size_t nblocks = (n + chunk - 1) / chunk;
    // tbb::task_arena takes int max_concurrency (oneTBB 2022's own
    // signature); thread counts are small — clamped, never truncated.
    const int arena_lanes = static_cast<int>(
        lanes > static_cast<std::size_t>(std::numeric_limits<int>::max())
            ? static_cast<std::size_t>(std::numeric_limits<int>::max())
            : lanes);
    tbb::task_arena arena(arena_lanes);
    arena.execute([&] {
        tbb::parallel_for(
            tbb::blocked_range<std::size_t>(0, nblocks, 1),
            [&](const tbb::blocked_range<std::size_t>& rb) {
                for (std::size_t b = rb.begin(); b != rb.end(); ++b) {
                    const std::size_t lo = b * chunk;
                    const std::size_t hi = std::min(lo + chunk, n);
                    root->sample_batch(
                        std::span<const math::Vec3f>(points.data() + lo,
                                                     hi - lo),
                        values + lo, gradients + lo);
                }
            });
    });
#else
    // The shipped serial road: one batch call, the leaves' kernels.
    root->sample_batch(points, values, gradients);
#endif
}

// ----------------------------------------------------------------------------
// GpuEvalBackend — stub
// ----------------------------------------------------------------------------
// The GPU backend is a stub. It is NOT a silent fallback to the CPU —
// the name() method explicitly announces "gpu-stub (cpu-fallback)" so
// callers can detect at runtime that they're not actually running on
// the GPU. This is required by ADR-0008 (no silent GPU→CPU fallback).
//
// Why the stub exists: the SDF tree-evaluation pipeline must be
// uniform across backends so that the rest of the codebase (mesh
// extractor, ray marcher, Python bindings) can be written against a
// stable EvalBackend interface. The actual GPU implementation
// (compute shader / CUDA / OpenCL) is deferred to Phase E.
//
// Link-safety notes
// -----------------
// All GpuEvalBackend methods are declared in evaluate.hpp and *defined*
// here (out-of-line). This avoids "multiple definition" link errors
// that occur when an inline-defined virtual method is included in
// multiple TUs and the linker tries to deduplicate the vtable.
// ----------------------------------------------------------------------------

GpuEvalBackend::GpuEvalBackend() = default;

void GpuEvalBackend::evaluate(const SDFBody& body,
                               std::span<const math::Vec3f> points,
                               EvalResult& out) {
    (void)body;
    // Fallback to a fresh CpuEvalBackend so the API is usable immediately.
    // The name() method makes this fallback explicit (not silent).
    CpuEvalBackend cpu;
    cpu.evaluate(body, points, out);
}

const char* GpuEvalBackend::name() const noexcept {
    // Explicitly announce the fallback so callers can detect that they
    // are not running on the GPU. This string is checked by the
    // "GpuEvalBackend: name reflects stub status" test in test_evaluate.cpp.
    return "gpu-stub (cpu-fallback)";
}

} // namespace CAD_0::sdf
