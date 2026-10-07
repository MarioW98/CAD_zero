// core/sdf/src/evaluate.cpp
//
// CPU implementation of EvalBackend. Uses xsimd for vectorized
// evaluation across points (batched, not SoA — we keep AoS layout
// because it makes the Python-facing API simpler).
//
// For massive grids (>=1M points) the caller should consider switching
// to a SoA representation. We can add that as an optimization in Phase B
// without changing the EvalBackend interface.
//
#include "cadforge/sdf/evaluate.hpp"
#include "cadforge/sdf/field.hpp"

#include <xsimd/xsimd.hpp>

#include <cadforge/config.h>
#if defined(CADFORGE_USE_TBB) && CADFORGE_USE_TBB
#  include <tbb/parallel_for.h>
#  include <tbb/blocked_range.h>
#  include <tbb/global_control.h>
#  define CADFORGE_HAVE_TBB 1
#else
#  define CADFORGE_HAVE_TBB 0
#endif

#include <thread>

namespace cadforge::sdf {

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

#if CADFORGE_HAVE_TBB
    // Use oneTBB's parallel_for with a reasonable grain size so that
    // small inputs (< 1000 points) don't pay for thread startup.
    constexpr std::size_t kGrain = 1024;
    tbb::parallel_for(
        tbb::blocked_range<std::size_t>(0, n, kGrain),
        [&](const tbb::blocked_range<std::size_t>& r) {
            for (std::size_t i = r.begin(); i != r.end(); ++i) {
                const auto s = root->sample(points[i]);
                out.values[i]    = s.value;
                out.gradients[i] = s.gradient;
            }
        });
#else
    // Fallback: single-threaded. Adequate for tests, not for production.
    for (std::size_t i = 0; i < n; ++i) {
        const auto s = root->sample(points[i]);
        out.values[i]    = s.value;
        out.gradients[i] = s.gradient;
    }
#endif
    // NOTE: explicit xsimd batch math would replace the inner loop body in
    // a SoA variant. For AoS, the compiler's auto-vectorizer handles the
    // common case (sphere, box) reasonably well. Phase B will add a
    // specialized SoA fast path for known primitive leaf types.
}

// ----------------------------------------------------------------------------
// GpuEvalBackend — stub
// ----------------------------------------------------------------------------
GpuEvalBackend::GpuEvalBackend() = default;

void GpuEvalBackend::evaluate(const SDFBody& body,
                               std::span<const math::Vec3f> points,
                               EvalResult& out) {
    (void)body;
    // Fallback to a fresh CpuEvalBackend so the API is usable immediately.
    CpuEvalBackend cpu;
    cpu.evaluate(body, points, out);
}

} // namespace cadforge::sdf
