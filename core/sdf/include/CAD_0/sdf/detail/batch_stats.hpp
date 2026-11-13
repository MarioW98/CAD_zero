// core/sdf/include/CAD_0/sdf/detail/batch_stats.hpp
//
// PERF 3 — THE DISPATCH WITNESS. The audit's own instrument, made
// permanent: two counters, one per execution path of the leaf field
// arithmetic, counting POINTS (never calls, never nanoseconds).
//
// THE SEMANTICS, declared once and pinned by the suite
// (tests/unit/sdf/test_perf3_evaluator.cpp):
//   * every point whose field value is computed by an xsimd kernel's
//     lanes counts vector_points;
//   * every point served by the scalar fallback — the SDFNode default
//     batch (nodes without a kernel: TwistSDF, ExtrudeSDF, and any
//     future class) or a kernel's scalar tail (n % width points) —
//     counts scalar_points;
//   * the interior nodes (CSG operators, rigid transforms) count
//     nothing: they are composition plumbing over their children's
//     batches, and their children's points are already counted where
//     the arithmetic happens.
//
// WHY COUNTING POINTS AND NOT TIME: the witness must be deterministic
// (a test pins its exact deltas per corpus — the m8 law, no timing
// gates on machines nobody pinned) and it must survive refactors of
// the interior. Point counts by path are both.
//
// THE CONTROPROVA this witness carries (the brief's own law): a build
// that declares vectorized leaves while execution walks the scalar
// path is caught by the suite, not by an audit — the pinned delta for
// a primitive corpus goes to zero vector points the moment the kernel
// stops running (the red-first proof lives in the register row). The
// reverse lie — a "vector" kernel that loops scalar while the counter
// claims otherwise — is caught the same way: the counters are
// incremented by the code that actually computes, one fetch_add per
// chunk, and a scalar loop feeding the vector counter is a visible
// falsification at the increment site.
//
// Thread safety: std::atomic, one relaxed fetch_add per chunk (never
// per point) — negligible on the serial path, correct under TBB.
#pragma once

#include <atomic>
#include <cstdint>

namespace CAD_0::sdf::detail {

struct BatchWitness {
    std::atomic<std::uint64_t> vector_points{0};
    std::atomic<std::uint64_t> scalar_points{0};

    // A snapshot for delta-based assertions: the suite reads the pair
    // before and after its own evaluation and asserts the difference —
    // robust to whatever the rest of the process counted meanwhile.
    struct Snapshot {
        std::uint64_t vector_points;
        std::uint64_t scalar_points;
    };
    Snapshot read() const noexcept {
        return Snapshot{vector_points.load(std::memory_order_relaxed),
                        scalar_points.load(std::memory_order_relaxed)};
    }
};

// The process-global witness. Function-local static in an inline
// function: one instance per program, initialized on first use,
// no static-initialization-order hazard.
inline BatchWitness& batch_witness() noexcept {
    static BatchWitness witness;
    return witness;
}

} // namespace CAD_0::sdf::detail
