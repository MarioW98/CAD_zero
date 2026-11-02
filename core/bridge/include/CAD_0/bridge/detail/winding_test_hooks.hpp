// core/bridge/include/CAD_0/bridge/detail/winding_test_hooks.hpp
//
// TEST-ONLY SEAM — NOT PART OF THE PUBLIC BRIDGE API.
//
// This header exposes the internals of the mesh→SDF bridge's sign
// machinery (H2-u4: the generalized winding number, its hierarchical
// evaluation over the BVH) to the unit-test suite. It is not part of
// the public CAD_0/bridge API: it will evolve without guarantee and
// may change or be removed at any time, without notice and without a
// deprecation cycle. Production code must include only
// CAD_0/bridge/brep_to_sdf.hpp and must never call anything declared
// here.
//
// Every entry point takes its budget from the CALLER — no hidden house
// value: the tests declare what they measure. The shipped
// configuration's own budget is exposed read-only
// (winding_budget_for_tests) so the discrimination can run the exact
// values the production path runs.
#pragma once

#include "CAD_0/math/vec.hpp"
#include "CAD_0/sdf/field.hpp"

#include <cstddef>
#include <memory>

namespace CAD_0::bridge {

// One hierarchical winding query's answer: the (possibly approximated)
// winding value and the bound B_accum it carries — the sum of the
// ε_node bounds of every node the traversal approximated (ADR-0035
// D2.5). B_accum ≤ the accumulation cap (9/10·½, the composition's
// ceiling), by the traversal rule.
struct WindingQueryResult {
    double w;     // the winding value the traversal answered
    double bound; // Σ ε_node over approximated nodes, ≤ the cap
};

// The BVH handle: audits the mesh (watertight or the actionable
// refusal — the SAME contract as mesh_to_sdf, both doors) and builds
// the hierarchy exactly as the shipped path builds it: the
// deterministic median split, the same bytes (ADR-0014).
//
// The exact references (exact_winding / exact_sample) are morso 1's
// own O(F) evaluation, index order, Kahan — kept verbatim as the
// corpus's reference and the speed comparison's baseline.
class WindingBVHForTests {
public:
    // Throws std::invalid_argument (naming the free edges and their
    // rim triangles) when the mesh is not watertight — before any
    // build, the morso 1 order intact.
    explicit WindingBVHForTests(const sdf::TriangleMesh& mesh);

    WindingBVHForTests(WindingBVHForTests&&) noexcept;
    WindingBVHForTests& operator=(WindingBVHForTests&&) noexcept;
    ~WindingBVHForTests();

    // The hierarchical winding query with the CALLER's per-node
    // threshold ε_w (a node is approximated only when its own bound
    // is under the threshold AND the accumulation's room — the cap —
    // can carry it). threshold = 0.0 never approximates (the
    // counterproof configuration: every node descends, every leaf
    // evaluates).
    WindingQueryResult winding(const math::Vec3d& q, double threshold) const;

    // The full sample with the CALLER's per-node threshold — the
    // shipped MeshSDF::sample with ε_w as a parameter (sign,
    // distance, gradient: one implementation, the threshold the only
    // variable).
    sdf::SDFSample sample(const math::Vec3f& q, double threshold) const;

    // Morso 1's exact O(F) full sample, verbatim — the reference.
    sdf::SDFSample exact_sample(const math::Vec3f& q) const;

    // Morso 1's exact O(F) winding sum, index order, Kahan.
    double exact_winding(const math::Vec3d& q) const;

    std::size_t triangle_count() const;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

// The derived sign band ε_w,noise(F) of morso 1 (ADR-0035 D2.4) — the
// arithmetic noise of the exact winding sum. One source of truth: the
// tests' composed bounds read it from here, never re-derive it.
double winding_sign_band_for_tests(std::size_t triangle_count);

// The DECLARED winding approximation tolerance ε_w of morso 2
// (ADR-0035 D2.5) — the per-node threshold the shipped configuration
// queries with (½/10 = 5·10⁻²).
double winding_budget_for_tests();

// The ACCUMULATION CAP of the same tolerance — 9/10·½ = 0.45, the
// composition's ceiling: B_accum ≤ cap keeps the composed
// classification band (ε_w,noise + B_accum) under ½, so the rounding
// stays total off the surface and the on-face tie-break survives.
double winding_accumulation_cap_for_tests();

} // namespace CAD_0::bridge
