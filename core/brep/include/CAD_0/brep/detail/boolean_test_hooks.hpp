// core/brep/include/CAD_0/brep/detail/boolean_test_hooks.hpp
//
// TEST-ONLY SEAM — NOT PART OF THE PUBLIC KERNEL API.
//
// This header exposes internal implementation details of the boolean
// kernel (ADR-0025) to the unit-test suite. It is not part of the
// public CAD_0/brep API: it will evolve without guarantee and may
// change or be removed at any time, without notice and without a
// deprecation cycle. Production code must include only
// CAD_0/brep/boolean.hpp and must never call anything declared here.
//
// Every entry point takes its tolerances from the CALLER — no hidden
// house value: the tests declare what they measure (H2-u3 morso 2).
#pragma once

#include "CAD_0/math/vec.hpp"

#include <vector>

namespace CAD_0::brep {

// Direct access to the internal polygon_subtract_3d slab decomposition
// (ADR-0025 v5, FASE 2) so the unit-test suite can assert the
// "exactly one outside fragment" contract on a minimal slab case.
//
// Both polygons are coplanar with `plane_normal` as surface normal.
// Returns the outside fragments of subject \ clip, one point loop per
// fragment. `eps_l` is the caller's declared linear tolerance.
std::vector<std::vector<math::Vec3d>> polygon_subtract_fragments_for_tests(
    const std::vector<math::Vec3d>& subject,
    const std::vector<math::Vec3d>& clip,
    math::Vec3d plane_normal,
    double eps_l);

// Direct access to the internal segment classification — the
// three-family decision of ADR-0035 (the derived parallelism band
// 4·ε_L·L, the parameter world's ε_P windows, the predicate-certified
// degeneracy) — so the regime goldens of H2-u3 morso 2 can read the
// decision directly and pin the two worlds' discrimination.
enum class SegmentIsectType { None, Point, Overlap };

// For Point: t_lo == t_hi == the crossing parameter along (p1,p2).
// For Overlap: [t_lo, t_hi] is the overlap range along (p1,p2).
//
// The tolerances arrive as the TYPED BUNDLE, never as bare doubles:
// the seam reads each world through its accessor (eps_l_of/eps_p_of,
// one per world — the same discipline as the operations above), so
// every decision the classifier makes is traceable to a declared
// field of the caller's bundle. The bundle's definition lives in
// CAD_0/brep/boolean.hpp; a forward declaration is all this seam
// needs.
struct BooleanTolerance;
SegmentIsectType segment_intersection_for_tests(
    math::Vec2d p1, math::Vec2d p2, math::Vec2d p3, math::Vec2d p4,
    const BooleanTolerance& tol,
    double& t_lo, double& t_hi);

// Direct access to the door's whole pair decision (ADR-0037 D2/D3,
// H2-u5 m2): the coplanarity gate (the composite ε_C, the order
// (a)-(d)) and, for the ALIGNED class, the overlay's area-overlap
// routing and the merged region's convex fragments. Returns EMPTY
// when the pair is NOT the door — the transversal and opposed pairs
// (today's path owns them) and the aligned pairs whose regions do
// not overlap in area (the edge-touching and disjoint alignments,
// P7's discipline). For the door's pairs the fragments are the
// region F_A ∪ F_B: ONE convex fragment for the convex merged
// region (P4), the slab decomposition's pieces for the stair (P8),
// the containing face untouched for the containment (P11). The
// polygons arrive as 3D loops on their own planes (the gate reads
// the pair's geometry, the projection rides A's face's frame); the
// tolerances arrive as the typed bundle.
std::vector<std::vector<math::Vec3d>> overlay_union_fragments_for_tests(
    const std::vector<math::Vec3d>& subject,
    math::Vec3d subject_normal,
    const std::vector<math::Vec3d>& clip,
    math::Vec3d clip_normal,
    const BooleanTolerance& tol);

} // namespace CAD_0::brep
