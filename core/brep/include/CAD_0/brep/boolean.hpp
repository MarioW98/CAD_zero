// core/brep/include/CAD_0/brep/boolean.hpp
//
// Boolean operations on planar-faced convex polyhedra.
// ADR-0023 (intersect, Sutherland-Hodgman), ADR-0024 (union/subtract),
// ADR-0025 (the slab decomposition, the parallel-plane rule, the
// T-junction subdivision, and the v5 assembly this file documents),
// ADR-0035 (the numerical taxonomy — the typed tolerance bundle).
//
// History note (the registry, corrected by H2 unit 1): this header
// spent its first phases declaring bool_union/bool_subtract "NOT
// IMPLEMENTED" while the .cpp implemented them (ADR-0025) — the
// declaration lagged the code. Both declarations below are now the
// code's truth, enforced.
#pragma once

#include "CAD_0/brep/topology.hpp"

#include "CAD_0/math/tolerance.hpp"
#include "CAD_0/math/vec.hpp"

#include <memory>
#include <vector>

namespace CAD_0::brep {

// The DECLARED and ENFORCED input class — shared by all three
// operations (H2 unit 1, the input gate):
//
//   * every face carries a PLANE surface (the typed SurfaceFamily
//     check — curved-surface booleans belong to the extended G2
//     table, ADR-0033);
//   * every face is a CONVEX polygon with EXACTLY ONE (outer) loop —
//     the holed-face class is out of scope and refused by name;
//   * the body is a CLOSED ORIENTABLE manifold: every edge exactly
//     two co-edges traversing it in opposite directions, every loop
//     chained, every loop vertex on its face's plane;
//   * the normals point OUTWARD and the polyhedron is CONVEX — the
//     half-space classification and the slab decomposition are
//     convex-class machinery.
//
// Violations refuse the operation with std::invalid_argument BEFORE
// anything is clipped, and the message names the violated class,
// the responsible entity and the remedy — one named class per
// violated prerequisite (input_empty_body, input_topology_reference,
// input_surface_not_plane, input_face_inner_loops, input_loop_open,
// input_edge_not_manifold, input_coedge_orientation,
// input_loop_off_plane, input_face_orientation, input_zero_volume,
// input_loop_winding, input_face_not_convex, input_not_convex).
// The gate is ONE function (boolean.cpp, validate_boolean_input);
// all three operations call it as their first statement on both
// operands — there are no copies of the contract to drift.
//
// THE RESULT CONTRACT (H2 unit 2, the output gate): the assembled
// result must be one of the honest outcome classes — the empty
// answer, the zero-volume contact disk (the pinned face-touching
// intersect: one face, every edge one co-edge, χ = 1), or a closed
// orientable manifold with an EVEN χ (multi-shell χ > 2 included —
// a classification, never an assembly defect, per ADR-0022).
// Anything else is REFUSED with std::runtime_error and its named
// class — result_loop_open, result_edge_not_manifold (the open-shell
// signature: the known partial coplanar overlap class, refused here
// pending its H2 unit 5 handling), result_edge_not_orientable,
// result_odd_chi — the body is not returned and cannot enter the
// feature tree. The refusal message names the class, the measured
// evidence and the remedy.
//
// The inputs are never modified by any of the three operations.

// ---------------------------------------------------------------------------
// The typed tolerance bundle (H2 unit 3, ADR-0035 D2) — three separate
// worlds, never one shared constant:
//
//   * linear (math::Tolerance, kind Linear, mm): the welding band, the
//     plane band, the degenerate-length guards — and the canonical
//     key's quantizer (D3: the tie-break and the equivalence threshold
//     are ONE declared value, two consumers, zero drift);
//   * parametric (ParamTolerance, domain units): the (t,u) windows,
//     the endpoint exclusions, the overlap widths (D2.2: a SEPARATE
//     typed tolerance — never the linear constant, never one constant
//     for both worlds);
//   * angular_prefilter / angular (math::Tolerance, kind Angular): the
//     two normal-parallelity decisions (D2 row 4: the loose pre-filter
//     stays loose, the strict threshold stays strict — both typed,
//     both declared).
//
// The determinant decisions carry no field of their own: the
// parallelism band is DERIVED from the linear tolerance (D2.3,
// |det| ≤ 4·ε_L·L — the bound scales with the square of the geometric
// scale because det is bilinear in the direction vectors).
//
// No defaults (ADR-0004 §1): the bundle has no default construction
// path a signature could exploit — every field is declared by the
// caller, and the three operations below take the bundle as a
// REQUIRED argument with no default value.
// ---------------------------------------------------------------------------
struct ParamTolerance {
    // The parameter world's own kind (D2.2): domain units, per
    // operation. A distinct type on purpose — the kind check is the
    // type (ADR-0004 §2), and a linear tolerance must never be passed
    // where a parameter tolerance is declared.
    double value;
    constexpr explicit ParamTolerance(double v) noexcept : value(v) {}
};

struct BooleanTolerance {
    math::Tolerance linear{};            // ε_L (mm) — D2.1
    ParamTolerance parametric{0.0};      // ε_P (domain units) — D2.2
    math::Tolerance angular_prefilter{}; // loose |n1·n2| pre-filter — D2 row 4
    math::Tolerance angular{};           // strict parallelity threshold — D2 row 4

    // The NAMED derivation per family (D2.2): this boolean class is
    // planar-faced (the gate enforces it), and the GH 2D frame is an
    // isometry (unit, orthogonal axes) — the plane family's Jacobian
    // is 1, so ε_P = ε_L. A family with a non-unit Jacobian would
    // derive its parametric tolerance through ITS declared Jacobian
    // in its own terms — never silently share this one.
    static constexpr BooleanTolerance plane_family(
        const math::Tolerance& linear,
        const math::Tolerance& angular_prefilter,
        const math::Tolerance& angular) noexcept {
        return BooleanTolerance{
            linear,
            ParamTolerance{linear.value},
            angular_prefilter,
            angular,
        };
    }
};

// The house declarations (ADR-0004 §1: default tolerances are exposed
// as named constants; the operation signatures take NO default
// argument — the caller passes one of these, or its own, explicitly).
//
//   * linear: the historical kWeldGrid / kPlaneTol value (1e-6 mm) —
//     the kind and the per-operation declaration are what H2 unit 3
//     pins; the value stands (ADR-0035 D2 row 1).
//   * angular_prefilter: the historical loose pre-filter
//     (|n1·n2| within 1e-3 of 1, ≈ 2.56°) as its angle —
//     cos(θ_prefilter) reproduces the historical dot threshold
//     bit-exactly in double (2·asin(√(1e-3/2)) round trip, 0 ulp).
//   * angular: the historical strict threshold (1 − 1e-6, ≈ 0.081°)
//     as its angle — cos(θ_strict) reproduces 1 − 1e-6 bit-exactly
//     in double (2·asin(√(1e-6/2)) round trip, 0 ulp).
inline constexpr math::Tolerance kBoolLinear =
    math::Tolerance::linear(1e-6);
inline constexpr math::Tolerance kBoolLooseNormalPrefilter =
    math::Tolerance::angular(0.044725087168733434);
inline constexpr math::Tolerance kBoolStrictNormalParallel =
    math::Tolerance::angular(0.0014142136802242518);
inline constexpr BooleanTolerance kBooleanHouseTolerance =
    BooleanTolerance::plane_family(
        kBoolLinear, kBoolLooseNormalPrefilter, kBoolStrictNormalParallel);

// bool_intersect — compute A ∩ B for two convex polyhedra.
//
// Algorithm: for each face of A, clip its polygon against every face
// plane of B (keeping the inside portion). Do the same for B's faces
// against A. Weld the resulting vertices, build edges, link radial.
// Returns a new BRepBody.
//
// `tol` is REQUIRED (ADR-0004: no default arguments): the whole
// operation — the welding grid, the plane bands, the segment
// classifications, the (t,u) windows — consumes only this bundle.
std::unique_ptr<BRepBody> bool_intersect(
    const BRepBody& A, const BRepBody& B,
    const BooleanTolerance& tol);

// bool_union — compute A ∪ B.
//
// Algorithm (ADR-0025): boundary(A ∪ B) = {A's faces outside B} ∪
// {B's faces outside A}; the outside parts come from the slab
// decomposition, interface points through the single-source
// IntersectionTable, the identical-body shortcut when A == B.
// Returns a new BRepBody. `tol` is REQUIRED (ADR-0004).
std::unique_ptr<BRepBody> bool_union(
    const BRepBody& A, const BRepBody& B,
    const BooleanTolerance& tol);

// bool_subtract — compute A \ B.
//
// Algorithm (ADR-0025): A's faces outside B keep A's normals; B's
// faces strictly inside A return as cavity caps with inverted normals
// and reversed loops (the parallel-plane rule skips the coplanar
// contact class). Returns a new BRepBody. `tol` is REQUIRED
// (ADR-0004).
std::unique_ptr<BRepBody> bool_subtract(
    const BRepBody& A, const BRepBody& B,
    const BooleanTolerance& tol);

} // namespace CAD_0::brep
