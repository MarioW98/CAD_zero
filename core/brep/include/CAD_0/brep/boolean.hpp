// core/brep/include/CAD_0/brep/boolean.hpp
//
// Boolean operations on planar-faced convex polyhedra.
// ADR-0023 (intersect, Sutherland-Hodgman), ADR-0024 (union/subtract),
// ADR-0025 (the slab decomposition, the parallel-plane rule, the
// T-junction subdivision, and the v5 assembly this file documents).
//
// History note (the registry, corrected by H2 unit 1): this header
// spent its first phases declaring bool_union/bool_subtract "NOT
// IMPLEMENTED" while the .cpp implemented them (ADR-0025) — the
// declaration lagged the code. Both declarations below are now the
// code's truth, enforced.
#pragma once

#include "CAD_0/brep/topology.hpp"

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

// bool_intersect — compute A ∩ B for two convex polyhedra.
//
// Algorithm: for each face of A, clip its polygon against every face
// plane of B (keeping the inside portion). Do the same for B's faces
// against A. Weld the resulting vertices, build edges, link radial.
// Returns a new BRepBody.
std::unique_ptr<BRepBody> bool_intersect(const BRepBody& A, const BRepBody& B);

// bool_union — compute A ∪ B.
//
// Algorithm (ADR-0025): boundary(A ∪ B) = {A's faces outside B} ∪
// {B's faces outside A}; the outside parts come from the slab
// decomposition, interface points through the single-source
// IntersectionTable, the identical-body shortcut when A == B.
// Returns a new BRepBody.
std::unique_ptr<BRepBody> bool_union(const BRepBody& A, const BRepBody& B);

// bool_subtract — compute A \ B.
//
// Algorithm (ADR-0025): A's faces outside B keep A's normals; B's
// faces strictly inside A return as cavity caps with inverted normals
// and reversed loops (the parallel-plane rule skips the coplanar
// contact class). Returns a new BRepBody.
std::unique_ptr<BRepBody> bool_subtract(const BRepBody& A, const BRepBody& B);

} // namespace CAD_0::brep
