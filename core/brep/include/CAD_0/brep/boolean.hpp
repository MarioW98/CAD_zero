// core/brep/include/CAD_0/brep/boolean.hpp
//
// Boolean operations on B-Rep bodies — ADR-0023.
//
// Phase D step (c) implements:
//   * bool_intersect — Sutherland-Hodgman clipping for convex polyhedra.
//
// Declared but NOT implemented (throw std::runtime_error):
//   * bool_union      — requires polygon subtraction (non-convex clip).
//   * bool_subtract   — requires polygon subtraction (non-convex clip).
//
// These two are deferred to a later ADR (Phase D step (c+) or Phase E)
// because they cannot use Sutherland-Hodgman: the clipping region is
// the complement of a convex polyhedron, which is non-convex.
//
#pragma once

#include "CAD_0/brep/topology.hpp"

#include <memory>

namespace CAD_0::brep {

// bool_intersect — compute A ∩ B for two convex polyhedra.
//
// Algorithm: for each face of A, clip its polygon against every face
// plane of B (keeping the inside portion). Do the same for B's faces
// against A. Weld the resulting vertices, build edges, link radial.
//
// Requirements:
//   * A and B must be convex polyhedra (every face is a convex polygon).
//   * Every face of A and B must have exactly one outer loop.
//   * The surface of every face must be a PlaneSurface (curved-surface
//     boolean is NOT supported — deferred to a later step).
//
// Throws std::invalid_argument if the inputs are not convex polyhedra
// with planar faces.
//
// Returns a new BRepBody. The inputs are not modified.
std::unique_ptr<BRepBody> bool_intersect(const BRepBody& A, const BRepBody& B);

// bool_union — compute A ∪ B.
//
// NOT IMPLEMENTED — throws std::runtime_error.
// Requires polygon subtraction (face minus convex region), which is
// non-convex and cannot use Sutherland-Hodgman. Deferred to a later
// ADR (Phase D step (c+) or Phase E).
std::unique_ptr<BRepBody> bool_union(const BRepBody& A, const BRepBody& B);

// bool_subtract — compute A \ B.
//
// NOT IMPLEMENTED — throws std::runtime_error.
// Same limitation as bool_union.
std::unique_ptr<BRepBody> bool_subtract(const BRepBody& A, const BRepBody& B);

} // namespace CAD_0::brep
