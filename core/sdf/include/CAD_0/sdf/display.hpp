// core/sdf/include/CAD_0/sdf/display.hpp
//
// The display pipeline's normal buffer — ONE definition, two roads
// (PERF 4):
//
//   * the SYNC road: the viewport's refresh builds the mirror from the
//     LIVE node (CAD_0.sdf.display_normals through the bindings — the
//     graphics-sprint road, unchanged);
//   * the ASYNC road: the display worker derives the whole display
//     payload off the UI thread from a SNAPSHOT (scene::
//     derive_display_mesh, core/scene/scene.hpp) so the gesture never
//     blocks on the derivation.
//
// Both roads must answer byte-identical normals for the same body and
// the same positions — so the arithmetic lives HERE, once, and both
// callers call it. The binding (bindings/python/src/sdf_bindings.cpp)
// keeps its ndarray plumbing but delegates the loop; the scene's
// derive calls it directly. A second copy of this loop anywhere would
// be a drift risk the bit-identity pins exist to refuse.
//
// DISPLAY PIPELINE ONLY: the MC mesh itself (positions, indices, mesh
// normals) is never touched here — the exporters, the bridge and the
// golden files consume mesh.normals and stay byte-identical (PERF 1's
// own contract, quoted and kept).
#pragma once

#include "CAD_0/sdf/field.hpp"
#include "CAD_0/math/vec.hpp"

#include <cstddef>
#include <span>
#include <vector>

namespace CAD_0::sdf {

// The display-only normal buffer for a set of mesh vertices: for every
// position p, n = normalize(body.sample(p).gradient) — the kernel's
// EXACT analytic gradient (never finite differences). On a CSG min the
// gradient is piecewise: smooth across curved regions, and it picks
// ONE operand's direction at a live edge (never an average across the
// crease) — the CAD look. This is strictly better than the MC mesh
// normals, which interpolate the gradients of the two GRID CORNERS
// flanking a crossing: near a crease those corners can sit in
// different CSG regions and the lerp smears the edge dark.
//
// Degenerate vertices (|grad| <= 1e-9, e.g. a sphere's center) keep
// the supplied mesh normal — or (0,0,0) when none is supplied — so the
// buffer NEVER contains NaN (a NaN length fails the > comparison and
// takes the same fallback). The guard constant is the kernel's own
// (interp_gradient_abs, CylinderSDF's axis guard): below it a
// direction is not defined.
//
// `mesh_normals` may be empty (or shorter than positions — no fallback
// for that row): the caller declares what it has, never a guess.
std::vector<math::Vec3f> display_normals(
    const SDFBody& body,
    std::span<const math::Vec3f> positions,
    std::span<const math::Vec3f> mesh_normals);

} // namespace CAD_0::sdf
