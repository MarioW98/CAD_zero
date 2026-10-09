// core/brep/include/CAD_0/brep/tessellate.hpp
//
// Tessellation of B-Rep faces into triangle meshes.
//
// For each face, the surface is sampled on a regular (u,v) grid
// and triangulated. The resolution is adaptive based on curvature:
//   * Planes: minimal subdivision (just the bounding loop)
//   * Cylinders/Spheres: proportional to radius
//   * Tori: proportional to both radii
//
#pragma once

#include "CAD_0/brep/topology.hpp"
#include "CAD_0/sdf/mesh_extract.hpp"

#include <cstdint>

namespace CAD_0::brep {

// Tessellation options.
struct TessellationOptions {
    std::uint32_t resolution{32};  // samples per parametric axis
    bool compute_normals{true};
};

// Tessellate a single face into a TriangleMesh.
// The face's surface is sampled on a (resolution × resolution) grid
// and triangulated. The loops are used to determine which grid points
// are inside the face boundary (simple bbox test for MVP).
sdf::TriangleMesh tessellate_face(const Face& face, const TessellationOptions& opts = {});

// Tessellate an entire BRepBody.
sdf::TriangleMesh tessellate_body(const BRepBody& body, const TessellationOptions& opts = {});

} // namespace CAD_0::brep
