// core/bridge/include/CAD_0/bridge/sdf_to_brep.hpp
//
// Bridge: SDF → B-Rep conversion.
//
// Converts an SDF body into a B-Rep body by:
//   1. Extracting a triangle mesh via marching cubes
//   2. Detecting planar patches in the mesh (groups of coplanar triangles)
//   3. Creating a faceted B-Rep where each planar patch becomes a Face
//      with a PlaneSurface, and the triangle edges become Edges.
//
// For non-planar regions (spheres, cylinders), the mesh is kept as
// individual triangles, each becoming a tiny planar Face.
//
// Also provides primitive fitting: detects if the SDF is a known
// primitive (sphere, box, cylinder) and creates the exact analytic
// B-Rep instead of a faceted approximation.
//
#pragma once

#include "CAD_0/sdf/field.hpp"
#include "CAD_0/sdf/mesh_extract.hpp"
#include "CAD_0/brep/topology.hpp"
#include "CAD_0/brep/analytic.hpp"
#include "CAD_0/brep/factory.hpp"

#include <memory>

namespace CAD_0::bridge {

// Options for SDF → B-Rep conversion.
struct SdfToBrepOptions {
    std::uint32_t march_resolution{48};
    float planar_tolerance{0.01f};  // max deviation from plane to be "planar"
    bool fit_primitives{true};      // try to detect known primitives
    bool compute_normals{true};
};

// Convert an SDF body to a faceted B-Rep body.
// Each triangle in the extracted mesh becomes a Face with a PlaneSurface.
std::unique_ptr<brep::BRepBody> sdf_to_brep_faceted(
    const sdf::SDFBody& sdf_body,
    const SdfToBrepOptions& opts = {});

// Try to detect if the SDF is a known primitive and create an exact
// B-Rep body for it. Returns nullptr if no primitive is detected.
std::unique_ptr<brep::BRepBody> sdf_to_brep_primitive(
    const sdf::SDFBody& sdf_body);

// High-level: tries primitive fitting first, falls back to faceted.
std::unique_ptr<brep::BRepBody> sdf_to_brep(
    const sdf::SDFBody& sdf_body,
    const SdfToBrepOptions& opts = {});

} // namespace CAD_0::bridge
