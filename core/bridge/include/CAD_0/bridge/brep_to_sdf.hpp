// core/bridge/include/CAD_0/bridge/brep_to_sdf.hpp
//
// Bridge: B-Rep → SDF conversion.
//
// Converts a B-Rep body into an SDF body by computing the signed
// distance field from the B-Rep's tessellated mesh.
//
// For MVP, this creates a mesh-based SDF: the signed distance is
// computed as the minimum distance to any triangle, with the sign
// determined by the surface normal direction.
//
// For analytic B-Rep primitives (box, sphere, cylinder), we detect
// the shape type and create the exact SDF primitive directly.
//
#pragma once

#include "CAD_0/brep/topology.hpp"
#include "CAD_0/brep/factory.hpp"
#include "CAD_0/brep/tessellate.hpp"
#include "CAD_0/sdf/field.hpp"
#include "CAD_0/sdf/primitives.hpp"
#include "CAD_0/sdf/operators.hpp"

#include <memory>

namespace CAD_0::bridge {

// Convert a B-Rep body to an SDF body.
// If the B-Rep is a recognized primitive (box, sphere, cylinder),
// the exact SDF is created. Otherwise, a mesh-based SDF is used.
std::unique_ptr<sdf::SDFBody> brep_to_sdf(const brep::BRepBody& body);

// Create a mesh-based SDF from a triangle mesh.
// The SDF evaluates the signed distance to the nearest triangle.
// (Phase E stub — for now returns an empty body for non-primitive shapes.)
std::unique_ptr<sdf::SDFBody> mesh_to_sdf(const sdf::TriangleMesh& mesh);

} // namespace CAD_0::bridge
