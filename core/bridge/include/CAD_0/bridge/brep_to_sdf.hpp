// core/bridge/include/CAD_0/bridge/brep_to_sdf.hpp
//
// Bridge: B-Rep → SDF conversion.
//
// Converts a B-Rep body into an SDF body: the exact SDF primitive
// when the analytic shape is recognized, the mesh bridge otherwise.
//
// The mesh bridge (mesh_to_sdf) answers the signed distance to the
// nearest triangle with the sign from the GENERALIZED WINDING NUMBER
// (H2-u4 m1; Jacobson, Kavan & Sorkine 2013): w(x) = (1/4π)·Σᵢ Ωᵢ(x),
// the sum of the signed solid angles the triangles subtend at the
// query — integer inside (1 for one outward shell), 0 outside, for
// ANY closed mesh, convex or not. The whole sign path computes in
// double (the H1/ADR-0034 discipline; float32 vertices and the
// float32 query enter double exactly, once, at the boundary).
//
// THE HIERARCHY (H2-u4 m2): the winding sum is evaluated over a
// deterministic AABB tree built once at load — far nodes contribute
// through their dipole proxy under the DECLARED approximation
// budget ε_w, every approximation bounded by the derived ε_node and
// the accumulated bound composed honestly with morso 1's sign band
// (ADR-0035 D2.5). The classification is identical to the exact
// evaluation's (derived), the answered distance is exact, the builds
// are byte-identical (ADR-0014), and the exact O(F) evaluation
// survives as the test seam's reference with the budget-zero
// counterproof: the machine can still answer without approximating.
// Queries are const and serial (ADR-0009's shipped world) — no
// parallelism here in secret.
//
// THE CONTRACT — WATERTIGHT ONLY: the winding number is defined only
// on CLOSED, consistently oriented meshes. mesh_to_sdf validates the
// mesh at load (O(E), once) and REFUSES open meshes with an
// actionable error that names the free edges and the triangles that
// border the hole — never a mute "invalid mesh".
//
// For analytic B-Rep primitives (box, sphere, cylinder), we detect
// the shape type and create the exact SDF primitive directly — that
// path needs no mesh and no watertightness.
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
// the exact SDF is created. Otherwise, the body is tessellated and
// handed to mesh_to_sdf — which refuses non-watertight meshes by
// name (the tessellator's own output is NOT watertight today: the
// fallback refuses it actionably rather than answer with a wrong
// sign — the honest stub, on the record).
std::unique_ptr<sdf::SDFBody> brep_to_sdf(const brep::BRepBody& body);

// Create a mesh-based SDF from a triangle mesh — WATERTIGHT ONLY.
// The SDF answers the signed distance to the nearest triangle; the
// sign is the generalized winding number, computed in double over
// the closed mesh. Throws std::invalid_argument, naming the free
// edges and their triangles, when the mesh is not closed and
// consistently oriented. An empty mesh yields an empty body (no
// geometry, no contract to violate).
std::unique_ptr<sdf::SDFBody> mesh_to_sdf(const sdf::TriangleMesh& mesh);

} // namespace CAD_0::bridge
