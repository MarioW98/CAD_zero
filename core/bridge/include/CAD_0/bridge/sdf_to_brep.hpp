// core/bridge/include/CAD_0/bridge/sdf_to_brep.hpp
//
// Bridge: SDF → B-Rep conversion (ADR-0026, Phase E).
//
// Two paths:
//
//   1. Primitive fitting: an SDF that IS a known primitive (sphere,
//      box, cylinder) converts to the exact analytic B-Rep
//      (seam-wired, is_closed_manifold since ADR-0023).
//
//   2. Faceted extraction: everything else goes through MC33
//      marching cubes + integer-keyed vertex welding (ADR-0015/0016);
//      each welded triangle becomes one Face on a PlaneSurface
//      (ADR-0026 D3 — 1:1 granularity, no planar merging). The body
//      is assembled with the ADR-0025 v5 patterns: canonical vertex
//      identity (1:1 with the welded mesh), edges keyed by the
//      (min, max) vertex pair, coedge orientation derived from the
//      edge's intrinsic direction vs the loop's traversal direction,
//      link_radial_edges, and the 4-invariant is_closed_manifold
//      contract as a hard postcondition.
//
// Error contract (ADR-0026 D2 + acceptance correction 2 — visible,
// actionable, never silent):
//
//   - empty surface (nothing to extract)              → std::invalid_argument
//   - degenerate triangle (likely on-grid zero, P2)   → std::runtime_error
//     naming the remedy: offset the sampling grid or change the
//     resolution
//   - is_closed_manifold postcondition failure        → std::runtime_error
//     (bridge bug, or input outside the supported class P1/P2/P3)
//
#pragma once

#include "CAD_0/sdf/field.hpp"
#include "CAD_0/sdf/mesh_extract.hpp"
#include "CAD_0/brep/topology.hpp"
#include "CAD_0/brep/analytic.hpp"
#include "CAD_0/brep/factory.hpp"

#include <cstdint>
#include <memory>
#include <string>

namespace CAD_0::bridge {

// Options for SDF → B-Rep conversion.
struct SdfToBrepOptions {
    std::uint32_t march_resolution{48};
    bool fit_primitives{true};      // try to detect known primitives
    bool compute_normals{true};
};

// Convert an SDF body to a faceted B-Rep body (ADR-0026).
// Each welded MC triangle becomes one Face with a PlaneSurface.
// Throws per the error contract above — never returns a silently
// broken body.
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

// Topology report (ADR-0026 consumer step — the app's Debug action).
//
// Runs the same dispatch as sdf_to_brep() and summarizes the result as
// plain values, so callers that only need to INSPECT the outcome (the
// Python binding behind the app's Debug menu) need no BRepBody API at
// all. The fields mirror the ADR-0026 D2 contract: V/E/F depend on the
// sampling grid; chi, closed and loops do not. `fit` names the path that
// produced the body: "sphere" | "box" | "cylinder" (exact analytic
// primitive) or "faceted" (MC33 extraction).
//
// Throws exactly what the underlying conversion throws
// (std::invalid_argument / std::runtime_error with their actionable
// messages) — this wrapper never masks a bridge error.
struct SdfToBrepReport {
    std::size_t vertex_count{0};  // V
    std::size_t edge_count{0};    // E
    std::size_t face_count{0};    // F
    std::int64_t chi{0};          // euler_characteristic: V - E + F
    bool closed{false};           // is_closed_manifold (ADR-0022 §7)
    bool loops{false};            // loops_are_closed
    std::string fit;              // "sphere"|"box"|"cylinder"|"faceted"
};

SdfToBrepReport sdf_to_brep_report(
    const sdf::SDFBody& sdf_body,
    const SdfToBrepOptions& opts = {});

} // namespace CAD_0::bridge
