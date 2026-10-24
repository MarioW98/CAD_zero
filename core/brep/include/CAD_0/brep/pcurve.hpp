// core/brep/include/CAD_0/brep/pcurve.hpp
//
// The p-curve vocabulary — ADR-0032 D2.1 (G1 unit 1).
//
// The parameter-domain curve set of the B-Rep layer IS the sketch
// kernel's typed 2D entity set (ADR-0030 D3), reused verbatim under
// B-Rep-side names:
//
//     Line2   == sdf::SketchSegment   (a → b, region side interior_left)
//     Circle2 == sdf::SketchCircle    (center, radius, disc | complement)
//     Arc2    == sdf::SketchArc       (center, radius, span, region side)
//     PCurve  == sdf::SketchEntity    (the typed chain entity itself)
//
// ONE opinion, one code — the ADR-0030 D5.5 lesson (the UI calls the
// kernel's validation, never a second copy) applied at the kernel
// layer: the p-curve of a trimming loop and the sketch chain that
// produced it are the same types, built by the same factories,
// validated by the same construction-time rules — so they cannot
// disagree. A second 2D vocabulary beside this one would be exactly
// the divergence ADR-0032 exists to prevent; the tests pin the type
// identity with static_asserts, so a fork stops compiling.
//
// Semantics pinned for the writers (G1 units 3–4; unit 1 only plants
// the field — no path writes a p-curve yet, and a test pins that):
//
//   * DOMAIN — the curve lives in the parameter domain (u, v) of the
//     face whose loop owns the co-edge that carries it. The same 3D
//     edge carries DIFFERENT p-curves in different faces' domains:
//     that pair (3D space reference, 2D p-curve) is the defining
//     structure of the exact B-Rep (D2.2).
//   * DIRECTION — the p-curve is stored in the CO-EDGE'S TRAVERSAL
//     direction: its start is the image of the co-edge's start
//     vertex, its end the image of the end vertex. A loop's p-curves
//     therefore chain endpoint-to-endpoint in the domain, the same
//     connectedness discipline the sketch chain already enforces
//     (ADR-0030 D2).
//   * REGION SIDE — the entity's orientation bit (interior_left /
//     hole) keeps its sketch meaning: the entity's REGION is where
//     the FACE'S MATERIAL lies in the domain. Loops authored per the
//     sketch discipline (material on the left of traversal) read CCW
//     as outer loops and CW as inner loops — one convention from the
//     sketcher to the trimming loop, with no translation step to get
//     wrong.
//   * FACETED WORLD — bridge-extracted faces (plane-per-triangle,
//     ADR-0026) carry NO p-curves: the field stays empty and the
//     Layer-2 provenance field (topology.hpp, D2.4) says
//     approximated(tol). The invariant is per-world, never fudged.
#pragma once

#include "CAD_0/sdf/sketch_entities.hpp"

namespace CAD_0::brep {

using Line2 = sdf::SketchSegment;
using Circle2 = sdf::SketchCircle;
using Arc2 = sdf::SketchArc;
using PCurve = sdf::SketchEntity;

} // namespace CAD_0::brep
