// bindings/python/src/brep_bindings.cpp
//
// The B-Rep query surface (ADR-0032 D8 + G1 unit 2, the H1 contract
// taking shape): "cosa è questa faccia" as plain values, so a
// preprocessor consuming CAD_0 bodies never needs a B-Rep API —
//
//   * CAD_0.brep.face_count(body) — how many faces to ask about;
//   * CAD_0.brep.face_query(body, face_id) — the face's family
//     (plane/cylinder/sphere/...), loop counts (outer + inner),
//     per-co-edge p-curve type + readable parameters, and the
//     Layer-2 provenance record (exact/0 vs approximated(tol)).
//   * CAD_0.brep.mass_properties(body, tol=1e-6) — the divergence
//     theorem over the exact faces (G1 unit 5, ADR-0032 D4): volume,
//     centroid, inertia_about_centroid (density 1) and the all_exact
//     honesty flag; a faceted body answers the polyhedral value with
//     the flag False — never silently mixed.
//   * CAD_0.brep.body_query(body) — the whole body as plain values
//     (G1 unit 6, D8 one rung up): the counts V/E/F/H, χ = V−E+F−H,
//     the genus under the ADR-0027 D6 rule, the structural flags,
//     and the per-kind provenance tallies — the report's single
//     source beside mass_properties.
//   * CAD_0.brep.ssi_query(body_a, face_a, body_b, face_b, tol) —
//     THE TABLE (G2 unit 1, ADR-0033 D5.1): the analytic SSI entries
//     queryable BEFORE any boolean consumes them (the face-query
//     precedent). The intersections answer as plain values: the
//     kind, the 3D curves (circles with their derived in-plane
//     frames, lines with point + direction), the typed p-curves in
//     BOTH domains (the same payload dicts face_query serves), the
//     re-frame declaration when an entry derived one, and the
//     Layer-2 provenance (exact/0 — closed forms on exact faces).
//     The out-of-table pairs are RETURNED records (kind
//     "out_of_table", the message naming the pair, the disposition
//     class and the remedies) — the capability ceiling is explicit
//     and queryable, a recorded answer, never a missing one.
//   * CAD_0.brep.translate_body(body, delta) — the exact side's own
//     placement operation (the subtract's tool-offset path, G1 unit
//     4), exposed so author bodies can be POSITIONED for the query:
//     the bridge's primitive fit serves origin-centered bodies only,
//     and the sphere pairs / the offset cylinder pairs need distance.
//     In place, like the C++ act; the loops, co-edges and p-curves
//     are untouched (the frames move with their surfaces).
//   * CAD_0.brep.edge_count(body) / CAD_0.brep.edge_query(body,
//     edge_id) — the edge's answer (ADR-0036 D1, H2-u6 morso 2):
//     endpoints, manifoldness, the Layer-2 record, and the NAME
//     beside the address (the fillet's references are edge-granular
//     — D4's first consumer). The same loud-miss culture as the
//     face's.
//   * CAD_0.brep.make_prism(chains, distance, direction, feature=0)
//     — the exact extrude (H2-u6 morso 3): the sketch chains become
//     the prism body, carrying the WALK-ROLE NAMES from the same
//     pass that builds (ADR-0036 D1: lat(i)/cap(lo)/cap(hi)/seam,
//     the multi-chain group prefix 'c<g>:' on the chain-owned
//     roles). The chains are CAD_0.scene's own SketchChain values
//     (sketch_circle/sketch_segment/sketch_arc + SketchChain) — the
//     recipe doctrine's typed payloads, untranslated; the refusals
//     (the certificate's, the exact-class boundary's) cross as
//     ValueError carrying the C++ diagnosis + remedy verbatim.
//
// BRepBody arrives as the OPAQUE handle the bridge submodule already
// registers (one registration per module — the factories' bodies and
// the bridge's extractions are the same Python type, by design: the
// query serves both worlds). Bodies enter Python through
// CAD_0.bridge.sdf_to_brep (the dispatching conversion: exact
// primitive fit first, MC33 faceted fallback).
//
// The dict layout (plain values, the sdf_to_brep_report pattern):
//
//   {"family": str, "has_surface": bool,
//    "loop_count": int, "outer_loop_count": int, "inner_loop_count": int,
//    "exact": bool, "linear_tolerance": float,
//    "name": str, "shape_id": {"feature": int, "path": [int],
//                              "generation": int},
//    "loops": [ {"is_outer": bool,
//                "coedges": [ {"edge": int, "reversed": bool,
//                               "pcurve": None | line/circle/arc dict } ]} ]}
//
// with the p-curve payloads mirroring the sketch kernel's typed
// entities verbatim: line {a, b, interior_left}, circle {center,
// radius, hole}, arc {center, radius, hole, start_angle, sweep} —
// the same numbers the sketcher would read, untranslated (D2.1).
// The name/shape_id pair (ADR-0036 D1): the string is the CANONICAL
// RENDER of the structure ("box#1:top", "ext#3:lat(0)") — empty
// when honestly unnamed (the v1 named classes: the three factories
// — box, sph, cyl — since morse 2, and the extrude's faces and
// seam edges since morse 3); the shape_id dict is the STRUCTURE the
// kernel compares (never the string — the kernel does not parse
// names).
//
// Errors: a face id that does not exist crosses as a ValueError
// carrying the C++ message's diagnosis + remedy verbatim (the
// nanobind default translation of std::invalid_argument — nothing
// here catches or rewraps).
#include <nanobind/nanobind.h>
#include <nanobind/make_iterator.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/unique_ptr.h>
#include <nanobind/stl/vector.h>
#include <nanobind/stl/optional.h>
#include <nanobind/stl/pair.h>
#include <nanobind/stl/tuple.h>
#include <deque>

#include "CAD_0/brep/topology.hpp"
#include "CAD_0/brep/naming.hpp"
// The polygon booleans' typed bundle (H2-u5 m2: bool_union at the
// boundary, the door reachable from Python).
#include "CAD_0/brep/boolean.hpp"
#include "CAD_0/brep/query.hpp"
#include "CAD_0/brep/descent.hpp"
#include "CAD_0/brep/resolve.hpp"
#include "CAD_0/brep/mass_properties.hpp"
#include "CAD_0/brep/ssi.hpp"
#include "CAD_0/brep/subtract.hpp"
#include "CAD_0/brep/extrude.hpp"
// THE FILLET, class 1 (G2-u3 m2, ADR-0039 D1.1): the rolling ball
// on the box's edge — the binding the ladder's m2 asks for (the
// flank reachable from Python; the m6 click-test is the phase's
// exit, declared and deferred).
#include "CAD_0/brep/fillet.hpp"
#include "CAD_0/brep/chain_gate.hpp"
#include "CAD_0/brep/survival_ledger.hpp"
#include "CAD_0/sdf/sketch_profile.hpp"

namespace nb = nanobind;
using CAD_0::brep::BRepBody;
using CAD_0::brep::BodySummary;
using CAD_0::brep::FaceId;
using CAD_0::brep::FaceInfo;
using CAD_0::brep::EdgeInfo;
using CAD_0::brep::LoopInfo;
using CAD_0::brep::PCurve;
using CAD_0::brep::PCurveInfo;
using CAD_0::brep::ProvenanceTally;
using CAD_0::brep::SsiCurve;
using CAD_0::brep::SsiDomainAnswer;
using CAD_0::brep::SsiResult;

namespace {

// One p-curve as plain values — None in the faceted world, else the
// typed payload verbatim (the sketch kernel's own parameters).
nb::object pcurve_obj(const PCurveInfo& p) {
    switch (p.kind) {
    case PCurveInfo::Kind::None:
        return nb::none();
    case PCurveInfo::Kind::Line: {
        nb::dict d;
        d["type"] = "line";
        d["a"] = nb::make_tuple(p.a.x, p.a.y);
        d["b"] = nb::make_tuple(p.b.x, p.b.y);
        d["interior_left"] = p.interior_left;
        return d;
    }
    case PCurveInfo::Kind::Circle: {
        nb::dict d;
        d["type"] = "circle";
        d["center"] = nb::make_tuple(p.center.x, p.center.y);
        d["radius"] = p.radius;
        d["hole"] = p.hole;
        return d;
    }
    case PCurveInfo::Kind::Arc: {
        nb::dict d;
        d["type"] = "arc";
        d["center"] = nb::make_tuple(p.center.x, p.center.y);
        d["radius"] = p.radius;
        d["hole"] = p.hole;
        d["start_angle"] = p.start_angle;
        d["sweep"] = p.sweep;
        return d;
    }
    }
    return nb::none();  // unreachable — all enum cases handled above
}

nb::dict face_info_dict(const FaceInfo& info) {
    nb::dict out;
    out["family"] = info.family;
    out["has_surface"] = info.has_surface;
    out["loop_count"] = info.loop_count;
    out["outer_loop_count"] = info.outer_loop_count;
    out["inner_loop_count"] = info.inner_loop_count;
    out["exact"] = info.exact;
    out["linear_tolerance"] = info.linear_tolerance;
    // The name (ADR-0036 D1, H2-u6 morso 2): the canonical render —
    // the empty string is the honest "unnamed" answer (the v1
    // named classes are the three factories), and the shape_id
    // dict carries the STRUCTURE (feature + path of role-table
    // indices + generation) — equality and comparison are
    // structural, the string is the record for the report and the
    // human. The id fields read as default when the name is empty.
    out["name"] = info.name;
    nb::dict sid;
    sid["feature"] = info.id.feature.value;
    nb::list path;
    for (std::uint8_t i = 0; i < info.id.subshape.depth; ++i)
        path.append(info.id.subshape.path[i]);
    sid["path"] = path;
    sid["generation"] = info.id.generation.value;
    out["shape_id"] = sid;
    nb::list loops;
    for (const LoopInfo& loop : info.loops) {
        nb::dict ld;
        ld["is_outer"] = loop.is_outer;
        nb::list coedges;
        for (const PCurveInfo& ce : loop.coedges) {
            nb::dict cd;
            cd["edge"] = ce.edge;
            cd["reversed"] = ce.reversed;
            cd["pcurve"] = pcurve_obj(ce);
            coedges.append(cd);
        }
        ld["coedges"] = coedges;
        loops.append(ld);
    }
    out["loops"] = loops;
    return out;
}

// One edge's answer as plain values (ADR-0036 D1, morso 2): the
// endpoints, the manifoldness, the Layer-2 record, and the name
// with the same honest-empty rule as the face's.
nb::dict edge_info_dict(const EdgeInfo& info) {
    nb::dict out;
    out["v0"] = info.v0;
    out["v1"] = info.v1;
    out["manifold"] = info.manifold;
    out["coedge_count"] = info.coedge_count;
    out["exact"] = info.exact;
    out["linear_tolerance"] = info.linear_tolerance;
    out["name"] = info.name;
    nb::dict sid;
    sid["feature"] = info.id.feature.value;
    nb::list path;
    for (std::uint8_t i = 0; i < info.id.subshape.depth; ++i)
        path.append(info.id.subshape.path[i]);
    sid["path"] = path;
    sid["generation"] = info.id.generation.value;
    out["shape_id"] = sid;
    return out;
}

// One kind's provenance tally as plain values (D8): {"exact": int,
// "approximated": int, "max_linear_tolerance": float} — the pairing
// exact|0 vs approximated|tol is the contract, never laundered.
nb::dict tally_dict(const ProvenanceTally& t) {
    nb::dict d;
    d["exact"] = t.exact;
    d["approximated"] = t.approximated;
    d["max_linear_tolerance"] = t.max_linear_tolerance;
    return d;
}

// The whole body as plain values (G1 unit 6): the counts, χ with
// its H term, the genus under the D6 rule (None when not derived —
// a multi-shell body reports n/d, never a fabricated half-integer),
// the structural flags, and the per-kind provenance tallies.
nb::dict body_summary_dict(const BodySummary& s) {
    nb::dict out;
    out["V"] = s.vertex_count;
    out["E"] = s.edge_count;
    out["F"] = s.face_count;
    out["H"] = s.inner_loops;
    out["chi"] = s.chi;
    out["orientable"] = s.orientable;
    out["loops_closed"] = s.loops_closed;
    out["genus"] = s.genus_derived ? nb::int_(s.genus) : nb::none();
    out["vertices"] = tally_dict(s.vertices);
    out["edges"] = tally_dict(s.edges);
    out["faces"] = tally_dict(s.faces);
    return out;
}

// One typed p-curve as plain values — None in the faceted world, else
// the typed payload verbatim (the sketch kernel's own parameters).
// This raw-PCurve flavor serves the SSI answer (the domain answers
// carry SketchEntity values, not co-edges) — the SAME payload mapping
// pcurve_obj() serves for the face query, one opinion, no drift.
nb::object pcurve_obj_raw(const PCurve& p) {
    switch (p.kind) {
    case PCurve::Kind::Segment: {
        nb::dict d;
        d["type"] = "line";
        d["a"] = nb::make_tuple(p.segment.a.x, p.segment.a.y);
        d["b"] = nb::make_tuple(p.segment.b.x, p.segment.b.y);
        d["interior_left"] = p.segment.interior_left;
        return d;
    }
    case PCurve::Kind::Circle: {
        nb::dict d;
        d["type"] = "circle";
        d["center"] = nb::make_tuple(p.circle.center.x, p.circle.center.y);
        d["radius"] = p.circle.radius;
        d["hole"] = p.circle.hole;
        return d;
    }
    case PCurve::Kind::Arc: {
        nb::dict d;
        d["type"] = "arc";
        d["center"] = nb::make_tuple(p.arc.center.x, p.arc.center.y);
        d["radius"] = p.arc.radius;
        d["hole"] = p.arc.hole;
        d["start_angle"] = p.arc.start_angle;
        d["sweep"] = p.arc.sweep;
        return d;
    }
    }
    return nb::none();  // unreachable — all enum cases handled above
}

// One operand's domain answer: the family, the re-frame declaration,
// and the typed p-curves in curve order.
nb::dict ssi_domain_dict(const SsiDomainAnswer& d) {
    nb::dict out;
    out["reframed"] = d.reframe_needed;
    out["frame_pole"] = d.reframe_needed
                             ? nb::make_tuple(double(d.frame_pole.x),
                                              double(d.frame_pole.y),
                                              double(d.frame_pole.z))
                             : nb::none();
    out["frame_azimuth"] = d.reframe_needed
                               ? nb::make_tuple(double(d.frame_azimuth.x),
                                                double(d.frame_azimuth.y),
                                                double(d.frame_azimuth.z))
                               : nb::none();
    nb::list pcs;
    for (const PCurve& p : d.pcurves) pcs.append(pcurve_obj_raw(p));
    out["pcurves"] = pcs;
    return out;
}

// The SSI answer as plain values — the dict layout:
//
//   {"kind": "empty" | "point" | "circle" | "circle_pair" |
//            "line_pair" | "tangent_line" | "coincident" |
//            "out_of_table",
//    "exact": bool, "linear_tolerance": float,
//    "families": (str, str),
//    "point": (x, y, z) | None,
//    "curves": [ {"type": "circle", "center": (x,y,z), "radius": r,
//                 "u_ref": (x,y,z), "v_ref": (x,y,z)}
//               | {"type": "line", "point": (x,y,z), "dir": (x,y,z)} ],
//    "operands": [ {"reframed": bool, "frame_pole": ... | None,
//                   "frame_azimuth": ... | None, "pcurves": [...]}, ... ],
//    "message": str}
//
nb::dict ssi_dict(const SsiResult& r) {
    nb::dict out;
    const char* kind = "out_of_table";
    switch (r.kind) {
    case SsiResult::Kind::Empty: kind = "empty"; break;
    case SsiResult::Kind::Point: kind = "point"; break;
    case SsiResult::Kind::Circle: kind = "circle"; break;
    case SsiResult::Kind::CirclePair: kind = "circle_pair"; break;
    case SsiResult::Kind::LinePair: kind = "line_pair"; break;
    case SsiResult::Kind::TangentLine: kind = "tangent_line"; break;
    case SsiResult::Kind::Coincident: kind = "coincident"; break;
    case SsiResult::Kind::OutOfTable: kind = "out_of_table"; break;
    }
    out["kind"] = kind;
    out["exact"] = r.provenance.is_exact();
    out["linear_tolerance"] = r.provenance.linear_tolerance;
    out["families"] = nb::make_tuple(r.family_a, r.family_b);
    out["point"] = r.kind == SsiResult::Kind::Point
                       ? nb::make_tuple(double(r.point.x), double(r.point.y),
                                        double(r.point.z))
                       : nb::none();
    nb::list curves;
    for (const SsiCurve& c : r.curves) {
        nb::dict d;
        if (c.kind == SsiCurve::Kind::Circle) {
            d["type"] = "circle";
            d["center"] = nb::make_tuple(double(c.center.x),
                                          double(c.center.y),
                                          double(c.center.z));
            d["radius"] = c.radius;
            d["u_ref"] = nb::make_tuple(double(c.u_ref.x),
                                         double(c.u_ref.y),
                                         double(c.u_ref.z));
            d["v_ref"] = nb::make_tuple(double(c.v_ref.x),
                                         double(c.v_ref.y),
                                         double(c.v_ref.z));
        } else {
            d["type"] = "line";
            d["point"] = nb::make_tuple(double(c.point.x),
                                         double(c.point.y),
                                         double(c.point.z));
            d["dir"] = nb::make_tuple(double(c.dir.x), double(c.dir.y),
                                       double(c.dir.z));
        }
        curves.append(d);
    }
    out["curves"] = curves;
    nb::list operands;
    operands.append(ssi_domain_dict(r.a));
    operands.append(ssi_domain_dict(r.b));
    out["operands"] = operands;
    out["message"] = r.message;
    return out;
}

// The subtract's naming context, from the binding's side (H2-u6
// morso 4, ADR-0036 D1): the operand tables are selected by the
// operand's OWN creating feature kind — the caller built the box
// with make_box and the tool with make_prism, so the kind is
// knowledge the caller already holds. The four classes with tables
// today map to the static normative tables; Sketch and Subtract
// have no operand body (the roots are authored data, the subtract
// is the consumer) — a loud refusal, never a guess.
const CAD_0::brep::naming::OperationNaming* naming_table_of(
    CAD_0::brep::naming::FeatureKind kind) {
    namespace n = CAD_0::brep::naming;
    switch (kind) {
    case n::FeatureKind::Box:      return &n::kBoxNaming;
    case n::FeatureKind::Sphere:   return &n::kSphereNaming;
    case n::FeatureKind::Cylinder: return &n::kCylinderNaming;
    case n::FeatureKind::Ellipsoid: return &n::kEllipsoidNaming;
    case n::FeatureKind::Extrude:  return &n::kExtNaming;
    case n::FeatureKind::Sketch:
    case n::FeatureKind::Subtract:
    case n::FeatureKind::Fillet:
        throw std::invalid_argument(
            "the naming-table selector: the operand kind has no "
            "naming table - the sketch's roots are authored data "
            "(named by the root reference, never a role table) and "
            "the subtract and the fillet are CONSUMERS, not "
            "operands; remedy: pass the CREATING operation's kind "
            "(Box, Sphere, Cylinder, Ellipsoid or Extrude) - the one "
            "that built the operand body.");
    }
    return nullptr;  // unreachable — all enum cases handled above
}

} // namespace

void bind_brep(nb::module_& m) {
    auto brep_mod = m.def_submodule(
        "brep", "B-Rep query surface: what a face is, as plain values");

    brep_mod.def("face_count",
        [](const BRepBody& body) { return body.face_count(); },
        nb::arg("body"),
        "Number of faces in the body's shell — the valid range for\n"
        "face_query's face_id (ids come from the body's own face\n"
        "numbering; the factories number by shell order).");

    brep_mod.def("face_query",
        [](const BRepBody& body, FaceId face_id) {
            return face_info_dict(CAD_0::brep::query_face(body, face_id));
        },
        nb::arg("body"), nb::arg("face_id"),
        "Ask a face what it is, as plain values: the surface family\n"
        "(\"plane\" | \"cylinder\" | \"sphere\" | ...), the loop counts\n"
        "(outer + inner), the Layer-2 provenance (exact/0 for the\n"
        "analytic world, the declared bound for the faceted one), and\n"
        "every co-edge's p-curve — its 3D edge reference and its typed\n"
        "2D parameters in the owning face's domain (line: a, b,\n"
        "interior_left; circle: center, radius, hole; arc: center,\n"
        "radius, hole, start_angle, sweep) — or None in the faceted\n"
        "world, where the provenance says approximated and the\n"
        "distinction is never a guess. The name (ADR-0036 D1) is the\n"
        "entity's identity beside its address — the canonical render\n"
        "of the shape_id dict, empty when honestly unnamed. Raises\n"
        "ValueError for a face id the body does not carry (diagnosis\n"
        "+ remedy verbatim).");

    brep_mod.def("edge_count",
        [](const BRepBody& body) { return body.edge_count(); },
        nb::arg("body"),
        "Number of edges in the body's own array — the valid range\n"
        "for edge_query's edge_id.");

    brep_mod.def("edge_query",
        [](const BRepBody& body, CAD_0::brep::EdgeId edge_id) {
            return edge_info_dict(
                CAD_0::brep::query_edge(body, edge_id));
        },
        nb::arg("body"), nb::arg("edge_id"),
        "Ask an edge what it is, as plain values (the face query's\n"
        "own contract one entity kind over — ADR-0036 D1): the\n"
        "endpoints (v0, v1), the manifoldness (exactly two\n"
        "co-edges), the Layer-2 provenance, and the NAME — the\n"
        "canonical render beside the address, empty when honestly\n"
        "unnamed (the fillet references edges: D4's first consumer\n"
        "is edge-granular). Raises ValueError for an edge id the\n"
        "body does not carry (diagnosis + remedy verbatim).");

    // THE EXACT EXTRUDE (H2-u6 morso 3, ADR-0032 D6 + ADR-0036 D1):
    // the sketch chains become the prism body, and the names are
    // stamped in the pass that builds — the walk roles lat(i) /
    // cap(lo) / cap(hi) / seam, the multi-chain group prefix 'c<g>:'
    // on the chain-owned roles (the numbering table's own layout:
    // the outer first — group 0 — then each hole in sketch order).
    // The `feature` argument is the DAG node's FeatureId (the
    // factories' own parameter, defaulted to 0: the standalone
    // caller reads ext#0 — the token is the pin, the id is the
    // caller's to supply).
    brep_mod.def(
        "make_prism",
        [](const std::vector<CAD_0::sdf::SketchChain>& chains,
           double distance,
           int direction, std::uint64_t feature) {
            return CAD_0::brep::make_prism(
                chains, distance, direction,
                CAD_0::geometry::FeatureId{feature});
        },
        nb::arg("chains"), nb::arg("distance"), nb::arg("direction"),
        nb::arg("feature") = 0,
        "The exact extrude (ADR-0032 D6): the sketch profile chain\n"
        "becomes the prism BREP with true faces — the lateral faces\n"
        "from the entities, the caps' outer loops the chains\n"
        "themselves, the seam-wired circle walls per ADR-0023. Every\n"
        "face and the seam edge carry the WALK-ROLE NAME beside the\n"
        "address (ADR-0036 D1, morse 3): 'ext#F:lat(i)' for chain\n"
        "entity i's lateral, 'ext#F:cap(lo)' / 'ext#F:cap(hi)' for\n"
        "the region's two lids, 'ext#F:seam' for the seam-wired\n"
        "vertical — the multi-chain bodies prefix the chain group on\n"
        "the chain-owned roles ('ext#F:c0:lat(i)', 'ext#F:c1:seam'),\n"
        "the caps never (the region's own). The walk's other edges\n"
        "read honestly unnamed. Raises ValueError for every refusal\n"
        "the C++ kernel makes (the certificate's, the exact-class\n"
        "boundary's — diagnosis + remedy verbatim).");

    // THE EXACT SUBTRACT (H2-u6 morso 4, ADR-0032 D7 + ADR-0036 D1):
    // the naming context's three facts — the subtract's own DAG id
    // (the born names' creator part, 'sub#4') and the two operands'
    // creating kinds (their role tables, the one thing the bodies'
    // own stamps do not record). With the context the born edges
    // carry the relational names ('sub#4:isect[box#1:top ×
    // ext#3:lat(0)]@hi' the upper rim — the fillet's string); without
    // it (the default, the unnamed mode) they read honestly empty —
    // the wall's inheritance flows in both modes (a stamp copy, no
    // table involved). The context is complete or absent: one table
    // without the other refuses loudly.
    brep_mod.def(
        "exact_subtract",
        [](const BRepBody& a, const BRepBody& b,
           const CAD_0::math::Vec3f& offset, double tol,
           std::uint64_t feature,
           std::optional<CAD_0::brep::naming::FeatureKind> target,
           std::optional<CAD_0::brep::naming::FeatureKind> tool) {
            CAD_0::brep::SubtractNaming naming{};
            if (target.has_value() != tool.has_value())
                throw std::invalid_argument(
                    "exact_subtract: the naming context is complete or "
                    "absent - pass BOTH operand kinds (target=..., "
                    "tool=...) or neither (the unnamed mode); a half "
                    "context would name one operand's side and leave the "
                    "other silent.");
            if (target.has_value() && tool.has_value()) {
                naming = CAD_0::brep::SubtractNaming{
                    CAD_0::geometry::FeatureId{feature},
                    naming_table_of(*target),
                    naming_table_of(*tool)};
            }
            // THE BINDING BOUNDARY (H1, ADR-0034): Vec3f in, the
            // lossless promotion to the exact world's double here.
            return CAD_0::brep::exact_subtract(
                a, b,
                CAD_0::math::Vec3d{offset.x, offset.y, offset.z},
                tol, naming);
        },
        nb::arg("a"), nb::arg("b"), nb::arg("offset"),
        nb::arg("tol") = 1e-6, nb::arg("feature") = 0,
        nb::arg("target") = std::nullopt,
        nb::arg("tool") = std::nullopt,
        "The exact subtract (ADR-0032 D7): the tool B', translated by\n"
        "the world offset, removes its through-hole class from A.\n"
        "With the naming context (feature + target/tool kinds) the\n"
        "born edges carry the RELATIONAL names (ADR-0036 D1, morso\n"
        "4): 'sub#F:isect[<A-face-name> × <B-entity-name>]@lo|hi' the\n"
        "rim circles (ONE logical name each, both materialized halves\n"
        "answering to it - the seam-wired pair, ADR-0023) and\n"
        "'sub#F:seam[<B-role>]' the seam vertical; the operands are\n"
        "named BY THEIR NAMES, in the SSI table's declared order.\n"
        "Without the context (the default) the born edges read\n"
        "honestly empty; the wall's inheritance (the birth name kept)\n"
        "flows in both modes. Raises ValueError for every refusal the\n"
        "C++ kernel makes (the class checks' — diagnosis + remedy\n"
        "verbatim).");

    // THE FACTORY BOX (H2-u6 morso 4): the binding the guide case's\n"
    // chain needs — the same `feature` parameter the other factories\n"
    // expose (make_prism's own precedent), Vec3f promoted at the\n"
    // boundary.
    brep_mod.def(
        "make_box",
        [](const CAD_0::math::Vec3f& half, std::uint64_t feature) {
            return CAD_0::brep::make_box(
                CAD_0::math::Vec3d{half.x, half.y, half.z},
                CAD_0::geometry::FeatureId{feature});
        },
        nb::arg("half"), nb::arg("feature") = 0,
        "The box factory (ADR-0032 D1): the six planar faces, the 12\n"
        "edges, the 8 corners — every entity stamped with its\n"
        "construction name (ADR-0036 D1, morse 2): 'box#F:bottom',\n"
        "':top', ':front', ':back', ':left', ':right' and the edge\n"
        "ring names. The `feature` is the DAG node's FeatureId\n"
        "(defaulted to 0: the standalone caller reads box#0 - the id\n"
        "is the caller's to supply).");

    // THE PROBE'S WRITER (K1 m3, ADR-0042): the ellipsoid of\n"
    // revolution — the first family to enter under ADR-0041's\n"
    // protocol, queryable from Python (the make_box precedent — the\n"
    // census reads the writer through the binding).\n"
    brep_mod.def(
        "make_ellipsoid",
        [](double equatorial, double polar,
           const CAD_0::math::Vec3f& center, std::uint64_t feature) {
            return CAD_0::brep::make_ellipsoid(
                equatorial, polar,
                CAD_0::math::Vec3d{center.x, center.y, center.z},
                CAD_0::geometry::FeatureId{feature});
        },
        nb::arg("equatorial"), nb::arg("polar"),
        nb::arg("center") = CAD_0::math::Vec3f{0, 0, 0},
        nb::arg("feature") = 0,
        "The ellipsoid factory (K1 m3, ADR-0042 D1): the spheroid of\n"
        "revolution, semiaxes (equatorial a, polar c) on the +Z axis\n"
        "— the seam-wired single-face body (2V/1E/1F, chi=2, the\n"
        "sphere's own topology one family over), the slit-rectangle\n"
        "trim [0,2pi]x[0,pi], the names 'elp#F:ellipsoid' and\n"
        "'elp#F:seam' (the ADR-0036 D1 stamp). Raises ValueError for\n"
        "the semiaxis class (finite and > 0, the ADR-0029 family).");
    // THE POLYGON UNION (H2-u5 m2, ADR-0025 + ADR-0037 D3): the door
    // reachable from Python — the three cases of the wall map's one
    // refusing brick answer (the crossing pair, the diagonal, the
    // contained flush), the report reads the fused faces, the
    // maintainer's click-test runs on the live body. The tolerance
    // is the caller's LINEAR declaration (the exact_subtract
    // precedent); the angular kinds are the house constants
    // (kBoolLooseNormalPrefilter / kBoolStrictNormalParallel) — the
    // typed bundle is assembled at the boundary, one world per field.
    brep_mod.def(
        "bool_union",
        [](const BRepBody& a, const BRepBody& b, double tol) {
            return CAD_0::brep::bool_union(
                a, b,
                CAD_0::brep::BooleanTolerance::plane_family(
                    CAD_0::math::Tolerance::linear(tol),
                    CAD_0::brep::kBoolLooseNormalPrefilter,
                    CAD_0::brep::kBoolStrictNormalParallel));
        },
        nb::arg("a"), nb::arg("b"), nb::arg("tol") = 1e-6,
        "The polygon union (ADR-0025, ADR-0037's overlay since\n"
        "H2-u5): boundary(A u B) = the outside parts of both\n"
        "operands — and the ALIGNED coplanar pairs (same plane,\n"
        "same-facing normals, overlapping regions) are SEWN by the\n"
        "overlay: the merged region F_A u F_B projected in the one\n"
        "frame, walked through the loops' crossing points, emitted\n"
        "as convex fragments with every point through the canonical\n"
        "pool. The honest box answers where the wall used to refuse\n"
        "(P4: A=[-1,1]^3, B=[-0.5,1.5]x[-1,1]x[-1,1] -> 8V/12E/6F,\n"
        "chi=2, closed). Raises ValueError for every input-gate\n"
        "refusal (the class checks', diagnosis + remedy verbatim)\n"
        "and RuntimeError for the output gate's honest refusals\n"
        "(the assembled result that is none of the honest classes).");

    // THE FILLET, CLASS 1 — LO SPIGOLO D'ANGOLO DEL BOX (G2-u3 m2,
    // ADR-0039 D1.1): the rolling ball on the box's convex 90°
    // vertical edge, the cylindrical flank born in the edge's place
    // — the first feature that SUBSTITUTES instead of subtracting.
    // The naming context's two facts (the exact_subtract precedent):
    // the fillet's own DAG id (the born names' creator part, 'fil#2')
    // and the filleted body's creating kind (its role tables — the
    // flank's operands render through them). With the context the
    // flank carries 'fil#F:flank[box#1:back × box#1:right]' and the
    // tangent wires 'fil#F:isect[<the landing face> × <the flank>]'
    // (the level ABSENT — the tangency IS the level); without it
    // (the default, the unnamed mode) the born entities read
    // honestly empty — the shrink faces' inheritance flows in both
    // modes (a stamp copy, no table involved).
    brep_mod.def(
        "fillet_edge",
        [](const BRepBody& body,
           const CAD_0::geometry::ShapeId& edge, double radius,
           std::uint64_t feature,
           std::optional<CAD_0::brep::naming::FeatureKind> body_kind,
           double tol) {
            CAD_0::brep::FilletNaming naming{};
            if (body_kind.has_value()) {
                naming = CAD_0::brep::FilletNaming{
                    CAD_0::geometry::FeatureId{feature},
                    naming_table_of(*body_kind)};
            }
            return CAD_0::brep::fillet_edge(body, edge, radius, naming,
                                           tol);
        },
        nb::arg("body"), nb::arg("edge"), nb::arg("radius"),
        nb::arg("feature") = 0, nb::arg("body_kind") = std::nullopt,
        nb::arg("tol") = 1e-6,
        "The constant-radius fillet on a box's vertical edge (ADR-0039\n"
        "D1.1, class 1: plane x plane, the convex 90 corner - the\n"
        "guide case box#1:vertical2 at r=0.5 answers 10/15/7,\n"
        "chi=2, volume 7.5 + pi/8). The `edge` is the structural\n"
        "ShapeId (the by-name reference, never an address); the\n"
        "radius must sit strictly inside the faces' reach (the typed\n"
        "Linear band, tol the declared epsilon). With the naming\n"
        "context (feature + body_kind=Box) the born face and the two\n"
        "tangent wires carry the relational names; the shrink faces\n"
        "KEEP theirs (box#1:top ...) — the mark rides the tavola's\n"
        "absorbed rows. Raises ValueError for every refusal the C++\n"
        "kernel makes (the band's bound carried, the class families\n"
        "by name - diagnosis + remedy verbatim).");

    // THE FILLET, CLASS 2 — IL RIM CIRCOLARE DEL FORO (G2-u3 m3,
    // ADR-0039 D1.2): the rolling ball on the blind hole's convex
    // top rim, the toroidal flank born in the rim's place. The
    // naming context: the fillet's own DAG id (fil#5), the body's
    // creating kind for its box faces, and — the m3 addition — the
    // SECOND creator's pair (far_feature + far_body_kind) for the
    // hole wall (the blind body's two creators, the leg whose
    // feature matches far_feature rendering through the far table).
    brep_mod.def(
        "fillet_rim_edge",
        [](const BRepBody& body,
           const CAD_0::geometry::ShapeId& edge, double radius,
           std::uint64_t feature,
           std::optional<CAD_0::brep::naming::FeatureKind> body_kind,
           std::uint64_t far_feature,
           std::optional<CAD_0::brep::naming::FeatureKind> far_body_kind,
           double tol) {
            CAD_0::brep::FilletNaming naming{};
            if (body_kind.has_value() && far_body_kind.has_value()) {
                naming = CAD_0::brep::FilletNaming{
                    CAD_0::geometry::FeatureId{feature},
                    naming_table_of(*body_kind),
                    CAD_0::geometry::FeatureId{far_feature},
                    naming_table_of(*far_body_kind)};
            } else if (body_kind.has_value() != far_body_kind.has_value()) {
                throw std::invalid_argument(
                    "fillet_rim_edge: the naming context is complete or "
                    "absent - pass BOTH creators' kinds (body_kind=Box, "
                    "far_feature=<the wall's DAG id>, far_body_kind="
                    "Extrude) or neither (the unnamed mode); a half "
                    "context would name one operand's side and leave "
                    "the other silent.");
            }
            return CAD_0::brep::fillet_rim_edge(body, edge, radius,
                                               naming, tol);
        },
        nb::arg("body"), nb::arg("edge"), nb::arg("radius"),
        nb::arg("feature") = 0, nb::arg("body_kind") = std::nullopt,
        nb::arg("far_feature") = 0,
        nb::arg("far_body_kind") = std::nullopt,
        nb::arg("tol") = 1e-6,
        "The constant-radius fillet on a blind hole's convex top rim\n"
        "(ADR-0039 D1.2, class 2: plane x cylinder, the convex corner\n"
        "- the guide case, the u2 blind body with its pierced rim\n"
        "filleted at r=0.25, answers 14/20/9, H=1, chi=2, volume\n"
        "(8 - 0.375*pi) - 2*pi*[a*r^2*(1-pi/4) + r^3*(5/6-pi/4)]).\n"
        "The `edge` is the structural ShapeId of the RIM (the by-name\n"
        "reference, never an address; the rim is the seam-wired arc\n"
        "PAIR - one logical name, two materialized edges, taken whole).\n"
        "With the naming context (feature + body_kind + far_feature +\n"
        "far_body_kind) the born face, the two contact circles and the\n"
        "flank's own seam carry the relational names; the shrink faces\n"
        "KEEP theirs. Raises ValueError for every refusal the C++\n"
        "kernel makes (the bands' bounds carried, the concave family\n"
        "named, the seam refused - diagnosis + remedy verbatim).\n");

    // The feature kinds the naming grammar speaks (ADR-0036 D1) —\n"
    // the operand-table selector of exact_subtract's naming context.\n""
    nb::enum_<CAD_0::brep::naming::FeatureKind>(brep_mod, "FeatureKind")
        .value("Box", CAD_0::brep::naming::FeatureKind::Box,
               "the box factory's kind")
        .value("Sphere", CAD_0::brep::naming::FeatureKind::Sphere,
               "the seam-wired sphere factory's kind (ADR-0023)")
        .value("Cylinder", CAD_0::brep::naming::FeatureKind::Cylinder,
               "the seam-wired cylinder factory's kind (ADR-0023)")
        .value("Sketch", CAD_0::brep::naming::FeatureKind::Sketch,
               "the sketch document's kind (the authored roots)")
        .value("Extrude", CAD_0::brep::naming::FeatureKind::Extrude,
               "the extrude operation's kind")
        .value("Subtract", CAD_0::brep::naming::FeatureKind::Subtract,
               "the exact subtract operation's kind")
        .value("Fillet", CAD_0::brep::naming::FeatureKind::Fillet,
               "the fillet operation's kind (ADR-0039, G2-u3: the "
               "rolling ball - the first feature that substitutes "
               "instead of subtracting)");

    // The mass properties (G1 unit 5, ADR-0032 D4): the divergence
    // theorem over the exact faces — the numbers as plain values, the
    // final consumer being the unit-6 report (the contract is usable
    // from the tests from day one). The tolerance argument is the
    // operation's declared Layer-3 decision tolerance (mm, linear);
    // the Python-side default mirrors the C++ suites' constant.
    brep_mod.def(
        "mass_properties",
        [](const BRepBody& body, double tol) {
            const auto mp = CAD_0::brep::mass_properties(
                body, CAD_0::math::Tolerance::linear(tol));
            nb::dict out;
            out["volume"] = mp.volume;
            out["centroid"] = nb::make_tuple(
                double(mp.centroid.x), double(mp.centroid.y),
                double(mp.centroid.z));
            nb::list rows;
            for (int r = 0; r < 3; ++r) {
                nb::list row;
                for (int c = 0; c < 3; ++c)
                    row.append(double(mp.inertia_about_centroid(r, c)));
                rows.append(row);
            }
            out["inertia_about_centroid"] = rows;
            out["all_exact"] = mp.all_exact;
            return out;
        },
        nb::arg("body"), nb::arg("tol") = 1e-6,
        "The body's mass properties by the divergence theorem over\n"
        "its faces (density 1): {volume: float, centroid: (x, y, z),\n"
        "inertia_about_centroid: 3x3 nested lists, all_exact: bool}.\n"
        "Exact faces contribute their closed forms (plane regions by\n"
        "Green's theorem, cylindrical patches by the (theta, z)\n"
        "rectangle integrals, the full sphere by its closed form); a\n"
        "body with ANY approximated face answers the POLYHEDRAL value\n"
        "with all_exact=False (never silently mixed). Raises ValueError\n"
        "for out-of-class faces, the empty body, a non-closed shell or\n"
        "a degenerate volume (diagnosis + remedy verbatim).");

    // The body query (G1 unit 6, ADR-0032 D8 at the body level): the
    // whole body as plain values — the report's single source beside
    // mass_properties. Every number is a pinned authority's own answer
    // (euler_characteristic, inner_loop_count, genus_from_chi, the
    // provenance fields) — no second opinion exists to drift from.
    brep_mod.def(
        "body_query",
        [](const BRepBody& body) {
            return body_summary_dict(CAD_0::brep::query_body(body));
        },
        nb::arg("body"),
        "The whole body as plain values: {V, E, F, H (inner loops),\n"
        "chi = V - E + F - H, orientable, loops_closed, genus (int or\n"
        "None — derived only under the ADR-0027 D6 rule: orientable\n"
        "body, chi <= 2 and even; a multi-shell body reports None,\n"
        "\"n/d\"), vertices/edges/faces: {exact, approximated,\n"
        "max_linear_tolerance} — the Layer-2 provenance tallies.\n"
        "Raises ValueError for the empty body (diagnosis + remedy\n"
        "verbatim — no shell to summarize).");

    // THE TABLE (G2 unit 1, ADR-0033 D5.1): the intersections
    // answer as plain values, queryable before any boolean consumes
    // them — the face-query precedent, one rung sideways. The
    // tolerance argument is the operation's declared Layer-3
    // decision tolerance (the exact_subtract discipline: the linear
    // bands and the cosine band read the same value).
    brep_mod.def(
        "ssi_query",
        [](const BRepBody& body_a, FaceId face_a, const BRepBody& body_b,
           FaceId face_b, double tol) {
            return ssi_dict(CAD_0::brep::ssi_query(
                body_a, face_a, body_b, face_b, tol));
        },
        nb::arg("body_a"), nb::arg("face_a"), nb::arg("body_b"),
        nb::arg("face_b"), nb::arg("tol") = 1e-6,
        "Consult the analytic SSI table with two faces of two bodies:\n"
        "the typed intersection as plain values — {kind, exact,\n"
        "linear_tolerance, families, point (tangency), curves (the 3D\n"
        "circles with their derived frames / the lines with point +\n"
        "direction), operands (each: reframed + the declared frame +\n"
        "the typed p-curves in that domain, one per curve in order),\n"
        "message (the disposition class; for out_of_table, the FULL\n"
        "actionable refusal: the pair, the class, the remedies)}. The\n"
        "entries: plane×sphere (the circle sqrt(R^2-d^2) at the foot),\n"
        "sphere×sphere (the radical plane), cylinder×sphere coaxial\n"
        "(the two circles at ±sqrt(R^2-r^2)), cylinder×cylinder\n"
        "parallel equal-radii (the mid-plane line pair) — with their\n"
        "degenerates (tangencies, the empty classes, the co-radial\n"
        "coincident). Everything else is the WALL: a returned record,\n"
        "never an exception — the capability ceiling is queryable.\n"
        "Raises ValueError only for a face id a body does not carry\n"
        "(diagnosis + remedy verbatim).");

    // The exact side's own placement (the subtract's tool-offset path,
    // G1 unit 4) — exposed so the author bodies can be POSITIONED for
    // the table's queries (the bridge's primitive fit serves
    // origin-centered bodies only). In place: the surfaces re-anchor
    // through their own accessors, the p-curves stay valid verbatim.
    brep_mod.def(
        "translate_body",
        [](BRepBody& body, const CAD_0::math::Vec3f& delta) {
            // THE BINDING BOUNDARY (H1, ADR-0034): the Python surface
            // keeps its Vec3f vocabulary; the promotion to the exact
            // world's double is lossless and happens here, one
            // direction, declared.
            CAD_0::brep::translate_body(
                body, CAD_0::math::Vec3d{delta.x, delta.y, delta.z});
        },
        nb::arg("body"), nb::arg("delta"),
        "Translate a B-Rep body by a world delta, in place: every\n"
        "vertex moves, every face's surface re-anchors (the plane's\n"
        "origin, the cylinders' placements, the sphere's center — the\n"
        "sphere keeping its DERIVED frame, the re-frame rule's own).\n"
        "The loops, the co-edges, the p-curves and the provenance are\n"
        "untouched: the p-curve coordinates are frame-relative and the\n"
        "frame moves with its surface. Raises ValueError for a surface\n"
        "family outside the analytic translation table (loud, never a\n"
        "silent skip).");

    // ------------------------------------------------------------------
    // THE RESOLVER (H2-u6 morso 5, ADR-0036 D3 + D6.4): the four
    // classes through mutation, the two query surfaces. The
    // descent-table derivations, the generation + edit-context
    // constructors, resolve() itself, and the by-name queries —
    // everything the click-test needs, from Python, on the guide
    // case: pick the rim, edit the radius, resolve, re-run.
    // ------------------------------------------------------------------

    // The tavola (the descent table) — an opaque handle with the
    // row count (the derivation functions emit it, pure and
    // deterministic; the resolution reads the rows).
    nb::class_<CAD_0::brep::DescentTable>(brep_mod, "DescentTable")
        .def("__len__",
             [](const CAD_0::brep::DescentTable& t) {
                 return t.rows.size();
             })
        .def("__repr__", [](const CAD_0::brep::DescentTable& t) {
            return "<DescentTable with " +
                   std::to_string(t.rows.size()) + " rows>";
        });

    brep_mod.def(
        "subtract_descent",
        [](const BRepBody& result, const BRepBody& a,
           const BRepBody& b) {
            return CAD_0::brep::subtract_descent(result, a, b);
        },
        nb::arg("result"), nb::arg("target"), nb::arg("tool"),
        "The exact subtract's descent table (ADR-0036 D2, morse 4):\n"
        "one row per entity of the result (born/kept/absorbed, with\n"
        "the operand refs and the names) plus the VANISHED tail (the\n"
        "dying operand's entities, each with its cause and — since\n"
        "morse 5 — its dead identity's ShapeId). Derived, never\n"
        "stored: recomputed deterministically from the bodies on\n"
        "every call.");

    // The fillet's tavola (K1 m6, the survival ledger's own need at
    // the report's door: the MEAL step's old side — the vanished
    // row's lodging, the death's cause verbatim).
    brep_mod.def(
        "fillet_descent",
        [](const BRepBody& result, const BRepBody& operand) {
            return CAD_0::brep::fillet_descent(result, operand);
        },
        nb::arg("result"), nb::arg("operand"),
        "The fillet's descent table (ADR-0039 D3): the kept, the\n"
        "absorbed (the re-spanned edges, the shrunk faces), the born\n"
        "(the flank, the tangent wires) and the VANISHED tail (the\n"
        "consumed edge and its corners, the cause verbatim -\n"
        "'replaced by the flank'). Derived, never stored: recomputed\n"
        "deterministically from the bodies on every call.");

    brep_mod.def(
        "extrude_descent",
        [](const BRepBody& body,
           const std::vector<CAD_0::sdf::SketchChain>& chains,
           std::uint64_t feature, std::uint64_t sketch) {
            return CAD_0::brep::extrude_descent(
                body, chains, CAD_0::geometry::FeatureId{feature},
                CAD_0::geometry::FeatureId{sketch});
        },
        nb::arg("body"), nb::arg("chains"), nb::arg("feature"),
        nb::arg("sketch"),
        "The extrude's descent table (ADR-0036 D2, morse 3): one\n"
        "born row per entity, each carrying its FROM — the authored\n"
        "root (the sketch chain entity). `feature` is the extrude's\n"
        "own FeatureId (the names' creator part), `sketch` the\n"
        "sketch's (the FROM references' own feature).");

    brep_mod.def(
        "factory_descent",
        [](const BRepBody& body) {
            return CAD_0::brep::factory_descent(body);
        },
        nb::arg("body"),
        "The factories' descent table (ADR-0036 D2, morse 2): the\n"
        "trivial all-born tavola, derived on every call.");

    // The generation — one materialized generation of the referenced
    // feature's output, made whole with the reference's grammar. The
    // operand bindings (the Subtract kind's legs) are built from the
    // (feature, kind) pairs the caller holds — it built the bodies.
    struct PyGeneration {
        // The storage and the instance declaration, owned HERE (the
        // resolver's view points into them; the object is built once
        // and never mutated, so the span stays valid).
        std::vector<CAD_0::brep::naming::OperandBinding> storage{};
        CAD_0::brep::naming::OperationNaming naming{};
        CAD_0::brep::Generation gen{};

        PyGeneration(const BRepBody* body,
                     const CAD_0::brep::DescentTable* tavola,
                     CAD_0::brep::naming::FeatureKind kind,
                     std::vector<std::pair<std::uint64_t,
                                          CAD_0::brep::naming::FeatureKind>>
                         operands,
                     bool multi_chain) {
            namespace n = CAD_0::brep::naming;
            for (const auto& [feature, op_kind] : operands) {
                storage.push_back(
                    {CAD_0::geometry::FeatureId{feature},
                     naming_table_of(op_kind)});
            }
            switch (kind) {
            case n::FeatureKind::Box:
                if (!storage.empty())
                    throw std::invalid_argument(
                        "Generation: the box's naming declares no "
                        "operands (the factories bootstrap from the "
                        "construction index alone); remedy: pass "
                        "operands=[] for the factory kinds.");
                naming = n::OperationNaming{
                    kind, n::kBoxFaceRoles, n::kBoxEdgeRoles, {}};
                break;
            case n::FeatureKind::Sphere:
                if (!storage.empty())
                    throw std::invalid_argument(
                        "Generation: the sphere's naming declares no "
                        "operands; remedy: pass operands=[].");
                naming = n::OperationNaming{
                    kind, n::kSphereFaceRoles, n::kSphereEdgeRoles, {}};
                break;
            case n::FeatureKind::Cylinder:
                if (!storage.empty())
                    throw std::invalid_argument(
                        "Generation: the cylinder's naming declares no "
                        "operands; remedy: pass operands=[].");
                naming = n::OperationNaming{
                    kind, n::kCylinderFaceRoles, n::kCylinderEdgeRoles,
                    {}};
                break;
            case n::FeatureKind::Ellipsoid:
                // The probe's family (K1 m3, ADR-0042): the factory
                // bootstrap, the sphere's own form — the construction
                // index alone, no operand bindings.
                if (!storage.empty())
                    throw std::invalid_argument(
                        "Generation: the ellipsoid's naming declares no "
                        "operands (the factories bootstrap from the "
                        "construction index alone); remedy: pass "
                        "operands=[].");
                naming = n::OperationNaming{
                    kind, n::kEllipsoidFaceRoles, n::kEllipsoidEdgeRoles,
                    {}};
                break;
            case n::FeatureKind::Extrude:
                // The extrude's roots ride the FROM channel (D4 + the
                // morse-3 register): the tavola's own rows name them,
                // never an operand binding.
                if (!storage.empty())
                    throw std::invalid_argument(
                        "Generation: the extrude's naming declares no "
                        "operand bindings - the authored roots ride "
                        "the tavola's FROM channel (ADR-0036 D4 + the "
                        "morse-3 register); remedy: pass operands=[].");
                naming = n::OperationNaming{
                    kind, n::kExtFaceRoles, n::kExtEdgeRoles, {}};
                break;
            case n::FeatureKind::Subtract:
                if (storage.size() != 2)
                    throw std::invalid_argument(
                        "Generation: the subtract's naming binds "
                        "EXACTLY TWO operands (the target and the "
                        "tool - the isect/seam slots' own declaration, "
                        "the SSI table's order); remedy: pass "
                        "operands=[(target_feature, target_kind), "
                        "(tool_feature, tool_kind)].");
                naming = n::OperationNaming{
                    kind, {}, n::kSubEdgeRoles,
                    std::span<const n::OperandBinding>{storage}};
                break;
            case n::FeatureKind::Sketch:
                throw std::invalid_argument(
                    "Generation: the sketch has no naming table - the "
                    "authored roots are named by the ROOT REFERENCE "
                    "(the tavola's FROM), never a role table; remedy: "
                    "pass the CREATING operation's kind (the one that "
                    "built the referenced body).");
            }
            gen = CAD_0::brep::Generation{body, tavola, &naming,
                                          multi_chain};
        }
    };

    nb::class_<PyGeneration>(brep_mod, "Generation",
        "One materialized generation of the referenced feature's "
        "output, made whole with the reference's grammar: the body\n"
        "(required on the NEW side), the tavola (required on the OLD\n"
        "side), and the naming declaration the reference's paths were\n"
        "rendered under — the creating operation's own tables for its\n"
        "born entities, the ORIGINAL creator's for an inherited one\n"
        "(the absorbed wall inside a subtract's result is the "
        "EXTRUDE's face).")
        .def(nb::init<const BRepBody*,
                      const CAD_0::brep::DescentTable*,
                      CAD_0::brep::naming::FeatureKind,
                      std::vector<std::pair<std::uint64_t,
                          CAD_0::brep::naming::FeatureKind>>,
                      bool>(),
             nb::arg("body").none(true),
             nb::arg("tavola").none(true), nb::arg("kind"),
             nb::arg("operands") = std::vector<std::pair<
                 std::uint64_t, CAD_0::brep::naming::FeatureKind>>{},
             nb::arg("multi_chain") = false,
             // The generation holds RAW pointers to the body and the
             // tavola (the resolver's own view) — the Python objects
             // must outlive it: the keep-alive ties their lifetimes
             // to the generation's own.
             nb::keep_alive<1, 2>(), nb::keep_alive<1, 3>(),
             "Build one generation: body=None allowed on the OLD\n"
             "side (unread there - the tavola's rows carry the\n"
             "identities), tavola=None allowed on the NEW side (the\n"
             "re-derivation reads the body's own stamps). For the\n"
             "Subtract kind pass the two operands as\n"
             "[(target_feature, target_kind), (tool_feature,\n"
             "tool_kind)] — the isect/seam slots' own declaration; "
             "the\n"
             "factory and extrude kinds declare no operands (the\n"
             "extrude's roots ride the tavola's FROM channel).")
        .def_prop_ro("body", [](const PyGeneration& g) {
            return g.gen.body;
        }, nb::rv_policy::reference_internal)
        .def_prop_ro("tavola", [](const PyGeneration& g) {
            return g.gen.tavola;
        }, nb::rv_policy::reference_internal);

    // The edit context — what the edit path EMITS (ADR-0036 D4's
    // amendment): the no-shift declaration for the v1-covered flows,
    // the root map when the edit shifts roots and declares them. The
    // map's entries are (old_chain, old_entity, new_chain,
    // new_entity) tuples — the sketch's own layout, the feature part
    // uniform (one sketch feeds one extrude in v1's shapes).
    struct PyEditContext {
        CAD_0::brep::EditContext ctx{};
        std::uint64_t sketch{0};

        PyEditContext(bool no_shift,
                      const std::vector<std::tuple<std::uint32_t,
                                                   std::uint32_t,
                                                   std::uint32_t,
                                                   std::uint32_t>>&
                          root_map,
                      std::uint64_t sketch_id)
            : sketch(sketch_id) {
            ctx.roots_no_shift = no_shift;
            for (const auto& [oc, oe, nc, ne] : root_map) {
                ctx.root_map.push_back(
                    {CAD_0::brep::naming::RootRef{
                         CAD_0::geometry::FeatureId{sketch_id},
                         CAD_0::brep::naming::RootKind::Entity, oc, oe},
                     CAD_0::brep::naming::RootRef{
                         CAD_0::geometry::FeatureId{sketch_id},
                         CAD_0::brep::naming::RootKind::Entity, nc, ne}});
            }
        }
    };

    nb::class_<PyEditContext>(brep_mod, "EditContext",
        "The edit context: what the edit path EMITS (ADR-0036 D4's\n"
        "amendment - the edit that shifts referenced roots EMITS the\n"
        "root map or REFUSES with the remedy; the resolver CONSUMES\n"
        "the declaration, never invents it).")
        .def(nb::init<bool,
                      std::vector<std::tuple<std::uint32_t, std::uint32_t,
                                             std::uint32_t, std::uint32_t>>,
                      std::uint64_t>(),
             nb::arg("no_shift") = false,
             nb::arg("root_map") = std::vector<std::tuple<
                 std::uint32_t, std::uint32_t, std::uint32_t,
                 std::uint32_t>>{},
             nb::arg("sketch") = 0,
             "no_shift=True declares the v1-covered flows (add-at-end,\n"
             "the parameter edits: the pre-existing roots kept their\n"
             "positions); root_map=[(old_chain, old_entity,\n"
             "new_chain, new_entity), ...] declares the shift (the\n"
             "registered future - v1's authored flows emit none);\n"
             "sketch is the sketch node's FeatureId (the roots' own\n"
             "feature, the map's render).");

    // resolve — THE MACHINE, as plain values: the kind, the code (the
    // four-word family registered for the fillet + the guard's own
    // token), the report text, and the class's own payload.
    brep_mod.def(
        "resolve",
        [](const CAD_0::geometry::ShapeId& reference,
           const PyGeneration& old_generation,
           const PyGeneration& new_generation,
           const PyEditContext& edit_ctx) {
            const CAD_0::brep::Resolution r = CAD_0::brep::resolve(
                reference, old_generation.gen, new_generation.gen,
                edit_ctx.ctx);
            nb::dict out;
            out["text"] = CAD_0::brep::resolution_text(r);
            out["code"] = CAD_0::brep::resolution_code(r.kind);
            const auto shape_id_dict =
                [](const CAD_0::geometry::ShapeId& id) {
                    nb::dict d;
                    d["feature"] = id.feature.value;
                    nb::list path;
                    for (std::uint8_t i = 0; i < id.subshape.depth; ++i)
                        path.append(id.subshape.path[i]);
                    d["path"] = path;
                    d["generation"] = id.generation.value;
                    return d;
                };
            using K = CAD_0::brep::Resolution::Kind;
            switch (r.kind) {
            case K::Resolved:
                out["kind"] = "resolved";
                out["id"] = shape_id_dict(r.resolved.id);
                out["name"] = r.resolved.name;
                break;
            case K::Renamed:
                out["kind"] = "renamed";
                out["from"] = shape_id_dict(r.renamed.from);
                out["to"] = shape_id_dict(r.renamed.to);
                out["from_name"] = r.renamed.from_name;
                out["to_name"] = r.renamed.to_name;
                out["map"] = r.renamed.map;
                break;
            case K::Ambiguous: {
                out["kind"] = "ambiguous";
                nb::list ids, names;
                for (std::size_t i = 0;
                     i < r.ambiguous.candidates.size(); ++i) {
                    ids.append(shape_id_dict(r.ambiguous.candidates[i]));
                    names.append(r.ambiguous.candidate_names[i]);
                }
                out["candidates"] = ids;
                out["candidate_names"] = names;
                out["entity_name"] = r.ambiguous.entity_name;
                out["remedy"] = r.ambiguous.remedy;
                break;
            }
            case K::Vanished: {
                out["kind"] = "vanished";
                nb::dict row;
                row["relation"] = CAD_0::brep::descent_token(
                    r.vanished.row.relation);
                row["entity"] = r.vanished.row.entity ==
                                CAD_0::brep::EntityKind::Vertex
                                    ? "vertex"
                                : r.vanished.row.entity ==
                                          CAD_0::brep::EntityKind::Edge
                                      ? "edge"
                                      : "face";
                row["address"] = r.vanished.row.address;
                row["name"] = r.vanished.row.name;
                row["cause"] = r.vanished.row.cause;
                out["row"] = row;
                out["why"] = r.vanished.why;
                out["remedy"] = r.vanished.remedy;
                break;
            }
            case K::ClosureDefect:
                out["kind"] = "closure_defect";
                out["orphan"] = shape_id_dict(
                    r.closure_defect.orphan);
                out["detail"] = r.closure_defect.detail;
                break;
            }
            return out;
        },
        nb::arg("reference"), nb::arg("old_generation"),
        nb::arg("new_generation"), nb::arg("edit_ctx"),
        "Resolve a reference BY NAME (the structural ShapeId - the\n"
        "pick's own answer) across an edit: {kind, code, text, ...}\n"
        "with the class's own payload - resolved: {id, name} (the\n"
        "feature re-runs, silently); renamed: {from, to, from_name,\n"
        "to_name, map} (the map declared, the log says so); ambiguous:\n"
        "{candidates, candidate_names, entity_name, remedy} (every\n"
        "candidate nominated, never a guess); vanished: {row, why,\n"
        "remedy} (what died, why, the remedy - never a mute 'not\n"
        "found'); closure_defect: {orphan, detail} (the guard: no row\n"
        "at all is a closure violation, a defect surfaced as such).\n"
        "Raises ValueError for the input contract's own refusals (a\n"
        "missing tavola/body/grammar, a foreign grammar - diagnosis +\n"
        "remedy verbatim).");

    // The query BY NAME — the name plane's own door (D6.4): the
    // structural name -> the materializing addresses (the seam-wired
    // pair answers BOTH halves). The loud miss refuses.
    brep_mod.def(
        "query_edge_by_name",
        [](const BRepBody& body, const CAD_0::geometry::ShapeId& name) {
            return CAD_0::brep::query_edge_by_name(body, name);
        },
        nb::arg("body"), nb::arg("name"),
        "The edges carrying this structural name, in address order:\n"
        "the seam-wired pair (ADR-0023) answers BOTH halves - one\n"
        "logical entity, two materialized arcs. Raises ValueError\n"
        "when no edge carries the name (diagnosis + remedy verbatim).");

    brep_mod.def(
        "query_face_by_name",
        [](const BRepBody& body, const CAD_0::geometry::ShapeId& name) {
            return CAD_0::brep::query_face_by_name(body, name);
        },
        nb::arg("body"), nb::arg("name"),
        "The faces carrying this structural name, in address order.\n"
        "Raises ValueError when no face carries the name (diagnosis +\n"
        "remedy verbatim).");

    // THE CHAIN GATE (K1 m5, ADR-0041 D1.3): the report's own door
    // to the topological correctness after every step — THE SAME
    // DOOR the C++ suite consumes (CAD_0/brep/chain_gate.hpp, one
    // module, never a second opinion), here with the report's own
    // evidence: the body alone. The radial, the loops and the chi
    // run on it; the census and the drift are HONESTLY not
    // applicable at this door (they read the step's tavola and the
    // re-run's digest — the caller's evidence, carried by the C++
    // ChainStep) and say so in not_applicable, never silently
    // skipped.
    brep_mod.def(
        "chain_gate",
        [](const BRepBody& body) {
            const CAD_0::brep::ChainStep step{&body};
            const auto v = CAD_0::brep::chain_gate(step);
            nb::dict d;
            d["pass"] = v.pass;
            nb::list failures;
            for (const auto& f : v.failures) {
                nb::dict fd;
                fd["check"] =
                    std::string(
                        CAD_0::brep::gate_check_token(f.check));
                fd["message"] = f.message;
                failures.append(fd);
            }
            d["failures"] = failures;
            nb::list na;
            for (const auto c : v.not_applicable)
                na.append(std::string(
                    CAD_0::brep::gate_check_token(c)));
            d["not_applicable"] = na;
            d["v"] = v.v;
            d["e"] = v.e;
            d["f"] = v.f;
            d["h"] = v.h;
            d["chi"] = v.chi;
            d["digest"] = v.digest;
            return d;
        },
        nb::arg("body"),
        "The chain gate on a body (ADR-0041 D1.3, K1 m5): the\n"
        "topological correctness as VALUES — pass, the named\n"
        "failures (check + message, every corruption quoted with\n"
        "its entity and its numbers), the not_applicable list (the\n"
        "census and the drift read the step's tavola and the\n"
        "re-run's digest, this door's body-alone evidence does not\n"
        "reach them — said, never hidden), and the body's own\n"
        "numbers: v/e/f, h (the inner-loop term), chi, and the\n"
        "digest (chain_digest's own answer, the drift's left\n"
        "side). The report and the tests share this one door.");

    brep_mod.def(
        "chain_digest",
        [](const BRepBody& body) {
            return CAD_0::brep::chain_digest(body);
        },
        nb::arg("body"),
        "The body's whole answer as one comparable multi-line\n"
        "string (the H2 unit 6 pattern grown the chain's spine):\n"
        "the summary line (V E F H chi), one line per vertex (the\n"
        "position), per edge (the endpoints, the name) and per\n"
        "face (the family, the loops, every co-edge's typed\n"
        "p-curve, the name) — the entities in the body's own\n"
        "order, every DOUBLE written with 12 significant digits\n"
        "(ADR-0014's serialization rule for double-originated\n"
        "numbers). The same recipe re-run reproduces it\n"
        "bit-identically — that is the drift predicate's own\n"
        "instrument.");

    // THE SURVIVAL LEDGER (K1 m6, ADR-0041 D1.4): the report's own
    // door to the feature stability after parameter edits — THE SAME
    // DOOR the C++ suite consumes (CAD_0/brep/survival_ledger.hpp,
    // one module, never a second opinion). The caller (the report,
    // the test) holds the corpus's evidence — the chains, the
    // tavole, the edit contexts — and hands it to the door as a list
    // of member dicts; the door walks the references through the
    // steps (the resolver's own answers, one opinion), enforces the
    // metric's discipline (a death without its name is REFUSED —
    // ValueError, the diagnosis verbatim) and returns the fraction
    // AND the deaths' register in one value.
    brep_mod.def(
        "survival_ledger",
        [](const nb::list& py_members) {
            namespace sl = CAD_0::brep;
            using SK = sl::SurvivalStepKind;

            // The grammars' storage — one PyGeneration per reference,
            // alive through the ledger call (the naming's operand
            // spans point into them; the DEQUE keeps every element's
            // address stable across the pushes — a vector's
            // reallocation would dangle the stored naming pointers).
            std::deque<PyGeneration> grammars;
            std::vector<sl::SurvivalMember> members;

            for (const auto& py_m : py_members) {
                const nb::dict d = nb::cast<nb::dict>(py_m);
                sl::SurvivalMember m;
                m.name = nb::cast<std::string>(d["name"]);
                const std::string kind_tok =
                    nb::cast<std::string>(d["kind"]);
                if (kind_tok == "silent_radius_edit")
                    m.kind = sl::SurvivalEditKind::SilentRadiusEdit;
                else if (kind_tok == "sketch_root_edit")
                    m.kind = sl::SurvivalEditKind::SketchRootEdit;
                else if (kind_tok == "load_boolean_rerun")
                    m.kind = sl::SurvivalEditKind::LoadBooleanRerun;
                else if (kind_tok == "chained_fillet_edit")
                    m.kind = sl::SurvivalEditKind::ChainedFilletEdit;
                else if (kind_tok == "recipe_rerun")
                    m.kind = sl::SurvivalEditKind::RecipeRerun;
                else
                    throw std::invalid_argument(
                        "survival_ledger: unknown member kind '" +
                        kind_tok +
                        "' - the corpus is TYPED (the five the "
                        "mandate named). Remedy: one of "
                        "silent_radius_edit, sketch_root_edit, "
                        "load_boolean_rerun, chained_fillet_edit, "
                        "recipe_rerun.");
                if (d.contains("digest"))
                    m.digest =
                        nb::cast<std::string>(d["digest"]);
                if (d.contains("seats"))
                    for (const auto& s : nb::cast<nb::list>(
                             d["seats"]))
                        m.seats_declared.push_back(
                            nb::cast<std::string>(s));

                // The references: the render, the structural id, the
                // grammar (the creating declaration — built from the
                // (kind, operands) pair the caller holds).
                for (const auto& py_r : nb::cast<nb::list>(
                         d["references"])) {
                    const nb::dict r =
                        nb::cast<nb::dict>(py_r);
                    std::vector<std::pair<
                        std::uint64_t,
                        CAD_0::brep::naming::FeatureKind>>
                        ops{};
                    if (r.contains("operands") &&
                        !nb::cast<nb::object>(r["operands"])
                             .is_none())
                        ops = nb::cast<std::vector<
                            std::pair<
                                std::uint64_t,
                                CAD_0::brep::naming::
                                    FeatureKind>>>(r["operands"]);
                    const bool multi =
                        r.contains("multi_chain") &&
                        nb::cast<bool>(r["multi_chain"]);
                    grammars.emplace_back(
                        nullptr, nullptr,
                        nb::cast<CAD_0::brep::naming::
                                     FeatureKind>(r["kind"]),
                        ops, multi);
                    m.references.push_back(sl::SurvivalReference{
                        nb::cast<std::string>(r["name"]),
                        nb::cast<CAD_0::geometry::ShapeId>(
                            r["id"]),
                        grammars.back().gen.naming, multi});
                }

                // The steps: the walk's own distinction (edit/meal)
                // and the per-reference evidence.
                for (const auto& py_s : nb::cast<nb::list>(
                         d["steps"])) {
                    const nb::dict st = nb::cast<nb::dict>(py_s);
                    sl::SurvivalStep step;
                    const std::string sk =
                        nb::cast<std::string>(st["kind"]);
                    if (sk == "edit")
                        step.kind = SK::Edit;
                    else if (sk == "meal")
                        step.kind = SK::Meal;
                    else
                        throw std::invalid_argument(
                            "survival_ledger: unknown step kind '" +
                            sk + "' (member '" + m.name +
                            "') - a step is the EDIT's own "
                            "(the class's step) or the MEAL's (the "
                            "consuming feature's act, the register's "
                            "step). Remedy: 'edit' or 'meal'.");
                    step.name =
                        nb::cast<std::string>(st["name"]);
                    if (st.contains("killer"))
                        step.killer = nb::cast<std::string>(
                            st["killer"]);
                    for (const auto& py_sr : nb::cast<nb::list>(
                             st["refs"])) {
                        const nb::dict sr =
                            nb::cast<nb::dict>(py_sr);
                        sl::SurvivalStepRef out;
                        out.reference = nb::cast<std::size_t>(
                            sr["reference"]);
                        out.old_tavola = nb::cast<const CAD_0::
                                                      brep::
                                                          DescentTable*>(
                            sr["old_tavola"]);
                        out.new_body = nb::cast<const BRepBody*>(
                            sr["new_body"]);
                        if (sr.contains("edit")) {
                            const PyEditContext& ec =
                                nb::cast<const PyEditContext&>(
                                    sr["edit"]);
                            out.edit = ec.ctx;
                        }
                        step.refs.push_back(std::move(out));
                    }
                    m.steps.push_back(std::move(step));
                }
                members.push_back(std::move(m));
            }

            const sl::SurvivalLedger ledger =
                sl::survival_ledger(members);

            // The verdict as plain values: the members' rows, the
            // counts, the FRACTION (a pair — never a naked number
            // without the register) and the register itself, line by
            // line — the report's own evidence, both or neither.
            nb::dict out;
            nb::list py_ms;
            for (const auto& m : ledger.members) {
                nb::dict md;
                md["name"] = m.name;
                md["kind"] = std::string(
                    sl::survival_edit_token(m.kind));
                md["digest"] = m.digest;
                nb::list seats;
                for (const auto& s : m.seats_declared)
                    seats.append(s);
                md["seats"] = seats;
                nb::list entries;
                for (const auto& e : m.entries) {
                    nb::dict ed;
                    ed["reference"] = e.reference;
                    ed["code"] = e.code;
                    ed["dead"] = e.dead;
                    ed["died"] = e.died;
                    ed["killer"] = e.killer;
                    ed["cause"] = e.cause;
                    ed["step"] = e.step;
                    ed["remedy"] = e.remedy;
                    ed["renamed_from"] = e.renamed_from;
                    ed["renamed_to"] = e.renamed_to;
                    ed["renamed_map"] = e.renamed_map;
                    nb::list cands;
                    for (const auto& c : e.candidates)
                        cands.append(c);
                    ed["candidates"] = cands;
                    entries.append(ed);
                }
                md["entries"] = entries;
                md["resolved"] = m.resolved;
                md["renamed"] = m.renamed;
                md["ambiguous"] = m.ambiguous;
                md["vanished"] = m.vanished;
                py_ms.append(md);
            }
            out["members"] = py_ms;
            out["total"] = ledger.total;
            out["resolved"] = ledger.resolved;
            out["renamed"] = ledger.renamed;
            out["ambiguous"] = ledger.ambiguous;
            out["vanished"] = ledger.vanished;
            out["fraction"] = std::make_pair(ledger.resolved,
                                             ledger.total);
            nb::list deaths;
            for (const auto& d : ledger.deaths) {
                nb::dict dd;
                dd["member"] = d.member;
                dd["reference"] = d.reference;
                dd["code"] = d.code;
                dd["died"] = d.died;
                dd["killer"] = d.killer;
                dd["cause"] = d.cause;
                dd["step"] = d.step;
                dd["remedy"] = d.remedy;
                deaths.append(dd);
            }
            out["deaths"] = deaths;
            out["deaths_text"] = ledger.deaths_text();
            out["text"] = ledger.text();
            return out;
        },
        nb::arg("members"),
        "The survival ledger (ADR-0041 D1.4, K1 m6): the feature\n"
        "stability after parameter edits, the deaths NAMED — THE\n"
        "SAME door the C++ suite consumes. Takes the corpus's\n"
        "evidence as a list of member dicts (name, kind, references,\n"
        "steps, seats, digest), walks every reference through every\n"
        "step (the resolver's own answers, one opinion), and returns\n"
        "the fraction AND the deaths' register in one value:\n"
        "{members: [...], total, resolved, renamed, ambiguous,\n"
        "vanished, fraction: (resolved, total), deaths: [...],\n"
        "deaths_text, text}. The fraction's NUMERATOR is class 1\n"
        "ALONE (ADR-0041 D1.4, verbatim: 'references that resolve\n"
        "class 1 (reference_resolved, the feature re-running\n"
        "silently, ONE undo entry) over the total') -\n"
        "reference_renamed (UNIQUE CONTINUER, a survival of the\n"
        "NAME, not a resolve) rides the denominator and the\n"
        "register's counts, never the numerator. A death without\n"
        "its name (the killer\n"
        "not declared, the cause wiped) is REFUSED - never a\n"
        "partial ledger, never an anonymous death riding a fraction\n"
        "(raises ValueError, diagnosis + remedy verbatim).");
}
