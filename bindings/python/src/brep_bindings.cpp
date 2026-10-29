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
//    "loops": [ {"is_outer": bool,
//                "coedges": [ {"edge": int, "reversed": bool,
//                               "pcurve": None | line/circle/arc dict } ]} ]}
//
// with the p-curve payloads mirroring the sketch kernel's typed
// entities verbatim: line {a, b, interior_left}, circle {center,
// radius, hole}, arc {center, radius, hole, start_angle, sweep} —
// the same numbers the sketcher would read, untranslated (D2.1).
//
// Errors: a face id that does not exist crosses as a ValueError
// carrying the C++ message's diagnosis + remedy verbatim (the
// nanobind default translation of std::invalid_argument — nothing
// here catches or rewraps).
#include <nanobind/nanobind.h>
#include <nanobind/make_iterator.h>
#include <nanobind/stl/string.h>

#include "CAD_0/brep/topology.hpp"
#include "CAD_0/brep/query.hpp"
#include "CAD_0/brep/mass_properties.hpp"
#include "CAD_0/brep/ssi.hpp"
#include "CAD_0/brep/subtract.hpp"
#include "CAD_0/brep/booleans_ext.hpp"

namespace nb = nanobind;
using CAD_0::brep::BRepBody;
using CAD_0::brep::BodySummary;
using CAD_0::brep::FaceId;
using CAD_0::brep::FaceInfo;
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
        "distinction is never a guess. Raises ValueError for a face id\n"
        "the body does not carry (diagnosis + remedy verbatim).");

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
                body_a, face_a, body_b, face_b, float(tol)));
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
        [](BRepBody& body, CAD_0::math::Vec3f delta) {
            CAD_0::brep::translate_body(body, delta);
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

    // THE EXTENDED BOOLEANS (G2 unit 2, ADR-0033 D5.2): the operations
    // CONSUME the unit-1 table — union/intersect/subtract run exact
    // exactly where every curved pair of the operands sits in the
    // typed table, the class check part of the ENTRY contract. The
    // sphere pairs (the classical union, the lens, the bitten sphere),
    // the sphere±prism family (the bead, the rivet, the core, the
    // truncations), and the prism-pair subtract's pierced-target class
    // (the two-holes body — the second cut). Every out-of-class input
    // raises ValueError with the actionable refusal (the pair, the
    // class, the remedies); the trimmed-sphere mass family lands with
    // the bodies (mass_properties reads the latitude-bounded windows).
    const auto bind_op = [&](const char* py_name,
                             std::unique_ptr<CAD_0::brep::BRepBody> (
                                 *fn)(const BRepBody&, const BRepBody&,
                                      CAD_0::math::Vec3f, float),
                             const char* doc) {
        // The body handle is the bridge submodule's opaque
        // registration (one type, two modules): the ownership crosses
        // as a released pointer under take_ownership, which nanobind
        // wraps in the registered holder's own deleter.
        brep_mod.def(
            py_name,
            [fn](const BRepBody& a, const BRepBody& b,
                 CAD_0::math::Vec3f offset,
                 double tol) -> BRepBody* {
                return fn(a, b, offset, float(tol)).release();
            },
            nb::arg("body_a"), nb::arg("body_b"),
            nb::arg("offset") = CAD_0::math::Vec3f{0, 0, 0},
            nb::arg("tol") = 1e-6, nb::rv_policy::take_ownership, doc);
    };
    bind_op("exact_union", CAD_0::brep::exact_union,
            "A ∪ (B + offset) on the extended exact class: the\n"
            "sphere×sphere classical union, the sphere∪prism rivet,\n"
            "the dome (the box truncation). The operand order is\n"
            "canonically normalized (center, then radius) — union(A,B)\n"
            "and union(B,A) assemble ONE body, bit-identical. Prism\n"
            "pairs are Phase D's landed opinion (bool_union) — pointed\n"
            "at, never duplicated. Raises ValueError for every\n"
            "out-of-class input (the actionable refusal: the pair, the\n"
            "class, the remedies).");
    bind_op("exact_intersect", CAD_0::brep::exact_intersect,
            "A ∩ (B + offset) on the same class: the lens, the core\n"
            "(the coaxial circle prism), the near cap (the box\n"
            "truncation). The same entry contract and refusal culture\n"
            "as exact_union.");
    bind_op("exact_subtract", CAD_0::brep::exact_subtract,
            "A − (B + offset): the G1 through-hole subtract EXTENDED —\n"
            "the SECOND cut on a pierced target (the two-holes body:\n"
            "each cut appends +4V/+5E/+1F/+2H, χ = χ(A)−2), and the\n"
            "sphere-class targets (the bitten sphere, the bead, the\n"
            "truncation family, the slab). Raises ValueError for the\n"
            "out-of-class inputs (the non-annular refusals name the\n"
            "dispositions — the unequal-radii crossing carries the\n"
            "offset plane x = (d²+r₁²−r₂²)/(2d) and its two typed\n"
            "lines, never 'untypable').");
}
