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

namespace nb = nanobind;
using CAD_0::brep::BRepBody;
using CAD_0::brep::FaceId;
using CAD_0::brep::FaceInfo;
using CAD_0::brep::LoopInfo;
using CAD_0::brep::PCurveInfo;

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
}
