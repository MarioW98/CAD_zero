// bindings/python/src/bridge_bindings.cpp
//
// Exposes the SDF → B-Rep bridge (ADR-0026) to Python:
//
//   * CAD_0.bridge.SdfToBrepOptions     — conversion options
//   * CAD_0.bridge.BRepBody             — OPAQUE handle (see below)
//   * CAD_0.bridge.sdf_to_brep_faceted  — faceted extraction (MC33)
//   * CAD_0.bridge.sdf_to_brep_report   — plain-values topology report
//
// BRepBody is deliberately registered as an OPAQUE handle: no methods,
// no properties. Python gets it back from sdf_to_brep_faceted() as a
// self-contained, freshly-built value it can hold and drop — nothing
// more. Reading topology data goes through sdf_to_brep_report(), which
// returns plain numbers/strings and therefore needs no B-Rep API on
// the Python side at all.
//
// Ownership (the one decision this file makes, stated explicitly):
// the bridge returns a std::unique_ptr<BRepBody> to a brand-new body
// that owns all of its data (vertices, edges, faces, surfaces) and has
// NO back-references into the scene, the SDF tree or any other kernel
// state. nanobind's default holder (std::unique_ptr) therefore gives
// Python exclusive ownership: the body lives until the last Python
// reference drops, then is destroyed. No aliasing, no lifetime hazard.
//
// Exceptions: the bridge throws std::invalid_argument /
// std::runtime_error with actionable messages that already contain the
// remedy (ADR-0026 D2 + acceptance correction 2). nanobind's default
// translation maps them to ValueError / RuntimeError carrying the FULL
// what() text; nothing here catches or rewraps — a generic wrap would
// erase exactly the remedy the message contract promises.
#include <nanobind/nanobind.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/unique_ptr.h>

#include "CAD_0/bridge/sdf_to_brep.hpp"
#include "CAD_0/brep/topology.hpp"

namespace nb = nanobind;
using CAD_0::bridge::SdfToBrepOptions;
using CAD_0::bridge::SdfToBrepReport;
using CAD_0::brep::BRepBody;
using CAD_0::sdf::SDFBody;

void bind_bridge(nb::module_& m) {
    auto bridge_mod = m.def_submodule(
        "bridge", "SDF ↔ B-Rep conversion bridge (ADR-0026)");

    // SdfToBrepOptions — mirrors the C++ struct 1:1 (field names
    // included: march_resolution, not resolution, so Python code
    // translates to/from C++ without renaming).
    nb::class_<SdfToBrepOptions>(bridge_mod, "SdfToBrepOptions")
        .def(nb::init<>())
        .def_rw("march_resolution", &SdfToBrepOptions::march_resolution,
                "Marching-cubes grid resolution per axis (default 48)")
        .def_rw("fit_primitives", &SdfToBrepOptions::fit_primitives,
                "Try the exact analytic primitive path first (default true)")
        .def_rw("compute_normals", &SdfToBrepOptions::compute_normals,
                "Compute vertex normals during extraction (default true)");

    // Opaque BRepBody handle — see the file header. The repr names the
    // type only; topology numbers come from sdf_to_brep_report().
    nb::class_<BRepBody>(bridge_mod, "BRepBody",
        "Opaque handle to a B-Rep body built by the bridge. Python owns\n"
        "it exclusively (fresh self-contained value, no kernel state\n"
        "inside); it exposes no API. Inspect results via\n"
        "sdf_to_brep_report().")
        .def("__repr__", [](const BRepBody&) {
            return "<CAD_0.bridge.BRepBody (opaque handle)>";
        });

    // Faceted extraction (ADR-0026 D3: one Face per welded MC
    // triangle, hard is_closed_manifold postcondition). Errors cross
    // the binding as ValueError / RuntimeError with the full bridge
    // message (diagnosis + remedy), never wrapped.
    bridge_mod.def("sdf_to_brep_faceted",
        [](const SDFBody& body, const SdfToBrepOptions& opts) {
            return CAD_0::bridge::sdf_to_brep_faceted(body, opts);
        },
        nb::arg("body"), nb::arg("opts") = SdfToBrepOptions(),
        "Extract a faceted B-Rep body from an SDF body (MC33 + welding,\n"
        "ADR-0026). Returns an opaque BRepBody handle owned by Python.");

    // Topology report — the consumer-friendly entry point: runs the
    // same dispatch as the C++ sdf_to_brep() and returns plain values,
    // so the caller needs no B-Rep API. Keys mirror the ADR-0026 D2
    // contract: V/E/F are grid-dependent; chi/closed/loops are not.
    bridge_mod.def("sdf_to_brep_report",
        [](const SDFBody& body, const SdfToBrepOptions& opts) {
            const SdfToBrepReport r =
                CAD_0::bridge::sdf_to_brep_report(body, opts);
            nb::dict out;
            out["V"] = r.vertex_count;
            out["E"] = r.edge_count;
            out["F"] = r.face_count;
            out["chi"] = r.chi;
            out["closed"] = r.closed;
            out["loops"] = r.loops;
            out["fit"] = r.fit;
            return out;
        },
        nb::arg("body"), nb::arg("opts") = SdfToBrepOptions(),
        "Convert an SDF body to B-Rep and report the resulting topology\n"
        "as a dict: V, E, F (grid-dependent), chi (V - E + F), closed\n"
        "(is_closed_manifold), loops (loops_are_closed), and fit — the\n"
        "path that produced the body: \"sphere\"|\"box\"|\"cylinder\"\n"
        "(exact analytic primitive) or \"faceted\" (MC33 extraction).");
}
