// core/bridge/src/sdf_to_brep.cpp
#include "CAD_0/bridge/sdf_to_brep.hpp"

#include "CAD_0/brep/topology.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>

namespace CAD_0::bridge {

namespace {

// ---------------------------------------------------------------------------
// Actionable error helpers (ADR-0026 D2 + acceptance correction 2:
// visible errors that name the likely cause and the remedy — never a
// bare assertion, never a silent skip).
// ---------------------------------------------------------------------------

[[noreturn]] void throw_empty_surface() {
    throw std::invalid_argument(
        "sdf_to_brep_faceted: the SDF produced no surface mesh - nothing "
        "to convert. The field has no zero crossing inside its bounds "
        "(empty or fully-outside field), or the resolution is 0. Give "
        "the SDF a non-empty interior or use a resolution > 0.");
}

[[noreturn]] void throw_degenerate_triangle(std::size_t tri) {
    std::ostringstream os;
    os << "sdf_to_brep_faceted: degenerate triangle " << tri
       << " (zero area) - likely cause: the SDF evaluates to exactly 0 "
          "ON a grid point (ADR-0026 input contract P2), which makes "
          "distinct grid-edge crossings coincide. Remedy: offset the "
          "sampling grid (e.g. translate the SDF by a small generic "
          "amount) or change the resolution.";
    throw std::runtime_error(os.str());
}

[[noreturn]] void throw_postcondition() {
    throw std::runtime_error(
        "sdf_to_brep_faceted: postcondition failed - the assembled body "
        "is not a closed orientable manifold (is_closed_orientable, "
        "ADR-0022 sec. 7 as amended by ADR-0027 D6 - the structural "
        "invariants; chi is reported in the topology report, not "
        "asserted here). Either this is a bridge bug, or the input "
        "violates the ADR-0026 supported input class (P1 enclosure / "
        "P2 on-grid zeros / P3 feature separation vs cell size). Check "
        "the bounds padding and the resolution against the minimum "
        "feature size; if the input is a supported sphere, please "
        "report this as a bridge bug.");
}

// The declared grid tolerance of the faceted path — the LINEAR bound
// every extracted vertex carries (ADR-0032 D2.4 + D8, G1 unit 1).
//
// Derivation, from the extraction's own geometry: a crossing vertex
// lies ON a grid edge of length h_axis = extent_axis / resolution,
// between two samples of opposite sign; the true surface crossing
// lies on the SAME edge (continuity), so the sample is within
// h_axis of a true surface point. The body-wide bound is the worst
// axis: h_max = max(padded extent) / resolution.
//
// The grid layout mirrors core/sdf/src/mesh_extract.cpp (bounds()
// padded by 1% of extent, march_resolution points per axis) — the
// same mirror discipline grid_min_abs_field applies at the bottom of
// this file, and the same one the G1 unit-1 test replicates
// independently: three copies cross-check each other, and a layout
// change breaks the mirrors first, forcing a conscious sync instead
// of a silent drift.
double grid_linear_tolerance(const sdf::SDFBody& sdf_body,
                              std::uint32_t res) {
    if (res == 0) {
        // Degenerate grid: the faceted path rejects resolution 0
        // outright (throw_empty_surface), so this is unreachable on
        // the success path — 0 mirrors grid_min_abs_field's honest
        // defensive reading.
        return 0.0;
    }
    math::Bboxf bx = sdf_body.bounds();
    if (bx.empty()) {
        // Unreachable after a successful extraction (marching_cubes
        // on empty bounds yields no mesh and throws first);
        // defensive.
        return 0.0;
    }
    const math::Vec3f pad = bx.extent() * 0.01f;
    bx.min = bx.min - pad;
    bx.max = bx.max + pad;
    const math::Vec3f ext = bx.extent();
    const float h_max = std::max({ext.x, ext.y, ext.z});
    return static_cast<double>(h_max) / static_cast<double>(res);
}

} // namespace

// ---------------------------------------------------------------------------
// Faceted conversion (ADR-0026 MVP): welded MC mesh → radial-edge body.
// Reuses the ADR-0025 v5 assembly patterns instead of inventing new ones.
// ---------------------------------------------------------------------------
std::unique_ptr<brep::BRepBody> sdf_to_brep_faceted(
    const sdf::SDFBody& sdf_body,
    const SdfToBrepOptions& opts) {

    // 1. Extraction pipeline — unchanged (ADR-0015/0016: MC33 with
    //    asymptotic decider, canonical edge-key welding, canonicalized
    //    edge interpolation).
    sdf::TriangleMesh mesh = sdf::marching_cubes(
        sdf_body, opts.march_resolution, opts.compute_normals);

    if (mesh.positions.empty() || mesh.indices.empty()) {
        throw_empty_surface();
    }

    auto body = std::make_unique<brep::BRepBody>();
    auto& shell = body->solid.shell_mut();

    // 2. Vertex map — 1:1 with the welded mesh. The MC welder already
    //    produces canonical vertex identity (integer edge keys), so the
    //    SharedVertexPool philosophy (ADR-0025 sec. 6) degenerates to
    //    identity here: by construction, not by tolerance.
    body->vertices.reserve(mesh.positions.size());
    for (const auto& p : mesh.positions) {
        // THE FACETED WORLD IN (ADR-0034): the float mesh promoted to
        // the exact world's double storage — lossless (every float32
        // is exactly a double); the provenance field carries the
        // declared tolerance, the promotion adds none.
        body->vertices.push_back(brep::Vertex{math::Vec3d{p.x, p.y, p.z}});
    }
    for (std::size_t i = 0; i < body->vertices.size(); ++i) {
        body->vertices[i].set_id(static_cast<brep::VertexId>(i));
    }

    // 3. Edge map — keyed by the canonical (min, max) vertex pair, the
    //    same integer pattern as ADR-0025 v5 assemble_body. Each edge's
    //    intrinsic direction is min -> max; a loop traversing a -> b
    //    uses the edge reversed iff a > b. This derives the coedge
    //    flag from the EDGE'S INTRINSIC DIRECTION vs the loop's
    //    traversal direction — fixing the pre-Phase-D stub, which
    //    derived it from the creating triangle's index order.
    std::map<std::pair<brep::VertexId, brep::VertexId>, brep::EdgeId> edge_map;
    auto get_or_create_edge = [&](brep::VertexId a, brep::VertexId b)
        -> std::pair<brep::EdgeId, bool> {
        const auto key = std::make_pair(std::min(a, b), std::max(a, b));
        const auto [it, inserted] = edge_map.emplace(
            key, static_cast<brep::EdgeId>(body->edges.size()));
        if (inserted) {
            body->edges.push_back(brep::Edge{key.first, key.second});
            body->edges.back().set_id(it->second);
        }
        return {it->second, a > b};
    };

    // 4. Faces — one per MC triangle (ADR-0026 D3), no skips. A
    //    degenerate triangle (repeated index or exactly-zero area —
    //    bit-identical coincident positions make the cross product
    //    exactly zero, no tolerance involved) is a P2 contract
    //    violation: a visible, actionable error instead of the old
    //    silent `continue` that punched holes while claiming closure.
    const std::size_t num_tris = mesh.indices.size() / 3;
    for (std::size_t t = 0; t < num_tris; ++t) {
        const auto i0 = mesh.indices[t * 3 + 0];
        const auto i1 = mesh.indices[t * 3 + 1];
        const auto i2 = mesh.indices[t * 3 + 2];

        const auto& p0 = mesh.positions[i0];
        const auto& p1 = mesh.positions[i1];
        const auto& p2 = mesh.positions[i2];

        const math::Vec3f n = (p1 - p0).cross(p2 - p0);
        if (i0 == i1 || i1 == i2 || i2 == i0 || n.length_sq() == 0.0f) {
            throw_degenerate_triangle(t);
        }

        auto surf = std::make_shared<brep::PlaneSurface>(
            math::Vec3d{p0.x, p0.y, p0.z},
            math::Vec3d{n.x, n.y, n.z});
        brep::Face face(surf);
        face.set_id(static_cast<brep::FaceId>(t));

        // The loop follows the mesh triangle's winding (i0 -> i1 ->
        // i2 -> i0), which MC33 emits consistently oriented (global
        // signed-volume pass), so the surface normal agrees with the
        // loop by the right-hand rule.
        brep::Loop loop;
        const auto [e01, r01] = get_or_create_edge(i0, i1);
        loop.add_coedge(brep::CoEdge{e01, r01});
        const auto [e12, r12] = get_or_create_edge(i1, i2);
        loop.add_coedge(brep::CoEdge{e12, r12});
        const auto [e20, r20] = get_or_create_edge(i2, i0);
        loop.add_coedge(brep::CoEdge{e20, r20});
        face.add_loop(std::move(loop));

        shell.add_face(std::move(face));
    }

    // 5. Assembly + hard postcondition. link_radial_edges is the only
    //    sanctioned radial population point (ADR-0021 sec. 4). The
    //    shell flag reflects the REAL invariant: claimed, verified
    //    immediately, and retracted (with the throw) if the structural
    //    contract (ADR-0022 sec. 7 invariants 0-3, as amended by
    //    ADR-0027 D6) does not hold. The input's topology class
    //    (chi/genus - sphere-like, torus-like, multi-shell) is NOT an
    //    assembly property: it is REPORTED by sdf_to_brep_report,
    //    never asserted here (a genus-1 pierced box extracts exactly
    //    like a genus-0 sphere).
    brep::link_radial_edges(*body);
    shell.set_closed(true);
    if (!brep::is_closed_orientable(*body)) {
        shell.set_closed(false);
        throw_postcondition();
    }

    // Layer-2 honesty (ADR-0032 D2.4 + D8, G1 unit 1): every entity
    // of the faceted extract is a SAMPLE and says so — approximated,
    // with the declared grid tolerance as the linear bound. Set after
    // the structural postcondition so a rejected body is discarded
    // whole; the primitive-fitting path above writes no records at
    // all, which is exact/0 — the factories' own contract.
    const brep::ProvenanceInfo approx = brep::ProvenanceInfo::approximated(
        grid_linear_tolerance(sdf_body, opts.march_resolution));
    for (auto& v : body->vertices) {
        v.set_provenance(approx);
    }
    for (auto& e : body->edges) {
        e.set_provenance(approx);
    }
    for (auto& f : shell.faces_mut()) {
        f.set_provenance(approx);
    }
    return body;
}

// ---------------------------------------------------------------------------
// Primitive fitting: detect known SDF types
// ---------------------------------------------------------------------------
std::unique_ptr<brep::BRepBody> sdf_to_brep_primitive(
    const sdf::SDFBody& sdf_body) {

    const sdf::SDFNode* root = sdf_body.root();
    if (!root) return nullptr;

    std::string desc = root->describe();

    // Check for sphere: description must START with "sphere(" (not contain it)
    if (desc.substr(0, 7) == "sphere(") {
        auto pos = desc.find("r=");
        if (pos != std::string::npos) {
            float r = std::stof(desc.substr(pos + 2));
            return brep::make_sphere(r);
        }
    }

    // Check for box
    if (desc.substr(0, 4) == "box(") {
        auto pos = desc.find("e=");
        if (pos != std::string::npos) {
            std::string vals = desc.substr(pos + 2);
            float ex = std::stof(vals);
            auto c1 = vals.find(",");
            if (c1 != std::string::npos) {
                float ey = std::stof(vals.substr(c1 + 1));
                auto c2 = vals.find(",", c1 + 1);
                if (c2 != std::string::npos) {
                    float ez = std::stof(vals.substr(c2 + 1));
                    return brep::make_box({ex, ey, ez});
                }
            }
        }
    }

    // Check for cylinder
    if (desc.substr(0, 9) == "cylinder(") {
        auto pos = desc.find("r=");
        if (pos != std::string::npos) {
            float r = std::stof(desc.substr(pos + 2));
            auto hpos = desc.find("h=");
            if (hpos != std::string::npos) {
                float h = std::stof(desc.substr(hpos + 2));
                return brep::make_cylinder(r, h);
            }
        }
    }

    // Not a recognized primitive
    return nullptr;
}

// ---------------------------------------------------------------------------
// High-level: try primitive first, fall back to faceted
// ---------------------------------------------------------------------------
std::unique_ptr<brep::BRepBody> sdf_to_brep(
    const sdf::SDFBody& sdf_body,
    const SdfToBrepOptions& opts) {

    // Try primitive fitting first
    if (opts.fit_primitives) {
        auto body = sdf_to_brep_primitive(sdf_body);
        if (body) return body;
    }

    // Fall back to faceted conversion
    return sdf_to_brep_faceted(sdf_body, opts);
}

// ---------------------------------------------------------------------------
// Topology report (ADR-0026 consumer step): same dispatch as
// sdf_to_brep(), plain-values summary — the caller needs no BRepBody
// API (the Python binding behind the app's Debug menu).
// ---------------------------------------------------------------------------
namespace {

// Label of the primitive the fitter matched, using the SAME describe()
// prefixes sdf_to_brep_primitive() matches on. Only consulted when the
// fit already succeeded, so the "faceted" fallback is unreachable in
// practice — it exists so the helper stays total (and fails safe: a
// label the table does not know reports "faceted", never a lie about
// topology). Keep the prefix list in sync with the fitter.
const char* primitive_fit_label(const sdf::SDFBody& sdf_body) {
    const sdf::SDFNode* root = sdf_body.root();
    if (root == nullptr) return "faceted";
    const std::string desc = root->describe();
    if (desc.rfind("sphere(", 0) == 0) return "sphere";
    if (desc.rfind("box(", 0) == 0) return "box";
    if (desc.rfind("cylinder(", 0) == 0) return "cylinder";
    return "faceted";
}

// min |f| over the marching-cubes sample grid — the machine-readable
// P2 margin (ADR-0026 Consequences left-over, resolved as a report
// field; see SdfToBrepReport::min_abs_field).
//
// The grid layout mirrors core/sdf/src/mesh_extract.cpp: bounds()
// padded by 1% of extent, points at min + extent * (i / res). The
// mirror is deliberately NOT shared with test T4 (which replicates
// the same layout independently): two copies cross-check each other,
// and a layout change breaks T4 first, forcing a conscious sync
// instead of a silent drift.
float grid_min_abs_field(const sdf::SDFBody& sdf_body, std::uint32_t res) {
    if (res == 0) {
        // Degenerate grid: no P2 margin exists. Reported as 0 — the
        // faceted path rejects resolution 0 outright (see
        // throw_empty_surface), so "zero margin" is the honest reading.
        return 0.0f;
    }
    math::Bboxf bx = sdf_body.bounds();
    if (bx.empty()) {
        // Unreachable after a successful dispatch (the faceted path
        // throws on empty bounds first); defensive.
        return 0.0f;
    }
    const math::Vec3f pad = bx.extent() * 0.01f;
    bx.min = bx.min - pad;
    bx.max = bx.max + pad;
    const auto ext = bx.extent();

    float min_abs_f = std::numeric_limits<float>::max();
    for (std::uint32_t zi = 0; zi <= res; ++zi) {
        for (std::uint32_t yi = 0; yi <= res; ++yi) {
            for (std::uint32_t xi = 0; xi <= res; ++xi) {
                const float fx = bx.min.x + ext.x * (static_cast<float>(xi) / static_cast<float>(res));
                const float fy = bx.min.y + ext.y * (static_cast<float>(yi) / static_cast<float>(res));
                const float fz = bx.min.z + ext.z * (static_cast<float>(zi) / static_cast<float>(res));
                min_abs_f = std::min(min_abs_f, std::abs(sdf_body.value({fx, fy, fz})));
            }
        }
    }
    return min_abs_f;
}

} // namespace

SdfToBrepReport sdf_to_brep_report(
    const sdf::SDFBody& sdf_body,
    const SdfToBrepOptions& opts) {

    // Same dispatch as sdf_to_brep(): primitive fit first (same
    // opts.fit_primitives gate), faceted fallback. Exceptions from
    // either path propagate unchanged — the report never masks a
    // bridge error (ADR-0026 D2 error contract).
    std::unique_ptr<brep::BRepBody> body;
    const char* fit = "faceted";
    if (opts.fit_primitives) {
        body = sdf_to_brep_primitive(sdf_body);
        if (body) {
            fit = primitive_fit_label(sdf_body);
        }
    }
    if (!body) {
        body = sdf_to_brep_faceted(sdf_body, opts);
    }

    SdfToBrepReport rep;
    rep.vertex_count = body->vertex_count();
    rep.edge_count = body->edge_count();
    rep.face_count = body->face_count();
    rep.chi = brep::euler_characteristic(*body);
    // Structural contract (ADR-0022 sec. 7 as amended by ADR-0027 D6):
    // the bridge's hard postcondition. The legacy `closed` field keeps
    // the wrapper's exact historical meaning (structural AND chi == 2)
    // so pre-Phase-F consumers see no semantic drift.
    rep.orientable = brep::is_closed_orientable(*body);
    rep.closed = brep::is_closed_manifold(*body);
    rep.loops = brep::loops_are_closed(*body);
    rep.fit = fit;
    // Genus, derived under the acceptance-correction-1 rule (the
    // blocking rule): only for a structurally sound body with
    // chi <= 2 and even (one shell of genus (2 - chi)/2). A
    // multi-shell body (chi > 2) reports n/d - genus_derived stays
    // false - and an odd chi (an assembly defect the structural
    // invariants cannot see) is never turned into a half-integer
    // genus: reporting, not asserting.
    if (rep.orientable) {
        if (const std::optional<std::int64_t> g =
                brep::genus_from_chi(rep.chi)) {
            rep.genus_derived = true;
            rep.genus = *g;
        }
    }
    // P2 margin as a diagnostic (see SdfToBrepReport::min_abs_field):
    // path-independent, so it is computed after EITHER dispatch.
    rep.min_abs_field = grid_min_abs_field(sdf_body, opts.march_resolution);
    return rep;
}

} // namespace CAD_0::bridge
