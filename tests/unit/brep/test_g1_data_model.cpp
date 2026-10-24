// tests/unit/brep/test_g1_data_model.cpp
//
// G1 unit 1 — THE DATA MODEL (ADR-0032 D2 + D3, accepted in review).
//
// The first unit of the exact-kernel ladder: it changes no behavior and
// plants the fence posts every later unit builds against —
//
//   * D2.1  the p-curve vocabulary: the sketch kernel's typed 2D
//           entities (ADR-0030 D3) ARE the B-Rep layer's parameter-
//           domain curve set — one opinion, one code;
//   * D2.2  CoEdge carries a p-curve in the owning face's domain,
//           alongside the existing 3D edge reference;
//   * D2.4  Layer-2 fields on vertex, edge, face: linear tolerance +
//           provenance (exact | approximated(tol));
//   * D3    euler_characteristic extended with the inner-loop term,
//           χ = V − E + F − H, with the derivation and the annulus
//           counterexample in the docstring — and the backward
//           compatibility PROVEN here on every body-producing path
//           and on every golden, not asserted;
//   * D8    the provenance rules: factories write exact/0, the faceted
//           bridge writes approximated(declared grid tol), booleans
//           propagate the worst of their operands — and NO path
//           presents approximated geometry as exact.
//
// Written FIRST (D9): before the implementation landed, this file
// failed for the right reason — the absence of the API itself (compile
// errors naming the missing header and members), the C++ analogue of
// the wiring-test AttributeError. The p-curve WRITERS arrive with
// units 3–4: unit 1 pins that every existing path leaves the field
// empty (the fence-post test below), so the day a path starts writing
// p-curves it does so by amending a visible contract, never silently.
#include <doctest/doctest.h>

#include "CAD_0/brep/topology.hpp"
#include "CAD_0/brep/pcurve.hpp"
#include "CAD_0/brep/factory.hpp"
#include "CAD_0/brep/boolean.hpp"
#include "CAD_0/bridge/sdf_to_brep.hpp"
#include "CAD_0/io/native.hpp"
#include "CAD_0/sdf/primitives.hpp"
#include "CAD_0/sdf/operators.hpp"
#include "CAD_0/sdf/transforms.hpp"

#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <limits>
#include <memory>
#include <sstream>
#include <string>
#include <type_traits>
#include <vector>

namespace b = CAD_0::brep;
namespace s = CAD_0::sdf;
namespace br = CAD_0::bridge;
using CAD_0::math::Vec2f;
using CAD_0::math::Vec3f;

// ---------------------------------------------------------------------------
// D2.1 — the compile-time proof: the B-Rep p-curve vocabulary IS the
// sketch kernel's entity set. Not "compatible with", not "mirrors" —
// the SAME types (the aliases in brep/pcurve.hpp). If anyone ever
// introduces a second 2D vocabulary, these lines stop compiling.
// ---------------------------------------------------------------------------
static_assert(std::is_same_v<b::Line2, s::SketchSegment>,
              "D2.1: Line2 must BE the sketch kernel's SketchSegment");
static_assert(std::is_same_v<b::Circle2, s::SketchCircle>,
              "D2.1: Circle2 must BE the sketch kernel's SketchCircle");
static_assert(std::is_same_v<b::Arc2, s::SketchArc>,
              "D2.1: Arc2 must BE the sketch kernel's SketchArc");
static_assert(std::is_same_v<b::PCurve, s::SketchEntity>,
              "D2.1: PCurve must BE the sketch kernel's SketchEntity");

// ---- helpers ----------------------------------------------------------------

namespace {

// The naive disk-faced formula, computed independently in the test —
// the value every existing pin was written against. The extended
// euler_characteristic must reproduce it exactly whenever H = 0 (the
// backward-compatibility contract, checked body by body).
std::int64_t naive_euler(const b::BRepBody& body) {
    return static_cast<std::int64_t>(body.vertex_count()) -
           static_cast<std::int64_t>(body.edge_count()) +
           static_cast<std::int64_t>(body.face_count());
}

bool all_entities_exact(const b::BRepBody& body) {
    const auto ok = [](const b::ProvenanceInfo& p) {
        return p.is_exact() && p.linear_tolerance == 0.0;
    };
    for (const auto& v : body.vertices) if (!ok(v.provenance())) return false;
    for (const auto& e : body.edges)    if (!ok(e.provenance())) return false;
    for (const auto& f : body.solid.shell().faces())
        if (!ok(f.provenance())) return false;
    return true;
}

bool all_entities_approximated_with(const b::BRepBody& body, double tol) {
    const auto ok = [tol](const b::ProvenanceInfo& p) {
        return !p.is_exact() && p.linear_tolerance == tol;
    };
    for (const auto& v : body.vertices) if (!ok(v.provenance())) return false;
    for (const auto& e : body.edges)    if (!ok(e.provenance())) return false;
    for (const auto& f : body.solid.shell().faces())
        if (!ok(f.provenance())) return false;
    return true;
}

// The no-lie detector: does ANY entity of this body claim exactness?
bool any_entity_claims_exact(const b::BRepBody& body) {
    for (const auto& v : body.vertices) if (v.provenance().is_exact()) return true;
    for (const auto& e : body.edges)    if (e.provenance().is_exact()) return true;
    for (const auto& f : body.solid.shell().faces())
        if (f.provenance().is_exact()) return true;
    return false;
}

// Unit 1's own boundary, as a query: no existing path writes p-curves.
bool body_has_no_p_curves(const b::BRepBody& body) {
    for (const auto& face : body.solid.shell().faces())
        for (const auto& loop : face.loops())
            for (const auto& ce : loop.coedges())
                if (ce.p_curve() != nullptr) return false;
    return true;
}

// The declared grid tolerance of the faceted path, mirrored from the
// bridge — the T12 discipline (two independent copies of the grid
// layout cross-check each other; a layout change breaks this mirror
// first, forcing a conscious sync instead of a silent drift).
//
// Layout (mirrors core/sdf/src/mesh_extract.cpp and the bridge's own
// mirror): bounds() padded by 1% of extent, march_resolution points
// per axis. The LINEAR bound a crossing vertex carries: the vertex
// lies on a grid edge of length h_axis between two sign-changing
// samples, the true crossing lies on the same edge, so the sample is
// within h_axis of a true surface point; the body-wide bound is the
// worst axis, h_max = max(padded extent) / resolution.
double expected_grid_tol(const s::SDFBody& body, std::uint32_t res) {
    REQUIRE(res > 0);
    CAD_0::math::Bboxf bx = body.bounds();
    REQUIRE(!bx.empty());
    const CAD_0::math::Vec3f pad = bx.extent() * 0.01f;
    bx.min = bx.min - pad;
    bx.max = bx.max + pad;
    const CAD_0::math::Vec3f ext = bx.extent();
    const float h_max = std::max({ext.x, ext.y, ext.z});
    return static_cast<double>(h_max) / static_cast<double>(res);
}

// A flat ANNULUS built directly at the topological layer: one
// PlaneSurface face, an outer square loop and an inner square loop.
// 8 vertices, 8 edges, each edge used by ONE co-edge — a surface with
// boundary, exactly the pierced face of the guide case before the
// wall is attached. The disk-faced formula miscounts it; the extended
// χ must not.
std::unique_ptr<b::BRepBody> make_annulus_body() {
    auto body = std::make_unique<b::BRepBody>();
    auto& shell = body->solid.shell_mut();
    body->vertices = {
        b::Vertex{{-2, -2, 0}}, b::Vertex{{ 2, -2, 0}},
        b::Vertex{{ 2,  2, 0}}, b::Vertex{{-2,  2, 0}},
        b::Vertex{{-1, -1, 0}}, b::Vertex{{ 1, -1, 0}},
        b::Vertex{{ 1,  1, 0}}, b::Vertex{{-1,  1, 0}},
    };
    for (std::size_t i = 0; i < body->vertices.size(); ++i)
        body->vertices[i].set_id(static_cast<b::VertexId>(i));
    body->edges = {
        b::Edge{0, 1}, b::Edge{1, 2}, b::Edge{2, 3}, b::Edge{3, 0},
        b::Edge{4, 5}, b::Edge{5, 6}, b::Edge{6, 7}, b::Edge{7, 4},
    };
    for (std::size_t i = 0; i < body->edges.size(); ++i)
        body->edges[i].set_id(static_cast<b::EdgeId>(i));

    b::Face face(std::make_shared<b::PlaneSurface>(Vec3f{0, 0, 0},
                                                   Vec3f{0, 0, 1}));
    // Outer loop CCW (material on the left of traversal): 0→1→2→3→0.
    b::Loop outer;
    outer.add_coedge(b::CoEdge{0, false});
    outer.add_coedge(b::CoEdge{1, false});
    outer.add_coedge(b::CoEdge{2, false});
    outer.add_coedge(b::CoEdge{3, false});
    face.add_loop(std::move(outer));
    // Inner loop CW around the hole (the annulus material stays on the
    // left of traversal): 4→7→6→5→4 — ADR-0021's "subsequent loops
    // are holes", flagged as the convention says.
    b::Loop inner;
    inner.set_outer(false);
    inner.add_coedge(b::CoEdge{7, true});   // 4→7 (edge 7: 7→4, reversed)
    inner.add_coedge(b::CoEdge{6, true});   // 7→6 (edge 6: 6→7, reversed)
    inner.add_coedge(b::CoEdge{5, true});   // 6→5 (edge 5: 5→6, reversed)
    inner.add_coedge(b::CoEdge{4, true});   // 5→4 (edge 4: 4→5, reversed)
    face.add_loop(std::move(inner));
    face.set_id(0);
    shell.add_face(std::move(face));
    b::link_radial_edges(*body);
    return body;
}

// A flat disk with TWO holes: outer square (±3) + two inner squares
// (half-extent 0.5 around (−1.5, 0) and (1.5, 0)). 12 vertices, 12
// edges, 1 face, H = 2 → χ = 12 − 12 + 1 − 2 = −1 (a pair-of-pants
// patch: χ = 1 − 2 holes).
std::unique_ptr<b::BRepBody> make_two_hole_disk_body() {
    auto body = std::make_unique<b::BRepBody>();
    auto& shell = body->solid.shell_mut();
    body->vertices = {
        // outer square ±3 (ids 0..3)
        b::Vertex{{-3, -3, 0}}, b::Vertex{{ 3, -3, 0}},
        b::Vertex{{ 3,  3, 0}}, b::Vertex{{-3,  3, 0}},
        // left hole square around (−1.5, 0) (ids 4..7)
        b::Vertex{{-2, -0.5f, 0}}, b::Vertex{{-1, -0.5f, 0}},
        b::Vertex{{-1,  0.5f, 0}}, b::Vertex{{-2,  0.5f, 0}},
        // right hole square around (1.5, 0) (ids 8..11)
        b::Vertex{{ 1, -0.5f, 0}}, b::Vertex{{ 2, -0.5f, 0}},
        b::Vertex{{ 2,  0.5f, 0}}, b::Vertex{{ 1,  0.5f, 0}},
    };
    for (std::size_t i = 0; i < body->vertices.size(); ++i)
        body->vertices[i].set_id(static_cast<b::VertexId>(i));
    body->edges = {
        b::Edge{0, 1}, b::Edge{1, 2}, b::Edge{2, 3}, b::Edge{3, 0},
        b::Edge{4, 5}, b::Edge{5, 6}, b::Edge{6, 7}, b::Edge{7, 4},
        b::Edge{8, 9}, b::Edge{9, 10}, b::Edge{10, 11}, b::Edge{11, 8},
    };
    for (std::size_t i = 0; i < body->edges.size(); ++i)
        body->edges[i].set_id(static_cast<b::EdgeId>(i));

    b::Face face(std::make_shared<b::PlaneSurface>(Vec3f{0, 0, 0},
                                                   Vec3f{0, 0, 1}));
    b::Loop outer;  // 0→1→2→3→0, CCW
    outer.add_coedge(b::CoEdge{0, false});
    outer.add_coedge(b::CoEdge{1, false});
    outer.add_coedge(b::CoEdge{2, false});
    outer.add_coedge(b::CoEdge{3, false});
    face.add_loop(std::move(outer));

    b::Loop left;   // 4→7→6→5→4, CW (material on the left)
    left.set_outer(false);
    left.add_coedge(b::CoEdge{7, true});
    left.add_coedge(b::CoEdge{6, true});
    left.add_coedge(b::CoEdge{5, true});
    left.add_coedge(b::CoEdge{4, true});
    face.add_loop(std::move(left));

    b::Loop right;  // 8→11→10→9→8, CW (material on the left)
    right.set_outer(false);
    right.add_coedge(b::CoEdge{11, true});
    right.add_coedge(b::CoEdge{10, true});
    right.add_coedge(b::CoEdge{9, true});
    right.add_coedge(b::CoEdge{8, true});
    face.add_loop(std::move(right));

    face.set_id(0);
    shell.add_face(std::move(face));
    b::link_radial_edges(*body);
    return body;
}

// The golden directory (tests run from the build tree; CAD_0_SOURCE_DIR
// is the repo root — the same macro test_native.cpp resolves its
// golden paths through).
std::filesystem::path golden_file(const char* name) {
    return std::filesystem::path(CAD_0_SOURCE_DIR) / "tests" / "golden" / name;
}

// Read a golden file as text (the v0.1.0 fixture is pure JSON — it goes
// through parse_document, not load_file; same helper shape as
// test_native.cpp's).
std::string read_file(const std::filesystem::path& p) {
    std::ifstream in(p, std::ifstream::binary);
    const std::string missing = "cannot open: " + p.string();
    REQUIRE_MESSAGE(static_cast<bool>(in), missing);
    std::ostringstream ss;
    ss << in.rdbuf();
    return ss.str();
}

} // namespace

// ---------------------------------------------------------------------------
// Family 1 — API discriminators (D9.1): the p-curve accessor, the
// provenance query, the loop semantics of euler_characteristic.
// ---------------------------------------------------------------------------

TEST_CASE("G1-U1 D2.1: the p-curve vocabulary is the sketch kernel's — one opinion, one code") {
    // The static_asserts at the top of this file are the compile-time
    // half. The runtime half: a p-curve is BUILT with the sketch
    // kernel's own factories — no B-Rep-side constructor exists, and
    // none is needed. The same typed entities, the same
    // construction-time validation, the same code.
    const b::PCurve seg = s::sketch_segment(Vec2f{0, 0}, Vec2f{2, 0}, true);
    CHECK(seg.kind == s::SketchEntity::Kind::Segment);
    CHECK(seg.segment.a == Vec2f{0, 0});
    CHECK(seg.segment.b == Vec2f{2, 0});
    CHECK(seg.segment.interior_left == true);

    const b::PCurve circ = s::sketch_circle(Vec2f{1, 1}, 0.25f, true);
    CHECK(circ.kind == s::SketchEntity::Kind::Circle);
    CHECK(circ.circle.center == Vec2f{1, 1});
    CHECK(circ.circle.radius == 0.25f);
    CHECK(circ.circle.hole == true);

    const b::PCurve arc = s::sketch_arc(Vec2f{0, 0}, 1.0f, false, 0.0f, 1.5f);
    CHECK(arc.kind == s::SketchEntity::Kind::Arc);
    CHECK(arc.arc.radius == 1.0f);
    CHECK(arc.arc.sweep == 1.5f);

    // The typed alias set reads naturally in B-Rep-side code — that is
    // the whole point of the names (D2.1):
    const b::Line2   line{Vec2f{0, 0}, Vec2f{1, 0}, true};  // a SketchSegment
    const b::Circle2 hole{Vec2f{0, 0}, 0.5f, true};         // a SketchCircle
    const b::Arc2    fillet{Vec2f{0, 0}, 0.5f, false, 0.0f, 1.5707964f};
    CHECK(line.interior_left);
    CHECK(hole.hole);
    CHECK(fillet.sweep == 1.5707964f);
}

TEST_CASE("G1-U1 D2.2: CoEdge carries a p-curve in the owning face's parameter domain") {
    // Default: NO p-curve — the faceted world (bridge output,
    // plane-per-triangle) never carries one; the provenance field
    // distinguishes the worlds, the accessor never guesses.
    b::CoEdge bare{3, false};
    CHECK(bare.p_curve() == nullptr);

    // Round-trip, verbatim: the writer's entity IS the reader's entity
    // (same code, D2.1) — a trimming loop and the sketch chain that
    // produced it cannot disagree because there is nothing to
    // translate.
    b::CoEdge ce{3, true};
    ce.set_p_curve(s::sketch_segment(Vec2f{0, 0}, Vec2f{4, 0}, true));
    REQUIRE(ce.p_curve() != nullptr);
    CHECK(ce.p_curve()->kind == s::SketchEntity::Kind::Segment);
    CHECK(ce.p_curve()->segment.a == Vec2f{0, 0});
    CHECK(ce.p_curve()->segment.b == Vec2f{4, 0});
    CHECK(ce.p_curve()->segment.interior_left == true);
    // The 3D reference is untouched — the p-curve sits ALONGSIDE it.
    CHECK(ce.edge() == 3);
    CHECK(ce.reversed() == true);

    // The same 3D edge carries DIFFERENT p-curves in different faces'
    // domains — two co-edges, two independent fields (the defining
    // structure of the exact B-Rep: one edge, one p-curve per use).
    b::CoEdge use_in_face_a{3, false};
    b::CoEdge use_in_face_b{3, false};
    use_in_face_a.set_p_curve(s::sketch_circle(Vec2f{0, 0}, 1.0f, false));
    use_in_face_b.set_p_curve(s::sketch_arc(Vec2f{0, 0}, 2.0f, true, 0.5f, 2.0f));
    REQUIRE(use_in_face_a.p_curve() != nullptr);
    REQUIRE(use_in_face_b.p_curve() != nullptr);
    CHECK(use_in_face_a.p_curve()->kind == s::SketchEntity::Kind::Circle);
    CHECK(use_in_face_b.p_curve()->kind == s::SketchEntity::Kind::Arc);
    CHECK(use_in_face_a.p_curve()->circle.radius == 1.0f);
    CHECK(use_in_face_b.p_curve()->arc.radius == 2.0f);
}

TEST_CASE("G1-U1 D2.4: Layer-2 fields — tolerance + provenance on vertex, edge, face") {
    // Default: exact, tolerance 0 — the mathematical ideal of a
    // constructed point/edge/face. The declaration burden falls on the
    // SAMPLING paths (the bridge declares; the tests pin it).
    const b::Vertex v{Vec3f{1, 2, 3}};
    CHECK(v.provenance().is_exact());
    CHECK(v.provenance().linear_tolerance == 0.0);

    const b::Edge e{0, 1};
    CHECK(e.provenance().is_exact());
    CHECK(e.provenance().linear_tolerance == 0.0);

    const b::Face f;
    CHECK(f.provenance().is_exact());
    CHECK(f.provenance().linear_tolerance == 0.0);

    // "Is this edge exact?" is a query, not an investigation (D8):
    b::Vertex vs{Vec3f{0, 0, 0}};
    vs.set_provenance(b::ProvenanceInfo::approximated(0.0625));
    CHECK_FALSE(vs.provenance().is_exact());
    CHECK(vs.provenance().linear_tolerance == 0.0625);

    b::Edge es{0, 1};
    es.set_provenance(b::ProvenanceInfo::approximated(0.0625));
    CHECK_FALSE(es.provenance().is_exact());

    b::Face fs;
    fs.set_provenance(b::ProvenanceInfo::approximated(0.0625));
    CHECK_FALSE(fs.provenance().is_exact());

    // The canonical records:
    CHECK(b::ProvenanceInfo::exact().is_exact());
    CHECK(b::ProvenanceInfo::exact().linear_tolerance == 0.0);
    CHECK_FALSE(b::ProvenanceInfo::approximated(0.5).is_exact());
    CHECK(b::ProvenanceInfo::approximated(0.5).linear_tolerance == 0.5);
}

TEST_CASE("G1-U1 D2.4: ProvenanceInfo validation — the pairing is the contract") {
    // approximated DEMANDS a finite tolerance > 0: a sample with a
    // zero or non-finite bound claims an exactness it does not have —
    // exactly the lie the field exists to prevent.
    CHECK_THROWS_AS(b::ProvenanceInfo(b::Provenance::Approximated, 0.0),
                    std::invalid_argument);
    CHECK_THROWS_AS(b::ProvenanceInfo(b::Provenance::Approximated, -1.0),
                    std::invalid_argument);
    CHECK_THROWS_AS(
        b::ProvenanceInfo(b::Provenance::Approximated,
                          std::numeric_limits<double>::quiet_NaN()),
        std::invalid_argument);
    CHECK_THROWS_AS(
        b::ProvenanceInfo(b::Provenance::Approximated,
                          std::numeric_limits<double>::infinity()),
        std::invalid_argument);

    // exact PAIRS with tolerance 0 — no fuzzy middle.
    CHECK_THROWS_AS(b::ProvenanceInfo(b::Provenance::Exact, 0.5),
                    std::invalid_argument);

    // The two sanctioned constructions never throw:
    CHECK_NOTHROW(b::ProvenanceInfo{});
    CHECK_NOTHROW(b::ProvenanceInfo::approximated(1e-3));
    CHECK_NOTHROW(b::ProvenanceInfo(b::Provenance::Approximated, 1e-3));
    CHECK_NOTHROW(b::ProvenanceInfo(b::Provenance::Exact, 0.0));
}

TEST_CASE("G1-U1 D3: euler_characteristic counts inner loops — the annulus counterexample") {
    const auto annulus = make_annulus_body();

    // The pinned counts of the counterexample (docstring + ADR-0032 D3):
    CHECK(annulus->vertex_count() == 8);
    CHECK(annulus->edge_count() == 8);
    CHECK(annulus->face_count() == 1);
    CHECK(annulus->solid.shell().faces()[0].loops().size() == 2);

    // The disk-faced formula miscounts an annulus as a disk:
    CHECK(naive_euler(*annulus) == 1);   // WRONG — and pinned as wrong
    // The extended formula is honest: H = 1 inner loop, χ = 0.
    CHECK(b::inner_loop_count(*annulus) == 1);
    CHECK(b::euler_characteristic(*annulus) == 0);

    // The count is positional (every loop beyond each face's first),
    // and agrees with the flag bookkeeping when both are honest:
    std::int64_t flagged = 0;
    for (const auto& face : annulus->solid.shell().faces())
        for (const auto& loop : face.loops())
            if (!loop.is_outer()) ++flagged;
    CHECK(flagged == b::inner_loop_count(*annulus));

    // Structural honesty, both directions: the loops DO close (the
    // body is a well-formed cell complex), and the annulus is NOT a
    // closed solid (each edge has ONE co-edge) — the extended χ is
    // honest for any complex, the closed-manifold contract is a
    // different, structural question. genus_from_chi(0) = 1 would be
    // the TORUS reading: legal only for closed orientable bodies, so
    // it is NOT derived here — the annulus is a surface with boundary.
    CHECK(b::loops_are_closed(*annulus));
    CHECK_FALSE(b::is_closed_orientable(*annulus));

    // A disk with two holes: H = 2, χ = 1 − 2 = −1.
    const auto pants = make_two_hole_disk_body();
    CHECK(pants->vertex_count() == 12);
    CHECK(pants->edge_count() == 12);
    CHECK(pants->face_count() == 1);
    CHECK(naive_euler(*pants) == 1);     // the disk formula, wrong again
    CHECK(b::inner_loop_count(*pants) == 2);
    CHECK(b::euler_characteristic(*pants) == -1);

    // genus_from_chi's rules are UNCHANGED (D3): even and ≤ 2 derives,
    // odd never does, > 2 means multi-shell and is not derived.
    CHECK(b::genus_from_chi(2).value_or(-1) == 0);
    CHECK(b::genus_from_chi(0).value_or(-1) == 1);   // the guide case's χ
    CHECK(b::genus_from_chi(-2).value_or(-1) == 2);
    CHECK_FALSE(b::genus_from_chi(4).has_value());
    CHECK_FALSE(b::genus_from_chi(1).has_value());
    CHECK_FALSE(b::genus_from_chi(-1).has_value());
}

// ---------------------------------------------------------------------------
// Family 2 — the extended invariant's backward compatibility: every
// body-producing path of the tree is disk-faced (H = 0), every pinned
// value survives — proven, not asserted.
// ---------------------------------------------------------------------------

TEST_CASE("G1-U1 D3: backward compatibility — every factory body is disk-faced, χ unchanged") {
    const std::pair<const char*, std::unique_ptr<b::BRepBody>> bodies[] = {
        {"make_box", b::make_box(Vec3f{1, 1, 1})},
        {"make_sphere", b::make_sphere(1.5f)},
        {"make_cylinder", b::make_cylinder(0.5f, 2.0f)},
        {"make_tetrahedron", b::make_tetrahedron()},
    };
    for (auto& [name, body] : bodies) {
        CAPTURE(name);
        // Every face has exactly ONE loop — the disk-faced proof at
        // the loop level. The seam-wired sphere (2V/1E/1F) and
        // cylinder (4V/5E/3F) faces are disks precisely because the
        // seam wire SLITS the periodic surface open; that is why their
        // pinned χ = 2 survives the H term unchanged.
        for (const auto& face : body->solid.shell().faces())
            CHECK(face.loops().size() == 1);
        CHECK(b::inner_loop_count(*body) == 0);
        CHECK(b::euler_characteristic(*body) == naive_euler(*body));
        CHECK(b::euler_characteristic(*body) == 2);
        CHECK(b::is_closed_orientable(*body));
    }
}

TEST_CASE("G1-U1 D3: backward compatibility — boolean outputs are disk-faced, χ pinned") {
    // The three fixtures whose exact counts test_boolean.cpp pins:
    // touching/corner geometries only (overlapping union is the
    // ADR-0025 v5 documented open-shell limitation, out of scope here).
    {
        auto A = b::make_box(Vec3f{1, 1, 1});
        auto B = b::make_box(Vec3f{1, 1, 1});
        for (auto& v : B->vertices)
            v.set_position(v.position() + Vec3f{0.5f, 0, 0});
        auto r = b::bool_intersect(*A, *B);
        REQUIRE(r != nullptr);
        CHECK(r->vertex_count() == 8);
        CHECK(r->edge_count() == 12);
        CHECK(r->face_count() == 6);
        CHECK(b::inner_loop_count(*r) == 0);
        CHECK(b::euler_characteristic(*r) == 2);
    }
    {
        auto A = b::make_box(Vec3f{1, 1, 1});
        auto B = b::make_box(Vec3f{1, 1, 1});
        for (auto& v : B->vertices)
            v.set_position(v.position() + Vec3f{0, 0, 2.0f});
        auto r = b::bool_union(*A, *B);
        REQUIRE(r != nullptr);
        CHECK(r->vertex_count() == 12);
        CHECK(r->edge_count() == 20);
        CHECK(r->face_count() == 10);
        CHECK(b::inner_loop_count(*r) == 0);
        CHECK(b::euler_characteristic(*r) == 2);
    }
    {
        auto A = b::make_box(Vec3f{1, 1, 1});
        auto B = b::make_box(Vec3f{1, 1, 1});
        for (auto& v : B->vertices)
            v.set_position(v.position() + Vec3f{1.0f, 1.0f, 1.0f});
        auto r = b::bool_subtract(*A, *B);
        REQUIRE(r != nullptr);
        CHECK(r->vertex_count() == 17);
        CHECK(r->edge_count() == 27);
        CHECK(r->face_count() == 12);
        CHECK(b::inner_loop_count(*r) == 0);
        CHECK(b::euler_characteristic(*r) == 2);
    }
}

TEST_CASE("G1-U1 D3: backward compatibility — every GOLDEN body is disk-faced, no pinned value moves") {
    // The walk: every golden document's SDF nodes, materialized
    // through the bridge's faceted path (the only body-producing path
    // a document has — D11: the format does not move). Each must be
    // disk-faced (H = 0), the extended χ must equal the old formula's
    // value, and the bodies with established pins must keep them.
    // Faceted triangles are one-loop faces BY CONSTRUCTION — this is
    // the proof that the H term changes nothing about any number any
    // test or report has ever pinned.
    const auto check = [](const s::SDFBody& sdf_body, std::uint32_t res,
                          std::int64_t pinned_chi, const char* what) {
        CAPTURE(what);
        br::SdfToBrepOptions opts;
        opts.march_resolution = res;
        opts.fit_primitives = false;  // the document path: faceted
        auto body = br::sdf_to_brep(sdf_body, opts);
        REQUIRE(body != nullptr);
        CHECK(b::inner_loop_count(*body) == 0);
        CHECK(b::euler_characteristic(*body) == naive_euler(*body));
        CHECK(b::euler_characteristic(*body) == pinned_chi);
        CHECK(b::is_closed_orientable(*body));
        // D8 on the golden bodies: faceted extracts are samples and
        // say so, with the declared grid tolerance of their grid.
        const double tol = expected_grid_tol(sdf_body, res);
        CHECK(tol > 0.0);
        CHECK(all_entities_approximated_with(*body, tol));
        CHECK_FALSE(any_entity_claims_exact(*body));
    };

    // v0.1.0 (frozen legacy): the primary sphere (χ = 2) and the
    // composite — the most decorated SDF in the goldens (subtract ∪
    // translate ∪ smooth ∪ twist ∪ scale). The composite's absolute χ
    // is not pinned anywhere: the backward-compat claim there is
    // precisely "extended == old formula, H = 0", checked above; the
    // pinned_chi argument passes the same naive value so the pinned
    // comparison is the compatibility one, not a new absolute pin.
    {
        const auto doc = CAD_0::io::native::parse_document(
            read_file(golden_file("native_v0_1_0.CAD_0")));
        REQUIRE(doc.nodes.size() == 2);
        check(doc.nodes[0].body, 16, 2, "v0.1.0 primary sphere");
        {
            CAPTURE("v0.1.0 composite");
            br::SdfToBrepOptions opts;
            opts.march_resolution = 16;
            opts.fit_primitives = false;
            auto body = br::sdf_to_brep(doc.nodes[1].body, opts);
            REQUIRE(body != nullptr);
            CHECK(b::inner_loop_count(*body) == 0);
            CHECK(b::euler_characteristic(*body) == naive_euler(*body));
            CHECK(b::is_closed_orientable(*body));
            CHECK(all_entities_approximated_with(
                *body, expected_grid_tol(doc.nodes[1].body, 16)));
        }
    }

    // v0.2.0: the canonical scene through the container era — the same
    // two SDF nodes (primary sphere + composite). The embedded
    // octahedron MESH contributes no B-Rep body (a DocumentMesh is
    // mesh data, not a kernel body — the day a mesh→B-Rep path exists
    // it will carry its own provenance rules; none exists in unit 1).
    {
        const auto gp = golden_file("native_v0_2_0.CAD_0");
        const std::string missing = "golden file missing: " + gp.string();
        REQUIRE_MESSAGE(std::filesystem::exists(gp), missing);
        const auto doc = CAD_0::io::native::load_file(gp);
        REQUIRE(doc.nodes.size() == 2);
        check(doc.nodes[0].body, 16, 2, "v0.2.0 primary sphere");
        {
            CAPTURE("v0.2.0 composite");
            br::SdfToBrepOptions opts;
            opts.march_resolution = 16;
            opts.fit_primitives = false;
            auto body = br::sdf_to_brep(doc.nodes[1].body, opts);
            REQUIRE(body != nullptr);
            CHECK(b::inner_loop_count(*body) == 0);
            CHECK(b::euler_characteristic(*body) == naive_euler(*body));
            CHECK(b::is_closed_orientable(*body));
            CHECK(all_entities_approximated_with(
                *body, expected_grid_tol(doc.nodes[1].body, 16)));
        }
    }

    // v0.3.0: the pierced box — the FIRST genus > 0 document in the
    // repository's history. χ = 0, genus 1: the faceted extract
    // certifies it with DISK faces (H = 0), which is exactly why the
    // extended formula keeps the value.
    {
        const auto gp = golden_file("native_v0_3_0.CAD_0");
        const std::string missing = "golden file missing: " + gp.string();
        REQUIRE_MESSAGE(std::filesystem::exists(gp), missing);
        const auto doc = CAD_0::io::native::load_file(gp);
        REQUIRE(doc.nodes.size() == 3);
        REQUIRE(doc.nodes[2].operation.has_value());
        check(doc.nodes[2].body, 48, 0, "v0.3.0 pierced box (subtract)");
        CHECK(b::genus_from_chi(0).value_or(-1) == 1);
    }

    // v0.4.0: the parametric hole — the sketcher's chain (sketch →
    // extrude → subtract) reaching the SAME topological contract as
    // v0.3: χ = 0, genus 1, H = 0.
    {
        const auto gp = golden_file("native_v0_4_0.CAD_0");
        const std::string missing = "golden file missing: " + gp.string();
        REQUIRE_MESSAGE(std::filesystem::exists(gp), missing);
        const auto doc = CAD_0::io::native::load_file(gp);
        REQUIRE(doc.nodes.size() == 4);
        REQUIRE(doc.nodes[3].operation.has_value());
        check(doc.nodes[3].body, 48, 0, "v0.4.0 parametric hole (subtract)");
    }
}

// ---------------------------------------------------------------------------
// Family 3 — the provenance rule (D8): bridge → approximated(tol),
// primitives → exact/0, booleans propagate the worst of their
// operands; NO path presents approximated geometry as exact.
// ---------------------------------------------------------------------------

TEST_CASE("G1-U1 D8: factories write exact, tolerance 0") {
    // Closed forms (D8): every entity of every factory body.
    const std::pair<const char*, std::unique_ptr<b::BRepBody>> bodies[] = {
        {"make_box", b::make_box(Vec3f{1, 2, 3})},
        {"make_sphere", b::make_sphere(2.0f)},
        {"make_cylinder", b::make_cylinder(1.0f, 3.0f)},
        {"make_tetrahedron", b::make_tetrahedron()},
    };
    for (auto& [name, body] : bodies) {
        CAPTURE(name);
        CHECK(all_entities_exact(*body));
    }
}

TEST_CASE("G1-U1 D8: the bridge's primitive path writes exact — closed forms, tolerance 0") {
    br::SdfToBrepOptions opts;
    opts.fit_primitives = true;
    const std::pair<const char*, s::SDFBody> fixtures[] = {
        {"sphere", s::make_sphere(2.0f)},
        {"box", s::make_box(Vec3f{1, 1, 1})},
        {"cylinder", s::make_cylinder(0.5f, 2.0f)},
    };
    for (auto& [name, sdf_body] : fixtures) {
        CAPTURE(name);
        auto body = br::sdf_to_brep(sdf_body, opts);
        REQUIRE(body != nullptr);
        CHECK(all_entities_exact(*body));
    }
}

TEST_CASE("G1-U1 D8: the bridge's faceted path writes approximated with the declared grid tolerance") {
    // Every entity of a faceted extract is a sample within the grid's
    // declared linear bound — the ADR-0026 declared-error culture as a
    // first-class field. The bound is mirrored independently here (the
    // T12 discipline).
    {
        s::SDFBody sph = s::make_sphere(1.0f);
        br::SdfToBrepOptions opts;
        opts.fit_primitives = false;
        opts.march_resolution = 16;
        auto body = br::sdf_to_brep(sph, opts);
        REQUIRE(body != nullptr);
        const double tol = expected_grid_tol(sph, 16);
        CHECK(tol > 0.0);
        CHECK(all_entities_approximated_with(*body, tol));
        CHECK_FALSE(any_entity_claims_exact(*body));
    }
    // The tolerance TRACKS the grid: a finer resolution halves-ish the
    // bound, and the field says so — a stale constant would not.
    {
        s::SDFBody sph = s::make_sphere(1.0f);
        br::SdfToBrepOptions opts;
        opts.fit_primitives = false;
        opts.march_resolution = 8;
        auto body = br::sdf_to_brep(sph, opts);
        REQUIRE(body != nullptr);
        CHECK(all_entities_approximated_with(*body, expected_grid_tol(sph, 8)));
        CHECK(expected_grid_tol(sph, 8) > expected_grid_tol(sph, 16));
    }
    // A non-primitive body (smooth union) — the faceted world is the
    // honest path for everything outside the exact class.
    {
        s::SDFBody blend = s::sdf_smooth_union(s::make_sphere(0.8f),
                                               s::make_sphere(0.6f), 0.2f);
        br::SdfToBrepOptions opts;
        opts.fit_primitives = false;
        opts.march_resolution = 16;
        auto body = br::sdf_to_brep(blend, opts);
        REQUIRE(body != nullptr);
        CHECK(all_entities_approximated_with(*body, expected_grid_tol(blend, 16)));
        CHECK_FALSE(any_entity_claims_exact(*body));
    }
}

TEST_CASE("G1-U1 D8: booleans propagate the worst of their operands — approximated is never presented as exact") {
    // Why marked bodies and not real faceted extracts as operands: the
    // booleans' supported class is EXACTLY-CONVEX polyhedra (ADR-0023/
    // 0025 — the half-space clipping and the exact-keyed assembly were
    // built on factory-grade fixtures), while an MC33 extract carries
    // grid-scale float noise: its vertices sit within the declared
    // tolerance of the true surface, not ON it, and the assembly's
    // weld/key machinery miscounts on that noise. This unit's first
    // test draft fed faceted spheres to bool_intersect and observed
    // χ = −15 on a body that should read χ = 2 — a DISCOVERED BOUNDARY
    // of the pre-existing machinery, not a unit-1 regression, and the
    // sharpest possible illustration of why the provenance field
    // exists: the boolean consumes exactness, and from now on it says
    // so per entity. The propagation itself is therefore proven on
    // factory boxes RELABELED as samples — the geometry stays inside
    // the supported class, the RECORD is the thing under test: the
    // machinery must honor the record, never launder it.
    const auto marked_box = [](Vec3f half, Vec3f shift, double tol) {
        auto body = b::make_box(half);
        const b::ProvenanceInfo approx = b::ProvenanceInfo::approximated(tol);
        for (auto& v : body->vertices) v.set_provenance(approx);
        for (auto& e : body->edges) e.set_provenance(approx);
        for (auto& f : body->solid.shell_mut().faces_mut())
            f.set_provenance(approx);
        for (auto& v : body->vertices)
            v.set_position(v.position() + shift);
        return body;
    };

    // exact ∘ exact → exact (the closed-form world stays closed-form).
    {
        auto A = b::make_box(Vec3f{1, 1, 1});
        auto B = b::make_box(Vec3f{1, 1, 1});
        for (auto& v : B->vertices)
            v.set_position(v.position() + Vec3f{0.5f, 0, 0});
        auto r = b::bool_intersect(*A, *B);
        REQUIRE(r != nullptr);
        CHECK(r->vertex_count() == 8);
        CHECK(r->edge_count() == 12);
        CHECK(r->face_count() == 6);
        CHECK(b::euler_characteristic(*r) == 2);
        CHECK(all_entities_exact(*r));
    }
    // marked ∘ exact (mixed) → approximated with the marked operand's
    // bound: one sampled operand makes every fragment a sample's
    // descendant — never silently reset to exact.
    {
        auto A = b::make_box(Vec3f{1, 1, 1});
        auto B = marked_box(Vec3f{1, 1, 1}, Vec3f{0.5f, 0, 0}, 0.05);
        auto r = b::bool_intersect(*A, *B);
        REQUIRE(r != nullptr);
        CHECK(r->vertex_count() == 8);
        CHECK(b::euler_characteristic(*r) == 2);
        CHECK(all_entities_approximated_with(*r, 0.05));
        CHECK_FALSE(any_entity_claims_exact(*r));
    }
    // marked(tol_a) ∘ marked(tol_b) → approximated with the WORST of
    // the bounds — the max, never the average, never the min.
    {
        auto A = marked_box(Vec3f{1, 1, 1}, Vec3f{0, 0, 0}, 0.05);
        auto B = marked_box(Vec3f{1, 1, 1}, Vec3f{0.5f, 0, 0}, 0.02);
        auto r = b::bool_intersect(*A, *B);
        REQUIRE(r != nullptr);
        CHECK(all_entities_approximated_with(*r, 0.05));
        CHECK_FALSE(any_entity_claims_exact(*r));
    }
    // marked − exact (the corner notch): the subtract's cavity caps
    // inherit the record like every other fragment.
    {
        auto A = b::make_box(Vec3f{1, 1, 1});
        auto B = marked_box(Vec3f{1, 1, 1}, Vec3f{1, 1, 1}, 0.03125);
        auto r = b::bool_subtract(*A, *B);
        REQUIRE(r != nullptr);
        CHECK(r->vertex_count() == 17);
        CHECK(r->edge_count() == 27);
        CHECK(r->face_count() == 12);
        CHECK(b::euler_characteristic(*r) == 2);
        CHECK(all_entities_approximated_with(*r, 0.03125));
        CHECK_FALSE(any_entity_claims_exact(*r));
    }
    // marked ∪ marked, identical operands — the A == B shortcut path
    // must propagate too (a copy of a sample is a sample).
    {
        auto A = marked_box(Vec3f{1, 1, 1}, Vec3f{0, 0, 0}, 0.05);
        auto B = marked_box(Vec3f{1, 1, 1}, Vec3f{0, 0, 0}, 0.05);
        auto r = b::bool_union(*A, *B);
        REQUIRE(r != nullptr);
        CHECK(b::euler_characteristic(*r) == 2);
        CHECK(all_entities_approximated_with(*r, 0.05));
        CHECK_FALSE(any_entity_claims_exact(*r));
    }
}

// ---------------------------------------------------------------------------
// The unit's own boundary, pinned: unit 1 writes NO p-curves — no
// factory, no bridge path, no boolean emits one (the writers arrive
// with units 3–4). The day a path starts writing p-curves, it amends
// THIS test consciously.
// ---------------------------------------------------------------------------

TEST_CASE("G1-U1: unit 1 writes no p-curves — every existing path leaves the field empty") {
    const std::pair<const char*, std::unique_ptr<b::BRepBody>> factories[] = {
        {"make_box", b::make_box(Vec3f{1, 1, 1})},
        {"make_sphere", b::make_sphere(1.0f)},
        {"make_cylinder", b::make_cylinder(0.5f, 2.0f)},
    };
    for (auto& [name, body] : factories) {
        CAPTURE(name);
        CHECK(body_has_no_p_curves(*body));
    }

    // The bridge, both paths:
    {
        auto exact_fit = br::sdf_to_brep(s::make_sphere(1.0f), {});
        REQUIRE(exact_fit != nullptr);
        CHECK(body_has_no_p_curves(*exact_fit));

        s::SDFBody blend = s::sdf_smooth_union(s::make_sphere(0.8f),
                                               s::make_sphere(0.6f), 0.2f);
        br::SdfToBrepOptions opts;
        opts.fit_primitives = false;
        opts.march_resolution = 16;
        auto faceted = br::sdf_to_brep(blend, opts);
        REQUIRE(faceted != nullptr);
        CHECK(body_has_no_p_curves(*faceted));
    }

    // A boolean over approximated operands (the heaviest existing
    // path — marked factory boxes, per the discovered boundary noted
    // in the propagation test above: the boolean's geometric class is
    // exactly-convex, so the record is what varies here, not the
    // geometry):
    {
        auto A = b::make_box(Vec3f{1, 1, 1});
        auto B = b::make_box(Vec3f{1, 1, 1});
        const b::ProvenanceInfo approx =
            b::ProvenanceInfo::approximated(0.03125);
        for (auto& e : B->edges) e.set_provenance(approx);
        for (auto& v : B->vertices)
            v.set_position(v.position() + Vec3f{1.0f, 1.0f, 1.0f});
        auto r = b::bool_subtract(*A, *B);
        REQUIRE(r != nullptr);
        CHECK(body_has_no_p_curves(*r));
    }
}
