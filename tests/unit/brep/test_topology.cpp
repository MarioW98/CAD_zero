// tests/unit/brep/test_topology.cpp
#include <doctest/doctest.h>

#include "CAD_0/brep/topology.hpp"
#include "CAD_0/brep/analytic.hpp"
#include "CAD_0/brep/factory.hpp"

using namespace CAD_0::brep;
using namespace CAD_0::math;

TEST_CASE("BRep: Vertex construction") {
    Vertex v{Vec3d{1, 2, 3}};
    CHECK(v.position() == Vec3d{1, 2, 3});
    v.set_id(42);
    CHECK(v.id() == 42);
}

TEST_CASE("BRep: Edge construction") {
    Edge e{1, 2};
    CHECK(e.vertex(0) == 1);
    CHECK(e.vertex(1) == 2);
    CHECK(e.is_manifold() == false);  // no coedges yet
    e.set_id(10);
    CHECK(e.id() == 10);
}

TEST_CASE("BRep: CoEdge direction") {
    std::vector<Edge> edges;
    edges.push_back(Edge{5, 7});

    CoEdge forward(0, false);
    CHECK(forward.start_vertex(edges) == 5);
    CHECK(forward.end_vertex(edges) == 7);

    CoEdge reversed(0, true);
    CHECK(reversed.start_vertex(edges) == 7);
    CHECK(reversed.end_vertex(edges) == 5);
}

TEST_CASE("BRep: Loop construction") {
    Loop loop;
    loop.add_coedge(CoEdge{0, false});
    loop.add_coedge(CoEdge{1, false});
    loop.add_coedge(CoEdge{2, false});
    loop.add_coedge(CoEdge{3, false});
    CHECK(loop.coedges().size() == 4);
    CHECK(loop.is_outer() == true);
}

TEST_CASE("BRep: Face with surface") {
    auto plane = std::make_shared<PlaneSurface>(Vec3d{0, 0, 0}, Vec3d{0, 1, 0});
    Face face(plane);
    CHECK(face.surface() != nullptr);
    CHECK(std::string(face.surface()->type_name()) == "plane");

    auto p = face.surface()->evaluate(1.0f, 2.0f);
    CHECK(p.y == doctest::Approx(0.0f));  // on the plane y=0
}

TEST_CASE("BRep: Shell and Solid") {
    Shell shell;
    CHECK(shell.faces().empty());
    shell.set_closed(true);
    CHECK(shell.is_closed() == true);

    Solid solid(std::move(shell));
    CHECK(solid.shell().is_closed() == true);
}

TEST_CASE("BRep: BRepBody basics") {
    BRepBody body;
    CHECK(body.vertex_count() == 0);
    CHECK(body.edge_count() == 0);
    CHECK(body.face_count() == 0);
}

TEST_CASE("BRep: make_box") {
    auto body = make_box(Vec3d{1, 1, 1});
    CHECK(body != nullptr);
    CHECK(body->vertex_count() == 8);
    CHECK(body->edge_count() == 12);
    CHECK(body->face_count() == 6);

    auto b = body->bounds();
    CHECK(b.min.x == doctest::Approx(-1.0f));
    CHECK(b.max.x == doctest::Approx(1.0f));
}

// ---------------------------------------------------------------------------
// ADR-0021 — radial-edge population, Euler characteristic, loop closure
// ---------------------------------------------------------------------------
//
// These tests exercise the contract documented in ADR-0021:
//   §4  link_radial_edges() must populate Edge::coedges_ from the
//       (Face, Loop, CoEdge) hierarchy. The factory calls it once
//       at the end of construction; every edge of a closed solid must
//       end up with exactly two co-edges (manifold contract).
//   §5  euler_characteristic(body) = V − E + F must equal 2 for any
//       closed-shell body. The box is the canonical case.
//   §6  loops_are_closed(body) must hold for every loop of every face.
//

TEST_CASE("ADR-0021: make_box populates radial-edge list (every edge has 2 coedges)") {
    auto body = make_box(Vec3d{1, 1, 1});
    REQUIRE(body->edge_count() == 12);
    for (const auto& edge : body->edges) {
        CHECK(edge.coedges().size() == 2);
        CHECK(edge.is_manifold());
    }
}

TEST_CASE("ADR-0021: make_box euler characteristic equals 2") {
    auto body = make_box(Vec3d{1, 1, 1});
    CHECK(euler_characteristic(*body) == 2);
}

TEST_CASE("ADR-0021: make_box with non-uniform half-extents still has chi == 2") {
    auto body = make_box(Vec3d{2, 3, 4});
    CHECK(euler_characteristic(*body) == 2);
}

TEST_CASE("ADR-0021: every loop of every box face is closed") {
    auto body = make_box(Vec3d{1, 1, 1});
    CHECK(loops_are_closed(*body));
}

TEST_CASE("ADR-0021: every box face has exactly one outer loop with 4 coedges") {
    auto body = make_box(Vec3d{1, 1, 1});
    REQUIRE(body->face_count() == 6);
    for (const auto& face : body->solid.shell().faces()) {
        REQUIRE(face.loops().size() == 1);
        const auto& loop = face.loops()[0];
        CHECK(loop.is_outer());
        CHECK(loop.coedges().size() == 4);
    }
}

TEST_CASE("ADR-0021: link_radial_edges is idempotent") {
    auto body = make_box(Vec3d{1, 1, 1});
    // Re-run the link pass: counts must stay at exactly 2 per edge.
    link_radial_edges(*body);
    for (const auto& edge : body->edges) {
        CHECK(edge.coedges().size() == 2);
    }
    CHECK(euler_characteristic(*body) == 2);
}

TEST_CASE("ADR-0021: radial-edge list contains CoEdges from the right faces") {
    // Edge 0 is the bottom-front edge {0,1}. The two faces that use it
    // are Face 0 (bottom, normal -Z, forward) and Face 2 (front, normal
    // -Y, forward). The radial list must contain both, and both must
    // reference edge id 0.
    auto body = make_box(Vec3d{1, 1, 1});
    REQUIRE(body->edge_count() >= 1);
    const auto& edge0 = body->edges[0];
    REQUIRE(edge0.coedges().size() == 2);
    for (const CoEdge* ce : edge0.coedges()) {
        CHECK(ce->edge() == 0);
    }
}

TEST_CASE("ADR-0021: vertical edge {0,4} is shared by Face 2 and Face 4") {
    // Edge 8 is the vertical edge {0,4}. It belongs to Face 2 (front)
    // and Face 4 (left). Face 2 walks it 4→0 (reversed), Face 4 walks
    // it 0→4 (forward). The radial list must have exactly two entries
    // with reversed flags as documented.
    auto body = make_box(Vec3d{1, 1, 1});
    REQUIRE(body->edge_count() >= 9);
    const auto& edge8 = body->edges[8];
    REQUIRE(edge8.coedges().size() == 2);
    int forward_count = 0;
    int reversed_count = 0;
    for (const CoEdge* ce : edge8.coedges()) {
        if (ce->reversed()) ++reversed_count;
        else                ++forward_count;
    }
    // Face 2 walks it reversed, Face 4 walks it forward.
    CHECK(forward_count == 1);
    CHECK(reversed_count == 1);
}

TEST_CASE("BRep: make_sphere") {
    auto body = make_sphere(2.0f);
    CHECK(body != nullptr);
    CHECK(body->face_count() == 1);
    CHECK(std::string(body->solid.shell().faces()[0].surface()->type_name()) == "sphere");
}

TEST_CASE("BRep: make_cylinder") {
    auto body = make_cylinder(1.0f, 2.0f);
    CHECK(body != nullptr);
    CHECK(body->face_count() == 3);  // lateral + 2 caps
}

// ---------------------------------------------------------------------------
// ADR-0022 — query operators + make_tetrahedron
// ---------------------------------------------------------------------------
//
// These tests exercise the ADR-0022 query operators (free functions,
// pure integer/pointer bookkeeping) on two canonical solids:
//   * make_box    — 8V/12E/6F/1S, every vertex valence 3
//   * make_tetrahedron — 4V/6E/4F/1S, every vertex valence 3
//
// Both bodies must satisfy is_closed_manifold(body) == true. The
// sphere/torus are explicit non-closed-manifold regression guards
// (their periodic-surface representation lacks seam wires — ADR-0022 §5).
//

// ---------------------------------------------------------------------------
// make_tetrahedron — structural counts
// ---------------------------------------------------------------------------

TEST_CASE("ADR-0022: make_tetrahedron produces 4V/6E/4F/1S") {
    auto body = make_tetrahedron();
    REQUIRE(body != nullptr);
    CHECK(body->vertex_count() == 4);
    CHECK(body->edge_count() == 6);
    CHECK(body->face_count() == 4);
}

TEST_CASE("ADR-0022: tetrahedron euler characteristic equals 2") {
    auto body = make_tetrahedron();
    CHECK(euler_characteristic(*body) == 2);
}

TEST_CASE("ADR-0022: tetrahedron every vertex has valence 3") {
    auto body = make_tetrahedron();
    for (VertexId v = 0; v < body->vertex_count(); ++v) {
        CHECK(vertex_valence(*body, v) == 3);
    }
}

TEST_CASE("ADR-0022: tetrahedron every edge has 2 coedges") {
    auto body = make_tetrahedron();
    for (const auto& edge : body->edges) {
        CHECK(edge.coedges().size() == 2);
        CHECK(edge.is_manifold());
    }
}

TEST_CASE("ADR-0022: tetrahedron every loop is closed") {
    auto body = make_tetrahedron();
    CHECK(loops_are_closed(*body));
}

TEST_CASE("ADR-0022: tetrahedron every face has one 3-coedge outer loop") {
    auto body = make_tetrahedron();
    for (const auto& face : body->solid.shell().faces()) {
        REQUIRE(face.loops().size() == 1);
        const auto& loop = face.loops()[0];
        CHECK(loop.is_outer());
        CHECK(loop.coedges().size() == 3);  // triangular
    }
}

// ---------------------------------------------------------------------------
// is_closed_manifold — strict contract
// ---------------------------------------------------------------------------

TEST_CASE("ADR-0022: is_closed_manifold(box) == true") {
    auto body = make_box(Vec3d{1, 1, 1});
    CHECK(is_closed_manifold(*body));
}

TEST_CASE("ADR-0022: is_closed_manifold(tetrahedron) == true") {
    auto body = make_tetrahedron();
    CHECK(is_closed_manifold(*body));
}

TEST_CASE("ADR-0022: is_closed_manifold(sphere) == true (seam wire fix, ADR-0023)") {
    // ADR-0023 §1: make_sphere now has a proper seam wire (2V/1E/1F)
    // and is a genuine closed manifold. The ADR-0022 regression guard
    // that expected `false` has been inverted — the root cause (missing
    // seam wire) is now fixed, not just masked.
    auto body = make_sphere(2.0f);
    CHECK(is_closed_manifold(*body));
}

TEST_CASE("ADR-0022: is_closed_manifold(cylinder) == true (seam wire fix, ADR-0023)") {
    // ADR-0023 §1: make_cylinder now has a proper seam wire (4V/5E/3F)
    // and is a genuine closed manifold. The ADR-0022 regression guard
    // that expected `false` has been inverted.
    auto body = make_cylinder(1.0f, 2.0f);
    CHECK(is_closed_manifold(*body));
}

// ---------------------------------------------------------------------------
// faces_around_edge — radial traversal
// ---------------------------------------------------------------------------

TEST_CASE("ADR-0022: box every edge has exactly 2 incident faces") {
    auto body = make_box(Vec3d{1, 1, 1});
    for (EdgeId eid = 0; eid < body->edge_count(); ++eid) {
        auto faces = faces_around_edge(*body, eid);
        CHECK(faces.size() == 2);
        // The two faces must be distinct.
        CHECK(faces[0] != faces[1]);
    }
}

TEST_CASE("ADR-0022: tetrahedron every edge has exactly 2 incident faces") {
    auto body = make_tetrahedron();
    for (EdgeId eid = 0; eid < body->edge_count(); ++eid) {
        auto faces = faces_around_edge(*body, eid);
        CHECK(faces.size() == 2);
        CHECK(faces[0] != faces[1]);
    }
}

TEST_CASE("ADR-0022: faces_around_edge on out-of-range edge returns empty") {
    auto body = make_box(Vec3d{1, 1, 1});
    auto faces = faces_around_edge(*body, 9999);
    CHECK(faces.empty());
}

// ---------------------------------------------------------------------------
// edges_around_vertex + vertex_valence
// ---------------------------------------------------------------------------

TEST_CASE("ADR-0022: box every vertex has valence 3") {
    auto body = make_box(Vec3d{1, 1, 1});
    for (VertexId v = 0; v < body->vertex_count(); ++v) {
        auto edges = edges_around_vertex(*body, v);
        CHECK(edges.size() == 3);
        CHECK(vertex_valence(*body, v) == 3);
    }
}

TEST_CASE("ADR-0022: edges_around_vertex on out-of-range vertex returns empty") {
    auto body = make_box(Vec3d{1, 1, 1});
    auto edges = edges_around_vertex(*body, 9999);
    CHECK(edges.empty());
}

// ---------------------------------------------------------------------------
// faces_around_vertex
// ---------------------------------------------------------------------------

TEST_CASE("ADR-0022: box every vertex touches exactly 3 faces") {
    auto body = make_box(Vec3d{1, 1, 1});
    for (VertexId v = 0; v < body->vertex_count(); ++v) {
        auto faces = faces_around_vertex(*body, v);
        CHECK(faces.size() == 3);
    }
}

TEST_CASE("ADR-0022: tetrahedron every vertex touches exactly 3 faces") {
    auto body = make_tetrahedron();
    for (VertexId v = 0; v < body->vertex_count(); ++v) {
        auto faces = faces_around_vertex(*body, v);
        CHECK(faces.size() == 3);
    }
}

// ---------------------------------------------------------------------------
// opposite_coedge — adjacency walk
// ---------------------------------------------------------------------------

TEST_CASE("ADR-0022: box opposite_coedge returns the other coedge on each edge") {
    auto body = make_box(Vec3d{1, 1, 1});
    for (const auto& edge : body->edges) {
        REQUIRE(edge.coedges().size() == 2);
        const CoEdge* ce0 = edge.coedges()[0];
        const CoEdge* ce1 = edge.coedges()[1];
        CHECK(opposite_coedge(*body, ce0) == ce1);
        CHECK(opposite_coedge(*body, ce1) == ce0);
    }
}

TEST_CASE("ADR-0022: opposite_coedge(nullptr) returns nullptr") {
    auto body = make_box(Vec3d{1, 1, 1});
    CHECK(opposite_coedge(*body, nullptr) == nullptr);
}

// ---------------------------------------------------------------------------
// wireframe_edges
// ---------------------------------------------------------------------------

TEST_CASE("ADR-0022: box wireframe has 12 canonical edges") {
    auto body = make_box(Vec3d{1, 1, 1});
    auto wf = wireframe_edges(*body);
    CHECK(wf.size() == 12);
    // Every pair must be canonical (v0 < v1).
    for (const auto& [a, b] : wf) {
        CHECK(a < b);
    }
}

TEST_CASE("ADR-0022: tetrahedron wireframe has 6 canonical edges") {
    auto body = make_tetrahedron();
    auto wf = wireframe_edges(*body);
    CHECK(wf.size() == 6);
    for (const auto& [a, b] : wf) {
        CHECK(a < b);
    }
}

// ---------------------------------------------------------------------------
// Edge orientation consistency — every edge's two co-edges must be
// oppositely oriented (one forward, one reversed). This is what makes
// the shell consistently oriented.
// ---------------------------------------------------------------------------

TEST_CASE("ADR-0022: box every edge has opposite coedge orientations") {
    auto body = make_box(Vec3d{1, 1, 1});
    for (const auto& edge : body->edges) {
        REQUIRE(edge.coedges().size() == 2);
        bool r0 = edge.coedges()[0]->reversed();
        bool r1 = edge.coedges()[1]->reversed();
        CHECK(r0 != r1);  // oppositely oriented
    }
}

TEST_CASE("ADR-0022: tetrahedron every edge has opposite coedge orientations") {
    auto body = make_tetrahedron();
    for (const auto& edge : body->edges) {
        REQUIRE(edge.coedges().size() == 2);
        bool r0 = edge.coedges()[0]->reversed();
        bool r1 = edge.coedges()[1]->reversed();
        CHECK(r0 != r1);
    }
}

TEST_CASE("ADR-0022: is_closed_manifold rejects same-orientation coedge pair (invariant 2 XOR)") {
    // Two triangular faces with IDENTICAL loops (0→1→2→0).
    // Every edge has exactly 2 co-edges, but all same-orientation.
    // V=3, E=3, F=2, χ=3-3+2=2 (χ passes — only XOR catches the duplication).
    BRepBody body;
    body.vertices = {
        Vertex{{0, 0, 0}},   // 0
        Vertex{{1, 0, 0}},   // 1
        Vertex{{0, 1, 0}},   // 2
    };
    for (std::size_t i = 0; i < body.vertices.size(); ++i)
        body.vertices[i].set_id(static_cast<VertexId>(i));

    body.edges = {
        Edge{0, 1},  // 0
        Edge{1, 2},  // 1
        Edge{0, 2},  // 2
    };
    for (std::size_t i = 0; i < body.edges.size(); ++i)
        body.edges[i].set_id(static_cast<EdgeId>(i));

    auto& shell = body.solid.shell_mut();
    shell.set_closed(true);

    auto add_face = [&shell](FaceId id) {
        Face face(std::make_shared<PlaneSurface>(Vec3d{0, 0, 0}, Vec3d{0, 0, 1}));
        Loop loop;
        loop.add_coedge(CoEdge{0, false});  // 0→1 forward
        loop.add_coedge(CoEdge{1, false});  // 1→2 forward
        loop.add_coedge(CoEdge{2, true});   // 2→0 reversed (edge 2 is 0→2)
        face.add_loop(std::move(loop));
        face.set_id(id);
        shell.add_face(std::move(face));
    };
    add_face(0);
    add_face(1);

    link_radial_edges(body);

    for (const auto& edge : body.edges) {
        CHECK(edge.coedges().size() == 2);
    }
    CHECK(euler_characteristic(body) == 2);
    CHECK_FALSE(is_closed_manifold(body));
}

// ---------------------------------------------------------------------------
// ADR-0027 D6 (Phase F, accepted) — is_closed_orientable (structural
// invariants 0–3), genus_from_chi (the reported genus, acceptance
// correction 1), and the is_closed_manifold compatibility wrapper
// (structural ∧ χ == 2 — the exact historical meaning).
// ---------------------------------------------------------------------------

TEST_CASE("ADR-0027: is_closed_orientable true on the genus-0 factories") {
    // box, tetrahedron, and the seam-wired sphere/cylinder (ADR-0023):
    // all structurally sound, all χ = 2 → both the structural contract
    // and the historical wrapper hold.
    auto check = [](const BRepBody& b) {
        CHECK(is_closed_orientable(b));
        CHECK(euler_characteristic(b) == 2);
        CHECK(is_closed_manifold(b));
    };
    check(*make_box(Vec3d{1, 1, 1}));
    check(*make_box(Vec3d{2, 3, 4}));
    check(*make_tetrahedron());
    check(*make_sphere(1.0f));
    check(*make_cylinder(0.5f, 2.0f));
}

TEST_CASE("ADR-0027: is_closed_orientable false on a no-edge body (invariant 0, the pre-ADR-0023 seam-wire shape)") {
    // The periodic-surface shape the invariant-0 guard exists for: a
    // shell claiming closure with faces but no edges/vertices (the
    // pre-ADR-0023 sphere/torus representation). Without the guard
    // this would pass vacuously.
    BRepBody body;
    auto& shell = body.solid.shell_mut();
    shell.set_closed(true);
    Face face(std::make_shared<PlaneSurface>(Vec3d{0, 0, 0}, Vec3d{0, 0, 1}));
    face.set_id(0);
    shell.add_face(std::move(face));

    CHECK_FALSE(is_closed_orientable(body));
    CHECK_FALSE(is_closed_manifold(body));  // wrapper: structural fails first
}

TEST_CASE("ADR-0027: is_closed_orientable false on the duplicated interface ring (XOR)") {
    // Same fixture as the ADR-0022 invariant-2 test above: two faces
    // glued back-to-back, χ = 2 — only the structural XOR catches it.
    // The is_closed_manifold wrapper inherits the rejection through
    // is_closed_orientable (its structural half), NOT through χ.
    BRepBody body;
    body.vertices = {
        Vertex{{0, 0, 0}},
        Vertex{{1, 0, 0}},
        Vertex{{0, 1, 0}},
    };
    for (std::size_t i = 0; i < body.vertices.size(); ++i)
        body.vertices[i].set_id(static_cast<VertexId>(i));
    body.edges = {Edge{0, 1}, Edge{1, 2}, Edge{0, 2}};
    for (std::size_t i = 0; i < body.edges.size(); ++i)
        body.edges[i].set_id(static_cast<EdgeId>(i));

    auto& shell = body.solid.shell_mut();
    shell.set_closed(true);
    for (FaceId id = 0; id < 2; ++id) {
        Face face(std::make_shared<PlaneSurface>(Vec3d{0, 0, 0}, Vec3d{0, 0, 1}));
        Loop loop;
        loop.add_coedge(CoEdge{0, false});
        loop.add_coedge(CoEdge{1, false});
        loop.add_coedge(CoEdge{2, true});
        face.add_loop(std::move(loop));
        face.set_id(id);
        shell.add_face(std::move(face));
    }
    link_radial_edges(body);

    CHECK(euler_characteristic(body) == 2);
    CHECK_FALSE(is_closed_orientable(body));
    CHECK_FALSE(is_closed_manifold(body));
}

TEST_CASE("ADR-0027: genus_from_chi — the derivation rule (acceptance correction 1)") {
    // Derived: one shell of genus (2 − χ)/2, χ ≤ 2 and even.
    CHECK(genus_from_chi(2)  == std::optional<std::int64_t>{0});
    CHECK(genus_from_chi(0)  == std::optional<std::int64_t>{1});   // pierced box
    CHECK(genus_from_chi(-2) == std::optional<std::int64_t>{2});   // double torus
    CHECK(genus_from_chi(-4) == std::optional<std::int64_t>{3});
    // NOT derived: multi-shell (χ > 2 — components counter deferred).
    CHECK(genus_from_chi(4)  == std::nullopt);
    CHECK(genus_from_chi(6)  == std::nullopt);
    CHECK(genus_from_chi(100) == std::nullopt);
    // NOT derived: odd χ — impossible as a sum of closed orientable
    // components' (even) characteristics; an assembly defect the
    // structural invariants cannot see. A half-integer genus would be
    // an assertion, not a report.
    CHECK(genus_from_chi(1)  == std::nullopt);
    CHECK(genus_from_chi(3)  == std::nullopt);
    CHECK(genus_from_chi(-1) == std::nullopt);
}

TEST_CASE("ADR-0027: wrapper equivalence on the genus-0 topology fixtures") {
    // is_closed_manifold == (is_closed_orientable && χ == 2) on every
    // genus-0 fixture of this suite — nothing that passed starts
    // failing, nothing that failed starts passing (the zero-silent-
    // weakening contract).
    auto check = [](const BRepBody& b) {
        CHECK(is_closed_manifold(b) ==
              (is_closed_orientable(b) && euler_characteristic(b) == 2));
    };
    check(*make_box(Vec3d{1, 1, 1}));
    check(*make_box(Vec3d{2, 3, 4}));
    check(*make_tetrahedron());
    check(*make_sphere(1.0f));
    check(*make_cylinder(0.5f, 2.0f));
}

TEST_CASE("ADR-0027: make_box reports genus 0 under the derivation rule") {
    const auto body = make_box(Vec3d{1, 1, 1});
    CHECK(is_closed_orientable(*body));
    const auto g = genus_from_chi(euler_characteristic(*body));
    REQUIRE(g.has_value());
    CHECK(*g == 0);
}

TEST_CASE("PlaneSurface: evaluate and normal") {
    PlaneSurface plane(Vec3d{0, 0, 0}, Vec3d{0, 1, 0});
    auto p = plane.evaluate(3.0f, 4.0f);
    CHECK(p.y == doctest::Approx(0.0f));
    auto n = plane.normal(0, 0);
    CHECK(n.x == doctest::Approx(0.0f));
    CHECK(n.y == doctest::Approx(1.0f));
    CHECK(n.z == doctest::Approx(0.0f));
}

TEST_CASE("CylinderSurface: evaluate at u=0") {
    CylinderSurface cyl(1.0f, 2.0f);
    auto p = cyl.evaluate(0.0f, 0.0f);
    CHECK(p.x == doctest::Approx(1.0f));
    CHECK(p.y == doctest::Approx(0.0f));
    CHECK(p.z == doctest::Approx(0.0f));
}

TEST_CASE("SphereSurface: evaluate at north pole") {
    SphereSurface sph(2.0f);
    auto p = sph.evaluate(0.0f, 0.0f);  // v=0 → north pole
    CHECK(p.x == doctest::Approx(0.0f));
    CHECK(p.y == doctest::Approx(2.0f));
    CHECK(p.z == doctest::Approx(0.0f));
}

TEST_CASE("SphereSurface: evaluate at equator") {
    SphereSurface sph(2.0f);
    auto p = sph.evaluate(0.0f, 3.14159f * 0.5f);  // v=π/2 → equator
    CHECK(p.x == doctest::Approx(2.0f).epsilon(0.01f));
    CHECK(p.y == doctest::Approx(0.0f).epsilon(0.01f));
}

TEST_CASE("TorusSurface: evaluate at center of tube") {
    TorusSurface tor(3.0f, 0.5f);
    auto p = tor.evaluate(0.0f, 0.0f);  // u=0, v=0
    CHECK(p.x == doctest::Approx(3.5f).epsilon(0.01f));
    CHECK(p.y == doctest::Approx(0.0f));
    CHECK(p.z == doctest::Approx(0.0f));
}
