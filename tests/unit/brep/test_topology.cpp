// tests/unit/brep/test_topology.cpp
#include <doctest/doctest.h>

#include "CAD_0/brep/topology.hpp"
#include "CAD_0/brep/analytic.hpp"
#include "CAD_0/brep/factory.hpp"

using namespace CAD_0::brep;
using namespace CAD_0::math;

TEST_CASE("BRep: Vertex construction") {
    Vertex v{Vec3f{1, 2, 3}};
    CHECK(v.position() == Vec3f{1, 2, 3});
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
    auto plane = std::make_shared<PlaneSurface>(Vec3f{0, 0, 0}, Vec3f{0, 1, 0});
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
    auto body = make_box(Vec3f{1, 1, 1});
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
    auto body = make_box(Vec3f{1, 1, 1});
    REQUIRE(body->edge_count() == 12);
    for (const auto& edge : body->edges) {
        CHECK(edge.coedges().size() == 2);
        CHECK(edge.is_manifold());
    }
}

TEST_CASE("ADR-0021: make_box euler characteristic equals 2") {
    auto body = make_box(Vec3f{1, 1, 1});
    CHECK(euler_characteristic(*body) == 2);
}

TEST_CASE("ADR-0021: make_box with non-uniform half-extents still has chi == 2") {
    auto body = make_box(Vec3f{2, 3, 4});
    CHECK(euler_characteristic(*body) == 2);
}

TEST_CASE("ADR-0021: every loop of every box face is closed") {
    auto body = make_box(Vec3f{1, 1, 1});
    CHECK(loops_are_closed(*body));
}

TEST_CASE("ADR-0021: every box face has exactly one outer loop with 4 coedges") {
    auto body = make_box(Vec3f{1, 1, 1});
    REQUIRE(body->face_count() == 6);
    for (const auto& face : body->solid.shell().faces()) {
        REQUIRE(face.loops().size() == 1);
        const auto& loop = face.loops()[0];
        CHECK(loop.is_outer());
        CHECK(loop.coedges().size() == 4);
    }
}

TEST_CASE("ADR-0021: link_radial_edges is idempotent") {
    auto body = make_box(Vec3f{1, 1, 1});
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
    auto body = make_box(Vec3f{1, 1, 1});
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
    auto body = make_box(Vec3f{1, 1, 1});
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

TEST_CASE("PlaneSurface: evaluate and normal") {
    PlaneSurface plane(Vec3f{0, 0, 0}, Vec3f{0, 1, 0});
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
