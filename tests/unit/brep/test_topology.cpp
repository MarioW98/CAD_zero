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
