// tests/unit/brep/test_analytic_surfaces.cpp
//
// Approfondimento: test estesi per superfici e curve analitiche.
//
#include <doctest/doctest.h>

#include "CAD_0/brep/analytic.hpp"
#include "CAD_0/brep/curve.hpp"
#include "CAD_0/brep/factory.hpp"
#include "CAD_0/brep/tessellate.hpp"

#include <cmath>

using namespace CAD_0::brep;
using namespace CAD_0::math;
using namespace CAD_0::sdf;

// ---------------------------------------------------------------------------
// PlaneSurface — test approfonditi
// ---------------------------------------------------------------------------
TEST_CASE("PlaneSurface: normal is always the same regardless of (u,v)") {
    PlaneSurface plane(Vec3d{1, 2, 3}, Vec3d{0, 0, 1});
    const auto n1 = plane.normal(0, 0);
    const auto n2 = plane.normal(5, -3);
    CHECK(n1.x == doctest::Approx(n2.x));
    CHECK(n1.y == doctest::Approx(n2.y));
    CHECK(n1.z == doctest::Approx(n2.z));
}

TEST_CASE("PlaneSurface: evaluate at origin") {
    PlaneSurface plane(Vec3d{1, 2, 3}, Vec3d{0, 1, 0});
    auto p = plane.evaluate(0, 0);
    CHECK(p.x == doctest::Approx(1.0f));
    CHECK(p.y == doctest::Approx(2.0f));
    CHECK(p.z == doctest::Approx(3.0f));
}

TEST_CASE("PlaneSurface: evaluate is linear in u and v") {
    PlaneSurface plane(Vec3d{0, 0, 0}, Vec3d{0, 1, 0});
    auto p0 = plane.evaluate(0, 0);
    auto p1 = plane.evaluate(1, 0);
    auto p2 = plane.evaluate(2, 0);
    // Check that u is linear: p2 - p1 == p1 - p0
    auto d1 = p1 - p0;
    auto d2 = p2 - p1;
    CHECK(d1.x == doctest::Approx(d2.x));
    CHECK(d1.y == doctest::Approx(d2.y));
    CHECK(d1.z == doctest::Approx(d2.z));
}

TEST_CASE("PlaneSurface: normal is unit length") {
    PlaneSurface plane(Vec3d{0, 0, 0}, Vec3d{1, 1, 1});
    auto n = plane.normal(0, 0);
    CHECK(n.length() == doctest::Approx(1.0f).epsilon(0.001f));
}

TEST_CASE("PlaneSurface: bounds is universe") {
    PlaneSurface plane(Vec3d{0, 0, 0}, Vec3d{0, 1, 0});
    auto b = plane.bounds();
    CHECK(b.empty() == false);
}

// ---------------------------------------------------------------------------
// CylinderSurface — test approfonditi
// ---------------------------------------------------------------------------
TEST_CASE("CylinderSurface: evaluate at multiple angles") {
    CylinderSurface cyl(1.0f, 2.0f);

    // G1 unit 2 corrected the frame's HANDEDNESS (see the next test):
    // evaluate(u,v) = (R cos u, v, −R sin u) — right-handed around the
    // OUTWARD normal, like every analytic surface. u=0 stays the seam
    // meridian (+X); u=π/2 now reads −Z where it read +Z before.
    // u=0 → punto su +X
    auto p0 = cyl.evaluate(0.0f, 0.0f);
    CHECK(p0.x == doctest::Approx(1.0f));
    CHECK(p0.z == doctest::Approx(0.0f));

    // u=π/2 → punto su -Z (corrected handedness)
    auto p1 = cyl.evaluate(3.14159f * 0.5f, 0.0f);
    CHECK(p1.x == doctest::Approx(0.0f).epsilon(0.001f));
    CHECK(p1.z == doctest::Approx(-1.0f).epsilon(0.001f));

    // u=π → punto su -X
    auto p2 = cyl.evaluate(3.14159f, 0.0f);
    CHECK(p2.x == doctest::Approx(-1.0f).epsilon(0.001f));
    CHECK(p2.z == doctest::Approx(0.0f).epsilon(0.001f));

    // u=3π/2 → punto su +Z (corrected handedness)
    auto p3 = cyl.evaluate(3.14159f * 1.5f, 0.0f);
    CHECK(p3.x == doctest::Approx(0.0f).epsilon(0.001f));
    CHECK(p3.z == doctest::Approx(1.0f).epsilon(0.001f));
}

TEST_CASE("CylinderSurface: the (u,v) frame is right-handed around the outward normal (G1 unit 2's convention correction, pinned)") {
    // WHY THIS PIN EXISTS: unit 1 planted the p-curve conventions
    // (material on the left of traversal, in the face's domain);
    // unit 2's first real use discovered the cylinder was the
    // analytic set's LEFT-handed parametrization — its natural frame
    // (∂u×∂v) pointed INWARD, while PlaneSurface and SphereSurface
    // are right-handed around the outward normal. With the caps'
    // and the wall's loops all authored material-on-left, the shared
    // rim edges were walked in the SAME direction by both faces —
    // breaking the radial-edge orientation-XOR. The correction (one
    // rule for every analytic surface) flipped the cylinder's frame;
    // this test is the regression guard: numerically,
    // (∂u × ∂v) · normal > 0, everywhere sampled.
    const auto frame_right_handed = [](const SurfaceGeometry& surf,
                                       float umin, float umax, float vmin,
                                       float vmax) {
        const float eps = 1e-3f;
        for (int i = 0; i <= 8; ++i) {
            for (int j = 0; j <= 4; ++j) {
                const float u = umin + (umax - umin) * float(i) / 8.0f;
                const float v = vmin + (vmax - vmin) * float(j) / 4.0f;
                const auto du = surf.evaluate(u + eps, v) - surf.evaluate(u - eps, v);
                const auto dv = surf.evaluate(u, v + eps) - surf.evaluate(u, v - eps);
                const auto n = du.cross(dv);
                const auto want = surf.normal(u, v);
                // > 0: the frame is right-handed around the outward
                // normal (the corrected convention). Normalized so the
                // check is scale-free.
                CHECK(n.dot(want) > 0.0f);
            }
        }
    };

    CylinderSurface cyl(1.0f, 2.0f);
    frame_right_handed(cyl, 0.0f, 6.2831853f, -0.9f, 0.9f);

    // The already-correct reference: the sphere's frame was
    // right-handed from the start (and the plane's is by
    // construction: u_axis × v_axis = normal).
    SphereSurface sph(2.0f);
    frame_right_handed(sph, 0.0f, 6.2831853f, 0.1f, 3.0415927f);
}

TEST_CASE("CylinderSurface: all surface points at distance R from axis") {
    CylinderSurface cyl(2.5f, 10.0f);
    for (int i = 0; i < 10; ++i) {
        float u = float(i) * 0.628f;
        double v = double(i) - 4.5;
        auto p = cyl.evaluate(u, v);
        double r = std::sqrt(p.x * p.x + p.z * p.z);
        CHECK(r == doctest::Approx(2.5f).epsilon(0.01f));
    }
}

TEST_CASE("CylinderSurface: normal points outward") {
    CylinderSurface cyl(1.0f, 2.0f);
    (void)cyl.evaluate(0.0f, 0.0f);
    auto n = cyl.normal(0.0f, 0.0f);
    // At u=0, point is (1, 0, 0), normal should be (1, 0, 0)
    CHECK(n.x == doctest::Approx(1.0f));
    CHECK(n.y == doctest::Approx(0.0f));
    CHECK(n.z == doctest::Approx(0.0f));
}

TEST_CASE("CylinderSurface: bounds") {
    CylinderSurface cyl(3.0f, 4.0f);
    auto b = cyl.bounds();
    CHECK(b.min.x == doctest::Approx(-3.0f));
    CHECK(b.max.x == doctest::Approx(3.0f));
    CHECK(b.min.y == doctest::Approx(-2.0f));
    CHECK(b.max.y == doctest::Approx(2.0f));
}

// ---------------------------------------------------------------------------
// SphereSurface — test approfonditi
// ---------------------------------------------------------------------------
TEST_CASE("SphereSurface: all points at distance R from center") {
    SphereSurface sph(3.0f);
    for (int i = 0; i < 10; ++i) {
        float u = float(i) * 0.628f;
        float v = float(i) * 0.314f;
        auto p = sph.evaluate(u, v);
        double r = p.length();
        CHECK(r == doctest::Approx(3.0f).epsilon(0.01f));
    }
}

TEST_CASE("SphereSurface: normal points outward") {
    SphereSurface sph(2.0f);
    (void)sph.evaluate(0.0f, 3.14159f * 0.5f);  // equator at u=0
    auto n = sph.normal(0.0f, 3.14159f * 0.5f);
    // At equator u=0, point is (R, 0, 0), normal should be (1, 0, 0)
    CHECK(n.x == doctest::Approx(1.0f));
    CHECK(n.y == doctest::Approx(0.0f));
    CHECK(n.z == doctest::Approx(0.0f));
}

TEST_CASE("SphereSurface: north and south pole") {
    SphereSurface sph(5.0f);
    

    auto north = sph.evaluate(0.0f, 0.0f);  // v=0 → north
    CHECK(north.x == doctest::Approx(0.0f).epsilon(0.01f));
    CHECK(north.y == doctest::Approx(5.0f));
    CHECK(north.z == doctest::Approx(0.0f).epsilon(0.01f));

    auto south = sph.evaluate(0.0f, 3.14159f);  // v=π → south
    CHECK(south.x == doctest::Approx(0.0f).epsilon(0.01f));
    CHECK(south.y == doctest::Approx(-5.0f).epsilon(0.01f));
    CHECK(south.z == doctest::Approx(0.0f).epsilon(0.01f));
}

TEST_CASE("SphereSurface: bounds with center offset") {
    SphereSurface sph(2.0f, Vec3d{1, 2, 3});
    auto b = sph.bounds();
    CHECK(b.min.x == doctest::Approx(-1.0f));
    CHECK(b.max.x == doctest::Approx(3.0f));
    CHECK(b.min.y == doctest::Approx(0.0f));
    CHECK(b.max.y == doctest::Approx(4.0f));
}

// ---------------------------------------------------------------------------
// ConeSurface — test approfonditi
// ---------------------------------------------------------------------------
TEST_CASE("ConeSurface: base radius correct at v=0") {
    ConeSurface cone(2.0f, 4.0f);
    auto p = cone.evaluate(0.0f, 0.0f);  // v=0 → base
    CHECK(p.x == doctest::Approx(2.0f));
    CHECK(p.y == doctest::Approx(0.0f));
    CHECK(p.z == doctest::Approx(0.0f));
}

TEST_CASE("ConeSurface: apex at v=height") {
    ConeSurface cone(2.0f, 4.0f);
    auto p = cone.evaluate(0.0f, 4.0f);  // v=h → apex
    CHECK(p.x == doctest::Approx(0.0f).epsilon(0.01f));
    CHECK(p.y == doctest::Approx(4.0f));
    CHECK(p.z == doctest::Approx(0.0f).epsilon(0.01f));
}

TEST_CASE("ConeSurface: radius decreases linearly with v") {
    ConeSurface cone(4.0f, 8.0f);
    auto p0 = cone.evaluate(0.0f, 0.0f);  // v=0, r=4
    auto p4 = cone.evaluate(0.0f, 4.0f);  // v=4, r should be 2
    double r0 = std::sqrt(p0.x * p0.x + p0.z * p0.z);
    double r4 = std::sqrt(p4.x * p4.x + p4.z * p4.z);
    CHECK(r0 == doctest::Approx(4.0f));
    CHECK(r4 == doctest::Approx(2.0f).epsilon(0.01f));
}

// ---------------------------------------------------------------------------
// TorusSurface — test approfonditi
// ---------------------------------------------------------------------------
TEST_CASE("TorusSurface: point on outer equator") {
    TorusSurface tor(3.0f, 0.5f);
    auto p = tor.evaluate(0.0f, 0.0f);  // outermost point
    double r = std::sqrt(p.x * p.x + p.z * p.z);
    CHECK(r == doctest::Approx(3.5f));
}

TEST_CASE("TorusSurface: point on inner equator") {
    TorusSurface tor(3.0f, 0.5f);
    
    auto p = tor.evaluate(0.0f, 3.14159f);  // innermost point
    double r = std::sqrt(p.x * p.x + p.z * p.z);
    CHECK(r == doctest::Approx(2.5f).epsilon(0.01f));
}

TEST_CASE("TorusSurface: point on top of tube") {
    TorusSurface tor(3.0f, 0.5f);
    auto p = tor.evaluate(0.0f, 3.14159f * 0.5f);  // top of tube
    CHECK(p.x == doctest::Approx(3.0f).epsilon(0.01f));
    CHECK(p.y == doctest::Approx(0.5f));
}

TEST_CASE("TorusSurface: all points at distance r from ring center") {
    TorusSurface tor(3.0f, 0.7f);
    for (int i = 0; i < 8; ++i) {
        float u = float(i) * 0.785f;
        float v = float(i) * 0.785f;
        auto p = tor.evaluate(u, v);
        // Distance from ring center (in XZ plane)
        double ring_r = std::sqrt(p.x * p.x + p.z * p.z);
        // Distance from tube center
        double tube_d = std::sqrt((ring_r - 3.0) * (ring_r - 3.0) + p.y * p.y);
        CHECK(tube_d == doctest::Approx(0.7f).epsilon(0.01f));
    }
}

// ---------------------------------------------------------------------------
// Curve tests
// ---------------------------------------------------------------------------
TEST_CASE("LineCurve: evaluate at t=0 returns origin") {
    LineCurve line(Vec3d{1, 2, 3}, Vec3d{0, 0, 1});
    auto p = line.evaluate(0.0f);
    CHECK(p.x == doctest::Approx(1.0f));
    CHECK(p.y == doctest::Approx(2.0f));
    CHECK(p.z == doctest::Approx(3.0f));
}

TEST_CASE("LineCurve: tangent is constant") {
    LineCurve line(Vec3d{0, 0, 0}, Vec3d{1, 1, 1});
    auto t0 = line.tangent(0.0f);
    auto t1 = line.tangent(10.0f);
    CHECK(t0.x == doctest::Approx(t1.x));
    CHECK(t0.y == doctest::Approx(t1.y));
    CHECK(t0.z == doctest::Approx(t1.z));
}

TEST_CASE("CircleCurve: evaluate at t=0") {
    CircleCurve circle(2.0f);
    auto p = circle.evaluate(0.0f);
    CHECK(p.x == doctest::Approx(2.0f));
    CHECK(p.y == doctest::Approx(0.0f));
    CHECK(p.z == doctest::Approx(0.0f));
}

TEST_CASE("CircleCurve: evaluate at t=π/2") {
    CircleCurve circle(2.0f);
    
    auto p = circle.evaluate(3.14159f * 0.5f);
    CHECK(p.x == doctest::Approx(0.0f).epsilon(0.01f));
    CHECK(p.z == doctest::Approx(2.0f));
}

TEST_CASE("CircleCurve: arc length = 2πR") {
    CircleCurve circle(3.0f);
    double len = circle.length();
    CHECK(len == doctest::Approx(18.849f).epsilon(0.01f));
}

TEST_CASE("EllipseCurve: evaluate at t=0") {
    EllipseCurve ellipse(3.0f, 1.0f);
    auto p = ellipse.evaluate(0.0f);
    CHECK(p.x == doctest::Approx(3.0f));
    CHECK(p.z == doctest::Approx(0.0f));
}

TEST_CASE("EllipseCurve: evaluate at t=π/2") {
    EllipseCurve ellipse(3.0f, 1.0f);
    
    auto p = ellipse.evaluate(3.14159f * 0.5f);
    CHECK(p.x == doctest::Approx(0.0f).epsilon(0.01f));
    CHECK(p.z == doctest::Approx(1.0f));
}

// ---------------------------------------------------------------------------
// Factory tests — struttura completa
// ---------------------------------------------------------------------------
TEST_CASE("Factory: box has correct vertex positions") {
    auto body = make_box(Vec3d{1, 2, 3});
    // Vertex 0: (-1, -2, -3)
    CHECK(body->vertices[0].position().x == doctest::Approx(-1.0f));
    CHECK(body->vertices[0].position().y == doctest::Approx(-2.0f));
    CHECK(body->vertices[0].position().z == doctest::Approx(-3.0f));
    // Vertex 6: (+1, +2, +3)
    CHECK(body->vertices[6].position().x == doctest::Approx(1.0f));
    CHECK(body->vertices[6].position().y == doctest::Approx(2.0f));
    CHECK(body->vertices[6].position().z == doctest::Approx(3.0f));
}

TEST_CASE("Factory: box has correct edge connectivity") {
    auto body = make_box(Vec3d{1, 1, 1});
    // Edge 0: vertices 0→1
    CHECK(body->edges[0].vertex(0) == 0);
    CHECK(body->edges[0].vertex(1) == 1);
    // Edge 8: vertices 0→4 (vertical)
    CHECK(body->edges[8].vertex(0) == 0);
    CHECK(body->edges[8].vertex(1) == 4);
}

TEST_CASE("Factory: box has 6 plane faces") {
    auto body = make_box(Vec3d{1, 1, 1});
    for (const auto& face : body->solid.shell().faces()) {
        CHECK(std::string(face.surface()->type_name()) == "plane");
    }
    CHECK(body->solid.shell().faces().size() == 6);
}

TEST_CASE("Factory: box shell is closed") {
    auto body = make_box(Vec3d{1, 1, 1});
    CHECK(body->solid.shell().is_closed() == true);
}

TEST_CASE("Factory: sphere has 1 face of type sphere") {
    auto body = make_sphere(2.0f);
    CHECK(body->face_count() == 1);
    CHECK(std::string(body->solid.shell().faces()[0].surface()->type_name()) == "sphere");
}

TEST_CASE("Factory: cylinder has 3 faces") {
    auto body = make_cylinder(1.0f, 4.0f);
    CHECK(body->face_count() == 3);
    // Face 0: cylinder lateral
    CHECK(std::string(body->solid.shell().faces()[0].surface()->type_name()) == "cylinder");
    // Face 1: bottom plane
    CHECK(std::string(body->solid.shell().faces()[1].surface()->type_name()) == "plane");
    // Face 2: top plane
    CHECK(std::string(body->solid.shell().faces()[2].surface()->type_name()) == "plane");
}

TEST_CASE("Factory: box bounds match half-extents") {
    auto body = make_box(Vec3d{2, 3, 4});
    auto b = body->bounds();
    CHECK(b.min.x == doctest::Approx(-2.0f));
    CHECK(b.max.x == doctest::Approx(2.0f));
    CHECK(b.min.y == doctest::Approx(-3.0f));
    CHECK(b.max.y == doctest::Approx(3.0f));
    CHECK(b.min.z == doctest::Approx(-4.0f));
    CHECK(b.max.z == doctest::Approx(4.0f));
}

// ---------------------------------------------------------------------------
// Tessellation tests
// ---------------------------------------------------------------------------
TEST_CASE("Tessellation: box face produces mesh") {
    auto body = make_box(Vec3d{1, 1, 1});
    TessellationOptions opts;
    opts.resolution = 4;
    auto mesh = tessellate_body(*body, opts);
    CHECK(mesh.vertex_count() > 0);
    CHECK(mesh.triangle_count() > 0);
    CHECK(mesh.triangle_count() % 2 == 0);  // grid cells produce pairs
}

TEST_CASE("Tessellation: sphere produces mesh") {
    auto body = make_sphere(2.0f);
    TessellationOptions opts;
    opts.resolution = 8;
    auto mesh = tessellate_body(*body, opts);
    CHECK(mesh.vertex_count() > 0);
    CHECK(mesh.triangle_count() > 0);
    // All vertices should be at distance ~2 from origin
    for (const auto& p : mesh.positions) {
        double r = p.length();
        CHECK(r == doctest::Approx(2.0f).epsilon(0.05f));
    }
}

TEST_CASE("Tessellation: cylinder produces mesh with 3 faces worth of triangles") {
    auto body = make_cylinder(1.0f, 2.0f);
    TessellationOptions opts;
    opts.resolution = 8;
    auto mesh = tessellate_body(*body, opts);
    CHECK(mesh.triangle_count() > 100);  // 3 faces × multiple triangles
}

TEST_CASE("Tessellation: normals are computed when requested") {
    auto body = make_sphere(2.0f);
    TessellationOptions opts;
    opts.resolution = 4;
    opts.compute_normals = true;
    auto mesh = tessellate_body(*body, opts);
    CHECK(!mesh.normals.empty());
    CHECK(mesh.normals.size() == mesh.positions.size());
}

TEST_CASE("Tessellation: normals are empty when not requested") {
    auto body = make_sphere(2.0f);
    TessellationOptions opts;
    opts.resolution = 4;
    opts.compute_normals = false;
    auto mesh = tessellate_body(*body, opts);
    CHECK(mesh.normals.empty());
}

// ---------------------------------------------------------------------------
// Topology validation tests
// ---------------------------------------------------------------------------
TEST_CASE("Topology: edge manifold check") {
    Edge e{0, 1};
    CHECK(e.is_manifold() == false);  // no coedges

    CoEdge ce1{0, false};
    CoEdge ce2{0, true};
    e.add_coedge(&ce1);
    CHECK(e.is_manifold() == false);  // only 1 coedge

    e.add_coedge(&ce2);
    CHECK(e.is_manifold() == true);   // 2 coedges = manifold
}

TEST_CASE("Topology: loop with 4 coedges") {
    Loop loop;
    loop.add_coedge(CoEdge{0, false});
    loop.add_coedge(CoEdge{1, false});
    loop.add_coedge(CoEdge{2, false});
    loop.add_coedge(CoEdge{3, true});
    CHECK(loop.coedges().size() == 4);
    CHECK(loop.coedges()[3].reversed() == true);
}

TEST_CASE("Topology: face forward/reverse") {
    auto surf = std::make_shared<PlaneSurface>(Vec3d{0,0,0}, Vec3d{0,1,0});
    Face face(surf);
    CHECK(face.forward() == true);
    face.set_forward(false);
    CHECK(face.forward() == false);
}

TEST_CASE("Topology: BRepBody bounds from vertices") {
    BRepBody body;
    body.vertices.push_back(Vertex{Vec3d{-1, -2, -3}});
    body.vertices.push_back(Vertex{Vec3d{1, 2, 3}});
    auto b = body.bounds();
    CHECK(b.min.x == doctest::Approx(-1.0f));
    CHECK(b.max.x == doctest::Approx(1.0f));
    CHECK(b.min.y == doctest::Approx(-2.0f));
    CHECK(b.max.y == doctest::Approx(2.0f));
}

// ---------------------------------------------------------------------------
// THE CHIRAL PINS (G2-u3 m3, ADR-0039 D1.2 — the flip's discriminating
// guards): the registered left-handed pair turned right-handed in the
// act that authors the torus's first trimming loop (the fillet's
// toroidal flank). Every guard asserts (du x dv) . n_out > 0
// POINTWISE — the test that FAILS on the left-handed frame:
// the missing flip would read the cross product NEGATED at every
// sample (the winding of any body carrying the face would change
// sign with it — the m3 golden's volume is the body-level pin).
// The surviving evaluate pins above sit on the seam meridian u = 0 or
// on u-symmetric quantities: the flip leaves them untouched, the
// re-pin EMPTY (0035 D5's cleanest case).
// ---------------------------------------------------------------------------
TEST_CASE("The frame rule pointwise: TorusSurface, ConeSurface and "
          "the TorusZ writer — the chiral pins (G2-u3 m3)") {
    const auto frame_rule =
        [](const auto& surf, double u0, double u1, double v0,
           double v1) {
            for (int i = 0; i <= 12; ++i) {
                const double u = u0 + (u1 - u0) * i / 12.0;
                for (int j = 0; j <= 12; ++j) {
                    const double v = v0 + (v1 - v0) * j / 12.0;
                    const double h = 1e-7;
                    const Vec3d pu = surf.evaluate(u, v);
                    const Vec3d du = (surf.evaluate(u + h, v) - pu) / h;
                    const Vec3d dv = (surf.evaluate(u, v + h) - pu) / h;
                    CHECK(du.cross(dv).dot(surf.normal(u, v)) > 0.0);
                }
            }
        };

    SUBCASE("TorusSurface: the one-line flip, right-handed now") {
        TorusSurface tor(3.0, 0.5);  // the surviving pins' own body
        frame_rule(tor, 0.0, 2.0 * 3.14159265358979, 0.0,
                   2.0 * 3.14159265358979);
        // The surviving pins re-asserted VERBATIM (the seam meridian
        // u = 0: the flip's one-liner leaves them untouched).
        auto p = tor.evaluate(0.0f, 0.0f);
        CHECK(std::sqrt(p.x * p.x + p.z * p.z) ==
              doctest::Approx(3.5f));
        p = tor.evaluate(0.0f, 3.14159f);
        CHECK(std::sqrt(p.x * p.x + p.z * p.z) ==
              doctest::Approx(2.5f).epsilon(0.01f));
        p = tor.evaluate(0.0f, 3.14159f * 0.5f);
        CHECK(p.x == doctest::Approx(3.0f).epsilon(0.01f));
        CHECK(p.y == doctest::Approx(0.5f));
    }

    SUBCASE("ConeSurface: riding with the family, right-handed") {
        ConeSurface cone(2.0, 4.0);
        // The sample stays off the APEX (v = h: du vanishes there —
        // the cone's own degenerate point, not a frame question).
        frame_rule(cone, 0.0, 2.0 * 3.14159265358979, 0.1, 3.9);
        // The surviving pins re-asserted VERBATIM (u = 0 again).
        auto p = cone.evaluate(0.0f, 0.0f);
        CHECK(p.x == doctest::Approx(2.0f));
        CHECK(p.z == doctest::Approx(0.0f));
        p = cone.evaluate(0.0f, 4.0f);
        CHECK(p.x == doctest::Approx(0.0f).epsilon(0.01f));
        CHECK(p.y == doctest::Approx(4.0f));
    }

    SUBCASE("TorusZSurface: the writer, born right-handed") {
        TorusZSurface tor(CAD_0::math::Vec2d{0.0, 0.0}, 0.75,
                     0.25, 0.75);
        frame_rule(tor, 0.0, 2.0 * 3.14159265358979,
                   3.14159265358979 / 2.0, 3.14159265358979);
        // The flank's own geometry pins: the contact circles' radii
        // and quotas (the plane contact at v = pi/2: the a+r circle
        // at the rim quota; the wall contact at v = pi: the a circle
        // at depth r), the seam meridian u = 0 at +X.
        auto p = tor.evaluate(0.0, 3.14159265358979 / 2.0);
        CHECK(p.x == doctest::Approx(0.75).epsilon(0.0001));
        CHECK(p.y == doctest::Approx(0.0).epsilon(0.0001));
        CHECK(p.z == doctest::Approx(1.0).epsilon(0.0001));
        p = tor.evaluate(0.0, 3.14159265358979);
        CHECK(p.x == doctest::Approx(0.5).epsilon(0.0001));
        CHECK(p.z == doctest::Approx(0.75).epsilon(0.0001));
        CHECK(tor.family() == SurfaceFamily::Torus);
        CHECK(tor.major_radius() == doctest::Approx(0.75));
        CHECK(tor.minor_radius() == doctest::Approx(0.25));
        CHECK(tor.z_center() == doctest::Approx(0.75));
    }
}
