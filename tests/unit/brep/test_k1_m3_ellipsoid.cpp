// tests/unit/brep/test_k1_m3_ellipsoid.cpp
//
// K1 m3 — LA SONDA: THE FIRST FAMILY ENTERS UNDER THE PROTOCOL
// (ADR-0042, the probe's derivation). This file lands the WRITER's
// golden: the frame rule pointwise (the guard that FAILS on the
// left-handed authoring), the evaluate pins, the body's own counts
// (the sphere's topology one family over), the names, the seam-wired
// slit rectangle, the class's honest refusals, translate_body's
// re-anchoring — every number quoted from the ADR's derivation,
// never re-derived differently.
//
// THE GUIDE CASE (ADR-0042 D3): a = 0.75, c = 0.5 — V = 0.375·π =
// 1.1780972450961724, I_z = 0.2650718801466388, I_x = I_y =
// 0.1914408023281280 (the mass cases, landed with the mass family's
// own act below).
#include <doctest/doctest.h>

#include "CAD_0/brep/analytic.hpp"
#include "CAD_0/brep/factory.hpp"
#include "CAD_0/brep/query.hpp"
#include "CAD_0/brep/subtract.hpp"
#include "CAD_0/brep/tessellate.hpp"
#include "CAD_0/brep/topology.hpp"

#include <cmath>
#include <limits>

using namespace CAD_0::brep;
using namespace CAD_0::math;
using namespace CAD_0::sdf;
using CAD_0::geometry::FeatureId;

namespace {

// The guide case's semiaxes (ADR-0042 D3's own pin).
constexpr double kA = 0.75;
constexpr double kC = 0.5;
// CAD_0::brep::kPi / CAD_0::brep::kTwoPi: pcurve.hpp's own constants, in scope via the using
// declarations — never re-declared (the ambiguity this comment
// replaces was the lesson).

// The m3 frame-rule lambda, verbatim one family over: (du x dv) . n
// sampled pointwise — the guard that FAILS on the left-handed
// authoring (the discriminating pin, the chirality column's own
// evidence).
const auto frame_rule = [](const auto& surf, double u0, double u1,
                           double v0, double v1) {
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

} // namespace

// ===========================================================================
// THE FRAME RULE POINTWISE — the chirality pin (ADR-0042 D2)
// ===========================================================================
TEST_CASE("K1 m3 the frame rule pointwise: EllipsoidSurface "
          "right-handed over the full window and the bands, off the "
          "poles (the guard)") {
    SUBCASE("the default frame (+Z pole), the full window") {
        const EllipsoidSurface e(kA, kC);
        // v in (0, pi) strictly: the poles are the parametrization's
        // degenerate points (sin v = 0, du vanishes — the sphere's
        // own pole discipline, the cone's apex rule one family over).
        frame_rule(e, 0.0, CAD_0::brep::kTwoPi, 0.05, CAD_0::brep::kPi - 0.05);
    }
    SUBCASE("the default frame, a band window (the barrel's own)") {
        const EllipsoidSurface e(kA, kC);
        frame_rule(e, 0.3, 2.2, 0.4, 2.1);
    }
    SUBCASE("the explicit frame (+Y pole, +X azimuth — the sphere's "
            "historical frame)") {
        const EllipsoidSurface e(kA, kC, {1, 2, 3}, {0, 1, 0}, {1, 0, 0});
        frame_rule(e, 0.0, CAD_0::brep::kTwoPi, 0.05, CAD_0::brep::kPi - 0.05);
    }
    SUBCASE("an oblate body (a > c) and a prolate one (a < c)") {
        const EllipsoidSurface oblate(1.0, 0.25);
        const EllipsoidSurface prolate(0.25, 1.0);
        frame_rule(oblate, 0.0, CAD_0::brep::kTwoPi, 0.05, CAD_0::brep::kPi - 0.05);
        frame_rule(prolate, 0.0, CAD_0::brep::kTwoPi, 0.05, CAD_0::brep::kPi - 0.05);
    }
}

// ===========================================================================
// THE EVALUATE PINS — the surviving geometry, the seam meridian and
// the equator (the ADR's own numbers)
// ===========================================================================
TEST_CASE("K1 m3 the evaluate pins: the poles, the equator, the seam "
          "meridian u = 0, the normal — ADR-0042 D1's form") {
    const EllipsoidSurface e(kA, kC, {1, 2, 3});

    // The poles: v = 0 the north (C + c·n̂), v = pi the south.
    auto p = e.evaluate(0.0, 0.0);
    CHECK(p.x == doctest::Approx(1.0));
    CHECK(p.y == doctest::Approx(2.0));
    CHECK(p.z == doctest::Approx(3.0 + kC));
    p = e.evaluate(0.7, CAD_0::brep::kPi);
    CHECK(p.z == doctest::Approx(3.0 - kC));
    CHECK(p.x == doctest::Approx(1.0));
    CHECK(p.y == doctest::Approx(2.0));

    // The equator v = pi/2: the u-circle of radius a at quota C_z.
    p = e.evaluate(0.0, CAD_0::brep::kPi / 2.0);
    CHECK(p.x == doctest::Approx(1.0 + kA));
    CHECK(p.y == doctest::Approx(2.0));
    CHECK(p.z == doctest::Approx(3.0));
    p = e.evaluate(CAD_0::brep::kPi / 2.0, CAD_0::brep::kPi / 2.0);
    // e1 = e0 x pole = X x Z = -Y: u = pi/2 rides the -Y half-plane.
    CHECK(p.x == doctest::Approx(1.0));
    CHECK(p.y == doctest::Approx(2.0 - kA));
    CHECK(p.z == doctest::Approx(3.0));

    // The seam meridian u = 0: the +X half-plane, z = c·cos v.
    p = e.evaluate(0.0, CAD_0::brep::kPi / 3.0);
    const double s = std::sin(CAD_0::brep::kPi / 3.0);
    CHECK(p.x == doctest::Approx(1.0 + kA * s));
    CHECK(p.y == doctest::Approx(2.0));
    CHECK(p.z == doctest::Approx(3.0 + kC * 0.5));

    // The normal: the outward gradient (x/a^2, y/a^2, z/c^2)
    // normalized — the equator's and the pole's own.
    auto n = e.normal(0.0, CAD_0::brep::kPi / 2.0);
    CHECK(n.x == doctest::Approx(1.0));
    CHECK(n.y == doctest::Approx(0.0).epsilon(0.0001));
    CHECK(n.z == doctest::Approx(0.0).epsilon(0.0001));
    n = e.normal(0.0, 0.0);  // the north pole: the pole direction
    CHECK(n.x == doctest::Approx(0.0).epsilon(0.0001));
    CHECK(n.y == doctest::Approx(0.0).epsilon(0.0001));
    CHECK(n.z == doctest::Approx(1.0));
    // The general point: the gradient's direction, verified against
    // the surface's own equation (the unnormalized gradient is
    // (sin v cos u / a, sin v sin u / a, cos v / c) in the frame).
    const double u = 0.9, v = 1.1;
    const Vec3d g{std::sin(v) * std::cos(u) / kA,
                  -std::sin(v) * std::sin(u) / kA,
                  std::cos(v) / kC};
    n = e.normal(u, v);
    CHECK(n.x == doctest::Approx(g.x / g.length()).epsilon(0.0001));
    CHECK(n.y == doctest::Approx(g.y / g.length()).epsilon(0.0001));
    CHECK(n.z == doctest::Approx(g.z / g.length()).epsilon(0.0001));

    // The accessors: the frame readable, the family the enum.
    CHECK(e.family() == SurfaceFamily::Ellipsoid);
    CHECK(e.equatorial_semiaxis() == doctest::Approx(kA));
    CHECK(e.polar_semiaxis() == doctest::Approx(kC));
    CHECK(e.pole().z == doctest::Approx(1.0));
    CHECK(e.azimuth().x == doctest::Approx(1.0));
}

// ===========================================================================
// THE CHIRALITY COUNTERPROOF — the naive left-handed authoring, the
// guard RED on it (the pin that guards the frame forever)
// ===========================================================================
TEST_CASE("K1 m3 the counterproof: the naive u = +azimuth authoring "
          "is LEFT-handed — the frame rule RED where the class is "
          "green (the discriminating pin)") {
    // THE NAIVE AUTHORING: e1 = -(e0 x pole) — the sphere's
    // pre-correction shape (the historical left-handed cylinder's
    // own mistake, one family over). It is NOT the landed class (a
    // left-handed spheroid is unrepresentable in it — no knob); this
    // local authoring exists so the guard's discrimination is pinned:
    // every sample of the rule reads the cross product NEGATED.
    struct NaiveLeftHanded {
        double a, c;
        Vec3d evaluate(double u, double v) const {
            const double sv = std::sin(v);
            // +Y rides +sin u (the flipped e1) — the left-handed form.
            return {a * sv * std::cos(u), a * sv * std::sin(u),
                    c * std::cos(v)};
        }
        Vec3d normal(double u, double v) const {
            const double sv = std::sin(v);
            const Vec3d g{sv * std::cos(u) / a, sv * std::sin(u) / a,
                          std::cos(v) / c};
            return g.normalized();
        }
    };
    const NaiveLeftHanded naive{kA, kC};
    int red = 0, total = 0;
    for (int i = 0; i <= 12; ++i) {
        const double u = CAD_0::brep::kTwoPi * i / 12.0;
        for (int j = 0; j <= 12; ++j) {
            const double v = 0.05 + (CAD_0::brep::kPi - 0.1) * j / 12.0;
            const double h = 1e-7;
            const Vec3d pu = naive.evaluate(u, v);
            const Vec3d du = (naive.evaluate(u + h, v) - pu) / h;
            const Vec3d dv = (naive.evaluate(u, v + h) - pu) / h;
            ++total;
            if (du.cross(dv).dot(naive.normal(u, v)) <= 0.0) ++red;
        }
    }
    // The naive authoring fails the rule at EVERY sample off the
    // poles — the guard's own discrimination, the m3 pattern's
    // cleanest case (the landed class has no such authoring to make).
    CHECK(red == total);
    CHECK(total == 169);
    // And the landed class passes the same grid (the pair read
    // together: the red is the naive's alone).
    const EllipsoidSurface e(kA, kC);
    int green = 0;
    for (int i = 0; i <= 12; ++i) {
        const double u = CAD_0::brep::kTwoPi * i / 12.0;
        for (int j = 0; j <= 12; ++j) {
            const double v = 0.05 + (CAD_0::brep::kPi - 0.1) * j / 12.0;
            const double h = 1e-7;
            const Vec3d pu = e.evaluate(u, v);
            const Vec3d du = (e.evaluate(u + h, v) - pu) / h;
            const Vec3d dv = (e.evaluate(u, v + h) - pu) / h;
            if (du.cross(dv).dot(e.normal(u, v)) > 0.0) ++green;
        }
    }
    CHECK(green == total);
}

// ===========================================================================
// THE CLASS'S HONEST REFUSALS — the semiaxis class, the skewed frame
// ===========================================================================
TEST_CASE("K1 m3 the refusals: the semiaxis class and the skewed "
          "frame, loud, with the remedy (the ADR-0029 family)") {
    REQUIRE_THROWS_AS(
        EllipsoidSurface(0.0, kC), std::invalid_argument);
    REQUIRE_THROWS_AS(
        EllipsoidSurface(kA, -1.0), std::invalid_argument);
    REQUIRE_THROWS_AS(
        EllipsoidSurface(std::numeric_limits<double>::quiet_NaN(), kC),
        std::invalid_argument);
    REQUIRE_THROWS_AS(
        EllipsoidSurface(kA, std::numeric_limits<double>::infinity()),
        std::invalid_argument);
    // The skewed frame: the azimuth NOT perpendicular to the pole —
    // no spheroid parametrization in the span ((1,0,1)·(0,0,1) = 1).
    REQUIRE_THROWS_AS(
        EllipsoidSurface(kA, kC, {0, 0, 0}, {0, 0, 1}, {1, 0, 1}),
        std::invalid_argument);
    // The message carries the diagnosis (the query_face culture).
    bool caught = false;
    try {
        EllipsoidSurface(0.0, kC);
    } catch (const std::invalid_argument& ex) {
        caught = true;
        const std::string m = ex.what();
        CHECK(m.find("semiaxes") != std::string::npos);
        CHECK(m.find("Remedy") != std::string::npos);
    }
    CHECK(caught);
}

// ===========================================================================
// THE WRITER — make_ellipsoid: the sphere's own topology one family
// over (2V/1E/1F, chi = 2), the seam-wired slit rectangle, the names
// ===========================================================================
TEST_CASE("K1 m3 the writer: make_ellipsoid's counts, census, names "
          "and provenance — the sphere's own numbers one family over") {
    const auto body = make_ellipsoid(kA, kC, {0, 0, 0}, FeatureId{9});
    REQUIRE(body != nullptr);

    // The counts (the ADR's own pins): 2/1/1, chi = 2, genus 0.
    const BodySummary q = query_body(*body);
    CHECK(q.vertex_count == 2);
    CHECK(q.edge_count == 1);
    CHECK(q.face_count == 1);
    CHECK(q.inner_loops == 0);
    CHECK(q.chi == 2);
    CHECK(q.orientable);
    CHECK(q.loops_closed);
    CHECK(q.genus_derived);
    CHECK(q.genus == 0);

    // The names (ADR-0036 D1's stamp, the sphere's vocabulary one
    // family over).
    CHECK(query_face(*body, 0).name == "elp#9:ellipsoid");
    CHECK(query_edge(*body, 0).name == "elp#9:seam");
    CHECK(query_face(*body, 0).family == "ellipsoid");

    // The provenance: exact/0 (the factory's Layer-2 contract).
    const FaceInfo fq = query_face(*body, 0);
    CHECK(fq.exact);
    CHECK(fq.linear_tolerance == 0.0);

    // The seam pair: the single loop carries the seam edge TWICE —
    // forward the RIGHT edge of the slit rectangle, reversed the LEFT
    // — opposite orientations (the radial-edge XOR's own demand), the
    // p-curves the two vertical sides of [0,2pi]x[0,pi].
    const Face& f = body->solid.shell().faces()[0];
    REQUIRE(f.loops().size() == 1);
    const auto& coedges = f.loops()[0].coedges();
    REQUIRE(coedges.size() == 2);
    CHECK(coedges[0].edge() == 0);
    CHECK(coedges[1].edge() == 0);
    CHECK(coedges[0].reversed() != coedges[1].reversed());
    REQUIRE(coedges[0].p_curve() != nullptr);
    REQUIRE(coedges[1].p_curve() != nullptr);
    CHECK(coedges[0].p_curve()->kind == PCurve::Kind::Segment);
    CHECK(coedges[1].p_curve()->kind == PCurve::Kind::Segment);
    // seam fwd: (2pi, 0) -> (2pi, pi), the RIGHT edge.
    CHECK(coedges[0].p_curve()->segment.a.x == doctest::Approx(CAD_0::brep::kTwoPi));
    CHECK(coedges[0].p_curve()->segment.a.y == doctest::Approx(0.0));
    CHECK(coedges[0].p_curve()->segment.b.x == doctest::Approx(CAD_0::brep::kTwoPi));
    CHECK(coedges[0].p_curve()->segment.b.y == doctest::Approx(CAD_0::brep::kPi));
    CHECK(coedges[0].p_curve()->segment.interior_left);
    // seam rev: (0, pi) -> (0, 0), the LEFT edge.
    CHECK(coedges[1].p_curve()->segment.a.x == doctest::Approx(0.0));
    CHECK(coedges[1].p_curve()->segment.a.y == doctest::Approx(CAD_0::brep::kPi));
    CHECK(coedges[1].p_curve()->segment.b.x == doctest::Approx(0.0));
    CHECK(coedges[1].p_curve()->segment.b.y == doctest::Approx(0.0));
    CHECK(coedges[1].p_curve()->segment.interior_left);

    // The radial census: every edge exactly two opposite co-edges
    // (the seam's pair, both on the one face — the sphere's own
    // closed manifold).
    std::size_t coedge_total = 0;
    for (const auto& loop : f.loops())
        coedge_total += loop.coedges().size();
    CHECK(coedge_total == 2);
    CHECK(query_body(*body).edge_count * 2 == coedge_total);

    // The vertices are the poles (the default +Z frame).
    CHECK(body->vertices[0].position().z == doctest::Approx(kC));
    CHECK(body->vertices[1].position().z == doctest::Approx(-kC));
}

// ===========================================================================
// THE WRITER'S COUNTERPROOF — the seam-skip authoring, the closed-
// orientable contract REFUSES (the red where the golden is green)
// ===========================================================================
TEST_CASE("K1 m3 the counterproof: the seam-skip body — the loop "
          "without the reversed copy fails the closed contract") {
    // THE NAIVE AUTHORING: the face's loop carries the seam edge
    // ONCE (forward only). The 3D walk never returns (the domain
    // curve ends at (2pi, pi) with nothing home), the co-edge census
    // reads ONE co-edge on the seam (not two opposite), and the
    // closed-orientable contract — the divergence theorem's own gate
    // — refuses the body: exactly what make_ellipsoid's seam pair
    // exists to prevent.
    auto body = std::make_unique<BRepBody>();
    auto& shell = body->solid.shell_mut();
    shell.set_closed(true);
    body->vertices = {Vertex{{0, 0, kC}}, Vertex{{0, 0, -kC}}};
    body->vertices[0].set_id(0);
    body->vertices[1].set_id(1);
    body->edges = {Edge{0, 1}};
    body->edges[0].set_id(0);
    Face face(std::make_shared<EllipsoidSurface>(kA, kC));
    Loop loop;
    CoEdge seam_fwd{0, false};
    seam_fwd.set_p_curve(pcurve_segment({CAD_0::brep::kTwoPi, 0.0}, {CAD_0::brep::kTwoPi, CAD_0::brep::kPi},
                                        true));
    loop.add_coedge(seam_fwd);  // the reversed copy SKIPPED — naive
    face.add_loop(std::move(loop));
    face.set_id(0);
    shell.add_face(std::move(face));
    link_radial_edges(*body);

    const BodySummary q = query_body(*body);
    CHECK_FALSE(q.orientable);  // the red: the walk does not close
    CHECK_FALSE(q.loops_closed);
}

// ===========================================================================
// TRANSLATE_BODY — the ellipsoid's own case: the mathematical object
// moves, the INTRINSIC frame rides verbatim
// ===========================================================================
TEST_CASE("K1 m3 translate_body: the ellipsoid re-anchors through its "
          "own accessors, the frame carried like the sphere's") {
    const auto body = make_ellipsoid(kA, kC, {0, 0, 0}, FeatureId{4});
    translate_body(*body, {1, 2, 3});
    const Face& f = body->solid.shell().faces()[0];
    struct Reader : SurfaceVisitor {
        const EllipsoidSurface* e{nullptr};
        void visit(const EllipsoidSurface& s) override { e = &s; }
    } r;
    f.surface()->accept(r);
    REQUIRE(r.e != nullptr);
    CHECK(r.e->origin().x == doctest::Approx(1.0));
    CHECK(r.e->origin().y == doctest::Approx(2.0));
    CHECK(r.e->origin().z == doctest::Approx(3.0));
    // The intrinsic frame rides verbatim (the +Z pole, +X azimuth).
    CHECK(r.e->pole().z == doctest::Approx(1.0));
    CHECK(r.e->azimuth().x == doctest::Approx(1.0));
    CHECK(r.e->equatorial_semiaxis() == doctest::Approx(kA));
    CHECK(r.e->polar_semiaxis() == doctest::Approx(kC));
    // The poles moved with the body.
    CHECK(body->vertices[0].position().z ==
          doctest::Approx(3.0 + kC));
    CHECK(body->vertices[1].position().z ==
          doctest::Approx(3.0 - kC));
    // The translated body stays a closed orientable shell.
    CHECK(query_body(*body).orientable);
    CHECK(query_body(*body).chi == 2);
}

// ===========================================================================
// THE TESSELLATION — the trim reads the seam pair's own window, the
// vertices on the surface's own world (the m6bis phantom census,
// one family over)
// ===========================================================================
TEST_CASE("K1 m3 the tessellation: the ellipsoid's display mesh — "
          "trimmed to the face's own window, every vertex inside the "
          "body's own world") {
    const auto body = make_ellipsoid(kA, kC, {0, 0, 0});
    const CAD_0::sdf::TriangleMesh mesh =
        tessellate_body(*body, TessellationOptions{});
    CHECK(mesh.positions.size() > 100);
    CHECK(mesh.indices.size() > 300);
    // The phantom census (m6-correttiva-bis's own pin, one family
    // over): no display vertex beyond the body's own world — the
    // ellipsoid's box is [-a,a]x[-a,a]x[-c,c], one cell of slack for
    // the float boundary.
    for (const auto& v : mesh.positions) {
        CHECK(v.x <= kA + 0.05f);
        CHECK(v.x >= -kA - 0.05f);
        CHECK(v.y <= kA + 0.05f);
        CHECK(v.y >= -kA - 0.05f);
        CHECK(v.z <= kC + 0.05f);
        CHECK(v.z >= -kC - 0.05f);
    }
    // The surface honored: every mesh vertex satisfies the spheroid's
    // equation to the float boundary's own tolerance.
    for (const auto& v : mesh.positions) {
        const double q = (double(v.x) * v.x) / (kA * kA) +
                         (double(v.y) * v.y) / (kA * kA) +
                         (double(v.z) * v.z) / (kC * kC);
        CHECK(q == doctest::Approx(1.0).epsilon(0.01));
    }
}
