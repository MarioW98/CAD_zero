// tests/unit/brep/test_h2_u3_pcurve_double.cpp
//
// H2 unit 3, MORSO 3 — THE P-CURVES IN DOUBLE (ADR-0035 D4, the road
// (a) decision): the BREP parametric domain's own double vocabulary,
// the demotions dead, the sketch frontier declared. This file pins
// the three contracts the order names:
//
//   * THE FRONTIER (D4) — where loops are born from sketch entities
//     (the extrude's caps), the conversion is float→double: EXACT,
//     ONE direction, never silent. The pin: the caps' p-curves carry
//     the sketch's OWN numbers bit-for-bit (every float32 is exactly
//     a double — the equality is exact, not a tolerance).
//
//   * THE COHERENCE CONTRACT 3D↔p-curve — a p-curve is not
//     free-floating domain data: its image under the owning surface
//     IS the 3D trimming curve. Two directions, both pinned:
//       - ENDPOINTS (the linear world, ε_L): the p-curve's evaluated
//         start/end coincide with the co-edge's start/end vertex
//         records within the house's declared linear tolerance
//         (kBoolLinear, ADR-0035 D2.1 — the value the booleans
//         declare, 1e-6 mm);
//       - THE SAMPLED ROUND TRIP (the parameter world, ε_P): sample
//         the p-curve, evaluate the surface, PULL the 3D point back
//         onto the domain (the family's inverse), compare — within
//         the per-family ε_P derived through the family's declared
//         Jacobian (D2.2's named derivation: the plane's is 1, the
//         cylinder's u is r, the sphere's are R·sin v and R).
//     The SSI table's answers get the contract's third reading: the
//     3D intersection curve (SsiCurve) is sampled INDEPENDENTLY of
//     the domain, pulled onto each operand's domain, and every
//     sample must land ON the domain answer's p-curve — the p-curve
//     is the image of the 3D curve, or the table lied.
//
//   * THE DISCRIMINATING GOLDEN — the stress case on the EXTENDED
//     domain that in double holds and in float collapses: the proof
//     the compromise ADR-0035 D4 closed was real. Two exhibits, both
//     at the declared ε_L:
//       - a large-radius cylinder (R = 100): the float rounding of
//         the period constant 2π moves the seam's 3D image by
//         R·δ ≈ 1.75e-5 mm — 17× over ε_L; the double payload's
//         image sits at ~1e-13 mm;
//       - a tall fractional extrude (h = 100000.3): the float
//         payload of the window's top reads 100000.296875 — 3.1e-3
//         mm off the vertex record; the double payload is exact.
//     (Why "extended domain" and not "far from origin": the domain
//     coordinates are FRAME-RELATIVE — a body translated far away
//     keeps O(1) domain numbers, and the honest stress is the
//     domain's own magnitude. The two exhibits stress exactly that:
//     a period constant whose 3D image scales with R, and a window
//     value at the edge of the float grid.)
//
// The per-world type pins live in pcurve.hpp (compile-time, the
// vocabulary's own) and in test_g1_data_model.cpp (the pair: BREP
// double, sketch float, non-identity, the one bridge).
#include <doctest/doctest.h>

#include "CAD_0/brep/topology.hpp"
#include "CAD_0/brep/pcurve.hpp"
#include "CAD_0/brep/factory.hpp"
#include "CAD_0/brep/analytic.hpp"
#include "CAD_0/brep/query.hpp"
#include "CAD_0/brep/ssi.hpp"
#include "CAD_0/brep/extrude.hpp"
#include "CAD_0/brep/boolean.hpp"
#include "CAD_0/sdf/sketch_entities.hpp"
#include "CAD_0/sdf/sketch_profile.hpp"

#include <cmath>
#include <string>
#include <vector>

namespace b = CAD_0::brep;
namespace s = CAD_0::sdf;
using CAD_0::math::Vec2d;
using CAD_0::math::Vec2f;
using CAD_0::math::Vec3d;

namespace {

constexpr double kPi = 3.14159265358979;
constexpr double kTwoPi = 6.28318530717958647692;

// THE DECLARED TOLERANCES (ADR-0035): the linear world's house value
// (D2.1 — the same 1e-6 the booleans declare as kBoolLinear), from
// which every ε_P below is DERIVED per family through the family's
// Jacobian (D2.2's named derivation — never a bare share).
constexpr double kEpsL = b::kBoolLinear.value;

// The p-curve's domain point at parameter t ∈ [0, 1] (the co-edge's
// traversal parametrization: segment lerp; arc angle lerp; circle
// the full turn from the u = 0 meridian).
Vec2d pcurve_at(const b::PCurve& p, double t) {
    switch (p.kind) {
    case b::PCurve::Kind::Segment:
        return Vec2d{p.segment.a.x + (p.segment.b.x - p.segment.a.x) * t,
                     p.segment.a.y + (p.segment.b.y - p.segment.a.y) * t};
    case b::PCurve::Kind::Arc: {
        const double ang = p.arc.start_angle + p.arc.sweep * t;
        return Vec2d{p.arc.center.x + p.arc.radius * std::cos(ang),
                     p.arc.center.y + p.arc.radius * std::sin(ang)};
    }
    case b::PCurve::Kind::Circle: {
        const double ang = kTwoPi * t;
        return Vec2d{p.circle.center.x + p.circle.radius * std::cos(ang),
                     p.circle.center.y + p.circle.radius * std::sin(ang)};
    }
    }
    return {};
}

// The periodic difference: du modulo 2π, in (−π, π].
double periodic_du(double a, double b) {
    double d = std::fmod(a - b, kTwoPi);
    if (d > kPi) d -= kTwoPi;
    if (d < -kPi) d += kTwoPi;
    return d;
}

// --- the per-family PULL (the inverse of evaluate) and ε_P -----------
//
// Each pull is the family's own closed form, written from the
// surface's stored parameters (the accessors), and each ε_P is the
// DERIVED parameter tolerance: ε_L mapped back through the family's
// Jacobian at the sample (D2.2: a NAMED derivation per family, in
// the family's own terms — never a silent share of the linear value).

struct PullResult {
    Vec2d uv{};
    double eps_u{0};
    double eps_v{0};
    bool periodic_u{false};
};

PullResult pull(const b::SurfaceGeometry& surf, const Vec3d& P) {
    PullResult r;
    if (auto* pl = dynamic_cast<const b::PlaneSurface*>(&surf)) {
        const Vec3d d = P - pl->origin();
        r.uv = {d.dot(pl->u_axis()), d.dot(pl->v_axis())};
        // Jacobian 1 (unit, orthogonal axes): ε_P = ε_L.
        r.eps_u = r.eps_v = kEpsL;
        return r;
    }
    if (auto* cy = dynamic_cast<const b::CylinderSurface*>(&surf)) {
        const double u = std::atan2(-(P.z - cy->origin().z),
                                    P.x - cy->origin().x);
        r.uv = {u < 0 ? u + kTwoPi : u, P.y - cy->origin().y};
        // u's Jacobian is r (mm/rad); v's is 1.
        r.eps_u = kEpsL / cy->radius();
        r.eps_v = kEpsL;
        r.periodic_u = true;
        return r;
    }
    if (auto* cz = dynamic_cast<const b::CylinderZSurface*>(&surf)) {
        const double phi = std::atan2(P.y - cz->axis_center().y,
                                      P.x - cz->axis_center().x);
        r.uv = {cz->u_sign() * phi, P.z - cz->z_base()};
        r.eps_u = kEpsL / cz->radius();
        r.eps_v = kEpsL;
        r.periodic_u = true;
        return r;
    }
    if (auto* sp = dynamic_cast<const b::SphereSurface*>(&surf)) {
        const Vec3d rel = (P - sp->origin()) * (1.0 / sp->radius());
        const Vec3d e0 = sp->azimuth();
        const Vec3d e1 = e0.cross(sp->pole());
        const double v = std::acos(std::clamp(rel.dot(sp->pole()), -1.0, 1.0));
        r.uv = {std::atan2(rel.dot(e1), rel.dot(e0)), v};
        // v's Jacobian is R; u's is R·sin v (the latitude's own
        // radius — the derivation names the degeneracy: at the poles
        // sin v → 0 and the u comparison loses meaning, which the
        // sampled latitudes below never reach).
        r.eps_u = kEpsL / (sp->radius() * std::max(std::sin(v), 0.25));
        r.eps_v = kEpsL / sp->radius();
        r.periodic_u = true;
        return r;
    }
    REQUIRE_MESSAGE(false, "pull: unsupported surface family");
    return r;
}

// --- the coherence contract, direction 1: ENDPOINTS (ε_L) is
// inlined in the per-body case below (each face against its OWN
// surface — the box's faces are six different planes). ---

// --- the coherence contract, direction 2: the sampled round trip ------

void check_round_trip(const b::BRepBody& body) {
    for (const auto& face : body.solid.shell().faces()) {
        const b::SurfaceGeometry* surf = face.surface();
        REQUIRE(surf != nullptr);
        for (const auto& loop : face.loops())
            for (const auto& ce : loop.coedges()) {
                const b::PCurve* pc = ce.p_curve();
                if (pc == nullptr) continue;
                for (int k = 0; k <= 8; ++k) {
                    const double t = static_cast<double>(k) / 8.0;
                    const Vec2d q = pcurve_at(*pc, t);
                    const Vec3d P = surf->evaluate(q.x, q.y);
                    const PullResult w = pull(*surf, P);
                    CAPTURE(ce.edge());
                    CAPTURE(k);
                    if (w.periodic_u) {
                        CHECK(std::abs(periodic_du(w.uv.x, q.x)) <= w.eps_u);
                    } else {
                        CHECK(std::abs(w.uv.x - q.x) <= w.eps_u);
                    }
                    CHECK(std::abs(w.uv.y - q.y) <= w.eps_v);
                }
            }
    }
}

// --- the sketch chains (the frontier's authors) ------------------------

std::vector<s::SketchChain> sector_chains() {
    const float a0 = -0.2f * static_cast<float>(kPi);
    const float a1 = 0.35f * static_cast<float>(kPi);
    const float r = 1.5f;
    const Vec2f c{0.0f, 0.0f};
    const Vec2f ps = c + Vec2f{r * std::cos(a0), r * std::sin(a0)};
    const Vec2f pe = c + Vec2f{r * std::cos(a1), r * std::sin(a1)};
    return {s::SketchChain{{
        s::sketch_segment(c, ps, true),
        s::sketch_arc(c, r, false, a0, a1 - a0),
        s::sketch_segment(pe, c, true),
    }}};
}

std::vector<s::SketchChain> square_with_hole() {
    return {s::SketchChain{{
                s::sketch_segment({-1.0f, -1.0f}, {1.0f, -1.0f}, true),
                s::sketch_segment({1.0f, -1.0f}, {1.0f, 1.0f}, true),
                s::sketch_segment({1.0f, 1.0f}, {-1.0f, 1.0f}, true),
                s::sketch_segment({-1.0f, 1.0f}, {-1.0f, -1.0f}, true),
            }},
            s::SketchChain{{s::sketch_circle(Vec2f{0.0f, 0.0f}, 0.4f,
                                              /*hole=*/true)}}};
}

} // namespace

// ---------------------------------------------------------------------------
// THE FRONTIER — one direction, exact, declared (D4)
// ---------------------------------------------------------------------------

TEST_CASE("H2-U3 morso 3 — the sketch frontier: the caps carry the sketch's own numbers, bit-for-bit") {
    auto body = b::make_prism(sector_chains(), 1.5, +1);
    REQUIRE(body != nullptr);
    // The two caps are the last two faces (face n, n+1 — the walk's
    // order). Their outer loops are the chain itself, entity for
    // entity: every segment co-edge's p-curve endpoints must be the
    // EXACT promotions of the sketch segment's own floats.
    const std::size_t nf = body->face_count();
    REQUIRE(nf >= 5);  // 3 lateral + 2 caps
    const std::vector<s::SketchChain> chains = sector_chains();
    const auto& ents = chains[0].entities;
    std::size_t checked = 0;
    for (const std::size_t cap : {nf - 2, nf - 1}) {
        const auto& face = body->solid.shell().faces()[cap];
        for (const auto& loop : face.loops())
            for (const auto& ce : loop.coedges()) {
                const b::PCurve* pc = ce.p_curve();
                REQUIRE(pc != nullptr);
                if (pc->kind != b::PCurve::Kind::Segment) continue;
                // Find the authoring sketch segment (the p-curve's
                // endpoints are its joints' domain images; the domain
                // image of the joint set {ps, pe} must contain the
                // promoted floats EXACTLY — the cap plane's frame at
                // these bodies maps (x, y) → (±x, ±y), so the
                // coordinates are the sketch's own, up to sign).
                bool matched = false;
                for (const auto& e : ents) {
                    if (e.kind != s::SketchEntity::Kind::Segment) continue;
                    const Vec2d a{static_cast<double>(e.segment.a.x),
                                  static_cast<double>(e.segment.a.y)};
                    const Vec2d bb{static_cast<double>(e.segment.b.x),
                                   static_cast<double>(e.segment.b.y)};
                    for (const Vec2d& q : {pc->segment.a, pc->segment.b}) {
                        for (const Vec2d& want :
                             {a, bb, Vec2d{-a.x, -a.y}, Vec2d{-bb.x, -bb.y},
                              Vec2d{a.x, -a.y}, Vec2d{bb.x, -bb.y},
                              Vec2d{-a.x, a.y}, Vec2d{-bb.x, bb.y}}) {
                            if (q.x == want.x && q.y == want.y) matched = true;
                        }
                    }
                }
                CHECK(matched);
                ++checked;
            }
    }
    CHECK(checked >= 4);  // the two radius segments on both caps

    // The arc's radius and sweep cross the frontier exactly too: the
    // caps' Arc2 carries the sketch's own radius bit-for-bit.
    for (const std::size_t cap : {nf - 2, nf - 1}) {
        for (const auto& ce :
             body->solid.shell().faces()[cap].loops()[0].coedges()) {
            const b::PCurve* pc = ce.p_curve();
            if (pc == nullptr || pc->kind != b::PCurve::Kind::Arc) continue;
            CHECK(pc->arc.radius ==
                  static_cast<double>(ents[1].arc.radius));
            CHECK(pc->arc.sweep == static_cast<double>(ents[1].arc.sweep));
        }
    }
}

// ---------------------------------------------------------------------------
// THE COHERENCE CONTRACT — endpoints (ε_L) and round trip (ε_P per family)
// ---------------------------------------------------------------------------

TEST_CASE("H2-U3 morso 3 — the coherence contract, per body (endpoints + round trip)") {
    struct Spec {
        const char* name;
        std::unique_ptr<b::BRepBody> body;
    };
    std::vector<Spec> bodies;
    bodies.push_back({"box", b::make_box(Vec3d{1, 2, 3})});
    bodies.push_back({"sphere", b::make_sphere(2.0)});
    bodies.push_back({"cylinder", b::make_cylinder(1.0, 2.0)});
    bodies.push_back({"tetrahedron", b::make_tetrahedron()});
    bodies.push_back({"prism(sector)", b::make_prism(sector_chains(), 1.5, +1)});
    bodies.push_back({"prism(hole)", b::make_prism(square_with_hole(), 2.0, +1)});

    for (const auto& spec : bodies) {
        CAPTURE(spec.name);
        REQUIRE(spec.body != nullptr);
        // ENDPOINTS (ε_L): per face, against the face's own surface.
        for (const auto& face : spec.body->solid.shell().faces()) {
            const b::SurfaceGeometry* surf = face.surface();
            REQUIRE(surf != nullptr);
            for (const auto& loop : face.loops())
                for (const auto& ce : loop.coedges()) {
                    const b::PCurve* pc = ce.p_curve();
                    if (pc == nullptr) continue;
                    if (pc->kind == b::PCurve::Kind::Circle) continue;
                    const Vec3d want_s = spec.body
                        ->vertices[ce.start_vertex(spec.body->edges)]
                        .position();
                    const Vec3d want_e = spec.body
                        ->vertices[ce.end_vertex(spec.body->edges)]
                        .position();
                    const Vec2d ps = pcurve_at(*pc, 0.0);
                    const Vec2d pe = pcurve_at(*pc, 1.0);
                    CAPTURE(ce.edge());
                    CHECK((surf->evaluate(ps.x, ps.y) - want_s).length() <=
                          kEpsL);
                    CHECK((surf->evaluate(pe.x, pe.y) - want_e).length() <=
                          kEpsL);
                }
        }
        // THE ROUND TRIP (ε_P per family, derived through the Jacobian).
        check_round_trip(*spec.body);
    }
}

// ---------------------------------------------------------------------------
// THE COHERENCE CONTRACT, the SSI reading: the 3D curve pulled onto the
// domain lands ON the p-curve (the table's two answers agree)
// ---------------------------------------------------------------------------

TEST_CASE("H2-U3 morso 3 — the SSI coherence: the 3D intersection curve lands on the domain answers") {
    // plane × sphere, the no-reframe path: the box's back face (+Y
    // normal) and the factory sphere (stored pole +Y) — the stored
    // frame serves, so both domain answers are typed in the faces'
    // OWN stored domains, and the pull below reads them directly.
    auto box = b::make_box(Vec3d{1, 1, 1});
    auto sph = b::make_sphere(1.25);
    REQUIRE(box != nullptr);
    REQUIRE(sph != nullptr);
    const b::FaceId plane_face = 3;   // back (+Y), origin (0, 1, 0)
    const b::FaceId sphere_face = 0;
    const b::SsiResult r =
        b::ssi_query(*box, plane_face, *sph, sphere_face, 1e-9);
    REQUIRE(r.kind == b::SsiResult::Kind::Circle);
    REQUIRE(r.curves.size() == 1);
    REQUIRE(r.a.pcurves.size() == 1);
    REQUIRE(r.b.pcurves.size() == 1);
    REQUIRE_FALSE(r.b.reframe_needed);

    const b::PlaneSurface* pl = dynamic_cast<const b::PlaneSurface*>(
        box->solid.shell().faces()[plane_face].surface());
    const b::SphereSurface* sp = dynamic_cast<const b::SphereSurface*>(
        sph->solid.shell().faces()[sphere_face].surface());
    REQUIRE(pl != nullptr);
    REQUIRE(sp != nullptr);

    // The plane-domain answer: the Circle2 at the foot. Every 3D
    // sample, pulled onto the plane's domain, lands ON the circle
    // (the distance to the centre equals the radius, within the
    // plane family's ε_P = ε_L — Jacobian 1).
    const b::PCurve& plane_pc = r.a.pcurves[0];
    REQUIRE(plane_pc.kind == b::PCurve::Kind::Circle);
    for (int k = 0; k < 16; ++k) {
        const double t = kTwoPi * static_cast<double>(k) / 16.0;
        const Vec3d P = r.curves[0].evaluate(t);
        const Vec3d d = P - pl->origin();
        const Vec2d uv{d.dot(pl->u_axis()), d.dot(pl->v_axis())};
        const double dist =
            (uv - plane_pc.circle.center).length();
        CAPTURE(k);
        CAPTURE(dist);
        CHECK(std::abs(dist - plane_pc.circle.radius) <= kEpsL);
    }

    // The sphere-domain answer: the latitude Line2 at v = const.
    // Every 3D sample, pulled onto the sphere's domain (the stored
    // frame), lands ON the latitude (v equals the stored v₀ within
    // the sphere family's ε_Pv = ε_L / R — the derived tolerance).
    const b::PCurve& sph_pc = r.b.pcurves[0];
    REQUIRE(sph_pc.kind == b::PCurve::Kind::Segment);
    for (int k = 0; k < 16; ++k) {
        const double t = kTwoPi * static_cast<double>(k) / 16.0;
        const Vec3d P = r.curves[0].evaluate(t);
        const Vec3d rel = (P - sp->origin()) * (1.0 / sp->radius());
        const double v =
            std::acos(std::clamp(rel.dot(sp->pole()), -1.0, 1.0));
        CAPTURE(k);
        CHECK(std::abs(v - sph_pc.segment.a.y) <= kEpsL / sp->radius());
    }

    // And the latitude spans the FULL period in u, exactly: the
    // endpoints are the double constants, not their float shadows.
    CHECK(sph_pc.segment.a.x == 0.0);
    CHECK(sph_pc.segment.b.x == kTwoPi);
    CHECK(sph_pc.segment.a.y == sph_pc.segment.b.y);
}

// ---------------------------------------------------------------------------
// THE DISCRIMINATING GOLDEN — double holds, float collapses (the proof)
// ---------------------------------------------------------------------------

TEST_CASE("H2-U3 morso 3 — the discriminating golden: the extended domain breaks the float payload at the declared eps_L, the double holds") {
    // ------------------------------------------------------------------
    // Exhibit A: the large-radius cylinder (R = 100). The seam's
    // p-curve lives at u = 2π; the float rounding of the constant
    // moves the seam's 3D image by R·δ with δ = |float(2π) − 2π|
    // ≈ 1.75e-7 rad — 1.75e-5 mm at R = 100, seventeen times over
    // the declared ε_L. The double payload's image sits at ~1e-13.
    // ------------------------------------------------------------------
    {
        auto body = b::make_cylinder(100.0, 50.0);
        REQUIRE(body != nullptr);
        const auto& wall = body->solid.shell().faces()[0];
        const b::SurfaceGeometry* surf = wall.surface();
        REQUIRE(surf != nullptr);
        // The seam's forward co-edge is the loop's first: its p-curve
        // runs (2π, −h) → (2π, +h).
        const b::CoEdge& seam = wall.loops()[0].coedges()[0];
        REQUIRE(seam.p_curve() != nullptr);
        // (a) THE STORED PAYLOAD IS THE EXACT DOUBLE CONSTANT — the
        // bit-level pin (the demotion would have stored 6.2831855f).
        CHECK(seam.p_curve()->segment.a.x == kTwoPi);
        CHECK(seam.p_curve()->segment.b.x == kTwoPi);
        // (b) THE COHERENCE HOLDS in double, at the declared ε_L.
        const Vec3d v_top =
            body->vertices[seam.end_vertex(body->edges)].position();
        const Vec2d pe = pcurve_at(*seam.p_curve(), 1.0);
        const double err_double =
            (surf->evaluate(pe.x, pe.y) - v_top).length();
        CAPTURE(err_double);
        CHECK(err_double <= kEpsL);
        // (c) THE FLOAT PAYLOAD COLLAPSES — the same evaluation at
        // the float-quantized constant (the bytes the demotion used
        // to store) misses the vertex record by more than ε_L: the
        // compromise was real, and it is closed.
        const double u_float = static_cast<double>(static_cast<float>(kTwoPi));
        const Vec3d p_float = surf->evaluate(u_float, pe.y);
        const double err_float = (p_float - v_top).length();
        CAPTURE(u_float);
        CAPTURE(err_float);
        CHECK(err_float > kEpsL);
        // The honest bill, named: the collapse scales with R (the
        // azimuth's Jacobian) — at R = 100 it is already 17× the
        // declared tolerance.
        CHECK(err_float > 17.0 * kEpsL);
    }

    // ------------------------------------------------------------------
    // Exhibit B: the tall fractional extrude (h = 100000.3). The
    // wall's window value at the top is the exact double; its float
    // rounding reads 100000.296875 — 3.1e-3 mm off the vertex
    // record (the float grid's spacing at 1e5 is 7.8e-3). This is
    // the "extended domain" stress in its purest form: the payload
    // value itself is large and fractional.
    // ------------------------------------------------------------------
    {
        auto body = b::make_prism(
            {s::SketchChain{{
                s::sketch_segment({-1.0f, -1.0f}, {1.0f, -1.0f}, true),
                s::sketch_segment({1.0f, -1.0f}, {1.0f, 1.0f}, true),
                s::sketch_segment({1.0f, 1.0f}, {-1.0f, 1.0f}, true),
                s::sketch_segment({-1.0f, 1.0f}, {-1.0f, -1.0f}, true),
            }}},
            100000.3, +1);
        REQUIRE(body != nullptr);
        // Face 0 is the first segment's lateral plane; its (u, v)
        // frame reads u = z (the front plane's rule) — the top edge's
        // p-curve carries u = 100000.3 exactly in double.
        const auto& face = body->solid.shell().faces()[0];
        const b::SurfaceGeometry* surf = face.surface();
        REQUIRE(surf != nullptr);
        const b::CoEdge& top = face.loops()[0].coedges()[2];  // top REV
        REQUIRE(top.p_curve() != nullptr);
        REQUIRE(top.p_curve()->kind == b::PCurve::Kind::Segment);
        // (a) The stored payload is the exact world double (the
        // projection of the exact vertex — no quantizer).
        const Vec3d v_ref = body->vertices[top.start_vertex(body->edges)]
                                .position();
        const double u_stored = top.p_curve()->segment.a.x;
        // u is the vertex's z-coordinate in this frame (the frame
        // rule pinned by the unit-3 tests): compare through the
        // surface's own evaluate to stay frame-honest.
        const Vec2d pa = top.p_curve()->segment.a;
        const Vec3d p_stored = surf->evaluate(pa.x, pa.y);
        CHECK((p_stored - v_ref).length() <= kEpsL);
        // (b) The float-quantized payload collapses: the same u
        // through the float grid misses by 3.1e-3 mm.
        const double u_float = static_cast<double>(static_cast<float>(u_stored));
        const Vec3d p_float = surf->evaluate(u_float, pa.y);
        const double err_float = (p_float - v_ref).length();
        CAPTURE(u_stored);
        CAPTURE(u_float);
        CAPTURE(err_float);
        CHECK(std::abs(u_float - u_stored) > 3e-3);  // the grid's bite
        CHECK(err_float > kEpsL);
        CHECK(err_float > 1000.0 * kEpsL);
        // And the payload itself is the vertex's own coordinate,
        // bit-exact: the projection is lossless in double.
        CHECK(u_stored == v_ref.z);
    }
}
