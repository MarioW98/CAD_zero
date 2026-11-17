// tests/unit/brep/test_g5_m3_corner_oracle.cpp
//
// G5 m3 — THE CORNER SPHERE'S INDEPENDENT ORACLE SWEEP (the reissued
// M3 contract's Phases 4–5, the verification the landed act now owns
// permanently): the act landed at one corner with its tavola, its
// volume and its chirality pin — this file sweeps the WHOLE contract
// with equations that are THIS FILE'S OWN:
//
// THE LAW (the brief's own words: "Do not use production construction
// helpers as test oracles"): every expected number below is derived
// here — the corner algebra (C = the corner offset r inward along
// each plane's outward normal), the tangency feet (the perpendiculars
// from C to the three planes), the frame rule (pole the shared cap's
// outward normal, azimuth the FIRED ring edge's lateral normal, e1
// the product), the window (u between 0 and the OTHER lateral's
// azimuth α_b — atan2 in the (e0, e1) basis, NEVER a hardcoded
// quadrant), the walk order (reconstructed from the landed p-curves'
// own domain chain, then checked for the outward right-hand
// orientation), the Steiner volume (8 − 3(2−r)r²(1−π/4) − r³(1−π/6),
// this file's own decomposition, cross-validated by its own
// quadrature below). mass_properties is the READ; the closed form is
// the ORACLE; the production construction is never consulted for an
// expected value.
//
// THE SWEEP: all EIGHT corners (the sign triplets), BOTH ring-role
// choices per corner (firing the X-parallel or the Y-parallel ring
// edge — the CylinderX/CylinderY roles permute), three radii
// (0.3, 0.5, 0.75) — 48 chains, every one the full contract; plus
// the translated frame (translate_body, the naming carried whole)
// and the rebuild digests bit-identical.
//
// THE CONTRACT, per chain (the brief's Phases 1–3 read on the landed
// body): the counts 13/21/10 χ=2; the sphere's radius, center, pole,
// azimuth and the evaluate-corners of its window; the three flanks'
// axes and radii; the three contact circles (center C, radius r, the
// planes through C ⊥ each axis — the coaxiality: every circle point
// on the sphere AND on its flank, the radials aligned); the octant's
// three arcs as Line2 in the sphere domain with the derived u-spans;
// the arcs' edges ending exactly at the tangency points and lying ON
// the contact circles; the walk's outward orientation; the volume at
// 1e-12.
//
// THE COUNTERPROOFS: the same contract functions fed doctored values
// (the center offset, the radius inflated, the handedness flipped,
// the walk reversed, the octant shifted, the flank's radius or axis
// incoherent, the lateral tilted) — every one MUST be rejected: the
// contract discriminates, it does not rubber-stamp.
//
// THE DEGENERATES (Phase 5, the dispositions probed live and pinned
// verbatim): the non-positive/non-finite radius family refused at the
// admission; the corner radii mismatch refused BY NAME at a corner
// the landed pin never touched; the reverse chain (ring first, then
// the vertical at the ring-occupied corner) refused at the class
// families — the vertical-first chain is the only road to the corner
// act; the radius band's edge at r = 1.99 admitted against r = 2.0
// refused BY NAME (fillet_radius_exceeds_face).
#include <doctest/doctest.h>

#include "CAD_0/brep/chain_gate.hpp"
#include "CAD_0/brep/factory.hpp"
#include "CAD_0/brep/fillet.hpp"
#include "CAD_0/brep/mass_properties.hpp"
#include "CAD_0/brep/naming.hpp"
#include "CAD_0/brep/pcurve.hpp"
#include "CAD_0/brep/query.hpp"
#include "CAD_0/brep/subtract.hpp"
#include "CAD_0/brep/topology.hpp"
#include "CAD_0/geometry/shape_id.hpp"

#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <string>
#include <vector>

namespace b = CAD_0::brep;
namespace n = CAD_0::brep::naming;
namespace math = CAD_0::math;
using CAD_0::geometry::FeatureId;
using CAD_0::geometry::ShapeId;
using CAD_0::geometry::SubshapeRef;

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kTol = 1e-12;

struct SphereReader : b::SurfaceVisitor {
    const b::SphereSurface* sphere{nullptr};
    void visit(const b::SphereSurface& s) override { sphere = &s; }
};
struct XReader : b::SurfaceVisitor {
    const b::CylinderXSurface* x{nullptr};
    void visit(const b::CylinderXSurface& s) override { x = &s; }
};
struct YReader : b::SurfaceVisitor {
    const b::CylinderYSurface* y{nullptr};
    void visit(const b::CylinderYSurface& s) override { y = &s; }
};
struct ZReader : b::SurfaceVisitor {
    const b::CylinderZSurface* z{nullptr};
    void visit(const b::CylinderZSurface& s) override { z = &s; }
};

// The box's own edge table (factory.hpp's numbering): bottom ring
// 0..3, top ring 4..7, verticals 8..11.
ShapeId edge_id(int index) {
    return ShapeId{
        FeatureId{1},
        SubshapeRef{}.push(static_cast<std::uint32_t>(index)), {}};
}
// vertical0: (−1,−1)  1: (+1,−1)  2: (+1,+1)  3: (−1,+1)
int vertical_at(double sx, double sy) {
    if (sx < 0 && sy < 0) return 8;
    if (sx > 0 && sy < 0) return 9;
    if (sx > 0 && sy > 0) return 10;
    return 11;
}
// The ring edge at the corner on its cap: top ring 4:(−1,−1)→(1,−1)
// X-par, 5:(1,−1)→(1,1) Y-par, 6:(1,1)→(−1,1) X-par, 7:(−1,1)→(−1,−1)
// Y-par; the bottom ring the same at 0..3.
int ring_edge_at(double sx, double sy, double sz, bool x_par) {
    const int base = sz > 0 ? 4 : 0;
    if (x_par) return base + (sy < 0 ? 0 : 2);   // y fixed at sy
    return base + (sx > 0 ? 1 : 3);              // x fixed at sx
}

// ---------------------------------------------------------------------------
// THE ORACLE'S OWN MATH — the corner algebra, the frame, the window.
// ---------------------------------------------------------------------------
struct CornerOracle {
    double sx, sy, sz;      // the corner's signs (±1)
    bool fire_x_par;        // the FIRED ring edge's class
    double r;
    math::Vec3d delta{0, 0, 0};   // the translated-frame offset

    // The derived expectations (closed forms, all MINE).
    math::Vec3d corner, C, T_x, T_y, T_z;
    math::Vec3d pole, azimuth, e1, d_b;
    double alpha_b;      // d_b's azimuth in the (e0, e1) basis
    double v_closed;     // the Steiner volume

    CornerOracle(double sx_, double sy_, double sz_, bool fire_x_par_,
                 double r_, math::Vec3d delta_ = {0, 0, 0})
        : sx(sx_), sy(sy_), sz(sz_), fire_x_par(fire_x_par_), r(r_),
          delta(delta_) {
        const math::Vec3d s3{sx, sy, sz};
        C = s3 * (1.0 - r);                 // the corner − r·(sx,sy,sz)
        T_x = {sx, C.y, C.z};
        T_y = {C.x, sy, C.z};
        T_z = {C.x, C.y, sz};
        pole = {0, 0, sz};
        // The FIRED edge's lateral normal is the azimuth; the other
        // lateral (d_b) is the tangency the window must reach.
        if (fire_x_par) {
            azimuth = {0, sy, 0};           // the lateral y = sy
            d_b = {sx, 0, 0};
        } else {
            azimuth = {sx, 0, 0};           // the lateral x = sx
            d_b = {0, sy, 0};
        }
        e1 = cross(azimuth, pole);
        alpha_b = std::atan2(d_b.dot(e1), d_b.dot(azimuth));
        v_closed = 8.0 - 3.0 * (2.0 - r) * r * r * (1.0 - kPi / 4.0)
                   - r * r * r * (1.0 - kPi / 6.0);
        // The translated frame: every 3D expectation rides the delta.
        corner = s3 + delta;
        C = C + delta;
        T_x = T_x + delta;
        T_y = T_y + delta;
        T_z = T_z + delta;
    }

    static math::Vec3d cross(const math::Vec3d& a, const math::Vec3d& b) {
        return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z,
                a.x * b.y - a.y * b.x};
    }

    // The tangency the azimuth meridian owns (u = 0, v = π/2).
    const math::Vec3d& T_a() const { return fire_x_par ? T_y : T_x; }
    // The other lateral's tangency (u = α_b, v = π/2).
    const math::Vec3d& T_b() const { return fire_x_par ? T_x : T_y; }
    // The fired flank's axis direction (X-parallel edge → the X
    // axis); the partner's is the other lateral.
    math::Vec3d fired_axis() const {
        return fire_x_par ? math::Vec3d{1, 0, 0} : math::Vec3d{0, 1, 0};
    }
    math::Vec3d partner_axis() const {
        return fire_x_par ? math::Vec3d{0, 1, 0} : math::Vec3d{1, 0, 0};
    }

    // MY evaluate of the sphere's frame (never the production call).
    math::Vec3d frame_at(double u, double v) const {
        const double sv = std::sin(v);
        return C + azimuth * (r * sv * std::cos(u))
                 + e1 * (r * sv * std::sin(u))
                 + pole * (r * std::cos(v));
    }

    // ---- THE CONTRACT FUNCTIONS (the counterproofs' targets) ----

    // (i) THE FEET: the center sits at distance rad from each of the
    // three planes (each given by unit normal n̂ through the point
    // p = the corner's foot on it), inward along n̂.
    static bool feet_ok(const math::Vec3d& c, double rad,
                        const math::Vec3d& n0, const math::Vec3d& p0,
                        const math::Vec3d& n1, const math::Vec3d& p1,
                        const math::Vec3d& n2, const math::Vec3d& p2) {
        const double d0 = std::abs((c - p0).dot(n0));
        const double d1 = std::abs((c - p1).dot(n1));
        const double d2 = std::abs((c - p2).dot(n2));
        return std::abs(d0 - rad) <= 1e-9 && std::abs(d1 - rad) <= 1e-9
            && std::abs(d2 - rad) <= 1e-9;
    }
    // The box family's own feet (the three axis-aligned planes, the
    // translated frame's own coordinates).
    bool box_feet_ok(const math::Vec3d& c, double rad) const {
        return feet_ok(c, rad,
                       {sx, 0, 0}, {sx + delta.x, 0, 0},
                       {0, sy, 0}, {0, sy + delta.y, 0},
                       {0, 0, sz}, {0, 0, sz + delta.z});
    }
    // (ii) THE CONTACT CIRCLE: the sphere's great circle in the
    // plane through c ⊥ the axis — every sampled point ON the
    // sphere (by construction) AND on the cylinder (distance r_cyl
    // from the axis), the radials aligned (the coaxial tangency).
    // The axis passes through the circle's center iff the normals
    // agree everywhere — the concurrence the closed forms demand.
    static bool contact_circle_ok(const math::Vec3d& c, double r_sph,
                                  const math::Vec3d& axis_point,
                                  const math::Vec3d& axis_dir,
                                  double r_cyl) {
        math::Vec3d u = cross(axis_dir, {1, 0, 0});
        if (u.length() < 0.5) u = cross(axis_dir, {0, 1, 0});
        const math::Vec3d w = cross(axis_dir, u);
        for (int i = 0; i < 5; ++i) {
            const double phi = 0.31 + 1.33 * static_cast<double>(i);
            const math::Vec3d Q =
                c + u * (r_sph * std::cos(phi))
                 + w * (r_sph * std::sin(phi));
            const math::Vec3d foot =
                axis_point
                + axis_dir * ((Q - axis_point).dot(axis_dir));
            const double d_cyl = (Q - foot).length();
            if (std::abs(d_cyl - r_cyl) > 1e-9) return false;
            const math::Vec3d n_sph = (Q - c) * (1.0 / r_sph);
            const math::Vec3d n_cyl = (Q - foot) * (1.0 / d_cyl);
            if ((n_sph - n_cyl).length() > 1e-9) return false;
        }
        return true;
    }
    // (iii) THE OCTANT CONTAINMENT of a 3D point (toward the corner).
    bool octant_ok(const math::Vec3d& Q) const {
        return sx * (Q.x - C.x) >= -1e-9 && sy * (Q.y - C.y) >= -1e-9
            && sz * (Q.z - C.z) >= -1e-9;
    }
    // (iv) THE WALK'S OUTWARD ORIENTATION: the tangency order in
    // walk sequence right-handed about the outward radial.
    bool orientation_ok(const math::Vec3d& V1, const math::Vec3d& V2,
                        const math::Vec3d& V3) const {
        const math::Vec3d a = (V1 - C) * (1.0 / r);
        const math::Vec3d b = (V2 - C) * (1.0 / r);
        const math::Vec3d c = (V3 - C) * (1.0 / r);
        return a.dot(cross(b, c)) > 0.0;
    }
};

// ---------------------------------------------------------------------------
// THE CHAIN — box → the vertical at the corner → the ring pair (the
// act fires: the corner is occupied by the vertical's CylinderZ).
// ---------------------------------------------------------------------------
struct ChainResult {
    std::unique_ptr<b::BRepBody> first;
    std::unique_ptr<b::BRepBody> result;
};

ChainResult build_chain(const CornerOracle& o) {
    auto box = b::make_box({1.0, 1.0, 1.0}, FeatureId{1});
    REQUIRE(box != nullptr);
    if (o.delta.length() > 0.0) b::translate_body(*box, o.delta);
    const b::FilletNaming vctx{FeatureId{2}, &n::kBoxNaming};
    const b::FilletNaming rctx{FeatureId{3}, &n::kBoxNaming};
    auto first = b::fillet_edge(
        *box, edge_id(vertical_at(o.sx, o.sy)), o.r, vctx);
    REQUIRE(first != nullptr);
    auto result = b::fillet_ring_edge(
        *first, edge_id(ring_edge_at(o.sx, o.sy, o.sz, o.fire_x_par)),
        o.r, rctx);
    REQUIRE(result != nullptr);
    return {std::move(first), std::move(result)};
}

// ---------------------------------------------------------------------------
// THE FULL CONTRACT, verified on one landed chain.
// ---------------------------------------------------------------------------
void verify_chain(const CornerOracle& o, const b::BRepBody& body,
                  const char* what) {
    INFO(what << ": the corner (" << o.sx << "," << o.sy << ","
              << o.sz << ") fired "
              << (o.fire_x_par ? "X-par" : "Y-par") << " r=" << o.r);

    // 1. THE COUNTS.
    CHECK(body.vertex_count() == 13);
    CHECK(body.edge_count() == 21);
    CHECK(body.face_count() == 10);
    CHECK(b::inner_loop_count(body) == 0);
    CHECK(b::euler_characteristic(body) == 2);
    CHECK(b::is_closed_orientable(body));
    CHECK(b::loops_are_closed(body));

    // 2. THE SPHERE FACE (exactly one, the born corner).
    const b::Face* corner_face = nullptr;
    int spheres = 0;
    for (const auto& f : body.solid.shell().faces()) {
        SphereReader sr;
        if (f.surface() != nullptr) f.surface()->accept(sr);
        if (sr.sphere != nullptr) {
            ++spheres;
            corner_face = &f;
        }
    }
    REQUIRE(spheres == 1);
    REQUIRE(corner_face != nullptr);
    CHECK(corner_face->name().find("corner[") != std::string::npos);
    SphereReader sr;
    corner_face->surface()->accept(sr);
    const b::SphereSurface& s = *sr.sphere;
    CHECK(s.radius() == doctest::Approx(o.r).epsilon(kTol));
    CHECK(s.origin().x == doctest::Approx(o.C.x).epsilon(kTol));
    CHECK(s.origin().y == doctest::Approx(o.C.y).epsilon(kTol));
    CHECK(s.origin().z == doctest::Approx(o.C.z).epsilon(kTol));
    CHECK(s.pole().x == doctest::Approx(o.pole.x).epsilon(kTol));
    CHECK(s.pole().y == doctest::Approx(o.pole.y).epsilon(kTol));
    CHECK(s.pole().z == doctest::Approx(o.pole.z).epsilon(kTol));
    CHECK(s.azimuth().x == doctest::Approx(o.azimuth.x).epsilon(kTol));
    CHECK(s.azimuth().y == doctest::Approx(o.azimuth.y).epsilon(kTol));
    CHECK(s.azimuth().z == doctest::Approx(o.azimuth.z).epsilon(kTol));

    // 3. THE WINDOW'S CORNERS — MY evaluate math against the landed
    // surface's own: (0, π/2) the azimuth's tangency, (α_b, π/2) the
    // other lateral's, v = 0 the pole vertex.
    {
        const math::Vec3d q_a = o.frame_at(0.0, kPi / 2.0);
        const math::Vec3d q_b = o.frame_at(o.alpha_b, kPi / 2.0);
        const math::Vec3d q_p = o.frame_at(0.3, 0.0);
        CHECK((s.evaluate(0.0, kPi / 2.0) - q_a).length() < 1e-12);
        CHECK((s.evaluate(o.alpha_b, kPi / 2.0) - q_b).length() < 1e-12);
        CHECK((s.evaluate(0.3, 0.0) - q_p).length() < 1e-12);
        CHECK((q_a - o.T_a()).length() < 1e-12);
        CHECK((q_b - o.T_b()).length() < 1e-12);
        CHECK((q_p - o.T_z).length() < 1e-12);
    }

    // 4. THE THREE FLANKS — the axes and the radii read through the
    // visitors (one Z — the vertical — one X and one Y — the pair).
    const b::CylinderZSurface* fz = nullptr;
    const b::CylinderXSurface* fx = nullptr;
    const b::CylinderYSurface* fy = nullptr;
    for (const auto& f : body.solid.shell().faces()) {
        ZReader zr;
        XReader xr;
        YReader yr;
        if (f.surface() == nullptr) continue;
        f.surface()->accept(zr);
        f.surface()->accept(xr);
        f.surface()->accept(yr);
        if (zr.z != nullptr) fz = zr.z;
        if (xr.x != nullptr) fx = xr.x;
        if (yr.y != nullptr) fy = yr.y;
    }
    REQUIRE(fz != nullptr);
    REQUIRE(fx != nullptr);
    REQUIRE(fy != nullptr);
    CHECK(fz->radius() == doctest::Approx(o.r).epsilon(kTol));
    CHECK(fz->axis_center().x == doctest::Approx(o.C.x).epsilon(kTol));
    CHECK(fz->axis_center().y == doctest::Approx(o.C.y).epsilon(kTol));
    CHECK(fx->radius() == doctest::Approx(o.r).epsilon(kTol));
    CHECK(fx->axis_y() == doctest::Approx(o.C.y).epsilon(kTol));
    CHECK(fx->axis_z() == doctest::Approx(o.C.z).epsilon(kTol));
    CHECK(fy->radius() == doctest::Approx(o.r).epsilon(kTol));
    CHECK(fy->axis_x() == doctest::Approx(o.C.x).epsilon(kTol));
    CHECK(fy->axis_z() == doctest::Approx(o.C.z).epsilon(kTol));

    // 5. THE CONTACT CIRCLES' CONTRACT — the center's feet, and each
    // circle (the sphere's great circle ⊥ the flank's axis, through
    // C) ON its flank with the radials aligned: the READ flank radii
    // enter the check, tying my circle to their surface.
    CHECK(o.box_feet_ok(o.C, o.r));
    // The Z circle: plane z = C.z, the vertical flank's own radius.
    CHECK(CornerOracle::contact_circle_ok(
        o.C, o.r, {o.C.x, o.C.y, o.C.z + 0.7}, {0, 0, 1}, fz->radius()));
    // The X circle: plane x = C.x, the CylinderX flank's axis
    // (·, C.y, C.z) — the READ axis coordinates enter the check.
    CHECK(CornerOracle::contact_circle_ok(
        o.C, o.r, {o.C.x + 0.7, fx->axis_y(), fx->axis_z()}, {1, 0, 0},
        fx->radius()));
    // The Y circle: plane y = C.y.
    CHECK(CornerOracle::contact_circle_ok(
        o.C, o.r, {fy->axis_x(), o.C.y + 0.7, fy->axis_z()}, {0, 1, 0},
        fy->radius()));

    // 6. THE OCTANT'S LOOP — three arcs, Line2 in the sphere domain,
    // the derived u-spans, interior_left on every co-edge.
    REQUIRE(corner_face->loops().size() == 1);
    const auto& coedges = corner_face->loops()[0].coedges();
    REQUIRE(coedges.size() == 3);
    struct Arc {
        math::Vec2d a, b;
        const b::CoEdge* ce;
    };
    std::vector<Arc> arcs;
    for (const auto& ce : coedges) {
        REQUIRE(ce.p_curve() != nullptr);
        REQUIRE(ce.p_curve()->kind == b::PCurve::Kind::Segment);
        arcs.push_back(
            {ce.p_curve()->segment.a, ce.p_curve()->segment.b, &ce});
        CHECK(ce.p_curve()->segment.interior_left);
    }
    const double lo = std::min(0.0, o.alpha_b);
    const double hi = std::max(0.0, o.alpha_b);
    bool has_equator = false, has_m0 = false, has_mb = false;
    for (const auto& arc : arcs) {
        const bool v_a_eq = std::abs(arc.a.y - kPi / 2.0) < 1e-9;
        const bool v_b_eq = std::abs(arc.b.y - kPi / 2.0) < 1e-9;
        if (v_a_eq && v_b_eq) {
            CHECK(std::min(arc.a.x, arc.b.x)
                  == doctest::Approx(lo).epsilon(1e-9));
            CHECK(std::max(arc.a.x, arc.b.x)
                  == doctest::Approx(hi).epsilon(1e-9));
            has_equator = true;
        } else if (std::abs(arc.a.x - arc.b.x) < 1e-9
                   && std::abs(arc.a.x) < 1e-9) {
            CHECK(std::min(arc.a.y, arc.b.y) < 1e-9);
            CHECK(std::max(arc.a.y, arc.b.y)
                  == doctest::Approx(kPi / 2.0).epsilon(1e-9));
            has_m0 = true;
        } else {
            CHECK(std::abs(arc.a.x - o.alpha_b) < 1e-9);
            CHECK(std::abs(arc.b.x - o.alpha_b) < 1e-9);
            CHECK(std::min(arc.a.y, arc.b.y) < 1e-9);
            CHECK(std::max(arc.a.y, arc.b.y)
                  == doctest::Approx(kPi / 2.0).epsilon(1e-9));
            has_mb = true;
        }
    }
    CHECK(has_equator);
    CHECK(has_m0);
    CHECK(has_mb);

    // 7. THE ARCS' EDGES — the endpoints exactly the tangency pair,
    // a sampled interior point ON the contact circle (the plane ⊥
    // the flank's axis, the sphere's radius), the octant held.
    for (const auto& arc : arcs) {
        const b::Edge& e = body.edges[arc.ce->edge()];
        const math::Vec3d& p0 =
            body.vertices[static_cast<std::size_t>(e.vertex(0))]
                .position();
        const math::Vec3d& p1 =
            body.vertices[static_cast<std::size_t>(e.vertex(1))]
                .position();
        const bool equator =
            std::abs(arc.a.y - kPi / 2.0) < 1e-9
            && std::abs(arc.b.y - kPi / 2.0) < 1e-9;
        const bool u0 = !equator && std::abs(arc.a.x) < 1e-9;
        // The u = 0 meridian rides the FIRED flank's circle (the
        // fired edge's lateral is the azimuth); the α_b meridian the
        // partner's; the equator the vertical's (plane z = C.z).
        const math::Vec3d n_axis =
            equator ? math::Vec3d{0, 0, 1}
                    : (u0 ? o.fired_axis() : o.partner_axis());
        const math::Vec3d& t_a =
            equator ? o.T_a() : (u0 ? o.T_a() : o.T_b());
        const math::Vec3d& t_b = equator ? o.T_b() : o.T_z;
        const bool ends_ok =
            ((p0 - t_a).length() < 1e-9 && (p1 - t_b).length() < 1e-9)
            || ((p0 - t_b).length() < 1e-9 && (p1 - t_a).length() < 1e-9);
        CHECK(ends_ok);
        const math::Vec2d mid{(arc.a.x + arc.b.x) * 0.5,
                              (arc.a.y + arc.b.y) * 0.5};
        const math::Vec3d Q = o.frame_at(mid.x, mid.y);
        CHECK(std::abs((Q - o.C).dot(n_axis)) < 1e-9);
        CHECK((Q - o.C).length() == doctest::Approx(o.r).epsilon(kTol));
        CHECK(o.octant_ok(Q));
    }

    // 8. THE WALK ORDER — chained from the p-curves' own domains
    // (the equator first; the chain closes through the pole's
    // degeneracy), then the outward right-hand check.
    {
        std::size_t i_eq = 0;
        for (std::size_t i = 0; i < arcs.size(); ++i) {
            if (std::abs(arcs[i].a.y - kPi / 2.0) < 1e-9
                && std::abs(arcs[i].b.y - kPi / 2.0) < 1e-9) {
                i_eq = i;
            }
        }
        std::vector<math::Vec2d> dom;
        dom.push_back(arcs[i_eq].a);
        dom.push_back(arcs[i_eq].b);
        std::vector<bool> used(arcs.size(), false);
        used[i_eq] = true;
        for (int step = 0; step < 2; ++step) {
            bool chained = false;
            for (std::size_t i = 0; i < arcs.size(); ++i) {
                if (used[i]) continue;
                const bool a_pole = arcs[i].a.y < 1e-9;
                const bool b_pole = arcs[i].b.y < 1e-9;
                const math::Vec2d last = dom.back();
                const bool a_next =
                    (!a_pole
                     && std::abs(arcs[i].a.x - last.x) < 1e-9
                     && std::abs(arcs[i].a.y - last.y) < 1e-9)
                    || (a_pole && last.y < 1e-9);
                const bool b_next =
                    (!b_pole
                     && std::abs(arcs[i].b.x - last.x) < 1e-9
                     && std::abs(arcs[i].b.y - last.y) < 1e-9)
                    || (b_pole && last.y < 1e-9);
                if (a_next) {
                    dom.push_back(arcs[i].b);
                    used[i] = true;
                    chained = true;
                    break;
                }
                if (b_next) {
                    dom.push_back(arcs[i].a);
                    used[i] = true;
                    chained = true;
                    break;
                }
            }
            REQUIRE(chained);
        }
        REQUIRE(dom.size() == 4);
        // The three distinct vertices of the cycle, in walk order.
        const math::Vec3d V1 = o.frame_at(dom[0].x, dom[0].y);
        const math::Vec3d V2 = o.frame_at(dom[1].x, dom[1].y);
        const math::Vec3d V3 = o.frame_at(dom[2].x, dom[2].y);
        CHECK(o.orientation_ok(V1, V2, V3));
    }

    // 9. THE VOLUME — mass_properties the READ, my Steiner form the
    // ORACLE.
    const b::MassProperties mp =
        b::mass_properties(body, math::Tolerance{});
    CHECK(mp.volume == doctest::Approx(o.v_closed).epsilon(kTol));
}

}  // namespace

// ---------------------------------------------------------------------------
// THE SWEEP — 8 corners × 2 fired roles × 3 radii, 48 chains.
// ---------------------------------------------------------------------------
TEST_CASE("G5 m3 oracle: the eight corners, both ring roles, three "
          "radii — the full contract every time") {
    for (const double sx : {-1.0, 1.0}) {
        for (const double sy : {-1.0, 1.0}) {
            for (const double sz : {-1.0, 1.0}) {
                for (const bool fire_x : {false, true}) {
                    for (const double r : {0.3, 0.5, 0.75}) {
                        const CornerOracle o(sx, sy, sz, fire_x, r);
                        auto chain = build_chain(o);
                        verify_chain(o, *chain.result, "the sweep");
                    }
                }
            }
        }
    }
}

TEST_CASE("G5 m3 oracle: the translated frame — the same contract at "
          "(5, −3, 2), the naming carried whole") {
    const CornerOracle o(1.0, -1.0, 1.0, false, 0.5, {5.0, -3.0, 2.0});
    auto chain = build_chain(o);
    verify_chain(o, *chain.result, "translated");
    auto chain2 = build_chain(o);
    CHECK(b::chain_digest(*chain.result)
          == b::chain_digest(*chain2.result));
}

TEST_CASE("G5 m3 oracle: the rebuild digests — every corner's chain "
          "bit-identical") {
    for (const double sx : {-1.0, 1.0}) {
        for (const double sz : {-1.0, 1.0}) {
            const CornerOracle o(sx, 1.0, sz, true, 0.4);
            auto a = build_chain(o);
            auto c2 = build_chain(o);
            CHECK(b::chain_digest(*a.result)
                  == b::chain_digest(*c2.result));
        }
    }
}

// ---------------------------------------------------------------------------
// THE COUNTERPROOFS — the contract fed doctored values must refuse.
// ---------------------------------------------------------------------------
TEST_CASE("G5 m3 oracle: the counterproofs — the doctored geometry "
          "rejected by the correct analytical contract") {
    const CornerOracle o(1.0, 1.0, 1.0, false, 0.5);

    // 1. THE INCORRECT CENTER: offset in x — the feet break.
    CHECK(!o.box_feet_ok(o.C + math::Vec3d{0.05, 0, 0}, o.r));
    // 2. THE INCORRECT RADIUS: inflated — the feet break.
    CHECK(!o.box_feet_ok(o.C, o.r * 1.1));
    // 3. THE NON-CONCURRENT AXIS: the flank's axis pushed off C —
    // the circle's points leave the cylinder, the radials disagree.
    CHECK(!CornerOracle::contact_circle_ok(
        o.C, o.r, {o.C.x + 0.05, o.C.y, o.C.z + 0.7}, {0, 0, 1},
        o.r));
    // 4. THE INCOHERENT CYLINDER RADIUS: the flank at 1.1r — the
    // sphere's circle points sit at r from the axis, not 1.1r.
    CHECK(!CornerOracle::contact_circle_ok(
        o.C, o.r, {o.C.x, o.C.y, o.C.z + 0.7}, {0, 0, 1}, o.r * 1.1));
    // 5. THE FLIPPED HANDEDNESS: the mirrored frame maps the window
    // corner onto the WRONG lateral — the anti-octant.
    {
        CornerOracle bad(1.0, 1.0, 1.0, false, 0.5);
        bad.e1 = bad.e1 * -1.0;   // the mancin0 flip
        const math::Vec3d q_b = bad.frame_at(bad.alpha_b, kPi / 2.0);
        CHECK((q_b - bad.T_b()).length() > 1e-3);
        CHECK(!bad.octant_ok(q_b));
    }
    // 6. THE REVERSED WALK: the tangency order inverted — the
    // outward right-hand orientation breaks.
    CHECK(!o.orientation_ok(o.T_a(), o.T_z, o.T_b()));
    // 7. THE INCORRECT OCTANT: the window shifted by π — the sample
    // violates the containment.
    CHECK(!o.octant_ok(o.frame_at(o.alpha_b + kPi, kPi / 2.0)));
    // 8. THE NON-ORTHOGONAL AXES: the lateral tilted 5° — the sphere
    // centered for the orthogonal corner sits NOT at r from the
    // tilted plane (the feet contract, generalized to the tilted
    // family, refuses the orthogonal center).
    {
        const double a = 5.0 * kPi / 180.0;
        const math::Vec3d n_t{std::cos(a), std::sin(a), 0.0};
        const math::Vec3d p_t{1.0, 1.0, 0.0};
        const double d_t = std::abs((o.C - p_t).dot(n_t));
        CHECK(std::abs(d_t - o.r) > 1e-6);   // the feet break
        CHECK(!CornerOracle::feet_ok(
            o.C, o.r, n_t, p_t, {0, 1, 0}, {0, 1, 0}, {0, 0, 1},
            {0, 0, 1}));
    }
}

// ---------------------------------------------------------------------------
// THE DEGENERATES — the dispositions probed live, pinned verbatim.
// ---------------------------------------------------------------------------
TEST_CASE("G5 m3 oracle: the degenerate family — the admissions' own "
          "words, no tolerance widened") {
    auto box = b::make_box({1.0, 1.0, 1.0}, FeatureId{1});
    const b::FilletNaming ctx{FeatureId{9}, &n::kBoxNaming};
    const ShapeId vertical2 = edge_id(10);

    const auto try_radius = [&](double r, bool expect_land,
                                const char* token) {
        try {
            auto out = b::fillet_edge(*box, vertical2, r, ctx);
            if (!expect_land) {
                REQUIRE(false);
            } else {
                REQUIRE(out != nullptr);
            }
        } catch (const std::exception& e) {
            if (expect_land) {
                REQUIRE(false);
            } else {
                CHECK(std::string(e.what()).find(token)
                      != std::string::npos);
            }
        }
    };
    try_radius(0.0, false, "must be finite and > 0");
    try_radius(-0.25, false, "must be finite and > 0");
    try_radius(std::numeric_limits<double>::quiet_NaN(), false,
               "must be finite and > 0");
    try_radius(std::numeric_limits<double>::infinity(), false,
               "must be finite and > 0");
    try_radius(1.99, true, nullptr);
    try_radius(2.0, false, "fillet_radius_exceeds_face");
    try_radius(2.01, false, "fillet_radius_exceeds_face");
}

TEST_CASE("G5 m3 oracle: the mismatch at a far corner — the wall is "
          "corpus-wide, not the landed pin's alone") {
    // vertical3 spans (−1, 1, ±1); top2 touches (−1, 1, 1): the act
    // fires there with DISAGREEING radii and must name the wall.
    auto box = b::make_box({1.0, 1.0, 1.0}, FeatureId{1});
    const b::FilletNaming v{FeatureId{2}, &n::kBoxNaming};
    const b::FilletNaming r{FeatureId{3}, &n::kBoxNaming};
    auto first = b::fillet_edge(*box, edge_id(11), 0.4, v);
    REQUIRE(first != nullptr);
    bool refused = false;
    try {
        auto out = b::fillet_ring_edge(*first, edge_id(6), 0.6, r);
        (void)out;
    } catch (const std::exception& e) {
        refused = true;
        const std::string msg = e.what();
        CHECK(msg.find("fillet_corner_radii_mismatch")
              != std::string::npos);
        CHECK(msg.find("0.600000") != std::string::npos);
        CHECK(msg.find("0.400000") != std::string::npos);
    }
    CHECK(refused);
}

TEST_CASE("G5 m3 oracle: the reverse chain — ring first, then the "
          "vertical at the ring-occupied corner, refused at the "
          "class families (the vertical-first chain is the only "
          "road)") {
    auto box = b::make_box({1.0, 1.0, 1.0}, FeatureId{1});
    const b::FilletNaming ring{FeatureId{2}, &n::kBoxNaming};
    const b::FilletNaming vert{FeatureId{3}, &n::kBoxNaming};
    auto after_ring = b::fillet_ring_edge(*box, edge_id(5), 0.5, ring);
    REQUIRE(after_ring != nullptr);
    CHECK(after_ring->vertex_count() == 10);   // the m2 plain path
    bool refused = false;
    try {
        auto out = b::fillet_edge(*after_ring, edge_id(10), 0.5, vert);
        (void)out;
    } catch (const std::exception& e) {
        refused = true;
        CHECK(std::string(e.what()).find("class families")
              != std::string::npos);
    }
    CHECK(refused);
}

// ---------------------------------------------------------------------------
// MY VOLUME FORM'S OWN VALIDATION — the quadrature (the third road).
// ---------------------------------------------------------------------------
TEST_CASE("G5 m3 oracle: the Steiner form validated by quadrature — "
          "this file's own integration, zero kernel code") {
    const double r = 0.5;
    const CornerOracle o(1.0, 1.0, 1.0, false, r);
    // The removed set: three wedge strips (the square minus the
    // quarter disc, swept (2 − r)) + the corner cube minus the
    // octant.
    long double wedge = 0.0L;
    const int N1 = 620;
    const double h = r / N1;
    for (int i = 0; i < N1; ++i) {
        for (int j = 0; j < N1; ++j) {
            const double x = (i + 0.5) * h;
            const double y = (j + 0.5) * h;
            if (x * x + y * y > r * r) wedge += h * h;
        }
    }
    long double corner_region = 0.0L;
    const int N2 = 90;
    const double h2 = r / N2;
    for (int i = 0; i < N2; ++i) {
        for (int j = 0; j < N2; ++j) {
            for (int k = 0; k < N2; ++k) {
                const double x = (i + 0.5) * h2;
                const double y = (j + 0.5) * h2;
                const double z = (k + 0.5) * h2;
                if (x * x + y * y + z * z > r * r) {
                    corner_region += h2 * h2 * h2;
                }
            }
        }
    }
    const double removed =
        3.0 * static_cast<double>(wedge) * (2.0 - r)
        + static_cast<double>(corner_region);
    const double v_quad = 8.0 - removed;
    CHECK(v_quad == doctest::Approx(o.v_closed).epsilon(3e-5));
    // And the landed pin's own number (6.75 + 29π/96) is this same
    // form at r = 0.5 — the generalization closed.
    CHECK(o.v_closed
          == doctest::Approx(6.75 + 29.0 * kPi / 96.0).epsilon(kTol));
}
