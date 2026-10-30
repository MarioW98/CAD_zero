// tests/unit/brep/test_g2_unit1_ssi_table.cpp
//
// G2 UNIT 1 — THE TABLE: the discriminating tests (ADR-0033 D1.1 +
// D5.1, written FIRST per the house discipline — they fail today for
// the right reason: the table's header does not exist yet, and the
// build itself names the absence, the G1 unit-1/unit-4 precedents).
//
// What this file pins, family by family (the order's own scope):
//
//   * THE FOUR ENTRIES, each a derivation whose golden IS the
//     classical formula:
//     - plane×sphere (d<R): the circle √(R²−d²) at the foot of the
//       perpendicular; Circle2 on the plane's own domain; the
//       latitude Line2 on the sphere's RE-FRAMED domain (pole = the
//       plane normal, azimuth = the plane's u axis — the derived
//       frame, declared in the answer);
//     - sphere×sphere (non-nested): the circle in the RADICAL PLANE
//       (h = (D²+R₁²−R₂²)/2D, ρ = √(R₁²−h²)), both spheres re-framed
//       to the center line;
//     - cylinder×sphere coaxial (r<R): the TWO circles at axial
//       offset ±√(R²−r²), the cylinder's own radius; Line2 v=const on
//       the cylinder domain, latitudes on the sphere re-framed to the
//       axis (or typed in the stored frame when the pole already
//       serves — the no-reframe path);
//     - cylinder×cylinder parallel, EQUAL radii, offset d<2r: the
//       two lines in the mid-plane, at ±√(r²−d²/4) along the
//       right-handed vertical δ̂⊥×â; Line2 u=const on each domain,
//       spanning the face's OWN authored v-window (derived from its
//       loops — the window the boolean will trim within).
//   * THE DEGENERATES per entry, each with its pinned disposition:
//     tangency (the point / the tangent circle / the double line),
//     empty (the class named: separate, nested, contained), and the
//     COINCIDENT degenerate (equal spheres, coaxial equal cylinders —
//     the co-radial dedup rule's trigger, the coplanar rule's analog).
//   * THE WALL (D2 = option (i), accepted): every out-of-table pair
//     is REFUSED with an actionable record naming the pair, the
//     disposition class and the remedies — the Viviani quartic (the
//     non-coaxial sphere×cylinder), the general cylinder pair (the
//     irreducible quartic; the Steinmetz special case typed in
//     neither domain), the UNEQUAL-RADII parallel pair (the
//     acceptance's own correction: the transverse case is two TYPED
//     lines in the offset plane, out of the registered entry until
//     the unit that consumes it — never called "untypable"), the
//     landed pairs pointed to their owners (plane×cylinder = the
//     subtract's own entry; plane×plane = Phase D), the cone (no
//     writer; the mathematics registered), the torus (registered for
//     the fillet's unit), and the provenance gate (the exact table
//     consults exact faces — a faceted operand is refused, remedy
//     named).
//   * THE RE-FRAME RULE: the seam-only revolution face REBUILDS on
//     the same mathematical object with the derived axis — the poles
//     move, the seam pair is re-wired in the SAME slit-rectangle form
//     (ADR-0023's pattern), the surface still evaluates onto the
//     sphere, χ and the structural contract are untouched; the
//     re-frame is a PURE function of the recipe (ADR-0014): rebuild
//     from scratch, re-frame, digest — bit-identical, hex floats.
//   * THE FRAME BUDGET: one frame = one axis family per face. A face
//     whose typed cuts budget the stored family finds THE WALL when
//     an incompatible axis arrives (the golden error, name + remedy);
//     the budget is DERIVED from the face's own record (the distinct
//     edges beyond the seam — the record is the thing under test, the
//     provenance test's precedent), never stored beside it.
//   * DETERMINISM: the query is a pure function of the operands —
//     same bodies, same answer, hex-digest identical; rebuilt bodies,
//     same digest again.
//
// The conventions pinned here as regression guards (the header
// documents the derivation): the latitude p-curve of a full periodic
// cut is ONE Line2 spanning the domain's full period (0→2π at v=const
// — the wall's own full-window precedent); the line p-curves span the
// face's authored v-window (the boolean will trim within it); the
// region-side bits in the ANSWER's p-curves are NEUTRAL defaults
// (interior_left = true, hole = false — the typed GEOMETRY; the
// region side is the consuming operation's authoring, unit 2's).
#include <doctest/doctest.h>

#include "CAD_0/brep/ssi.hpp"
#include "CAD_0/brep/analytic.hpp"
#include "CAD_0/brep/curve.hpp"
#include "CAD_0/brep/factory.hpp"
#include "CAD_0/brep/extrude.hpp"
#include "CAD_0/brep/query.hpp"
#include "CAD_0/brep/subtract.hpp"
#include "CAD_0/brep/topology.hpp"
#include "CAD_0/sdf/sketch_entities.hpp"
#include "CAD_0/sdf/sketch_profile.hpp"

#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

namespace {

using CAD_0::brep::BRepBody;
using CAD_0::brep::FaceId;
using CAD_0::brep::SsiResult;
using CAD_0::math::Vec2f;
using CAD_0::math::Vec3d;

constexpr float kTol = 1e-6f;
constexpr double kPi = 3.14159265358979;
constexpr double kTwoPi = 6.28318530717958647692;

bool close(double a, double b, double tol = 1e-5) { return std::abs(a - b) <= tol; }
bool close3(Vec3d a, Vec3d b, double tol = 1e-5) {
    return close(a.x, b.x, tol) && close(a.y, b.y, tol) && close(a.z, b.z, tol);
}

// The face of `body` whose surface family matches, by shell order —
// the same walk the face query serves. REQUIREs exactly one match.
FaceId face_of_family(const BRepBody& body, const char* family) {
    FaceId found = static_cast<FaceId>(-1);
    int n = 0;
    for (const auto& f : body.solid.shell().faces()) {
        if (f.surface() && std::string(f.surface()->type_name()) == family) {
            found = f.id();
            ++n;
        }
    }
    REQUIRE_MESSAGE(n >= 1, "no face of family ", family, " in this body");
    return found;
}

// The +X normal plane face of a box (the golden's operand).
FaceId box_posx_face(const BRepBody& box) {
    for (const auto& f : box.solid.shell().faces()) {
        const auto* pl =
            dynamic_cast<const CAD_0::brep::PlaneSurface*>(f.surface());
        if (pl && close3(pl->normal_vec(), {1, 0, 0}))
            return f.id();
    }
    REQUIRE_MESSAGE(false, "the box has no +X face — report the factory");
    return 0;
}

// --- the digest (ADR-0014): every number as a hex float -------------
// H1 (ADR-0034): the digest reads the exact world's doubles; the
// p-curve payloads promote (float -> double) losslessly at the read.
void af(std::string& out, double v) {
    char buf[64];
    std::snprintf(buf, sizeof(buf), "%a", v);
    out += buf;
}
void av3(std::string& out, const Vec3d& v) {
    out += '(';
    af(out, v.x); out += ',';
    af(out, v.y); out += ',';
    af(out, v.z); out += ')';
}
void av2(std::string& out, const CAD_0::math::Vec2d& v) {
    out += '(';
    af(out, v.x); out += ',';
    af(out, v.y); out += ')';
}
void apc(std::string& out, const CAD_0::brep::PCurve& p) {
    af(out, static_cast<double>(static_cast<int>(p.kind)));
    switch (p.kind) {
    case CAD_0::brep::PCurve::Kind::Segment:
        av2(out, p.segment.a); av2(out, p.segment.b);
        af(out, p.segment.interior_left ? 1.0 : 0.0);
        break;
    case CAD_0::brep::PCurve::Kind::Circle:
        av2(out, p.circle.center); af(out, p.circle.radius);
        af(out, p.circle.hole ? 1.0 : 0.0);
        break;
    case CAD_0::brep::PCurve::Kind::Arc:
        av2(out, p.arc.center); af(out, p.arc.radius);
        af(out, p.arc.hole ? 1.0 : 0.0);
        af(out, p.arc.start_angle); af(out, p.arc.sweep);
        break;
    }
}
std::string ssi_digest(const SsiResult& r) {
    std::string out;
    af(out, static_cast<float>(static_cast<int>(r.kind)));
    af(out, static_cast<float>(r.provenance.is_exact()));
    af(out, static_cast<float>(r.provenance.linear_tolerance));
    out += r.family_a; out += '|'; out += r.family_b; out += '|';
    out += r.message; out += '|';
    av3(out, r.point);
    for (const auto& c : r.curves) {
        af(out, static_cast<float>(static_cast<int>(c.kind)));
        av3(out, c.center); af(out, c.radius);
        av3(out, c.u_ref); av3(out, c.v_ref);
        av3(out, c.point); av3(out, c.dir);
    }
    for (const auto* d : {&r.a, &r.b}) {
        af(out, d->reframe_needed ? 1.0f : 0.0f);
        av3(out, d->frame_pole); av3(out, d->frame_azimuth);
        for (const auto& p : d->pcurves) apc(out, p);
    }
    return out;
}

// One segment p-curve of the answer, as (a, b, interior_left).
struct SegPC {
    CAD_0::math::Vec2d a{}, b{};
    bool interior_left{true};
};
SegPC seg_of(const CAD_0::brep::SsiDomainAnswer& d, std::size_t i) {
    REQUIRE(i < d.pcurves.size());
    REQUIRE(d.pcurves[i].kind == CAD_0::brep::PCurve::Kind::Segment);
    return {d.pcurves[i].segment.a, d.pcurves[i].segment.b,
            d.pcurves[i].segment.interior_left};
}

// The record-under-test: a sphere face BUDGETED by a typed latitude
// cut of the STORED frame (+Y pole). The cut is a record, not a
// boolean product — the gate honors the record without laundering it
// (the G1 provenance test's precedent, verbatim in spirit): two extra
// co-edges on a second edge, carrying the latitude's segment pair.
std::unique_ptr<BRepBody> budgeted_sphere(float radius) {
    auto body = CAD_0::brep::make_sphere(radius);
    // The second edge (id 1) and its latitude pair at v = π/3 of the
    // stored +Y-pole frame — horizontal segments, the typed cut class
    // the ADR's budget rule names.
    body->edges.push_back(CAD_0::brep::Edge{0, 0});
    body->edges.back().set_id(1);
    auto& face = body->solid.shell_mut().faces_mut()[0];
    CAD_0::brep::CoEdge cut_fwd{1, false};
    cut_fwd.set_p_curve(CAD_0::brep::pcurve_segment(
        CAD_0::math::Vec2d{0.5, kPi / 3.0},
        CAD_0::math::Vec2d{1.5, kPi / 3.0}, true));
    CAD_0::brep::CoEdge cut_rev{1, true};
    cut_rev.set_p_curve(CAD_0::brep::pcurve_segment(
        CAD_0::math::Vec2d{1.5, kPi / 3.0},
        CAD_0::math::Vec2d{0.5, kPi / 3.0}, true));
    CAD_0::brep::Loop extra;
    extra.add_coedge(cut_fwd);
    extra.add_coedge(cut_rev);
    extra.set_outer(false);
    face.add_loop(std::move(extra));
    CAD_0::brep::link_radial_edges(*body);
    return body;
}

// A one-face body carrying an arbitrary analytic surface — the
// cone/torus refusal records (no writer exists; the family itself is
// the thing under test).
std::unique_ptr<BRepBody> one_face_body(
    std::shared_ptr<CAD_0::brep::SurfaceGeometry> surf) {
    auto body = std::make_unique<BRepBody>();
    auto& shell = body->solid.shell_mut();
    shell.set_closed(false);  // a record, not a solid — the query is
                              // surface-level on these families
    CAD_0::brep::Face face(std::move(surf));
    face.set_id(0);
    shell.add_face(std::move(face));
    body->vertices.emplace_back(Vec3d{0, 0, 0});
    body->vertices.back().set_id(0);
    body->edges.emplace_back(CAD_0::brep::Edge{0, 0});
    body->edges.back().set_id(0);
    CAD_0::brep::link_radial_edges(*body);
    return body;
}

} // namespace

// ===========================================================================
// FAMILY 1 — plane×sphere: the circle √(R²−d²) at the foot
// ===========================================================================
TEST_CASE("G2-U1 plane×sphere: the circle at the foot, Circle2 + the re-framed latitude") {
    const auto box = CAD_0::brep::make_box({1.0f, 2.0f, 3.0f});
    const auto sph = CAD_0::brep::make_sphere(2.0f);
    const FaceId pf = box_posx_face(*box);
    const FaceId sf = face_of_family(*sph, "sphere");

    const SsiResult r = CAD_0::brep::ssi_query(*box, pf, *sph, sf, kTol);

    CHECK(r.kind == SsiResult::Kind::Circle);
    CHECK(r.family_a == "plane");
    CHECK(r.family_b == "sphere");
    CHECK(r.provenance.is_exact());
    CHECK(r.provenance.linear_tolerance == 0.0);

    // The classical formula IS the test: radius √(R²−d²) = √(4−1),
    // centered at the foot of the perpendicular (1, 0, 0).
    REQUIRE(r.curves.size() == 1);
    CHECK(r.curves[0].kind == CAD_0::brep::SsiCurve::Kind::Circle);
    CHECK(close(r.curves[0].radius, std::sqrt(3.0f)));
    CHECK(close3(r.curves[0].center, {1.0f, 0.0f, 0.0f}));
    // The circle's frame is the plane's own (u_axis, v_axis): for the
    // +X face the derived frame is (+Z, −Y) — the PlaneSurface rule.
    CHECK(close3(r.curves[0].u_ref, {0, 0, 1}));
    CHECK(close3(r.curves[0].v_ref, {0, -1, 0}));
    // The curve lies on both surfaces (the geometric identity, sampled).
    const auto* pl = dynamic_cast<const CAD_0::brep::PlaneSurface*>(
        box->solid.shell().faces()[pf].surface());
    for (float t : {0.0f, 1.0f, 2.0f, 4.0f}) {
        const Vec3d p = r.curves[0].evaluate(t);
        CHECK(close((p - pl->origin()).dot(pl->normal_vec()), 0.0f, 1e-5f));
        CHECK(close((p - Vec3d{0, 0, 0}).length(), 2.0f, 1e-5f));
    }

    // The plane-domain p-curve: the Circle2 at the foot's domain
    // coordinates, radius √3 — the same numbers the sketcher reads.
    REQUIRE(r.a.pcurves.size() == 1);
    REQUIRE(r.a.pcurves[0].kind == CAD_0::brep::PCurve::Kind::Circle);
    CHECK(close(r.a.pcurves[0].circle.radius, std::sqrt(3.0f)));
    CHECK(!r.a.pcurves[0].circle.hole);

    // The sphere-domain p-curve: the latitude of the RE-FRAMED domain
    // (pole = the plane normal +X, azimuth = the plane's u axis +Z),
    // v = arccos(−d/R) = arccos(1/2) = π/3, the full-period Line2.
    CHECK(r.b.reframe_needed);
    CHECK(close3(r.b.frame_pole, {1, 0, 0}));
    CHECK(close3(r.b.frame_azimuth, {0, 0, 1}));
    const SegPC lat = seg_of(r.b, 0);
    CHECK(close(lat.a.y, kPi / 3.0f));
    CHECK(close(lat.b.y, kPi / 3.0f));
    CHECK(close(lat.a.x, 0.0f));
    CHECK(close(lat.b.x, kTwoPi));
    CHECK(lat.interior_left);
}

TEST_CASE("G2-U1 plane×sphere degenerates: the tangent point and the empty classes") {
    // d = R exactly: the tangent point (2, 0, 0).
    {
        const auto box = CAD_0::brep::make_box({2.0f, 2.0f, 2.0f});
        const auto sph = CAD_0::brep::make_sphere(2.0f);
        const SsiResult r = CAD_0::brep::ssi_query(
            *box, box_posx_face(*box), *sph, face_of_family(*sph, "sphere"), kTol);
        CHECK(r.kind == SsiResult::Kind::Point);
        CHECK(close3(r.point, {2, 0, 0}));
        CHECK(r.curves.empty());
        CHECK(r.provenance.is_exact());
    }
    // d > R: separate — the honest empty, the class named.
    {
        const auto box = CAD_0::brep::make_box({3.0f, 2.0f, 2.0f});
        const auto sph = CAD_0::brep::make_sphere(2.0f);
        const SsiResult r = CAD_0::brep::ssi_query(
            *box, box_posx_face(*box), *sph, face_of_family(*sph, "sphere"), kTol);
        CHECK(r.kind == SsiResult::Kind::Empty);
        CHECK(r.curves.empty());
        CHECK(r.message.find("separate") != std::string::npos);
    }
    // The symmetric image: the same entry with the operands swapped
    // (the pair order is the same entry, the ADR's word).
    {
        const auto box = CAD_0::brep::make_box({1.0f, 2.0f, 3.0f});
        const auto sph = CAD_0::brep::make_sphere(2.0f);
        const SsiResult r = CAD_0::brep::ssi_query(
            *sph, face_of_family(*sph, "sphere"), *box, box_posx_face(*box), kTol);
        CHECK(r.kind == SsiResult::Kind::Circle);
        REQUIRE(r.curves.size() == 1);
        CHECK(close(r.curves[0].radius, std::sqrt(3.0f)));
        CHECK(close3(r.curves[0].center, {1.0f, 0.0f, 0.0f}));
        CHECK(r.family_a == "sphere");
        CHECK(r.family_b == "plane");
        CHECK(r.a.reframe_needed);  // the sphere is now operand A
    }
}

// ===========================================================================
// FAMILY 2 — sphere×sphere: the radical plane
// ===========================================================================
TEST_CASE("G2-U1 sphere×sphere: the circle in the radical plane, both re-framed") {
    auto sph1 = CAD_0::brep::make_sphere(2.0f);
    auto sph2 = CAD_0::brep::make_sphere(1.0f);
    CAD_0::brep::translate_body(*sph2, {2.5f, 0.0f, 0.0f});

    const FaceId f1 = face_of_family(*sph1, "sphere");
    const FaceId f2 = face_of_family(*sph2, "sphere");
    const SsiResult r = CAD_0::brep::ssi_query(*sph1, f1, *sph2, f2, kTol);

    CHECK(r.kind == SsiResult::Kind::Circle);
    // The classical radical-plane arithmetic, hand-checkable:
    //   D = 2.5, h = (D²+R₁²−R₂²)/2D = (6.25+4−1)/5 = 1.85,
    //   ρ = √(R₁²−h²) = √0.5775, center = c₁ + h·ŵ.
    REQUIRE(r.curves.size() == 1);
    CHECK(close(r.curves[0].radius, std::sqrt(0.5775f), 1e-4f));
    CHECK(close3(r.curves[0].center, {1.85f, 0.0f, 0.0f}, 1e-4f));
    // Every point of the curve sits on BOTH spheres (the definition).
    for (float t : {0.5f, 2.0f, 4.5f}) {
        const Vec3d p = r.curves[0].evaluate(t);
        CHECK(close((p - Vec3d{0, 0, 0}).length(), 2.0f, 1e-4f));
        CHECK(close((p - Vec3d{2.5f, 0, 0}).length(), 1.0f, 1e-4f));
    }

    // Both spheres re-frame to the center line (+X), azimuth = the
    // derived radical plane's u axis (+Z, the PlaneSurface rule).
    CHECK(r.a.reframe_needed);
    CHECK(r.b.reframe_needed);
    CHECK(close3(r.a.frame_pole, {1, 0, 0}));
    CHECK(close3(r.b.frame_pole, {1, 0, 0}));
    CHECK(close3(r.a.frame_azimuth, {0, 0, 1}));
    CHECK(close3(r.b.frame_azimuth, {0, 0, 1}));

    // The latitudes: v₁ = arccos(h/R₁) = arccos(0.925);
    //                v₂ = arccos((h−D)/R₂) = arccos(−0.65).
    const SegPC l1 = seg_of(r.a, 0);
    const SegPC l2 = seg_of(r.b, 0);
    CHECK(close(l1.a.y, std::acos(0.925f)));
    CHECK(close(l2.a.y, std::acos(-0.65f)));
    CHECK(close(l1.b.x, kTwoPi));
    CHECK(close(l2.b.x, kTwoPi));
}

TEST_CASE("G2-U1 sphere×sphere degenerates: tangencies, the nested/separate empty, the co-radial coincident") {
    const FaceId f0 = 0;
    // External tangency: D = R₁+R₂ = 3 → the point (2, 0, 0).
    {
        auto s1 = CAD_0::brep::make_sphere(2.0f);
        auto s2 = CAD_0::brep::make_sphere(1.0f);
        CAD_0::brep::translate_body(*s2, {3.0f, 0, 0});
        const SsiResult r = CAD_0::brep::ssi_query(*s1, f0, *s2, f0, kTol);
        CHECK(r.kind == SsiResult::Kind::Point);
        CHECK(close3(r.point, {2, 0, 0}, 1e-4f));
        CHECK(r.message.find("external") != std::string::npos);
    }
    // Internal tangency: D = R₁−R₂ = 1.5 → the point (2, 0, 0).
    {
        auto s1 = CAD_0::brep::make_sphere(2.0f);
        auto s2 = CAD_0::brep::make_sphere(0.5f);
        CAD_0::brep::translate_body(*s2, {1.5f, 0, 0});
        const SsiResult r = CAD_0::brep::ssi_query(*s1, f0, *s2, f0, kTol);
        CHECK(r.kind == SsiResult::Kind::Point);
        CHECK(close3(r.point, {2, 0, 0}, 1e-4f));
        CHECK(r.message.find("internal") != std::string::npos);
    }
    // Nested: D < |R₁−R₂| → the empty, the class named.
    {
        auto s1 = CAD_0::brep::make_sphere(2.0f);
        auto s2 = CAD_0::brep::make_sphere(0.5f);
        CAD_0::brep::translate_body(*s2, {0.5f, 0, 0});
        const SsiResult r = CAD_0::brep::ssi_query(*s1, f0, *s2, f0, kTol);
        CHECK(r.kind == SsiResult::Kind::Empty);
        CHECK(r.message.find("nested") != std::string::npos);
    }
    // Separate: D > R₁+R₂.
    {
        auto s1 = CAD_0::brep::make_sphere(2.0f);
        auto s2 = CAD_0::brep::make_sphere(1.0f);
        CAD_0::brep::translate_body(*s2, {4.0f, 0, 0});
        const SsiResult r = CAD_0::brep::ssi_query(*s1, f0, *s2, f0, kTol);
        CHECK(r.kind == SsiResult::Kind::Empty);
        CHECK(r.message.find("separate") != std::string::npos);
    }
    // The COINCIDENT degenerate: equal spheres, same center — the
    // co-radial dedup rule's trigger (the coplanar rule's analog).
    {
        auto s1 = CAD_0::brep::make_sphere(2.0f);
        auto s2 = CAD_0::brep::make_sphere(2.0f);
        const SsiResult r = CAD_0::brep::ssi_query(*s1, f0, *s2, f0, kTol);
        CHECK(r.kind == SsiResult::Kind::Coincident);
        CHECK(r.curves.empty());
        CHECK(r.message.find("co-radial") != std::string::npos);
    }
}

// ===========================================================================
// FAMILY 3 — cylinder×sphere coaxial: the two circles at ±√(R²−r²)
// ===========================================================================
TEST_CASE("G2-U1 cylinder×sphere coaxial: two circles at ±√(R²−r²), no re-frame on the stored pole") {
    const auto cyl = CAD_0::brep::make_cylinder(1.0f, 2.0f);  // axis Y
    const auto sph = CAD_0::brep::make_sphere(2.0f);          // on the axis
    const FaceId wf = face_of_family(*cyl, "cylinder");
    const FaceId sf = face_of_family(*sph, "sphere");

    const SsiResult r = CAD_0::brep::ssi_query(*cyl, wf, *sph, sf, kTol);

    CHECK(r.kind == SsiResult::Kind::CirclePair);
    REQUIRE(r.curves.size() == 2);
    // The classical pair: circles of the cylinder's radius r at axial
    // offsets ±√(R²−r²) = ±√3 from the sphere's center.
    for (int i = 0; i < 2; ++i) {
        CHECK(r.curves[i].kind == CAD_0::brep::SsiCurve::Kind::Circle);
        CHECK(close(r.curves[i].radius, 1.0f));
    }
    CHECK(close3(r.curves[0].center, {0, std::sqrt(3.0f), 0}, 1e-5f));
    CHECK(close3(r.curves[1].center, {0, -std::sqrt(3.0f), 0}, 1e-5f));
    // The circle frames are right-handed around the axis, anchored at
    // the cylinder's own u=0 meridian (+X).
    CHECK(close3(r.curves[0].u_ref, {1, 0, 0}));
    CHECK(close3(r.curves[0].v_ref, {0, 0, -1}));  // â×e₀ = ŷ×x̂ = −ẑ
    // The geometric identity: on both surfaces.
    for (int i = 0; i < 2; ++i)
        for (float t : {0.0f, 2.0f, 5.0f}) {
            const Vec3d p = r.curves[i].evaluate(t);
            CHECK(close((p - Vec3d{0, 0, 0}).length(), 2.0f, 1e-5f));
            CHECK(close(std::sqrt(p.x * p.x + p.z * p.z), 1.0f, 1e-5f));
        }

    // The cylinder domain: the horizontal Line2s at v = ±√3 (the
    // factory's v = the y coordinate of the axis frame).
    {
        const SegPC up = seg_of(r.a, 0), dn = seg_of(r.a, 1);
        CHECK(close(up.a.y, std::sqrt(3.0f)));
        CHECK(close(dn.a.y, -std::sqrt(3.0f)));
        CHECK(close(up.a.x, 0.0f));
        CHECK(close(up.b.x, kTwoPi));
    }
    // The sphere domain: the stored pole is +Y — PARALLEL to the
    // entry's canonical axis, so NO re-frame (the no-reframe path),
    // and the latitudes are v = arccos(±√3/2) = π/6, 5π/6.
    CHECK(!r.b.reframe_needed);
    {
        const SegPC up = seg_of(r.b, 0), dn = seg_of(r.b, 1);
        CHECK(close(up.a.y, kPi / 6.0f));
        CHECK(close(dn.a.y, 5.0f * kPi / 6.0f));
    }
    CHECK(r.provenance.is_exact());
}

TEST_CASE("G2-U1 cylinder×sphere degenerates: the tangent circle, the empty, the Viviani wall") {
    const FaceId f0 = 0;
    // R = r: the ONE tangent circle (the equator at the center).
    {
        const auto cyl = CAD_0::brep::make_cylinder(1.0f, 2.0f);
        const auto sph = CAD_0::brep::make_sphere(1.0f);
        const SsiResult r = CAD_0::brep::ssi_query(
            *cyl, f0, *sph, face_of_family(*sph, "sphere"), kTol);
        CHECK(r.kind == SsiResult::Kind::Circle);
        REQUIRE(r.curves.size() == 1);
        CHECK(close3(r.curves[0].center, {0, 0, 0}));
        CHECK(close(r.curves[0].radius, 1.0f));
        CHECK(r.message.find("tangent") != std::string::npos);
    }
    // R < r: the sphere strictly inside the cylinder — the empty.
    {
        const auto cyl = CAD_0::brep::make_cylinder(1.0f, 2.0f);
        const auto sph = CAD_0::brep::make_sphere(0.5f);
        const SsiResult r = CAD_0::brep::ssi_query(
            *cyl, f0, *sph, face_of_family(*sph, "sphere"), kTol);
        CHECK(r.kind == SsiResult::Kind::Empty);
        CHECK(r.message.find("contained") != std::string::npos);
    }
    // Non-coaxial: the Viviani quartic — the wall, named, with the
    // remedies (D2 option (i), the acceptance's own decision).
    {
        const auto cyl = CAD_0::brep::make_cylinder(1.0f, 2.0f);
        auto sph = CAD_0::brep::make_sphere(2.0f);
        CAD_0::brep::translate_body(*sph, {0.3f, 0.0f, 0.0f});
        const SsiResult r = CAD_0::brep::ssi_query(
            *cyl, f0, *sph, face_of_family(*sph, "sphere"), kTol);
        CHECK(r.kind == SsiResult::Kind::OutOfTable);
        CHECK(r.curves.empty());
        CHECK(r.message.find("Viviani") != std::string::npos);
        CHECK(r.message.find("non-coaxial") != std::string::npos);
        CHECK(r.message.find("Remedy") != std::string::npos);
    }
}

// ===========================================================================
// FAMILY 4 — cylinder×cylinder parallel, equal radii: the mid-plane lines
// ===========================================================================
TEST_CASE("G2-U1 cylinder×cylinder equal radii: the two lines in the mid-plane") {
    auto c1 = CAD_0::brep::make_cylinder(1.0f, 2.0f);
    auto c2 = CAD_0::brep::make_cylinder(1.0f, 2.0f);
    CAD_0::brep::translate_body(*c2, {0.8f, 0.0f, 0.0f});
    const FaceId w1 = face_of_family(*c1, "cylinder");
    const FaceId w2 = face_of_family(*c2, "cylinder");

    const SsiResult r = CAD_0::brep::ssi_query(*c1, w1, *c2, w2, kTol);

    CHECK(r.kind == SsiResult::Kind::LinePair);
    REQUIRE(r.curves.size() == 2);
    // The classical arithmetic (the ADR's own, in the canonical frame):
    // the mid-plane at x = d/2 = 0.4, the lines at z = ±√(r²−d²/4)
    // = ±√0.84, direction = the axis.
    const float zhalf = std::sqrt(1.0f - 0.16f);
    for (int i = 0; i < 2; ++i) {
        CHECK(r.curves[i].kind == CAD_0::brep::SsiCurve::Kind::Line);
        CHECK(close3(r.curves[i].dir, {0, 1, 0}));
        CHECK(close(r.curves[i].point.x, 0.4f));
        CHECK(close(std::abs(r.curves[i].point.z), zhalf));
    }
    CHECK(close(r.curves[0].point.z, +zhalf));
    CHECK(close(r.curves[1].point.z, -zhalf));
    // The geometric identity: both lines lie on BOTH cylinders.
    for (int i = 0; i < 2; ++i)
        for (float s : {-1.0f, 0.0f, 1.5f}) {
            const Vec3d p = r.curves[i].point + r.curves[i].dir * s;
            CHECK(close(std::sqrt(p.x * p.x + p.z * p.z), 1.0f, 1e-5f));
            CHECK(close(std::sqrt((p.x - 0.8f) * (p.x - 0.8f) + p.z * p.z),
                        1.0f, 1e-5f));
        }

    // The p-curves: the vertical Line2s at the derived azimuths,
    // spanning the face's OWN authored v-window (the factory wall's
    // [−h, +h] = [−1, +1]).
    const double xa = 0.4, za = zhalf;
    const double u1p = std::atan2(-za, xa);           // cyl 1, +z line
    const double u1m = std::atan2(+za, xa);           // cyl 1, −z line
    const auto norm2pi = [](double a) {
        double x = std::fmod(a, kTwoPi);
        if (x < 0) x += kTwoPi;
        return x;
    };
    {
        const SegPC up = seg_of(r.a, 0), dn = seg_of(r.a, 1);
        CHECK(close(up.a.x, norm2pi(u1p)));
        CHECK(close(dn.a.x, norm2pi(u1m)));
        CHECK(close(up.a.y, -1.0f));
        CHECK(close(up.b.y, +1.0f));
        CHECK(close(dn.a.y, -1.0f));
        CHECK(close(dn.b.y, +1.0f));
    }
    {
        // Cylinder 2's azimuths: measured from ITS axis at x = 0.8.
        const double u2p = std::atan2(-za, xa - 0.8);
        const double u2m = std::atan2(+za, xa - 0.8);
        const SegPC up = seg_of(r.b, 0), dn = seg_of(r.b, 1);
        CHECK(close(up.a.x, norm2pi(u2p)));
        CHECK(close(dn.a.x, norm2pi(u2m)));
    }
    CHECK(r.provenance.is_exact());
}

TEST_CASE("G2-U1 cylinder×cylinder degenerates: tangent, empty, coincident, the unequal-radii wall") {
    const FaceId f0 = 0;
    // d = 2r: the tangent double line.
    {
        auto c1 = CAD_0::brep::make_cylinder(1.0f, 2.0f);
        auto c2 = CAD_0::brep::make_cylinder(1.0f, 2.0f);
        CAD_0::brep::translate_body(*c2, {2.0f, 0, 0});
        const SsiResult r = CAD_0::brep::ssi_query(*c1, f0, *c2, f0, kTol);
        CHECK(r.kind == SsiResult::Kind::TangentLine);
        REQUIRE(r.curves.size() == 1);
        CHECK(close3(r.curves[0].point, {1.0f, 0.0f, 0.0f}));
        CHECK(close3(r.curves[0].dir, {0, 1, 0}));
    }
    // d > 2r: the empty.
    {
        auto c1 = CAD_0::brep::make_cylinder(1.0f, 2.0f);
        auto c2 = CAD_0::brep::make_cylinder(1.0f, 2.0f);
        CAD_0::brep::translate_body(*c2, {2.5f, 0, 0});
        const SsiResult r = CAD_0::brep::ssi_query(*c1, f0, *c2, f0, kTol);
        CHECK(r.kind == SsiResult::Kind::Empty);
        CHECK(r.message.find("separate") != std::string::npos);
    }
    // Coaxial, equal radii: the COINCIDENT degenerate (the co-radial
    // dedup rule's trigger, the coplanar rule's analog).
    {
        auto c1 = CAD_0::brep::make_cylinder(1.0f, 2.0f);
        auto c2 = CAD_0::brep::make_cylinder(1.0f, 2.0f);
        const SsiResult r = CAD_0::brep::ssi_query(*c1, f0, *c2, f0, kTol);
        CHECK(r.kind == SsiResult::Kind::Coincident);
        CHECK(r.message.find("co-radial") != std::string::npos);
    }
    // UNEQUAL radii: the entry's gate — the acceptance's corrected
    // disposition, named honestly (typable, out of the registered
    // entry; never "untypable").
    {
        auto c1 = CAD_0::brep::make_cylinder(1.0f, 2.0f);
        auto c2 = CAD_0::brep::make_cylinder(1.2f, 2.0f);
        CAD_0::brep::translate_body(*c2, {0.8f, 0, 0});
        const SsiResult r = CAD_0::brep::ssi_query(*c1, f0, *c2, f0, kTol);
        CHECK(r.kind == SsiResult::Kind::OutOfTable);
        CHECK(r.message.find("unequal radii") != std::string::npos);
        CHECK(r.message.find("two lines") != std::string::npos);
        CHECK(r.message.find("Remedy") != std::string::npos);
    }
}

// ===========================================================================
// The Z-axis cylinder — the extrude wall consults the same entry
// ===========================================================================
TEST_CASE("G2-U1 the extrude wall (axis Z): the same entry, the sphere re-framed to Z") {
    // A circular prism of radius 1, height 2, up +Z — its wall is a
    // CylinderZSurface, the boolean class's own cylinder.
    const std::vector<CAD_0::sdf::SketchChain> chains{
        CAD_0::sdf::SketchChain{
            {CAD_0::sdf::sketch_circle(Vec2f{0, 0}, 1.0f, false)}}};
    auto prism = CAD_0::brep::make_prism(chains, 2.0f, +1);
    const auto sph = CAD_0::brep::make_sphere(2.0f);
    const FaceId wf = face_of_family(*prism, "cylinder");
    const FaceId sf = face_of_family(*sph, "sphere");

    const SsiResult r = CAD_0::brep::ssi_query(*prism, wf, *sph, sf, kTol);

    CHECK(r.kind == SsiResult::Kind::CirclePair);
    REQUIRE(r.curves.size() == 2);
    // The ADR's literal numbers: the circles at z = ±√(R²−r²) = ±√3.
    CHECK(close3(r.curves[0].center, {0, 0, +std::sqrt(3.0f)}));
    CHECK(close3(r.curves[1].center, {0, 0, -std::sqrt(3.0f)}));
    CHECK(close(r.curves[0].radius, 1.0f));
    // The sphere re-frames its stored +Y pole to the Z axis.
    CHECK(r.b.reframe_needed);
    CHECK(close3(r.b.frame_pole, {0, 0, 1}));
    // The cylinder domain: v = z − z_base = ±√3.
    {
        const SegPC up = seg_of(r.a, 0), dn = seg_of(r.a, 1);
        CHECK(close(up.a.y, +std::sqrt(3.0f)));
        CHECK(close(dn.a.y, -std::sqrt(3.0f)));
    }
    // The sphere latitudes: arccos(±√3/2) = π/6, 5π/6.
    {
        const SegPC up = seg_of(r.b, 0), dn = seg_of(r.b, 1);
        CHECK(close(up.a.y, kPi / 6.0f));
        CHECK(close(dn.a.y, 5.0f * kPi / 6.0f));
    }

    // The non-parallel pair: the extrude's Z wall × the factory's Y
    // cylinder — the general quartic / the Steinmetz disposition,
    // typed in neither domain. THE WALL.
    {
        const auto cylY = CAD_0::brep::make_cylinder(1.0f, 2.0f);
        const SsiResult q = CAD_0::brep::ssi_query(
            *prism, wf, *cylY, face_of_family(*cylY, "cylinder"), kTol);
        CHECK(q.kind == SsiResult::Kind::OutOfTable);
        CHECK(q.message.find("quartic") != std::string::npos);
        CHECK(q.message.find("Steinmetz") != std::string::npos);
        CHECK(q.message.find("Remedy") != std::string::npos);
    }
}

// ===========================================================================
// The wall — the landed pairs, the registered-but-unreachable families,
// the provenance gate
// ===========================================================================
TEST_CASE("G2-U1 the wall: landed pairs, no-writer families, the fillet's registration, the provenance gate") {
    // plane×cylinder: landed in the exact subtract's own entry (G1
    // unit 4); the general dispatch arrives with the booleans.
    {
        const auto box = CAD_0::brep::make_box({1.0f, 2.0f, 3.0f});
        const auto cyl = CAD_0::brep::make_cylinder(1.0f, 2.0f);
        const SsiResult r = CAD_0::brep::ssi_query(
            *box, box_posx_face(*box), *cyl, face_of_family(*cyl, "cylinder"),
            kTol);
        CHECK(r.kind == SsiResult::Kind::OutOfTable);
        CHECK(r.message.find("landed") != std::string::npos);
        CHECK(r.message.find("subtract") != std::string::npos);
    }
    // plane×plane: Phase D's own class (the convex polygon booleans).
    {
        const auto b1 = CAD_0::brep::make_box({1.0f, 2.0f, 3.0f});
        const auto b2 = CAD_0::brep::make_box({2.0f, 1.0f, 1.0f});
        const SsiResult r = CAD_0::brep::ssi_query(
            *b1, box_posx_face(*b1), *b2, box_posx_face(*b2), kTol);
        CHECK(r.kind == SsiResult::Kind::OutOfTable);
        CHECK(r.message.find("Phase D") != std::string::npos);
    }
    // The cone: the mathematics is registered (row 8), NO writer —
    // the record is the thing under test (the one-face body).
    {
        const auto cone = one_face_body(
            std::make_shared<CAD_0::brep::ConeSurface>(1.0f, 2.0f));
        const auto sph = CAD_0::brep::make_sphere(2.0f);
        const SsiResult r = CAD_0::brep::ssi_query(
            *cone, 0, *sph, face_of_family(*sph, "sphere"), kTol);
        CHECK(r.kind == SsiResult::Kind::OutOfTable);
        CHECK(r.message.find("cone") != std::string::npos);
        CHECK(r.message.find("no writer") != std::string::npos);
    }
    // The torus: registered for the fillet's unit (rows 9–10), where
    // the construction consumes the entries — declared, not discovered.
    {
        const auto torus = one_face_body(
            std::make_shared<CAD_0::brep::TorusSurface>(2.0f, 0.5f));
        const auto sph = CAD_0::brep::make_sphere(2.0f);
        const SsiResult r = CAD_0::brep::ssi_query(
            *torus, 0, *sph, face_of_family(*sph, "sphere"), kTol);
        CHECK(r.kind == SsiResult::Kind::OutOfTable);
        CHECK(r.message.find("fillet") != std::string::npos);
        CHECK(r.message.find("registered") != std::string::npos);
    }
    // The provenance gate: the exact table consults EXACT faces — a
    // faceted operand (the bridge's own writing, approximated) is
    // refused with the remedy named.
    {
        const auto faceted = one_face_body(
            std::make_shared<CAD_0::brep::PlaneSurface>(
                Vec3d{1, 0, 0}, Vec3d{1, 0, 0}));
        faceted->solid.shell_mut().faces_mut()[0].set_provenance(
            CAD_0::brep::ProvenanceInfo::approximated(0.01));
        const auto sph = CAD_0::brep::make_sphere(2.0f);
        const SsiResult r = CAD_0::brep::ssi_query(
            *faceted, 0, *sph, face_of_family(*sph, "sphere"), kTol);
        CHECK(r.kind == SsiResult::Kind::OutOfTable);
        CHECK(r.message.find("approximated") != std::string::npos);
        CHECK(r.message.find("exact") != std::string::npos);
    }
    // A face id that does not exist: the caller bug, loud (the
    // query_face culture — diagnosis + remedy).
    {
        const auto sph1 = CAD_0::brep::make_sphere(2.0f);
        const auto sph2 = CAD_0::brep::make_sphere(2.0f);
        bool threw = false;
        try {
            (void)CAD_0::brep::ssi_query(*sph1, 7, *sph2, 0, kTol);
        } catch (const std::invalid_argument& e) {
            threw = true;
            CHECK(std::string(e.what()).find("no face with id") !=
                  std::string::npos);
        }
        CHECK(threw);
    }
}

// ===========================================================================
// The re-frame rule — the rebuild, the determinism, the budget
// ===========================================================================
TEST_CASE("G2-U1 the re-frame: the seam-only sphere rebuilds on the canonical axis") {
    auto sph = CAD_0::brep::make_sphere(2.0f);
    const Vec3d pole = Vec3d{1.0f, 1.0f, 1.0f}.normalized();
    const Vec3d azimuth = Vec3d{1.0f, -1.0f, 0.0f}.normalized();  // ⊥ pole

    const auto rec = CAD_0::brep::reframe_sphere_face(
        *sph, 0, pole, azimuth, kTol);

    // The frame is the caller's axis, orthonormalized on the record.
    CHECK(close3(rec.pole, pole));
    CHECK(close(rec.pole.dot(rec.azimuth), 0.0f));
    CHECK(close(rec.azimuth.length(), 1.0f));

    // The surface: SAME mathematical object (center, radius), the NEW
    // frame — every evaluated point still lies on the sphere, and the
    // poles sit at center ± R·pole.
    const auto* surf = dynamic_cast<const CAD_0::brep::SphereSurface*>(
        sph->solid.shell().faces()[0].surface());
    REQUIRE(surf != nullptr);
    CHECK(close3(surf->pole(), pole));
    for (float u : {0.0f, 1.3f, 4.0f})
        for (float v : {0.3f, 1.2f, 2.7f}) {
            const Vec3d p = surf->evaluate(u, v);
            CHECK(close((p - Vec3d{0, 0, 0}).length(), 2.0f, 1e-5f));
        }
    CHECK(close3(surf->evaluate(0.0f, 0.0f), (pole * 2.0f)));
    CHECK(close3(surf->evaluate(0.0f, kPi), (pole * -2.0f)));

    // The seam edge's vertices MOVED to the new poles (the seam is
    // derived state; the ADR-0023 pattern re-wires it, it does not
    // survive the frame).
    CHECK(close3(sph->vertices[0].position(), pole * 2.0f));
    CHECK(close3(sph->vertices[1].position(), pole * -2.0f));

    // The seam p-curves: the SAME slit-rectangle form — the forward
    // copy the RIGHT edge (2π,0)→(2π,π), the reversed copy the LEFT
    // edge (0,π)→(0,0), material on the left (the factory's own
    // authoring, one form for every frame).
    const auto fi = CAD_0::brep::query_face(*sph, 0);
    REQUIRE(fi.loops.size() == 1);
    REQUIRE(fi.loops[0].coedges.size() == 2);
    const auto& cf = fi.loops[0].coedges[0];
    const auto& cr = fi.loops[0].coedges[1];
    REQUIRE(cf.kind == CAD_0::brep::PCurveInfo::Kind::Line);
    CHECK(close(cf.a.x, kTwoPi));
    CHECK(close(cf.a.y, 0.0f));
    CHECK(close(cf.b.x, kTwoPi));
    CHECK(close(cf.b.y, kPi));
    CHECK(cf.interior_left);
    CHECK(close(cr.a.x, 0.0f));
    CHECK(close(cr.a.y, kPi));
    CHECK(close(cr.b.x, 0.0f));
    CHECK(close(cr.b.y, 0.0f));

    // The structure is untouched: 2V/1E/1F, χ = 2, closed orientable.
    CHECK(sph->vertex_count() == 2);
    CHECK(sph->edge_count() == 1);
    CHECK(sph->face_count() == 1);
    CHECK(CAD_0::brep::euler_characteristic(*sph) == 2);
    CHECK(CAD_0::brep::is_closed_orientable(*sph));
}

TEST_CASE("G2-U1 the re-frame is a pure function of the recipe: bit-identical at regeneration") {
    const Vec3d pole = Vec3d{0.3f, -0.8f, 0.52f}.normalized();
    // An azimuth ⊥ the pole, derived the way the entries derive theirs
    // (the cross product with a reference axis, normalized).
    const Vec3d azimuth = Vec3d{-0.8f, -0.3f, 0.0f}.normalized();

    std::string d1, d2;
    for (int rep = 0; rep < 2; ++rep) {
        auto sph = CAD_0::brep::make_sphere(1.7f);
        CAD_0::brep::reframe_sphere_face(*sph, 0, pole, azimuth, kTol);
        // The digest: the re-framed face's own query answer (the
        // unit-3 oracle's shape — every number a hex float).
        std::string out;
        const auto fi = CAD_0::brep::query_face(*sph, 0);
        const auto* surf = dynamic_cast<const CAD_0::brep::SphereSurface*>(
            sph->solid.shell().faces()[0].surface());
        av3(out, surf->pole());
        av3(out, surf->azimuth());
        av3(out, surf->origin());
        af(out, surf->radius());
        av3(out, sph->vertices[0].position());
        av3(out, sph->vertices[1].position());
        for (const auto& loop : fi.loops)
            for (const auto& ce : loop.coedges) {
                af(out, static_cast<double>(ce.edge));
                af(out, ce.reversed ? 1.0 : 0.0);
                af(out, static_cast<double>(static_cast<int>(ce.kind)));
                av2(out, ce.a);
                av2(out, ce.b);
                af(out, ce.interior_left ? 1.0 : 0.0);
            }
        (rep == 0 ? d1 : d2) = out;
    }
    CHECK(d1 == d2);
    CHECK(!d1.empty());
}

TEST_CASE("G2-U1 the frame budget: the incompatible family that arrives later finds the wall") {
    const FaceId f0 = 0;
    // The budgeted record: a sphere face with a typed latitude cut of
    // the STORED +Y family (the record-under-test — the gate honors
    // the record, it does not launder it).
    auto cut_sph = budgeted_sphere(2.0f);
    const auto box = CAD_0::brep::make_box({1.0f, 2.0f, 3.0f});

    // A cut plane ⊥ X: the canonical axis is INCOMPATIBLE with the
    // budgeted +Y family — the wall, the budget named, the remedy
    // carried (option (i), coherent to the end).
    const SsiResult r = CAD_0::brep::ssi_query(
        *box, box_posx_face(*box), *cut_sph, f0, kTol);
    CHECK(r.kind == SsiResult::Kind::OutOfTable);
    CHECK(r.message.find("frame budget") != std::string::npos);
    CHECK(r.message.find("one axis family") != std::string::npos);
    CHECK(r.message.find("Remedy") != std::string::npos);

    // The same budgeted face against a cut plane ⊥ Y — the COMPATIBLE
    // family: no re-frame is needed at all, the entry answers in the
    // stored domain (the budget never fires).
    {
        const auto boxY = CAD_0::brep::make_box({3.0f, 1.0f, 2.0f});
        FaceId pf = static_cast<FaceId>(-1);
        for (const auto& f : boxY->solid.shell().faces()) {
            const auto* pl =
                dynamic_cast<const CAD_0::brep::PlaneSurface*>(f.surface());
            if (pl && close3(pl->normal_vec(), {0, 1, 0})) pf = f.id();
        }
        REQUIRE(pf != static_cast<FaceId>(-1));
        const SsiResult ok = CAD_0::brep::ssi_query(
            *boxY, pf, *cut_sph, f0, kTol);
        CHECK(ok.kind == SsiResult::Kind::Circle);
        CHECK(!ok.b.reframe_needed);
    }

    // The standalone re-frame on the budgeted face: the wall too —
    // the cut face's re-frame is the boolean unit's own step (unit 2
    // re-authors the cuts beside the frame), the refusal says so.
    bool threw = false;
    try {
        (void)CAD_0::brep::reframe_sphere_face(
            *cut_sph, f0, Vec3d{1, 0, 0}, Vec3d{0, 1, 0}, kTol);
    } catch (const std::invalid_argument& e) {
        threw = true;
        const std::string m = e.what();
        CHECK(m.find("frame budget") != std::string::npos);
        CHECK(m.find("Remedy") != std::string::npos);
    }
    CHECK(threw);

    // The re-frame's own argument gates: a non-sphere face refuses
    // naming the family; a bad face id refuses naming the range.
    {
        auto cyl = CAD_0::brep::make_cylinder(1.0f, 2.0f);
        bool t1 = false, t2 = false;
        try {
            (void)CAD_0::brep::reframe_sphere_face(
                *cyl, 0, Vec3d{0, 1, 0}, Vec3d{1, 0, 0}, kTol);
        } catch (const std::invalid_argument& e) {
            t1 = true;
            CHECK(std::string(e.what()).find("cylinder") != std::string::npos);
        }
        try {
            (void)CAD_0::brep::reframe_sphere_face(
                *cyl, 9, Vec3d{0, 1, 0}, Vec3d{1, 0, 0}, kTol);
        } catch (const std::invalid_argument& e) {
            t2 = true;
            CHECK(std::string(e.what()).find("no face with id") !=
                  std::string::npos);
        }
        CHECK(t1);
        CHECK(t2);
    }
}

// ===========================================================================
// Determinism — the query is a pure function of the operands
// ===========================================================================
TEST_CASE("G2-U1 determinism: the answers are bit-identical across re-runs and rebuilds") {
    const auto make_operands = [] {
        struct Ops {
            std::unique_ptr<BRepBody> box;
            std::unique_ptr<BRepBody> sph;
            std::unique_ptr<BRepBody> sph2;
            std::unique_ptr<BRepBody> cyl;
            std::unique_ptr<BRepBody> cyl2;
        };
        Ops o;
        o.box = CAD_0::brep::make_box({1.0f, 2.0f, 3.0f});
        o.sph = CAD_0::brep::make_sphere(2.0f);
        o.sph2 = CAD_0::brep::make_sphere(1.0f);
        CAD_0::brep::translate_body(*o.sph2, {2.5f, 0, 0});
        o.cyl = CAD_0::brep::make_cylinder(1.0f, 2.0f);
        o.cyl2 = CAD_0::brep::make_cylinder(1.0f, 2.0f);
        CAD_0::brep::translate_body(*o.cyl2, {0.8f, 0, 0});
        return o;
    };

    std::string digests[2][4];
    for (int rep = 0; rep < 2; ++rep) {
        const auto ops = make_operands();
        const FaceId bp = box_posx_face(*ops.box);
        const FaceId sf = face_of_family(*ops.sph, "sphere");
        const FaceId sf2 = face_of_family(*ops.sph2, "sphere");
        const FaceId wf = face_of_family(*ops.cyl, "cylinder");
        const FaceId wf2 = face_of_family(*ops.cyl2, "cylinder");
        digests[rep][0] = ssi_digest(CAD_0::brep::ssi_query(
            *ops.box, bp, *ops.sph, sf, kTol));
        digests[rep][1] = ssi_digest(CAD_0::brep::ssi_query(
            *ops.sph, sf, *ops.sph2, sf2, kTol));
        digests[rep][2] = ssi_digest(CAD_0::brep::ssi_query(
            *ops.cyl, wf, *ops.sph, sf, kTol));
        digests[rep][3] = ssi_digest(CAD_0::brep::ssi_query(
            *ops.cyl, wf, *ops.cyl2, wf2, kTol));
    }
    for (int e = 0; e < 4; ++e)
        CHECK(digests[0][e] == digests[1][e]);
}
