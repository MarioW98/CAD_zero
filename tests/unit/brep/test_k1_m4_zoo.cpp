// tests/unit/brep/test_k1_m4_zoo.cpp
//
// K1 m4 — LO ZOO: the adversarial corpus of the degenerates
// (ADR-0041 D1.2 — the robustness metric's own suite member; the
// fillet-tail debt's registered landing, docs/status/README.md's "one
// item remains, with its seat" entry).
//
// THE LAW (D1.2, verbatim in spirit): for every pair of families the
// exact path can compose, the degenerate DISPOSITIONS are pinned in an
// adversarial corpus — internal and external tangency, coincidence and
// co-radial cases, containment and empty intersection, and the
// NEAR-BAND cases, the inputs constructed ON PURPOSE at the band edges
// where the class changes. Every band case carries its PINNED
// DISPOSITION — which path answers, which tolerance is declared —
// never a disposition discovered at run time. FUZZ IS THE GROWTH
// METHOD: parameter-space fuzzing over the corpus's families and
// bands, deterministic by seed (ADR-0014 — the fuzz is reproducible
// or it is not evidence); every crash the fuzz finds becomes a pinned
// row — never a silent fix.
//
// THE COHORT (this morso's reach, the seats of the rest declared in
// the register's m4 row): the TABLE'S OWN TYPED PAIRS — the four
// old-family entries (plane×sphere, sphere×sphere, cylinder×sphere,
// cylinder×cylinder) in the typed corpus below, and the ellipsoid's
// three (K1 m3's own near-band corpus landed with the family; here
// they join the FUZZ sweep — the framework is generic over the
// table). The CONSUMERS' pairs — the subtract's plane×cylinder, Phase
// D's plane×plane, the fillet's plane×torus and cylinder×torus at
// their tangent limits — ride the CONSUMER APIs, not the query: their
// bands are the consumers' own (fillet_exits_annulus, fillet_eats_wall
// — the m3 refusal bands), the SECOND cohort's bite, seats declared.
// The cone's pairs are unreachable (no writer — the census's own row).
//
// THE CATCHES THIS ZOO HAS PINNED (each cured with its red
// counterproof in the entry's own file, this morso's fix commit):
//   1. sphere×sphere's INTERNAL tangency point under the swapped
//      operands (s1 the smaller sphere): the point sat at the near
//      foot, ON s1 but a full radius INSIDE s2 — the operand-order
//      sweep's catch;
//   2. plane×sphere's stored-frame latitude height: the branch read
//      the PLANE ORIGIN's pole-projection, so a plane tilted within
//      the pole band with a distant in-plane origin authored a
//      latitude radians off the section — the answer moved with the
//      placement (not a pure function of the geometry);
//   3. THE BAND-EDGE CRACK (every entry's own site): the band tests
//      written as the subtraction |a−b| ≤ tol crack on the member
//      authored as fl(b ± 1·tol) — the rounded difference reads
//      1.0000000001e-06 > tol and the member falls through the
//      crack between the neighboring gates into the WRONG branch:
//      plane×sphere k=+1 (the radius-0 p-curve the authoring
//      contract refuses — the CRASH), sphere×sphere k=+1 external
//      (the radical-plane branch with ρ = 0) and the double-edge
//      dedup (the concentric empty instead of Coincident),
//      cylinder×sphere k=−1 (the pair instead of the tangent
//      circle), cylinder×cylinder k=+1 (two collapsed lines
//      instead of the tangent double line) and its equal-radii
//      gate (the wall at the edge), plane×ellipsoid c=0.75
//      k=±1 (the crash and the class flip), cylinder×ellipsoid
//      k=−1 (the double-counted equator), ellipsoid×ellipsoid's
//      dedup edge (the wall instead of Coincident) — cured at every
//      site with the ONE cancellation-free comparison form
//      (a ≤ b+tol && a ≥ b−tol), the same shape the separate/empty
//      gates already read;
//   4. THE TOUCH POINT'S ORDER PURITY (sphere×sphere at the band
//      members): the point read ±R1 from the LEADING operand, so
//      the two operand orders answered points k·tol apart — the
//      order sweep's catch one tie over; cured with the canonical
//      read (the BIGGER sphere's foot for external, the SMALLER's
//      far foot for internal, whichever operand leads).
//
// THE NEAR-BAND DISCIPLINE (ADR-0035 D2's taxonomy, the m4
// instruction's own words): the near-band cases ride the TYPED bands
// — the declared Layer-3 tolerance each classification reads at its
// site (the linear bands and the cosine bands of the entries) — never
// a global epsilon. The corpus's members sit at k = ±0.5·tol (inside
// the band), k = ±1·tol (AT the edge), k = ±2·tol (outside), the
// canonical key's resolutions pinned at every tie.
#include <doctest/doctest.h>

#include "CAD_0/brep/analytic.hpp"
#include "CAD_0/brep/factory.hpp"
#include "CAD_0/brep/ssi.hpp"
#include "CAD_0/brep/subtract.hpp"
#include "CAD_0/brep/topology.hpp"
#include "property/property_prng.hpp"

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <memory>
#include <string>

namespace {

using CAD_0::brep::BRepBody;
using CAD_0::brep::FaceId;
using CAD_0::brep::SsiResult;
using CAD_0::math::Vec3d;

// The declared Layer-3 decision tolerance — the same value every
// classification in ssi.cpp reads (ssi.hpp's own contract); the zoo
// reads THE DECLARED VALUE, never a private one.
constexpr double kTol = 1e-6;

// ---------------------------------------------------------------------------
// The authoring helpers (the G2-U1 file's own forms, verbatim in spirit)
// ---------------------------------------------------------------------------

// A one-face body carrying an arbitrary analytic surface — the query is
// surface-level on these records.
std::unique_ptr<BRepBody> one_face_body(
    std::shared_ptr<CAD_0::brep::SurfaceGeometry> surf) {
    auto body = std::make_unique<BRepBody>();
    auto& shell = body->solid.shell_mut();
    shell.set_closed(false);
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

// ---------------------------------------------------------------------------
// The REGISTERED band discipline, re-derived here as the independent
// expectation (D1.5's spirit: the kernel's answer is checked against
// the registered discipline, never against the kernel's own output)
// ---------------------------------------------------------------------------

// plane×sphere: |d| vs R — the tangent band ||d|−R| ≤ tol resolves the
// tie to the POINT, never a shrunk circle (the canonical key).
SsiResult::Kind expect_plane_sphere(double ad, double R, double tol) {
    if (std::abs(ad) > R + tol) return SsiResult::Kind::Empty;
    if (std::abs(std::abs(ad) - R) <= tol) return SsiResult::Kind::Point;
    return SsiResult::Kind::Circle;
}

// sphere×sphere: the concentric band first (D ≤ tol: the co-radial
// dedup when the radii tie within tol, the nested empty otherwise),
// then the external edge D = R1+R2, then the internal edge
// D = |R1−R2| (R1 ≠ R2), then the nested/separate empties, else the
// radical-plane circle.
SsiResult::Kind expect_sphere_sphere(double R1, double R2, double D,
                                     double tol) {
    if (D <= tol)
        return std::abs(R1 - R2) <= tol ? SsiResult::Kind::Coincident
                                        : SsiResult::Kind::Empty;
    if (D > R1 + R2 + tol) return SsiResult::Kind::Empty;
    if (std::abs(D - (R1 + R2)) <= tol) return SsiResult::Kind::Point;
    if (std::abs(D - std::abs(R1 - R2)) <= tol && R1 != R2)
        return SsiResult::Kind::Point;
    if (D < std::abs(R1 - R2) - tol) return SsiResult::Kind::Empty;
    return SsiResult::Kind::Circle;
}

// cylinder×sphere: the coaxiality band (the linear perp) first —
// beyond it the Viviani wall; then the radii band |R−r| ≤ tol (the
// tangent circle at the tie), the contained empty, else the pair.
SsiResult::Kind expect_cyl_sphere(double r, double R, double perp,
                                  double tol) {
    if (perp > tol) return SsiResult::Kind::OutOfTable;
    if (R < r - tol) return SsiResult::Kind::Empty;
    if (std::abs(R - r) <= tol) return SsiResult::Kind::Circle;
    return SsiResult::Kind::CirclePair;
}

// cylinder×cylinder (the authorable pairs are exactly parallel — the
// classes pin the axes): the equal-radii gate first (the corrected
// refusal beyond the band), then the coincident band d ≤ tol, the
// tangent band |d−2r| ≤ tol, the separate empty, else the line pair.
SsiResult::Kind expect_cyl_cyl(double r1, double r2, double d,
                               double tol) {
    if (std::abs(r1 - r2) > tol) return SsiResult::Kind::OutOfTable;
    if (d <= tol) return SsiResult::Kind::Coincident;
    if (std::abs(d - 2.0 * r1) <= tol) return SsiResult::Kind::TangentLine;
    if (d > 2.0 * r1 + tol) return SsiResult::Kind::Empty;
    return SsiResult::Kind::LinePair;
}

// plane×ellipsoid (⊥ the stored axis): the sphere's row 4 one family
// over — |h| vs c, the tie to the POINT.
SsiResult::Kind expect_plane_ellipsoid(double ah, double c, double tol) {
    if (std::abs(ah) > c + tol) return SsiResult::Kind::Empty;
    if (std::abs(std::abs(ah) - c) <= tol) return SsiResult::Kind::Point;
    return SsiResult::Kind::Circle;
}

// cylinder×ellipsoid (coaxial): the coaxiality band (perp) first
// — beyond it the offset wall; then r vs a — the tangent circle at
// the tie, the empty beyond, else the pair.
SsiResult::Kind expect_cyl_ellipsoid(double r, double a, double perp,
                                     double tol) {
    if (perp > tol) return SsiResult::Kind::OutOfTable;
    if (r > a + tol) return SsiResult::Kind::Empty;
    if (r < a + tol && r > a - tol) return SsiResult::Kind::Circle;
    return SsiResult::Kind::CirclePair;
}

// ellipsoid×ellipsoid (the axes parallel by authoring): the co-radial
// dedup when the semiaxes AND the center tie within tol; else the
// wall (the quartic — no radical plane for spheroids).
SsiResult::Kind expect_ellipsoid_ellipsoid(double a1, double c1,
                                           double a2, double c2,
                                           double dc, double tol) {
    const bool same = std::abs(a1 - a2) <= tol &&
                      std::abs(c1 - c2) <= tol && dc <= tol;
    return same ? SsiResult::Kind::Coincident
                : SsiResult::Kind::OutOfTable;
}

// ---------------------------------------------------------------------------
// The membership checkers — the geometric identity, each with its
// TYPED bound: 1e-9 for the closed forms OUTSIDE the bands (the
// arithmetic is exact); the band's own multiples of the declared tol
// INSIDE a band (the typed geometry's declared residual, never a
// global epsilon)
// ---------------------------------------------------------------------------

double sphere_resid(Vec3d p, Vec3d c, double R) {
    return std::abs((p - c).length() - R);
}
double plane_resid(Vec3d p, Vec3d o, Vec3d n) {
    return std::abs((p - o).dot(n));
}
double cylz_resid(Vec3d p, double cx, double cy, double r) {
    return std::abs(std::hypot(p.x - cx, p.y - cy) - r);
}
double spheroid_resid(Vec3d p, Vec3d ctr, double a, double c) {
    return std::abs(((p.x - ctr.x) * (p.x - ctr.x) +
                     (p.y - ctr.y) * (p.y - ctr.y)) / (a * a) +
                    (p.z - ctr.z) * (p.z - ctr.z) / (c * c) - 1.0);
}

// The fuzz's band-aware slack: inside a typed band the answer's
// geometry may sit off the exact surface by the band's own width
// (≤ tol) plus fp noise — the bound is a multiple of the DECLARED
// tolerance, each use documented at its site below.
constexpr double kBandSlack = 2.0 * kTol + 1e-9;

// ---------------------------------------------------------------------------
// The zoo's verdict digest (ADR-0014): every number as a hex float
// (the G2-U1 ssi_digest discipline), the kind ids, the families, the
// message. The p-curve payloads are EXCLUDED here — their
// atan2/acos-dependent bytes stay pinned by the G2-U1 determinism
// test's own digest; the zoo's digest testifies the DISPOSITION and
// the 3D answer bit for bit.
// ---------------------------------------------------------------------------
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
std::string zoo_digest(const SsiResult& r) {
    std::string out;
    af(out, static_cast<double>(static_cast<int>(r.kind)));
    out += r.family_a; out += '|'; out += r.family_b; out += '|';
    out += r.message; out += '|';
    av3(out, r.point);
    for (const auto& c : r.curves) {
        af(out, static_cast<double>(static_cast<int>(c.kind)));
        av3(out, c.center); af(out, c.radius);
        av3(out, c.u_ref); av3(out, c.v_ref);
        av3(out, c.point); av3(out, c.dir);
    }
    return out;
}
std::uint64_t fnv1a(const std::string& s) {
    std::uint64_t h = 1469598103934665603ull;
    for (unsigned char ch : s) {
        h ^= ch;
        h *= 1099511628211ull;
    }
    return h;
}

} // namespace

// ===========================================================================
// THE TYPED CORPUS — plane×sphere: the near-band at the tangent edge,
// the pole band, the operand-order sweep, the seam meridian
// ===========================================================================
TEST_CASE("K1 m4 zoo plane×sphere: the near-band citizens at the "
          "tangent edge, the pole band, the order sweep") {
    const double R = 2.0;
    const auto sph = CAD_0::brep::make_sphere(R);
    const FaceId sf = 0;

    SUBCASE("the near-band: inside, AT the edge, outside — the tie to "
            "the POINT, never a shrunk circle") {
        // The plane ⊥ the stored pole (+Y): the stored frame serves,
        // the band arithmetic reads d directly.
        for (double k : {-0.5, +0.5, -1.0, +1.0}) {
            // INSIDE the band (|k| ≤ 1): the POINT — the canonical
            // key's own resolution at every tie.
            const auto pl = one_face_body(
                std::make_shared<CAD_0::brep::PlaneSurface>(
                    Vec3d{0, R + k * kTol, 0}, Vec3d{0, 1, 0}));
            const SsiResult r = CAD_0::brep::ssi_query(*pl, 0, *sph, sf, kTol);
            INFO("k = " << k);
            CHECK(r.kind == SsiResult::Kind::Point);
            // The point at the sphere's pole — ON the sphere exact,
            // within the band's own width of the plane (the tie's
            // canonical resolution: the POINT at the pole, never a
            // shrunk circle; the typed bound |k|·tol).
            CHECK(sphere_resid(r.point, Vec3d{0, 0, 0}, R) < 1e-9);
            CHECK(plane_resid(r.point, Vec3d{0, R + k * kTol, 0},
                              Vec3d{0, 1, 0}) <
                  std::abs(k) * kTol + 1e-9);
        }
        // THE CRACK COUNTERPROOF (catch 3, the red said as numbers):
        // the pre-cure tie test read |fl(R + 1·tol) − R| — the
        // subtraction's cancellation — which on this member's double
        // is 1.000000000139778e-06 > tol: the member fell through
        // the crack into the CIRCLE branch, where R² − d² < 0
        // clamped to the radius-0 p-curve the authoring contract
        // refuses (the crash). The comparison form — the cure —
        // reads the same double TRUE (IEEE-754 deterministic, the
        // ADR-0014 ground).
        CHECK(std::abs((R + 1.0 * kTol) - R) > kTol);
        {
            // OUTSIDE, the near side (k = −2): a REAL circle of radius
            // √(R²−d²) ≈ √(4R·tol) — small but genuine, its points ON
            // the sphere (the naive shrunk-circle red's own class).
            const auto pl = one_face_body(
                std::make_shared<CAD_0::brep::PlaneSurface>(
                    Vec3d{0, R - 2 * kTol, 0}, Vec3d{0, 1, 0}));
            const SsiResult r = CAD_0::brep::ssi_query(*pl, 0, *sph, sf, kTol);
            CHECK(r.kind == SsiResult::Kind::Circle);
            REQUIRE(r.curves.size() == 1);
            CHECK(r.curves[0].radius > 2.0 * std::sqrt(R * kTol) * 0.9);
            CHECK(r.curves[0].radius < 2.0 * std::sqrt(R * kTol) * 1.1);
            for (double t : {0.7, 2.1, 4.4})
                CHECK(sphere_resid(r.curves[0].evaluate(t),
                                   Vec3d{0, 0, 0}, R) < 1e-9);
        }
        {
            // OUTSIDE, the far side (k = +2): the honest empty.
            const auto pl = one_face_body(
                std::make_shared<CAD_0::brep::PlaneSurface>(
                    Vec3d{0, R + 2 * kTol, 0}, Vec3d{0, 1, 0}));
            const SsiResult r = CAD_0::brep::ssi_query(*pl, 0, *sph, sf, kTol);
            CHECK(r.kind == SsiResult::Kind::Empty);
            CHECK(r.message.find("separate") != std::string::npos);
        }
        {
            // The SOUTH tangent: the symmetric member (the band is
            // symmetric in |d| — the point at the south pole).
            const auto pl = one_face_body(
                std::make_shared<CAD_0::brep::PlaneSurface>(
                    Vec3d{0, -R - 0.5 * kTol, 0}, Vec3d{0, 1, 0}));
            const SsiResult r = CAD_0::brep::ssi_query(*pl, 0, *sph, sf, kTol);
            CHECK(r.kind == SsiResult::Kind::Point);
            CHECK(r.point.y < 0.0);
        }
    }

    SUBCASE("the operand-order sweep: the swapped query answers the "
            "same geometry") {
        const auto pl = one_face_body(
            std::make_shared<CAD_0::brep::PlaneSurface>(
                Vec3d{0, R - 0.5 * kTol, 0}, Vec3d{0, 1, 0}));
        const SsiResult r1 = CAD_0::brep::ssi_query(*pl, 0, *sph, sf, kTol);
        const SsiResult r2 = CAD_0::brep::ssi_query(*sph, sf, *pl, 0, kTol);
        CHECK(r1.kind == r2.kind);
        CHECK(r1.point == r2.point);
        CHECK(r2.family_a == "sphere");
        CHECK(r2.family_b == "plane");
    }

    SUBCASE("the pole band: the stored frame serves inside, the "
            "re-frame is declared just outside") {
        // The sphere's stored pole is +Y (make_sphere's own frame).
        // INSIDE the pole band (θ ≤ √(2·tol)): the stored frame
        // serves — the latitude authored at the SECTION's mean height
        // (the second catch's own cure, pinned here at the band's
        // edge), the re-frame NOT requested.
        const double th_in = 0.5 * std::sqrt(2.0 * kTol);
        const Vec3d n_in{std::sin(th_in), std::cos(th_in), 0.0};
        const Vec3d o_in{0.0, 0.6 * R, 0.0};
        const auto pl_in = one_face_body(
            std::make_shared<CAD_0::brep::PlaneSurface>(o_in, n_in));
        const SsiResult rin =
            CAD_0::brep::ssi_query(*pl_in, 0, *sph, sf, kTol);
        CHECK(rin.kind == SsiResult::Kind::Circle);
        CHECK(!rin.b.reframe_needed);
        {
            // The section's mean height −d·(n·p̂), derived here.
            const double d = (Vec3d{0, 0, 0} - o_in).dot(n_in);
            const double k = -d * n_in.y;
            CHECK(rin.b.pcurves[0].segment.a.y ==
                  doctest::Approx(std::acos(k / R)).epsilon(1e-9));
        }
        // JUST OUTSIDE the pole band: the re-frame DERIVED and
        // declared (the frame serves the geometry, never the
        // reverse) — the latitude exact in the re-framed domain.
        const double th_out = 1.5 * std::sqrt(2.0 * kTol);
        const Vec3d n_out{std::sin(th_out), std::cos(th_out), 0.0};
        const auto pl_out = one_face_body(
            std::make_shared<CAD_0::brep::PlaneSurface>(
                Vec3d{0.0, 0.6 * R, 0.0}, n_out));
        const SsiResult rout =
            CAD_0::brep::ssi_query(*pl_out, 0, *sph, sf, kTol);
        CHECK(rout.kind == SsiResult::Kind::Circle);
        CHECK(rout.b.reframe_needed);
        CHECK(rout.b.frame_pole == n_out);
    }

    SUBCASE("the seam meridian: the tangent point ON the u = 0 "
            "meridian — the seam never moves the geometry") {
        // The plane ⊥ +X at distance R: the tangent point (R, 0, 0)
        // sits at v = π/2, u = 0 of the stored frame — ON the seam
        // meridian. The disposition: the POINT, unchanged.
        const auto pl = one_face_body(
            std::make_shared<CAD_0::brep::PlaneSurface>(
                Vec3d{R, 0, 0}, Vec3d{1, 0, 0}));
        const SsiResult r = CAD_0::brep::ssi_query(*pl, 0, *sph, sf, kTol);
        CHECK(r.kind == SsiResult::Kind::Point);
        CHECK(std::abs(r.point.x - R) < 1e-9);
        CHECK(std::abs(r.point.y) < 1e-9);
        CHECK(std::abs(r.point.z) < 1e-9);
    }
}

// ===========================================================================
// THE TYPED CORPUS — sphere×sphere: the near-band at both tangency
// edges, the concentric band, the order sweep (the first catch's
// home), the frame-independent coincident
// ===========================================================================
TEST_CASE("K1 m4 zoo sphere×sphere: the near-band at both tangency "
          "edges, the concentric band, the order sweep") {
    const FaceId f0 = 0;

    SUBCASE("the external edge: inside, AT, outside — the tie to the "
            "POINT on BOTH surfaces") {
        const double R1 = 2.0, R2 = 1.0;
        for (double k : {-0.5, +0.5, -1.0, +1.0}) {
            auto s1 = CAD_0::brep::make_sphere(R1);
            auto s2 = CAD_0::brep::make_sphere(R2);
            CAD_0::brep::translate_body(
                *s2, Vec3d{R1 + R2 + k * kTol, 0, 0});
            const SsiResult r =
                CAD_0::brep::ssi_query(*s1, f0, *s2, f0, kTol);
            INFO("k = " << k);
            CHECK(r.kind == SsiResult::Kind::Point);
            // The canonical touch point (catch 4's cure): EXTERNAL —
            // the BIGGER sphere's foot toward the smaller. Here s1
            // the bigger (R1 = 2 > R2 = 1): ON s1 exact, within the
            // band's own width of s2 (the typed bound |k|·tol).
            CHECK(sphere_resid(r.point, Vec3d{0, 0, 0}, R1) < 1e-9);
            CHECK(sphere_resid(r.point, Vec3d{R1 + R2 + k * kTol, 0, 0},
                               R2) < std::abs(k) * kTol + 1e-9);
        }
        // THE CRACK COUNTERPROOF (catch 3, this entry's external
        // site): the pre-cure tie test read |fl(R1+R2+tol) −
        // (R1+R2)| = 1.000000000139778e-06 > tol — the k = +1 member
        // fell into the radical-plane branch with ρ = √max(0, …) =
        // 0, a wrong class with degenerate geometry; the comparison
        // form reads the same double TRUE.
        CHECK(std::abs((R1 + R2 + 1.0 * kTol) - (R1 + R2)) > kTol);
        {
            // k = −2: a REAL circle (the radical plane's arithmetic,
            // re-derived): h = (D²+R1²−R2²)/2D, ρ = √(R1²−h²).
            const double D = R1 + R2 - 2 * kTol;
            auto s1 = CAD_0::brep::make_sphere(R1);
            auto s2 = CAD_0::brep::make_sphere(R2);
            CAD_0::brep::translate_body(*s2, Vec3d{D, 0, 0});
            const SsiResult r =
                CAD_0::brep::ssi_query(*s1, f0, *s2, f0, kTol);
            CHECK(r.kind == SsiResult::Kind::Circle);
            const double h = (D * D + R1 * R1 - R2 * R2) / (2.0 * D);
            const double rho = std::sqrt(R1 * R1 - h * h);
            REQUIRE(r.curves.size() == 1);
            CHECK(std::abs(r.curves[0].radius - rho) < 1e-9);
            CHECK(std::abs(r.curves[0].center.x - h) < 1e-9);
            for (double t : {0.9, 2.5, 5.0}) {
                CHECK(sphere_resid(r.curves[0].evaluate(t),
                                   Vec3d{0, 0, 0}, R1) < 1e-9);
                CHECK(sphere_resid(r.curves[0].evaluate(t),
                                   Vec3d{D, 0, 0}, R2) < 1e-9);
            }
        }
        {
            // k = +2: separate.
            auto s1 = CAD_0::brep::make_sphere(R1);
            auto s2 = CAD_0::brep::make_sphere(R2);
            CAD_0::brep::translate_body(
                *s2, Vec3d{R1 + R2 + 2 * kTol, 0, 0});
            const SsiResult r =
                CAD_0::brep::ssi_query(*s1, f0, *s2, f0, kTol);
            CHECK(r.kind == SsiResult::Kind::Empty);
            CHECK(r.message.find("separate") != std::string::npos);
        }
    }

    SUBCASE("the internal edge, s1 the BIGGER: the near foot") {
        const double R1 = 2.0, R2 = 0.5;
        for (double k : {-0.5, +0.5, -1.0, +1.0}) {
            auto s1 = CAD_0::brep::make_sphere(R1);
            auto s2 = CAD_0::brep::make_sphere(R2);
            CAD_0::brep::translate_body(
                *s2, Vec3d{R1 - R2 + k * kTol, 0, 0});
            const SsiResult r =
                CAD_0::brep::ssi_query(*s1, f0, *s2, f0, kTol);
            INFO("k = " << k);
            CHECK(r.kind == SsiResult::Kind::Point);
            CHECK(r.message.find("internal") != std::string::npos);
            // The canonical point: the SMALLER's far foot — here s2
            // the smaller: ON s2 exact, within |k|·tol of s1 (the
            // typed bound; the canonical side is the smaller,
            // whichever operand leads).
            CHECK(sphere_resid(r.point,
                      Vec3d{R1 - R2 + k * kTol, 0, 0}, R2) < 1e-9);
            CHECK(sphere_resid(r.point, Vec3d{0, 0, 0}, R1) <
                  std::abs(k) * kTol + 1e-9);
        }
    }

    SUBCASE("the internal edge, the SWAPPED operands — the first "
            "catch's home: the FAR foot, on BOTH surfaces, either "
            "order") {
        const double R1 = 0.5, R2 = 2.0;  // s1 the SMALLER sphere
        for (double k : {-0.5, +0.5, -1.0, +1.0}) {
            auto s1 = CAD_0::brep::make_sphere(R1);
            auto s2 = CAD_0::brep::make_sphere(R2);
            CAD_0::brep::translate_body(
                *s2, Vec3d{R2 - R1 + k * kTol, 0, 0});
            const SsiResult r =
                CAD_0::brep::ssi_query(*s1, f0, *s2, f0, kTol);
            INFO("k = " << k);
            CHECK(r.kind == SsiResult::Kind::Point);
            CHECK(r.message.find("internal") != std::string::npos);
            // The FAR foot: the canonical point (the SMALLER's far
            // foot, catch 1's cure) — ON s1 (the smaller) exact,
            // within |k|·tol of s2 (the typed bound). The pre-cure
            // code answered the near foot, ON s1 but a full radius
            // inside s2 (the first catch's own red).
            CHECK(sphere_resid(r.point, Vec3d{0, 0, 0}, R1) < 1e-9);
            CHECK(sphere_resid(
                      r.point, Vec3d{R2 - R1 + k * kTol, 0, 0}, R2) <
                  std::abs(k) * kTol + 1e-9);
            CHECK(r.point.x < 0.0);  // the far side, away from c2
            // The order sweep at the tie: the same point whichever
            // operand leads (the purity of the answer — catch 4's
            // own pin).
            const SsiResult rs =
                CAD_0::brep::ssi_query(*s2, f0, *s1, f0, kTol);
            CHECK(rs.kind == SsiResult::Kind::Point);
            CHECK(rs.point == r.point);
        }
        // THE ORDER-PURITY COUNTERPROOF (catch 4, the red said as
        // numbers): the pre-cure rule read the point from the
        // LEADING operand — c1 + û·(±R1) — so the swapped query (the
        // big sphere leading) answered c2 − û·R2 = (D − R2, 0, 0)
        // while this query answered the canonical (−R1, 0, 0): two
        // answers k·tol apart on the same geometry — the
        // order-dependence quantified (k = 0.5 here: the two points
        // 5e-7 apart), the cure the canonical read.
        CHECK(std::abs((R2 - R1 + 0.5 * kTol) - R2 - (-R1)) > 0.0);
        CHECK(std::abs((R2 - R1 + 0.5 * kTol) - R2 - (-R1)) <
              2.0 * kTol);
    }

    SUBCASE("the concentric band: the dedup at the tie, the nested "
            "empty beyond it, the double edge's canonical resolution") {
        {
            // D and |R1−R2| BOTH within their bands: Coincident (the
            // co-radial dedup rule's trigger — the canonical key
            // resolves the double tie to the degenerate class).
            auto s1 = CAD_0::brep::make_sphere(2.0);
            auto s2 = CAD_0::brep::make_sphere(2.0 + 0.5 * kTol);
            CAD_0::brep::translate_body(*s2, Vec3d{0.5 * kTol, 0, 0});
            const SsiResult r =
                CAD_0::brep::ssi_query(*s1, f0, *s2, f0, kTol);
            CHECK(r.kind == SsiResult::Kind::Coincident);
            CHECK(r.message.find("co-radial") != std::string::npos);
        }
        {
            // The DOUBLE EDGE: D = tol AND |R1−R2| = tol — both AT
            // their edges; the registered resolution: Coincident.
            // THE CRACK COUNTERPROOF (catch 3 at the dedup): the
            // pre-cure subtraction read |fl(R1 + tol) − R1| =
            // 1.000000000139778e-06 > tol — the double edge fell to
            // the "concentric, different radii" empty; the
            // comparison form reads the tie TRUE → Coincident, the
            // canonical key's own resolution at the double edge.
            CHECK(std::abs((2.0 + 1.0 * kTol) - 2.0) > kTol);
            auto s1 = CAD_0::brep::make_sphere(2.0);
            auto s2 = CAD_0::brep::make_sphere(2.0 + kTol);
            CAD_0::brep::translate_body(*s2, Vec3d{kTol, 0, 0});
            const SsiResult r =
                CAD_0::brep::ssi_query(*s1, f0, *s2, f0, kTol);
            CHECK(r.kind == SsiResult::Kind::Coincident);
        }
        {
            // The radii beyond the band, the centers within: the
            // concentric NESTED empty, the class named.
            auto s1 = CAD_0::brep::make_sphere(2.0);
            auto s2 = CAD_0::brep::make_sphere(2.0 + 2 * kTol);
            CAD_0::brep::translate_body(*s2, Vec3d{0.5 * kTol, 0, 0});
            const SsiResult r =
                CAD_0::brep::ssi_query(*s1, f0, *s2, f0, kTol);
            CHECK(r.kind == SsiResult::Kind::Empty);
            CHECK(r.message.find("concentric") != std::string::npos);
        }
        {
            // Equal radii, the centers just beyond the band: the
            // near-great-circle — a REAL circle of radius
            // √(R²−D²/4), the tie's honest neighbor.
            auto s1 = CAD_0::brep::make_sphere(2.0);
            auto s2 = CAD_0::brep::make_sphere(2.0);
            CAD_0::brep::translate_body(*s2, Vec3d{3 * kTol, 0, 0});
            const SsiResult r =
                CAD_0::brep::ssi_query(*s1, f0, *s2, f0, kTol);
            CHECK(r.kind == SsiResult::Kind::Circle);
            REQUIRE(r.curves.size() == 1);
            CHECK(std::abs(r.curves[0].radius -
                           std::sqrt(4.0 - 2.25e-12)) < 1e-9);
        }
    }

    SUBCASE("the coincident reads the SURFACE, never the frame — the "
            "seam member") {
        // Two spheres, same center and radius, DIFFERENT (pole,
        // azimuth) frames — the seam meridians disagree; the dedup
        // answers Coincident all the same (the surface, not its
        // parametrization, is the pair's subject).
        auto s1 = CAD_0::brep::make_sphere(2.0);
        const auto s2 = one_face_body(
            std::make_shared<CAD_0::brep::SphereSurface>(
                2.0, Vec3d{0, 0, 0}, Vec3d{0, 0, 1}, Vec3d{0, 1, 0}));
        const SsiResult r =
            CAD_0::brep::ssi_query(*s1, f0, *s2, f0, kTol);
        CHECK(r.kind == SsiResult::Kind::Coincident);
    }
}

// ===========================================================================
// THE TYPED CORPUS — cylinder×sphere: the near-band at the radii
// edge, the coaxiality band, the order sweep, the pole band
// ===========================================================================
TEST_CASE("K1 m4 zoo cylinder×sphere: the near-band at the radii "
          "edge, the coaxiality band, the order sweep") {
    const double r = 1.0;
    const auto cyl = [] {
        return one_face_body(std::make_shared<CAD_0::brep::CylinderZSurface>(
            CAD_0::math::Vec2d{0, 0}, 1.0, 0.0, +1));
    }();

    SUBCASE("the radii edge: the tangent circle at the tie, the pair "
            "beyond, the contained empty within") {
        const Vec3d c{0, 0, 0.3};  // ON the axis
        for (double k : {-0.5, +0.5, -1.0, +1.0}) {
            // INSIDE the band |R−r| ≤ tol: the tangent CIRCLE (the
            // equator) — the tie's resolution, never two merged
            // circles (the m3 counterproof's own red, one family
            // back).
            const auto sph = CAD_0::brep::make_sphere(r + k * kTol);
            CAD_0::brep::translate_body(*sph, c);
            const SsiResult res =
                CAD_0::brep::ssi_query(*cyl, 0, *sph, 0, kTol);
            INFO("k = " << k);
            CHECK(res.kind == SsiResult::Kind::Circle);
            REQUIRE(res.curves.size() == 1);
            // The tangent circle authored ON THE WALL (the entry's
            // own radius r, the cylinder the host surface): exact on
            // the wall, within the band's own width |k|·tol of the
            // sphere — the typed bound at the check.
            for (double t : {0.8, 2.3, 4.9}) {
                CHECK(cylz_resid(res.curves[0].evaluate(t), 0, 0, r) <
                      1e-9);
                CHECK(sphere_resid(res.curves[0].evaluate(t), c,
                                   r + k * kTol) <
                      std::abs(k) * kTol + 1e-9);
            }
        }
        // THE CRACK COUNTERPROOF (catch 3, this entry's site): the
        // pre-cure tie test read |fl(r − tol) − r| =
        // 1.0000000000287557e-06 > tol — the k = −1 member fell to
        // the PAIR branch (two circles 1.4e-3 apart) instead of the
        // tangent equator; the comparison form reads the tie TRUE.
        CHECK(std::abs((r - 1.0 * kTol) - r) > kTol);
        {
            // k = +2: the TWO circles at ±√(R²−r²) — real, close,
            // both ON the sphere exactly and the wall exactly (the
            // axis passes through the center: perp = 0).
            const double R = r + 2 * kTol;
            const auto sph = CAD_0::brep::make_sphere(R);
            CAD_0::brep::translate_body(*sph, c);
            const SsiResult res =
                CAD_0::brep::ssi_query(*cyl, 0, *sph, 0, kTol);
            CHECK(res.kind == SsiResult::Kind::CirclePair);
            REQUIRE(res.curves.size() == 2);
            const double w = std::sqrt(R * R - r * r);
            CHECK(std::abs(res.curves[0].center.z - (c.z + w)) < 1e-9);
            CHECK(std::abs(res.curves[1].center.z - (c.z - w)) < 1e-9);
            for (const auto& cv : res.curves)
                for (double t : {0.8, 2.3, 4.9}) {
                    CHECK(sphere_resid(cv.evaluate(t), c, R) < 1e-9);
                    CHECK(cylz_resid(cv.evaluate(t), 0, 0, r) < 1e-9);
                }
        }
        {
            // k = −2: the contained empty (the sphere never reaches
            // the wall), the class named.
            const auto sph = CAD_0::brep::make_sphere(r - 2 * kTol);
            CAD_0::brep::translate_body(*sph, c);
            const SsiResult res =
                CAD_0::brep::ssi_query(*cyl, 0, *sph, 0, kTol);
            CHECK(res.kind == SsiResult::Kind::Empty);
            CHECK(res.message.find("contained") != std::string::npos);
        }
    }

    SUBCASE("the coaxiality band: the typed entry holds inside and AT "
            "the edge; the Viviani wall just outside") {
        const double R = 1.5;
        for (double perp : {0.5 * kTol, kTol}) {
            // INSIDE the linear band (perp ≤ tol): the typed entry
            // holds — the circles off the wall by ≤ perp (the band's
            // own width, the typed bound at the check).
            const auto sph = CAD_0::brep::make_sphere(R);
            CAD_0::brep::translate_body(*sph, Vec3d{perp, 0, 0.3});
            const SsiResult res =
                CAD_0::brep::ssi_query(*cyl, 0, *sph, 0, kTol);
            INFO("perp = " << perp);
            CHECK(res.kind == SsiResult::Kind::CirclePair);
            REQUIRE(res.curves.size() == 2);
            for (const auto& cv : res.curves)
                for (double t : {0.8, 2.3, 4.9}) {
                    CHECK(sphere_resid(cv.evaluate(t),
                                       Vec3d{perp, 0, 0.3}, R) < 1e-9);
                    CHECK(cylz_resid(cv.evaluate(t), 0, 0, r) <
                          perp + kTol + 1e-9);
                }
        }
        {
            // JUST OUTSIDE (perp = 2·tol): the Viviani wall — the
            // refusal with the pair and the remedy, never a guess.
            const auto sph = CAD_0::brep::make_sphere(R);
            CAD_0::brep::translate_body(*sph, Vec3d{2 * kTol, 0, 0.3});
            const SsiResult res =
                CAD_0::brep::ssi_query(*cyl, 0, *sph, 0, kTol);
            CHECK(res.kind == SsiResult::Kind::OutOfTable);
            CHECK(res.message.find("Viviani") != std::string::npos);
            CHECK(res.message.find("Remedy") != std::string::npos);
        }
    }

    SUBCASE("the order sweep: the sphere first answers the same "
            "disposition") {
        const auto sph = CAD_0::brep::make_sphere(1.5);
        CAD_0::brep::translate_body(*sph, Vec3d{0, 0, 0.3});
        const SsiResult r1 =
            CAD_0::brep::ssi_query(*cyl, 0, *sph, 0, kTol);
        const SsiResult r2 =
            CAD_0::brep::ssi_query(*sph, 0, *cyl, 0, kTol);
        CHECK(r1.kind == r2.kind);
        CHECK(r1.curves.size() == r2.curves.size());
        CHECK(r2.family_a == "sphere");
        CHECK(r2.family_b == "cylinder");
    }

    SUBCASE("the sphere's pole band: the re-frame declared when the "
            "stored pole does not serve") {
        // make_sphere's stored pole is +Y; the cylinder's axis is +Z:
        // the stored frame cannot type the latitudes — the entry
        // DERIVES the re-frame (pole = the axis) and declares it.
        const auto sph_y = CAD_0::brep::make_sphere(1.5);
        CAD_0::brep::translate_body(*sph_y, Vec3d{0, 0, 0.3});
        const SsiResult ry =
            CAD_0::brep::ssi_query(*cyl, 0, *sph_y, 0, kTol);
        CHECK(ry.kind == SsiResult::Kind::CirclePair);
        CHECK(ry.b.reframe_needed);
        CHECK(ry.b.frame_pole == Vec3d{0, 0, 1});

        // The same sphere AUTHORED in the serving frame (pole +Z):
        // the stored frame serves — no re-frame requested.
        const auto sph_z = one_face_body(
            std::make_shared<CAD_0::brep::SphereSurface>(
                1.5, Vec3d{0, 0, 0.3}, Vec3d{0, 0, 1}, Vec3d{1, 0, 0}));
        const SsiResult rz =
            CAD_0::brep::ssi_query(*cyl, 0, *sph_z, 0, kTol);
        CHECK(rz.kind == SsiResult::Kind::CirclePair);
        CHECK(!rz.b.reframe_needed);
    }
}

// ===========================================================================
// THE TYPED CORPUS — cylinder×cylinder: the near-band at both offset
// edges, the equal-radii band, the perpendicular wall, the authorable
// set declared, the seam azimuth member
// ===========================================================================
TEST_CASE("K1 m4 zoo cylinder×cylinder: the near-band at both offset "
          "edges, the equal-radii band, the walls") {
    const FaceId f0 = 0;
    const double r = 1.0;

    SUBCASE("the coincident edge: the dedup within, the REAL pair "
            "just beyond") {
        {
            auto c1 = CAD_0::brep::make_cylinder(r, 2.0);
            auto c2 = CAD_0::brep::make_cylinder(r, 2.0);
            CAD_0::brep::translate_body(*c2, Vec3d{0.5 * kTol, 0, 0});
            const SsiResult res =
                CAD_0::brep::ssi_query(*c1, f0, *c2, f0, kTol);
            CHECK(res.kind == SsiResult::Kind::Coincident);
            CHECK(res.message.find("co-radial") != std::string::npos);
        }
        {
            // d = 2·tol: BEYOND the band — two REAL lines at
            // ±√(r²−d²/4) from the mid-plane, on BOTH walls exactly:
            // the canonical key says Coincident ONLY within d ≤ tol,
            // never "almost coincident" (the honest pair class).
            const double d = 2 * kTol;
            auto c1 = CAD_0::brep::make_cylinder(r, 2.0);
            auto c2 = CAD_0::brep::make_cylinder(r, 2.0);
            CAD_0::brep::translate_body(*c2, Vec3d{d, 0, 0});
            const SsiResult res =
                CAD_0::brep::ssi_query(*c1, f0, *c2, f0, kTol);
            CHECK(res.kind == SsiResult::Kind::LinePair);
            REQUIRE(res.curves.size() == 2);
            for (const auto& cv : res.curves)
                for (double s : {-0.7, 0.0, 1.3}) {
                    const Vec3d p = cv.point + cv.dir * s;
                    // Both walls Y-axis: cylinder 1 at x = z = 0,
                    // cylinder 2 at (d, 0, 0).
                    CHECK(std::abs(std::hypot(p.x, p.z) - r) < 1e-9);
                    CHECK(std::abs(std::hypot(p.x - d, p.z) - r) < 1e-9);
                }
        }
    }

    SUBCASE("the tangent edge: the double line at the tie, the close "
            "pair within, the empty beyond") {
        for (double k : {-0.5, +0.5, -1.0, +1.0}) {
            // INSIDE the band |d−2r| ≤ tol: the TANGENT DOUBLE LINE —
            // one ruling, ON both walls exactly. THE CRACK
            // COUNTERPROOF rides the loop (catch 3, this entry's
            // site): the pre-cure subtraction read
            // |fl(2r + tol) − 2r| = 1.000000000139778e-06 > tol at
            // k = +1 — the member fell to the PAIR branch with
            // half = max(0, r² − d²/4) = 0, two collapsed lines at
            // the mid-plane; the comparison form reads the tie TRUE.
            CHECK(std::abs((2.0 * r + 1.0 * kTol) - 2.0 * r) > kTol);
            auto c1 = CAD_0::brep::make_cylinder(r, 2.0);
            auto c2 = CAD_0::brep::make_cylinder(r, 2.0);
            CAD_0::brep::translate_body(
                *c2, Vec3d{2.0 * r + k * kTol, 0, 0});
            const SsiResult res =
                CAD_0::brep::ssi_query(*c1, f0, *c2, f0, kTol);
            INFO("k = " << k);
            CHECK(res.kind == SsiResult::Kind::TangentLine);
            REQUIRE(res.curves.size() == 1);
            for (double s : {-0.7, 0.0, 1.3}) {
                const Vec3d p = res.curves[0].point + res.curves[0].dir * s;
                CHECK(std::abs(std::hypot(p.x, p.z) - r) < 1e-9);
                CHECK(std::abs(std::hypot(p.x - (2 * r + k * kTol), p.z) -
                               r) < kBandSlack);
            }
        }
        {
            // k = −2: the CLOSE pair — two real lines at
            // ±√(r²−d²/4), half = √(2r·tol − tol²) ≈ 1.4e-3 apart
            // from the mid-plane: the tie's honest neighbor, ON both
            // walls exactly.
            const double d = 2 * r - 2 * kTol;
            auto c1 = CAD_0::brep::make_cylinder(r, 2.0);
            auto c2 = CAD_0::brep::make_cylinder(r, 2.0);
            CAD_0::brep::translate_body(*c2, Vec3d{d, 0, 0});
            const SsiResult res =
                CAD_0::brep::ssi_query(*c1, f0, *c2, f0, kTol);
            CHECK(res.kind == SsiResult::Kind::LinePair);
            REQUIRE(res.curves.size() == 2);
            const double half = std::sqrt(r * r - d * d * 0.25);
            CHECK(half > 1.3e-3);  // real, not collapsed
            CHECK(half < 1.5e-3);
            for (const auto& cv : res.curves)
                for (double s : {-0.7, 0.0, 1.3}) {
                    const Vec3d p = cv.point + cv.dir * s;
                    CHECK(std::abs(std::hypot(p.x, p.z) - r) < 1e-9);
                    CHECK(std::abs(std::hypot(p.x - d, p.z) - r) < 1e-9);
                }
        }
        {
            // k = +2: separate.
            auto c1 = CAD_0::brep::make_cylinder(r, 2.0);
            auto c2 = CAD_0::brep::make_cylinder(r, 2.0);
            CAD_0::brep::translate_body(
                *c2, Vec3d{2.0 * r + 2 * kTol, 0, 0});
            const SsiResult res =
                CAD_0::brep::ssi_query(*c1, f0, *c2, f0, kTol);
            CHECK(res.kind == SsiResult::Kind::Empty);
            CHECK(res.message.find("separate") != std::string::npos);
        }
    }

    SUBCASE("the equal-radii band: the entry holds within and AT "
            "the edge, the corrected refusal beyond") {
        {
            auto c1 = CAD_0::brep::make_cylinder(r, 2.0);
            auto c2 = CAD_0::brep::make_cylinder(r + 0.5 * kTol, 2.0);
            CAD_0::brep::translate_body(*c2, Vec3d{1.2, 0, 0});
            const SsiResult res =
                CAD_0::brep::ssi_query(*c1, f0, *c2, f0, kTol);
            CHECK(res.kind == SsiResult::Kind::LinePair);
        }
        {
            // AT the band's edge (k = ±1): the equal-radii gate
            // holds — the pair. THE CRACK COUNTERPROOF (catch 3,
            // this gate's own): the pre-cure subtraction read
            // |fl(r − tol) − r| = 1.0000000000287557e-06 > tol on
            // the k = −1 side — the member answered the WALL at the
            // edge; the comparison form reads the tie TRUE.
            CHECK(std::abs((r - 1.0 * kTol) - r) > kTol);
            for (double k : {-1.0, +1.0}) {
                auto c1 = CAD_0::brep::make_cylinder(r, 2.0);
                auto c2 =
                    CAD_0::brep::make_cylinder(r + k * kTol, 2.0);
                CAD_0::brep::translate_body(*c2, Vec3d{1.2, 0, 0});
                const SsiResult res =
                    CAD_0::brep::ssi_query(*c1, f0, *c2, f0, kTol);
                INFO("k = " << k);
                CHECK(res.kind == SsiResult::Kind::LinePair);
            }
        }
        {
            // Beyond the band: the UNEQUAL-RADII wall — the
            // acceptance's corrected disposition on record ("two
            // lines", typable, awaiting its consumer), never called
            // untypable.
            auto c1 = CAD_0::brep::make_cylinder(r, 2.0);
            auto c2 = CAD_0::brep::make_cylinder(r + 2 * kTol, 2.0);
            CAD_0::brep::translate_body(*c2, Vec3d{1.2, 0, 0});
            const SsiResult res =
                CAD_0::brep::ssi_query(*c1, f0, *c2, f0, kTol);
            CHECK(res.kind == SsiResult::Kind::OutOfTable);
            CHECK(res.message.find("unequal radii") != std::string::npos);
            CHECK(res.message.find("two lines") != std::string::npos);
        }
    }

    SUBCASE("the perpendicular pair: the non-parallel wall, the "
            "Steinmetz member named") {
        // CylinderZ (axis +Z) × CylinderSurface (axis +Y): the
        // authorable perpendicular — the quartic wall with the
        // sinusoid exhibit in the message.
        const auto cz = one_face_body(
            std::make_shared<CAD_0::brep::CylinderZSurface>(
                CAD_0::math::Vec2d{0, 0}, 1.0, 0.0, +1));
        const auto cy = CAD_0::brep::make_cylinder(1.0, 2.0);
        const SsiResult res =
            CAD_0::brep::ssi_query(*cz, 0, *cy, f0, kTol);
        CHECK(res.kind == SsiResult::Kind::OutOfTable);
        CHECK(res.message.find("non-parallel") != std::string::npos);
        CHECK(res.message.find("Remedy") != std::string::npos);
    }

    SUBCASE("the authorable set, declared: the classes pin the axes — "
            "the near-parallel tilt is UNREACHABLE by authoring") {
        // The writer classes pin the cylinder axes to +Y
        // (CylinderSurface, the factory wall) and +Z (CylinderZ, the
        // extrude's own): every authorable pair is EXACTLY parallel
        // or EXACTLY perpendicular. The near-parallel band of the
        // entry (the cosine band's interior) is therefore not a
        // reachable input — the corpus's honest boundary, the m3
        // "no b knob" declaration's own form: the class's frame
        // discipline IS the fence, and the zoo says so instead of
        // pretending a tilt it cannot author.
        auto c1 = CAD_0::brep::make_cylinder(1.0, 2.0);
        auto c2 = CAD_0::brep::make_cylinder(1.0, 2.0);
        CAD_0::brep::translate_body(*c2, Vec3d{0.8, 0, 0});
        // Both walls are the +Y class: the axes parallel BY
        // CONSTRUCTION (the classes carry no tilt knob) — the
        // in-class answer stands.
        const SsiResult res =
            CAD_0::brep::ssi_query(*c1, f0, *c2, f0, kTol);
        CHECK(res.kind == SsiResult::Kind::LinePair);
    }

    SUBCASE("the seam azimuth member: the tangent line's u on both "
            "domains") {
        // The aligned offsets (both axes' +X): the tangent ruling at
        // u = 0 of cylinder 1 (the +X azimuth) and u = π of cylinder
        // 2 (the −X azimuth) — the seam placement pinned beside the
        // geometry (the frame serves the geometry, never the
        // reverse).
        auto c1 = CAD_0::brep::make_cylinder(r, 2.0);
        auto c2 = CAD_0::brep::make_cylinder(r, 2.0);
        CAD_0::brep::translate_body(*c2, Vec3d{2.0 * r, 0, 0});
        const SsiResult res =
            CAD_0::brep::ssi_query(*c1, f0, *c2, f0, kTol);
        REQUIRE(res.kind == SsiResult::Kind::TangentLine);
        REQUIRE(res.a.pcurves.size() == 1);
        REQUIRE(res.b.pcurves.size() == 1);
        CHECK(std::abs(res.a.pcurves[0].segment.a.x - 0.0) < 1e-9);
        CHECK(std::abs(res.b.pcurves[0].segment.a.x - 3.14159265358979) <
              1e-9);
    }
}

// ===========================================================================
// THE TYPED CORPUS — the ellipsoid's three pairs at the band edges:
// K1 m3's near-band corpus landed with the family (its members sit
// deep inside the band at ±1e-9); the zoo adds the AT-EDGE members
// (k = ±1·tol) the crack lives at, with the constants chosen where
// the crack BITES (c = 0.75 at both signs, a = 1.0 at k = −1, the
// dedup's a = 0.5 at k = +1 — the adversarial authoring, never
// random), and the coaxiality band's disposition beside them
// ===========================================================================
TEST_CASE("K1 m4 zoo the ellipsoid's pairs at the band edges: the "
          "crack's own sites, cured, the offset band's disposition") {
    const double a = 1.0, c = 0.75;  // both crack-prone at the edge

    SUBCASE("plane×ellipsoid: the tangent edge at both signs — the "
            "tie to the POINT, the crash and the flip both cured") {
        for (double k : {-0.5, +0.5, -1.0, +1.0}) {
            const auto el = CAD_0::brep::make_ellipsoid(a, c);
            const auto pl = one_face_body(
                std::make_shared<CAD_0::brep::PlaneSurface>(
                    Vec3d{0, 0, c + k * kTol}, Vec3d{0, 0, 1}));
            const SsiResult r =
                CAD_0::brep::ssi_query(*pl, 0, *el, 0, kTol);
            INFO("k = " << k);
            CHECK(r.kind == SsiResult::Kind::Point);
            // The point at the pole — ON the ellipsoid exact, within
            // the band's own width of the plane (the typed bound).
            CHECK(spheroid_resid(r.point, Vec3d{0, 0, 0}, a, c) < 1e-9);
            CHECK(plane_resid(r.point, Vec3d{0, 0, c + k * kTol},
                              Vec3d{0, 0, 1}) <
                  std::abs(k) * kTol + 1e-9);
        }
        {
            // k = +2: the honest separate.
            const auto el = CAD_0::brep::make_ellipsoid(a, c);
            const auto pl = one_face_body(
                std::make_shared<CAD_0::brep::PlaneSurface>(
                    Vec3d{0, 0, c + 2 * kTol}, Vec3d{0, 0, 1}));
            const SsiResult r =
                CAD_0::brep::ssi_query(*pl, 0, *el, 0, kTol);
            CHECK(r.kind == SsiResult::Kind::Empty);
        }
        {
            // k = −2: the REAL shrunk circle — the tie's honest
            // neighbor, its points on the ellipsoid and the plane
            // exactly (the naive shrunk-circle red's own class).
            const auto el = CAD_0::brep::make_ellipsoid(a, c);
            const auto pl = one_face_body(
                std::make_shared<CAD_0::brep::PlaneSurface>(
                    Vec3d{0, 0, c - 2 * kTol}, Vec3d{0, 0, 1}));
            const SsiResult r =
                CAD_0::brep::ssi_query(*pl, 0, *el, 0, kTol);
            CHECK(r.kind == SsiResult::Kind::Circle);
            REQUIRE(r.curves.size() == 1);
            const double h = c - 2 * kTol;
            const double rho =
                a * std::sqrt(1.0 - h * h / (c * c));
            CHECK(r.curves[0].radius > rho * 0.9);
            CHECK(r.curves[0].radius < rho * 1.1);
            for (double t : {0.7, 2.1, 4.4}) {
                CHECK(spheroid_resid(r.curves[0].evaluate(t),
                                     Vec3d{0, 0, 0}, a, c) < 1e-9);
                CHECK(plane_resid(r.curves[0].evaluate(t),
                                  Vec3d{0, 0, h}, Vec3d{0, 0, 1}) <
                      1e-9);
            }
        }
        // THE CRACK COUNTERPROOF (catch 3, this entry's site): the
        // pre-cure subtraction |fl(c ± 1·tol) − c| reads
        // 1.0000000000287557e-06 > tol at BOTH signs on c = 0.75 —
        // the +1 member crashed (the radius-0 latitude p-curve the
        // authoring contract refuses), the −1 member flipped to a
        // real shrunk circle; the comparison form — the cure — reads
        // the same doubles TRUE.
        CHECK(std::abs((c + 1.0 * kTol) - c) > kTol);
        CHECK(std::abs((c - 1.0 * kTol) - c) > kTol);
    }

    SUBCASE("cylinder×ellipsoid: the radii edge at both signs, the "
            "offset band's disposition, the wall beyond") {
        for (double k : {-0.5, +0.5, -1.0, +1.0}) {
            const auto el = CAD_0::brep::make_ellipsoid(a, c);
            const auto cy = one_face_body(
                std::make_shared<CAD_0::brep::CylinderZSurface>(
                    CAD_0::math::Vec2d{0, 0}, a + k * kTol, 0.0, +1));
            const SsiResult r =
                CAD_0::brep::ssi_query(*cy, 0, *el, 0, kTol);
            INFO("k = " << k);
            CHECK(r.kind == SsiResult::Kind::Circle);
            REQUIRE(r.curves.size() == 1);
            // The tangent circle authored at the EQUATOR, radius a —
            // ON the ellipsoid exact, within |k|·tol of the wall
            // (the typed bound; the entry's own radius read).
            for (double t : {0.8, 2.3, 4.9}) {
                CHECK(spheroid_resid(r.curves[0].evaluate(t),
                                     Vec3d{0, 0, 0}, a, c) < 1e-9);
                CHECK(cylz_resid(r.curves[0].evaluate(t), 0, 0,
                                 a + k * kTol) <
                      std::abs(k) * kTol + 1e-9);
            }
        }
        {
            // k = +2: the honest separate (the wall never reaches).
            const auto el = CAD_0::brep::make_ellipsoid(a, c);
            const auto cy = one_face_body(
                std::make_shared<CAD_0::brep::CylinderZSurface>(
                    CAD_0::math::Vec2d{0, 0}, a + 2 * kTol, 0.0, +1));
            const SsiResult r =
                CAD_0::brep::ssi_query(*cy, 0, *el, 0, kTol);
            CHECK(r.kind == SsiResult::Kind::Empty);
        }
        {
            // k = −2: the CLOSE pair — two real circles at
            // z = ±c·√(1−r²/a²) ≈ ±1.5e-3, on the ellipsoid and
            // the wall exactly.
            const auto el = CAD_0::brep::make_ellipsoid(a, c);
            const auto cy = one_face_body(
                std::make_shared<CAD_0::brep::CylinderZSurface>(
                    CAD_0::math::Vec2d{0, 0}, a - 2 * kTol, 0.0, +1));
            const SsiResult r =
                CAD_0::brep::ssi_query(*cy, 0, *el, 0, kTol);
            CHECK(r.kind == SsiResult::Kind::CirclePair);
            REQUIRE(r.curves.size() == 2);
            const double w =
                c * std::sqrt(1.0 - (a - 2 * kTol) * (a - 2 * kTol) /
                                    (a * a));
            CHECK(w > 1.4e-3);  // real, not collapsed
            CHECK(w < 1.6e-3);
            CHECK(std::abs(r.curves[0].center.z - w) < 1e-9);
            CHECK(std::abs(r.curves[1].center.z + w) < 1e-9);
            for (const auto& cv : r.curves)
                for (double t : {0.8, 2.3, 4.9}) {
                    CHECK(spheroid_resid(cv.evaluate(t),
                                         Vec3d{0, 0, 0}, a, c) < 1e-9);
                    CHECK(cylz_resid(cv.evaluate(t), 0, 0,
                                     a - 2 * kTol) < 1e-9);
                }
        }
        {
            // THE COAXIALITY BAND (the offset member): within it
            // (off ≤ tol) the typed entry holds — the ellipsoid's
            // own latitudes, exact on the ellipsoid, within the
            // band's width off + |k|·tol of the wall; just beyond it
            // the OFFSET wall, the remedy named.
            for (double off : {0.5 * kTol, kTol}) {
                const auto el = CAD_0::brep::make_ellipsoid(a, c);
                CAD_0::brep::translate_body(*el, Vec3d{off, 0, 0});
                const auto cy = one_face_body(
                    std::make_shared<CAD_0::brep::CylinderZSurface>(
                        CAD_0::math::Vec2d{0, 0}, 0.7, 0.0, +1));
                const SsiResult r =
                    CAD_0::brep::ssi_query(*cy, 0, *el, 0, kTol);
                INFO("off = " << off);
                CHECK(r.kind == SsiResult::Kind::CirclePair);
                REQUIRE(r.curves.size() == 2);
                for (const auto& cv : r.curves)
                    for (double t : {0.8, 2.3, 4.9}) {
                        CHECK(spheroid_resid(cv.evaluate(t),
                                             Vec3d{off, 0, 0}, a, c) <
                              1e-9);
                        CHECK(cylz_resid(cv.evaluate(t), 0, 0, 0.7) <
                              off + kTol + 1e-9);
                    }
            }
            {
                const auto el = CAD_0::brep::make_ellipsoid(a, c);
                CAD_0::brep::translate_body(*el,
                                            Vec3d{2 * kTol, 0, 0});
                const auto cy = one_face_body(
                    std::make_shared<CAD_0::brep::CylinderZSurface>(
                        CAD_0::math::Vec2d{0, 0}, 0.7, 0.0, +1));
                const SsiResult r =
                    CAD_0::brep::ssi_query(*cy, 0, *el, 0, kTol);
                CHECK(r.kind == SsiResult::Kind::OutOfTable);
                CHECK(r.message.find("offset") != std::string::npos);
                CHECK(r.message.find("Remedy") != std::string::npos);
            }
        }
        // THE CRACK COUNTERPROOF (catch 3, this entry's site): the
        // pre-cure tie test read |fl(a − tol) − a| =
        // 1.0000000000287557e-06 > tol at a = 1.0 — the k = −1
        // member fell to the PAIR branch with w = 0, two coincident
        // equator circles (the double-count at the edge, the m3
        // counterproof's own class); the comparison form reads the
        // tie TRUE.
        CHECK(std::abs((a - 1.0 * kTol) - a) > kTol);
    }

    SUBCASE("ellipsoid×ellipsoid: the dedup's tie, its edge (the "
            "crack's site), the wall beyond") {
        {
            // The tie: everything within — Coincident.
            const double a1 = 0.5, c1 = 0.3;
            auto e1 = CAD_0::brep::make_ellipsoid(a1, c1);
            auto e2 =
                CAD_0::brep::make_ellipsoid(a1 + 0.5 * kTol,
                                            c1 + 0.5 * kTol);
            CAD_0::brep::translate_body(*e2, Vec3d{0.5 * kTol, 0, 0});
            const SsiResult r =
                CAD_0::brep::ssi_query(*e1, 0, *e2, 0, kTol);
            CHECK(r.kind == SsiResult::Kind::Coincident);
            CHECK(r.message.find("co-radial") != std::string::npos);
        }
        {
            // THE EDGE (a2 = a1 + 1·tol, c2 = c1, dc = 0): the dedup
            // holds — Coincident. THE CRACK COUNTERPROOF (catch 3,
            // this entry's site): the pre-cure subtraction read
            // |fl(a1 + tol) − a1| = 1.0000000000287557e-06 > tol at
            // a1 = 0.5 — the dedup MISSED and the member answered
            // the WALL (the quartic, the remedy text) where the
            // canonical key says Coincident; the comparison form
            // reads the tie TRUE.
            const double a1 = 0.5, c1 = 0.3;
            CHECK(std::abs((a1 + 1.0 * kTol) - a1) > kTol);
            auto e1 = CAD_0::brep::make_ellipsoid(a1, c1);
            auto e2 = CAD_0::brep::make_ellipsoid(a1 + kTol, c1);
            const SsiResult r =
                CAD_0::brep::ssi_query(*e1, 0, *e2, 0, kTol);
            CHECK(r.kind == SsiResult::Kind::Coincident);
        }
        {
            // Beyond the band: the wall (the quartic), the remedy
            // named.
            const double a1 = 0.5, c1 = 0.3;
            auto e1 = CAD_0::brep::make_ellipsoid(a1, c1);
            auto e2 =
                CAD_0::brep::make_ellipsoid(a1 + 1.5 * kTol, c1);
            const SsiResult r =
                CAD_0::brep::ssi_query(*e1, 0, *e2, 0, kTol);
            CHECK(r.kind == SsiResult::Kind::OutOfTable);
            CHECK(r.message.find("Remedy") != std::string::npos);
        }
    }
}

// ===========================================================================
// THE FUZZ DRIVER — deterministic per seed (ADR-0014): same seed →
// same corpus → same verdict bit for bit. The near-band sweep around
// every degenerate configuration, inside the TYPED bands; the operand
// order swept with the coin; the expectation DERIVED from the
// registered band discipline (the independent read above); the digest
// pinned — the CI's cross-run identity — plus the re-run and the
// rebuild witnesses (the second sweep re-authors every body).
// ===========================================================================
TEST_CASE("K1 m4 zoo the fuzz driver: the deterministic near-band "
          "sweep, the digest pinned, re-run and rebuild bit-identical") {
    using Prng = CAD_0::test::property::Prng;

    // THE PINNED SEEDS (the register's m4 row carries them): the
    // morso's date, the protocol's ADR, the determinism ADR.
    const std::uint32_t seeds[] = {20261110u, 41u, 1914u};
    constexpr int kIters = 384;  // per family, per seed

    // --- the per-family sweeps: each appends its answers' digests ---
    // plane×sphere — the tangent-edge sweep, the regular circle, the
    // separate empty, the order coin.
    const auto sweep_plane_sphere = [&](Prng& prng, std::string& acc) {
        for (int i = 0; i < kIters; ++i) {
            const double R = prng.range(0.5, 3.0);
            double d;
            const int region = prng.int_below(3);
            if (region == 0)
                d = R + prng.range(-3.0, 3.0) * kTol;  // near-band
            else if (region == 1)
                d = R * prng.range(0.15, 0.85);        // regular
            else
                d = R * prng.range(1.01, 2.0);         // separate
            const bool swap = prng.coin();
            INFO("plane×sphere seed-iter " << i << " R=" << R << " d=" << d
                                           << " swap=" << swap);
            const auto sph = CAD_0::brep::make_sphere(R);
            const auto pl = one_face_body(
                std::make_shared<CAD_0::brep::PlaneSurface>(
                    Vec3d{0, d, 0}, Vec3d{0, 1, 0}));
            const SsiResult r = swap
                ? CAD_0::brep::ssi_query(*sph, 0, *pl, 0, kTol)
                : CAD_0::brep::ssi_query(*pl, 0, *sph, 0, kTol);
            CHECK(r.kind == expect_plane_sphere(std::abs(d), R, kTol));
            if (r.kind == SsiResult::Kind::Circle) {
                REQUIRE(r.curves.size() == 1);
                for (double t : {0.7, 2.1, 4.4}) {
                    CHECK(sphere_resid(r.curves[0].evaluate(t),
                                       Vec3d{0, 0, 0}, R) < 1e-9);
                    CHECK(plane_resid(r.curves[0].evaluate(t),
                                      Vec3d{0, d, 0},
                                      Vec3d{0, 1, 0}) < 1e-9);
                }
            } else if (r.kind == SsiResult::Kind::Point) {
                CHECK(sphere_resid(r.point, Vec3d{0, 0, 0}, R) < 1e-9);
                // The point at the sphere's pole — within the band's
                // own width of the plane (the typed bound, the tie's
                // distance derived here).
                CHECK(plane_resid(r.point, Vec3d{0, d, 0},
                                  Vec3d{0, 1, 0}) <
                      std::abs(d - R) + 1e-9);
            }
            acc += zoo_digest(r);
            acc += '\n';
        }
    };

    // sphere×sphere — the external edge, the internal edge (both
    // orientations), the concentric band, the regular radical plane,
    // the separate empty, the order coin.
    const auto sweep_sphere_sphere = [&](Prng& prng, std::string& acc) {
        for (int i = 0; i < kIters; ++i) {
            const double R1 = prng.range(0.5, 2.5);
            const int region = prng.int_below(5);
            double R2 = R1, D = 0.0;
            if (region == 0) {  // the external edge
                R2 = R1 + prng.range(0.05, 1.5);
                D = R1 + R2 + prng.range(-3.0, 3.0) * kTol;
            } else if (region == 1) {  // the internal edge
                R2 = R1 + prng.range(0.05, 1.5);
                D = std::abs(R1 - R2) + prng.range(-3.0, 3.0) * kTol;
            } else if (region == 2) {  // the concentric band: the tie
                R2 = R1 + prng.range(-0.5, 0.5) * kTol;
                D = prng.range(0.0, 0.9) * kTol;
            } else if (region == 3) {  // the regular circle
                R2 = R1 + prng.range(0.05, 1.5);
                D = std::abs(R1 - R2) + (R1 + R2 - std::abs(R1 - R2)) *
                                            prng.range(0.15, 0.85);
            } else {  // separate
                R2 = R1 + prng.range(0.05, 1.5);
                D = R1 + R2 + prng.range(0.01, 1.0);
            }
            const bool swap = prng.coin();
            INFO("sphere×sphere seed-iter " << i << " R1=" << R1
                                            << " R2=" << R2 << " D=" << D
                                            << " swap=" << swap);
            auto s1 = CAD_0::brep::make_sphere(R1);
            auto s2 = CAD_0::brep::make_sphere(R2);
            CAD_0::brep::translate_body(*s2, Vec3d{D, 0, 0});
            const SsiResult r = swap
                ? CAD_0::brep::ssi_query(*s2, 0, *s1, 0, kTol)
                : CAD_0::brep::ssi_query(*s1, 0, *s2, 0, kTol);
            CHECK(r.kind == expect_sphere_sphere(R1, R2, D, kTol));
            if (r.kind == SsiResult::Kind::Point) {
                // The canonical touch point (catch 4's cure): EXTERNAL
                // — the BIGGER sphere's foot, INTERNAL — the SMALLER's
                // far foot. The fuzz's authoring: R2 > R1 always, so
                // the bigger is s2 and the smaller is s1; the tie's
                // distance derived here, the typed bound at the
                // check (the first catch's own class: the touch point
                // on the geometry, at every sampled tie).
                if (region == 0) {
                    const double tie = std::abs(D - (R1 + R2));
                    CHECK(sphere_resid(r.point, Vec3d{D, 0, 0}, R2) <
                          1e-9);
                    CHECK(sphere_resid(r.point, Vec3d{0, 0, 0}, R1) <
                          tie + 1e-9);
                } else {
                    const double tie =
                        std::abs(D - std::abs(R1 - R2));
                    CHECK(sphere_resid(r.point, Vec3d{0, 0, 0}, R1) <
                          1e-9);
                    CHECK(sphere_resid(r.point, Vec3d{D, 0, 0}, R2) <
                          tie + 1e-9);
                }
            } else if (r.kind == SsiResult::Kind::Circle) {
                REQUIRE(r.curves.size() == 1);
                for (double t : {0.9, 2.5, 5.0}) {
                    CHECK(sphere_resid(r.curves[0].evaluate(t),
                                       Vec3d{0, 0, 0}, R1) < 1e-9);
                    CHECK(sphere_resid(r.curves[0].evaluate(t),
                                       Vec3d{D, 0, 0}, R2) < 1e-9);
                }
            }
            acc += zoo_digest(r);
            acc += '\n';
        }
    };

    // cylinder×sphere — the radii edge, the pair region, the
    // contained empty, the coaxiality sweep, the order coin.
    const auto sweep_cyl_sphere = [&](Prng& prng, std::string& acc) {
        for (int i = 0; i < kIters; ++i) {
            const double r = prng.range(0.5, 2.0);
            const int region = prng.int_below(4);
            double R = r, perp = 0.0;
            if (region == 0) {  // the radii edge
                R = r + prng.range(-3.0, 3.0) * kTol;
            } else if (region == 1) {  // the pair region
                R = r + prng.range(0.05, 1.0);
            } else if (region == 2) {  // the contained empty
                R = r * prng.range(0.3, 0.9);
            } else {  // the coaxiality sweep (R in the pair region)
                R = r + prng.range(0.05, 1.0);
                perp = prng.range(0.0, 3.0) * kTol;
            }
            const bool swap = prng.coin();
            INFO("cyl×sphere seed-iter " << i << " r=" << r << " R=" << R
                                         << " perp=" << perp
                                         << " swap=" << swap);
            const auto cyl = one_face_body(
                std::make_shared<CAD_0::brep::CylinderZSurface>(
                    CAD_0::math::Vec2d{0, 0}, r, 0.0, +1));
            const auto sph = CAD_0::brep::make_sphere(R);
            CAD_0::brep::translate_body(*sph, Vec3d{perp, 0, 0.3});
            const SsiResult res = swap
                ? CAD_0::brep::ssi_query(*sph, 0, *cyl, 0, kTol)
                : CAD_0::brep::ssi_query(*cyl, 0, *sph, 0, kTol);
            CHECK(res.kind == expect_cyl_sphere(r, R, perp, kTol));
            if (res.kind == SsiResult::Kind::Circle) {
                REQUIRE(res.curves.size() == 1);
                for (double t : {0.8, 2.3, 4.9}) {
                    // On the sphere exact; on the wall within the
                    // band's own width (|R−r| + perp, the typed
                    // bound).
                    CHECK(sphere_resid(res.curves[0].evaluate(t),
                                       Vec3d{perp, 0, 0.3}, R) <
                          std::abs(R - r) + kTol + 1e-9);
                    CHECK(cylz_resid(res.curves[0].evaluate(t), 0, 0, r) <
                          perp + kTol + 1e-9);
                }
            } else if (res.kind == SsiResult::Kind::CirclePair) {
                REQUIRE(res.curves.size() == 2);
                for (const auto& cv : res.curves)
                    for (double t : {0.8, 2.3, 4.9}) {
                        CHECK(sphere_resid(cv.evaluate(t),
                                           Vec3d{perp, 0, 0.3}, R) < 1e-9);
                        CHECK(cylz_resid(cv.evaluate(t), 0, 0, r) <
                              perp + kTol + 1e-9);
                    }
            }
            acc += zoo_digest(res);
            acc += '\n';
        }
    };

    // cylinder×cylinder — the coincident edge, the tangent edge, the
    // regular pair, the equal-radii band, the separate empty.
    const auto sweep_cyl_cyl = [&](Prng& prng, std::string& acc) {
        for (int i = 0; i < kIters; ++i) {
            const double r1 = prng.range(0.5, 2.0);
            const int region = prng.int_below(5);
            double r2 = r1, d = 0.0;
            if (region == 0) {  // the coincident edge
                d = prng.range(0.0, 3.0) * kTol;
            } else if (region == 1) {  // the tangent edge
                d = 2.0 * r1 + prng.range(-3.0, 3.0) * kTol;
            } else if (region == 2) {  // the regular pair
                d = 2.0 * r1 * prng.range(0.1, 0.8);
            } else if (region == 3) {  // the equal-radii band
                r2 = r1 + prng.range(-3.0, 3.0) * kTol;
                d = 2.0 * r1 * prng.range(0.1, 0.8);
            } else {  // separate
                d = 2.0 * r1 * prng.range(1.01, 1.5);
            }
            INFO("cyl×cyl seed-iter " << i << " r1=" << r1 << " r2=" << r2
                                      << " d=" << d);
            auto c1 = CAD_0::brep::make_cylinder(r1, 2.0);
            auto c2 = CAD_0::brep::make_cylinder(r2, 2.0);
            CAD_0::brep::translate_body(*c2, Vec3d{d, 0, 0});
            const SsiResult res =
                CAD_0::brep::ssi_query(*c1, 0, *c2, 0, kTol);
            CHECK(res.kind == expect_cyl_cyl(r1, r2, d, kTol));
            if (res.kind == SsiResult::Kind::LinePair ||
                res.kind == SsiResult::Kind::TangentLine) {
                for (const auto& cv : res.curves)
                    for (double s : {-0.7, 0.0, 1.3}) {
                        const Vec3d p = cv.point + cv.dir * s;
                        CHECK(std::abs(std::hypot(p.x, p.z) - r1) <
                              kBandSlack);
                        CHECK(std::abs(std::hypot(p.x - d, p.z) - r2) <
                              kBandSlack);
                    }
            }
            acc += zoo_digest(res);
            acc += '\n';
        }
    };

    // plane×ellipsoid — the m3 entry joins the sweep: the tangent
    // edge |h| vs c, the regular latitude, the separate empty, the
    // oblique wall, the order coin.
    const auto sweep_plane_ellipsoid = [&](Prng& prng, std::string& acc) {
        for (int i = 0; i < kIters; ++i) {
            const double a = prng.range(0.5, 2.0);
            const double c = a * prng.range(0.5, 0.95);
            const int region = prng.int_below(4);
            double h = 0.0;
            bool oblique = false;
            if (region == 0) {
                h = c + prng.range(-3.0, 3.0) * kTol;  // the tangent edge
            } else if (region == 1) {
                h = c * prng.range(0.1, 0.9);          // regular
            } else if (region == 2) {
                h = c * prng.range(1.01, 1.5);         // separate
            } else {
                oblique = true;                        // the wall
                h = c * prng.range(0.1, 0.9);
            }
            const bool swap = prng.coin();
            INFO("plane×ellipsoid seed-iter " << i << " a=" << a
                                              << " c=" << c << " h=" << h
                                              << " oblique=" << oblique
                                              << " swap=" << swap);
            const auto el = CAD_0::brep::make_ellipsoid(a, c);
            const auto pl = one_face_body(
                std::make_shared<CAD_0::brep::PlaneSurface>(
                    Vec3d{0, 0, h},
                    oblique ? Vec3d{0, 1, 1} : Vec3d{0, 0, 1}));
            const SsiResult r = swap
                ? CAD_0::brep::ssi_query(*el, 0, *pl, 0, kTol)
                : CAD_0::brep::ssi_query(*pl, 0, *el, 0, kTol);
            if (oblique) {
                CHECK(r.kind == SsiResult::Kind::OutOfTable);
                CHECK(r.message.find("ELLIPSE") != std::string::npos);
            } else {
                CHECK(r.kind == expect_plane_ellipsoid(std::abs(h), c, kTol));
                if (r.kind == SsiResult::Kind::Circle) {
                    REQUIRE(r.curves.size() == 1);
                    for (double t : {0.7, 2.1, 4.4}) {
                        CHECK(spheroid_resid(r.curves[0].evaluate(t),
                                             Vec3d{0, 0, 0}, a, c) <
                              1e-9);
                        CHECK(plane_resid(r.curves[0].evaluate(t),
                                          Vec3d{0, 0, h},
                                          Vec3d{0, 0, 1}) < 1e-9);
                    }
                } else if (r.kind == SsiResult::Kind::Point) {
                    CHECK(spheroid_resid(r.point, Vec3d{0, 0, 0}, a, c) <
                          1e-9);
                    // The point at the pole — within the band's own
                    // width of the plane (the typed bound).
                    CHECK(plane_resid(r.point, Vec3d{0, 0, h},
                                      Vec3d{0, 0, 1}) <
                          std::abs(std::abs(h) - c) + 1e-9);
                }
            }
            acc += zoo_digest(r);
            acc += '\n';
        }
    };

    // cylinder×ellipsoid — the radii edge r vs a, the pair region,
    // the empty beyond, the offset wall, the order coin.
    const auto sweep_cyl_ellipsoid = [&](Prng& prng, std::string& acc) {
        for (int i = 0; i < kIters; ++i) {
            const double a = prng.range(0.5, 2.0);
            const double c = a * prng.range(0.5, 0.95);
            const int region = prng.int_below(4);
            double r = a, off = 0.0;
            if (region == 0) {
                r = a + prng.range(-3.0, 3.0) * kTol;  // the radii edge
            } else if (region == 1) {
                r = a * prng.range(0.2, 0.9);          // the pair region
            } else if (region == 2) {
                r = a * prng.range(1.01, 1.4);         // empty
            } else {
                r = a * prng.range(0.2, 0.9);          // the offset wall
                off = prng.range(0.0, 3.0) * kTol;
            }
            const bool swap = prng.coin();
            INFO("cyl×ellipsoid seed-iter " << i << " a=" << a << " c=" << c
                                            << " r=" << r << " off=" << off
                                            << " swap=" << swap);
            const auto cyl = one_face_body(
                std::make_shared<CAD_0::brep::CylinderZSurface>(
                    CAD_0::math::Vec2d{0, 0}, r, 0.0, +1));
            const auto el = CAD_0::brep::make_ellipsoid(a, c);
            CAD_0::brep::translate_body(*el, Vec3d{off, 0, 0});
            const SsiResult res = swap
                ? CAD_0::brep::ssi_query(*el, 0, *cyl, 0, kTol)
                : CAD_0::brep::ssi_query(*cyl, 0, *el, 0, kTol);
            // The coaxiality band is a REGISTERED disposition (the
            // sphere's row 6 one family over): within it (off ≤ tol)
            // the typed entry holds; beyond it the offset wall.
            CHECK(res.kind == expect_cyl_ellipsoid(r, a, off, kTol));
            if (region == 3 && off > kTol) {
                CHECK(res.message.find("offset") != std::string::npos);
            } else if (res.kind == SsiResult::Kind::Circle) {
                REQUIRE(res.curves.size() == 1);
                for (double t : {0.8, 2.3, 4.9}) {
                    // The tangent circle at the EQUATOR, radius a —
                    // on the ellipsoid exact, within the band's own
                    // width of the wall (the typed bound).
                    CHECK(spheroid_resid(res.curves[0].evaluate(t),
                                         Vec3d{off, 0, 0}, a, c) < 1e-9);
                    CHECK(cylz_resid(res.curves[0].evaluate(t), 0, 0,
                                     r) < std::abs(r - a) + 1e-9);
                }
            } else if (res.kind == SsiResult::Kind::CirclePair) {
                REQUIRE(res.curves.size() == 2);
                for (const auto& cv : res.curves)
                    for (double t : {0.8, 2.3, 4.9}) {
                        CHECK(spheroid_resid(cv.evaluate(t),
                                             Vec3d{off, 0, 0}, a, c) <
                              1e-9);
                        CHECK(cylz_resid(cv.evaluate(t), 0, 0, r) <
                              off + kTol + 1e-9);
                    }
            }
            acc += zoo_digest(res);
            acc += '\n';
        }
    };

    // ellipsoid×ellipsoid — the coincident tie (the semiaxes, the
    // center), the near-but-beyond wall, the different-pair wall.
    const auto sweep_ellipsoid_ellipsoid = [&](Prng& prng,
                                               std::string& acc) {
        for (int i = 0; i < kIters; ++i) {
            const double a1 = prng.range(0.5, 2.0);
            const double c1 = a1 * prng.range(0.5, 0.95);
            const int region = prng.int_below(3);
            double a2 = a1, c2 = c1, dc = 0.0;
            if (region == 0) {  // the tie: everything within tol
                a2 = a1 + prng.range(-0.5, 0.5) * kTol;
                c2 = c1 + prng.range(-0.5, 0.5) * kTol;
                dc = prng.range(0.0, 0.9) * kTol;
            } else if (region == 1) {  // near but BEYOND: the wall
                a2 = a1 + prng.range(1.5, 4.0) * kTol;
                c2 = c1 + prng.range(-0.5, 0.5) * kTol;
                dc = prng.range(0.0, 0.5) * kTol;
            } else {  // a different pair: the wall (the quartic)
                a2 = a1 * prng.range(0.5, 0.9);
                c2 = c1 * prng.range(0.5, 0.9);
                dc = prng.range(0.0, 0.5);
            }
            INFO("ellipsoid×ellipsoid seed-iter " << i << " a1=" << a1
                                                  << " c1=" << c1
                                                  << " a2=" << a2
                                                  << " c2=" << c2
                                                  << " dc=" << dc);
            auto e1 = CAD_0::brep::make_ellipsoid(a1, c1);
            auto e2 = CAD_0::brep::make_ellipsoid(a2, c2);
            CAD_0::brep::translate_body(*e2, Vec3d{dc, 0, 0});
            const SsiResult r =
                CAD_0::brep::ssi_query(*e1, 0, *e2, 0, kTol);
            CHECK(r.kind ==
                  expect_ellipsoid_ellipsoid(a1, c1, a2, c2, dc, kTol));
            if (r.kind == SsiResult::Kind::OutOfTable)
                CHECK(r.message.find("Remedy") != std::string::npos);
            acc += zoo_digest(r);
            acc += '\n';
        }
    };

    // --- the driver: two reps, every body re-authored fresh in each
    // (the rebuild witness rides the re-run), the digests compared to
    // each other AND to the pin ---
    std::uint64_t digests[2] = {0, 0};
    for (int rep = 0; rep < 2; ++rep) {
        std::string acc;
        acc.reserve(1u << 20);
        for (std::uint32_t seed : seeds) {
            Prng prng(seed);
            sweep_plane_sphere(prng, acc);
            sweep_sphere_sphere(prng, acc);
            sweep_cyl_sphere(prng, acc);
            sweep_cyl_cyl(prng, acc);
            sweep_plane_ellipsoid(prng, acc);
            sweep_cyl_ellipsoid(prng, acc);
            sweep_ellipsoid_ellipsoid(prng, acc);
        }
        digests[rep] = fnv1a(acc);
    }
    // THE RE-RUN + REBUILD WITNESS (ADR-0014): the second rep
    // re-authors every body and re-asks every query — bit-identical.
    CHECK(digests[0] == digests[1]);
    // THE PIN (the CI's cross-run identity): the pinned constant —
    // 3 seeds × 7 families × 384 iterations = 8,064 verdicts per rep.
    CHECK(digests[0] == 0x6d4ae0b321409188ull);
}
