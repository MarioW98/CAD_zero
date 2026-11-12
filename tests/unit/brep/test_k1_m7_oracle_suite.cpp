// tests/unit/brep/test_k1_m7_oracle_suite.cpp
//
// THE ORACLE SUITE (K1 m7, ADR-0041 D1.5 — Metric 5's own tool): the
// standing record that re-derives, by roads sharing ZERO code with
// the kernel, the precision claims the census's mass columns and the
// register's goldens carry.
//
// THE LAW (D1.5, verbatim in spirit): a precision claim is VERIFIED
// only when an oracle that shares ZERO computation code with the
// kernel re-derives it. The comparison road IS the public query
// surface (mass_properties, ssi_query — the oracle must READ the
// kernel's answer somehow); the DERIVATION road is this file's own:
// closed forms with their provenance cited (a classical form
// pre-dates the kernel, and that is exactly why it qualifies), and
// quadratures written HERE on the test's own loops (the m3 ellipsoid
// precedent's form), plus one fresh derivation — the rim fillet's
// removal by Pappus's composite centroids, a different road than
// ADR-0039 D5's ring-sector measure and a third than the kernel's
// (u,v) window integrals. A golden that is the kernel's own output
// re-affirmed proves determinism, not truth: independence is the
// difference between verified and believed.
//
// THE COVERAGE, the census's own seats (never beyond):
//   * the five mass families whose column is PASS — plane (the box),
//     cylinder, sphere, torus, ellipsoid: the closed forms beside
//     the kernel's answers;
//   * the register's pinned goldens — the box fillet 7.5+pi/8, the
//     blind rim 8-0.375pi, the rim fillet at r=0.25 AND at the m6
//     silent edit's r=0.125, the chained fillets 7+pi/4, the full
//     torus 2pi^2Rr^2, the ellipsoid 0.375pi with its classical
//     moments;
//   * the degenerate pairs the census PASSes (the sphere's three and
//     the ellipsoid's three, K1 m4's first cohort): the exact
//     degenerate answers derived by elementary geometry, pinned at
//     the tie itself.
//   The zoo pairs the census does NOT pass (the consumers' pairs;
//   cylinder x cylinder's own cell a dash) stay out: the suite never
//   invents coverage the census did not grant.
//
// THE TOLERANCES, pinned with their reasons (never widened to pass):
//   * closed form vs exact kernel arithmetic: 1e-12 absolute (both
//     roads exact in double — the m3 classical pins' own bound);
//   * quadrature roads: 2e-6 relative (the m3 precedent's declared
//     floor for 2e6-point sums — the honest bound is the
//     accumulation's rounding; each site's truncation is smaller and
//     declared in place).
//
// THE COUNTERPROOF LAW (D1.5's discriminating proof): an oracle that
// cannot say NO is not an oracle. The suite's reds are assertions —
// a radius inflated, a half-extent moved, a fillet radius quietly
// edited, a wrong closed form — each CAUGHT beyond tolerance, each
// with its control (the same comparison green on the true value):
// the catch is the defect's own, never the scenario's.
//
// THE DIGEST (ADR-0014): the whole verdict rendered at 12 significant
// digits (the m5 chain digest's own boundary — libm's 1-ULP
// sin/cos differences and the quadrature sums' last-bit noise are
// absorbed at that width, real drift is not), FNV-1a 64 pinned: the
// cross-run identity. The verdict is built twice in-process and the
// two texts required equal — the re-run witness.
#include "CAD_0/brep/analytic.hpp"
#include "CAD_0/brep/factory.hpp"
#include "CAD_0/brep/fillet.hpp"
#include "CAD_0/brep/mass_properties.hpp"
#include "CAD_0/brep/naming.hpp"
#include "CAD_0/brep/query.hpp"
#include "CAD_0/brep/ssi.hpp"
#include "CAD_0/brep/subtract.hpp"
#include "CAD_0/brep/topology.hpp"
#include "CAD_0/brep/extrude.hpp"
#include "CAD_0/sdf/sketch_profile.hpp"
#include "CAD_0/geometry/shape_id.hpp"

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <memory>
#include <string>
#include <vector>

#include <doctest/doctest.h>

namespace {

namespace b = CAD_0::brep;
namespace n = CAD_0::brep::naming;
namespace s = CAD_0::sdf;
using CAD_0::brep::BRepBody;
using CAD_0::brep::FaceId;
using CAD_0::brep::SsiResult;
using CAD_0::geometry::FeatureId;
using CAD_0::geometry::ShapeId;
using CAD_0::geometry::SubshapeRef;
using CAD_0::math::Vec3d;

constexpr double kPi = 3.14159265358979323846;

// The declared Layer-3 decision tolerance the SSI classifications
// read (the zoo's own contract: THE DECLARED VALUE, never a private
// one).
constexpr double kSsiTol = 1e-6;

// The mass query's own tolerance (the m3 rite's form).
const CAD_0::math::Tolerance kMassTol =
    CAD_0::math::Tolerance::linear(1e-9);

bool close(double a, double b, double eps = 1e-12) {
    return std::abs(a - b) <= eps;
}

// The relative form for the quadrature roads (2e-6, declared above).
bool rel_close(double a, double b, double rel) {
    return std::abs(a - b) <=
           rel * std::max(std::abs(a), std::abs(b));
}

// ---------------------------------------------------------------------------
// THE ORACLES — pure double arithmetic, ZERO kernel types, ZERO
// kernel headers. Every road's provenance cited at its site.
// ---------------------------------------------------------------------------

// SPHERE — Archimedes, "On the Sphere and Cylinder" I (the sphere is
// two-thirds the circumscribing cylinder): V = (4/3)pi R^3. The form
// pre-dates the kernel by twenty-two centuries — D1.5-qualified.
double archimedes_sphere_volume(double R) {
    return (4.0 / 3.0) * kPi * R * R * R;
}

// SPHERE's moment — the classical companion (also Archimedes' era,
// read through the modern principal-axis form): I_axis = (2/5) V R^2
// about the centroid for the homogeneous solid.
double archimedes_sphere_axis_inertia(double R) {
    const double V = archimedes_sphere_volume(R);
    return (2.0 / 5.0) * V * R * R;
}

// TORUS — Pappus's centroid theorem (Collection, Book VIII; the
// surface-of-revolution volume as area times centroid path): the disc
// of radius r revolved about the external axis at distance R gives
// V = pi r^2 * 2 pi R = 2 pi^2 R r^2.
double pappus_torus_volume(double R, double r) {
    return 2.0 * kPi * kPi * R * r * r;
}

// CYLINDER — the classical base-times-height form (Eudoxus' method,
// Elements XII.10 for the cone's third; the cylinder read directly
// through Cavalieri): V = pi R^2 h.
double euclid_cylinder_volume(double R, double h) {
    return kPi * R * R * h;
}

// BOX — the rectangular parallelepiped, elementary (Elements XI).
// half-extents in, full volume out: V = 8 hx hy hz.
double euclid_box_volume(double hx, double hy, double hz) {
    return 8.0 * hx * hy * hz;
}

// ELLIPSOID (the spheroid a=a, c) — the sphere's affine image, the
// classical scaling: V = (4/3) pi a^2 c. Pre-dates the kernel (the
// m3 census row's own qualification).
double ellipsoid_classical_volume(double a, double c) {
    return (4.0 / 3.0) * kPi * a * a * c;
}

// ELLIPSOID, road two — Cavalieri's stack, written here: the slice at
// height z is the disc of area A(z) = pi a^2 (1 - z^2/c^2), a QUADRATIC
// in z, and V = integral of A over [-c, c] by the 1D midpoint rule
// with N slices. Truncation: the midpoint rule's per-cell error
// (h^3/12) f'' sums to 2c^3/(3 N^2) ~ 2e-14 at c=0.5, N=2e6 — the
// honest bound is the 2e6-term accumulation's rounding, hence the
// declared 2e-6 relative. A different road than both the kernel's
// divergence-theorem face integrals and m3's (u,v) parametrization
// quadrature: the question asked a third time.
double cavalieri_ellipsoid_volume(double a, double c, int n) {
    const double h = (2.0 * c) / static_cast<double>(n);
    double sum = 0.0;
    for (int i = 0; i < n; ++i) {
        const double z = -c + (static_cast<double>(i) + 0.5) * h;
        sum += kPi * a * a * (1.0 - (z * z) / (c * c));
    }
    return sum * h;
}

// THE FRESH DERIVATION — the rim fillet's removed volume by Pappus's
// composite centroids (K1 m7's own road; ADR-0039 D5 derives the same
// quantity by the ring-sector measure integral — the two roads agree
// symbolically, and the KERNEL computes it a third way, the toroidal
// flank's (u,v) window integrals through the divergence theorem).
//
// Per meridian plane (rho the radius from the hole axis, z up): the
// removed region is the corner window [a, a+r] x [1-r, 1] minus the
// quarter disc of radius r centered (a+r, 1-r) facing the corner.
// The corner (a, 1) is where the hole's wall meets the top plane; the
// rolling ball of radius r is tangent to both, its center at
// (a+r, 1-r). Revolved about the z axis (the region never crosses
// it: rho >= a > 0), Pappus reads
//
//     V_removed = 2 pi * (moment of the region about the axis)
//                = 2 pi * (moment(square) - moment(quarter disc))
//
// The square's moment about the axis (its area r^2 at centroid radius
// a + r/2):  a r^2 + r^3 / 2.  The quarter disc's, in polar
// coordinates about its OWN center (rho = (a+r) + s cos(theta),
// theta in [pi/2, pi] — the corner-facing quarter): the area term
// (a+r) pi r^2 / 4 plus the correction integral of s cos(theta),
// which carries the -r^3/3 (cos integrates to -1 over the quarter):
// (a+r) pi r^2 / 4 - r^3 / 3.  The difference, times 2 pi:
//
//     V_r = 2 pi [ a r^2 (1 - pi/4) + r^3 (5/6 - pi/4) ]
//
// — the ADR's own closed form, arrived at by the centroid road.
double pappus_corner_removal(double a, double r) {
    const double square_moment = a * r * r + r * r * r / 2.0;
    const double quarter_moment =
        (a + r) * kPi * r * r / 4.0 - r * r * r / 3.0;
    return 2.0 * kPi * (square_moment - quarter_moment);
}

// THE FILLET WEDGE (a box edge's 90-degree corner) — elementary: the
// removed cross-section is the square r x r minus the quadrant
// pi r^2 / 4 (Euclid I.47 for the square; the quadrant classical),
// swept along the edge's length L: V_removed = r^2 (1 - pi/4) L.
double wedge_elementary(double r, double L) {
    return r * r * (1.0 - kPi / 4.0) * L;
}

// THE FILLET WEDGE, road two — the quadrant's area by a polar
// midpoint quadrature written here: A_quadrant = integral over
// [0, r] x [0, pi/2] of s ds dtheta (the polar rectangle's own
// measure — NO trigonometry evaluated, the Jacobian is s). Midpoint
// over NU x NV cells; truncation O(h^2) (~1e-13 at this grid), the
// rounding accumulation the declared 2e-6. The wedge then reads
// r^2 - A_quadrant per unit length: the same question the elementary
// form answers, asked numerically.
double wedge_quadrant_quadrature(double r, int nu, int nv) {
    const double ds = r / static_cast<double>(nu);
    const double dt = (kPi / 2.0) / static_cast<double>(nv);
    double sum = 0.0;
    for (int i = 0; i < nu; ++i) {
        const double rad = (static_cast<double>(i) + 0.5) * ds;
        for (int j = 0; j < nv; ++j) {
            sum += rad * ds * dt;
        }
    }
    return sum;
}

// TORUS, road two — the shell quadrature with the singularity killed
// by substitution (written here): V = integral of 2 pi rho h(rho)
// d rho with h the tube's height 2 sqrt(r^2 - (rho-R)^2); the
// substitution rho = R + r sin(phi) makes the integrand the
// trig polynomial 4 pi r^2 (R + r sin(phi)) cos^2(phi) over
// [-pi/2, pi/2] — smooth, the odd term self-cancelling on the
// symmetric midpoint grid. Truncation: the periodic midpoint rule is
// exact for the band-limited cos^2 at this N; the honest bound is
// again the accumulation's rounding, 2e-6 declared.
double torus_shell_quadrature(double R, double r, int n) {
    const double dphi = kPi / static_cast<double>(n);
    double sum = 0.0;
    for (int i = 0; i < n; ++i) {
        const double phi =
            -kPi / 2.0 + (static_cast<double>(i) + 0.5) * dphi;
        const double sp = std::sin(phi);
        const double cp = std::cos(phi);
        sum += (R + r * sp) * cp * cp;
    }
    return 4.0 * kPi * r * r * sum * dphi;
}

// ---------------------------------------------------------------------------
// THE CONSTRUCTIONS — the public factory surface, the golden bodies'
// own forms (the comparison road; the derivation stays above)
// ---------------------------------------------------------------------------

// A one-face body carrying an arbitrary analytic surface (the zoo's
// own authoring helper, verbatim in spirit — the SSI query is
// surface-level on these records).
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

// The box edge fillet's guide (the m2 golden's own construction):
// the box (half-extents 1,1,1 — the 2x2x2), the edge vertical2 (edge
// 10, the (+X,+Y) vertical, length 2), fillet fil#2 at r.
struct FilletGuide {
    std::unique_ptr<b::BRepBody> box;
    std::unique_ptr<b::BRepBody> result;

    explicit FilletGuide(double radius) {
        box = b::make_box({1.0, 1.0, 1.0}, FeatureId{1});
        const ShapeId reference{FeatureId{1},
                                SubshapeRef{}.push(10), {}};
        const b::FilletNaming ctx{FeatureId{2}, &n::kBoxNaming};
        result = b::fillet_edge(*box, reference, radius, ctx);
    }
};

// The blind rim's guide (the u2 construction, the m3 comment's own
// words): the circle r = 0.5, prism height 2 offset (0,0,-0.5) — the
// tool z in [-0.5, 1.5], the floor at -0.5, the rim through the top
// at z = 1: the bite is the cylinder r=0.5 depth 1.5. The rim by
// NAME from the query (the seam-wired pair answers to the one
// logical name), then fillet_rim_edge at the radius.
struct RimGuide {
    std::unique_ptr<b::BRepBody> box;
    std::unique_ptr<b::BRepBody> tool;
    std::unique_ptr<b::BRepBody> blind;
    std::unique_ptr<b::BRepBody> result;

    explicit RimGuide(double radius) {
        box = b::make_box({1.0, 1.0, 1.0}, FeatureId{1});
        std::vector<s::SketchChain> chains;
        chains.push_back(s::SketchChain{
            {s::sketch_circle({0.0f, 0.0f}, 0.5f, false)}});
        tool = b::make_prism(chains, 2.0f, +1, FeatureId{3});
        const b::SubtractNaming sub_ctx{
            FeatureId{4}, &n::kBoxNaming, &n::kExtNaming};
        blind = b::exact_subtract(*box, *tool, {0.0, 0.0, -0.5},
                                  b::kSubtractTolerance, sub_ctx);
        ShapeId rim{};
        for (std::size_t i = 0; i < blind->edge_count(); ++i) {
            if (b::query_edge(*blind, i).name ==
                "sub#4:isect[box#1:top \xc3\x97 ext#3:lat(0)]@hi") {
                rim = blind->edges[i].shape_id();
                break;
            }
        }
        const b::FilletNaming ctx{FeatureId{5}, &n::kBoxNaming,
                                  FeatureId{3}, &n::kExtNaming};
        result = b::fillet_rim_edge(*blind, rim, radius, ctx);
    }
};

// The chained fillets' guide (the m6ter chain): box -> fil#2
// (vertical2, r2) -> fil#3 (vertical0's kept stamp, r3) — two
// DISJOINT vertical corners, the removals independent and additive.
struct ChainGuide {
    std::unique_ptr<b::BRepBody> box;
    std::unique_ptr<b::BRepBody> first;
    std::unique_ptr<b::BRepBody> result;

    ChainGuide(double r2, double r3) {
        box = b::make_box({1.0, 1.0, 1.0}, FeatureId{1});
        const ShapeId vertical2{FeatureId{1},
                                SubshapeRef{}.push(10), {}};
        const ShapeId vertical0{FeatureId{1},
                                SubshapeRef{}.push(8), {}};
        const b::FilletNaming ctx2{FeatureId{2}, &n::kBoxNaming};
        first = b::fillet_edge(*box, vertical2, r2, ctx2);
        const b::FilletNaming ctx3{FeatureId{3}, &n::kBoxNaming};
        result = b::fillet_edge(*first, vertical0, r3, ctx3);
    }
};

// ---------------------------------------------------------------------------
// THE VERDICT'S RENDERING (the digest's own text, ADR-0014): every
// number at 12 significant digits — the m5 chain digest's boundary.
// ---------------------------------------------------------------------------
void g12(std::string& out, const char* name, double v) {
    char buf[64];
    std::snprintf(buf, sizeof(buf), "%.12g", v);
    out += name;
    out += '=';
    out += buf;
    out += '\n';
}

std::uint64_t fnv1a64(const std::string& str) {
    std::uint64_t h = 0xcbf29ce484222325ull;
    for (const char c : str) {
        h ^= static_cast<std::uint64_t>(static_cast<unsigned char>(c));
        h *= 0x100000001b3ull;
    }
    return h;
}

// The aggregate witness: every read the suite makes, re-run and
// rendered — the single text whose digest is pinned. Deterministic
// by construction (fixed bodies, fixed grids, fixed loop orders,
// zero time or randomness — ADR-0014's clause).
std::string oracle_verdict() {
    std::string v;

    // -- the five mass families --------------------------------------
    {
        const auto body = b::make_box({1.0, 1.5, 0.75});
        const auto mp = b::mass_properties(*body, kMassTol);
        g12(v, "plane.box.kernel", mp.volume);
        g12(v, "plane.box.oracle",
            euclid_box_volume(1.0, 1.5, 0.75));
    }
    {
        const double r = 0.8, h = 2.6;  // unit5's own parameters
        const auto body = b::make_cylinder(r, h);
        const auto mp = b::mass_properties(*body, kMassTol);
        g12(v, "cyl.factory.kernel", mp.volume);
        g12(v, "cyl.factory.oracle", euclid_cylinder_volume(r, h));
    }
    {
        const double R = 2.0;  // the zoo's own sphere
        const auto body = b::make_sphere(R);
        const auto mp = b::mass_properties(*body, kMassTol);
        g12(v, "sphere.R2.kernel", mp.volume);
        g12(v, "sphere.R2.oracle", archimedes_sphere_volume(R));
        g12(v, "sphere.R2.Izz.kernel",
            mp.inertia_about_centroid(1, 1));
        g12(v, "sphere.R2.Izz.oracle",
            archimedes_sphere_axis_inertia(R));
    }
    {
        const double a = 0.75, c = 0.5;  // the m3 probe's own body
        const auto body = b::make_ellipsoid(a, c);
        const auto mp = b::mass_properties(*body, kMassTol);
        g12(v, "ellipsoid.kernel", mp.volume);
        g12(v, "ellipsoid.classical",
            ellipsoid_classical_volume(a, c));
        g12(v, "ellipsoid.cavalieri",
            cavalieri_ellipsoid_volume(a, c, 2000000));
        g12(v, "ellipsoid.Izz.kernel",
            mp.inertia_about_centroid(2, 2));
        g12(v, "ellipsoid.Izz.oracle",
            (2.0 / 5.0) * ellipsoid_classical_volume(a, c) * a * a);
        g12(v, "ellipsoid.Ixx.kernel",
            mp.inertia_about_centroid(0, 0));
        g12(v, "ellipsoid.Ixx.oracle",
            ellipsoid_classical_volume(a, c) / 5.0 * (a * a + c * c));
    }
    {
        const double R = 0.75, r = 0.25;  // the flank's own torus
        g12(v, "torus.pappus", pappus_torus_volume(R, r));
        g12(v, "torus.quad", torus_shell_quadrature(R, r, 2000000));
        g12(v, "torus.verbatim", 0.9252754126021273);
    }

    // -- the register's goldens --------------------------------------
    {
        const FilletGuide g(0.5);
        const auto mp = b::mass_properties(*g.result, kMassTol);
        g12(v, "golden.boxFillet.kernel", mp.volume);
        g12(v, "golden.boxFillet.oracle",
            euclid_box_volume(1.0, 1.0, 1.0) -
                wedge_elementary(0.5, 2.0));
        g12(v, "golden.boxFillet.quad",
            euclid_box_volume(1.0, 1.0, 1.0) -
                (0.25 - wedge_quadrant_quadrature(0.5, 2000, 1000)) *
                    2.0);
        g12(v, "golden.boxFillet.verbatim",
            7.892699081698725);
    }
    {
        const RimGuide g(0.25);
        const auto blind_mp = b::mass_properties(*g.blind, kMassTol);
        g12(v, "golden.blind.kernel", blind_mp.volume);
        g12(v, "golden.blind.oracle",
            euclid_box_volume(1.0, 1.0, 1.0) -
                euclid_cylinder_volume(0.5, 1.5));
        const auto mp = b::mass_properties(*g.result, kMassTol);
        g12(v, "golden.rimFillet.kernel", mp.volume);
        g12(v, "golden.rimFillet.oracle",
            euclid_box_volume(1.0, 1.0, 1.0) -
                euclid_cylinder_volume(0.5, 1.5) -
                pappus_corner_removal(0.5, 0.25));
        g12(v, "golden.rimFillet.verbatim",
            6.775059758517764);
    }
    {
        const RimGuide g(0.125);  // the m6 silent edit's own radius
        const auto mp = b::mass_properties(*g.result, kMassTol);
        g12(v, "golden.rimFillet125.kernel", mp.volume);
        g12(v, "golden.rimFillet125.oracle",
            euclid_box_volume(1.0, 1.0, 1.0) -
                euclid_cylinder_volume(0.5, 1.5) -
                pappus_corner_removal(0.5, 0.125));
        g12(v, "golden.rimFillet125.verbatim",
            6.810780258845277);
    }
    {
        const ChainGuide g(0.5, 0.5);
        const auto mp = b::mass_properties(*g.result, kMassTol);
        g12(v, "golden.chain.kernel", mp.volume);
        g12(v, "golden.chain.oracle",
            euclid_box_volume(1.0, 1.0, 1.0) -
                2.0 * wedge_elementary(0.5, 2.0));
        g12(v, "golden.chain.verbatim", 7.0 + kPi / 4.0);
    }

    // -- the degenerate pairs the census passes (the exact ties) ----
    {
        const double R = 2.0;
        const auto sph = b::make_sphere(R);
        const auto pl = one_face_body(
            std::make_shared<CAD_0::brep::PlaneSurface>(
                Vec3d{0, R, 0}, Vec3d{0, 1, 0}));
        const SsiResult r =
            CAD_0::brep::ssi_query(*pl, 0, *sph, 0, kSsiTol);
        g12(v, "deg.planeSphere.kind",
            static_cast<double>(static_cast<int>(r.kind)));
        g12(v, "deg.planeSphere.px", r.point.x);
        g12(v, "deg.planeSphere.py", r.point.y);
        g12(v, "deg.planeSphere.pz", r.point.z);
    }
    {
        auto s1 = b::make_sphere(2.0);
        auto s2 = b::make_sphere(1.0);
        b::translate_body(*s2, Vec3d{3.0, 0, 0});
        const SsiResult r =
            CAD_0::brep::ssi_query(*s1, 0, *s2, 0, kSsiTol);
        g12(v, "deg.sphereExt.kind",
            static_cast<double>(static_cast<int>(r.kind)));
        g12(v, "deg.sphereExt.px", r.point.x);
        g12(v, "deg.sphereExt.py", r.point.y);
    }
    {
        auto s1 = b::make_sphere(2.0);
        auto s2 = b::make_sphere(0.5);
        b::translate_body(*s2, Vec3d{1.5, 0, 0});
        const SsiResult r =
            CAD_0::brep::ssi_query(*s1, 0, *s2, 0, kSsiTol);
        g12(v, "deg.sphereInt.kind",
            static_cast<double>(static_cast<int>(r.kind)));
        g12(v, "deg.sphereInt.px", r.point.x);
    }
    {
        const auto cyl = one_face_body(
            std::make_shared<CAD_0::brep::CylinderZSurface>(
                CAD_0::math::Vec2d{0, 0}, 1.0, 0.0, +1));
        auto sph = b::make_sphere(1.0);
        b::translate_body(*sph, Vec3d{0, 0, 0.3});
        const SsiResult r =
            CAD_0::brep::ssi_query(*cyl, 0, *sph, 0, kSsiTol);
        g12(v, "deg.cylSphere.kind",
            static_cast<double>(static_cast<int>(r.kind)));
        if (!r.curves.empty()) {
            g12(v, "deg.cylSphere.cz", r.curves[0].center.z);
            g12(v, "deg.cylSphere.radius", r.curves[0].radius);
        }
    }
    {
        const auto el = b::make_ellipsoid(1.0, 0.75);
        const auto pl = one_face_body(
            std::make_shared<CAD_0::brep::PlaneSurface>(
                Vec3d{0, 0, 0.75}, Vec3d{0, 0, 1}));
        const SsiResult r =
            CAD_0::brep::ssi_query(*pl, 0, *el, 0, kSsiTol);
        g12(v, "deg.planeEllipsoid.kind",
            static_cast<double>(static_cast<int>(r.kind)));
        g12(v, "deg.planeEllipsoid.pz", r.point.z);
    }
    {
        const auto el = b::make_ellipsoid(1.0, 0.75);
        const auto cy = one_face_body(
            std::make_shared<CAD_0::brep::CylinderZSurface>(
                CAD_0::math::Vec2d{0, 0}, 1.0, 0.0, +1));
        const SsiResult r =
            CAD_0::brep::ssi_query(*cy, 0, *el, 0, kSsiTol);
        g12(v, "deg.cylEllipsoid.kind",
            static_cast<double>(static_cast<int>(r.kind)));
        if (!r.curves.empty()) {
            g12(v, "deg.cylEllipsoid.cz", r.curves[0].center.z);
            g12(v, "deg.cylEllipsoid.radius",
                r.curves[0].radius);
        }
    }
    {
        const auto e1 = b::make_ellipsoid(1.0, 0.75);
        const auto e2 = b::make_ellipsoid(1.0, 0.75);
        const SsiResult r =
            CAD_0::brep::ssi_query(*e1, 0, *e2, 0, kSsiTol);
        g12(v, "deg.ellipsoidDedup.kind",
            static_cast<double>(static_cast<int>(r.kind)));
    }

    // -- the counterproofs (the catches, each with its control) ------
    {
        const double R = 2.0;
        const double R_bad = R * (1.0 + 1e-3);
        const auto body = b::make_sphere(R_bad);
        const auto mp = b::mass_properties(*body, kMassTol);
        const bool caught = !rel_close(
            mp.volume, archimedes_sphere_volume(R), 2e-6);
        const bool control = close(
            mp.volume, archimedes_sphere_volume(R_bad), 1e-12);
        g12(v, "counter.radiusInflated.caught",
            caught ? 1.0 : 0.0);
        g12(v, "counter.radiusInflated.control",
            control ? 1.0 : 0.0);
    }
    {
        const auto body = b::make_box({1.0 + 1e-3, 1.0, 1.0});
        const auto mp = b::mass_properties(*body, kMassTol);
        const bool caught = !close(
            mp.volume, euclid_box_volume(1.0, 1.0, 1.0), 1e-9);
        const bool control = close(
            mp.volume, euclid_box_volume(1.0 + 1e-3, 1.0, 1.0),
            1e-12);
        g12(v, "counter.halfExtentMoved.caught",
            caught ? 1.0 : 0.0);
        g12(v, "counter.halfExtentMoved.control",
            control ? 1.0 : 0.0);
    }
    {
        const ChainGuide g(0.5, 0.5 * (1.0 + 1e-3));
        const auto mp = b::mass_properties(*g.result, kMassTol);
        const bool caught = !close(
            mp.volume, 7.0 + kPi / 4.0, 1e-9);
        const bool control = rel_close(
            mp.volume,
            euclid_box_volume(1.0, 1.0, 1.0) -
                wedge_elementary(0.5, 2.0) -
                wedge_elementary(0.5 * (1.0 + 1e-3), 2.0),
            1e-12);
        g12(v, "counter.radiusEdited.caught",
            caught ? 1.0 : 0.0);
        g12(v, "counter.radiusEdited.control",
            control ? 1.0 : 0.0);
    }
    {
        // The wrong closed form (D1.5's own discriminating proof):
        // the sphere's form worn by the spheroid — grossly wrong,
        // the oracle road refuses it where the true form agrees.
        const double naive =
            (4.0 / 3.0) * kPi * 0.75 * 0.75 * 0.75;
        const bool caught =
            !rel_close(ellipsoid_classical_volume(0.75, 0.5),
                       naive, 2e-6);
        g12(v, "counter.wrongForm.caught",
            caught ? 1.0 : 0.0);
        g12(v, "counter.wrongForm.naive", naive);
    }
    return v;
}

} // namespace

// ===========================================================================
// THE FIVE MASS FAMILIES — the closed forms beside the kernel's answers
// ===========================================================================

TEST_CASE("K1 m7 oracle: the plane family — the box's elementary form") {
    const auto body = b::make_box({1.0, 1.5, 0.75});
    const auto mp = b::mass_properties(*body, kMassTol);
    REQUIRE(mp.all_exact);
    // Elements XI: the rectangular solid. Both roads exact in double.
    CHECK(close(mp.volume, euclid_box_volume(1.0, 1.5, 0.75)));
    CHECK(mp.volume == doctest::Approx(9.0).epsilon(1e-12));
    // The centroid at the box's own center (the frame's origin).
    CHECK(mp.centroid.x == doctest::Approx(0.0).epsilon(1e-12));
    CHECK(mp.centroid.y == doctest::Approx(0.0).epsilon(1e-12));
    CHECK(mp.centroid.z == doctest::Approx(0.0).epsilon(1e-12));
}

TEST_CASE("K1 m7 oracle: the cylinder family — the factory body against "
          "the classical form") {
    const double r = 0.8, h = 2.6;  // unit5's own parameters, declared
    const auto body = b::make_cylinder(r, h);
    const auto mp = b::mass_properties(*body, kMassTol);
    REQUIRE(mp.all_exact);
    // The base-times-height form (Eudoxus' method, read through
    // Cavalieri) — pre-dates the kernel.
    CHECK(close(mp.volume, euclid_cylinder_volume(r, h)));
    CHECK(mp.volume == doctest::Approx(5.227610175573417)
              .epsilon(1e-12));
}

TEST_CASE("K1 m7 oracle: the sphere family — Archimedes beside the "
          "kernel, the moment with it") {
    const double R = 2.0;  // the zoo's own sphere, declared
    const auto body = b::make_sphere(R);
    const auto mp = b::mass_properties(*body, kMassTol);
    REQUIRE(mp.all_exact);
    // "On the Sphere and Cylinder" I: two-thirds the circumscribing
    // cylinder.
    CHECK(close(mp.volume, archimedes_sphere_volume(R)));
    CHECK(mp.volume == doctest::Approx(33.510321638291124)
              .epsilon(1e-12));
    // The classical axis moment (the factory sphere's axis is Y).
    CHECK(close(mp.inertia_about_centroid(1, 1),
                archimedes_sphere_axis_inertia(R)));
}

TEST_CASE("K1 m7 oracle: the torus family — Pappus and the shell "
          "quadrature, the full-torus form pinned verbatim") {
    const double R = 0.75, r = 0.25;  // the flank's own torus
    // Pappus's centroid theorem: the disc of radius r revolved about
    // the external axis at distance R.
    const double v_pappus = pappus_torus_volume(R, r);
    CHECK(v_pappus == doctest::Approx(0.9252754126021273)
              .epsilon(1e-12));
    // The shell quadrature road (the substitution killing the square
    // root) — the same question asked numerically, 2e-6 declared.
    const double v_quad = torus_shell_quadrature(R, r, 2000000);
    CHECK(rel_close(v_quad, v_pappus, 2e-6));
    // The register's verbatim pin (the family's discriminator).
    CHECK(pappus_torus_volume(R, r) ==
          doctest::Approx(0.925275412602127).epsilon(1e-12));
    // There is no full-torus BODY on today's tree (the family's
    // kernel read rides the rim fillet's flank, in the goldens
    // below) — the form's two oracle roads stand on their own here,
    // and the flank's consumption is pinned where the kernel answers.
}

TEST_CASE("K1 m7 oracle: the ellipsoid family — the classical form, the "
          "Cavalieri stack, the classical moments") {
    const double a = 0.75, c = 0.5;  // the m3 probe's own body
    const auto body = b::make_ellipsoid(a, c);
    const auto mp = b::mass_properties(*body, kMassTol);
    REQUIRE(mp.all_exact);
    // The sphere's affine image: V = (4/3) pi a^2 c (pre-dates the
    // kernel — the m3 census row's own qualification).
    CHECK(close(mp.volume, ellipsoid_classical_volume(a, c)));
    CHECK(mp.volume == doctest::Approx(1.1780972450961724)
              .epsilon(1e-12));
    // CAVALIERI, road two (this file's own): the slice areas
    // stacked — a different road than the kernel's face integrals
    // and than m3's (u,v) quadrature.
    CHECK(rel_close(mp.volume,
                    cavalieri_ellipsoid_volume(a, c, 2000000), 2e-6));
    // The classical moments (the m3 pins, re-derived here):
    // I_z = (2/5) V a^2, I_x = I_y = (V/5)(a^2 + c^2).
    CHECK(close(mp.inertia_about_centroid(2, 2),
                (2.0 / 5.0) * ellipsoid_classical_volume(a, c) * a * a));
    CHECK(close(mp.inertia_about_centroid(0, 0),
                ellipsoid_classical_volume(a, c) / 5.0 * (a * a + c * c)));
    CHECK(mp.inertia_about_centroid(2, 2) ==
          doctest::Approx(0.2650718801466388).epsilon(1e-12));
    CHECK(mp.inertia_about_centroid(0, 0) ==
          doctest::Approx(0.1914408023281280).epsilon(1e-12));
}

// ===========================================================================
// THE REGISTER'S GOLDENS — every pinned volume by its own oracle roads
// ===========================================================================

TEST_CASE("K1 m7 oracle: the box fillet golden — the wedge by two roads, "
          "7.5+pi/8 verbatim") {
    const FilletGuide g(0.5);
    REQUIRE(g.result != nullptr);
    const auto mp = b::mass_properties(*g.result, kMassTol);
    REQUIRE(mp.all_exact);
    // The removal: the r x r corner square minus the quadrant, swept
    // along the edge (length 2) — elementary.
    CHECK(close(mp.volume,
                euclid_box_volume(1.0, 1.0, 1.0) -
                    wedge_elementary(0.5, 2.0)));
    // Road two: the quadrant's area by the polar quadrature — the
    // same question numerically, 2e-6 declared.
    const double wedge_quad =
        (0.25 - wedge_quadrant_quadrature(0.5, 2000, 1000)) * 2.0;
    CHECK(rel_close(8.0 - mp.volume, wedge_quad, 2e-6));
    // The register's verbatim pin (the m2 golden: 7.5 + pi/8).
    CHECK(mp.volume == doctest::Approx(7.892699081698725)
              .epsilon(1e-12));
    CHECK(mp.volume == doctest::Approx(7.5 + kPi / 8.0).epsilon(1e-12));
}

TEST_CASE("K1 m7 oracle: the blind rim golden — the bite by Euclid, "
          "8-0.375pi verbatim") {
    const RimGuide g(0.25);
    REQUIRE(g.blind != nullptr);
    const auto mp = b::mass_properties(*g.blind, kMassTol);
    REQUIRE(mp.all_exact);
    // The bite: the cylinder r=0.5 through the top, depth 1.5 (the
    // tool z in [-0.5, 1.5] against the box's top at z=1) — the
    // classical form, and nothing else moved.
    CHECK(close(mp.volume,
                euclid_box_volume(1.0, 1.0, 1.0) -
                    euclid_cylinder_volume(0.5, 1.5)));
    CHECK(mp.volume == doctest::Approx(8.0 - 0.375 * kPi)
              .epsilon(1e-12));
    CHECK(mp.volume ==
          doctest::Approx(6.821902754903828).epsilon(1e-12));
}

TEST_CASE("K1 m7 oracle: the rim fillet golden — the Pappus composite "
          "fresh, both radii verbatim") {
    // THE FRESH DERIVATION at the golden's own radius: the removal by
    // Pappus's composite centroids (the derivation at the function's
    // side) — a different road than ADR-0039 D5's ring-sector
    // measure, and a third than the kernel's window integrals.
    {
        const RimGuide g(0.25);
        REQUIRE(g.result != nullptr);
        const auto mp = b::mass_properties(*g.result, kMassTol);
        REQUIRE(mp.all_exact);
        CHECK(close(mp.volume,
                    euclid_box_volume(1.0, 1.0, 1.0) -
                        euclid_cylinder_volume(0.5, 1.5) -
                        pappus_corner_removal(0.5, 0.25)));
        CHECK(mp.volume ==
              doctest::Approx(6.775059758517764).epsilon(1e-12));
        // The removal itself, the number the ADR pinned.
        CHECK(pappus_corner_removal(0.5, 0.25) ==
              doctest::Approx(0.046842996386064).epsilon(1e-15));
    }
    // The m6 silent edit's own radius (the survival ledger's member
    // 1): the SAME closed form at r=0.125 — the oracle follows the
    // parameter, the pin rides with it.
    {
        const RimGuide g(0.125);
        REQUIRE(g.result != nullptr);
        const auto mp = b::mass_properties(*g.result, kMassTol);
        REQUIRE(mp.all_exact);
        CHECK(close(mp.volume,
                    euclid_box_volume(1.0, 1.0, 1.0) -
                        euclid_cylinder_volume(0.5, 1.5) -
                        pappus_corner_removal(0.5, 0.125)));
        CHECK(mp.volume ==
              doctest::Approx(6.810780258845277).epsilon(1e-12));
    }
}

TEST_CASE("K1 m7 oracle: the chained fillet golden — two independent "
          "wedges, 7+pi/4 verbatim") {
    const ChainGuide g(0.5, 0.5);
    REQUIRE(g.result != nullptr);
    const auto mp = b::mass_properties(*g.result, kMassTol);
    REQUIRE(mp.all_exact);
    // Two DISJOINT vertical corners (vertical2 then vertical0's kept
    // stamp), each an r=0.5 wedge on a length-2 edge: the removals
    // independent and additive — the composition's own claim, checked
    // as the sum.
    CHECK(close(mp.volume,
                euclid_box_volume(1.0, 1.0, 1.0) -
                    2.0 * wedge_elementary(0.5, 2.0)));
    CHECK(mp.volume == doctest::Approx(7.0 + kPi / 4.0).epsilon(1e-12));
    // The removal decomposition itself: 2 links x 2 length x wedge.
    CHECK(close(8.0 - mp.volume,
                2.0 * 2.0 * 0.25 * (1.0 - kPi / 4.0)));
}

// ===========================================================================
// THE DEGENERATE PAIRS THE CENSUS PASSES — the exact ties by elementary
// geometry (K1 m4's first cohort, the sphere's three and the
// ellipsoid's three; the near-band belongs to the zoo, the tie itself
// to the oracle)
// ===========================================================================

TEST_CASE("K1 m7 oracle: the degenerate pairs at the exact tie — the "
          "answers by elementary geometry") {
    SUBCASE("plane x sphere: the tangent point is the pole on the "
            "plane's normal line") {
        const double R = 2.0;
        const auto sph = b::make_sphere(R);
        const auto pl = one_face_body(
            std::make_shared<CAD_0::brep::PlaneSurface>(
                Vec3d{0, R, 0}, Vec3d{0, 1, 0}));
        const SsiResult r =
            CAD_0::brep::ssi_query(*pl, 0, *sph, 0, kSsiTol);
        CHECK(r.kind == SsiResult::Kind::Point);
        // The foot of the center's perpendicular, C + R n^ — the
        // elementary answer, exact in double at the tie.
        CHECK(close(r.point.x, 0.0));
        CHECK(close(r.point.y, R));
        CHECK(close(r.point.z, 0.0));
        // The south tangent, the symmetric member.
        const auto pl_s = one_face_body(
            std::make_shared<CAD_0::brep::PlaneSurface>(
                Vec3d{0, -R, 0}, Vec3d{0, 1, 0}));
        const SsiResult rs =
            CAD_0::brep::ssi_query(*pl_s, 0, *sph, 0, kSsiTol);
        CHECK(rs.kind == SsiResult::Kind::Point);
        CHECK(close(rs.point.y, -R));
        // The seam meridian's member: the plane at +X, the point on
        // the u=0 meridian — the seam never moves the geometry.
        const auto pl_x = one_face_body(
            std::make_shared<CAD_0::brep::PlaneSurface>(
                Vec3d{R, 0, 0}, Vec3d{1, 0, 0}));
        const SsiResult rx =
            CAD_0::brep::ssi_query(*pl_x, 0, *sph, 0, kSsiTol);
        CHECK(rx.kind == SsiResult::Kind::Point);
        CHECK(close(rx.point.x, R));
        CHECK(close(rx.point.y, 0.0));
        CHECK(close(rx.point.z, 0.0));
    }

    SUBCASE("sphere x sphere: both tangencies on the center line") {
        // External, R1=2 > R2=1, D=3: the canonical touch point is
        // the BIGGER sphere's foot toward the smaller — C1 + R1 x^,
        // which at the exact tie is (2, 0, 0).
        {
            auto s1 = b::make_sphere(2.0);
            auto s2 = b::make_sphere(1.0);
            b::translate_body(*s2, Vec3d{3.0, 0, 0});
            const SsiResult r =
                CAD_0::brep::ssi_query(*s1, 0, *s2, 0, kSsiTol);
            CHECK(r.kind == SsiResult::Kind::Point);
            CHECK(close(r.point.x, 2.0));
            CHECK(close(r.point.y, 0.0));
            CHECK(close(r.point.z, 0.0));
        }
        // Internal, R1=2 > R2=0.5, D=1.5: the SMALLER's far foot —
        // C2 + R2 x^ = (1.5 + 0.5, 0, 0) = (2, 0, 0): both surfaces'
        // touch, either read.
        {
            auto s1 = b::make_sphere(2.0);
            auto s2 = b::make_sphere(0.5);
            b::translate_body(*s2, Vec3d{1.5, 0, 0});
            const SsiResult r =
                CAD_0::brep::ssi_query(*s1, 0, *s2, 0, kSsiTol);
            CHECK(r.kind == SsiResult::Kind::Point);
            CHECK(r.message.find("internal") != std::string::npos);
            CHECK(close(r.point.x, 2.0));
            CHECK(close(r.point.y, 0.0));
            CHECK(close(r.point.z, 0.0));
        }
    }

    SUBCASE("cylinder x sphere at the radii tie: the tangent circle "
            "is the equator, center at the sphere's center, radius "
            "the wall's own") {
        const auto cyl = one_face_body(
            std::make_shared<CAD_0::brep::CylinderZSurface>(
                CAD_0::math::Vec2d{0, 0}, 1.0, 0.0, +1));
        auto sph = b::make_sphere(1.0);
        b::translate_body(*sph, Vec3d{0, 0, 0.3});
        const SsiResult r =
            CAD_0::brep::ssi_query(*cyl, 0, *sph, 0, kSsiTol);
        CHECK(r.kind == SsiResult::Kind::Circle);
        REQUIRE(r.curves.size() == 1);
        // The equator: z = the sphere center's height, radius = the
        // tie's own r (the entry's own radius read).
        CHECK(close(r.curves[0].center.x, 0.0));
        CHECK(close(r.curves[0].center.y, 0.0));
        CHECK(close(r.curves[0].center.z, 0.3));
        CHECK(close(r.curves[0].radius, 1.0));
    }

    SUBCASE("plane x ellipsoid: the tangent points are the poles") {
        const auto el = b::make_ellipsoid(1.0, 0.75);
        {
            const auto pl = one_face_body(
                std::make_shared<CAD_0::brep::PlaneSurface>(
                    Vec3d{0, 0, 0.75}, Vec3d{0, 0, 1}));
            const SsiResult r =
                CAD_0::brep::ssi_query(*pl, 0, *el, 0, kSsiTol);
            CHECK(r.kind == SsiResult::Kind::Point);
            CHECK(close(r.point.x, 0.0));
            CHECK(close(r.point.y, 0.0));
            CHECK(close(r.point.z, 0.75));
        }
        {
            const auto pl = one_face_body(
                std::make_shared<CAD_0::brep::PlaneSurface>(
                    Vec3d{0, 0, -0.75}, Vec3d{0, 0, 1}));
            const SsiResult r =
                CAD_0::brep::ssi_query(*pl, 0, *el, 0, kSsiTol);
            CHECK(r.kind == SsiResult::Kind::Point);
            CHECK(close(r.point.z, -0.75));
        }
    }

    SUBCASE("cylinder x ellipsoid at the radii tie: the tangent "
            "circle is the equator, radius a") {
        const auto el = b::make_ellipsoid(1.0, 0.75);
        const auto cy = one_face_body(
            std::make_shared<CAD_0::brep::CylinderZSurface>(
                CAD_0::math::Vec2d{0, 0}, 1.0, 0.0, +1));
        const SsiResult r =
            CAD_0::brep::ssi_query(*cy, 0, *el, 0, kSsiTol);
        CHECK(r.kind == SsiResult::Kind::Circle);
        REQUIRE(r.curves.size() == 1);
        CHECK(close(r.curves[0].center.x, 0.0));
        CHECK(close(r.curves[0].center.y, 0.0));
        CHECK(close(r.curves[0].center.z, 0.0));
        CHECK(close(r.curves[0].radius, 1.0));
    }

    SUBCASE("ellipsoid x ellipsoid: the co-radial dedup is Coincident "
            "at the exact tie") {
        const auto e1 = b::make_ellipsoid(1.0, 0.75);
        const auto e2 = b::make_ellipsoid(1.0, 0.75);
        const SsiResult r =
            CAD_0::brep::ssi_query(*e1, 0, *e2, 0, kSsiTol);
        CHECK(r.kind == SsiResult::Kind::Coincident);
    }
}

// ===========================================================================
// THE COUNTERPROOF — the oracle CATCHES (D1.5's discriminating proof:
// the failure is the assertion, the control beside each catch)
// ===========================================================================

TEST_CASE("K1 m7 oracle: the counterproof — a radius inflated, a "
          "half-extent moved, a link's radius quietly edited, a wrong "
          "closed form — each caught, each with its control") {
    // 1. THE RADIUS INFLATED (the sphere): a VALID body, green
    //    against its own form — the R-oracle refuses it, and the
    //    refusal is the radius's own (the control agrees at 1e-12
    //    with the inflated radius's form: the catch is the defect's,
    //    never the scenario's).
    {
        const double R = 2.0;
        const double R_bad = R * (1.0 + 1e-3);
        const auto body = b::make_sphere(R_bad);
        const auto mp = b::mass_properties(*body, kMassTol);
        CHECK(mp.all_exact);
        CHECK_FALSE(rel_close(mp.volume,
                              archimedes_sphere_volume(R), 2e-6));
        CHECK(close(mp.volume, archimedes_sphere_volume(R_bad)));
    }
    // 2. THE HALF-EXTENT MOVED (the box's vertices): the body valid,
    //    the 8-oracle refuses it by exactly the moved wall's own
    //    volume — and the control green on the moved body's form.
    {
        const auto body = b::make_box({1.0 + 1e-3, 1.0, 1.0});
        const auto mp = b::mass_properties(*body, kMassTol);
        CHECK(mp.all_exact);
        CHECK_FALSE(close(mp.volume,
                          euclid_box_volume(1.0, 1.0, 1.0), 1e-9));
        CHECK(close(mp.volume,
                    euclid_box_volume(1.0 + 1e-3, 1.0, 1.0)));
    }
    // 3. THE LINK'S RADIUS QUIETLY EDITED (the chained fillet, the
    //    m6 survival ledger's own scenario shape): fil#3 at
    //    0.5(1+1e-3) — the golden's 7+pi/4 oracle refuses the edited
    //    chain; the control agrees with the EDITED wedge sum: the
    //    oracle catches the edit, not the chain.
    {
        const ChainGuide g(0.5, 0.5 * (1.0 + 1e-3));
        REQUIRE(g.result != nullptr);
        const auto mp = b::mass_properties(*g.result, kMassTol);
        CHECK(mp.all_exact);
        CHECK_FALSE(close(mp.volume, 7.0 + kPi / 4.0, 1e-9));
        CHECK(rel_close(
            mp.volume,
            euclid_box_volume(1.0, 1.0, 1.0) -
                wedge_elementary(0.5, 2.0) -
                wedge_elementary(0.5 * (1.0 + 1e-3), 2.0),
            1e-12));
    }
    // 4. THE WRONG CLOSED FORM (D1.5's own naive): the sphere's form
    //    worn by the spheroid — (4/3)pi a^3 against the true
    //    (4/3)pi a^2 c: grossly wrong (the m3 counterproof's own
    //    numbers), the oracle road refuses it where the true form
    //    agrees; and the full-torus form misread as the flank's flux
    //    (the m3 discrimination, quoted) — a wrong form DISAGREES.
    {
        const double naive_sphere_form =
            (4.0 / 3.0) * kPi * 0.75 * 0.75 * 0.75;
        CHECK(naive_sphere_form > 1.7);  // 1.767... grossly off
        CHECK_FALSE(rel_close(ellipsoid_classical_volume(0.75, 0.5),
                              naive_sphere_form, 2e-6));
        CHECK_FALSE(close(0.9252754126021273,
                          0.448519633389893, 1e-9));
    }
}

// ===========================================================================
// THE DIGEST — the whole verdict, 12 significant digits, FNV-1a 64
// pinned: the cross-run identity (ADR-0014)
// ===========================================================================

TEST_CASE("K1 m7 oracle: the determinism digest — the verdict built "
          "twice, the texts equal, the FNV-1a 64 pinned") {
    const std::string v1 = oracle_verdict();
    const std::string v2 = oracle_verdict();
    // The re-run witness: bit-identical texts, in-process.
    CHECK(v1 == v2);
    REQUIRE(v1.size() > 500);

    // The pinned digest — the cross-run identity. CALIBRATION NOTE:
    // the constant below was read from the suite's own first clean
    // run and pinned; any drift in any rendered value — oracle,
    // kernel, or catch — moves it and the test goes red.
    const std::uint64_t d = fnv1a64(v1);
    CHECK(d == 0xfc55acb1e6b1ca14ull);

    std::printf("[k1-m7-oracle-suite] digest %016llx  text %zu bytes\n",
                static_cast<unsigned long long>(d), v1.size());
}
