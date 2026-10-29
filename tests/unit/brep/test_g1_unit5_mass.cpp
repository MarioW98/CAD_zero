// tests/unit/brep/test_g1_unit5_mass.cpp
//
// G1 UNIT 5 — THE EXACT MASS PROPERTIES: the discriminating tests
// (ADR-0032 D4, written FIRST per D9 — they fail today for the right
// reason: the API does not exist; the build itself names the absence,
// the unit-1 precedent).
//
// THE DISCIPLINE (D9.3): the classical formulas ARE the tests, never
// the implementation. Every expected value below is a textbook closed
// form — box, sphere, cylinder, prism, sector, ring, tetrahedron —
// and the guide case's table is the user's hand-check (D7.5's
// arithmetic, now with the numbers):
//
//   box a×b×c:                V = abc,            I_xx = abc(b²+c²)/12
//   sphere R:                 V = (4/3)πR³,       I = (2/5)VR²
//   cylinder r×h (axis Y):    V = πr²h,           I_axis = πr⁴h/2,
//                                                 I_diam = V(3r²+h²)/12
//   circle prism r×h:         V = πr²h (the per-face decomposition:
//                             caps 0 + (1/3)hπr², wall (2/3)πr²h)
//   sector prism (pie r,Δ):   V = (r²Δ/2)h, the centroid on the
//                             bisector at (2r sin(Δ/2))/(3Δ/2)
//   ring prism:               V = (A_rect − πr_h²)h
//   guide case (box − hole):  V = abc − πr²h,
//                             I_zz = abc(a²+b²)/12 − πr⁴h/2
//                             (the removed cylinder is coaxial)
//
// The machinery under test: the divergence theorem over the EXACT
// faces — V = (1/3)∮x·n, M_i = (1/2)∮x_i²n_i, S_ij = (1/5)∮x_i x_j
// (x·n) — the planar families by Green's theorem in the face's own
// (u,v) domain (the outer-minus-inner arithmetic IS the region
// semantics), the cylindrical patches by the (θ,z) rectangle
// integrals, the full sphere by its closed form.
//
// The honesty contract: all_exact true only when every contributing
// face was exact; a faceted body reports the POLYHEDRAL value with
// the flag false (never silently mixed); out-of-class faces error
// actionably (the family named, the remedy named).
#include <doctest/doctest.h>

#include "CAD_0/brep/mass_properties.hpp"
#include "CAD_0/brep/factory.hpp"
#include "CAD_0/brep/extrude.hpp"
#include "CAD_0/brep/subtract.hpp"
#include "CAD_0/brep/analytic.hpp"
#include "CAD_0/brep/query.hpp"
#include "CAD_0/brep/topology.hpp"
#include "CAD_0/sdf/sketch_entities.hpp"
#include "CAD_0/sdf/sketch_profile.hpp"
#include "CAD_0/sdf/sketch_extrude.hpp"
#include "CAD_0/bridge/sdf_to_brep.hpp"
#include "CAD_0/scene/scene.hpp"
#include "CAD_0/scene/operations.hpp"
#include "CAD_0/io/native.hpp"

#include <cmath>
#include <cstdio>
#include <filesystem>
#include <string>
#include <vector>

namespace {

using CAD_0::brep::BRepBody;
using CAD_0::math::Tolerance;
using CAD_0::math::Vec2f;
using CAD_0::math::Vec3d;
namespace b = CAD_0::brep;
namespace s = CAD_0::sdf;

constexpr double kPid = 3.14159265358979323846;
constexpr float kPi = 3.14159265358979f;

const Tolerance kTol = Tolerance::linear(1e-6);

// The precision doctrine (pinned once, used everywhere): the
// machinery's arithmetic is closed-form and double, and the INPUTS are
// the stored trimming parameters — the p-curves' floats (the
// float-pi sweeps of the seam-wired walks, the projected rectangle
// coordinates of a diagonal frame, the rounded window heights). The
// honest comparison bound is therefore a few float ulps, 1e-6
// relative — still five orders tighter than any faceted extract
// (which the honesty tests bound at the percent level). The
// exactly-representable bodies (the box's integer frames, the
// sphere's lone radius) would pass tighter; the uniform honest bound
// is what the suite pins.
bool close(double a, double b, double rel = 1e-6, double abs = 1e-12) {
    return std::abs(a - b) <= std::max(abs, rel * std::max(std::abs(a), std::abs(b)));
}

// The inertia as a comparable 3x3 (row-major access via Mat3(r, c)).
// The off-diagonal zeros are CANCELLATION-HEAVY: the products of
// inertia come together from ~O(10) kernel terms whose float-stored
// inputs (the pi-sweeps, the projected frames) leave ~1e-6 residue on
// values that cancel to zero — the absolute bound follows the storage
// (the diagonal's relative bound stays 1e-5).
bool close3x3(const CAD_0::math::Mat3d& m, double v00, double v01, double v02,
              double v10, double v11, double v12, double v20, double v21,
              double v22, double rel = 1e-5) {
    const double want[3][3] = {{v00, v01, v02}, {v10, v11, v12}, {v20, v21, v22}};
    for (int r = 0; r < 3; ++r)
        for (int c = 0; c < 3; ++c) {
            const double abs_bound = want[r][c] == 0.0 ? 1e-5 : 1e-12;
            if (!close(m(r, c), want[r][c], rel, abs_bound))
                return false;
        }
    return true;
}

// ---- the profiles (the unit-3/4 vocabulary, reused) ----------------------

std::vector<s::SketchChain> circle_chains(Vec2f center = {0.0f, 0.0f},
                                          float radius = 0.25f) {
    return {s::SketchChain{{s::sketch_circle(center, radius, false)}}};
}

std::vector<s::SketchChain> rect_chains(Vec2f lo = {-2.0f, -1.0f},
                                        Vec2f hi = {2.0f, 1.0f}) {
    return {s::SketchChain{{
        s::sketch_segment(lo, {hi.x, lo.y}, true),
        s::sketch_segment({hi.x, lo.y}, hi, true),
        s::sketch_segment(hi, {lo.x, hi.y}, true),
        s::sketch_segment({lo.x, hi.y}, lo, true),
    }}};
}

std::vector<s::SketchChain> sector_chains() {
    const float a0 = -0.2f * kPi, a1 = 0.35f * kPi;
    const float r = 0.4f;
    const Vec2f c{0.0f, 0.0f};
    const Vec2f ps = c + Vec2f{r * std::cos(a0), r * std::sin(a0)};
    const Vec2f pe = c + Vec2f{r * std::cos(a1), r * std::sin(a1)};
    return {s::SketchChain{{
        s::sketch_segment(c, ps, true),
        s::sketch_arc(c, r, false, a0, a1 - a0),
        s::sketch_segment(pe, c, true),
    }}};
}

std::vector<s::SketchChain> ring_chains() {
    auto chains = rect_chains();
    chains.push_back(s::SketchChain{
        {s::sketch_circle({0.0f, 0.0f}, 0.2f, true)}});
    return chains;
}

// The digest for the determinism pins (the unit-3 oracle's shape):
// every number as a hex float, concatenated.
std::string mass_digest(const b::MassProperties& mp) {
    std::string out;
    char buf[64];
    std::snprintf(buf, sizeof buf, "%a", mp.volume);
    out += buf;
    out += ",";
    for (int i = 0; i < 3; ++i) {
        std::snprintf(buf, sizeof buf, "%a",
                      i == 0 ? mp.centroid.x : i == 1 ? mp.centroid.y
                                                      : mp.centroid.z);
        out += buf;
    }
    for (int r = 0; r < 3; ++r)
        for (int c = 0; c < 3; ++c) {
            std::snprintf(buf, sizeof buf, "%a", mp.inertia_about_centroid(r, c));
            out += ",";
            out += buf;
        }
    out += mp.all_exact ? ",E" : ",A";
    return out;
}

std::uint32_t add_extrude(CAD_0::scene::Scene& sc, const std::string& name,
                          std::uint32_t sketch_id, float distance,
                          int direction) {
    CAD_0::scene::OperationRecipe r;
    r.type = CAD_0::scene::OperationType::Extrude;
    r.sketch = sketch_id;
    r.distance = distance;
    r.direction = direction;
    return sc.add_operation_node(name, r);
}

} // namespace

// ---------------------------------------------------------------------------
// FAMILY 1 — the classical formulas (the golden values ARE the tests)
// ---------------------------------------------------------------------------
TEST_CASE("G1-U5 golden: the box a×b×c — volume, centroid, the inertia diag") {
    const float hx = 1.3f, hy = 0.9f, hz = 2.1f;  // half extents
    auto body = b::make_box({hx, hy, hz});
    auto mp = b::mass_properties(*body, kTol);

    // The classical formulas in FULL dims (the test's own arithmetic).
    const double A = 2.0 * hx, B = 2.0 * hy, C = 2.0 * hz;
    const double V = A * B * C;
    const double Ixx = V * (B * B + C * C) / 12.0;
    const double Iyy = V * (A * A + C * C) / 12.0;
    const double Izz = V * (A * A + B * B) / 12.0;

    CHECK(mp.all_exact);
    CHECK(close(mp.volume, V));
    CHECK(close(mp.centroid.x, 0.0, 1e-6, 1e-6));
    CHECK(close(mp.centroid.y, 0.0, 1e-6, 1e-6));
    CHECK(close(mp.centroid.z, 0.0, 1e-6, 1e-6));
    CHECK(close3x3(mp.inertia_about_centroid,
                   Ixx, 0, 0, 0, Iyy, 0, 0, 0, Izz));
}

TEST_CASE("G1-U5 golden: the sphere R — (4/3)πR³, I = (2/5)VR²") {
    const float R = 1.7f;
    auto body = b::make_sphere(R);
    auto mp = b::mass_properties(*body, kTol);

    const double V = (4.0 / 3.0) * kPid * R * R * R;
    const double I = (2.0 / 5.0) * V * R * R;
    CHECK(mp.all_exact);
    CHECK(close(mp.volume, V));
    CHECK(close(mp.centroid.x, 0.0, 1e-6, 1e-6));
    CHECK(close(mp.centroid.y, 0.0, 1e-6, 1e-6));
    CHECK(close(mp.centroid.z, 0.0, 1e-6, 1e-6));
    CHECK(close3x3(mp.inertia_about_centroid,
                   I, 0, 0, 0, I, 0, 0, 0, I));
}

TEST_CASE("G1-U5 golden: the TRANSLATED sphere — the parallel-axis check") {
    const float R = 1.1f;
    auto body = b::make_sphere(R);
    b::translate_body(*body, {2.0f, -3.0f, 0.5f});
    auto mp = b::mass_properties(*body, kTol);

    const double V = (4.0 / 3.0) * kPid * R * R * R;
    const double I = (2.0 / 5.0) * V * R * R;  // about the centroid: invariant
    CHECK(mp.all_exact);
    CHECK(close(mp.volume, V));
    CHECK(close(mp.centroid.x, 2.0, 1e-6));
    CHECK(close(mp.centroid.y, -3.0, 1e-6));
    CHECK(close(mp.centroid.z, 0.5, 1e-6));
    CHECK(close3x3(mp.inertia_about_centroid,
                   I, 0, 0, 0, I, 0, 0, 0, I));
}

TEST_CASE("G1-U5 golden: the factory cylinder r×h — V, the axis and diameter inertias") {
    const float r = 0.8f, h = 2.6f;
    auto body = b::make_cylinder(r, h);
    auto mp = b::mass_properties(*body, kTol);

    const double V = kPid * r * r * h;
    const double I_axis = kPid * r * r * r * r * h / 2.0;   // about Y
    const double I_diam = V * (3.0 * r * r + h * h) / 12.0; // about X, Z
    CHECK(mp.all_exact);
    CHECK(close(mp.volume, V));
    CHECK(close(mp.centroid.y, 0.0, 1e-6, 1e-6));
    CHECK(close3x3(mp.inertia_about_centroid,
                   I_diam, 0, 0, 0, I_axis, 0, 0, 0, I_diam));
}

TEST_CASE("G1-U5 golden: the circle prism r×h — the per-face decomposition's total") {
    // The decomposition IS the derivation: the cap at z=0 contributes
    // (1/3)·(x·n=−z=0)·A = 0; the cap at z=h contributes (1/3)h·πr²;
    // the wall (1/3)·r·2πrh = (2/3)πr²h — total πr²h. The caps' arcs
    // and the wall's seam-wired rectangle are the exact faces.
    const Vec2f c{0.5f, 0.25f};
    const float r = 1.25f, h = 3.0f;
    auto body = b::make_prism(circle_chains(c, r), h, +1);
    auto mp = b::mass_properties(*body, kTol);

    const double V = kPid * r * r * h;
    const double Izz = kPid * r * r * r * r * h / 2.0;      // the axis
    const double Ixx = V * (3.0 * r * r + h * h) / 12.0;    // a diameter
    CHECK(mp.all_exact);
    CHECK(close(mp.volume, V));
    CHECK(close(mp.centroid.x, c.x, 1e-6));
    CHECK(close(mp.centroid.y, c.y, 1e-6));
    CHECK(close(mp.centroid.z, h / 2.0, 1e-6));
    CHECK(close3x3(mp.inertia_about_centroid,
                   Ixx, 0, 0, 0, Ixx, 0, 0, 0, Izz));
}

TEST_CASE("G1-U5 golden: the sector prism — the arc machinery vs the pie formulas") {
    // The pie slice: area r²Δ/2, the centroid on the bisector at
    // (2r sin(Δ/2))/(3Δ/2) from the apex; I_zz(origin) = r⁴Δ/4 per
    // unit height (∫ρ²ρ dρ dθ), then the parallel axis to the
    // centroid — every number a textbook sector formula.
    const float r = 0.4f, a0 = -0.2f * kPi, a1 = 0.35f * kPi;
    const double sweep = a1 - a0, h = 2.0;
    auto body = b::make_prism(sector_chains(), static_cast<float>(h), +1);
    auto mp = b::mass_properties(*body, kTol);

    const double area = r * r * sweep / 2.0;
    const double V = area * h;
    const double alpha = sweep / 2.0;                      // the half angle
    const double d = 2.0 * r * std::sin(alpha) / (3.0 * alpha);
    const double bis = 0.5 * (a0 + a1);
    const double cx = d * std::cos(bis), cy = d * std::sin(bis);
    const double Izz_o = r * r * r * r * sweep / 4.0 * h;  // about the apex axis
    const double Izz = Izz_o - V * (cx * cx + cy * cy);    // parallel axis

    CHECK(mp.all_exact);
    CHECK(close(mp.volume, V));
    CHECK(close(mp.centroid.x, cx, 1e-5));
    CHECK(close(mp.centroid.y, cy, 1e-5));
    CHECK(close(mp.centroid.z, h / 2.0, 1e-6));
    CHECK(close(mp.inertia_about_centroid(2, 2), Izz, 1e-5));
    // The symmetric body: the products of inertia vanish about the
    // bisector... not in the world axes — but the tensor IS symmetric.
    CHECK(close(mp.inertia_about_centroid(0, 1), mp.inertia_about_centroid(1, 0), 1e-12, 0.0));
}

TEST_CASE("G1-U5 golden: the ring prism — the Circle2 hole subtraction") {
    // The rect [−2,2]×[−1,1] with a hole r=0.2 at the centre: the
    // caps carry the hole as an inner Circle2 loop — the disc's
    // closed-form moments subtracted by the positional semantics.
    const float rh = 0.2f, h = 1.5f;
    auto body = b::make_prism(ring_chains(), h, +1);
    auto mp = b::mass_properties(*body, kTol);

    const double V = (8.0 - kPid * rh * rh) * h;
    const double V_rect = 8.0 * h;
    const double Izz_rect = V_rect * (16.0 + 4.0) / 12.0;  // box A=4,B=2
    const double Izz_hole = kPid * rh * rh * rh * rh * h / 2.0;
    CHECK(mp.all_exact);
    CHECK(close(mp.volume, V));
    CHECK(close(mp.centroid.z, h / 2.0, 1e-6));
    CHECK(close(mp.inertia_about_centroid(2, 2), Izz_rect - Izz_hole, 1e-5));
}

TEST_CASE("G1-U5 golden: the tetrahedron — V from the vertex determinant") {
    auto body = b::make_tetrahedron();
    auto mp = b::mass_properties(*body, kTol);
    // The factory's vertices: (1,1,1), (−1,−1,1), (−1,1,−1), (1,−1,−1):
    // V = |det(v1−v0, v2−v0, v3−v0)|/6 = 16/6; the centroid is the
    // vertex average (the origin).
    const double V = 16.0 / 6.0;
    CHECK(mp.all_exact);
    CHECK(close(mp.volume, V));
    CHECK(close(mp.centroid.x, 0.0, 1e-6, 1e-6));
    CHECK(close(mp.centroid.y, 0.0, 1e-6, 1e-6));
    CHECK(close(mp.centroid.z, 0.0, 1e-6, 1e-6));
}

// ---------------------------------------------------------------------------
// FAMILY 2 — THE GUIDE CASE (D7.5's table, the user's hand-check)
// ---------------------------------------------------------------------------
TEST_CASE("G1-U5 guide: box − through hole — abc − πr²h, I_zz, the full table") {
    // The unit-4 guide shape: the box [−1,1]³ (a=b=c=2), the hole
    // r=0.25 through Z (h=2). Every number below is D7.5's table.
    auto a = b::make_box({1.0f, 1.0f, 1.0f});
    auto tool = b::make_prism(circle_chains(), 3.0f, +1);
    auto body = b::exact_subtract(*a, *tool, {0.0f, 0.0f, -1.5f},
                                  b::kSubtractTolerance);
    REQUIRE(body != nullptr);
    auto mp = b::mass_properties(*body, kTol);

    const double abc = 2.0 * 2.0 * 2.0;
    const double r = 0.25, h = 2.0;
    const double m = kPid * r * r * h;                 // the removed mass
    const double V = abc - m;
    const double Izz = abc * (4.0 + 4.0) / 12.0 - kPid * r * r * r * r * h / 2.0;
    const double Ixx = abc * (4.0 + 4.0) / 12.0 - m * (3.0 * r * r + h * h) / 12.0;

    CHECK(mp.all_exact);
    CHECK(close(mp.volume, V));
    CHECK(close(mp.centroid.x, 0.0, 1e-6, 1e-6));
    CHECK(close(mp.centroid.y, 0.0, 1e-6, 1e-6));
    CHECK(close(mp.centroid.z, 0.0, 1e-6, 1e-6));
    CHECK(close3x3(mp.inertia_about_centroid,
                   Ixx, 0, 0, 0, Ixx, 0, 0, 0, Izz, 1e-5));
}

TEST_CASE("G1-U5 guide: the OFF-CENTRE hole — the centroid shift and the parallel axis") {
    // The hole at (0.3, −0.2): the body is no longer symmetric — the
    // centroid shifts AWAY from the removed material, and the inertia
    // picks up the composite parallel-axis terms. The test derives
    // every expected value by hand, the classical composite way.
    auto a = b::make_box({1.0f, 1.0f, 1.0f});
    auto tool = b::make_prism(circle_chains({0.3f, -0.2f}, 0.25f), 3.0f, +1);
    auto body = b::exact_subtract(*a, *tool, {0.0f, 0.0f, -1.5f},
                                  b::kSubtractTolerance);
    REQUIRE(body != nullptr);
    auto mp = b::mass_properties(*body, kTol);

    const double abc = 8.0;
    const double r = 0.25, h = 2.0;
    const double dx = 0.3, dy = -0.2;
    const double m = kPid * r * r * h;
    const double V = abc - m;

    // First moments: the box's are zero (the centroid at the origin);
    // the removed cylinder's are −m·(dx, dy, 0).
    const double Mx = -m * dx, My = -m * dy;
    const double cx = Mx / V, cy = My / V;

    // I_zz about the world origin: box − [cylinder(own) + m·(dx²+dy²)].
    const double Izz_o = abc * 8.0 / 12.0 -
                         (kPid * r * r * r * r * h / 2.0 + m * (dx * dx + dy * dy));
    // Then the parallel axis to the body's centroid.
    const double Izz = Izz_o - V * (cx * cx + cy * cy);

    // I_xx about the origin: box − [cylinder(own, about its centroid) +
    // m·(dy² + dz²)] with the cylinder's centroid at (dx, dy, 0).
    const double Icyl_xx_own = m * (3.0 * r * r + h * h) / 12.0;
    const double Ixx_o = abc * 8.0 / 12.0 - (Icyl_xx_own + m * dy * dy);
    const double Ixx = Ixx_o - V * (cy * cy);

    CHECK(mp.all_exact);
    CHECK(close(mp.volume, V));
    CHECK(close(mp.centroid.x, cx, 1e-5));
    CHECK(close(mp.centroid.y, cy, 1e-5));
    CHECK(close(mp.centroid.z, 0.0, 1e-6, 1e-6));
    CHECK(close(mp.inertia_about_centroid(2, 2), Izz, 1e-5));
    CHECK(close(mp.inertia_about_centroid(0, 0), Ixx, 1e-5));
}

TEST_CASE("G1-U5 guide: the COPLANAR flush cap — the same numbers (absorbed)") {
    // The tool's bottom cap flush with the box's bottom face: the
    // coplanar dedup absorbs it — the geometry is the same through
    // hole, the numbers must be the guide's own.
    auto a = b::make_box({1.0f, 1.0f, 1.0f});
    auto tool = b::make_prism(circle_chains(), 3.0f, +1);
    auto body = b::exact_subtract(*a, *tool, {0.0f, 0.0f, -1.0f},
                                  b::kSubtractTolerance);
    REQUIRE(body != nullptr);
    auto mp = b::mass_properties(*body, kTol);
    const double V = 8.0 - kPid * 0.0625 * 2.0;
    CHECK(mp.all_exact);
    CHECK(close(mp.volume, V));
    CHECK(close(mp.centroid.z, 0.0, 1e-6, 1e-6));
}

TEST_CASE("G1-U5 guide: the PRISM TARGET — the extrude's own caps pierced") {
    // A = the extruded rectangle [−1,1]² × [0,2] (the extrude's caps,
    // not the factory's): the same through hole, the same numbers —
    // the planar machinery reads whichever authoring produced the
    // faces (the factories' Line2 rectangles or the caps' chain
    // loops: the arithmetic is the same region semantics).
    auto a = b::make_prism(rect_chains({-1.0f, -1.0f}, {1.0f, 1.0f}), 2.0f, +1);
    auto tool = b::make_prism(circle_chains(), 3.0f, +1);
    auto body = b::exact_subtract(*a, *tool, {0.0f, 0.0f, -0.5f},
                                  b::kSubtractTolerance);
    REQUIRE(body != nullptr);
    auto mp = b::mass_properties(*body, kTol);
    const double V = 4.0 * 2.0 - kPid * 0.0625 * 2.0;
    CHECK(mp.all_exact);
    CHECK(close(mp.volume, V));
    CHECK(close(mp.centroid.z, 1.0, 1e-6));  // the target spans [0, 2]
}

TEST_CASE("G1-U5 golden: the v0.4 golden's exact subtract — the guide numbers on load") {
    const std::filesystem::path golden =
        std::filesystem::path(CAD_0_SOURCE_DIR) / "tests" / "golden" /
        "native_v0_4_0.CAD_0";
    REQUIRE(std::filesystem::exists(golden));

    auto doc = CAD_0::io::native::load_file(golden);
    CAD_0::scene::Scene sc;
    CAD_0::io::native::install_document(sc, std::move(doc));
    const auto* sub = sc.get_node(4);
    REQUIRE(sub != nullptr);
    REQUIRE(sub->hybrid != nullptr);

    auto mp = b::mass_properties(*sub->hybrid->brep, kTol);
    const double V = 8.0 - kPid * 0.0625 * 2.0;
    const double Izz = 16.0 / 3.0 - kPid * 0.00390625 * 2.0 / 2.0;
    CHECK(mp.all_exact);
    CHECK(close(mp.volume, V));
    CHECK(close(mp.centroid.x, 0.0, 1e-6, 1e-6));
    CHECK(close(mp.centroid.y, 0.0, 1e-6, 1e-6));
    CHECK(close(mp.inertia_about_centroid(2, 2), Izz, 1e-5));

    // Determinism through a save/reload cycle (ADR-0014 + D11: the
    // hybrid re-materializes, the numbers regenerate identically).
    CAD_0::io::native::NativeCamera cam{};
    const std::string text =
        CAD_0::io::native::serialize_document(sc, cam);
    auto doc2 = CAD_0::io::native::parse_document(text);
    CAD_0::scene::Scene sc2;
    CAD_0::io::native::install_document(sc2, std::move(doc2));
    const auto* sub2 = sc2.get_node(4);
    REQUIRE(sub2 != nullptr);
    REQUIRE(sub2->hybrid != nullptr);
    auto mp2 = b::mass_properties(*sub2->hybrid->brep, kTol);
    CHECK(mass_digest(mp2) == mass_digest(mp));
}

// ---------------------------------------------------------------------------
// FAMILY 3 — the honesty of all_exact (the polyhedral path, never mixed)
// ---------------------------------------------------------------------------
TEST_CASE("G1-U5 honesty: the faceted body says polyhedral + all_exact=false") {
    // The same sphere through the faceted bridge: N triangles, no
    // p-curves, approximated — the polyhedral divergence value (the
    // Mirtich reduction) with the flag FALSE; the exact sphere says
    // TRUE. Never a silent mix.
    CAD_0::bridge::SdfToBrepOptions opts;
    opts.fit_primitives = false;
    opts.march_resolution = 24;
    auto faceted = CAD_0::bridge::sdf_to_brep_faceted(
        s::SDFBody(std::make_unique<s::SphereSDF>(1.7f)), opts);
    REQUIRE(faceted != nullptr);
    auto mpf = b::mass_properties(*faceted, kTol);
    CHECK_FALSE(mpf.all_exact);
    const double V = (4.0 / 3.0) * kPid * 1.7 * 1.7 * 1.7;
    CHECK(close(mpf.volume, V, 2e-2));       // the polyhedral approximation
    CHECK(mpf.volume < V);                    // inside the sphere

    auto exact = b::make_sphere(1.7f);
    auto mpe = b::mass_properties(*exact, kTol);
    CHECK(mpe.all_exact);
    CHECK(close(mpe.volume, V));
}

TEST_CASE("G1-U5 honesty: the guide's SDF side faceted, the exact side exact") {
    // The unit-4 landing's own contrast, now with numbers: the
    // subtract node's SDF through the bridge is polyhedral+false; the
    // SAME node's exact B-Rep is the closed form+true. Two
    // materializations, both honest, never confused.
    CAD_0::scene::Scene sc;
    const auto bid = sc.add_node(
        "Box", s::SDFBody(std::make_unique<s::BoxSDF>(
                               CAD_0::math::Vec3f{1.0f, 1.0f, 1.0f})));
    const auto sid = sc.add_sketch_node("S", circle_chains());
    const auto eid = add_extrude(sc, "Extrude", sid, 3.0f, +1);
    CAD_0::scene::OperationRecipe sub;
    sub.type = CAD_0::scene::OperationType::Subtract;
    sub.target = bid;
    sub.tool = eid;
    sub.offset_z = -1.5f;
    const auto did = sc.add_operation_node("Sub", sub);
    const auto* n = sc.get_node(did);
    REQUIRE(n != nullptr);
    REQUIRE(n->hybrid != nullptr);

    auto mpe = b::mass_properties(*n->hybrid->brep, kTol);
    CHECK(mpe.all_exact);
    CHECK(close(mpe.volume, 8.0 - kPid * 0.0625 * 2.0));

    CAD_0::bridge::SdfToBrepOptions opts;
    opts.fit_primitives = false;
    opts.march_resolution = 24;
    auto faceted = CAD_0::bridge::sdf_to_brep_faceted(n->body, opts);
    REQUIRE(faceted != nullptr);
    auto mpf = b::mass_properties(*faceted, kTol);
    CHECK_FALSE(mpf.all_exact);
    CHECK(close(mpf.volume, 8.0 - kPid * 0.0625 * 2.0, 2e-2));
}

TEST_CASE("G1-U5 honesty: a MIXED body takes the polyhedral path, never a mix") {
    // A box with ONE face relabeled approximated: the record is the
    // thing under test (the unit-1 precedent) — the machinery answers
    // the polyhedral value with the flag false, never a silent mix of
    // closed-form and polyhedral faces in one sum. A box's polyhedral
    // reduction is numerically its exact value (planar faces), so the
    // FLAG is the discriminator here.
    auto body = b::make_box({1.0f, 0.7f, 1.4f});
    body->solid.shell_mut().faces_mut()[3].set_provenance(
        CAD_0::brep::ProvenanceInfo::approximated(1e-3));
    auto mp = b::mass_properties(*body, kTol);
    CHECK_FALSE(mp.all_exact);
    CHECK(close(mp.volume, 2.0 * 1.4f * 2.8f));
    CHECK(close(mp.centroid.y, 0.0, 1e-6, 1e-6));
}

TEST_CASE("G1-U5 honesty: an exact-RELABELED faceted body errors (no p-curves)") {
    // The lie detector: a faceted body whose record CLAIMS exact —
    // the exact path demands p-curves (the exact world's invariant)
    // and refuses loudly. The machinery honours the record and never
    // launders it.
    CAD_0::bridge::SdfToBrepOptions opts;
    opts.fit_primitives = false;
    opts.march_resolution = 16;
    auto faceted = CAD_0::bridge::sdf_to_brep_faceted(
        s::SDFBody(std::make_unique<s::SphereSDF>(1.0f)), opts);
    REQUIRE(faceted != nullptr);
    for (auto& f : faceted->solid.shell_mut().faces_mut())
        f.set_provenance(CAD_0::brep::ProvenanceInfo::exact());
    bool threw = false;
    std::string msg;
    try {
        auto mp = b::mass_properties(*faceted, kTol);
        (void)mp;
    } catch (const std::invalid_argument& e) {
        threw = true;
        msg = e.what();
    }
    REQUIRE(threw);
    CHECK(msg.find("p-curve") != std::string::npos);
    CHECK(msg.find("Remedy") != std::string::npos);
}

// ---------------------------------------------------------------------------
// FAMILY 4 — the error contract (actionable, the family and remedy named)
// ---------------------------------------------------------------------------
namespace {

// A structurally-valid one-face body carrying the given surface —
// the class-check fixture: two vertices, one edge, one face whose
// single loop uses the edge as a SEAM PAIR (the sphere's own pattern,
// forward + reversed), so the closed-orientable contract passes and
// the FAMILY check is what fires.
std::unique_ptr<BRepBody> one_face_body(
    std::shared_ptr<CAD_0::brep::SurfaceGeometry> surf) {
    auto body = std::make_unique<BRepBody>();
    b::Vertex v0{Vec3d{0.0f, 0.0f, 0.5f}};
    v0.set_id(0);
    b::Vertex v1{Vec3d{0.0f, 0.0f, -0.5f}};
    v1.set_id(1);
    body->vertices.push_back(v0);
    body->vertices.push_back(v1);
    b::Edge e{0, 1};
    e.set_id(0);
    body->edges.push_back(e);
    b::Face f(std::move(surf));
    b::Loop loop;
    loop.add_coedge(b::CoEdge{0, false});
    loop.add_coedge(b::CoEdge{0, true});
    f.add_loop(std::move(loop));
    f.set_id(0);
    auto& shell = body->solid.shell_mut();
    shell.set_closed(true);
    shell.add_face(std::move(f));
    b::link_radial_edges(*body);
    return body;
}

} // namespace

TEST_CASE("G1-U5 error contract: the cone and torus families refuse actionably") {
    std::vector<std::shared_ptr<CAD_0::brep::SurfaceGeometry>> surfs;
    surfs.push_back(std::make_shared<b::ConeSurface>(0.5f, 1.0f));
    surfs.push_back(std::make_shared<b::TorusSurface>(1.0f, 0.3f));
    for (const auto& surf : surfs) {
        auto body = one_face_body(surf);
        bool threw = false;
        std::string msg;
        try {
            auto mp = b::mass_properties(*body, kTol);
            (void)mp;
        } catch (const std::invalid_argument& e) {
            threw = true;
            msg = e.what();
        }
        REQUIRE(threw);
        CHECK(msg.find(surf->type_name()) != std::string::npos);
        CHECK(msg.find("Remedy") != std::string::npos);
    }
}

TEST_CASE("G1-U5 error contract: the EMPTY body refuses actionably") {
    auto body = std::make_unique<BRepBody>();
    bool threw = false;
    std::string msg;
    try {
        auto mp = b::mass_properties(*body, kTol);
        (void)mp;
    } catch (const std::invalid_argument& e) {
        threw = true;
        msg = e.what();
    }
    REQUIRE(threw);
    CHECK(msg.find("no faces") != std::string::npos);
    CHECK(msg.find("Remedy") != std::string::npos);
}

// ---------------------------------------------------------------------------
// FAMILY 5 — determinism (ADR-0014): bit-identical re-runs
// ---------------------------------------------------------------------------
TEST_CASE("G1-U5 determinism: the guide case re-runs bit-identically") {
    const auto build = [] {
        auto a = b::make_box({1.0f, 1.0f, 1.0f});
        auto tool = b::make_prism(circle_chains({0.2f, 0.1f}, 0.3f), 3.0f, +1);
        return b::exact_subtract(*a, *tool, {0.0f, 0.0f, -1.5f},
                                 b::kSubtractTolerance);
    };
    auto b1 = build();
    auto b2 = build();
    REQUIRE(b1 != nullptr);
    REQUIRE(b2 != nullptr);
    auto mp1 = b::mass_properties(*b1, kTol);
    auto mp2 = b::mass_properties(*b2, kTol);
    CHECK(mass_digest(mp1) == mass_digest(mp2));

    // And the same body twice (the pure re-run).
    auto mp3 = b::mass_properties(*b1, kTol);
    CHECK(mass_digest(mp1) == mass_digest(mp3));
}
