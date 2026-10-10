// tests/unit/sdf/test_primitives.cpp
#include <doctest/doctest.h>

#include "CAD_0/sdf/primitives.hpp"
#include "CAD_0/sdf/transforms.hpp"

#include <cmath>

using namespace CAD_0::sdf;
using namespace CAD_0::math;

TEST_CASE("SphereSDF: value at center is -radius") {
    auto s = make_sphere(2.0f);
    CHECK(s.value({0, 0, 0}) == doctest::Approx(-2.0f));
}

TEST_CASE("SphereSDF: value at surface is zero") {
    auto s = make_sphere(1.0f);
    CHECK(s.value({1, 0, 0}) == doctest::Approx(0.0f).epsilon(1e-5f));
    CHECK(s.value({0, 1, 0}) == doctest::Approx(0.0f).epsilon(1e-5f));
    CHECK(s.value({0, 0, 1}) == doctest::Approx(0.0f).epsilon(1e-5f));
}

TEST_CASE("SphereSDF: value outside is positive") {
    auto s = make_sphere(1.0f);
    CHECK(s.value({3, 0, 0}) == doctest::Approx(2.0f));
}

TEST_CASE("SphereSDF: gradient magnitude is ≤ 1 (Lipschitz)") {
    auto s = make_sphere(2.0f);
    const auto sample = s.sample({1, 1, 1});
    CHECK(sample.gradient.length() <= 1.0f + 1e-5f);
}

TEST_CASE("SphereSDF: bounds") {
    auto s = make_sphere(3.0f);
    auto b = s.bounds();
    CHECK(b.min == Vec3f{-3, -3, -3});
    CHECK(b.max == Vec3f{3, 3, 3});
}

TEST_CASE("SphereSDF: lipschitz is 1") {
    auto s = make_sphere(1.0f);
    CHECK(s.lipschitz() == 1.0f);
}

TEST_CASE("BoxSDF: value inside is negative") {
    auto s = make_box({1, 1, 1});
    CHECK(s.value({0, 0, 0}) < 0.0f);
}

TEST_CASE("BoxSDF: value at corner surface is zero") {
    auto s = make_box({1, 1, 1});
    CHECK(s.value({1, 0, 0}) == doctest::Approx(0.0f).epsilon(1e-5f));
    CHECK(s.value({1, 1, 1}) == doctest::Approx(0.0f).epsilon(1e-5f));
}

TEST_CASE("BoxSDF: value outside is positive") {
    auto s = make_box({1, 1, 1});
    CHECK(s.value({2, 0, 0}) > 0.0f);
}

TEST_CASE("CylinderSDF: value at axis center is -radius") {
    auto s = make_cylinder(1.0f, 2.0f); // r=1, h=2 → half-height=1
    CHECK(s.value({0, 0, 0}) == doctest::Approx(-1.0f));
}

TEST_CASE("CylinderSDF: value on top cap surface") {
    auto s = make_cylinder(1.0f, 2.0f);
    CHECK(s.value({0, 1, 0}) == doctest::Approx(0.0f).epsilon(1e-5f));
}

// CylinderSDF gradient must have |∇| ≤ 1 everywhere (Lipschitz invariant).
// The previous implementation left g.y = ±1 *and* the radial component
// non-zero on the rim, producing |g| = √2. This regression test would
// catch that bug.
TEST_CASE("CylinderSDF: gradient magnitude is ≤ 1 (Lipschitz)") {
    auto s = make_cylinder(1.0f, 2.0f);
    // Sample at a few representative points.
    const Vec3f pts[] = {
        {0.0f, 0.0f, 0.0f},    // axis center, inside
        {0.5f, 0.0f, 0.0f},    // inside, on +X axis
        {1.0f, 0.0f, 0.0f},    // on the lateral surface
        {1.0f, 0.5f, 0.0f},    // on the lateral surface, near top
        {0.2f, 1.0f, 0.0f},    // on the top cap, near center  ← user-reported failure
        {0.9f, 1.0f, 0.0f},    // on the top cap, near rim
        {1.0f, 1.0f, 0.0f},    // exactly on the rim
        {2.0f, 2.0f, 0.0f},    // outside both axes
    };
    for (const auto& p : pts) {
        const auto samp = s.sample(p);
        const float gl = samp.gradient.length();
        INFO("point:", p.x, p.y, p.z, " |g| =", gl);
        CHECK(gl <= 1.0f + 1e-5f);
    }
}

TEST_CASE("CylinderSDF: gradient on lateral surface is radial (pointing outward)") {
    auto s = make_cylinder(1.0f, 2.0f);
    // At (1, 0, 0) — on the lateral surface, the gradient should be (1, 0, 0).
    const auto samp = s.sample({1.0f, 0.0f, 0.0f});
    CHECK(samp.gradient.x == doctest::Approx(1.0f).epsilon(1e-4f));
    CHECK(samp.gradient.y == doctest::Approx(0.0f).epsilon(1e-4f));
    CHECK(samp.gradient.z == doctest::Approx(0.0f).epsilon(1e-4f));
}

TEST_CASE("CylinderSDF: gradient on top cap is +Y") {
    auto s = make_cylinder(1.0f, 2.0f);
    // At (0.5, 1, 0) — on the top cap, the gradient should be (0, 1, 0).
    const auto samp = s.sample({0.5f, 1.0f, 0.0f});
    CHECK(samp.gradient.x == doctest::Approx(0.0f).epsilon(1e-4f));
    CHECK(samp.gradient.y == doctest::Approx(1.0f).epsilon(1e-4f));
    CHECK(samp.gradient.z == doctest::Approx(0.0f).epsilon(1e-4f));
}

TEST_CASE("TorusSDF: value at center is -minor_radius") {
    auto s = make_torus(2.0f, 0.5f); // R=2, r=0.5
    // Point on the ring center: {R, 0, 0} → distance = -r
    CHECK(s.value({2, 0, 0}) == doctest::Approx(-0.5f).epsilon(1e-5f));
}

TEST_CASE("PlaneSDF: y > 0 is inside (negative)") {
    auto s = make_plane();
    CHECK(s.value({0, 1, 0}) < 0.0f);   // y > 0 → inside → negative
    CHECK(s.value({0, -1, 0}) > 0.0f); // y < 0 → outside → positive
}

TEST_CASE("CapsuleSDF: value along axis") {
    auto s = make_capsule({0, 0, 0}, {0, 2, 0}, 0.5f);
    // Midpoint of capsule, on axis → distance = -radius
    CHECK(s.value({0, 1, 0}) == doctest::Approx(-0.5f));
}

// ----------------------------------------------------------------------
// ConeSDF — exact signed distance verification (after the formula fix)
// ----------------------------------------------------------------------
TEST_CASE("ConeSDF: value at center axis (inside) is negative") {
    // Cone with base r=1 at y=0, apex at y=2.
    auto c = make_cone(1.0f, 2.0f);
    // Point on the axis at y=1 (middle of the cone height): well inside.
    CHECK(c.value({0, 1, 0}) < 0.0f);
}

TEST_CASE("ConeSDF: value at base rim is zero") {
    auto c = make_cone(1.0f, 2.0f);
    // Point exactly on the base rim (r=1, y=0).
    CHECK(c.value({1, 0, 0}) == doctest::Approx(0.0f).epsilon(1e-4f));
    // Same with z instead of x.
    CHECK(c.value({0, 0, 1}) == doctest::Approx(0.0f).epsilon(1e-4f));
}

TEST_CASE("ConeSDF: value at apex is zero") {
    auto c = make_cone(1.0f, 2.0f);
    // Apex is at (0, 2, 0).
    CHECK(c.value({0, 2, 0}) == doctest::Approx(0.0f).epsilon(1e-4f));
}

TEST_CASE("ConeSDF: value at base center is zero (on the base cap)") {
    auto c = make_cone(1.0f, 2.0f);
    // Base center (0, 0, 0) is on the base cap surface.
    CHECK(c.value({0, 0, 0}) == doctest::Approx(0.0f).epsilon(1e-4f));
}

TEST_CASE("ConeSDF: value outside lateral surface is positive") {
    auto c = make_cone(1.0f, 2.0f);
    // Point outside the cone in the +X direction at mid-height.
    // At y=1, the cone radius is 0.5 (linear interp from r=1 at y=0 to r=0 at y=2).
    // So at (1, 1, 0) we are 0.5 outside the lateral surface.
    const float v = c.value({1.0f, 1.0f, 0.0f});
    CHECK(v > 0.0f);
    // Sanity: should be roughly 0.5 * cos(θ) where θ is the cone half-angle.
    // For r=1, h=2: cos(θ) = h/sqrt(r²+h²) = 2/sqrt(5) ≈ 0.894.
    // Expected: 0.5 * 0.894 ≈ 0.447.
    CHECK(v == doctest::Approx(0.4472f).epsilon(0.05f));
}

TEST_CASE("ConeSDF: value below the base is positive") {
    auto c = make_cone(1.0f, 2.0f);
    // Point at (0, -0.5, 0) — below the base cap.
    CHECK(c.value({0, -0.5f, 0}) == doctest::Approx(0.5f).epsilon(1e-4f));
}

TEST_CASE("ConeSDF: gradient magnitude is ≤ 1 (Lipschitz)") {
    auto c = make_cone(1.0f, 2.0f);
    const auto s = c.sample({1.0f, 1.0f, 0.0f});
    CHECK(s.gradient.length() <= 1.0f + 1e-5f);
}

TEST_CASE("ConeSDF: gradient on the lateral surface points outward") {
    auto c = make_cone(1.0f, 2.0f);
    // On the lateral surface at y=1 (where cone radius is 0.5).
    const auto s = c.sample({0.5f, 1.0f, 0.0f});
    // Outward normal in (rho, y) plane is (cos_θ, sin_θ) = (2/√5, 1/√5).
    // In 3D, at point (0.5, 1, 0): rho_hat = (1, 0, 0), so gradient = (cos_θ, sin_θ, 0).
    CHECK(s.gradient.x == doctest::Approx(2.0f / std::sqrt(5.0f)).epsilon(1e-3f));
    CHECK(s.gradient.y == doctest::Approx(1.0f / std::sqrt(5.0f)).epsilon(1e-3f));
    CHECK(s.gradient.z == doctest::Approx(0.0f).epsilon(1e-3f));
}

// ===========================================================================
// Bug 3 — ConeSDF under rotation
//
// Rotating cone(r=1, h=2) by rotZ(+90°) puts the cone axis along -X.
// The solid occupies x ∈ [-2, 0]. Expected values:
//   f(-1, 0, 0) → local (0, 1, 0) → inside, d ≈ -0.4472 (on axis at mid-height)
//   f(-2, 0, 0) → local (0, 2, 0) → apex, d = 0
//   f(-3, 0, 0) → local (0, 3, 0) → past apex, d = +1
//   f( 0, 0, 0) → local (0, 0, 0) → base center, d = 0
//   f( 1, 0, 0) → local (0, -1, 0) → below base, d = +1
// ===========================================================================

TEST_CASE("ConeSDF: rotated cone (rotZ +90°) values at axis points") {
    auto cone = make_cone(1.0f, 2.0f);
    // q = Quatf::from_axis_angle({0,0,1}, π/2) rotates +Y to -X.
    auto q = Quatf::from_axis_angle({0, 0, 1}, 3.14159265358979f / 2.0f);
    auto rotated = sdf_rotate(cone, q);

    // f(-1, 0, 0): on the cone axis at mid-height → inside.
    // Local (0, 1, 0) is inside the cone: d = -0.5·cos θ = -0.5 · 2/√5 ≈ -0.4472.
    CHECK(rotated.value({-1.0f, 0.0f, 0.0f}) == doctest::Approx(-0.4472f).epsilon(0.05f));

    // f(-2, 0, 0): apex → d = 0.
    CHECK(rotated.value({-2.0f, 0.0f, 0.0f}) == doctest::Approx(0.0f).epsilon(1e-3f));

    // f(-3, 0, 0): past apex → d = +1.
    CHECK(rotated.value({-3.0f, 0.0f, 0.0f}) == doctest::Approx(1.0f).epsilon(1e-3f));

    // f(0, 0, 0): base center → d = 0.
    CHECK(rotated.value({0.0f, 0.0f, 0.0f}) == doctest::Approx(0.0f).epsilon(1e-3f));

    // f(1, 0, 0): below base → d = +1.
    CHECK(rotated.value({1.0f, 0.0f, 0.0f}) == doctest::Approx(1.0f).epsilon(1e-3f));
}

TEST_CASE("ConeSDF: rotated cone gradient magnitude ≤ 1 (Lipschitz)") {
    auto cone = make_cone(1.0f, 2.0f);
    auto q = Quatf::from_axis_angle({0, 0, 1}, 3.14159265358979f / 2.0f);
    auto rotated = sdf_rotate(cone, q);

    // Sample at several points along the rotated axis.
    const Vec3f pts[] = {
        {-1.0f, 0.0f, 0.0f},   // inside
        {-1.5f, 0.0f, 0.0f},   // near apex
        {-2.0f, 0.0f, 0.0f},   // apex
        {-0.5f, 0.0f, 0.0f},   // near base
        {-1.0f, 0.5f, 0.0f},   // off-axis, inside
    };
    for (const auto& p : pts) {
        const auto s = rotated.sample(p);
        const float gl = s.gradient.length();
        INFO("point:", p.x, p.y, p.z, " |g| =", gl);
        CHECK(gl <= 1.0f + 1e-4f);
    }
}

TEST_CASE("ConeSDF: rotated cone + translate") {
    auto cone = make_cone(1.0f, 2.0f);
    auto q = Quatf::from_axis_angle({0, 0, 1}, 3.14159265358979f / 2.0f);
    auto rotated = sdf_rotate(cone, q);
    auto moved = sdf_translate(rotated, {5.0f, 0.0f, 0.0f});

    // The rotated cone had apex at (-2, 0, 0). After +5 translate, apex at (3, 0, 0).
    CHECK(moved.value({3.0f, 0.0f, 0.0f}) == doctest::Approx(0.0f).epsilon(1e-3f));
    // The rotated cone had base center at (0, 0, 0). After +5, at (5, 0, 0).
    CHECK(moved.value({5.0f, 0.0f, 0.0f}) == doctest::Approx(0.0f).epsilon(1e-3f));
    // Inside point (4, 0, 0) was at (-1, 0, 0) before translate → inside.
    CHECK(moved.value({4.0f, 0.0f, 0.0f}) < 0.0f);
}
