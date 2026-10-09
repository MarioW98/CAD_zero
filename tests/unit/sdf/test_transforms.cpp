// tests/unit/sdf/test_transforms.cpp
#include <doctest/doctest.h>

#include "CAD_0/sdf/primitives.hpp"
#include "CAD_0/sdf/transforms.hpp"

using namespace CAD_0::sdf;
using namespace CAD_0::math;

TEST_CASE("Translate: shifts the field") {
    auto sph = make_sphere(1.0f);
    auto t   = sdf_translate(std::move(sph), {5, 0, 0});
    CHECK(t.value({5, 0, 0}) == doctest::Approx(-1.0f));
    CHECK(t.value({6, 0, 0}) == doctest::Approx(0.0f).epsilon(1e-5f));
    CHECK(t.value({0, 0, 0}) == doctest::Approx(4.0f).epsilon(1e-5f));
}

TEST_CASE("Translate: bounds move with the field") {
    auto sph = make_sphere(2.0f);
    auto t   = sdf_translate(std::move(sph), {3, 0, 0});
    auto b = t.bounds();
    CHECK(b.min.x == doctest::Approx(1.0f));
    CHECK(b.max.x == doctest::Approx(5.0f));
}

TEST_CASE("Scale: scales the field") {
    auto sph = make_sphere(1.0f);
    auto s   = sdf_scale(std::move(sph), 2.0f);
    CHECK(s.value({0, 0, 0}) == doctest::Approx(-2.0f));
    CHECK(s.value({3, 0, 0}) == doctest::Approx(1.0f));
}

TEST_CASE("Rotate: rotates the field") {
    auto box = make_box({1, 1, 1});
    // Rotate 90 degrees around Z. A box of extent 1,1,1 looks the same
    // after this rotation, so we expect the field at (1,0,0) to equal
    // the field at (0,1,0) before rotation (i.e. 0).
    auto q = Quatf::from_axis_angle({0, 0, 1}, 3.14159265358979f / 2.0f);
    auto r = sdf_rotate(std::move(box), q);
    CHECK(r.value({1, 0, 0}) == doctest::Approx(0.0f).epsilon(1e-4f));
    CHECK(r.value({0, 1, 0}) == doctest::Approx(0.0f).epsilon(1e-4f));
}

TEST_CASE("Twist: changes the field along the twist axis") {
    auto box = make_box({1, 5, 1});
    auto t   = sdf_twist(std::move(box), 0.5f);
    // Sanity: at origin the twist is zero so the field equals the original box's
    CHECK(t.value({0, 0, 0}) < 0.0f); // Inside the box
    CHECK(t.lipschitz() > 1.0f); // Twist increases lipschitz constant
}

TEST_CASE("Translate + Scale composition") {
    auto sph = make_sphere(1.0f);
    auto s   = sdf_scale(std::move(sph), 2.0f);
    auto st  = sdf_translate(std::move(s), {10, 0, 0});
    CHECK(st.value({10, 0, 0}) == doctest::Approx(-2.0f));
    CHECK(st.value({12, 0, 0}) == doctest::Approx(0.0f).epsilon(1e-5f));
    CHECK(st.value({0, 0, 0})  == doctest::Approx(8.0f).epsilon(1e-5f));
}

// ===========================================================================
// ScaleSDF Lipschitz invariant regression tests (added after the bug fix).
//
// Bug: the previous implementation reported lipschitz = scale · child_lipschitz.
// For a unit-Lipschitz child (e.g. SphereSDF) scaled by s > 1, this gave
// lipschitz = s, which is wrong. The scaled field f(x) = s · g(x/s) has
// ∇f(x) = ∇g(x/s), so |∇f| = |∇g| ≤ lipschitz(g). The Lipschitz constant
// of the scaled field is the same as the child's, NOT scaled.
// ===========================================================================

TEST_CASE("ScaleSDF: lipschitz is preserved (not multiplied by scale)") {
    auto sph = make_sphere(1.0f);
    auto s2  = sdf_scale(make_sphere(1.0f), 2.0f);   // scale > 1
    auto s05 = sdf_scale(make_sphere(1.0f), 0.5f);   // scale < 1
    CHECK(sph.lipschitz() == 1.0f);
    CHECK(s2.lipschitz()  == 1.0f);  // was 2.0f before fix
    CHECK(s05.lipschitz() == 1.0f);  // was 0.5f before fix (also wrong!)
}

TEST_CASE("ScaleSDF: gradient magnitude is ≤ 1 (coherent with lipschitz)") {
    // For a unit-Lipschitz child, the scaled field must also have |∇f| ≤ 1.
    auto s = sdf_scale(make_sphere(1.0f), 2.0f);
    const Vec3f pts[] = {
        {0.0f, 0.0f, 0.0f},    // center
        {1.0f, 0.0f, 0.0f},    // on the surface
        {2.0f, 0.0f, 0.0f},    // outside
        {3.0f, 0.0f, 0.0f},    // far outside
    };
    for (const auto& p : pts) {
        const auto samp = s.sample(p);
        const float gl = samp.gradient.length();
        INFO("point:", p.x, p.y, p.z, " |g| =", gl);
        CHECK(gl <= 1.0f + 1e-5f);
    }
}

TEST_CASE("ScaleSDF: scaled sphere value is correct") {
    // Sanity: the value at the new surface (radius 2) is 0, and the value
    // at the center is -2 (the new radius, not -1 as for the unscaled sphere).
    auto s = sdf_scale(make_sphere(1.0f), 2.0f);
    CHECK(s.value({0, 0, 0}) == doctest::Approx(-2.0f));
    CHECK(s.value({2, 0, 0}) == doctest::Approx(0.0f).epsilon(1e-5f));
    CHECK(s.value({3, 0, 0}) == doctest::Approx(1.0f).epsilon(1e-5f));
}
