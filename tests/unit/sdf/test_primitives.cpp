// tests/unit/sdf/test_primitives.cpp
#include <doctest/doctest.h>

#include "CAD_0/sdf/primitives.hpp"

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
