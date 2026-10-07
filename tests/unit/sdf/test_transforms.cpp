// tests/unit/sdf/test_transforms.cpp
#include <doctest/doctest.h>

#include "cadforge/sdf/primitives.hpp"
#include "cadforge/sdf/transforms.hpp"

using namespace cadforge::sdf;
using namespace cadforge::math;

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
