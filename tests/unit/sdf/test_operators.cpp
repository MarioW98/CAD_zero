// tests/unit/sdf/test_operators.cpp
#include <doctest/doctest.h>

#include "cadforge/sdf/primitives.hpp"
#include "cadforge/sdf/operators.hpp"
#include "cadforge/sdf/transforms.hpp"

using namespace cadforge::sdf;

TEST_CASE("Union: two spheres — value at midpoint") {
    auto a = make_sphere(1.0f);
    auto b = make_sphere(1.0f);
    auto u = sdf_union(std::move(a), std::move(b));
    // The union of two unit spheres centered at origin is just a unit sphere.
    CHECK(u.value({0, 0, 0}) == doctest::Approx(-1.0f));
}

TEST_CASE("Union: shifted spheres") {
    auto a = make_sphere(1.0f);
    auto b = sdf_translate(make_sphere(1.0f), {3, 0, 0});
    auto u = sdf_union(std::move(a), std::move(b));
    // Midpoint between the two spheres
    const float v = u.value({1.5f, 0, 0});
    // At x=1.5, distance to sphere A (center 0, r 1) is 0.5
    // Distance to sphere B (center 3, r 1) is 0.5
    // Union (min) = 0.5
    CHECK(v == doctest::Approx(0.5f).epsilon(1e-5f));
}

TEST_CASE("Subtract: cube minus sphere") {
    auto box = make_box({1, 1, 1});
    auto sph = make_sphere(0.5f);
    auto cut = sdf_subtract(std::move(box), std::move(sph));
    // Center of the cube — sphere center is also at origin, so center is "outside" (in the carved region)
    const float v = cut.value({0, 0, 0});
    // Inside the cube (negative) intersected with "not in sphere" (= positive, since sphere is inside cube)
    // Result: max(box_value, -sphere_value) = max(-1, 0.5) = 0.5
    // Wait: subtraction is intersect(a, complement(b)) = max(a_value, -b_value)
    // a (box) at center = -1 (inside)
    // b (sphere r=0.5) at center = -0.5 (inside)
    // -b = 0.5
    // max(-1, 0.5) = 0.5  → outside the result
    CHECK(v == doctest::Approx(0.5f).epsilon(1e-5f));
}

TEST_CASE("Intersect: two unit spheres overlap fully") {
    auto a = make_sphere(1.0f);
    auto b = make_sphere(1.0f);
    auto i = sdf_intersect(std::move(a), std::move(b));
    // Intersection of identical spheres = the sphere itself
    CHECK(i.value({0, 0, 0}) == doctest::Approx(-1.0f));
}

TEST_CASE("SmoothMin: k=0 equals hard union") {
    auto a = make_sphere(1.0f);
    auto b = sdf_translate(make_sphere(1.0f), {3, 0, 0});
    auto smin = sdf_smooth_union(std::move(a), std::move(b), 0.0f);
    auto hard = sdf_union(make_sphere(1.0f),
                         sdf_translate(make_sphere(1.0f), {3, 0, 0}));
    CHECK(smin.value({1.5f, 0, 0}) == doctest::Approx(hard.value({1.5f, 0, 0})).epsilon(1e-4f));
}

TEST_CASE("SmoothMin: k>0 produces smoother blend") {
    auto a = make_sphere(1.0f);
    auto b = sdf_translate(make_sphere(1.0f), {3, 0, 0});
    auto smin = sdf_smooth_union(std::move(a), std::move(b), 0.5f);
    // At the midpoint, the smooth union should produce a value slightly
    // LESS than the hard union (i.e. a wider "inside" region).
    const float v_smooth = smin.value({1.5f, 0, 0});
    auto ha = make_sphere(1.0f);
    auto hb = sdf_translate(make_sphere(1.0f), {3, 0, 0});
    auto hard = sdf_union(std::move(ha), std::move(hb));
    const float v_hard = hard.value({1.5f, 0, 0});
    CHECK(v_smooth <= v_hard + 1e-5f);
}

TEST_CASE("Operators: lipschitz is composed correctly") {
    auto a = make_sphere(1.0f);
    auto b = make_sphere(1.0f);
    auto u = sdf_union(std::move(a), std::move(b));
    CHECK(u.lipschitz() == 1.0f);
}
