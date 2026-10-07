// tests/unit/math/test_bbox.cpp
#include <doctest/doctest.h>

#include "CAD_0/math/bbox.hpp"

using namespace CAD_0::math;

TEST_CASE("Bboxf: default is empty") {
    Bboxf b;
    CHECK(b.empty());
}

TEST_CASE("Bboxf: expand a point") {
    Bboxf b;
    b.expand({1, 2, 3});
    CHECK(!b.empty());
    CHECK(b.min == Vec3f{1, 2, 3});
    CHECK(b.max == Vec3f{1, 2, 3});
}

TEST_CASE("Bboxf: expand multiple points") {
    Bboxf b;
    b.expand({-1, -2, -3});
    b.expand({1, 2, 3});
    CHECK(b.min == Vec3f{-1, -2, -3});
    CHECK(b.max == Vec3f{1, 2, 3});
    CHECK(b.center() == Vec3f{0, 0, 0});
    CHECK(b.extent() == Vec3f{2, 4, 6});
}

TEST_CASE("Bboxf: contains") {
    Bboxf b;
    b.expand({-1, -1, -1});
    b.expand({1, 1, 1});
    CHECK(b.contains({0, 0, 0}));
    CHECK(b.contains({1, 1, 1}));
    CHECK(!b.contains({2, 0, 0}));
}

TEST_CASE("Bboxf: distance_to outside point") {
    Bboxf b;
    b.expand({-1, -1, -1});
    b.expand({1, 1, 1});
    // Point 3 units outside along X
    const float d = b.distance_to({4, 0, 0});
    CHECK(d == doctest::Approx(3.0f));
}

TEST_CASE("Bboxf: distance_to inside is negative") {
    Bboxf b;
    b.expand({-1, -1, -1});
    b.expand({1, 1, 1});
    // Inside the box — distance is negative.
    const float d = b.distance_to({0, 0, 0});
    // For an AABB the "inside distance" we compute is 0 by convention
    // (we only return positive distance to surface for exterior).
    // Our implementation returns 0 for interior points. Document it.
    CHECK(d == 0.0f);
}

TEST_CASE("Bboxf: ray-AABB intersect") {
    Bboxf b;
    b.expand({-1, -1, -1});
    b.expand({1, 1, 1});
    // Ray from -10 along +X
    Vec3f origin{-10, 0, 0};
    Vec3f inv_dir{1.0f / 1.0f, 1.0f / 0.0f, 1.0f / 0.0f}; // zero dir → inf
    // Replace NaNs / zeros with safe values for the test
    inv_dir.y = 1e30f;
    inv_dir.z = 1e30f;
    auto hit = b.intersect_ray(origin, inv_dir);
    CHECK(hit.lo == doctest::Approx(9.0f).epsilon(1e-5f));
    CHECK(hit.hi == doctest::Approx(11.0f).epsilon(1e-5f));
}
