// tests/unit/math/test_vec.cpp
#include <doctest/doctest.h>

#include "CAD_0/math/vec.hpp"

using namespace CAD_0::math;

TEST_CASE("Vec3f: basic construction and access") {
    Vec3f v{1.0f, 2.0f, 3.0f};
    CHECK(v.x == 1.0f);
    CHECK(v.y == 2.0f);
    CHECK(v.z == 3.0f);
    CHECK(v[0] == 1.0f);
    CHECK(v[2] == 3.0f);
}

TEST_CASE("Vec3f: arithmetic") {
    Vec3f a{1, 2, 3};
    Vec3f b{4, 5, 6};
    CHECK(a + b == Vec3f{5, 7, 9});
    CHECK(b - a == Vec3f{3, 3, 3});
    CHECK(a * 2.0f == Vec3f{2, 4, 6});
    CHECK(-a == Vec3f{-1, -2, -3});
}

TEST_CASE("Vec3f: dot and cross") {
    Vec3f x{1, 0, 0}, y{0, 1, 0}, z{0, 0, 1};
    CHECK(dot(x, y) == 0.0f);
    CHECK(dot(x, x) == 1.0f);
    CHECK(cross(x, y) == z);
    CHECK(cross(y, z) == x);
    CHECK(cross(z, x) == y);
}

TEST_CASE("Vec3f: length and normalize") {
    Vec3f v{3, 4, 0};
    CHECK(v.length() == doctest::Approx(5.0f));
    CHECK(v.length_sq() == 25.0f);
    Vec3f n = v.normalized();
    CHECK(n.length() == doctest::Approx(1.0f).epsilon(1e-5f));
}

TEST_CASE("Vec3f: zero vector normalizes to zero") {
    Vec3f z{0, 0, 0};
    Vec3f n = z.normalized();
    CHECK(n.x == 0.0f);
    CHECK(n.y == 0.0f);
    CHECK(n.z == 0.0f);
}

TEST_CASE("Vec2f: arithmetic") {
    Vec2f a{1, 2};
    Vec2f b{3, 4};
    CHECK(a + b == Vec2f{4, 6});
    CHECK(dot(a, b) == 11.0f);
}
