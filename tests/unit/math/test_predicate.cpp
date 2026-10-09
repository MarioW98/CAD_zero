// tests/unit/math/test_predicate.cpp
//
// NOTE: doctest's macro expansion sometimes clashes with variable names
// that look like macros (e.g. `out`, `in`, `above`, `outside` on
// Windows). We use array-based variable names like `p4` here to be safe.
//
#include <doctest/doctest.h>

#include "CAD_0/math/predicate.hpp"

using namespace CAD_0::math;
using namespace CAD_0::math::predicate;

TEST_CASE("predicate: orient2d counter-clockwise triangle") {
    initialize();
    const Vec2d p1{0, 0};
    const Vec2d p2{1, 0};
    const Vec2d p3{0, 1};
    CHECK(orient2d(p1, p2, p3) > 0.0);
}

TEST_CASE("predicate: orient2d clockwise triangle") {
    const Vec2d p1{0, 0};
    const Vec2d p2{0, 1};
    const Vec2d p3{1, 0};
    CHECK(orient2d(p1, p2, p3) < 0.0);
}

TEST_CASE("predicate: orient2d collinear") {
    const Vec2d p1{0, 0};
    const Vec2d p2{1, 1};
    const Vec2d p3{2, 2};
    CHECK(orient2d(p1, p2, p3) == 0.0);
}

TEST_CASE("predicate: orient3d non-coplanar") {
    const Vec3d p1{0, 0, 0};
    const Vec3d p2{1, 0, 0};
    const Vec3d p3{0, 1, 0};
    const Vec3d p4{0, 0, 1};
    CHECK(orient3d(p1, p2, p3, p4) > 0.0);
}

TEST_CASE("predicate: orient3d coplanar") {
    const Vec3d p1{0, 0, 0};
    const Vec3d p2{1, 0, 0};
    const Vec3d p3{0, 1, 0};
    const Vec3d p4{1, 1, 0};
    CHECK(orient3d(p1, p2, p3, p4) == 0.0);
}

TEST_CASE("predicate: incircle inside") {
    // Three points on the unit circle at 0, 120, 240 degrees
    const Vec2d p1{1, 0};
    const Vec2d p2{-0.5, 0.8660254037844386};
    const Vec2d p3{-0.5, -0.8660254037844386};
    const Vec2d p4{0, 0}; // origin is inside the circle
    CHECK(incircle2d(p1, p2, p3, p4) > 0.0);
}

TEST_CASE("predicate: incircle outside") {
    const Vec2d p1{1, 0};
    const Vec2d p2{-0.5, 0.8660254037844386};
    const Vec2d p3{-0.5, -0.8660254037844386};
    const Vec2d p4{2, 2}; // outside the circle
    CHECK(incircle2d(p1, p2, p3, p4) < 0.0);
}
