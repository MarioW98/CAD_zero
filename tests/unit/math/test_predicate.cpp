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
    // CORRECTED with the H1 vendoring (ADR-0034): the historical stub's
    // orient3d carried a sign error in its middle term — it computed
    // bdx*(ady*cdz - adz*cdy) where Shewchuk's determinant reads
    // bdx*(cdy*adz - cdz*ady) — and this test pinned the stub's output
    // (> 0), contradicting predicate.hpp's own documented convention
    // ("> 0 if d is BELOW plane a,b,c"): pd = (0,0,1) is ABOVE the
    // plane z=0 through pa,pb,pc, so the true orient3d is NEGATIVE.
    // The authentic adaptive predicate returns -1.0 here (verified
    // against exact rational arithmetic); the pin now says the truth.
    const Vec3d p1{0, 0, 0};
    const Vec3d p2{1, 0, 0};
    const Vec3d p3{0, 1, 0};
    const Vec3d p4{0, 0, 1};
    CHECK(orient3d(p1, p2, p3, p4) < 0.0);
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
