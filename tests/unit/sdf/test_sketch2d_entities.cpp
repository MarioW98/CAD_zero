// tests/unit/sdf/test_sketch2d_entities.cpp
//
// ADR-0030 D3, MVP unit 1 — the 2D entity fields, unit by unit:
// exact fields, constant/radial gradients, the half-box and disc
// bounds (pinned), and the ADR-0029 D5 construction validation.
#include <doctest/doctest.h>

#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>

#include "CAD_0/sdf/sketch_entities.hpp"

using namespace CAD_0::sdf;
using namespace CAD_0::math;

namespace {

constexpr float kInf = std::numeric_limits<float>::infinity();

bool is_universe(const Bbox2f& b) {
    return b.min.x == -kInf && b.min.y == -kInf &&
           b.max.x == kInf && b.max.y == kInf;
}

} // namespace

// --- Halfplane2D: the exact field ---------------------------------------

TEST_CASE("Halfplane2D: axis-aligned field, interior on the left") {
    // a->b along +x: walking east, the left hand points north (+y).
    const Halfplane2D h({0.0f, 0.0f}, {2.0f, 0.0f}, /*interior_left=*/true);
    CHECK(h.sample({0.0f, 1.0f}).value == doctest::Approx(-1.0f));
    CHECK(h.sample({0.0f, -3.0f}).value == doctest::Approx(3.0f));
    CHECK(h.sample({5.0f, 0.0f}).value == doctest::Approx(0.0f).epsilon(1e-6f));
    CHECK(h.sample({-5.0f, 0.25f}).value == doctest::Approx(-0.25f));
    // Constant gradient = the outward normal (0, -1): interior above.
    const auto g = h.sample({7.0f, 2.0f}).gradient;
    CHECK(g.x == doctest::Approx(0.0f));
    CHECK(g.y == doctest::Approx(-1.0f));
    CHECK(h.lipschitz() == 1.0f);
}

TEST_CASE("Halfplane2D: interior on the right flips the sign") {
    const Halfplane2D h({0.0f, 0.0f}, {2.0f, 0.0f}, /*interior_left=*/false);
    CHECK(h.sample({0.0f, 1.0f}).value == doctest::Approx(1.0f));
    CHECK(h.sample({0.0f, -3.0f}).value == doctest::Approx(-3.0f));
    CHECK(h.sample({7.0f, 2.0f}).gradient.y == doctest::Approx(1.0f));
}

TEST_CASE("Halfplane2D: diagonal segment, exact half-plane field") {
    // a->b along (1,1)/√2: left of the traversal is the north-west side.
    // Outward normal n = (1,-1)/√2; f(P) = (P-a)·n.
    const Halfplane2D h({0.0f, 0.0f}, {1.0f, 1.0f}, /*interior_left=*/true);
    const float k = 1.0f / std::sqrt(2.0f);
    CHECK(h.sample({1.0f, 0.0f}).value == doctest::Approx(k).epsilon(1e-6f));
    CHECK(h.sample({0.0f, 1.0f}).value == doctest::Approx(-k).epsilon(1e-6f));
    CHECK(h.sample({2.0f, -2.0f}).value == doctest::Approx(4.0f * k).epsilon(1e-6f));
    CHECK(h.sample({-1.0f, 3.0f}).value == doctest::Approx(-4.0f * k).epsilon(1e-6f));
    const auto g = h.sample({9.0f, 9.0f}).gradient;
    CHECK(g.x == doctest::Approx(k).epsilon(1e-6f));
    CHECK(g.y == doctest::Approx(-k).epsilon(1e-6f));
}

TEST_CASE("Halfplane2D: bounds — the axis-aligned half-box, pinned") {
    // Interior above y = 3 (a->b along +x, left = north): the region
    // {f < 0} is {y > 3}. x free, y clipped at the boundary.
    const Halfplane2D up({-1.0f, 3.0f}, {4.0f, 3.0f}, /*interior_left=*/true);
    const auto b_up = up.bounds();
    CHECK(b_up.min.x == -kInf);
    CHECK(b_up.max.x == kInf);
    CHECK(b_up.min.y == doctest::Approx(3.0f));
    CHECK(b_up.max.y == kInf);

    // Interior below (flip the side bit): region {y < 3}.
    const Halfplane2D down({-1.0f, 3.0f}, {4.0f, 3.0f}, /*interior_left=*/false);
    const auto b_dn = down.bounds();
    CHECK(b_dn.min.y == -kInf);
    CHECK(b_dn.max.y == doctest::Approx(3.0f));

    // Interior left of x = -2 (a->b along +y, left is -x).
    const Halfplane2D left({-2.0f, -1.0f}, {-2.0f, 5.0f}, /*interior_left=*/true);
    const auto b_lf = left.bounds();
    CHECK(b_lf.min.x == -kInf);
    CHECK(b_lf.max.x == doctest::Approx(-2.0f));
    CHECK(b_lf.min.y == -kInf);
    CHECK(b_lf.max.y == kInf);
}

TEST_CASE("Halfplane2D: bounds — the diagonal half-plane degenerates to "
           "the universe (declared over-estimate)") {
    // A diagonal region reaches every axis extreme, so no axis-aligned
    // box tighter than the universe covers it. Contract-true; the
    // profile-level contour hull closes the gap (test_sketch2d_profile).
    const Halfplane2D h({0.0f, 0.0f}, {1.0f, 1.0f}, true);
    CHECK(is_universe(h.bounds()));
}

TEST_CASE("Halfplane2D: construction validation names the parameter") {
    const float nan = std::numeric_limits<float>::quiet_NaN();
    REQUIRE_THROWS_AS(Halfplane2D({nan, 0.0f}, {1.0f, 0.0f}, true),
                      std::invalid_argument);
    REQUIRE_THROWS_AS(Halfplane2D({0.0f, 0.0f}, {kInf, 0.0f}, true),
                      std::invalid_argument);
    // p0 == p1: no normal to orient.
    REQUIRE_THROWS_AS(Halfplane2D({1.0f, 2.0f}, {1.0f, 2.0f}, true),
                      std::invalid_argument);
    try {
        Halfplane2D h({1.0f, 2.0f}, {1.0f, 2.0f}, true);
        (void)h;
    } catch (const std::invalid_argument& e) {
        CHECK(std::string(e.what()).find("Halfplane2D") != std::string::npos);
        CHECK(std::string(e.what()).find("p0 != p1") != std::string::npos);
    }
}

// --- Disc2D: the exact disc / complement field ---------------------------

TEST_CASE("Disc2D: outer orientation is the disc, exact everywhere") {
    const Disc2D d({2.0f, -1.0f}, 0.5f, /*hole=*/false);
    CHECK(d.sample({2.0f, -1.0f}).value == doctest::Approx(-0.5f)); // center
    CHECK(d.sample({2.5f, -1.0f}).value == doctest::Approx(0.0f).epsilon(1e-6f));
    CHECK(d.sample({2.0f, -0.2f}).value == doctest::Approx(0.3f));
    CHECK(d.sample({-10.0f, -1.0f}).value == doctest::Approx(11.5f));
    // Radial gradient, outward.
    auto g = d.sample({4.0f, -1.0f}).gradient;
    CHECK(g.x == doctest::Approx(1.0f));
    CHECK(g.y == doctest::Approx(0.0f));
    g = d.sample({2.0f, 1.0f}).gradient;
    CHECK(g.x == doctest::Approx(0.0f));
    CHECK(g.y == doctest::Approx(1.0f));
    CHECK(d.lipschitz() == 1.0f);
}

TEST_CASE("Disc2D: hole orientation is the complement (the ring half)") {
    const Disc2D d({0.0f, 0.0f}, 0.4f, /*hole=*/true);
    CHECK(d.sample({0.0f, 0.0f}).value == doctest::Approx(0.4f));  // center outside
    CHECK(d.sample({0.4f, 0.0f}).value == doctest::Approx(0.0f).epsilon(1e-6f));
    CHECK(d.sample({1.3f, 0.0f}).value == doctest::Approx(-0.9f)); // far outside: inside profile
    // Gradient points the other way (into the hole = outward of the region).
    CHECK(d.sample({2.0f, 0.0f}).gradient.x == doctest::Approx(-1.0f));
    CHECK(d.sample({0.0f, -3.0f}).gradient.y == doctest::Approx(1.0f));
}

TEST_CASE("Disc2D: gradient at the center is the zero vector (SphereSDF "
           "precedent, documented)") {
    const Disc2D d({1.0f, 1.0f}, 2.0f, false);
    const auto g = d.sample({1.0f, 1.0f}).gradient;
    CHECK(g.x == 0.0f);
    CHECK(g.y == 0.0f);
}

TEST_CASE("Disc2D: bounds — [C±r]² for the disc, universe for the hole") {
    const Disc2D outer({2.0f, -1.0f}, 0.5f, false);
    const auto b = outer.bounds();
    CHECK(b.min.x == doctest::Approx(1.5f));
    CHECK(b.min.y == doctest::Approx(-1.5f));
    CHECK(b.max.x == doctest::Approx(2.5f));
    CHECK(b.max.y == doctest::Approx(-0.5f));
    CHECK(b.bounded());

    // The hole's region is the unbounded complement: the universe box
    // is the contract-true answer ([C±r]² would clip ring profiles).
    const Disc2D hole({2.0f, -1.0f}, 0.5f, true);
    CHECK(is_universe(hole.bounds()));
}

TEST_CASE("Disc2D: construction validation names the parameter") {
    const float nan = std::numeric_limits<float>::quiet_NaN();
    REQUIRE_THROWS_AS(Disc2D({0.0f, 0.0f}, 0.0f, false), std::invalid_argument);
    REQUIRE_THROWS_AS(Disc2D({0.0f, 0.0f}, -0.5f, false), std::invalid_argument);
    REQUIRE_THROWS_AS(Disc2D({0.0f, 0.0f}, nan, false), std::invalid_argument);
    REQUIRE_THROWS_AS(Disc2D({nan, 0.0f}, 1.0f, true), std::invalid_argument);
    REQUIRE_THROWS_AS(Disc2D({0.0f, kInf}, 1.0f, true), std::invalid_argument);
    try {
        Disc2D d({0.0f, 0.0f}, -1.0f, false);
        (void)d;
    } catch (const std::invalid_argument& e) {
        CHECK(std::string(e.what()).find("Disc2D") != std::string::npos);
        CHECK(std::string(e.what()).find("radius") != std::string::npos);
    }
}

// --- Bbox2f: the 2D box companions ---------------------------------------

TEST_CASE("Bbox2f: empty/universe/intersect/expand semantics") {
    CHECK(Bbox2f{}.empty());
    CHECK(!Bbox2f{}.bounded());
    CHECK(is_universe(Bbox2f::universe()));
    CHECK(!Bbox2f::universe().bounded());

    Bbox2f a;
    a.expand({0.0f, 0.0f});
    a.expand({1.0f, 2.0f});
    Bbox2f b;
    b.expand({0.5f, -1.0f});
    b.expand({2.0f, 0.5f});
    const auto i = a.intersect(b);
    CHECK(i.min.x == doctest::Approx(0.5f));
    CHECK(i.min.y == doctest::Approx(0.0f));
    CHECK(i.max.x == doctest::Approx(1.0f));
    CHECK(i.max.y == doctest::Approx(0.5f));

    // Disjoint intersect -> empty; universe intersect box -> box.
    Bbox2f c;
    c.expand({10.0f, 10.0f});
    c.expand({11.0f, 11.0f});
    CHECK(a.intersect(c).empty());
    const auto u = Bbox2f::universe().intersect(a);
    CHECK(u.min.x == doctest::Approx(0.0f));
    CHECK(u.max.y == doctest::Approx(2.0f));

    // A half-box intersect a half-box: the axis-aligned profile rule.
    // (Build them from the universe, not from the default empty box.)
    Bbox2f top_free = Bbox2f::universe();
    top_free.max.y = 1.0f; // {y <= 1}, x free
    Bbox2f right_free = Bbox2f::universe();
    right_free.min.x = 0.0f; // {x >= 0}, y free
    const auto quadrant = top_free.intersect(right_free);
    CHECK(quadrant.min.x == doctest::Approx(0.0f));
    CHECK(quadrant.max.y == doctest::Approx(1.0f));
    CHECK(quadrant.min.y == -kInf);
    CHECK(quadrant.max.x == kInf);
}
