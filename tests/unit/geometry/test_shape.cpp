// tests/unit/geometry/test_shape.cpp
#include <doctest/doctest.h>

#include "cadforge/geometry/shape.hpp"
#include "cadforge/sdf/primitives.hpp"

using namespace cadforge::geometry;
using namespace cadforge::sdf;

TEST_CASE("Shape: default is empty (no body)") {
    Shape s;
    // An empty variant should not match any index — representation() returns SDF by default.
    // (See shape.hpp for the fallback path.)
    CHECK(s.representation() == Representation::SDF);
    CHECK(s.as_sdf() == nullptr);
    CHECK(s.as_brep() == nullptr);
    CHECK(s.as_hybrid() == nullptr);
}

TEST_CASE("Shape: construct from SDFBody") {
    auto body = make_sphere(1.0f);
    Shape s(BodyVariant(std::make_unique<SDFBody>(std::move(body))));
    CHECK(s.representation() == Representation::SDF);
    REQUIRE(s.as_sdf() != nullptr);
    CHECK(s.as_sdf()->value({0, 0, 0}) == doctest::Approx(-1.0f));
    CHECK(s.as_sdf()->value({2, 0, 0}) == doctest::Approx(1.0f));
}

TEST_CASE("Shape: transform does not affect payload directly") {
    auto body = make_box({1, 1, 1});
    Shape s(BodyVariant(std::make_unique<SDFBody>(std::move(body))));
    Transform t;
    t.translation = {10, 0, 0};
    s.set_transform(t);
    CHECK(s.transform().translation.x == 10.0f);
}

TEST_CASE("Shape: move semantics") {
    auto body = make_sphere(2.0f);
    Shape a(BodyVariant(std::make_unique<SDFBody>(std::move(body))));
    Shape b = std::move(a);
    // a is in a moved-from state; b owns the body
    REQUIRE(b.as_sdf() != nullptr);
    CHECK(b.as_sdf()->value({0, 0, 0}) == doctest::Approx(-2.0f));
}
