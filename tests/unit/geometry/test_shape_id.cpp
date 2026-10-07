// tests/unit/geometry/test_shape_id.cpp
#include <doctest/doctest.h>

#include "cadforge/geometry/shape_id.hpp"

using namespace cadforge::geometry;

TEST_CASE("ShapeId: default is zero") {
    ShapeId id;
    CHECK(id.feature.value == 0);
    CHECK(id.subshape.depth == 0);
    CHECK(id.generation.value == 0);
}

TEST_CASE("ShapeId: equality") {
    ShapeId a, b;
    a.feature = FeatureId{42};
    a.subshape = SubshapeRef{}.push(1).push(2);
    b.feature = FeatureId{42};
    b.subshape = SubshapeRef{}.push(1).push(2);
    CHECK(a == b);
}

TEST_CASE("ShapeId: subshape path") {
    SubshapeRef r;
    r = r.push(3);
    r = r.push(7);
    CHECK(r.depth == 2);
    CHECK(r.path[0] == 3);
    CHECK(r.path[1] == 7);
}

TEST_CASE("ShapeId: to_string format") {
    ShapeId id;
    id.feature = FeatureId{5};
    id.subshape = SubshapeRef{}.push(1).push(2).push(3);
    id.generation = Generation{7};
    std::string s = id.to_string();
    CHECK(s.find("F5") != std::string::npos);
    CHECK(s.find("S1.2.3") != std::string::npos);
    CHECK(s.find("G7") != std::string::npos);
}

TEST_CASE("ShapeId: stale check") {
    ShapeId id;
    id.generation = Generation{3};
    CHECK(id.is_stale_relative_to(Generation{5}));
    CHECK(!id.is_stale_relative_to(Generation{3}));
    CHECK(!id.is_stale_relative_to(Generation{2}));
}
