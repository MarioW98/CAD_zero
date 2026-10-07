// tests/unit/math/test_tolerance.cpp
#include <doctest/doctest.h>

#include "cadforge/math/tolerance.hpp"

using namespace cadforge::math;

TEST_CASE("Tolerance: default linear") {
    auto t = Tolerance::linear();
    CHECK(t.is_linear());
    CHECK(t.value == doctest::Approx(1e-6));
}

TEST_CASE("Tolerance: default angular") {
    auto t = Tolerance::angular();
    CHECK(t.is_angular());
    CHECK(t.value == doctest::Approx(1e-7));
}

TEST_CASE("Tolerance: almost_equal") {
    auto t = Tolerance::linear(0.01);
    CHECK(t.almost_equal(1.0, 1.005));
    CHECK(!t.almost_equal(1.0, 1.05));
}

TEST_CASE("ToleranceAccumulator: add") {
    ToleranceAccumulator a, b;
    a.add_linear(0.001);
    a.add_angular(0.0001);
    b.add_linear(0.002);
    b.add_angular(0.0002);
    a.add(b);
    CHECK(a.linear  == doctest::Approx(0.003));
    CHECK(a.angular == doctest::Approx(0.0003));
}

TEST_CASE("Tolerance: describe string") {
    auto t = Tolerance::linear(1.5);
    CHECK(t.describe().find("linear=") != std::string::npos);
    CHECK(t.describe().find("mm") != std::string::npos);
}
