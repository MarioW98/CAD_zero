// tests/unit/bridge/test_bridge.cpp
//
// Tests for the SDF ↔ B-Rep bridge.
//
#include <doctest/doctest.h>

#include "CAD_0/bridge/sdf_to_brep.hpp"
#include "CAD_0/bridge/brep_to_sdf.hpp"
#include "CAD_0/sdf/primitives.hpp"
#include "CAD_0/sdf/operators.hpp"
#include "CAD_0/sdf/transforms.hpp"
#include "CAD_0/brep/factory.hpp"

using namespace CAD_0::bridge;
using CAD_0::math::Vec3f;

// Use explicit namespace to disambiguate make_sphere/make_box/make_cylinder
// (they exist in both CAD_0::sdf and CAD_0::brep)
namespace s = CAD_0::sdf;
namespace b = CAD_0::brep;

TEST_CASE("Bridge: SDF sphere → B-Rep sphere (primitive fit)") {
    auto sph = s::make_sphere(2.0f);
    SdfToBrepOptions opts;
    opts.fit_primitives = true;
    auto body = sdf_to_brep(sph, opts);

    REQUIRE(body != nullptr);
    CHECK(body->face_count() == 1);
    CHECK(std::string(body->solid.shell().faces()[0].surface()->type_name()) == "sphere");
}

TEST_CASE("Bridge: SDF box → B-Rep box (primitive fit)") {
    auto box = s::make_box(Vec3f{1, 1, 1});
    SdfToBrepOptions opts;
    opts.fit_primitives = true;
    auto body = sdf_to_brep(box, opts);

    REQUIRE(body != nullptr);
    CHECK(body->vertex_count() == 8);
    CHECK(body->edge_count() == 12);
    CHECK(body->face_count() == 6);
}

TEST_CASE("Bridge: SDF cylinder → B-Rep cylinder (primitive fit)") {
    auto cyl = s::make_cylinder(1.0f, 2.0f);
    SdfToBrepOptions opts;
    opts.fit_primitives = true;
    auto body = sdf_to_brep(cyl, opts);

    REQUIRE(body != nullptr);
    CHECK(body->face_count() == 3);
    CHECK(std::string(body->solid.shell().faces()[0].surface()->type_name()) == "cylinder");
}

TEST_CASE("Bridge: SDF sphere → B-Rep faceted (no primitive fit)") {
    auto sph = s::make_sphere(1.0f);
    SdfToBrepOptions opts;
    opts.fit_primitives = false;
    opts.march_resolution = 8;
    auto body = sdf_to_brep(sph, opts);

    REQUIRE(body != nullptr);
    CHECK(body->face_count() > 0);
    CHECK(body->vertex_count() > 0);
    for (const auto& face : body->solid.shell().faces()) {
        CHECK(std::string(face.surface()->type_name()) == "plane");
    }
}

TEST_CASE("Bridge: SDF smooth_union → B-Rep faceted") {
    auto a = s::make_sphere(1.0f);
    auto sdf_b = s::sdf_translate(s::make_sphere(1.0f), {2, 0, 0});
    auto u = s::sdf_smooth_union(std::move(a), std::move(sdf_b), 0.5f);

    SdfToBrepOptions opts;
    opts.fit_primitives = true;
    opts.march_resolution = 8;
    auto body = sdf_to_brep(u, opts);

    REQUIRE(body != nullptr);
    CHECK(body->face_count() > 0);
    for (const auto& face : body->solid.shell().faces()) {
        CHECK(std::string(face.surface()->type_name()) == "plane");
    }
}

TEST_CASE("Bridge: B-Rep sphere → SDF sphere") {
    auto body = b::make_sphere(2.0f);
    auto sdf_body = brep_to_sdf(*body);

    REQUIRE(sdf_body != nullptr);
    CHECK(sdf_body->value(Vec3f{0, 0, 0}) == doctest::Approx(-2.0f).epsilon(0.01f));
    CHECK(sdf_body->value(Vec3f{3, 0, 0}) == doctest::Approx(1.0f).epsilon(0.01f));
}

TEST_CASE("Bridge: B-Rep box → SDF box") {
    auto body = b::make_box(Vec3f{1, 1, 1});
    auto sdf_body = brep_to_sdf(*body);

    REQUIRE(sdf_body != nullptr);
    CHECK(sdf_body->value(Vec3f{0, 0, 0}) < 0.0f);
    CHECK(sdf_body->value(Vec3f{5, 0, 0}) > 0.0f);
}

TEST_CASE("Bridge: B-Rep cylinder → SDF cylinder") {
    auto body = b::make_cylinder(1.0f, 4.0f);
    auto sdf_body = brep_to_sdf(*body);

    REQUIRE(sdf_body != nullptr);
    CHECK(sdf_body->value(Vec3f{0, 0, 0}) < 0.0f);
    CHECK(sdf_body->value(Vec3f{5, 0, 0}) > 0.0f);
}

TEST_CASE("Bridge: round-trip sphere SDF → B-Rep → SDF") {
    auto original = s::make_sphere(2.0f);
    auto body = sdf_to_brep(original);
    auto roundtrip = brep_to_sdf(*body);

    REQUIRE(roundtrip != nullptr);
    CHECK(roundtrip->value(Vec3f{0, 0, 0}) == doctest::Approx(-2.0f).epsilon(0.01f));
    CHECK(roundtrip->value(Vec3f{3, 0, 0}) == doctest::Approx(1.0f).epsilon(0.01f));
}

TEST_CASE("Bridge: round-trip box SDF → B-Rep → SDF") {
    auto original = s::make_box(Vec3f{1, 1, 1});
    auto body = sdf_to_brep(original);
    auto roundtrip = brep_to_sdf(*body);

    REQUIRE(roundtrip != nullptr);
    CHECK(roundtrip->value(Vec3f{0, 0, 0}) < 0.0f);
    CHECK(roundtrip->value(Vec3f{5, 0, 0}) > 0.0f);
}

TEST_CASE("Bridge: faceted B-Rep → SDF mesh") {
    auto a = s::make_sphere(1.0f);
    auto sdf_b = s::sdf_translate(s::make_sphere(1.0f), {2, 0, 0});
    auto u = s::sdf_smooth_union(std::move(a), std::move(sdf_b), 0.5f);

    SdfToBrepOptions opts;
    opts.fit_primitives = true;
    opts.march_resolution = 8;
    auto body = sdf_to_brep(u, opts);

    REQUIRE(body != nullptr);
    CHECK(body->face_count() > 0);

    auto sdf_body = brep_to_sdf(*body);
    REQUIRE(sdf_body != nullptr);

    auto bnds = sdf_body->bounds();
    CHECK(!bnds.empty());
}
