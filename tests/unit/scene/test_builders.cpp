// tests/unit/scene/test_builders.cpp
//
// Parametric builder registry (core/scene/builders.hpp) — unit tests.
//
// The registry is the SINGLE SOURCE OF TRUTH for builder tags, parameter
// schemas, defaults and body construction (FASE 2: the duplicate tables
// in io/native.cpp and the desktop UI were removed). These tests pin:
//   * the registry contents (tags, display names, param tables);
//   * make_body() consistency: defaults-from-table == missing-key
//     fallback == fresh primitive built through the plain sdf API;
//   * the ShapeFactory handed out by spec->factory() rebuilds exactly
//     like make_body (this is what the load path re-wires);
//   * unknown tags: find_builder → nullptr (tolerant load path), while
//     the explicit API (default_params / make_body) throws loudly;
//   * end-to-end parametric editing through Scene::set_node_param.
#include <doctest/doctest.h>

#include "CAD_0/scene/builders.hpp"
#include "CAD_0/scene/scene.hpp"
#include "CAD_0/sdf/primitives.hpp"

#include <cmath>
#include <map>
#include <string>
#include <vector>

namespace scene_ns = CAD_0::scene;
using namespace CAD_0;

namespace {

// Deterministic probes used to compare two bodies bit-exactly.
std::vector<float> probe(const sdf::SDFBody& b) {
    std::vector<float> out;
    const math::Vec3f pts[] = {
        {0.0f, 0.0f, 0.0f}, {0.75f, -0.25f, 0.5f}, {-1.2f, 0.9f, -0.3f},
    };
    for (const auto& p : pts) out.push_back(b.value(p));
    return out;
}

const scene_ns::BuilderSpec& spec(const char* tag) {
    const scene_ns::BuilderSpec* s = scene_ns::find_builder(tag);
    REQUIRE(s != nullptr);
    return *s;
}

} // namespace

TEST_SUITE("scene builder registry") {

TEST_CASE("registry: the four UI builders, in Create-menu order") {
    const auto tags = scene_ns::builder_tags();
    REQUIRE(tags.size() == 4);
    CHECK(tags[0] == "sphere");
    CHECK(tags[1] == "box");
    CHECK(tags[2] == "cylinder");
    CHECK(tags[3] == "torus");

    for (const auto& s : scene_ns::builder_registry()) {
        CHECK_FALSE(s.display_name.empty());
        CHECK_FALSE(s.params.empty());
        CHECK(s.build != nullptr);
    }
}

TEST_CASE("registry: parameter schemas and defaults (the pinned table)") {
    const auto& sphere = spec("sphere");
    REQUIRE(sphere.params.size() == 1);
    CHECK(sphere.params[0].name == "radius");
    CHECK(sphere.params[0].default_value == 1.0f);

    const auto& box = spec("box");
    REQUIRE(box.params.size() == 3);
    CHECK(box.params[0].name == "extent_x");
    CHECK(box.params[0].default_value == 0.5f);
    CHECK(box.params[1].name == "extent_y");
    CHECK(box.params[1].default_value == 0.5f);
    CHECK(box.params[2].name == "extent_z");
    CHECK(box.params[2].default_value == 0.5f);

    const auto& cyl = spec("cylinder");
    REQUIRE(cyl.params.size() == 2);
    CHECK(cyl.params[0].name == "radius");
    CHECK(cyl.params[0].default_value == 0.5f);
    CHECK(cyl.params[1].name == "height");
    CHECK(cyl.params[1].default_value == 2.0f);

    const auto& torus = spec("torus");
    REQUIRE(torus.params.size() == 2);
    CHECK(torus.params[0].name == "major_radius");
    CHECK(torus.params[0].default_value == 1.0f);
    CHECK(torus.params[1].name == "minor_radius");
    CHECK(torus.params[1].default_value == 0.3f);

    // default_params(tag) is derived from the same table.
    CHECK(scene_ns::default_params("torus") ==
          std::map<std::string, scene_ns::ParamValue>{
              {"major_radius", 1.0f}, {"minor_radius", 0.3f}});
}

TEST_CASE("registry: make_body equals the plain sdf constructors") {
    // The registry build functions must agree with the public sdf API —
    // this pins table-vs-build-lambda consistency inside every entry.
    CHECK(probe(scene_ns::make_body("sphere", {{"radius", 2.0f}})) ==
          probe(sdf::make_sphere(2.0f)));
    CHECK(probe(scene_ns::make_body("sphere", {})) ==
          probe(sdf::make_sphere(1.0f))); // missing key -> table default

    CHECK(probe(scene_ns::make_body("box", {{"extent_x", 1.25f}})) ==
          probe(sdf::make_box(math::Vec3f{1.25f, 0.5f, 0.5f})));
    CHECK(probe(scene_ns::make_body("box", {})) ==
          probe(sdf::make_box(math::Vec3f{0.5f, 0.5f, 0.5f})));

    CHECK(probe(scene_ns::make_body("cylinder", {{"height", 3.5f}})) ==
          probe(sdf::make_cylinder(0.5f, 3.5f)));

    CHECK(probe(scene_ns::make_body("torus", {{"minor_radius", 0.45f}})) ==
          probe(sdf::make_torus(1.0f, 0.45f)));

    // Explicit params override the defaults; defaults fill the rest.
    CHECK(probe(scene_ns::make_body("torus", {{"major_radius", 2.0f}})) ==
          probe(sdf::make_torus(2.0f, 0.3f)));
}

TEST_CASE("registry: spec->factory() rebuilds exactly like make_body") {
    // This is the ShapeFactory the native loader re-wires on SceneNode —
    // it must behave identically to the explicit make_body() API.
    for (const auto& s : scene_ns::builder_registry()) {
        CAPTURE(s.tag);
        const auto factory = s.factory();
        REQUIRE(factory != nullptr);

        std::map<std::string, scene_ns::ParamValue> edited = s.default_params();
        const std::string key = s.params.front().name;
        edited[key] = scene_ns::ParamValue{2.75f};

        const sdf::SDFBody via_factory = factory(edited);
        const sdf::SDFBody via_make = scene_ns::make_body(s.tag, edited);
        CHECK(probe(via_factory) == probe(via_make));

        // The factory also tolerates missing keys (defaults merge inside
        // make()) — same as the old make_builder_factory behavior.
        std::map<std::string, scene_ns::ParamValue> partial;
        partial[key] = scene_ns::ParamValue{0.75f};
        const sdf::SDFBody partial_factory = factory(partial);
        const sdf::SDFBody partial_make = scene_ns::make_body(s.tag, partial);
        CHECK(probe(partial_factory) == probe(partial_make));
    }
}

TEST_CASE("registry: unknown builder tags") {
    // Tolerant path (used by the native load path): no spec, no factory.
    CHECK(scene_ns::find_builder("metaball") == nullptr);
    CHECK(scene_ns::find_builder("") == nullptr);

    // Explicit API: loud, actionable errors naming the tag.
    CHECK_THROWS_WITH_AS(scene_ns::default_params("metaball"),
        doctest::Contains("unknown builder tag \"metaball\""), std::runtime_error);
    CHECK_THROWS_WITH_AS(scene_ns::make_body("metaball", {}),
        doctest::Contains("unknown builder tag \"metaball\""), std::runtime_error);
    // The message lists the registered tags so the caller can fix it.
    CHECK_THROWS_WITH_AS(scene_ns::default_params("metaball"),
        doctest::Contains("\"sphere\""), std::runtime_error);

    // A spec without a build function fails loudly too (registry hygiene).
    scene_ns::BuilderSpec broken;
    broken.tag = "broken";
    broken.params = {{"x", 1.0f}};
    CHECK_THROWS_WITH_AS(broken.make({}),
        doctest::Contains("no build function"), std::runtime_error);
}

TEST_CASE("registry: end-to-end parametric editing through the kernel") {
    // Exactly the sequence the load path + property panel drive: a node
    // built from the registry, edited via Scene::set_node_param.
    scene_ns::Scene s;
    const auto& sp = spec("sphere");

    scene_ns::SceneNode n;
    n.id = 1;
    n.name = "Sphere";
    n.builder = sp.tag;
    n.params = sp.default_params();
    n.body = scene_ns::make_body(sp.tag, n.params);
    n.factory = sp.factory();
    s.restore_node(std::move(n));
    s.set_next_id(2);

    const float before = s.get_node(1)->body.value(math::Vec3f{0.0f, 0.0f, 0.0f});
    CHECK(before == doctest::Approx(-1.0f)); // sphere(1.0) at the origin

    s.set_node_param(1, "radius", 3.0f);

    const scene_ns::SceneNode* edited = s.get_node(1);
    REQUIRE(edited != nullptr);
    CHECK(edited->params.at("radius") == scene_ns::ParamValue{3.0f});
    CHECK(edited->mesh_dirty == true);
    CHECK(probe(edited->body) == probe(sdf::make_sphere(3.0f)));
}

} // TEST_SUITE
